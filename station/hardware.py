"""The one owner of the arms and the cameras.

A COM port and a USB camera can each be held by exactly one process at a time.
That single fact decides the whole shape of this file: there is one worker, it is
in one mode at a time, and every mode change goes through one lock. The failure
this avoids is the one that bit the SO-101 side repeatedly -- a preview window
left open makes the next thing that touches the camera die on connect, and the
error it dies with says nothing about a preview window.

Modes:

    IDLE     cameras open for the live picture, arms closed and free to move
    TELEOP   cameras open, plus a leader->follower loop running in this process
    RECORD   this process holds nothing; lerobot-record owns both, and frames are
             copied out of the robot's own observations so the picture keeps going

RECORD reuses lerobot's ``record()`` rather than reimplementing it, because the
dataset layout is not something worth having a second, subtly different copy of.
Two small patches make that possible: ``init_keyboard_listener`` is replaced so
the browser's buttons become the Enter / redo / stop keys, and the follower's
``get_observation`` is wrapped so frames can be teed to the browser.

Which arm it is comes from robots.py. A NexArm runs exactly the path it always
did; any other arm goes through LeRobot's own classes, and the first time one is
used on a computer LeRobot's calibration runs with its terminal prompts turned
into a Next button on the page.
"""

from __future__ import annotations

import builtins
import contextlib
import csv
import json
import math
import threading
import time
import traceback
from collections import deque
from pathlib import Path

from . import detect, robots

ROOT = Path(__file__).resolve().parent.parent
DATASETS_DIR = ROOT / "datasets"

# --- Speed -------------------------------------------------------------------
#
# The arm runs at the firmware's normal speed. It is NOT throttled: a throttled
# follower lags the leader, and a demonstration where the arm is visibly behind
# the hand is a worse demonstration, not a safer one.
#
# acc is 0-254, where 0 means no ramp at all and higher means a softer ramp.
# These are lerobot's own defaults for this arm.
MOTION_SPEED = 2000
MOTION_ACC = 100

# Ceiling on how far a joint may be asked to travel in one control tick. This is
# a jump guard, not a speed limit, and the number is chosen so that it never
# touches real movement: at 30 Hz this allows 7500 counts/second, nearly twice
# the arm's entire range every second, which no hand produces. What it does catch
# is a target that teleports -- the 2000-count leap that drove the arm into its
# end stop during development came from a single corrupt reading, and a corrupt
# reading is always a teleport.
MAX_STEP_PER_TICK = 250

# The same ceiling during the opening sync, when the follower is crossing from
# wherever it was parked to wherever the leader is being held. Nobody is driving
# it yet, the gap can be most of the range, and there is no reason for that first
# move to be quick. 12 counts/tick is about 360 counts/second.
SYNC_STEP_PER_TICK = 12

# How long the opening sync is allowed to take, in seconds, and how much gap it
# covers per second. A short gap should not take ten seconds just because a
# constant said so, and a long one should not be rushed.
SYNC_COUNTS_PER_SECOND = 320
SYNC_MIN_SECONDS = 0.3      # was 2.0: arms that nearly match waited 2 s for nothing
SYNC_MAX_SECONDS = 12.0

# The gripper guard. A gripper stopped by its own end stop, or by the block it is
# holding, still has its target beyond it, and a servo held short of its target
# keeps pulling until it overheats or its overload protection switches it off --
# the gripper then "stops working". The NexArm's open end is the everyday case:
# its trigger at rest asks for 2833 and the jaw stops at about 2753, so it pushed
# into its own stop whenever nobody was squeezing (88% of a recorded session).
# Same scale and thresholds as the SO-101's guard, but quicker: there, waiting
# 0.3 s was too slow and the servo tripped first. In raw servo counts:
GRIP_STALL_ERROR = 40     # target this far past the jaw ...
GRIP_STILL = 6            # ... and the jaw moving less than this per tick (a straining
                          # servo jitters about 5; a moving jaw goes tens) ...
GRIP_STALL_TICKS = 8      # ... for this many ticks while pushing (0.27 s at 30 Hz) =
                          # blocked. 3 caught a jaw still speeding up from rest.
GRIP_SQUEEZE = 15         # then ask for only this far past where it stopped

# A camera whose picture has not changed for this long has frozen -- it stops
# sending frames and DirectShow keeps handing back the last one -- so it is
# reopened. Real frames always differ a little: sensor noise.
CAMERA_FROZEN_SECONDS = 3.0

# Labels typed on the page, written per try to labels.csv in the job's folder.
LABEL_FIELDS = ("driver", "recorder", "labeler", "position", "wrong", "type", "notes")
LABEL_COLUMNS = ("try", "episode", "status", "job", "driver", "recorder", "labeler", "position",
                 "something_wrong", "type", "notes", "saved_at")
SWAP_EVERY = 10           # the group swaps jobs every this many tries

# A recording runs until the Done button, not until a timer. lerobot needs some
# number for episode_time_s, so this is a ceiling nobody reaches rather than a
# limit anyone is meant to feel.
NO_TIME_LIMIT_SECONDS = 3600

# Readings this close to either end of the 0-4095 range are treated as corrupt
# rather than real. The firmware returns a railed value on a dropped packet, and
# a railed value used as a target drives the arm into its own end stop -- which
# is exactly what happened while this was being built.
RAIL_MARGIN = 6
POSITION_MIN, POSITION_MAX = 0, 4095


def reading_is_sane(values) -> bool:
    """True only if every joint reading is inside the range and off the rails."""
    try:
        return all(
            RAIL_MARGIN < float(v) < POSITION_MAX - RAIL_MARGIN for v in values
        )
    except (TypeError, ValueError):
        return False


class _NoCamera:
    """Stands in for an unplugged camera: never a frame, so its reader keeps looking."""

    def isOpened(self):
        return False

    def read(self):
        time.sleep(0.2)
        return False, None

    def release(self):
        pass


class _StationCamera:
    """A camera for lerobot's record() that reads the station's own running camera.

    record() used to close the station's cameras and open its own. That was most
    of the wait before every try -- DirectShow opens one camera at a time, then
    lerobot warms each one up -- it failed outright when a slow camera's first
    frame took longer than the warm-up, and a camera that stalled mid-try ended
    the try. Now the station's readers keep running through a recording, frozen-
    camera restarts and all, and record() is handed their newest frame.
    """

    def __init__(self, station, name: str, width: int, height: int):
        self.station, self.name = station, name
        self.width, self.height = width, height
        self.is_connected = False
        self._warned = False

    def connect(self, warmup: bool = True) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def read_latest(self, *args, **kwargs):
        return self._frame()

    def async_read(self, *args, **kwargs):
        return self._frame()

    def read(self, *args, **kwargs):
        return self._frame()

    def _frame(self):
        import cv2
        import numpy as np

        bgr = self.station.latest_bgr(self.name)
        if bgr is None:
            if not self._warned:
                self._warned = True
                self.station.say(f"The {self.name} camera has no picture. Throw this try away.")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        if bgr.shape[0] != self.height or bgr.shape[1] != self.width:
            bgr = cv2.resize(bgr, (self.width, self.height))
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


class Station:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = "idle"            # idle | teleop | record
        self.cfg: dict = {}
        self.log_lines: deque[str] = deque(maxlen=400)

        self._caps: dict[str, object] = {}     # camera name -> cv2.VideoCapture
        self._frames: dict[str, bytes] = {}    # camera name -> latest JPEG
        self._bgr: dict[str, object] = {}      # camera name -> latest frame, for record()
        self._frame_lock = threading.Condition()   # also signals a new frame
        self._frame_seq: dict[str, int] = {}       # camera name -> frames so far
        self._fps_window: dict[str, tuple] = {}
        self.camera_fps: dict[str, float] = {}     # measured, for the page
        self._pump_stop = threading.Event()
        self._pump_threads: list[threading.Thread] = []
        self._camera_restarted: dict[str, float] = {}
        self._cam_lock = threading.Lock()         # one camera re-identification at a time
        # Opening and closing the cameras has a lock of its own, never self.lock:
        # with a USB cable acting up, opening a camera or waiting for a stuck
        # reader to let go can take seconds, and no button may wait on that.
        self._capture_lock = threading.RLock()
        self._camera_missing: dict[str, bool] = {}

        self._stop = threading.Event()

        # Teleop / record status surfaced to the browser.
        self.status: dict = {"fps": 0.0, "episode": 0, "target_episodes": 0, "phase": ""}
        self.error: str | None = None

        # Set-up wizard state.
        self.arm_session: robots.ArmFinder | None = None
        self.setup_stage = "unknown"   # unknown | need_arms | dragging | ready
        self.drag_deadline: float = 0.0
        self.cameras_found: list[int] = []

        # Recording controls, read by lerobot's record loop through the patch.
        self.events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

        # Last target sent to each joint, for the per-tick travel limit.
        self._last_sent: dict[str, float] = {}

        # Per-joint safety numbers for a LeRobot arm. None means NexArm, whose
        # numbers are the module constants above.
        self._limits: dict[str, robots.Limits] | None = None

        # Record and replay: still gliding over to where they were told to go.
        self._catching_up = False

        # Teleop has finished its opening sync, so _last_sent is a real pose.
        self._synced = False

        # The gripper guard's memory, per gripper joint; and the follower's
        # latest reading, for modes where lerobot reads it for us.
        self._grip: dict[str, dict] = {}
        self._last_obs: dict | None = None

        # The try that just ended was marked deleted. It is still saved -- see
        # _throw_away -- and only its label says so.
        self._deleted = False

        # The labels for the try about to end, typed on the page, and the
        # folder of the job last recorded, whose labels.csv the page lists.
        self.labels: dict[str, str] = {k: "" for k in LABEL_FIELDS}
        self._labels_root: Path | None = None

        # EMERGENCY STOP: pressed, when, and whether the motors are still being
        # switched off. Cleared by the next Start of anything.
        self._estop = threading.Event()
        self.estop_at: str | None = None
        self._estop_busy = False
        # Every arm connection a worker has open right now, by role, so the
        # emergency stop can switch motors off through it at once.
        self._live: dict[str, object] = {}

        # The page's "is the setup still plugged in" answer, refreshed in the
        # background: listing COM ports can take seconds with a USB cable acting up.
        self._usable: tuple[bool, str] | None = None
        self._usable_at = 0.0
        self._usable_busy = False

        # Port of the second web server that carries only the camera pictures.
        self.stream_port: int | None = None

        # A question for the person at the page -- LeRobot's calibration asks
        # them -- and the Next button that answers it.
        self.prompt: str | None = None
        self._prompt_since = 0.0
        self._next = threading.Event()
        self._cancel = threading.Event()
        self._calibrating = ""

    @property
    def kind(self) -> robots.Kind | None:
        """The kind of arm the saved setup is for.

        Setups saved before there was a choice were all NexArm, and say so.
        """
        return robots.by_key(self.cfg.get("robot_type") or "nexarm")

    # ---------------------------------------------------------------- logging

    def say(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')}  {msg}"
        self.log_lines.append(line)
        print(line, flush=True)

    # ------------------------------------------------------- where it is saved

    def datasets_dir(self) -> Path:
        """Where recordings go. Configurable, and remembered between runs."""
        raw = self.cfg.get("datasets_dir")
        return Path(raw).expanduser() if raw else DATASETS_DIR

    def set_datasets_dir(self, raw: str) -> dict:
        """Point recording at a folder of the user's choosing.

        Checked by actually creating it and writing to it, not by inspecting the
        string. A path can look perfectly valid and still be unwritable, and the
        moment to discover that is now rather than at the end of an episode.
        """
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": "press Stop first"}
            text = (raw or "").strip()
            if not text:
                self.cfg.pop("datasets_dir", None)
                detect.save_config(self.cfg)
                self.say(f"Saving recordings to the default folder, {DATASETS_DIR}")
                return {"ok": True, "datasets_dir": str(DATASETS_DIR)}
            try:
                path = Path(text).expanduser()
                path.mkdir(parents=True, exist_ok=True)
                probe = path / ".station_write_test"
                probe.write_text("ok", encoding="utf-8")
                probe.unlink()
            except Exception as exc:
                return {"ok": False, "why": f"cannot write there: {exc}"}
            self.cfg["datasets_dir"] = str(path)
            detect.save_config(self.cfg)
            self.say(f"Saving recordings to {path}")
            return {"ok": True, "datasets_dir": str(path)}

    def list_datasets(self) -> list[dict]:
        """Recordings already on disk, newest first, for the replay picker."""
        out = []
        root = self.datasets_dir()
        if not root.is_dir():
            return out
        for d in root.iterdir():
            info = d / "meta" / "info.json"
            if not info.is_file():
                continue
            try:
                meta = json.loads(info.read_text(encoding="utf-8"))
                episodes = int(meta.get("total_episodes", 0))
            except Exception:
                episodes = 0
            out.append({
                "name": d.name,
                "episodes": episodes,
                "modified": d.stat().st_mtime,
            })
        out.sort(key=lambda r: r["modified"], reverse=True)
        return out

    # ----------------------------------------------------------------- config

    def load(self) -> tuple[bool, str]:
        self.cfg = detect.load_config() or {}
        ok, why = self.usable()
        self.setup_stage = "ready" if ok else "need_arms"
        if ok:
            self.cameras_found = detect.probe_cameras(log=lambda m: None)
        self.say(why)
        return ok, why

    def usable(self) -> tuple[bool, str]:
        """Is the saved setup still true of what is plugged in?

        An arm whose USB adapter has a serial number is followed to its new port
        first, so moving a cable does not mean doing the wave test again.
        """
        if detect.remap_by_serial(self.cfg):
            detect.save_config(self.cfg)
            self.say(f"Found the arms again on {self.cfg.get('leader_port')} and "
                     f"{self.cfg.get('follower_port')}.")
        ok, why = detect.config_is_usable(self.cfg)
        if ok and self.kind is None:
            return False, (f"this computer does not know the robot type "
                           f"{self.cfg.get('robot_type')!r} -- find the robot again")
        return ok, why

    # ------------------------------------------------------------ set-up flow

    def begin_arm_detect(self, kind: str | None = None) -> dict:
        """Open every candidate port, then start the drag window.

        ``kind`` is set when someone picked the robot on the page instead of
        leaving it to be recognised.
        """
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": "stop what is running first"}
            if kind and robots.by_key(kind) is None:
                return {"ok": False, "why": f"unknown robot type {kind!r}"}
            self.close_cameras()
            if self.arm_session:
                self.arm_session.close()
            self.arm_session = robots.ArmFinder(force=kind or None)
            ports = self.arm_session.open_all(log=self.say)
            if len(ports) < 2:
                self.arm_session.close()
                self.arm_session = None
                self.setup_stage = "need_arms"
                return {
                    "ok": False,
                    "why": f"found {len(ports)} arm(s), need 2. Check both USB cables "
                           f"and that both arms are switched on.",
                    "ports": ports,
                }
            self.setup_stage = "dragging"
            self.drag_deadline = time.monotonic() + 15.0
            self.say(f"Found two {self.arm_session.kind.label} arms. "
                     f"Drag test: wave the arm you hold in your hand.")
            return {"ok": True, "ports": ports, "seconds": 15,
                    "robot": self.arm_session.kind.label}

    def poll_arm_detect(self) -> dict:
        """Called repeatedly by the page while the child waves an arm."""
        with self.lock:
            if not self.arm_session:
                return {"stage": self.setup_stage, "why": "no detection running"}
            moved = self.arm_session.sample()
            remaining = max(0.0, self.drag_deadline - time.monotonic())
            if remaining > 0:
                return {"stage": "dragging", "moved": moved, "remaining": round(remaining, 1)}

            leader, follower = self.arm_session.verdict()
            kind, serials = self.arm_session.kind, dict(self.arm_session.serials)
            self.arm_session.close()
            self.arm_session = None

            if not leader:
                self.setup_stage = "need_arms"
                return {
                    "stage": "failed",
                    "moved": moved,
                    "why": "Could not tell them apart. Move ONE arm, and move it a long "
                           "way. If the arm you hold will not budge, its motors are still "
                           "powered -- switch it off and on, then try again.",
                }

            # A new kind of arm means none of the old kind's names apply.
            # The start position stays: it belongs to the table, not to a USB port,
            # and it only applies to the kind of robot it was saved on anyway.
            for stale in ("baudrate", "leader_id", "follower_id",
                          "leader_serial", "follower_serial"):
                self.cfg.pop(stale, None)
            self.cfg.update({
                "robot_type": kind.key,
                "leader_port": leader,
                "follower_port": follower,
                "fps": self.cfg.get("fps", 30),
                **({"baudrate": detect.BAUD} if kind.raw_counts else {}),
                **robots.arm_ids(kind, leader, follower, serials),
            })
            self.say(f"{kind.label}: leader {leader}, follower {follower}")

            self.say("Looking for cameras.")
            self.cameras_found = detect.probe_cameras(log=self.say)
            if not self.cfg.get("cameras_chosen"):
                self.cfg["cameras"] = detect.guess_cameras(self.cameras_found)
            detect.save_config(self.cfg)
            self.setup_stage = "ready"
            self.open_cameras()
            return {"stage": "ready", "moved": moved, "config": self.cfg}

    def swap_cameras(self) -> dict:
        """Front and wrist guessed the wrong way round. One click, not a prompt."""
        with self.lock:
            cams = self.cfg.get("cameras", {})
            if "front" not in cams or "wrist" not in cams:
                return {"ok": False, "why": "needs two cameras to swap"}
            return self.set_cameras(cams["wrist"], cams["front"])

    def camera_choices(self) -> list[int]:
        """Every camera index known to work: the last search, plus any in use."""
        used = [int(i) for i in self.cfg.get("cameras", {}).values()]
        return sorted(set(self.cameras_found) | set(used))

    def set_cameras(self, front, wrist) -> dict:
        """Choose which camera is the front one and which the wrist one.

        Any camera found can go in either slot, or neither -- which is how the
        laptop's own webcam, the one filming the room, is kept out. Picked from
        the page, so a third camera is just a third choice.
        """
        with self.lock:
            if self.mode not in ("idle", "teleop"):
                return {"ok": False, "why": "press Stop first"}
            choices = self.camera_choices()
            roles = {}
            for role, idx in (("front", front), ("wrist", wrist)):
                if idx is None or idx == "":
                    continue
                try:
                    idx = int(idx)
                except (TypeError, ValueError):
                    return {"ok": False, "why": f"not a camera: {idx!r}"}
                if idx not in choices:
                    return {"ok": False, "why": f"there is no camera {idx}. Look for cameras again."}
                roles[role] = idx
            if len(set(roles.values())) < len(roles):
                return {"ok": False, "why": "pick two different cameras"}
            if "wrist" in roles and "front" not in roles:
                # One camera is always the front one; that is the view a policy needs.
                roles = {"front": roles["wrist"]}
            self.cfg["cameras"] = roles
            self.cfg["cameras_chosen"] = True
        # Outside self.lock: listing and opening cameras can take seconds.
        with self._capture_lock:
            self._remember_camera_ids()
            detect.save_config(self.cfg)
            self.say("Cameras: " + (", ".join(f"{k} = camera {v}" for k, v in roles.items()) or "none"))
            # The person chose; a camera that fails to open is reported, not replaced.
            self.open_cameras(rescanned=True)
        return {"ok": True, "cameras": roles}

    def rescan_cameras(self) -> dict:
        """Look for cameras again, e.g. after plugging one in."""
        with self.lock:
            if self.mode not in ("idle", "teleop"):
                return {"ok": False, "why": "press Stop first"}
        with self._capture_lock:
            self.close_cameras()
            self.say("Looking for cameras.")
            self.cameras_found = detect.probe_cameras(log=self.say)
            self.open_cameras(rescanned=True)
        return {"ok": True, "found": self.cameras_found}

    # ------------------------------------------------- cameras, by identity

    def _remember_camera_ids(self) -> None:
        """Note which physical camera each slot holds, so it can be found again."""
        present = {d["index"]: d["path"] for d in detect.camera_identities() if d["path"]}
        ids = {role: present[idx] for role, idx in self.cfg.get("cameras", {}).items()
               if idx in present}
        if ids:
            self.cfg["camera_ids"] = ids
        else:
            self.cfg.pop("camera_ids", None)

    def _resolve_cameras(self) -> dict[str, int | None]:
        """Where each slot's own camera is now: its number, or None if unplugged.

        Found by its exact device path first (same camera, same port), then by
        its USB model (same camera moved to another port), as long as that model
        is not already taken. A slot whose camera is gone gets None -- never the
        number it used to have, which after an unplug belongs to another camera.
        Updates and saves the config when anything moved.
        """
        with self._cam_lock:
            roles = dict(self.cfg.get("cameras", {}))
            ids = self.cfg.get("camera_ids") or {}
            if not ids:
                return roles                       # nothing known: follow numbers
            present = [d for d in detect.camera_identities() if d["path"]]
            if not present:
                return roles                       # cannot ask Windows: follow numbers
            by_path = {d["path"]: d["index"] for d in present}
            taken = {by_path[p] for p in ids.values() if p in by_path}
            found: dict[str, int | None] = {}
            for role in roles:
                path = ids.get(role)
                idx = by_path.get(path) if path else roles[role]
                if idx is None and path:
                    model = detect.usb_model(path)
                    same = [d["index"] for d in present
                            if detect.usb_model(d["path"]) == model and d["index"] not in taken]
                    idx = same[0] if len(same) == 1 else None
                    if idx is not None:
                        taken.add(idx)
                found[role] = idx
            moved = {r: i for r, i in found.items() if i is not None and i != roles.get(r)}
            if moved:
                for role, idx in moved.items():
                    roles[role] = idx
                    ids[role] = next(d["path"] for d in present if d["index"] == idx)
                    self.say(f"The {role} camera is now camera {idx}. Following it.")
                self.cfg["cameras"], self.cfg["camera_ids"] = roles, ids
                detect.save_config(self.cfg)
            return found

    # ---------------------------------------------------------------- cameras

    def open_cameras(self, rescanned: bool = False) -> None:
        """Station-owned capture, used for the picture when nothing else is running.

        The backend comes from detect.open_capture rather than being named here:
        DirectShow is correct on Windows and makes every index fail to open on
        macOS, which is indistinguishable from having no cameras plugged in.
        """
        with self._capture_lock:
            self.close_cameras()
            failed = []
            where = self._resolve_cameras()
            for name, idx in self.cfg.get("cameras", {}).items():
                if where.get(name, idx) is None:
                    # Unplugged. Its reader waits for it; opening the old number
                    # would show whichever camera moved into that place.
                    self._caps[name] = _NoCamera()
                    self._camera_missing[name] = True
                    self.say(f"The {name} camera is unplugged. It comes back by itself "
                             f"when you plug it in.")
                    continue
                cap = detect.open_capture(idx)
                if not cap.isOpened():
                    failed.append((name, idx))
                if cap.isOpened():
                    self._caps[name] = cap
                    self.say(f"{name} camera {idx}: {detect.capture_format(cap)}")
                else:
                    cap.release()
                    self.say(f"camera {idx} ({name}) would not open")

            # A configured camera that will not open means the setup has changed
            # underneath us -- unplugged, or renumbered by being moved to another
            # socket. Look again and save what is actually there, rather than
            # showing a black rectangle for the rest of the session. Recording
            # reads the same config, so a stale entry here would make lerobot fail
            # on connect too.
            if failed and self.cfg.get("cameras_chosen"):
                # Chosen on the page: report it, never swap in a guess. A camera
                # that is only busy -- held by another program, or a second copy
                # of this station -- comes back as soon as it is free.
                self.say("A chosen camera would not open. Is another program using it? "
                         "Close it, then pick the camera again.")
            elif failed and not rescanned:
                self.say("A camera is missing. Looking again.")
                found = detect.probe_cameras(log=self.say)
                self.cameras_found = found
                fresh = detect.guess_cameras(found)
                if fresh != self.cfg.get("cameras"):
                    self.cfg["cameras"] = fresh
                    detect.save_config(self.cfg)
                    self.say(f"Cameras are now: {fresh or 'none'}")
                    self.open_cameras(rescanned=True)
                    return

            # One reader per camera. A single loop reading each camera in turn
            # waits on every camera's next frame, so the slowest camera set the
            # pace for all of them -- and it also slept to cap the picture at 25
            # frames a second. Now each picture runs as fast as its camera can.
            #
            # Each set of readers gets its own stop signal, set once and never
            # cleared. With one shared signal that close_cameras cleared again, a
            # reader slow to notice -- in the middle of restarting a frozen camera
            # -- woke up to find it clear, carried on, and fought the next reader
            # over the same camera.
            stop = threading.Event()
            self._pump_stop = stop
            for name, cap in self._caps.items():
                idx = self.cfg["cameras"][name]
                t = threading.Thread(target=self._pump_one, args=(name, cap, idx, stop),
                                     name=f"cam-{name}", daemon=True)
                t.start()
                self._pump_threads.append(t)

    def close_cameras(self) -> None:
        # Its own stop signal. This used to share _stop with teleop, so closing
        # the cameras while the arm was moving would also have stopped the arm.
        with self._capture_lock:
            self._pump_stop.set()
            for t in self._pump_threads:
                t.join(timeout=3.0)
            self._pump_threads = []
            for cap in self._caps.values():
                try:
                    cap.release()
                except Exception:
                    pass
            self._caps.clear()

    def _pump_one(self, name: str, cap, idx: int, stop: threading.Event) -> None:
        """Read one station-owned camera into JPEGs for the browser, flat out.

        Only frames whose picture changed are passed on, so the frames-per-second
        number on the page is real; a frozen camera shows as 0 and, after
        CAMERA_FROZEN_SECONDS, is reopened.
        """
        last_sig, last_new = None, time.monotonic()
        # Slow, not frozen: a camera that was giving a proper frame rate and drops
        # to a trickle -- seen on these cameras while the arm moves, 1 fps of real
        # frames -- comes back to full speed when reopened.
        win_start, win_frames, best_fps, slow_windows = time.monotonic(), 0, 0.0, 0
        try:
            while not stop.is_set():
                ok, frame = cap.read()       # blocks until the camera's next frame
                now = time.monotonic()
                if ok and frame is not None:
                    sig = frame[::16, ::16].tobytes()
                    if sig != last_sig:
                        last_sig, last_new = sig, now
                        win_frames += 1
                        self._publish_bgr(name, frame)
                else:
                    time.sleep(0.05)
                restart = now - last_new > CAMERA_FROZEN_SECONDS
                if now - win_start >= 2.0:
                    fps_now = win_frames / (now - win_start)
                    best_fps = max(best_fps, fps_now)
                    slow_windows = slow_windows + 1 if best_fps >= 8 and fps_now < 3 else 0
                    win_start, win_frames = now, 0
                    restart = restart or slow_windows >= 3          # 6 s of a trickle
                if restart:
                    cap = self._restart_camera(name, idx, cap, stop)
                    if cap is None:
                        return
                    last_sig, last_new = None, time.monotonic()
                    win_start, win_frames, slow_windows = time.monotonic(), 0, 0
        finally:
            # A camera this reader opened itself, which close_cameras never saw.
            if self._caps.get(name) is not cap:
                try:
                    cap.release()
                except Exception:
                    pass

    def _restart_camera(self, name: str, idx: int, old, stop: threading.Event):
        """Reopen a frozen or unplugged camera -- its own camera, wherever it is now.

        None if the cameras are being closed meanwhile. While its camera is
        unplugged, a stand-in that never gives a frame is returned, so this runs
        again a few seconds later and finds it the moment it is back.
        """

        now = time.monotonic()
        was_missing = self._camera_missing.get(name, False)
        if not was_missing and now - self._camera_restarted.get(name, 0.0) > 20.0:
            self.say(f"The {name} camera stopped. Restarting it.")
        self._camera_restarted[name] = now
        try:
            old.release()
        except Exception:
            pass
        for _ in range(10):                  # let the driver let go, about a second
            if stop.is_set():
                return None
            time.sleep(0.1)
        where = self._resolve_cameras().get(name, idx)
        if where is None:
            if not was_missing:
                self.say(f"The {name} camera is unplugged. It comes back by itself "
                         f"when you plug it in.")
            self._camera_missing[name] = True
            stand_in = _NoCamera()
            self._caps[name] = stand_in
            return stand_in
        if was_missing:
            self.say(f"The {name} camera is back.")
        self._camera_missing[name] = False
        idx = where
        new = detect.open_capture(idx)
        if stop.is_set():
            new.release()
            return None
        self._caps[name] = new
        return new

    def _publish_bgr(self, name: str, bgr) -> None:
        import cv2

        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        now = time.monotonic()
        with self._frame_lock:
            self._frames[name] = buf.tobytes()
            self._bgr[name] = bgr
            self._frame_seq[name] = self._frame_seq.get(name, 0) + 1
            # Frames per second, measured, so a slow camera shows as slow.
            count, since = self._fps_window.get(name, (0, now))
            count += 1
            if now - since >= 1.0:
                self.camera_fps[name] = round(count / (now - since), 1)
                count, since = 0, now
            self._fps_window[name] = (count, since)
            self._frame_lock.notify_all()

    def latest_bgr(self, name: str):
        with self._frame_lock:
            return self._bgr.get(name)

    def latest_jpeg(self, name: str) -> bytes | None:
        with self._frame_lock:
            return self._frames.get(name)

    def wait_frame(self, name: str, after: int, timeout: float = 1.0):
        """The next frame from camera ``name`` newer than number ``after``.

        Returns (jpeg, number), or (None, after) on timeout. The page's video
        streams wait on this instead of polling at a fixed rate, so each new
        frame goes out the moment it exists.
        """
        with self._frame_lock:
            self._frame_lock.wait_for(lambda: self._frame_seq.get(name, 0) != after, timeout)
            seq = self._frame_seq.get(name, 0)
            if seq == after:
                return None, after
            return self._frames.get(name), seq

    def camera_names(self) -> list[str]:
        return list(self.cfg.get("cameras", {}).keys())

    # ----------------------------------------------------------------- teleop

    def _clear_estop(self) -> dict | None:
        """A new Start acknowledges an emergency stop. Not while it is still working."""
        if self._estop_busy:
            return {"ok": False, "why": "wait a moment -- the motors are still being switched off"}
        self._estop.clear()
        self.estop_at = None
        return None

    def start_teleop(self) -> dict:
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": f"already {self.mode}"}
            refused = self._clear_estop()
            if refused:
                return refused
            ok, why = self.usable()
            if not ok:
                return {"ok": False, "why": why}
            self.error = None
            self.mode = "teleop"
            self._last_sent = {}
            self._synced = False
            self._stop.clear()
            self._cancel.clear()
            threading.Thread(target=self._teleop_worker, name="teleop", daemon=True).start()
            return {"ok": True}

    def _teleop_worker(self) -> None:
        from lerobot.utils.robot_utils import precise_sleep

        fps = int(self.cfg.get("fps", 30))
        follower = leader = None
        follower_cls = real_configure = None
        try:
            # Cameras stay with the station here, so the robot is built with none.
            follower = self._make_follower(cameras={})
            leader = self._make_leader()
            follower_cls = type(follower)
            real_configure = self._park_goal_before_torque(follower_cls)
            self._limits = self._limits_for(follower)
            with self._page_prompts():
                self._calibrating = robots.which_arm("follower")
                follower.connect()
                self._calibrating = robots.which_arm("leader")
                leader.connect()
            self._live.update(follower=follower, leader=leader)
            self.say("Arms connected. Easing the follower over to match the leader.")
            if not self._first_sync(follower, leader, fps):
                # Stop pressed during the opening glide is not a fault; say so only
                # when the arms really did report something unbelievable.
                if not (self._stop.is_set() or self._cancel.is_set()):
                    self.say("Not moving: the arms are not reporting sane positions yet. "
                             "Switch both off and on, then try again.")
                return
            self._synced = True
            self.say("Teleop running. Move the arm in your hand.")

            ticks, t_window = 0, time.perf_counter()
            self._grip = {}
            while not self._stop.is_set():
                start = time.perf_counter()
                action = leader.get_action()
                try:
                    obs = follower.get_observation()   # where the gripper really is
                except Exception:
                    obs = None
                action = self._guard_gripper(action, obs)
                if self._estop.is_set():
                    break                       # never another move after the button
                if self._sane(action):
                    follower.send_action(self._rate_limit(action))
                ticks += 1
                if start - t_window >= 1.0:
                    self.status["fps"] = round(ticks / (start - t_window), 1)
                    ticks, t_window = 0, start
                precise_sleep(1.0 / fps - (time.perf_counter() - start))
        except robots.Cancelled:
            self.say("Stopped before the arms were ready.")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Teleop stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            self._live.clear()
            if self._estop.is_set():
                for dev in (follower, leader):
                    if dev is not None:
                        robots.torque_off(dev)
            elif follower is not None and self._synced:
                robots.soft_hold(follower)
            if follower_cls is not None:
                follower_cls.configure = real_configure
            for dev in (follower, leader):
                try:
                    if dev is not None:
                        dev.disconnect()
                except Exception:
                    pass
            with self.lock:
                self.mode = "idle"
                self.status["fps"] = 0.0
                self._synced = False
            self.say("Teleop stopped.")

    # ------------------------------------------------------- which arm it is

    def _follower_config(self, cameras: dict):
        kind = self.kind
        if kind.raw_counts:
            from lerobot.robots.nexarm_follower import NexArmFollowerConfig

            return NexArmFollowerConfig(
                port=self.cfg["follower_port"], cameras=cameras,
                motion_acc=MOTION_ACC, motion_speed=MOTION_SPEED)
        return robots.follower_config(
            kind, self.cfg["follower_port"], self.cfg.get("follower_id"), cameras)

    def _leader_config(self):
        kind = self.kind
        if kind.raw_counts:
            from lerobot.teleoperators.nexarm_leader import NexArmLeaderConfig

            return NexArmLeaderConfig(port=self.cfg["leader_port"])
        return robots.leader_config(kind, self.cfg["leader_port"], self.cfg.get("leader_id"))

    def _make_follower(self, cameras: dict):
        """Build the follower. Nothing is opened or moved until connect()."""
        if self.kind.raw_counts:
            from lerobot.robots.nexarm_follower import NexArmFollower

            return NexArmFollower(self._follower_config(cameras))
        from lerobot.robots.utils import make_robot_from_config

        return make_robot_from_config(self._follower_config(cameras))

    def _make_leader(self):
        if self.kind.raw_counts:
            from lerobot.teleoperators.nexarm_leader import NexArmLeader

            return NexArmLeader(self._leader_config())
        from lerobot.teleoperators.utils import make_teleoperator_from_config

        return make_teleoperator_from_config(self._leader_config())

    def _limits_for(self, follower) -> dict[str, robots.Limits] | None:
        return None if self.kind.raw_counts else robots.lerobot_limits(follower)

    def _sane(self, values: dict) -> bool:
        """Every joint reading believable, in this arm's own units."""
        if not self._limits:
            return reading_is_sane(values.values())
        for key, value in values.items():
            lim = self._limits.get(key)
            if lim is not None:
                if not lim.sane(value):
                    return False
            else:
                try:
                    if not math.isfinite(float(value)):
                        return False
                except (TypeError, ValueError):
                    return False
        return True

    # ------------------------------------------------- calibration on the page

    @contextlib.contextmanager
    def _page_prompts(self):
        """Answer LeRobot's terminal prompts from the page instead.

        LeRobot calibrates an arm the first time it is used on a computer, and it
        does that with ``input()`` and by polling the keyboard for Enter. There is
        no keyboard here, only a page, so both are pointed at the page's Next
        button for as long as the arms are connecting. A NexArm never asks.
        """
        import sys

        real_input = builtins.input
        # Every motor bus module that polls the keyboard holds its own reference
        # to enter_pressed: the shared MotorsBus, and a few buses (Damiao,
        # Robstride) that carry their own copy of the range-recording loop.
        enter_owners = [m for name, m in list(sys.modules.items())
                        if name.startswith("lerobot.motors") and hasattr(m, "enter_pressed")]
        real_enters = [(m, m.enter_pressed) for m in enter_owners]
        station = self

        def page_input(prompt=""):
            text = str(prompt)
            if "provided calibration file" in text:
                # This arm already has a calibration on this computer: use it.
                return ""
            station._calibrating = robots.which_arm(text) or station._calibrating
            station._ask(robots.friendly_prompt(text, station._calibrating))
            return ""

        def page_enter():
            if station.prompt is None:
                station._set_prompt(robots.ranges_prompt(station._calibrating))
            return station._take_next()

        builtins.input = page_input
        for m, _ in real_enters:
            m.enter_pressed = page_enter
        try:
            yield
        finally:
            builtins.input = real_input
            for m, fn in real_enters:
                m.enter_pressed = fn
            self.prompt = None

    # A Next that arrives sooner than this after a question appeared is ignored.
    # Children double-click, and the second click of a double-click on "put every
    # joint halfway" landed on the next question, "move every joint to both
    # ends", and ended it before anything had moved -- which LeRobot rejects with
    # "Some motors have the same min and max values", failing the calibration.
    NEXT_DEBOUNCE_S = 1.0

    def _set_prompt(self, text: str) -> None:
        self._next.clear()
        self._prompt_since = time.monotonic()
        self.prompt = text
        self.say(text)

    def _ask(self, text: str) -> None:
        """Show a question and wait for Next. Stop cancels the wait."""
        self._set_prompt(text)
        while not self._next.wait(0.2):
            if self._cancel.is_set():
                self.prompt = None
                raise robots.Cancelled("stopped from the page")
        self._next.clear()
        self.prompt = None

    def _take_next(self) -> bool:
        if self._cancel.is_set():
            self.prompt = None
            raise robots.Cancelled("stopped from the page")
        if self._next.is_set():
            self._next.clear()
            self.prompt = None
            return True
        return False

    def prompt_next(self) -> dict:
        """The page's Next button. Never an error to press it: at worst it is ignored."""
        if self.prompt is None:
            return {"ok": True, "ignored": "nothing is waiting for Next"}
        if time.monotonic() - self._prompt_since < self.NEXT_DEBOUNCE_S:
            return {"ok": True, "ignored": "too soon after the question appeared"}
        self._next.set()
        return {"ok": True}

    @staticmethod
    def _park_goal_before_torque(follower_cls):
        """Stop the follower snapping when its motors switch on.

        ``configure()`` switches the torque on, and the servos come up aiming at
        whatever Goal_Position was last written to them -- which, after a
        previous session, is a pose from minutes ago. The arm lunges there the
        instant torque arrives, before any of our code has sent an action. From
        the outside this looks like the opening sync jerking, because it happens
        one moment before the sync starts.

        The fix is to write where the arm actually IS as the goal, and only then
        allow the torque on. Then switching the motors on holds it still. The
        writing itself is in robots.park_goal, because it differs per kind of bus.

        Returns the original configure so the caller can put it back.
        """
        real_configure = follower_cls.configure

        def parked_configure(self_robot):
            # Worst case we are no worse off than the stock behaviour.
            robots.full_torque(self_robot.bus)      # undo a soft hold left from before
            robots.park_goal(self_robot.bus)
            return real_configure(self_robot)

        follower_cls.configure = parked_configure
        return real_configure

    def _guard_numbers(self, key: str) -> tuple[float, float, float]:
        """The guard's thresholds in this arm's units: (stall error, squeeze, still)."""
        lim = self._limits.get(key) if self._limits else None
        if lim is None:                      # NexArm: raw counts
            return GRIP_STALL_ERROR, GRIP_SQUEEZE, GRIP_STILL
        if lim.hi - lim.lo > 300:            # degrees: 40 counts is about 3.5 degrees
            return 3.5, 1.3, 0.3
        return 3.0, 1.0, 0.25                # percent of the gripper's range

    def _guard_gripper(self, action: dict, obs: dict | None) -> dict:
        """Never let the gripper keep pushing against something it cannot move.

        When the jaw has stopped moving while its target is well past it, the target
        becomes where the jaw is plus a small squeeze: enough to hold a block, not
        enough to trip. Squeezing or releasing the trigger moves the target the other
        way, the jaw moves, and the guard lets go at once. Joints other than the
        gripper are never touched.
        """
        if not obs:
            return action
        out = dict(action)
        for key, raw in action.items():
            if not key.endswith("gripper.pos") or key not in obs:
                continue
            try:
                actual, target = float(obs[key]), float(raw)
            except (TypeError, ValueError):
                continue
            if not self._sane({key: actual}):
                continue                    # a dropped packet, not a stuck jaw
            stall_error, squeeze, still = self._guard_numbers(key)
            mem = self._grip.setdefault(key, {"last": actual, "still": 0, "held": False})
            gap = target - actual
            # Stalled means pushing AND not moving, tick after tick. Counting every
            # tick the jaw sat still -- pushing or not -- made the first tick of any
            # new command look like a stall: a jaw resting closed was pinned where
            # it was the moment the trigger let go, before it could start to open,
            # and stayed shut until the next squeeze. A recorded try showed exactly
            # that: closed for 10 s with the trigger released.
            pushing = abs(gap) > stall_error
            same_way = (gap > 0) == mem.get("up", gap > 0)
            if pushing and same_way and abs(actual - mem["last"]) <= still:
                mem["still"] += 1
            else:
                mem["still"] = 0
            mem["up"] = gap > 0
            mem["last"] = actual
            if mem["held"]:
                # Stay held through sensor jitter -- letting go on every few counts
                # of noise made it strain again a dozen times in ten seconds. Let
                # go only when the trigger asks for the other way, or when the jaw
                # really moved (the block slipped out, say).
                asks_back = gap * mem["dir"] <= stall_error
                jaw_moved = abs(actual - mem["at"]) > 2 * squeeze
                if asks_back or jaw_moved:
                    mem["held"] = False
                    mem["still"] = 0
                else:
                    out[key] = mem["at"] + mem["dir"] * squeeze
                    continue
            if abs(gap) > stall_error and mem["still"] >= GRIP_STALL_TICKS:
                mem.update(held=True, dir=1.0 if gap > 0 else -1.0, at=actual)
                out[key] = actual + mem["dir"] * squeeze
                now = time.monotonic()
                if now - mem.get("said", 0.0) > 10.0:
                    mem["said"] = now
                    self.say("Gripper is stopped (end of its travel, or holding something): "
                             "easing off so it does not strain.")
        return out

    def _step_for(self, key: str, sync: bool) -> float:
        lim = self._limits.get(key) if self._limits else None
        if lim is not None:
            return lim.sync_step if sync else lim.step
        return SYNC_STEP_PER_TICK if sync else MAX_STEP_PER_TICK

    def _rate_limit(self, action: dict, max_step: float | None = None,
                    sync: bool = False) -> dict:
        """Never let a target be more than one tick's allowance from the last one.

        Without this, one bad leader reading is a lunge. With it, the worst a bad
        reading can do is start a slow drift that Stop or the next good reading
        ends. The cost is that a genuinely fast hand movement is followed slightly
        behind, which for demonstrating a task to a robot is a fair trade.

        The allowance is MAX_STEP_PER_TICK counts on a NexArm and the joint's own
        limit on anything else; ``sync`` selects the slow allowance used while the
        follower is first walked over to the leader.
        """
        limited = {}
        for k, raw in action.items():
            want = float(raw)
            prev = self._last_sent.get(k)
            if prev is None:
                limited[k] = want
            else:
                allowed = max_step if max_step is not None else self._step_for(k, sync)
                step = max(-allowed, min(allowed, want - prev))
                limited[k] = prev + step
        self._last_sent = limited
        return limited

    def _first_sync(self, follower, leader, fps: int, quick_if_matched: bool = False) -> bool:
        """Walk the follower to the leader's pose slowly before live teleop starts.

        ``quick_if_matched`` skips the ramp when the arms already match to within
        one tick's allowance -- recording does this before every try, and after a
        teleop session that held its pose there is nothing to walk.

        At startup the two arms are almost never in the same pose, so an unguarded
        first send_action asks the follower to cross the whole gap in one tick. It
        slams and it clicks. Easing across hands the loop two arms already matched.

        Both poses are checked for sanity first. A railed reading here is the
        dangerous case: used as a target it drives the arm into its own end stop.
        Returns False if the arms are not reporting believable positions.

        The duration is worked out from how far it has to go, and every tick is
        additionally capped by SYNC_STEP_PER_TICK, so this move is slow no matter
        what the gap turns out to be.
        """
        from lerobot.utils.robot_utils import precise_sleep

        target = leader.get_action()
        obs = follower.get_observation()
        start = {k: float(obs[k]) for k in target if k in obs}
        if not start:
            return False
        if not self._sane(start):
            self.say(f"Follower reading looks wrong: {[round(v) for v in start.values()]}")
            return False
        if not self._sane({k: target[k] for k in start}):
            self.say(f"Leader reading looks wrong: {[round(target[k]) for k in start]}")
            return False

        gap = max(abs(target[k] - start[k]) for k in start)
        if quick_if_matched and all(
                abs(target[k] - start[k]) <= self._matched(k) for k in start):
            self._last_sent = dict(start)
            return True

        # Time the sync to the distance rather than using a fixed number, and
        # cap every tick as well. Either alone is not enough: a fixed duration
        # makes a large gap fast, and an eased curve still peaks in the middle.
        # Each joint is timed in its own units; the slowest one sets the pace.
        seconds = max(abs(target[k] - start[k]) / self._sync_rate(k) for k in start)
        seconds = max(SYNC_MIN_SECONDS, min(SYNC_MAX_SECONDS, seconds))
        self.say(f"Closing a gap of {gap:.0f} gently, over about {seconds:.0f} seconds.")

        self._last_sent = dict(start)
        steps = max(1, int(seconds * fps))
        for i in range(1, steps + 1):
            # _cancel too: during a recording, Stop sets that and not _stop.
            if self._stop.is_set() or self._cancel.is_set():
                return False
            f = i / steps
            e = f * f * (3.0 - 2.0 * f)      # ease in and out, no jerk at either end
            t0 = time.perf_counter()
            follower.send_action(self._rate_limit(
                {k: start[k] + (target[k] - start[k]) * e for k in start},
                sync=True))
            precise_sleep(1.0 / fps - (time.perf_counter() - t0))

        # The ramp is eased, and the slow per-tick cap can hold its middle back,
        # so it can end a little short of the target -- more so now that a short
        # gap is not padded out to 2 s. Finish the last stretch at the same slow
        # cap so it always arrives.
        deadline = time.monotonic() + 3.0
        while any(abs(self._last_sent.get(k, start[k]) - target[k]) > 1.0 for k in start):
            if self._stop.is_set() or self._cancel.is_set():
                return False
            if time.monotonic() > deadline:
                break
            t0 = time.perf_counter()
            follower.send_action(self._rate_limit({k: target[k] for k in start}, sync=True))
            precise_sleep(1.0 / fps - (time.perf_counter() - t0))

        # Hand the live loop a clean slate: the sync ended at the target, and the
        # loop's own limit starts from there.
        self._last_sent = dict(self._last_sent)
        self.say("Synced.")
        return True

    def _matched(self, key: str) -> float:
        """Close enough to count as in sync: two ticks of the slow sync allowance.

        The handover from the slow glide to full-speed following happens here, so
        it has to be small. It was one tick of the FULL allowance (250 counts on a
        NexArm), which let the end of every glide finish with a 20-degree jump.
        """
        return 2 * self._step_for(key, sync=True)

    def _sync_rate(self, key: str) -> float:
        lim = self._limits.get(key) if self._limits else None
        return lim.sync_rate if lim is not None else SYNC_COUNTS_PER_SECOND

    def _presync(self, fps: int) -> bool:
        """Walk the follower over to the leader before a recording starts.

        Recording used to start by sending the leader's pose straight to the
        follower. After Stop the follower has switched its motors off and
        dropped, so the first frame of every try jumped it up from the table at
        full speed -- at exactly the moment a child had just put the block down
        beside it. Teleop never did this because it walks the follower over
        first; now recording does the same.

        The follower is let go of with its motors still on, so it holds the
        leader's pose for the moment it takes record() to connect again.
        """
        follower = leader = None
        try:
            follower = self._make_follower(cameras={})
            follower.config.disable_torque_on_disconnect = False
            leader = self._make_leader()
            follower.connect()
            leader.connect()
            self._live.update(follower=follower, leader=leader)
            self.say("Lining the robot up with the arm you hold. Keep still for a moment.")
            if not self._first_sync(follower, leader, fps, quick_if_matched=True):
                self.say("Not recording: the arms are not reporting sane positions yet, "
                         "or Stop was pressed. Switch both off and on if it keeps happening.")
                return False
            return True
        finally:
            self._live.clear()
            for dev in (follower, leader):
                try:
                    if dev is not None:
                        if self._estop.is_set():
                            robots.torque_off(dev)
                        dev.disconnect()
                except Exception:
                    pass

    def _measured_pose(self, robot, action: dict) -> dict:
        """Where the follower is now, in the units of ``action``. {} if unreadable."""
        try:
            obs = robot.get_observation()
            pose = {k: float(obs[k]) for k in action if k in obs}
        except Exception:
            pose = {}
        if pose and self._sane(pose):
            return pose
        self.say("Could not read where the arm is, so its first move is not eased.")
        return {}

    def _seed_from_arm(self, robot, action: dict) -> None:
        """Start the travel limit from where the arm IS, and glide until caught up.

        For record() and replay(), which drive the follower themselves. The limit
        used to start from the first target instead -- which let the very first
        move through unlimited: a full-speed jump to the leader's pose when
        recording, or to the first frame of a recording when replaying, from
        wherever the arm had dropped to.
        """
        self._last_sent = self._measured_pose(robot, action)
        self._catching_up = bool(self._last_sent)

    def _glide_in(self, action: dict) -> dict:
        """Rate-limit one action; the slow sync allowance until the arm has caught up."""
        if not self._catching_up:
            return self._rate_limit(action)
        out = self._rate_limit(action, sync=True)
        if all(abs(float(action[k]) - out[k]) <= self._matched(k) for k in out):
            self._catching_up = False
        return out

    # ----------------------------------------------------------------- record

    def start_record(self, task: str) -> dict:
        """Record exactly one try, for as long as it takes.

        There is no episode timer and no reset timer. A child doing a job does
        not know in advance how many seconds it will take, and a countdown that
        cuts them off mid-reach produces a truncated demonstration that is worse
        than no demonstration. The try ends when they say it ends.

        Pressing this again for the same job name appends another try to the same
        dataset rather than starting a new one, so repeated recordings build up
        something trainable instead of scattering single-episode folders.
        """
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": f"already {self.mode}"}
            ok, why = self.usable()
            if not ok:
                return {"ok": False, "why": why}
            if not task.strip():
                return {"ok": False, "why": "give the job a name first"}
            # The station's own readers already know -- no camera search here,
            # which held the button for as long as Windows took to list cameras.
            missing = [n for n in self.camera_names()
                       if self._camera_missing.get(n) or self.latest_bgr(n) is None]
            if missing:
                return {"ok": False, "why": f"the {' and '.join(missing)} camera has no picture. "
                                            f"Plug it back in, or wait a moment -- it comes "
                                            f"back by itself."}
            refused = self._clear_estop()
            if refused:
                return refused
            self.error = None
            self.mode = "record"
            self._stop.clear()      # a teleop Stop must not cancel this try's sync
            self._cancel.clear()
            self._deleted = False
            self.events.update(exit_early=False, rerecord_episode=False, stop_recording=False)
            self.status.update(episode=0, target_episodes=1, phase="starting")
            # The cameras stay open: record() reads them through _StationCamera.
            threading.Thread(
                target=self._record_worker, args=(task.strip(),),
                name="record", daemon=True,
            ).start()
            return {"ok": True}

    def record_config(self, task: str):
        """Exactly what is handed to lerobot's record() for one try of ``task``.

        Built here and nowhere else, so the selftest checks the real thing. The
        dataset folder is named after the kind of arm as well as the job: a
        recording only makes sense played back, or trained on, with the same kind.
        """
        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.configs.dataset import DatasetRecordConfig
        from lerobot.scripts import lerobot_record as lr

        fps = int(self.cfg.get("fps", 30))
        slug = "".join(c if c.isalnum() else "_" for c in task.lower()).strip("_")[:40] or "task"
        # These only give the dataset its picture size: record() is handed the
        # station's running cameras (_StationCamera) and never opens these.
        cams = {
            name: OpenCVCameraConfig(index_or_path=int(idx), width=640, height=480, fps=fps)
            for name, idx in self.cfg.get("cameras", {}).items()
        }
        root = self.datasets_dir() / f"{self.kind.key}_{slug}"

        # Already recorded this job before? Add to it instead of colliding
        # with it. LeRobotDataset.create refuses to write over an existing
        # dataset, and a child pressing the same button twice should get a
        # second try, not an error.
        resume = (root / "meta" / "info.json").is_file()

        return lr.RecordConfig(
            robot=self._record_robot_config(cams),
            teleop=self._leader_config(),
            dataset=DatasetRecordConfig(
                repo_id=f"local_user/{root.name}",
                single_task=task,
                root=str(root),
                fps=fps,
                # One try per press. The stop comes from the button, not a
                # clock, so the episode limit is a ceiling that should never
                # be reached -- an hour of continuous recording.
                num_episodes=1,
                episode_time_s=NO_TIME_LIMIT_SECONDS,
                # No tidy-up window either. With a single episode lerobot
                # skips the reset phase anyway; this makes it instant if a
                # re-record ever reaches it.
                reset_time_s=0,
                # Encode the video while the try is being recorded, as lerobot
                # recommends. Otherwise every Done waited while the whole try was
                # encoded first -- 7 s for a 6 s try on the class laptop -- before
                # the robot could even start back to the start position.
                streaming_encoding=True,
                encoder_threads=2,
                # The dataset is video of the room, and the room has children in
                # it. lerobot's own default here is True, which would publish it
                # to a public Hugging Face dataset the moment recording finished.
                push_to_hub=False,
            ),
            resume=resume,
            display_data=False,     # frames go to the browser, not to rerun
            play_sounds=False,
        )

    def _record_worker(self, task: str) -> None:
        from lerobot.scripts import lerobot_record as lr

        # Three patches, all undone in the finally block. They go on the
        # follower's class because record() builds its own instance of it.
        real_listener = lr.init_keyboard_listener
        try:
            probe = self._make_follower(cameras={})     # for its class and limits only
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Recording could not start: {self.error}")
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""
            return
        follower_cls = type(probe)
        self._limits = self._limits_for(probe)
        real_get_obs = follower_cls.get_observation
        real_send = follower_cls.send_action
        real_connect = follower_cls.connect
        station = self
        self._last_sent = {}
        began = time.monotonic()

        def station_connect(self_robot, *args, **kwargs):
            """record()'s robot gets the station's running cameras, and is noted as live."""
            for cam_key in list(getattr(self_robot, "cameras", {})):
                cam_cfg = self_robot.config.cameras[cam_key]
                self_robot.cameras[cam_key] = _StationCamera(
                    station, cam_key, cam_cfg.width or 640, cam_cfg.height or 480)
            if station._estop.is_set():
                raise robots.Cancelled()
            out = real_connect(self_robot, *args, **kwargs)
            station._live["follower"] = self_robot
            if station._estop.is_set():
                robots.torque_off(self_robot)
            return out

        def browser_listener():
            """The browser's buttons stand in for the keyboard."""
            return None, station.events

        def teeing_get_obs(self_robot):
            # The page's pictures come straight from the station's own readers,
            # which keep running through the recording.
            if station._estop.is_set():
                robots.torque_off(self_robot)
            obs = real_get_obs(self_robot)
            station._last_obs = obs                  # for the gripper guard
            return obs

        def limited_send(self_robot, action):
            """Same travel limit as teleop.

            record_loop calls send_action itself, so the limit has to live on the
            robot rather than in the caller, or recording would be the one mode
            that can still lunge. After _presync the arm is already where the
            leader is, so this is the ordinary limit; _glide_in covers the case
            where nothing has been sent yet.
            """
            if station._estop.is_set():
                robots.torque_off(self_robot)
                return dict(station._last_sent) or action
            if station.status.get("phase") != "recording":
                # The first frame of the try. Until now lerobot was still opening
                # the cameras, and the page said to hold still.
                station.status["phase"] = "recording"
                station.say(f"Recording now (ready in {time.monotonic() - began:.1f} s). "
                            f"Do the job, then press Done.")
            if not station._last_sent:
                station._seed_from_arm(self_robot, action)
            action = station._guard_gripper(action, station._last_obs)
            if not station._sane(action):
                return dict(station._last_sent) or action
            return real_send(self_robot, station._glide_in(action))

        real_configure = self._park_goal_before_torque(follower_cls)
        self._catching_up = False
        self._grip, self._last_obs = {}, None
        go_home = False
        try:
            # The first time a LeRobot arm is used on this computer it calibrates
            # while connecting, here or in record(), and the questions come to
            # the page.
            with self._page_prompts():
                if not self._presync(int(self.cfg.get("fps", 30))):
                    return

                lr.init_keyboard_listener = browser_listener
                follower_cls.get_observation = teeing_get_obs
                follower_cls.send_action = limited_send
                follower_cls.connect = station_connect

                self.datasets_dir().mkdir(parents=True, exist_ok=True)
                cfg = self.record_config(task)
                root = Path(cfg.dataset.root)
                try_no = self._episodes_in(root)
                self._labels_root = root
                self.status.update(phase="starting", root=str(root), try_no=try_no, job=task)
                self.say(f'Getting ready to record "{task}". Hold still a moment.')
                self.say(f"{'Adding to' if cfg.resume else 'Saving to'} {root} on this computer only.")
                self._watch_episodes()
                lr.record(cfg)
            if self._episodes_in(root) > try_no:
                self._write_label(root, try_no, task)
                self.say(f"Try {try_no + 1} saved, marked DELETED." if self._deleted
                         else f"Try {try_no + 1} saved.")
            else:
                self.say("This try was not saved.")
            go_home = not self._cancel.is_set()
        except robots.Cancelled:
            self.say("Stopped before the arms were ready. Nothing was recorded.")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Recording stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            self._live.clear()
            lr.init_keyboard_listener = real_listener
            follower_cls.get_observation = real_get_obs
            follower_cls.send_action = real_send
            follower_cls.connect = real_connect
            # After record()'s own patches are off: going back must use the plain
            # send, not the recording's -- whose gripper guard reads the last
            # observation record() took, which stops updating when it returns.
            # The park-before-torque patch stays on for this connect.
            if go_home:
                self._return_to_start(int(self.cfg.get("fps", 30)))
            if not self._estop.is_set():
                self._soft_hold_follower()
            follower_cls.configure = real_configure
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""
                self.status.pop("root", None)

    # ----------------------------------------------------------------- labels

    @staticmethod
    def _episodes_in(root: Path) -> int:
        """How many tries a job's dataset holds: the number the next one gets."""
        try:
            info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
            return int(info.get("total_episodes", 0))
        except Exception:
            return 0

    def set_labels(self, body: dict) -> dict:
        for k in LABEL_FIELDS:
            if k in body:
                self.labels[k] = str(body.get(k) or "").strip()[:200]
        return {"ok": True, "labels": self.labels}

    def _write_label(self, root: Path, try_no: int, task: str) -> None:
        """One line per try in <job>/labels.csv -- kept and deleted tries alike.

        The try number is the one the page shows and the sheet gets: the dataset's
        episode number plus one, so children count from 1. Both are written. Deleted tries stay in the dataset;
        leave them out when training by this file's status column.
        """
        path = root / "labels.csv"
        lab = self.labels
        row = {"try": try_no + 1, "episode": try_no, "status": "deleted" if self._deleted else "kept", "job": task,
               "driver": lab["driver"], "recorder": lab["recorder"], "labeler": lab["labeler"],
               "position": lab["position"], "something_wrong": lab["wrong"],
               "type": lab["type"], "notes": lab["notes"],
               "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")}
        try:
            new = not path.is_file()
            with path.open("a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=LABEL_COLUMNS)
                if new:
                    w.writeheader()
                w.writerow(row)
        except Exception as exc:
            self.say(f"Could not write the labels: {type(exc).__name__}: {exc}")
        # Per-try answers start blank again; who is doing which job stays.
        self.labels.update(wrong="", notes="")

    def recent_labels(self, limit: int = 12) -> list[dict]:
        root = self._labels_root
        if root is None or not (root / "labels.csv").is_file():
            return []
        try:
            with (root / "labels.csv").open(newline="", encoding="utf-8") as f:
                return list(csv.DictReader(f))[-limit:]
        except Exception:
            return []

    def _watch_episodes(self) -> None:
        """Count saved episodes off disk so the page has a progress number.

        record() does not expose its counter, and reaching into its locals would
        break on the next lerobot bump. The episode directory is a stable thing
        to count instead -- the one this try is going into, which the worker
        names in status["root"].
        """
        def run():
            while self.mode == "record":
                root = self.status.get("root")
                if root:
                    # info.json, not the parquet files: one file holds many episodes.
                    self.status["episode"] = self._episodes_in(Path(root))
                time.sleep(1.0)
        threading.Thread(target=run, name="ep-watch", daemon=True).start()

    # ----------------------------------------------------------------- replay

    def start_replay(self, name: str, episode: int) -> dict:
        """Play a recorded episode back on the arm, with nobody holding the leader."""
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": f"already {self.mode}"}
            ok, why = self.usable()
            if not ok:
                return {"ok": False, "why": why}
            root = self.datasets_dir() / name
            if not (root / "meta" / "info.json").is_file():
                return {"ok": False, "why": f"no recording called {name}"}
            refused = self._clear_estop()
            if refused:
                return refused
            other = self._recorded_on_other_kind(root)
            if other:
                return {"ok": False, "why": f"{name} was recorded on a {other}. "
                                            f"This computer is set up for the {self.kind.label}."}
            self.error = None
            self.mode = "replay"
            self._cancel.clear()
            self.status.update(phase=f"replaying {name} episode {episode}")
            threading.Thread(target=self._replay_worker, args=(name, episode),
                             name="replay", daemon=True).start()
            return {"ok": True}

    def _recorded_on_other_kind(self, root: Path) -> str | None:
        """The robot a recording was made on, if it is not the one plugged in.

        Replaying one arm's joint values on a different kind of arm is at best an
        error from lerobot and at worst a move nobody intended, so it is refused
        up front. A recording that does not say which robot made it is allowed.
        """
        try:
            recorded = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8")
                                  ).get("robot_type")
            here = self._make_follower(cameras={}).name
        except Exception:
            return None
        return recorded if recorded and recorded != here else None

    def _replay_worker(self, name: str, episode: int) -> None:
        from lerobot.scripts import lerobot_replay as lrp

        try:
            probe = self._make_follower(cameras={})     # for its class and limits only
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Replay could not start: {self.error}")
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""
            return
        follower_cls = type(probe)
        self._limits = self._limits_for(probe)
        real_send = follower_cls.send_action
        real_get_obs = follower_cls.get_observation
        station = self
        self._last_sent = {}
        self._grip, self._last_obs = {}, None
        root = self.datasets_dir() / name

        def keeping_get_obs(self_robot):
            obs = real_get_obs(self_robot)
            station._last_obs = obs                  # for the gripper guard
            return obs

        def limited_send(self_robot, action):
            # Same jump guard as teleop and recording. A dataset can contain a
            # railed frame just as a live reading can, and on replay there is no
            # hand on the leader to notice it going wrong.
            #
            # Stop is checked here because lerobot's replay() is one blocking
            # call with no cancel. This is the only place the loop passes through
            # often enough to abort it promptly.
            if station._stop.is_set():
                if station._estop.is_set():
                    robots.torque_off(self_robot)
                raise KeyboardInterrupt("stopped from the page")
            # The arm starts wherever it was left -- usually dropped, after Stop
            # -- and the first frame of the recording can be anywhere. So the
            # limit starts from where the arm is, and it glides over first.
            if not station._last_sent:
                station._seed_from_arm(self_robot, action)
            action = station._guard_gripper(action, station._last_obs)
            if not station._sane(action):
                return dict(station._last_sent) or action
            return real_send(self_robot, station._glide_in(action))

        real_configure = self._park_goal_before_torque(follower_cls)
        self._catching_up = False
        try:
            self._stop.clear()
            follower_cls.send_action = limited_send
            follower_cls.get_observation = keeping_get_obs
            self.say(f"Replaying {name}, episode {episode}. Keep hands clear.")
            cfg = lrp.ReplayConfig(
                robot=self._follower_config({}),
                dataset=lrp.DatasetReplayConfig(
                    repo_id=f"local_user/{name}", root=str(root),
                    episode=int(episode), fps=int(self.cfg.get("fps", 30)),
                ),
                play_sounds=False,
            )
            with self._page_prompts():
                lrp.replay(cfg)
            self.say("Replay finished.")
        except (KeyboardInterrupt, robots.Cancelled):
            self.say("Replay stopped.")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Replay stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            follower_cls.send_action = real_send
            follower_cls.get_observation = real_get_obs
            follower_cls.configure = real_configure
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""

    # ------------------------------------------------------------------- stop

    def stop(self) -> dict:
        with self.lock:
            if self.mode == "idle":
                return {"ok": True, "why": "nothing was running"}
            self.say("Stopping.")
            # Also ends a wait on the Next button, if calibration is asking.
            self._cancel.set()
            if self.mode == "record":
                self._throw_away()
            else:
                self._stop.set()
            return {"ok": True}

    # ------------------------------------------------------- start position

    def _start_pose(self) -> dict | None:
        """The saved start position, if it belongs to the robot plugged in now."""
        pose = self.cfg.get("reset_pose")
        if not pose or not self.kind or self.cfg.get("reset_robot") != self.kind.key:
            return None
        return {k: float(v) for k, v in pose.items()}

    def save_start_position(self) -> dict:
        """Remember where the follower is now, to go back to after every try.

        Only while moving, after the opening sync: then what was last sent to
        the follower is where it is. Saved with the kind of robot, because one
        arm's joint values mean nothing to another.
        """
        with self.lock:
            if self.mode != "teleop" or not self._synced:
                return {"ok": False, "why": "press Start moving first, then put the arm where "
                                            "every try should start"}
            pose = dict(self._last_sent)
            if not pose or not self._sane(pose):
                return {"ok": False, "why": "the arm is not reporting a sane position right now"}
            self.cfg["reset_pose"] = pose
            self.cfg["reset_robot"] = self.kind.key
            detect.save_config(self.cfg)
            self.say("Start position saved. After every try the robot goes back here, slowly.")
            return {"ok": True}

    def clear_start_position(self) -> dict:
        with self.lock:
            self.cfg.pop("reset_pose", None)
            self.cfg.pop("reset_robot", None)
            detect.save_config(self.cfg)
            self.say("Start position cleared.")
            return {"ok": True}

    def _record_robot_config(self, cameras: dict):
        """The follower for record(). It keeps holding at the end of a try when it
        is about to go back to a start position, instead of dropping first."""
        cfg = self._follower_config(cameras)
        if self._start_pose() is not None:
            cfg.disable_torque_on_disconnect = False
        return cfg

    def _return_to_start(self, fps: int) -> None:
        """Glide the follower back to the start position, then let it hold there.

        The same slow, eased walk as the opening sync -- that function walks the
        follower to whatever pose it is handed, so it is handed this one.
        """
        pose = self._start_pose()
        if pose is None:
            return

        class _StartPosition:
            def get_action(self):
                return dict(pose)

        follower = None
        self.status["phase"] = "returning"
        try:
            follower = self._make_follower(cameras={})
            follower.config.disable_torque_on_disconnect = False
            follower.connect()
            self._live["follower"] = follower
            self._last_sent = {}
            self.say("Going back to the start position, slowly.")
            if self._first_sync(follower, _StartPosition(), fps, quick_if_matched=True):
                self.say("Robot back at the start position.")
        except Exception as exc:
            self.say(f"Could not go back to the start position: {type(exc).__name__}: {exc}")
        finally:
            self._live.pop("follower", None)
            try:
                if follower is not None:
                    if self._estop.is_set():
                        robots.torque_off(follower)
                    follower.disconnect()
            except Exception:
                pass
        if not (self._estop.is_set() or self._cancel.is_set()):
            self._return_leader_to_start(pose, fps)

    def _soft_hold_follower(self) -> None:
        """After a try, leave an SO follower resting on low torque where it is.

        record() drops it when it lets go, and a dropped arm folds onto the table
        between tries. A NexArm is left as it was (robots.soft_hold says why).
        """
        if self.kind is None or self.kind.raw_counts:
            return
        follower = None
        try:
            follower = self._make_follower(cameras={})
            follower.connect()
            if robots.soft_hold(follower):
                self.say("Follower resting on low power, so it stays up by itself.")
        except Exception as exc:
            self.say(f"Could not rest the follower on low power: {type(exc).__name__}: {exc}")
        finally:
            try:
                if follower is not None:
                    follower.disconnect()
            except Exception:
                pass

    def _return_leader_to_start(self, follower_pose: dict, fps: int) -> None:
        """Walk the leader -- the arm in the child's hand -- back to the start too.

        Its motors come on holding where it is, it glides at the same slow sync
        speed, and it is left holding the start so the next try begins from the
        same place. The next Start lets go of it again.
        """
        arm = None
        try:
            leader = self._make_leader()
            leader.connect()                     # leaders come up with motors off
            arm = robots.LeaderAsArm(self.kind, leader)
            self._live["leader"] = arm
            target = arm.to_leader_units(follower_pose)
            arm.hold_where_it_is()
            self._last_sent = {}

            class _Target:
                def get_action(self):
                    return dict(target)

            self.say("Bringing the arm you hold back to the start as well. Let go of it.")
            if self._first_sync(arm, _Target(), fps, quick_if_matched=True):
                self.say("Both arms are at the start position.")
        except Exception as exc:
            self.say(f"Could not bring the leader back to start: {type(exc).__name__}: {exc}")
        finally:
            self._live.pop("leader", None)
            if arm is not None:
                try:
                    if self._estop.is_set():
                        robots.torque_off(arm)
                    arm.let_go_of_port()
                except Exception:
                    pass

    def _throw_away(self) -> None:
        """End the try, keep it, and mark it deleted in labels.csv.

        Every try is saved and numbered, so the try numbers on the paper sheet,
        in labels.csv and in the dataset always agree -- a deleted try still
        uses up its number. It is left out of training by its label, not erased.
        """
        self._deleted = True
        self.events["exit_early"] = True

    def throw_away_try(self) -> dict:
        """The page's "That went wrong" button: throw this try away, then go back to start."""
        with self.lock:
            if self.mode != "record":
                return {"ok": False, "why": "nothing is recording"}
            self._throw_away()
            return {"ok": True}

    # --------------------------------------------------------- EMERGENCY STOP

    def emergency_stop(self) -> dict:
        """EMERGENCY STOP: every motor off now, everything stopped, the try thrown away.

        Never takes self.lock: whatever is holding it -- a camera search, a Start --
        must not delay this by even a moment. The motors are switched off from a
        thread of its own, at once, through whatever connection is open (a COM
        port can only be open once); the flags stop every worker; and once the
        ports are free both arms are switched off again through them, to confirm.
        """
        self._estop.set()
        self.estop_at = time.strftime("%H:%M:%S")
        self._cancel.set()          # a calibration question, a sync, a return to start
        self._stop.set()            # teleop and replay
        recording = self.mode == "record"
        if recording:
            self._throw_away()      # a try that ended in an emergency is marked deleted
        self.say("EMERGENCY STOP. Switching every motor off. Everything stops"
                 + (", and this try is marked deleted." if recording else "."))
        self._estop_busy = True
        threading.Thread(target=self._estop_release, name="estop", daemon=True).start()
        return {"ok": True}

    def _estop_release(self) -> None:
        """Switch every motor off now, then again through the ports to confirm.

        The off command used to wait for the running worker's next move, and a
        recording never made one: the same button also ends lerobot's loop, which
        checks for that before moving again. The arm stayed powered until lerobot
        had closed everything -- about 6 s. Now it goes out from here, through the
        connection that is open (a NexArm's bus takes commands from two threads
        one at a time), three times in a tenth of a second so a move already on
        its way cannot undo it. With nothing connected -- between a sync and the
        recording connecting, say -- the ports are free, so it goes through them.

        Then, once the worker has let go of the ports, both arms are switched off
        through them as well. That also catches the leader -- a Koch leader holds
        its gripper -- and an arm left holding a pose while nothing was running.
        """
        try:
            kind = self.kind or robots.NEXARM
            cut = False
            # Three times each, a NexArm move on its way cannot undo it; and on
            # a LeRobot bus, which has no lock, until one gets through between
            # the worker's own commands (a second at most).
            live = list(self._live.values())
            sent = {id(d): 0 for d in live}
            done_ok = set()
            deadline = time.monotonic() + 1.0
            while live and time.monotonic() < deadline:
                for dev in live:
                    if robots.torque_off(dev):
                        done_ok.add(id(dev))
                    sent[id(dev)] += 1
                if len(done_ok) == len(live) and min(sent.values()) >= 3:
                    break
                time.sleep(0.03)
            cut = bool(done_ok)
            if not cut:
                for role in ("follower", "leader"):
                    port = self.cfg.get(f"{role}_port")
                    if port and robots.release(kind, port, role):
                        cut = True
            if cut:
                self.say("Motors off: the arms are floppy.")
            deadline = time.monotonic() + 10
            while self.mode != "idle" and time.monotonic() < deadline:
                for dev in list(self._live.values()):    # anything that connected meanwhile
                    robots.torque_off(dev)
                time.sleep(0.05)
            done = []
            for role in ("follower", "leader"):
                port = self.cfg.get(f"{role}_port")
                if port and robots.release(kind, port, role):
                    done.append(port)
            if done:
                self.say("Emergency stop: motors confirmed off on " + ", ".join(done) + ".")
        except Exception as exc:
            self.say(f"Emergency stop backstop failed: {exc}. Use the power switch.")
        finally:
            self._estop_busy = False

    def release_motors(self) -> dict:
        """Switch the motors off so a stuck arm can be moved by hand.

        Needed more often than it sounds: an arm driven into its own end stop
        stays there, held, and no amount of pulling helps until the motors let go.
        Only allowed from idle, because the ports have to be free to say it.
        """
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": "press Stop first"}
            kind = self.kind or robots.NEXARM
            roles = [(self.cfg.get(f"{r}_port"), r) for r in ("leader", "follower")]
            roles = [(p, r) for p, r in roles if p]
            if not roles:
                # Nothing set up yet: the only arms that can be released blind
                # are NexArms, which is what this did before there was a choice.
                kind = robots.NEXARM
                roles = [(p, "follower") for p in detect.candidate_ports()]
            self.close_cameras()
            done = [p for p, r in roles if robots.release(kind, p, r)]
            for p in done:
                self.say(f"{p}: motors off, that arm moves by hand now")
            if not done:
                self.say("Could not switch the motors off. Use the power switch.")
            self.open_cameras()
            return {"ok": bool(done), "released": done,
                    "why": "" if done else "no port would open"}

    def _usable_for_page(self) -> tuple[bool, str]:
        """detect.config_is_usable for the page: never waited on after the first time.

        It lists every COM port, which can take seconds while a USB cable acts up.
        The page asks over and over, and those waits used to pile up until the
        buttons -- EMERGENCY STOP included -- stopped answering. The answer is now
        refreshed in the background, at most every 2 s.
        """
        if self._usable is None:
            self._usable, self._usable_at = detect.config_is_usable(self.cfg), time.monotonic()
        elif time.monotonic() - self._usable_at > 2.0 and not self._usable_busy:
            self._usable_busy = True

            def run():
                try:
                    self._usable = detect.config_is_usable(self.cfg)
                finally:
                    self._usable_at = time.monotonic()
                    self._usable_busy = False

            threading.Thread(target=run, name="ports", daemon=True).start()
        return self._usable

    def snapshot(self) -> dict:
        ok, why = self._usable_for_page()
        kind = self.kind
        return {
            "mode": self.mode,
            "setup_stage": self.setup_stage,
            "config_ok": ok,
            "config_why": why,
            "config": self.cfg,
            "robot": {"key": kind.key, "label": kind.label} if kind else None,
            "robots": robots.choices(),
            "prompt": self.prompt,
            "start_saved": self._start_pose() is not None,
            "estop_at": self.estop_at,
            "estop_busy": self._estop_busy,
            "stream_port": self.stream_port,
            "cameras": self.camera_names(),
            "camera_choices": self.camera_choices(),
            "camera_fps": dict(self.camera_fps),
            "status": self.status,
            "labels": self.labels,
            "recent_tries": self.recent_labels(),
            "swap_every": SWAP_EVERY,
            "error": self.error,
            "datasets_dir": str(self.datasets_dir()),
            "datasets": self.list_datasets(),
            "log": list(self.log_lines)[-40:],
        }
