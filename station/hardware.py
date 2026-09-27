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
"""

from __future__ import annotations

import json
import threading
import time
import traceback
from collections import deque
from pathlib import Path

from . import detect

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
SYNC_MIN_SECONDS = 2.0
SYNC_MAX_SECONDS = 12.0

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


class Station:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = "idle"            # idle | teleop | record
        self.cfg: dict = {}
        self.log_lines: deque[str] = deque(maxlen=400)

        self._caps: dict[str, object] = {}     # camera name -> cv2.VideoCapture
        self._frames: dict[str, bytes] = {}    # camera name -> latest JPEG
        self._frame_lock = threading.Lock()

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Teleop / record status surfaced to the browser.
        self.status: dict = {"fps": 0.0, "episode": 0, "target_episodes": 0, "phase": ""}
        self.error: str | None = None

        # Set-up wizard state.
        self.arm_session: detect.ArmSession | None = None
        self.setup_stage = "unknown"   # unknown | need_arms | dragging | ready
        self.drag_deadline: float = 0.0
        self.cameras_found: list[int] = []

        # Recording controls, read by lerobot's record loop through the patch.
        self.events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

        # Last target sent to each joint, for the per-tick travel limit.
        self._last_sent: dict[str, float] = {}

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
        cfg = detect.load_config()
        ok, why = detect.config_is_usable(cfg)
        if ok:
            self.cfg = cfg
            self.setup_stage = "ready"
        else:
            self.cfg = cfg or {}
            self.setup_stage = "need_arms"
        self.say(why)
        return ok, why

    # ------------------------------------------------------------ set-up flow

    def begin_arm_detect(self) -> dict:
        """Open every candidate port, then start the drag window."""
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": "stop what is running first"}
            self.close_cameras()
            if self.arm_session:
                self.arm_session.close()
            self.arm_session = detect.ArmSession()
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
            self.say("Drag test: wave the arm you hold in your hand.")
            return {"ok": True, "ports": ports, "seconds": 15}

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

            self.cfg.update({
                "robot_type": "nexarm",
                "leader_port": leader,
                "follower_port": follower,
                "baudrate": detect.BAUD,
                "fps": self.cfg.get("fps", 30),
            })
            self.say(f"leader {leader}, follower {follower}")

            self.say("Looking for cameras.")
            self.cameras_found = detect.probe_cameras(log=self.say)
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
            cams["front"], cams["wrist"] = cams["wrist"], cams["front"]
            detect.save_config(self.cfg)
            self.say(f"Swapped: front={cams['front']} wrist={cams['wrist']}")
            if self.mode == "idle":
                self.close_cameras()
                self.open_cameras()
            return {"ok": True, "cameras": cams}

    # ---------------------------------------------------------------- cameras

    def open_cameras(self, rescanned: bool = False) -> None:
        """Station-owned capture, used for the picture when nothing else is running.

        The backend comes from detect.open_capture rather than being named here:
        DirectShow is correct on Windows and makes every index fail to open on
        macOS, which is indistinguishable from having no cameras plugged in.
        """
        import cv2

        with self.lock:
            self.close_cameras()
            failed = []
            for name, idx in self.cfg.get("cameras", {}).items():
                cap = detect.open_capture(idx)
                if not cap.isOpened():
                    failed.append((name, idx))
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self._caps[name] = cap
                else:
                    cap.release()
                    self.say(f"camera {idx} ({name}) would not open")

            # A configured camera that will not open means the setup has changed
            # underneath us -- unplugged, or renumbered by being moved to another
            # socket. Look again and save what is actually there, rather than
            # showing a black rectangle for the rest of the session. Recording
            # reads the same config, so a stale entry here would make lerobot fail
            # on connect too.
            if failed and not rescanned:
                self.say("A camera is missing. Looking again.")
                found = detect.probe_cameras(log=self.say)
                fresh = detect.guess_cameras(found)
                if fresh != self.cfg.get("cameras"):
                    self.cfg["cameras"] = fresh
                    detect.save_config(self.cfg)
                    self.say(f"Cameras are now: {fresh or 'none'}")
                    self.open_cameras(rescanned=True)
                    return

            if self._caps and not (self._thread and self._thread.is_alive()):
                self._start_pump()

    def close_cameras(self) -> None:
        with self.lock:
            self._stop.set()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
            self._thread = None
            for cap in self._caps.values():
                try:
                    cap.release()
                except Exception:
                    pass
            self._caps.clear()
            self._stop.clear()

    def _start_pump(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._pump, name="cam-pump", daemon=True)
        self._thread.start()

    def _pump(self) -> None:
        """Read the station-owned cameras into JPEGs for the browser."""
        import cv2

        while not self._stop.is_set():
            got_any = False
            for name, cap in list(self._caps.items()):
                ok, frame = cap.read()
                if ok and frame is not None:
                    self._publish_bgr(name, frame)
                    got_any = True
            if not got_any:
                time.sleep(0.05)
            else:
                time.sleep(1 / 25)

    def _publish_bgr(self, name: str, bgr) -> None:
        import cv2

        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            with self._frame_lock:
                self._frames[name] = buf.tobytes()

    def publish_rgb(self, name: str, rgb) -> None:
        """Frames arriving from lerobot's own camera reads are RGB, not BGR."""
        import cv2

        try:
            self._publish_bgr(name, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        except Exception:
            pass

    def latest_jpeg(self, name: str) -> bytes | None:
        with self._frame_lock:
            return self._frames.get(name)

    def camera_names(self) -> list[str]:
        return list(self.cfg.get("cameras", {}).keys())

    # ----------------------------------------------------------------- teleop

    def start_teleop(self) -> dict:
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": f"already {self.mode}"}
            ok, why = detect.config_is_usable(self.cfg)
            if not ok:
                return {"ok": False, "why": why}
            self.error = None
            self.mode = "teleop"
            self._last_sent = {}
            self._stop.clear()
            threading.Thread(target=self._teleop_worker, name="teleop", daemon=True).start()
            return {"ok": True}

    def _teleop_worker(self) -> None:
        from lerobot.robots.nexarm_follower import NexArmFollower, NexArmFollowerConfig
        from lerobot.teleoperators.nexarm_leader import NexArmLeader, NexArmLeaderConfig
        from lerobot.utils.robot_utils import precise_sleep

        fps = int(self.cfg.get("fps", 30))
        follower = leader = None
        real_configure = self._park_goal_before_torque(NexArmFollower)
        try:
            # Cameras stay with the station here, so the robot is built with none.
            follower = NexArmFollower(NexArmFollowerConfig(
                port=self.cfg["follower_port"], cameras={},
                motion_acc=MOTION_ACC, motion_speed=MOTION_SPEED))
            leader = NexArmLeader(NexArmLeaderConfig(port=self.cfg["leader_port"]))
            follower.connect()
            leader.connect()
            self.say("Arms connected. Easing the follower over to match the leader.")
            if not self._first_sync(follower, leader, fps):
                self.say("Not moving: the arms are not reporting sane positions yet. "
                         "Switch both off and on, then try again.")
                return
            self.say("Teleop running. Move the arm in your hand.")

            ticks, t_window = 0, time.perf_counter()
            while not self._stop.is_set():
                start = time.perf_counter()
                action = leader.get_action()
                if reading_is_sane(action.values()):
                    follower.send_action(self._rate_limit(action))
                ticks += 1
                if start - t_window >= 1.0:
                    self.status["fps"] = round(ticks / (start - t_window), 1)
                    ticks, t_window = 0, start
                precise_sleep(1.0 / fps - (time.perf_counter() - start))
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Teleop stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            NexArmFollower.configure = real_configure
            for dev in (follower, leader):
                try:
                    if dev is not None:
                        dev.disconnect()
                except Exception:
                    pass
            with self.lock:
                self.mode = "idle"
                self.status["fps"] = 0.0
            self.say("Teleop stopped.")

    @staticmethod
    def _park_goal_before_torque(follower_cls):
        """Stop the follower snapping when its motors switch on.

        ``configure()`` calls ``set_torque(True)``, and the servos come up aiming
        at whatever Goal_Position was last written to them -- which, after a
        previous session, is a pose from minutes ago. The arm lunges there the
        instant torque arrives, before any of our code has sent an action. From
        the outside this looks like the opening sync jerking, because it happens
        one moment before the sync starts.

        The fix is to write where the arm actually IS as the goal, and only then
        allow the torque on. Then switching the motors on holds it still.

        Returns the original configure so the caller can put it back.
        """
        real_configure = follower_cls.configure

        def parked_configure(self_robot):
            try:
                current = self_robot.bus.read_positions()
                if reading_is_sane(current):
                    self_robot.bus.write_positions(list(current))
                    time.sleep(0.05)
            except Exception:
                # Worst case we are no worse off than the stock behaviour.
                pass
            return real_configure(self_robot)

        follower_cls.configure = parked_configure
        return real_configure

    def _rate_limit(self, action: dict, max_step: float = MAX_STEP_PER_TICK) -> dict:
        """Never let a target be more than MAX_STEP_PER_TICK from the last one.

        Without this, one bad leader reading is a lunge. With it, the worst a bad
        reading can do is start a slow drift that Stop or the next good reading
        ends. The cost is that a genuinely fast hand movement is followed slightly
        behind, which for demonstrating a task to a robot is a fair trade.
        """
        limited = {}
        for k, raw in action.items():
            want = float(raw)
            prev = self._last_sent.get(k)
            if prev is None:
                limited[k] = want
            else:
                step = max(-max_step, min(max_step, want - prev))
                limited[k] = prev + step
        self._last_sent = limited
        return limited

    def _first_sync(self, follower, leader, fps: int) -> bool:
        """Walk the follower to the leader's pose slowly before live teleop starts.

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
        if not reading_is_sane(start.values()):
            self.say(f"Follower reading looks wrong: {[round(v) for v in start.values()]}")
            return False
        if not reading_is_sane([target[k] for k in start]):
            self.say(f"Leader reading looks wrong: {[round(target[k]) for k in start]}")
            return False

        gap = max(abs(target[k] - start[k]) for k in start)

        # Time the sync to the distance rather than using a fixed number, and
        # cap every tick as well. Either alone is not enough: a fixed duration
        # makes a large gap fast, and an eased curve still peaks in the middle.
        seconds = gap / SYNC_COUNTS_PER_SECOND
        seconds = max(SYNC_MIN_SECONDS, min(SYNC_MAX_SECONDS, seconds))
        self.say(f"Closing a {gap:.0f} count gap gently, over about {seconds:.0f} seconds.")

        self._last_sent = dict(start)
        steps = max(1, int(seconds * fps))
        for i in range(1, steps + 1):
            if self._stop.is_set():
                return False
            f = i / steps
            e = f * f * (3.0 - 2.0 * f)      # ease in and out, no jerk at either end
            t0 = time.perf_counter()
            follower.send_action(self._rate_limit(
                {k: start[k] + (target[k] - start[k]) * e for k in start},
                max_step=SYNC_STEP_PER_TICK))
            precise_sleep(1.0 / fps - (time.perf_counter() - t0))

        # Hand the live loop a clean slate: the sync ended wherever the ramp
        # reached, and the loop's own limit starts from there.
        self._last_sent = dict(self._last_sent)
        self.say("Synced.")
        return True

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
            ok, why = detect.config_is_usable(self.cfg)
            if not ok:
                return {"ok": False, "why": why}
            if not task.strip():
                return {"ok": False, "why": "give the job a name first"}
            self.error = None
            self.mode = "record"
            self.events.update(exit_early=False, rerecord_episode=False, stop_recording=False)
            self.status.update(episode=0, target_episodes=1, phase="starting")
            # lerobot-record opens the cameras itself, so let go of them first.
            self.close_cameras()
            threading.Thread(
                target=self._record_worker, args=(task.strip(),),
                name="record", daemon=True,
            ).start()
            return {"ok": True}

    def _record_worker(self, task: str) -> None:
        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.configs.dataset import DatasetRecordConfig
        from lerobot.robots.nexarm_follower import NexArmFollower, NexArmFollowerConfig
        from lerobot.scripts import lerobot_record as lr
        from lerobot.teleoperators.nexarm_leader import NexArmLeaderConfig

        fps = int(self.cfg.get("fps", 30))
        slug = "".join(c if c.isalnum() else "_" for c in task.lower()).strip("_")[:40] or "task"

        cams = {
            name: OpenCVCameraConfig(index_or_path=int(idx), width=640, height=480, fps=fps)
            for name, idx in self.cfg.get("cameras", {}).items()
        }

        # Three patches, all undone in the finally block.
        real_listener = lr.init_keyboard_listener
        real_get_obs = NexArmFollower.get_observation
        real_send = NexArmFollower.send_action
        station = self
        self._last_sent = {}

        def browser_listener():
            """The browser's buttons stand in for the keyboard."""
            return None, station.events

        def teeing_get_obs(self_robot):
            obs = real_get_obs(self_robot)
            for cam_key in self_robot.cameras:
                frame = obs.get(cam_key)
                if frame is not None:
                    station.publish_rgb(cam_key, frame)
            return obs

        def limited_send(self_robot, action):
            """Same travel limit as teleop.

            record_loop calls send_action itself, so the limit has to live on the
            robot rather than in the caller, or recording would be the one mode
            that can still lunge.
            """
            if not reading_is_sane(action.values()):
                return dict(station._last_sent) or action
            return real_send(self_robot, station._rate_limit(action))

        real_configure = self._park_goal_before_torque(NexArmFollower)
        try:
            lr.init_keyboard_listener = browser_listener
            NexArmFollower.get_observation = teeing_get_obs
            NexArmFollower.send_action = limited_send

            out_dir = self.datasets_dir()
            out_dir.mkdir(parents=True, exist_ok=True)
            root = out_dir / f"nexarm_{slug}"

            # Already recorded this job before? Add to it instead of colliding
            # with it. LeRobotDataset.create refuses to write over an existing
            # dataset, and a child pressing the same button twice should get a
            # second try, not an error.
            resume = (root / "meta" / "info.json").is_file()

            cfg = lr.RecordConfig(
                robot=NexArmFollowerConfig(
                    port=self.cfg["follower_port"], cameras=cams,
                    motion_acc=MOTION_ACC, motion_speed=MOTION_SPEED),
                teleop=NexArmLeaderConfig(port=self.cfg["leader_port"]),
                dataset=DatasetRecordConfig(
                    repo_id=f"local_user/nexarm_{slug}",
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
                    # The dataset is video of the room, and the room has children in
                    # it. lerobot's own default here is True, which would publish it
                    # to a public Hugging Face dataset the moment recording finished.
                    push_to_hub=False,
                ),
                resume=resume,
                display_data=False,     # frames go to the browser, not to rerun
                play_sounds=False,
            )
            self.status["phase"] = "recording"
            self.say(f'Recording "{task}". Take as long as you need, then press Done.')
            self.say(f"{'Adding to' if resume else 'Saving to'} {root} on this computer only.")
            self._watch_episodes()
            lr.record(cfg)
            self.say("Try saved.")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Recording stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            lr.init_keyboard_listener = real_listener
            NexArmFollower.get_observation = real_get_obs
            NexArmFollower.send_action = real_send
            NexArmFollower.configure = real_configure
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""
            self.open_cameras()

    def _watch_episodes(self) -> None:
        """Count saved episodes off disk so the page has a progress number.

        record() does not expose its counter, and reaching into its locals would
        break on the next lerobot bump. The episode directory is a stable thing
        to count instead.
        """
        def run():
            root = Path(self.status.get("root", "")) if self.status.get("root") else None
            while self.mode == "record":
                try:
                    if root is None:
                        root = next(self.datasets_dir().glob("nexarm_*"), None)
                    if root is not None:
                        n = len(list((root / "data").rglob("*.parquet")))
                        self.status["episode"] = n
                except Exception:
                    pass
                time.sleep(1.0)
        threading.Thread(target=run, name="ep-watch", daemon=True).start()

    # ----------------------------------------------------------------- replay

    def start_replay(self, name: str, episode: int) -> dict:
        """Play a recorded episode back on the arm, with nobody holding the leader."""
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": f"already {self.mode}"}
            ok, why = detect.config_is_usable(self.cfg)
            if not ok:
                return {"ok": False, "why": why}
            root = self.datasets_dir() / name
            if not (root / "meta" / "info.json").is_file():
                return {"ok": False, "why": f"no recording called {name}"}
            self.error = None
            self.mode = "replay"
            self.status.update(phase=f"replaying {name} episode {episode}")
            threading.Thread(target=self._replay_worker, args=(name, episode),
                             name="replay", daemon=True).start()
            return {"ok": True}

    def _replay_worker(self, name: str, episode: int) -> None:
        from lerobot.robots.nexarm_follower import NexArmFollower, NexArmFollowerConfig
        from lerobot.scripts import lerobot_replay as lrp

        real_send = NexArmFollower.send_action
        station = self
        self._last_sent = {}
        root = self.datasets_dir() / name

        def limited_send(self_robot, action):
            # Same jump guard as teleop and recording. A dataset can contain a
            # railed frame just as a live reading can, and on replay there is no
            # hand on the leader to notice it going wrong.
            #
            # Stop is checked here because lerobot's replay() is one blocking
            # call with no cancel. This is the only place the loop passes through
            # often enough to abort it promptly.
            if station._stop.is_set():
                raise KeyboardInterrupt("stopped from the page")
            if not reading_is_sane(action.values()):
                return dict(station._last_sent) or action
            return real_send(self_robot, station._rate_limit(action))

        real_configure = self._park_goal_before_torque(NexArmFollower)
        try:
            self._stop.clear()
            NexArmFollower.send_action = limited_send
            self.say(f"Replaying {name}, episode {episode}. Keep hands clear.")
            cfg = lrp.ReplayConfig(
                robot=NexArmFollowerConfig(
                    port=self.cfg["follower_port"], cameras={},
                    motion_acc=MOTION_ACC, motion_speed=MOTION_SPEED),
                dataset=lrp.DatasetReplayConfig(
                    repo_id=f"local_user/{name}", root=str(root),
                    episode=int(episode), fps=int(self.cfg.get("fps", 30)),
                ),
                play_sounds=False,
            )
            lrp.replay(cfg)
            self.say("Replay finished.")
        except KeyboardInterrupt:
            self.say("Replay stopped.")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.say(f"Replay stopped with an error: {self.error}")
            traceback.print_exc()
        finally:
            NexArmFollower.send_action = real_send
            NexArmFollower.configure = real_configure
            with self.lock:
                self.mode = "idle"
                self.status["phase"] = ""

    # ------------------------------------------------------------------- stop

    def stop(self) -> dict:
        with self.lock:
            if self.mode == "idle":
                return {"ok": True, "why": "nothing was running"}
            self.say("Stopping.")
            if self.mode == "record":
                self.events["stop_recording"] = True
                self.events["exit_early"] = True
            else:
                self._stop.set()
            return {"ok": True}

    def release_motors(self) -> dict:
        """Switch the motors off so a stuck arm can be moved by hand.

        Needed more often than it sounds: an arm driven into its own end stop
        stays there, held, and no amount of pulling helps until the motors let go.
        Only allowed from idle, because the ports have to be free to say it.
        """
        with self.lock:
            if self.mode != "idle":
                return {"ok": False, "why": "press Stop first"}
            from station import release as rel

            ports = [p for p in (self.cfg.get("leader_port"), self.cfg.get("follower_port")) if p]
            if not ports:
                ports = detect.candidate_ports()
            self.close_cameras()
            done = [p for p in ports if rel.release(p)]
            for p in done:
                self.say(f"{p}: motors off, that arm moves by hand now")
            if not done:
                self.say("Could not switch the motors off. Use the power switch.")
            self.open_cameras()
            return {"ok": bool(done), "released": done,
                    "why": "" if done else "no port would open"}

    def snapshot(self) -> dict:
        ok, why = detect.config_is_usable(self.cfg)
        return {
            "mode": self.mode,
            "setup_stage": self.setup_stage,
            "config_ok": ok,
            "config_why": why,
            "config": self.cfg,
            "cameras": self.camera_names(),
            "status": self.status,
            "error": self.error,
            "datasets_dir": str(self.datasets_dir()),
            "datasets": self.list_datasets(),
            "log": list(self.log_lines)[-40:],
        }
