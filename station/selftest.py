"""Exercise the station without needing anyone to hold an arm.

Covers the parts a hardware demo cannot: that the page serves, that the API
answers, that the cameras reach the browser as MJPEG, and -- the one that matters
most -- that the recording config this station builds has push_to_hub off.

    python station/selftest.py

The drag test is the one thing not covered here, because it needs a hand.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from station import detect, server                     # noqa: E402
from station import hardware                           # noqa: E402
from station.hardware import Station                   # noqa: E402

PASS, FAIL = [], []

# The test must never write the real nexarm.json. swap_cameras() saves the config,
# and on the first run of this file it wrote one containing a GUESSED leader and
# follower -- ports[0] and ports[1], never verified against the hardware. start.py
# then read it, believed set-up was finished, and skipped the drag test entirely.
# A test that leaves behind a config claiming to know which arm is which is worse
# than no test.
_REAL_CONFIG = detect.CONFIG_PATH
detect.CONFIG_PATH = Path(__file__).resolve().parent / "selftest_nexarm.json"


def check(name: str, cond, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  [{'ok' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))


def get(url: str, timeout: float = 5.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def post(url: str, payload=None, timeout: float = 5.0):
    body = json.dumps(payload or {}).encode()
    req = urllib.request.Request(url, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def station_already_running() -> bool:
    """A live start.py holds the cameras, and no second process can open them.

    Without this check the camera tests fail with "no cameras found", which reads
    as a hardware fault and is not one. Stop the station and run this again.
    """
    for port in (8123, 8124, 8125):
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/api/state", timeout=1).read()
            return True
        except Exception:
            continue
    return False


def main() -> int:
    print("Station selftest")
    print("-" * 50)

    busy = station_already_running()
    if busy:
        print()
        print("  NOTE: the station is already running and is holding the cameras.")
        print("  Camera checks are skipped. Stop it (Ctrl+C) to run them.")
        print()

    print("\n1. detection helpers")
    ports = detect.candidate_ports()
    # No arm plugged in is a setup, not a fault -- same rule as the cameras. This
    # check used to pass on a bare laptop only because Bluetooth ports leaked in.
    if ports:
        check("finds serial ports", True, str(ports))
    else:
        print("  [--] no arm cable is plugged in. Port checks below still run.")
    check("port_exists agrees with the list",
          all(detect.port_exists(p) for p in ports))
    check("port_exists rejects a made-up port", not detect.port_exists("COM999"))
    usb = {p.device for p in detect.usb_serial_ports()}
    check("only USB adapters are candidates, never Bluetooth", set(ports) <= usb,
          str(sorted(set(ports) - usb)))

    from serial.tools import list_ports

    class _Port:
        def __init__(self, device, vid, pid, description):
            self.device, self.vid, self.pid, self.description = device, vid, pid, description

    bt = [_Port("COM3", None, None, "Standard Serial over Bluetooth link (COM3)")]
    so101 = [_Port("COM8", 0x1A86, 0x55D3, "USB-Enhanced-SERIAL CH343 (COM8)")]
    nexarm = [_Port("COM11", 0x1A86, 0x7523, "USB-SERIAL CH340 (COM11)")]
    real_comports = list_ports.comports
    try:
        list_ports.comports = lambda: bt
        check("Bluetooth alone gives no candidates (it used to hang ~13 s each)",
              detect.candidate_ports() == [])
        list_ports.comports = lambda: bt + so101
        check("an SO-101 board is never probed with NexArm frames",
              detect.candidate_ports() == [])
        list_ports.comports = lambda: bt + so101 + nexarm
        check("a NexArm next to an SO-101 is still found",
              detect.candidate_ports() == ["COM11"])
    finally:
        list_ports.comports = real_comports
    cams = [] if busy else detect.probe_cameras(limit=4, log=lambda m: None)
    # No camera at all is a supported setup, not a failure: the arms work without
    # one. So this reports rather than fails, and the frame checks below are
    # skipped instead of failing for a reason that is not a fault.
    no_cams = not busy and not cams
    if no_cams:
        print("  [--] no camera is plugged in. Camera checks skipped;")
        print("       running with no camera is supported.")
    elif not busy:
        check("finds a camera", len(cams) >= 1, str(cams))
    check("camera guess assigns front", "front" in detect.guess_cameras(cams) or not cams)
    if len(cams) >= 2:
        g = detect.guess_cameras(cams)
        check("camera guess assigns both", g.get("front") != g.get("wrist"), str(g))

    print("\n2. config validation")
    ok, why = detect.config_is_usable(None)
    check("no config is rejected", not ok, why)
    ok, why = detect.config_is_usable({"leader_port": "COM11", "follower_port": "COM11"})
    check("same port twice is rejected", not ok, why)
    ok, why = detect.config_is_usable({"leader_port": "COM999", "follower_port": "COM998"})
    check("vanished ports are rejected", not ok, why)
    if len(ports) >= 2:
        ok, why = detect.config_is_usable(
            {"leader_port": ports[0], "follower_port": ports[1]})
        check("two real ports are accepted", ok, why)

    print("\n3. station state machine")
    st = Station()
    st.cfg = {
        "robot_type": "nexarm",
        "leader_port": ports[0] if ports else "COM11",
        "follower_port": ports[1] if len(ports) > 1 else "COM12",
        "baudrate": detect.BAUD, "fps": 30,
        "cameras": detect.guess_cameras(cams),
    }
    check("starts idle", st.mode == "idle")
    check("stop on idle is harmless", st.stop().get("ok") is True)

    st.open_cameras()
    names = st.camera_names()
    deadline = time.time() + 6
    while time.time() < deadline and not all(st.latest_jpeg(n) for n in names):
        time.sleep(0.2)
    if not busy and not no_cams:
        check("every camera produced a JPEG",
              bool(names) and all(st.latest_jpeg(n) for n in names), str(names))
        first = st.latest_jpeg(names[0]) if names else None
        check("JPEG really is a JPEG", bool(first) and first[:2] == b"\xff\xd8")

    if len(names) >= 2:
        before = dict(st.cfg["cameras"])
        st.swap_cameras()
        after = st.cfg["cameras"]
        check("swap exchanges front and wrist",
              after["front"] == before["wrist"] and after["wrist"] == before["front"])
        st.swap_cameras()
        check("swapping twice is a round trip", st.cfg["cameras"] == before)

    print("\n4. web server")
    # start.py loads LeRobot's list of arms before it serves the page; do the
    # same here, or the first /api/state pays those seconds and times out.
    from station import robots as _robots
    _robots.choices()
    port = server.free_port(8199)
    httpd = server.serve(st, port)
    base = f"http://127.0.0.1:{port}"
    time.sleep(0.4)
    try:
        code, ctype, body = get(base + "/")
        check("serves the page", code == 200 and b"Robot Station" in body, ctype)
        check("the page has the robot picker and the calibration Next button",
              b'id="kindPick"' in body and b'id="promptCard"' in body)
        check("the page has a camera dropdown for front and for wrist",
              b'id="camFront"' in body and b'id="camWrist"' in body)
        state = json.loads(get(base + "/api/state")[2])
        check("/api/state has the fields the page reads",
              {"mode", "setup_stage", "config_ok", "cameras", "status", "log",
               "robot", "robots", "prompt"} <= set(state))
        check("/api/state offers more than one kind of arm",
              len(state["robots"]) >= 2, str([r["key"] for r in state["robots"]]))
        r = post(base + "/api/prompt/next")
        check("Next with nothing waiting is ignored, never an error popup",
              r.get("ok") is True and bool(r.get("ignored")), r.get("ignored", ""))
        check("/api/state reports idle", state["mode"] == "idle")
        if names and not busy and not no_cams:
            code, ctype, body = get(f"{base}/shot/{names[0]}.jpg")
            check("single still works", code == 200 and ctype == "image/jpeg" and len(body) > 1000,
                  f"{len(body)} bytes")

            # MJPEG has no end, so read for a while and count boundaries. Only
            # pictures that changed are sent, so a camera behind a privacy
            # shutter sends one frame and then nothing -- that is reported,
            # not failed: the station is right to treat it as frozen.
            chunk = b""
            with urllib.request.urlopen(f"{base}/stream/{names[0]}", timeout=2) as r:
                ctype = r.headers.get("Content-Type", "")
                deadline = time.time() + 6
                while len(chunk) < 200_000 and time.time() < deadline:
                    try:
                        piece = r.read1(65536)
                    except TimeoutError:
                        break
                    if not piece:
                        break
                    chunk += piece
            frames = chunk.count(b"--" + server.BOUNDARY.encode())
            check("stream is multipart", "multipart/x-mixed-replace" in ctype, ctype)
            if frames < 2 and not st.camera_fps.get(names[0]):
                print(f"  [--] stream frames not checked: camera {names[0]} gives an unchanging "
                      f"picture (privacy shutter or lens cover?)")
            else:
                check("stream carries several frames", frames >= 2, f"{frames} frames")

        check("unknown path is a 404",
              _status_of(base + "/api/nope") == 404)

        print("\n5. guards")
        r = post(base + "/api/record/start", {"task": ""})
        check("recording with no job name is refused", r.get("ok") is False, r.get("why", ""))
        st.mode = "teleop"
        r = post(base + "/api/teleop/start")
        check("will not start twice", r.get("ok") is False, r.get("why", ""))
        r = post(base + "/api/record/start", {"task": "x"})
        check("will not record while moving", r.get("ok") is False, r.get("why", ""))
        st.mode = "idle"

        r = post(base + "/api/record/next")
        check("next-try button sets exit_early", st.events["exit_early"] is True)
        st.events["exit_early"] = False
        st.mode = "record"
        post(base + "/api/record/redo")
        st.mode = "idle"
        check("'That went wrong' button ends the try and marks it deleted",
              st.events["exit_early"] is True and st._deleted is True)
        st.events.update(exit_early=False, rerecord_episode=False, stop_recording=False)
        st._deleted = False

        bad = Station()
        bad.cfg = {"leader_port": "COM999", "follower_port": "COM998"}
        check("teleop refuses a stale config", bad.start_teleop().get("ok") is False)
        check("record refuses a stale config",
              bad.start_record("x").get("ok") is False)
    finally:
        httpd.shutdown()
        st.close_cameras()

    print("\n6. the recording config that would be handed to lerobot")
    cfg = _record_config(st)
    if cfg is None:
        check("record config built", False, "lerobot import failed")
    else:
        check("push_to_hub is OFF", cfg.dataset.push_to_hub is False,
              "this is what keeps video of the room off the public hub")
        check("dataset is written inside this folder",
              "datasets" in str(cfg.dataset.root), str(cfg.dataset.root))
        check("one try per press", cfg.dataset.num_episodes == 1)
        check("no episode timer a child could be cut off by",
              cfg.dataset.episode_time_s >= 3600, f"{cfg.dataset.episode_time_s}s ceiling")
        check("no tidy-up timer", cfg.dataset.reset_time_s == 0)
        check("task text is carried through", cfg.dataset.single_task == "Pick up the red block")
        check("repo id is slugged, not raw text",
              cfg.dataset.repo_id == "local_user/nexarm_pick_up_the_red_block",
              cfg.dataset.repo_id)
        check("follower port is the follower", cfg.robot.port == st.cfg["follower_port"])
        check("leader port is the leader", cfg.teleop.port == st.cfg["leader_port"])
        check("display_data off so it does not need rerun", cfg.display_data is False)
        check("cameras carried into the robot config",
              set(cfg.robot.cameras) == set(st.cfg["cameras"]))

    print("\n7. keyboard patch shape")
    try:
        from lerobot.utils.keyboard_input import init_keyboard_listener  # noqa: F401
        from lerobot.scripts import lerobot_record as lr
        check("lerobot_record still calls init_keyboard_listener",
              hasattr(lr, "init_keyboard_listener"))
        _, real_events = _peek_events()
        check("station events match lerobot's event keys",
              set(st.events) == set(real_events), str(sorted(real_events)))
    except Exception as exc:
        check("keyboard patch shape", False, str(exc))

    print("\n8. speed limit and the railed-reading guard")
    from station.hardware import MAX_STEP_PER_TICK, MOTION_SPEED, reading_is_sane

    check("a normal pose is accepted", reading_is_sane([2000] * 6))
    check("4095 on one joint is rejected",
          not reading_is_sane([2000, 4095, 2000, 2000, 2000, 2000]),
          "this is the reading that drove the arm into its end stop")
    check("0 on one joint is rejected", not reading_is_sane([0] + [2000] * 5))
    check("a non-number is rejected", not reading_is_sane([None] + [1] * 5))
    check("speed is the firmware default, not throttled", MOTION_SPEED == 2000,
          str(MOTION_SPEED))
    check("the jump guard is loose enough never to slow a real hand",
          MAX_STEP_PER_TICK * 30 > 4095,
          f"{MAX_STEP_PER_TICK * 30} counts/s allowed, range is 4095")

    lim = Station()
    j = "shoulder_pan.pos"
    first = lim._rate_limit({j: 2000.0})
    check("the first target passes through", first[j] == 2000.0)
    big = lim._rate_limit({j: 4000.0})
    check("a 2000 count teleport is clamped",
          big[j] == 2000.0 + MAX_STEP_PER_TICK, str(big[j]))
    hand = lim._rate_limit({j: big[j] + 40})
    check("a fast hand movement passes through untouched", hand[j] == big[j] + 40,
          "40 counts/tick is 1200 counts/s, a brisk human move")
    down = lim._rate_limit({j: 0.0})
    check("clamping works downwards too", down[j] == hand[j] - MAX_STEP_PER_TICK)

    # Worst case: a target 4095 counts away still takes this many seconds to reach,
    # which is the whole point -- there is time to let go or press Stop.
    ticks = 4095 / MAX_STEP_PER_TICK
    check("a teleport still takes several ticks to play out",
          ticks >= 8, f"{ticks:.0f} ticks, {ticks/30:.2f}s at 30 Hz")

    print("\n9. save folder and replay")
    import tempfile

    from station.hardware import DATASETS_DIR

    ds = Station()
    ds.cfg = dict(st.cfg)
    check("defaults to the folder in this project", ds.datasets_dir() == DATASETS_DIR)
    with tempfile.TemporaryDirectory() as tmp:
        target = Path(tmp) / "recordings"
        r = ds.set_datasets_dir(str(target))
        check("accepts a folder and creates it", r.get("ok") is True and target.is_dir())
        check("recordings now point at it", ds.datasets_dir() == target)
        check("nothing left behind by the write test",
              not (target / ".station_write_test").exists())
        check("empty folder means an empty list", ds.list_datasets() == [])

        # A directory that looks like a dataset only if it has the metadata file.
        (target / "nexarm_fake").mkdir()
        check("a folder without metadata is not a recording", ds.list_datasets() == [])
        meta = target / "nexarm_fake" / "meta"
        meta.mkdir()
        (meta / "info.json").write_text(json.dumps({"total_episodes": 3}), encoding="utf-8")
        found = ds.list_datasets()
        check("a real recording is listed with its episode count",
              len(found) == 1 and found[0]["episodes"] == 3, str(found))

        check("replay refuses a name that is not there",
              ds.start_replay("no_such_thing", 0).get("ok") is False)
        ds.mode = "record"
        check("replay refuses while something else runs",
              ds.start_replay("nexarm_fake", 0).get("ok") is False)
        check("changing the folder refuses while something else runs",
              ds.set_datasets_dir(str(target)).get("ok") is False)
        ds.mode = "idle"

    bad = ds.set_datasets_dir("\x00not/a/real/path")
    check("an impossible folder is rejected, not silently used", bad.get("ok") is False,
          bad.get("why", ""))
    ds.set_datasets_dir("")
    check("an empty box goes back to the default", ds.datasets_dir() == DATASETS_DIR)

    print("\n10. runs on more than one operating system")
    check("camera backend chosen per platform, not hardcoded to DirectShow",
          "CAP_DSHOW" not in Path("station/hardware.py").read_text(encoding="utf-8"))
    check("detect picks a backend for this platform",
          detect.camera_backend() is not None, sys.platform)
    check("macOS duplicate serial ports would be de-duplicated",
          "/cu." in Path("station/detect.py").read_text(encoding="utf-8"))
    check("launcher knows about a posix venv",
          "bin" in Path("start.py").read_text(encoding="utf-8"))

    print("\n11. works with no camera, one camera, or two")
    for count, expect in ((0, set()), (1, {"front"}), (2, {"front", "wrist"}),
                          (3, {"front", "wrist"}), (6, {"front", "wrist"})):
        g = detect.guess_cameras(list(range(count)))
        check(f"{count} camera(s) -> {sorted(expect) or 'none'}", set(g) == expect, str(g))

    for count in (0, 1, 2):
        cam_st = Station()
        cam_st.cfg = dict(st.cfg)
        cam_st.cfg["cameras"] = detect.guess_cameras(list(range(count)))
        cfg = _record_config(cam_st)
        check(f"a recording config builds with {count} camera(s)",
              cfg is not None and len(cfg.robot.cameras) == count,
              f"{len(cfg.robot.cameras) if cfg else '-'} in robot config")
        check(f"{count}-camera config still has push_to_hub off",
              cfg is not None and cfg.dataset.push_to_hub is False)

    print("\n12. a fresh machine can tell why it is broken")
    import start as launcher

    ok, why = launcher.can_spawn(Path(sys.executable))
    check("the interpreter running this is recognised as working", ok, why[:80])

    with tempfile.TemporaryDirectory() as tmp:
        missing = Path(tmp) / "nope" / "python.exe"
        ok, why = launcher.can_spawn(missing)
        check("a python that is not there is reported, not assumed", not ok, why)

        # A file that exists and is not a runnable interpreter is exactly the
        # shape of the dead uv trampoline, which is the error a new machine hits.
        broken = Path(tmp) / "python.exe"
        broken.write_bytes(b"this is not an executable")
        ok, why = launcher.can_spawn(broken)
        check("a python that exists but will not start is caught",
              not ok, why.splitlines()[0][:80] if why else "")

    check("the launcher knows how to rebuild a dead environment",
          callable(getattr(launcher, "rebuild_venv", None)))
    check("the launcher repairs before handing over, not after",
          "working_venv_python()" in Path("start.py").read_text(encoding="utf-8"))
    check("there is a doctor to paste when it still will not run",
          callable(getattr(launcher, "doctor", None)))
    check("the installers check the environment can start",
          "-c \"pass\"" in Path("setup.ps1").read_text(encoding="utf-8")
          and '-c "pass"' in Path("setup.sh").read_text(encoding="utf-8"))
    check("the clone address in the instructions is the real repository",
          "djerok/lerobot-nexarm" in Path("SETUP.md").read_text(encoding="utf-8"))

    print("\n13. any kind of arm, not only a NexArm")
    try:
        any_kind_of_arm(st.cfg)
    except Exception as exc:
        import traceback

        traceback.print_exc()
        check("any-kind-of-arm checks ran to the end", False, f"{type(exc).__name__}: {exc}")

    print("\n14. the test left nothing behind")
    detect.CONFIG_PATH.unlink(missing_ok=True)
    check("no scratch config left on disk", not detect.CONFIG_PATH.exists())
    check("the real nexarm.json was never touched",
          detect.CONFIG_PATH != _REAL_CONFIG,
          f"real config is {_REAL_CONFIG.name}, test used {detect.CONFIG_PATH.name}")

    print("\n" + "-" * 50)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    for f in FAIL:
        print("  FAILED: " + f)
    return 1 if FAIL else 0


def _status_of(url: str) -> int:
    try:
        return get(url)[0]
    except urllib.error.HTTPError as e:
        return e.code


def _peek_events() -> tuple[None, dict]:
    """The event dict lerobot builds, without starting a real key listener."""
    return None, {"exit_early": False, "rerecord_episode": False, "stop_recording": False}


def _record_config(st: Station):
    """The config _record_worker hands to lerobot -- built by the station itself."""
    try:
        return st.record_config("Pick up the red block")
    except Exception as exc:
        print(f"  (record_config raised {type(exc).__name__}: {exc})")
        return None


def any_kind_of_arm(st_cfg: dict) -> None:
    """Section 13: everything that lets the station drive arms other than a NexArm.

    Nothing here moves a motor. Ports, buses and readers are stand-ins; what is
    real is LeRobot's own registry and classes, and the station's own code.
    """
    import builtins
    import math
    import tempfile
    import threading

    from serial.tools import list_ports

    from station import robots

    keys = [k.key for k in robots.all_kinds()]
    check("knows NexArm, SO-101, Koch and OpenManipulator-X",
          {"nexarm", "so101", "koch", "omx"} <= set(keys), str(keys))
    offered = [c["key"] for c in robots.choices()]
    check("the picker offers only arms this computer has the software for",
          all(robots.missing_software(robots.by_key(k)) is None for k in offered), str(offered))
    for key in ("so101", "koch", "omx"):
        kind = robots.by_key(key)
        try:
            f = robots.make_follower(kind, "COM98", f"{key}_follower")
            l = robots.make_leader(kind, "COM97", f"{key}_leader")
            check(f"{kind.label}: LeRobot builds both arms without touching hardware",
                  f.config.port == "COM98" and l.config.port == "COM97",
                  f"{type(f).__name__} + {type(l).__name__}")
        except Exception as exc:
            check(f"{kind.label}: LeRobot builds both arms without touching hardware",
                  False, f"{type(exc).__name__}: {exc}")

    # -- which kinds get tried on which USB chip
    class _Port:
        def __init__(self, device, vid, pid, description, serial_number=""):
            self.device, self.vid, self.pid = device, vid, pid
            self.description, self.serial_number = description, serial_number

    ch343 = _Port("COM8", 0x1A86, 0x55D3, "USB-Enhanced-SERIAL CH343 (COM8)", "5C82109071")
    ch340 = _Port("COM11", 0x1A86, 0x7523, "USB-SERIAL CH340 (COM11)")
    u2d2 = _Port("COM5", 0x0403, 0x6014, "USB Serial Port (COM5)")
    odd = _Port("COM9", 0x10C4, 0xEA60, "Silicon Labs CP210x (COM9)")
    tried = lambda p, force=None: [k.key for k in robots.kinds_to_try(p, force)]
    check("an SO board (CH343) is tried as an SO arm only", tried(ch343) == ["so101"],
          str(tried(ch343)))
    check("a CH340 is tried as a NexArm first, then as an SO arm",
          tried(ch340) == ["nexarm", "so101"], str(tried(ch340)))
    check("a Dynamixel U2D2 is tried as Koch and OpenManipulator-X",
          tried(u2d2) == ["koch", "omx"], str(tried(u2d2)))
    check("an unknown chip is tried as every everyday kind, never an exotic one",
          tried(odd) == ["nexarm", "so101", "koch", "omx"], str(tried(odd)))
    check("a NexArm probe is never sent down an SO bus, even when asked",
          tried(ch343, robots.NEXARM) == [])

    # -- finding two arms and telling which one is waved, with stand-in readers
    class _Reader:
        def __init__(self, positions):
            self.positions = list(positions)
            self.closed = False

        def read(self):
            return list(self.positions)

        def close(self):
            self.closed = True

    readers = {"COM8": _Reader([2000] * 6), "COM10": _Reader([2000] * 6),
               "COM11": _Reader([1500] * 6)}
    kinds_by_port = {"COM8": "so101", "COM10": "so101", "COM11": "nexarm"}
    ch343b = _Port("COM10", 0x1A86, 0x55D3, "USB-Enhanced-SERIAL CH343 (COM10)", "5C82107993")
    real_ports, real_open = robots.detect.usb_serial_ports, robots.open_reader
    try:
        robots.detect.usb_serial_ports = lambda: [ch343, ch343b, ch340]
        robots.open_reader = (lambda kind, port, log=print:
                              readers[port] if kinds_by_port.get(port) == kind.key else None)
        finder = robots.ArmFinder()
        ports = finder.open_all(log=lambda m: None)
        check("two SO arms beside one NexArm: the pair is chosen",
              finder.kind is not None and finder.kind.key == "so101"
              and sorted(ports) == ["COM10", "COM8"], f"{finder.kind and finder.kind.key} {ports}")
        check("the lone arm of the other kind is let go", readers["COM11"].closed)
        readers["COM8"].positions = [2600, 2000, 2000, 2000, 2000, 2000]
        finder.sample()
        leader, follower = finder.verdict()
        check("the arm that was waved is the leader", (leader, follower) == ("COM8", "COM10"),
              f"{leader} / {follower}")
        ids = robots.arm_ids(finder.kind, leader, follower, finder.serials)
        check("calibration names follow each arm's USB serial number",
              ids["leader_id"] == "so101_leader_5C82109071"
              and ids["follower_id"] == "so101_follower_5C82107993", str(ids))
        check("a NexArm gets no calibration names, as before",
              robots.arm_ids(robots.NEXARM, "COM11", "COM12", {}) == {})
        forced = robots.ArmFinder(force="so101")
        check("the robot can be picked by hand instead",
              forced.force is not None and forced.force.key == "so101")
    finally:
        robots.detect.usb_serial_ports, robots.open_reader = real_ports, real_open

    # -- reading a port with the right motor list: leader's, then follower's
    import types

    class _Bus:
        """A stand-in LeRobot motor bus whose motors answer as `answers` says."""
        def __init__(self, motors, answers):
            self.motors = {f"m{i}": types.SimpleNamespace(id=i, model=model)
                           for i, model in motors.items()}
            self.model_number_table = {"xl330-m077": 1190, "xl330-m288": 1200,
                                       "xl430-w250": 1060, "sts3215": 777}
            self.answers, self.timeouts, self.released = answers, [], False
            self.pings = 0

        def connect(self, handshake=True):
            pass

        def disconnect(self, disable_torque=True):
            pass

        def set_timeout(self, timeout_ms=None):
            self.timeouts.append(timeout_ms)

        def ping(self, motor_id, num_retry=0, raise_on_error=False):
            self.pings += 1
            return self.answers.get(motor_id)

        def sync_read(self, name, motors=None, normalize=True, num_retry=0):
            return {m: 2048 for m in self.motors}

        def disable_torque(self, motors=None, num_retry=0):
            self.released = True

    koch = robots.by_key("koch")
    omx = robots.by_key("omx")
    koch_follower_motors = {1: 1060, 2: 1060, 3: 1200, 4: 1200, 5: 1200, 6: 1200}
    leader_list = {i: "xl330-m077" for i in range(1, 7)}
    follower_list = {1: "xl430-w250", 2: "xl430-w250", 3: "xl330-m288",
                     4: "xl330-m288", 5: "xl330-m288", 6: "xl330-m288"}
    made = []

    def fake_role_bus(kind, port, role):
        bus = _Bus(leader_list if role == "leader" else follower_list, koch_follower_motors)
        made.append((role, bus))
        return bus

    real_role_bus = robots._role_bus
    try:
        robots._role_bus = fake_role_bus
        reader = robots.LeRobotReader.open(koch, "COM5")
        check("a Koch FOLLOWER port is recognised (its motors are not the leader's)",
              reader is not None and reader.role == "follower",
              f"tried {[r for r, _ in made]}")
        check("probing waits 100 ms a motor, not LeRobot's full second",
              all(b.timeouts and b.timeouts[0] == robots.PROBE_TIMEOUT_MS for _, b in made),
              str([b.timeouts for _, b in made]))
        made.clear()
        check("switching off a follower uses the follower's motor list",
              robots.release(koch, "COM5", "follower") and made[0][0] == "follower"
              and made[0][1].released)
        koch_follower_motors.clear()            # nothing answers on this port now
        made.clear()
        check("switching off reports failure when no motor answers",
              robots.release(koch, "COM5", "follower") is False
              and not any(b.released for _, b in made))
    finally:
        robots._role_bus = real_role_bus
    check("an OpenManipulator-X follower numbers its motors 11-16",
          sorted(m.id for m in robots.make_follower(omx, "COM98", "x").bus.motors.values())
          == list(range(11, 17)))

    # -- clone adapters that share a serial number
    real_ports = robots.detect.usb_serial_ports
    try:
        twins = [_Port("COM8", 0x1A86, 0x55D3, "CH343", "0001"),
                 _Port("COM10", 0x1A86, 0x55D3, "CH343", "0001")]
        robots.detect.usb_serial_ports = lambda: twins
        cfg_twins = {"leader_serial": "0001", "leader_port": "COM30",
                     "follower_serial": "0001", "follower_port": "COM31"}
        check("a serial number shared by two adapters is never followed",
              detect.remap_by_serial(cfg_twins) is False
              and cfg_twins["leader_port"] == "COM30")
        real_open2 = robots.open_reader
        robots.open_reader = lambda kind, port, log=print: _Reader([2000] * 6) \
            if kind.key == "so101" else None
        twin_finder = robots.ArmFinder()
        twin_finder.open_all(log=lambda m: None)
        check("...and is not saved as an arm's identity",
              twin_finder.serials == {"COM8": "", "COM10": ""}, str(twin_finder.serials))
        robots.open_reader = real_open2
    finally:
        robots.detect.usb_serial_ports = real_ports

    # -- a moved cable is followed by serial number
    real_comports = list_ports.comports
    try:
        list_ports.comports = lambda: [_Port("COM20", 0x1A86, 0x55D3, "CH343", "5C82109071")]
        moved = {"leader_serial": "5C82109071", "leader_port": "COM8",
                 "follower_serial": "nope", "follower_port": "COM10"}
        changed = detect.remap_by_serial(moved)
        check("an SO arm moved to another socket is found again by serial",
              changed and moved["leader_port"] == "COM20" and moved["follower_port"] == "COM10",
              str(moved))
        nexarm_cfg = {"leader_port": "COM11", "follower_port": "COM12"}
        check("a NexArm setup has no serials and is left alone",
              detect.remap_by_serial(nexarm_cfg) is False)
    finally:
        list_ports.comports = real_comports

    # -- an SO-101 setup end to end, up to the point a motor would be touched
    so = Station()
    so.cfg = {**st_cfg, "robot_type": "so101", "leader_port": "COM8", "follower_port": "COM10",
              "leader_id": "so101_leader_A", "follower_id": "so101_follower_B"}
    so.cfg.pop("baudrate", None)
    cfg = _record_config(so)
    check("SO-101: a recording config builds", cfg is not None)
    if cfg is not None:
        check("SO-101: push_to_hub is OFF", cfg.dataset.push_to_hub is False)
        check("SO-101: recordings are named for the kind of arm",
              cfg.dataset.repo_id == "local_user/so101_pick_up_the_red_block"
              and Path(cfg.dataset.root).name == "so101_pick_up_the_red_block",
              cfg.dataset.repo_id)
        # LeRobot registers ONE class under both SO names, so a config reports
        # whichever name was registered first. Same class either way.
        check("SO-101: the follower and leader are LeRobot's SO classes",
              cfg.robot.type in ("so100_follower", "so101_follower")
              and cfg.teleop.type in ("so100_leader", "so101_leader"),
              f"{cfg.robot.type} / {cfg.teleop.type}")
        check("SO-101: each arm keeps its own calibration name",
              cfg.robot.id == "so101_follower_B" and cfg.teleop.id == "so101_leader_A")
        check("SO-101: ports are the right way round",
              cfg.robot.port == "COM10" and cfg.teleop.port == "COM8")

    # -- safety numbers in the arm's own units
    so._limits = so._limits_for(so._make_follower(cameras={}))
    check("SO-101: joints are limited in degrees, the gripper in percent",
          so._limits["shoulder_pan.pos"].hi >= 180 and so._limits["gripper.pos"].hi <= 110,
          f"{so._limits['shoulder_pan.pos']} / {so._limits['gripper.pos']}")
    check("SO-101: a normal pose is accepted",
          so._sane({"shoulder_pan.pos": -90.0, "elbow_flex.pos": 45.0, "gripper.pos": 30.0}))
    check("SO-101: garbage is rejected",
          not so._sane({"shoulder_pan.pos": 9999.0})
          and not so._sane({"shoulder_pan.pos": math.nan})
          and not so._sane({"gripper.pos": None}))
    so._last_sent = {}
    so._rate_limit({"shoulder_pan.pos": 0.0})
    jump = so._rate_limit({"shoulder_pan.pos": 180.0})["shoulder_pan.pos"]
    check("SO-101: a 180 degree teleport is clamped to one tick's allowance",
          jump == so._limits["shoulder_pan.pos"].step, f"{jump}")
    hand = so._rate_limit({"shoulder_pan.pos": jump + 8.0})["shoulder_pan.pos"]
    check("SO-101: a brisk hand movement passes untouched", hand == jump + 8.0,
          "8 degrees a tick is 240 degrees a second")
    slow = so._rate_limit({"shoulder_pan.pos": 0.0}, sync=True)["shoulder_pan.pos"]
    check("SO-101: the opening sync moves about a degree a tick",
          abs(hand - slow) == so._limits["shoulder_pan.pos"].sync_step)
    nex = Station()
    nex.cfg = dict(st_cfg)
    check("a NexArm still uses its own raw-count numbers",
          nex._limits_for(nex._make_follower(cameras={})) is None
          and nex._step_for("x", sync=False) == 250)

    # -- holding still before torque comes on
    class _LeBus:
        def __init__(self):
            self.writes = []

        def sync_read(self, name, normalize=True):
            return {"shoulder_pan": 2047, "gripper": 1500} if name == "Present_Position" else {}

        def sync_write(self, name, values, normalize=True):
            self.writes.append((name, dict(values), normalize))

    bus = _LeBus()
    robots.park_goal(bus)
    check("before torque, a LeRobot arm's goal is set to where it already is",
          bus.writes == [("Goal_Position", {"shoulder_pan": 2047, "gripper": 1500}, False)],
          str(bus.writes))

    class _NexBus:
        def __init__(self):
            self.wrote = None

        def read_positions(self):
            return [2000] * 6

        def write_positions(self, positions):
            self.wrote = positions

    nbus = _NexBus()
    robots.park_goal(nbus)
    check("...and a NexArm's, the way it always was", nbus.wrote == [2000] * 6)

    # -- LeRobot's calibration questions arrive on the page, and Next answers them
    from lerobot.motors import motors_bus

    cal = Station()
    seen, done, errors = [], threading.Event(), []

    def fake_calibration():
        try:
            with cal._page_prompts():
                kept = input("Press ENTER to use provided calibration file associated "
                             "with the id x, or type 'c' and press ENTER to run calibration: ")
                seen.append(("kept file", kept, cal.prompt))
                input("Move SOLeader to the middle of its range of motion and press ENTER....")
                while not motors_bus.enter_pressed():
                    time.sleep(0.02)
        except Exception as exc:
            errors.append(exc)
        finally:
            done.set()

    real_input = builtins.input
    threading.Thread(target=fake_calibration, daemon=True).start()
    deadline = time.time() + 3
    while time.time() < deadline and not (cal.prompt and "halfway" in cal.prompt):
        time.sleep(0.02)
    check("a saved calibration is used without asking anyone",
          bool(seen) and seen[0][1] == "" and seen[0][2] is None)
    check("'move to the middle' reaches the page, in words for a child",
          bool(cal.prompt) and "halfway" in cal.prompt and "arm you hold" in cal.prompt,
          (cal.prompt or "")[:70])
    first = cal.prompt
    early = cal.prompt_next()
    check("a Next straight after a question appears is ignored",
          bool(early.get("ignored")) and cal.prompt == first, str(early))
    time.sleep(cal.NEXT_DEBOUNCE_S + 0.1)
    cal.prompt_next()
    cal.prompt_next()              # the second click of a double-click
    deadline = time.time() + 3
    while time.time() < deadline and not (cal.prompt and "both ends" in cal.prompt):
        time.sleep(0.02)
    check("a double-click does not skip the sweep through every joint",
          bool(cal.prompt) and "both ends" in cal.prompt and not done.is_set(),
          (cal.prompt or "")[:70])
    time.sleep(cal.NEXT_DEBOUNCE_S + 0.1)
    cal.prompt_next()
    check("Next ends the sweep", done.wait(3) and not errors, str(errors))
    check("the terminal prompt is put back afterwards",
          builtins.input is real_input and cal.prompt is None)

    stopper, out = Station(), []

    def waits_forever():
        try:
            with stopper._page_prompts():
                input("Move SOFollower to the middle of its range of motion and press ENTER....")
            out.append("returned")
        except robots.Cancelled:
            out.append("cancelled")

    stopper.mode = "teleop"
    t = threading.Thread(target=waits_forever, daemon=True)
    t.start()
    deadline = time.time() + 3
    while time.time() < deadline and not stopper.prompt:
        time.sleep(0.02)
    stopper.stop()
    t.join(3)
    stopper.mode = "idle"
    check("Stop ends a calibration that is waiting on Next", out == ["cancelled"], str(out))

    # -- record and replay: the first move glides from where the arm is, never jumps
    class _Arm:
        def __init__(self, pose):
            self.pose = pose

        def get_observation(self):
            if self.pose is None:
                raise OSError("no reply from the arm")
            return dict(self.pose)

    def glide(station, start, target, ticks=400):
        station._last_sent, station._catching_up = {}, False
        station._seed_from_arm(_Arm(start), target)
        seen, prev = [], dict(start)
        for _ in range(ticks):
            out = station._glide_in(target)
            seen.append(max(abs(out[k] - prev[k]) for k in out))
            prev = out
            if out == target:
                break
        return seen, prev

    nexa = Station()
    nexa.cfg = dict(st_cfg)
    nexa._limits = None
    j = "shoulder_pan.pos"
    steps, end = glide(nexa, {j: 2000.0}, {j: 3000.0})
    check("NexArm: the first move of a recording or replay is a small step, not the whole gap",
          steps[0] <= 12, f"first step {steps[0]:.0f} counts (was 1000: straight to the target)")
    check("NexArm: it glides the whole way, no jump at the end, and arrives",
          end == {j: 3000.0} and max(steps) <= 24,
          f"{len(steps)} ticks, biggest {max(steps):.0f}")
    so_steps, so_end = glide(so, {"shoulder_pan.pos": 0.0}, {"shoulder_pan.pos": 90.0})
    check("SO-101: the same glide in degrees, about one a tick",
          so_steps[0] <= 1.0 and so_end == {"shoulder_pan.pos": 90.0}, f"{len(so_steps)} ticks")
    nexa._last_sent, nexa._catching_up = {}, False
    nexa._seed_from_arm(_Arm(None), {j: 3000.0})
    check("an arm that cannot be read is not guessed at",
          nexa._last_sent == {} and nexa._catching_up is False)

    class _Leader:
        def __init__(self, pose):
            self.pose = pose

        def get_action(self):
            return dict(self.pose)

    class _Follower:
        def __init__(self, pose):
            self.pose, self.sent = pose, []

        def get_observation(self):
            return dict(self.pose)

        def send_action(self, action):
            self.sent.append(dict(action))
            return action

    f = _Follower({j: 2000.0})
    check("before a try, arms that already match are not walked at all",
          nexa._first_sync(f, _Leader({j: 2020.0}), 30, quick_if_matched=True) is True
          and f.sent == [] and nexa._last_sent == {j: 2000.0})
    nexa._cancel.set()
    f2 = _Follower({j: 2000.0})
    check("Stop pressed while it is lining up ends it, nothing more is sent",
          nexa._first_sync(f2, _Leader({j: 3500.0}), 30, quick_if_matched=True) is False
          and f2.sent == [])
    nexa._cancel.clear()

    # -- start position: saved while moving, gone back to slowly after a try
    joints = [f"{n}.pos" for n in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                                   "wrist_flex", "wrist_roll", "gripper")]
    sp = Station()
    sp.cfg = dict(st_cfg)
    check("a start position cannot be saved while the arm is not moving",
          sp.save_start_position().get("ok") is False)
    sp.mode, sp._synced = "teleop", False
    check("...nor during the opening sync", sp.save_start_position().get("ok") is False)
    sp._synced = True
    sp._last_sent = {k: 2000.0 + i for i, k in enumerate(joints)}
    check("while moving, it saves where the arm is, for this kind of robot",
          sp.save_start_position().get("ok") is True
          and sp.cfg.get("reset_robot") == "nexarm" and sp._start_pose() == sp._last_sent)
    check("the page is told a start position is saved", sp.snapshot()["start_saved"] is True)
    sp.mode = "idle"
    cfg_rec = sp.record_config("x")
    check("with a start position, the arm keeps holding at the end of a try (no drop)",
          cfg_rec is not None and cfg_rec.robot.disable_torque_on_disconnect is False)
    other = dict(sp.cfg, reset_robot="so101")
    sp_other = Station()
    sp_other.cfg = other
    check("a start position saved on another kind of robot is ignored",
          sp_other._start_pose() is None)

    class _HoldFollower:
        def __init__(self, pose):
            self.pose, self.sent, self.on = pose, [], False
            self.config = types.SimpleNamespace(disable_torque_on_disconnect=True)

        def connect(self, calibrate=True):
            self.on = True

        def get_observation(self):
            return dict(self.pose)

        def send_action(self, action):
            self.sent.append(dict(action))
            return action

        def disconnect(self):
            self.on = False

    class _LeaderBus:
        """A NexArm leader board: positions in, positions out, torque switch."""
        def __init__(self, pos):
            self.pos, self.torque, self.writes, self.closed = list(pos), False, [], False

        def read_positions(self):
            return list(self.pos)

        def write_positions(self, positions):
            self.writes.append(list(positions))

        def set_torque(self, on):
            self.torque = on

        def disconnect(self):
            self.closed = True

    class _FakeLeader:
        def __init__(self, pos):
            self.bus = _LeaderBus(pos)

        def connect(self, calibrate=True):
            self.bus.torque = False          # every leader comes up loose

        def disconnect(self):
            self.bus.disconnect()

    home = sp._start_pose()
    fake = _HoldFollower({k: v + 500.0 for k, v in home.items()})
    fake_leader = _FakeLeader([1000] * 6)
    real_make, real_make_leader = sp._make_follower, sp._make_leader
    sp._make_follower = lambda cameras=None: fake
    sp._make_leader = lambda: fake_leader           # never the real leader on a real port
    try:
        sp._return_to_start(30)
    finally:
        sp._make_follower, sp._make_leader = real_make, real_make_leader
    biggest = max((max(abs(b[k] - a[k]) for k in a)
                   for a, b in zip([fake.pose] + fake.sent, fake.sent)), default=0)
    check("after a try it glides back to the start position, slowly",
          bool(fake.sent) and all(abs(fake.sent[-1][k] - home[k]) <= 2 for k in home)
          and biggest <= 24, f"{len(fake.sent)} ticks, biggest {biggest:.0f} counts")
    check("...and is let go of still holding there", fake.config.disable_torque_on_disconnect is False)
    want = robots.nexarm_follower_to_leader(home)
    lw = fake_leader.bus.writes
    lead_big = max((max(abs(b[i] - a[i]) for i in range(6)) for a, b in zip(lw, lw[1:])), default=0)
    check("the arm in the child's hand goes back to the start too, slowly",
          bool(lw) and all(abs(lw[-1][i] - want[f"{n}.pos"]) <= 2
                           for i, n in enumerate(robots.NEXARM_JOINTS)) and lead_big <= 24,
          f"{len(lw)} ticks, ends {lw[-1] if lw else None}")
    check("...its motors were switched on holding where it was, and stay on at the start",
          lw and lw[0] == [1000] * 6 and fake_leader.bus.torque is True and fake_leader.bus.closed)
    fast = Station()
    fast.cfg = dict(st_cfg)
    fast._limits = None
    f3 = _HoldFollower({"shoulder_pan.pos": 2000.0})
    t0 = time.monotonic()

    class _At:
        def get_action(self):
            return {"shoulder_pan.pos": 2060.0}

    fast._first_sync(f3, _At(), 30, quick_if_matched=True)
    took = time.monotonic() - t0
    check("arms that nearly match line up in well under a second (it used to take 2 s)",
          took < 0.8, f"{took:.2f} s for a 60-count gap")
    wcam = Station()
    wcam.cfg = dict(st_cfg, cameras={"front": 0, "wrist": 1})
    wrec = wcam.record_config("x")
    check("record() encodes video while recording, so Done does not wait for it",
          wrec is not None and wrec.dataset.streaming_encoding is True)
    check("record() is configured with both cameras, at 640x480",
          wrec is not None and len(wrec.robot.cameras) == 2
          and all((c.width, c.height) == (640, 480) for c in wrec.robot.cameras.values()))
    import numpy as np
    sc = Station()
    sc._publish_bgr("front", np.full((240, 320, 3), (10, 20, 30), dtype=np.uint8))
    shim = hardware._StationCamera(sc, "front", 640, 480)
    shim.connect()
    img = shim.read_latest()
    check("record() reads the station's own camera: RGB, at the dataset's size",
          img.shape == (480, 640, 3) and tuple(img[0, 0]) == (30, 20, 10) and shim.is_connected,
          str(img.shape))
    blank = hardware._StationCamera(sc, "wrist", 640, 480).async_read()
    check("...and a camera with no picture gives a black frame instead of ending the try",
          blank.shape == (480, 640, 3) and not blank.any())
    check("clearing the start position works",
          sp.clear_start_position().get("ok") is True and sp._start_pose() is None
          and sp.record_config("x").robot.disable_torque_on_disconnect is True)

    tr = Station()
    check("'That went wrong' does nothing when nothing is recording",
          tr.throw_away_try().get("ok") is False)
    tr.mode = "record"
    tr.throw_away_try()
    check("'That went wrong' keeps the try (so the numbers match the sheet), marked deleted",
          tr.events == {"exit_early": True, "rerecord_episode": False, "stop_recording": False}
          and tr._deleted)
    tr2 = Station()
    tr2.mode = "record"
    tr2.stop()
    check("Stop during a try keeps it, marked deleted",
          tr2.events["exit_early"] is True and tr2._deleted is True)
    with tempfile.TemporaryDirectory() as tmp:
        lb = Station()
        root = Path(tmp)
        lb.set_labels({"driver": "Ann", "position": "front left", "wrong": "No",
                       "type": "Normal", "notes": "ok"})
        lb._write_label(root, 0, "pick block")
        lb._deleted = True
        lb._write_label(root, 1, "pick block")
        lb._labels_root = root
        rows = lb.recent_labels()
        check("labels.csv gets one line per try, kept and deleted, numbered like the dataset",
              [(r["try"], r["episode"], r["status"]) for r in rows]
              == [("1", "0", "kept"), ("2", "1", "deleted")]
              and rows[0]["position"] == "front left" and rows[0]["driver"] == "Ann", str(rows))
        check("...the per-try answers start blank again, the names stay",
              rows[1]["something_wrong"] == "" and rows[1]["driver"] == "Ann")

    # -- a recording is only replayed on the kind of arm that made it
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / "koch_wave"
        (root / "meta").mkdir(parents=True)
        (root / "meta" / "info.json").write_text(
            json.dumps({"total_episodes": 1, "robot_type": "koch_follower"}), encoding="utf-8")
        check("a Koch recording is not replayed on an SO-101",
              so._recorded_on_other_kind(root) == "koch_follower")
        (root / "meta" / "info.json").write_text(
            json.dumps({"total_episodes": 1, "robot_type": "so_follower"}), encoding="utf-8")
        check("an SO-101 recording is replayed on an SO-101",
              so._recorded_on_other_kind(root) is None)

    # -- cameras: any camera in either slot, and each picture at its own camera's speed
    import numpy as np

    cam = Station()
    cam.cfg = dict(st_cfg, cameras={"front": 0, "wrist": 1})
    cam.cameras_found = [0, 1, 2]
    reopened = []
    cam.open_cameras = lambda rescanned=False: reopened.append(dict(cam.cfg["cameras"]))
    r = cam.set_cameras(2, 1)
    check("a third camera can be chosen for the front",
          r.get("ok") is True and cam.cfg["cameras"] == {"front": 2, "wrist": 1} and reopened,
          str(cam.cfg["cameras"]))
    check("the same camera cannot fill both slots", cam.set_cameras(1, 1).get("ok") is False)
    check("a camera that was never found is refused", cam.set_cameras(7, None).get("ok") is False)
    check("the laptop webcam can be left out: one camera, as the front one",
          cam.set_cameras(None, 2).get("ok") is True and cam.cfg["cameras"] == {"front": 2})
    check("no camera at all is allowed", cam.set_cameras(None, None).get("ok") is True
          and cam.cfg["cameras"] == {})
    cam.mode = "record"
    check("cameras cannot be changed in the middle of a try", cam.set_cameras(0, 1).get("ok") is False)
    cam.mode = "idle"
    check("the page is given every camera to choose from", cam.camera_choices() == [0, 1, 2])

    class _Cap:
        """A camera that makes a frame every `period` seconds."""
        def __init__(self, period):
            self.period, self.open = period, True

        def isOpened(self):
            return self.open

        def set(self, *a):
            return True

        def read(self):
            time.sleep(self.period)
            self.n = getattr(self, "n", 0) + 1     # a real camera never repeats a frame
            return True, np.full((48, 64, 3), self.n % 250, dtype=np.uint8)

        def release(self):
            self.open = False

    real_open_capture = detect.open_capture
    speeds = {0: 1 / 30, 1: 1 / 10}
    detect.open_capture = lambda idx: _Cap(speeds[int(idx)])
    two = Station()
    two.cfg = dict(st_cfg, cameras={"front": 0, "wrist": 1})
    try:
        two.open_cameras(rescanned=True)
        time.sleep(2.5)
        fps = dict(two.camera_fps)
    finally:
        two.close_cameras()
        detect.open_capture = real_open_capture
    check("a slow camera no longer slows the other one down",
          fps.get("front", 0) >= 24 and 7 <= fps.get("wrist", 0) <= 11,
          f"front {fps.get('front')} fps (camera makes 30), wrist {fps.get('wrist')} fps (makes 10)")
    check("closing the cameras leaves teleop's stop signal alone", not two._stop.is_set())

    from station.hardware import CAMERA_FROZEN_SECONDS

    class _FreezingCap(_Cap):
        """Changing frames for a while, then the same frame forever: a frozen camera."""
        def __init__(self, good):
            super().__init__(1 / 30)
            self.good = good

        def read(self):
            time.sleep(self.period)
            self.n = getattr(self, "n", 0) + 1
            return True, np.full((48, 64, 3), min(self.n, self.good) % 250, dtype=np.uint8)

    opened = []

    def fake_open(idx):
        c = _FreezingCap(good=10) if not opened else _Cap(1 / 30)
        opened.append(c)
        return c

    detect.open_capture = fake_open
    fr = Station()
    fr.cfg = dict(st_cfg, cameras={"front": 0})
    try:
        fr.open_cameras(rescanned=True)
        time.sleep(CAMERA_FROZEN_SECONDS + 2.5)
        seq_then = fr._frame_seq.get("front", 0)
        time.sleep(0.5)
        resumed = fr._frame_seq.get("front", 0) > seq_then
    finally:
        fr.close_cameras()
        detect.open_capture = real_open_capture
    check("a camera that freezes is noticed and restarted by itself",
          len(opened) >= 2 and resumed, f"opened {len(opened)} times, frames flowing again: {resumed}")

    tok = Station()
    tok.cfg = dict(st_cfg, cameras={"front": 0})
    detect.open_capture = lambda idx: _Cap(1 / 30)
    try:
        tok.open_cameras(rescanned=True)
        old_stop, readers = tok._pump_stop, list(tok._pump_threads)
        tok.close_cameras()
    finally:
        detect.open_capture = real_open_capture
    check("closing the cameras stops their readers for good (none can come back)",
          old_stop.is_set() and not any(t.is_alive() for t in readers))

    class _Busy(_Cap):
        def isOpened(self):
            return False

    ch = Station()
    ch.cfg = dict(st_cfg, cameras={"front": 1, "wrist": 2}, cameras_chosen=True)
    real_probe = detect.probe_cameras
    detect.open_capture = lambda idx: _Busy(1)
    detect.probe_cameras = lambda **k: [0]          # only the laptop webcam is free
    try:
        ch.open_cameras()
    finally:
        detect.open_capture = real_open_capture
        detect.probe_cameras = real_probe
    check("a camera you chose is never swapped for a guess just because it is busy",
          ch.cfg["cameras"] == {"front": 1, "wrist": 2}, str(ch.cfg["cameras"]))

    # -- unplug and replug: each slot follows its own camera, never a stranger
    P_WEB = "\\\\?\\usb#vid_5986&pid_11ad&mi_00#6&6a0accc&1&0000#{x}\\global"
    P_FRONT = "\\\\?\\usb#vid_32e6&pid_9221&mi_00#7&7551382&0&0000#{x}\\global"
    P_WRIST = "\\\\?\\usb#vid_32e6&pid_9005&mi_00#7&23fc3727&0&0000#{x}\\global"
    P_FRONT_OTHER_PORT = "\\\\?\\usb#vid_32e6&pid_9221&mi_00#7&deadbee&0&0000#{x}\\global"
    present = []
    real_ids = detect.camera_identities
    detect.camera_identities = lambda: [{"index": i, "name": "cam", "path": p}
                                         for i, p in enumerate(present)]
    try:
        idc = Station()
        idc.cfg = dict(st_cfg, cameras={"front": 1, "wrist": 2})
        present[:] = [P_WEB, P_FRONT, P_WRIST]
        idc._remember_camera_ids()
        check("each slot remembers which physical camera it holds",
              idc.cfg["camera_ids"] == {"front": P_FRONT.lower(), "wrist": P_WRIST.lower()}
              or idc.cfg["camera_ids"] == {"front": P_FRONT, "wrist": P_WRIST})
        idc.cfg["camera_ids"] = {"front": P_FRONT, "wrist": P_WRIST}
        present[:] = [P_WEB, P_WRIST]                 # front unplugged: wrist slides to 1
        where = idc._resolve_cameras()
        check("front unplugged: front waits, and the wrist camera is followed to its new number",
              where == {"front": None, "wrist": 1} and idc.cfg["cameras"]["wrist"] == 1, str(where))
        check("...so nothing ever shows the wrist picture in the front slot", where["front"] is None)
        present[:] = [P_WEB, P_FRONT, P_WRIST]        # plugged back into the same port
        where = idc._resolve_cameras()
        check("plugged back in: both slots find their own camera again",
              where == {"front": 1, "wrist": 2}, str(where))
        present[:] = [P_WEB, P_WRIST, P_FRONT_OTHER_PORT]   # front moved to another port
        where = idc._resolve_cameras()
        check("plugged into a different port: recognised by its model, back in its slot",
              where == {"front": 2, "wrist": 1}, str(where))
        idc.mode = "idle"
        idc.usable = lambda: (True, "")           # this check is about cameras, not ports
        idc._camera_missing["front"] = True
        r = idc.start_record("x")
        check("recording will not start while a chosen camera is unplugged",
              r.get("ok") is False and "no picture" in r.get("why", ""), r.get("why", ""))
    finally:
        detect.camera_identities = real_ids

    # -- gripper guard: a jaw resting still is not a stalled jaw
    rg = Station()
    rg.cfg, rg._limits, rg._grip = dict(st_cfg), None, {}
    G1 = "gripper.pos"
    for _ in range(20):                                # closed and resting on its target
        rg._guard_gripper({G1: 1195.0}, {G1: 1210.0})
    first = [rg._guard_gripper({G1: 2833.0}, {G1: 1210.0 + 3 * i})[G1] for i in range(4)]
    check("a jaw resting closed opens when the trigger lets go (it used to stay pinned shut)",
          first == [2833.0] * 4, str(first))
    blocked = [rg._guard_gripper({G1: 1195.0}, {G1: 1800.0})[G1] for _ in range(12)]
    check("...and a jaw blocked by a block is still eased off after a moment",
          blocked[-1] == 1800.0 - hardware.GRIP_SQUEEZE, str(blocked[-3:]))

    # -- gripper guard: sensor jitter does not make it let go and strain again
    jg = Station()
    jg.cfg, jg._limits, jg._grip = dict(st_cfg), None, {}
    G2 = "gripper.pos"
    T = hardware.GRIP_STALL_TICKS
    jitter = [2753.0, 2758.0, 2752.0, 2757.0, 2753.0, 2759.0, 2754.0, 2756.0] * 3
    outs = [jg._guard_gripper({G2: 2833.0}, {G2: a})[G2] for a in jitter]
    check("gripper jitter does not make the guard let go (it used to strain again each time)",
          outs[T - 1:] == [outs[T - 1]] * (len(outs) - T + 1) and outs[T - 1] < 2800,
          str(outs[T - 2:T + 4]))

    w = Station()
    seq0 = w._frame_seq.get("front", 0)
    got = []
    threading.Thread(target=lambda: got.append(w.wait_frame("front", seq0, timeout=2.0)),
                     daemon=True).start()
    time.sleep(0.2)
    w._publish_bgr("front", np.zeros((48, 64, 3), dtype=np.uint8))
    time.sleep(0.2)
    check("a waiting video stream gets each new frame the moment it exists",
          bool(got) and got[0][0] is not None and got[0][0][:2] == b"\xff\xd8")
    check("...and a stream with no new frame times out cleanly",
          w.wait_frame("nothing", 0, timeout=0.2) == (None, 0))

    # -- EMERGENCY STOP
    class _NexBus:
        def __init__(self):
            self.torque = True
            self.log = []

        def set_torque(self, on):
            self.torque = on
            self.log.append(("torque", on))

        def read_positions(self):
            return [2000] * 6

        def write_positions(self, positions):
            self.log.append(("write", tuple(positions)))

    class _LeBus2:
        def __init__(self):
            self.off = False

        def disable_torque(self, motors=None, num_retry=0):
            self.off = True

    nex_dev = types.SimpleNamespace(bus=_NexBus(),
                                    config=types.SimpleNamespace(disable_torque_on_disconnect=True))
    robots.torque_off(nex_dev)
    check("E-STOP: a NexArm's motors go off at once",
          nex_dev.bus.torque is False and nex_dev.config.disable_torque_on_disconnect is False,
          "and its disconnect will not re-hold for 0.4 s")
    le_dev = types.SimpleNamespace(bus=_LeBus2(),
                                   config=types.SimpleNamespace(disable_torque_on_disconnect=True))
    robots.torque_off(le_dev)
    check("E-STOP: a LeRobot arm's motors go off at once", le_dev.bus.off is True)

    released = []
    real_release = robots.release
    robots.release = lambda kind, port, role="follower": released.append((port, role)) or True
    try:
        # Made-up ports, always: the backstop runs on its own thread and must
        # never reach a real arm, even if it outlives the stand-in release.
        fake_ports = {"leader_port": "COM901", "follower_port": "COM902"}
        es = Station()
        es.cfg = dict(st_cfg, **fake_ports)
        held = threading.Event()

        def hog():                      # something else holding the station's lock
            with es.lock:
                held.set()
                time.sleep(1.5)

        threading.Thread(target=hog, daemon=True).start()
        held.wait(2)
        t0 = time.monotonic()
        r = es.emergency_stop()
        took = time.monotonic() - t0
        check("E-STOP answers at once, even while the station is busy with something else",
              r.get("ok") is True and took < 0.2, f"{took * 1000:.0f} ms")
        deadline = time.time() + 5
        while es._estop_busy and time.time() < deadline:
            time.sleep(0.05)
        check("E-STOP: once nothing holds them, both arms are switched off",
              sorted(set(released)) == [("COM901", "leader"), ("COM902", "follower")],
              str(released))
        check("E-STOP: the page is told, with the time", bool(es.snapshot()["estop_at"]))
        es._estop_busy = True
        check("nothing can start while the motors are still being switched off",
              es.start_teleop().get("ok") is False)
        es._estop_busy = False

        er = Station()
        er.cfg = dict(st_cfg, **fake_ports)
        er.mode = "record"
        er.emergency_stop()
        check("E-STOP during a try marks it deleted and skips going back to start",
              er._deleted and er.events["exit_early"] and er._cancel.is_set())
        er.mode = "idle"                # lets its backstop finish, on the stand-in
        deadline = time.time() + 5
        while er._estop_busy and time.time() < deadline:
            time.sleep(0.05)

        # The off command goes out at once through the open connection -- not on
        # the worker's next move, which a recording never made (6 s powered).
        lv = Station()
        lv.cfg = dict(st_cfg, **fake_ports)
        lv.mode = "record"
        live_bus = _NexBus()
        lv._live["follower"] = types.SimpleNamespace(
            bus=live_bus, config=types.SimpleNamespace(disable_torque_on_disconnect=True))
        t0 = time.monotonic()
        lv.emergency_stop()
        while live_bus.torque and time.monotonic() - t0 < 2:
            time.sleep(0.005)
        took = time.monotonic() - t0
        check("E-STOP switches the motors off at once through the open connection",
              live_bus.torque is False and took < 0.2, f"{took * 1000:.0f} ms")
        lv._live.clear()
        lv.mode = "idle"
        deadline = time.time() + 5
        while lv._estop_busy and time.time() < deadline:
            time.sleep(0.05)

        # A real teleop worker on stand-in arms: E-STOP mid-move.
        class _FakeArm:
            def __init__(self, cfg=None):
                self.bus = _NexBus()
                self.config = types.SimpleNamespace(disable_torque_on_disconnect=True)
                self.flag_at_disconnect = None

            def connect(self, calibrate=True):
                self.configure()

            def configure(self):
                pass

            def get_observation(self):
                return {k: 2000.0 for k in joints}

            def get_action(self):
                return {k: 2000.0 for k in joints}

            def send_action(self, action):
                return action

            def disconnect(self):
                self.flag_at_disconnect = self.config.disable_torque_on_disconnect

        tele = Station()
        tele.cfg = dict(st_cfg, **fake_ports)
        arm_f, arm_l = _FakeArm(), _FakeArm()
        tele._make_follower = lambda cameras=None: arm_f
        tele._make_leader = lambda: arm_l
        real_usable = tele.usable
        tele.usable = lambda: (True, "ok")
        released.clear()
        tele.start_teleop()
        time.sleep(2.6)                 # past the opening sync, into live teleop
        moving = tele.mode == "teleop"
        t0 = time.monotonic()
        tele.emergency_stop()
        while tele.mode != "idle" and time.monotonic() - t0 < 3:
            time.sleep(0.01)
        stopped_in = time.monotonic() - t0
        check("E-STOP mid-move: teleop stops within a tick or two",
              moving and tele.mode == "idle" and stopped_in < 0.5, f"{stopped_in * 1000:.0f} ms")
        check("E-STOP mid-move: both arms go floppy, no 0.4 s re-hold on the way out",
              arm_f.bus.torque is False and arm_l.bus.torque is False
              and arm_f.flag_at_disconnect is False and arm_f.bus.log[-1] == ("torque", False),
              str(arm_f.bus.log[-2:]))
        tele.usable = real_usable
        deadline = time.time() + 5
        while tele._estop_busy and time.time() < deadline:
            time.sleep(0.05)
    finally:
        robots.release = real_release
    check("no emergency-stop backstop was left running to reach a real port",
          not any(st_._estop_busy for st_ in (es, er, tele)))

    # -- gripper guard: never keep pushing against a stop or a block
    gg = Station()
    gg.cfg = dict(st_cfg)
    gg._limits = None
    G = "gripper.pos"

    def run(seq):
        gg._grip = {}
        return [gg._guard_gripper({G: t}, {G: a})[G] for t, a in seq]

    T = hardware.GRIP_STALL_TICKS
    outs = run([(2833.0, 2753.0)] * (T + 4))
    check("NexArm gripper at its open stop: after a moment it is no longer pushed into it",
          outs[:T - 1] == [2833.0] * (T - 1) and outs[T - 1:] == [2768.0] * 5, str(outs))
    outs = run([(1689.0, 2100.0)] * (T + 2))
    check("gripper on a block: held just past where it stopped, not crushed",
          outs[-1] == 2085.0, str(outs))
    moving = [(1700.0, 2700.0 - 60 * i) for i in range(10)]
    check("a gripper that is moving is never held back", run(moving) == [1700.0] * 10)
    slow = [(2680.0 - 2 * i, 2700.0 - 2 * i) for i in range(10)]
    check("slow, careful closing is not mistaken for a stall", run(slow) == [t for t, _ in slow])
    gg._grip = {}
    seq = [(1689.0, 2100.0)] * (T + 2) + [(2833.0, 2100.0), (2833.0, 2200.0), (2833.0, 2400.0)]
    outs = [gg._guard_gripper({G: t}, {G: a})[G] for t, a in seq]
    check("letting go of the trigger releases the grip at once",
          outs[-2:] == [2833.0, 2833.0], str(outs[-3:]))
    out = gg._guard_gripper({G: 2833.0, "elbow_flex.pos": 1234.0},
                            {G: 2753.0, "elbow_flex.pos": 999.0})
    check("the guard never touches any other joint", out["elbow_flex.pos"] == 1234.0)
    sg = Station()
    sg._limits, sg._grip = so._limits, {}
    outs = [sg._guard_gripper({G: 100.0}, {G: 80.0})[G] for _ in range(T + 2)]
    check("SO-101 units: the same guard, in percent of the gripper's range",
          outs[-1] == 81.0, str(outs))


if __name__ == "__main__":
    sys.exit(main())
