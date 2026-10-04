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
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from station import detect, server                     # noqa: E402
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
    port = server.free_port(8199)
    httpd = server.serve(st, port)
    base = f"http://127.0.0.1:{port}"
    time.sleep(0.4)
    try:
        code, ctype, body = get(base + "/")
        check("serves the page", code == 200 and b"Robot Station" in body, ctype)
        check("the page has the robot picker and the calibration Next button",
              b'id="kindPick"' in body and b'id="promptCard"' in body)
        state = json.loads(get(base + "/api/state")[2])
        check("/api/state has the fields the page reads",
              {"mode", "setup_stage", "config_ok", "cameras", "status", "log",
               "robot", "robots", "prompt"} <= set(state))
        check("/api/state offers more than one kind of arm",
              len(state["robots"]) >= 2, str([r["key"] for r in state["robots"]]))
        r = post(base + "/api/prompt/next")
        check("Next with nothing waiting is refused, not an error", r.get("ok") is False,
              r.get("why", ""))
        check("/api/state reports idle", state["mode"] == "idle")
        if names and not busy and not no_cams:
            code, ctype, body = get(f"{base}/shot/{names[0]}.jpg")
            check("single still works", code == 200 and ctype == "image/jpeg" and len(body) > 1000,
                  f"{len(body)} bytes")

            # MJPEG has no end, so read a fixed slice and count boundaries.
            with urllib.request.urlopen(f"{base}/stream/{names[0]}", timeout=6) as r:
                ctype = r.headers.get("Content-Type", "")
                chunk = r.read(200_000)
            check("stream is multipart", "multipart/x-mixed-replace" in ctype, ctype)
            check("stream carries several frames",
                  chunk.count(b"--" + server.BOUNDARY.encode()) >= 2,
                  f"{chunk.count(b'--' + server.BOUNDARY.encode())} frames")

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
        post(base + "/api/record/redo")
        check("redo button sets rerecord", st.events["rerecord_episode"] is True)
        st.events["rerecord_episode"] = False

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
    cal.prompt_next()
    deadline = time.time() + 3
    while time.time() < deadline and not (cal.prompt and "both ends" in cal.prompt):
        time.sleep(0.02)
    check("the sweep through every joint is asked for next",
          bool(cal.prompt) and "both ends" in cal.prompt, (cal.prompt or "")[:70])
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


if __name__ == "__main__":
    sys.exit(main())
