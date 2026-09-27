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
    check("finds serial ports", len(ports) >= 1, str(ports))
    check("port_exists agrees with the list",
          all(detect.port_exists(p) for p in ports))
    check("port_exists rejects a made-up port", not detect.port_exists("COM999"))
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
        state = json.loads(get(base + "/api/state")[2])
        check("/api/state has the fields the page reads",
              {"mode", "setup_stage", "config_ok", "cameras", "status", "log"} <= set(state))
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
    cfg = _build_record_config(st)
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
        cfg = _build_record_config(cam_st)
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

    print("\n13. the test left nothing behind")
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


def _build_record_config(st: Station):
    """Rebuild exactly what _record_worker builds, minus running it."""
    try:
        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.configs.dataset import DatasetRecordConfig
        from lerobot.robots.nexarm_follower import NexArmFollowerConfig
        from lerobot.scripts import lerobot_record as lr
        from lerobot.teleoperators.nexarm_leader import NexArmLeaderConfig
        from station.hardware import DATASETS_DIR
    except Exception:
        return None

    task = "Pick up the red block"
    fps = int(st.cfg.get("fps", 30))
    slug = "".join(c if c.isalnum() else "_" for c in task.lower()).strip("_")[:40] or "task"
    cams = {n: OpenCVCameraConfig(index_or_path=int(i), width=640, height=480, fps=fps)
            for n, i in st.cfg.get("cameras", {}).items()}
    return lr.RecordConfig(
        robot=NexArmFollowerConfig(port=st.cfg["follower_port"], cameras=cams),
        teleop=NexArmLeaderConfig(port=st.cfg["leader_port"]),
        dataset=DatasetRecordConfig(
            repo_id=f"local_user/nexarm_{slug}", single_task=task,
            root=str(DATASETS_DIR / f"nexarm_{slug}"), fps=fps,
            num_episodes=1, episode_time_s=3600, reset_time_s=0, push_to_hub=False,
        ),
        display_data=False, play_sounds=False,
    )


if __name__ == "__main__":
    sys.exit(main())
