#!/usr/bin/env python
"""Run the station's whole SO-100/101 path on simulated arms.

    python station/simtest.py                 every flow, in order (about five minutes)
    python station/simtest.py replay estop_teleop      the setup and teleop flows, then these

No SO-101 had been run on this station when this was written, so this drives the
real station -- detection, the wave test, calibration on the page, teleop, the
gripper guard, the start position, recording, replay, EMERGENCY STOP, lost
replies, Switch the motors off, a moved cable -- against feetech_sim.py: real
scservo_sdk, LeRobot and station code, fake bytes on the wire. The flows build
on each other (calibration, the start position, the recorded job), so they run
in order and the first two always run.

It needs nothing plugged in and touches nothing real: no serial port, no camera,
no calibration file, and not this folder's nexarm.json or datasets/ -- all of
that is pointed into a fresh temp folder. Everything the station prints goes to
station.log there; only the checks are printed. ESTOP_RUNS=15 repeats each
EMERGENCY STOP flow more often (default 4).
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
import tempfile
import threading
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TMP = Path(tempfile.mkdtemp(prefix="so101sim_"))
# Before LeRobot is imported: it reads these once, at import.
os.environ["HF_LEROBOT_HOME"] = str(TMP / "hf")
os.environ["HF_LEROBOT_CALIBRATION"] = str(TMP / "calibration")
os.environ["HF_HUB_OFFLINE"] = "1"
sys.path.insert(0, str(ROOT))

LOG = open(TMP / "station.log", "w", encoding="utf-8", buffering=1)
CONSOLE = os.fdopen(os.dup(1), "w", encoding="utf-8", buffering=1)
os.dup2(LOG.fileno(), 1)       # libx264 and friends write to the raw descriptors
os.dup2(LOG.fileno(), 2)
sys.stdout = LOG
sys.stderr = LOG

from station import feetech_sim as fs  # noqa: E402

fs.install()

from station import detect, hardware  # noqa: E402

detect.CONFIG_PATH = TMP / "nexarm.json"
detect.SHOTS_DIR = TMP / "shots"
hardware.DATASETS_DIR = TMP / "datasets"
detect.open_capture = lambda idx: fs.FakeCapture(idx)
detect.camera_identities = lambda: []

LEADER_PORT, FOLLOWER_PORT = "COM8", "COM10"
MID = {j: 2048.0 for j in fs.JOINTS}
FOLLOWER_REST = [2048, 1150, 2950, 2600, 2048, 2048]     # folded on the table
SWEEP = {"shoulder_pan": 1000, "shoulder_lift": 900, "elbow_flex": 900,
         "wrist_flex": 900, "gripper": 700}

RESULTS: list[tuple[bool, str, str]] = []
ALL_LOG: list[str] = []      # every line the station said; st.log_lines keeps only 400


def out(msg: str) -> None:
    print(msg, file=CONSOLE, flush=True)
    print(f"#### {msg}", flush=True)


def check(name: str, cond, detail: str = "") -> bool:
    ok = bool(cond)
    RESULTS.append((ok, name, detail))
    out(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))
    return ok


def wait_for(cond, timeout: float, step: float = 0.02) -> bool:
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        try:
            if cond():
                return True
        except Exception:
            pass
        time.sleep(step)
    return False


def swing(amplitudes: dict, rate: float):
    """A hand script: these joints swing around where they were grabbed."""
    return lambda t, j, b: b + amplitudes.get(j, 0) * math.sin(t * rate)


def build_arms() -> tuple[fs.Arm, fs.Arm]:
    fs.SIM.arms.clear()
    fs.SIM.ports.clear()
    leader = fs.Arm(LEADER_PORT, "5C82109071", [2048] * 6)
    follower = fs.Arm(FOLLOWER_PORT, "5C82109072", FOLLOWER_REST, rest=FOLLOWER_REST)
    for arm in (leader, follower):
        fs.SIM.arms[arm.port] = arm
        fs.SIM.ports.append(fs.FakePortInfo(arm.port, arm.serial_number))
    return leader, follower


def pose_report(leader, follower) -> str:
    rl, rf = leader.raws(), follower.raws()
    return " ".join(f"{j[:5]}:{a:.0f}/{b:.0f}(g{follower.servos[i + 1].target_raw():.0f})"
                    for i, (j, a, b) in enumerate(zip(fs.JOINTS, rl, rf)))


def log_has(text: str, since: int = 0) -> bool:
    return any(text in line for line in ALL_LOG[since:])


def last_lines(st, n: int) -> str:
    return " | ".join(list(st.log_lines)[-n:])


def ports_free() -> bool:
    return all(a.open_handle is None for a in fs.SIM.arms.values())


def gap_raw(a: fs.Arm, b: fs.Arm, joints=fs.JOINTS[:5]) -> float:
    ra, rb = a.raws(), b.raws()
    return max(abs(ra[fs.JOINTS.index(j)] - rb[fs.JOINTS.index(j)]) for j in joints)


def labels(root: Path) -> list[dict]:
    if not (root / "labels.csv").is_file():
        return []
    with (root / "labels.csv").open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


class Sampler:
    """The follower's pose, torque and torque limit every 5 ms, for looking at afterwards."""

    def __init__(self, arm: fs.Arm):
        self.arm, self.rows, self.stop = arm, [], threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop.is_set():
            self.rows.append((time.perf_counter(), self.arm.raws(), self.arm.torques(),
                              self.arm.limits_now()))
            time.sleep(0.005)

    def between(self, t0, t1):
        return [r for r in self.rows if t0 <= r[0] <= t1]


def calibrate_both(st, leader: fs.Arm, follower: fs.Arm) -> bool:
    """Answer LeRobot's first-time calibration on the page, the way a person would."""
    for arm, word in ((follower, "follower"), (leader, "leader")):
        if not wait_for(lambda: st.prompt and "halfway" in st.prompt, 25):
            check(f"calibration asks to put the {word} halfway", False, str(st.prompt))
            return False
        check(f"calibration asks to put the {word} halfway, in words for a child",
              ("robot arm" if word == "follower" else "arm you hold") in st.prompt,
              st.prompt[:90])
        arm.grab(lambda t, j, b: MID[j])
        time.sleep(st.NEXT_DEBOUNCE_S + 0.2)
        st.prompt_next()
        if not wait_for(lambda: st.prompt and "both ends" in st.prompt, 15):
            check(f"calibration asks to move every {word} joint to both ends", False,
                  str(st.prompt))
            return False
        # Slowly, as the page asks: LeRobot samples the range in a loop, and a
        # fast sweep can be missed between two samples.
        arm.grab(fs.sweep(SWEEP, period=2.0))
        time.sleep(len(SWEEP) * 2.0 + 0.3)
        st.prompt_next()
        # The person holds the follower until its motors come on.
        if word == "follower":
            wait_for(lambda: all(arm.torques()), 10)
        arm.grab(lambda t, j, b: MID[j])
    return True


# ====================================================================== flows

def flow_setup(ctx):
    out("\n== 1. First run: find the arms, wave test, cameras")
    st = hardware.Station()
    real_say = st.say

    def say(msg):
        ALL_LOG.append(msg)
        real_say(msg)

    st.say = say
    ok, why = st.load()
    check("no saved setup yet: the page asks for the arms",
          not ok and st.setup_stage == "need_arms", why)
    leader, follower = ctx["leader"], ctx["follower"]
    r = st.begin_arm_detect()
    check("both SO-101 arms are found and recognised",
          r.get("ok") and r.get("robot") == "SO-100 / SO-101", json.dumps(r)[:160])
    if not r.get("ok"):
        return None
    leader.grab(fs.sweep({"shoulder_pan": 500, "elbow_flex": 400}, period=0.8))
    st.drag_deadline = time.monotonic() + 2.5           # the page's 15 s, shortened
    res = {}
    while True:
        res = st.poll_arm_detect()
        if res.get("stage") != "dragging":
            break
        time.sleep(0.1)
    leader.grab()
    cfg = st.cfg
    check("the waved arm is the leader", res.get("stage") == "ready"
          and cfg.get("leader_port") == LEADER_PORT and cfg.get("follower_port") == FOLLOWER_PORT,
          f"{res.get('stage')} {cfg.get('leader_port')} / {cfg.get('follower_port')} "
          f"moved={res.get('moved')}")
    check("calibration names carry each adapter's serial",
          cfg.get("leader_id") == "so101_leader_5C82109071"
          and cfg.get("follower_id") == "so101_follower_5C82109072", str(cfg.get("leader_id")))
    check("two cameras found, front and wrist", cfg.get("cameras") == {"front": 0, "wrist": 1},
          str(cfg.get("cameras")))
    check("the wave test let go of both ports", ports_free())
    check("nothing was switched on while finding the arms",
          not any(leader.torques()) and not any(follower.torques()))
    wait_for(lambda: st.latest_bgr("front") is not None and st.latest_bgr("wrist") is not None, 5)
    check("both camera pictures are running",
          st.latest_bgr("front") is not None and st.latest_bgr("wrist") is not None)
    return st


def flow_teleop(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out("\n== 2. First Start moving: calibration on the page, then teleop")
    mark = len(ALL_LOG)
    r = st.start_teleop()
    check("Start moving starts", r.get("ok"), str(r))
    if not calibrate_both(st, leader, follower):
        st.stop()
        wait_for(lambda: st.mode == "idle", 10)
        return False
    ok = wait_for(lambda: log_has("Teleop running", mark), 30)
    check("both arms calibrate, connect and sync", ok, last_lines(st, 4))
    if not ok:
        check("(error)", False, str(st.error))
        return False
    follower.let_go()
    cal_dir = TMP / "calibration"
    files = sorted(p.relative_to(cal_dir).as_posix() for p in cal_dir.rglob("*.json"))
    spans = {}
    for f in cal_dir.rglob("*.json"):
        c = json.loads(f.read_text())
        spans[f.stem] = {j: c[j]["range_max"] - c[j]["range_min"] for j in SWEEP}
    check("each calibration caught both ends of every joint",
          all(abs(v - 2 * SWEEP[j]) <= 0.03 * 2 * SWEEP[j]
              for sp in spans.values() for j, v in sp.items()), str(spans))
    check("calibration saved once per arm, under its serial name",
          "robots/so_follower/so101_follower_5C82109072.json" in files
          and "teleoperators/so_leader/so101_leader_5C82109071.json" in files, str(files))
    check("follower motors on, leader motors off while teleoperating",
          all(follower.torques()) and not any(leader.torques()),
          f"{follower.torques()} / {leader.torques()}")
    leader.grab(swing({"shoulder_pan": 450, "elbow_flex": -350, "wrist_flex": 300}, 1.2))
    lags = []
    t_end = time.perf_counter() + 6
    while time.perf_counter() < t_end:
        time.sleep(0.1)
        lags.append(gap_raw(leader, follower))
    leader.grab()
    wait_for(lambda: gap_raw(leader, follower) < 6, 3)
    settled = gap_raw(leader, follower)
    check("the follower keeps up with the hand", max(lags) < 120,
          f"worst lag {max(lags):.0f} counts ({max(lags) * 360 / 4095:.1f} deg), "
          f"settled {settled:.1f}; loop {st.status.get('fps')} Hz")
    check("...and ends where the hand stopped", settled < 6,
          f"{settled:.1f} counts; leader/follower(goal) {pose_report(leader, follower)}")
    return True


def flow_gripper(ctx):
    leader, follower = ctx["leader"], ctx["follower"]
    out("\n== 3. Gripper closing on a block it cannot pass")
    g = follower.by_name("gripper")
    block_at = g.raw - 250              # a block, 250 counts in
    g.block = (block_at, -1)
    mark = len(ALL_LOG)
    leader.grab(lambda t, j, b: b - min(1.0, t / 0.6) * 650 if j == "gripper" else b)
    time.sleep(2.0)
    goal = g.target_raw()
    check("the guard notices the jaw has stopped on the block",
          log_has("Gripper is stopped", mark), f"jaw {g.raw:.0f}, block {block_at:.0f}")
    check("...and asks for only a little past it (no straining)", abs(goal - block_at) <= 25,
          f"goal {goal:.0f} vs jaw {g.raw:.0f} (unguarded it would be {g.raw - 400:.0f})")
    leader.grab(lambda t, j, b: b + min(1.0, t / 0.6) * 650 if j == "gripper" else b)
    time.sleep(1.2)
    g.block = None
    check("opening the trigger lets go of the guard at once", g.raw > block_at + 150,
          f"jaw {g.raw:.0f}")
    leader.grab()
    time.sleep(0.5)


def flow_start_and_stop(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out("\n== 4. Save a start position, Stop, the follower rests on low power")
    pose = {"shoulder_pan": -350, "shoulder_lift": 250, "elbow_flex": -300, "wrist_flex": 150}
    leader.grab(lambda t, j, b: b + pose.get(j, 0) * min(1.0, t / 1.5))
    time.sleep(2.5)
    leader.grab()
    time.sleep(0.6)
    r = st.save_start_position()
    check("Save this spot as the start", r.get("ok"), str(r))
    ctx["start_raw"] = follower.raws()
    check("the saved start is in degrees and percent",
          all(-200 < v < 200 for v in (st.cfg.get("reset_pose") or {"x": 999}).values()),
          str(st.cfg.get("reset_pose")))
    # Away from the start, so going back to it shows.
    leader.grab(lambda t, j, b: b + {"shoulder_pan": 500, "elbow_flex": 350}.get(j, 0)
                * min(1.0, t / 1.2))
    time.sleep(2.0)
    leader.grab()
    time.sleep(0.4)
    before = follower.raws()
    st.stop()
    wait_for(lambda: st.mode == "idle", 10)
    time.sleep(1.5)
    drop = max(abs(a - b) for a, b in zip(follower.raws(), before))
    check("after Stop the follower holds where it was (no drop)", drop < 15,
          f"moved {drop:.0f} counts")
    check("...on about 30% torque, motors on", all(follower.torques())
          and follower.limits_now() == [300] * 6, f"{follower.torques()} {follower.limits_now()}")
    check("...and the arm you hold is loose", not any(leader.torques()))
    check("Stop let go of both ports", ports_free())


def _record_try(st, leader, task, seconds=3.0, done="next"):
    """Start recording, move the leader for a while, then end the try as `done` says."""
    r = st.start_record(task)
    if not r.get("ok"):
        return r, None
    if not wait_for(lambda: st.status.get("phase") == "recording", 30):
        return {"ok": False, "why": "never started recording: " + last_lines(st, 3)}, None
    leader.grab(swing({"shoulder_pan": 300, "wrist_flex": -250}, 2.0))
    time.sleep(seconds)
    leader.grab()
    t_done = time.perf_counter()
    if done == "next":
        st.events["exit_early"] = True        # what the Done button does
    elif done == "redo":
        st.throw_away_try()
    elif done == "estop":
        st.emergency_stop()
    return {"ok": True}, t_done


def flow_record_with_start(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out("\n== 5. Record a try, with a start position saved")
    st.set_labels({"driver": "Ana", "recorder": "Ben", "labeler": "Cy", "position": "left"})
    smp = Sampler(follower)
    r, t_done = _record_try(st, leader, "pick red block", seconds=3.0)
    check("recording starts", r.get("ok"), str(r))
    if not r.get("ok"):
        smp.stop.set()
        return
    ok = wait_for(lambda: st.mode == "idle", 60)
    smp.stop.set()
    check("the try ends and the robot goes back", ok, last_lines(st, 3))
    root = TMP / "datasets" / "so101_pick_red_block"
    info_path = root / "meta" / "info.json"
    info = json.loads(info_path.read_text()) if info_path.is_file() else {}
    check("the dataset is saved, with one try, named for the SO arm",
          info.get("total_episodes") == 1, f"{root.name}: {info.get('total_episodes')} episodes")
    frames = st._episode_frames(root, 0)
    check("the try holds about 30 frames a second of it", 75 <= frames <= 110,
          f"{frames} frames for 3 s")
    feats = info.get("features", {})
    names = feats.get("action", {}).get("names")
    check("actions are the six SO joints", names == [f"{j}.pos" for j in fs.JOINTS], str(names))
    check("both camera videos are in it",
          {"observation.images.front", "observation.images.wrist"} <= set(feats), str(sorted(feats)))
    vids = sorted(p.name for p in (root / "videos").rglob("*.mp4"))
    check("...as video files", len(vids) == 2, str(vids))
    rows = labels(root)
    check("labels.csv has the try, kept, with the names", len(rows) == 1 and rows[0]["try"] == "1"
          and rows[0]["status"] == "kept" and rows[0]["driver"] == "Ana", str(rows[:1]))
    check("no recording problem was noted", rows and "station:" not in rows[0]["notes"],
          rows[0]["notes"] if rows else "")
    start = ctx.get("start_raw")
    now = follower.raws()
    gap = max(abs(a - b) for a, b in zip(now[:5], start[:5])) if start else 999
    per = {j[:8]: round(a - b) for j, a, b in zip(fs.JOINTS, now, start or now)}
    check("the robot went back to the start position", gap < 12, f"{gap:.0f} counts off {per}")
    check("...and rests there on low power", all(follower.torques())
          and follower.limits_now() == [300] * 6, f"{follower.torques()} {follower.limits_now()}")
    check("the arm you hold went back to the start too, and holds it",
          all(leader.torques()) and gap_raw(leader, follower) < 15,
          f"leader torque {leader.torques()}, {gap_raw(leader, follower):.0f} counts from the follower")
    # LeRobot's own configure() switches the motors off for a moment on every
    # connect, and going back to the start connects once. Nothing else may.
    rows_after = smp.between(t_done, time.perf_counter())
    limp_ms = 1000 * sum(b[0] - a[0] for a, b in zip(rows_after, rows_after[1:]) if not all(a[2]))
    lifts = [r[1][1] for r in rows_after]
    check("between Done and resting, the follower is never limp for long", limp_ms < 60,
          f"{limp_ms:.0f} ms in total; shoulder_lift moved over {max(lifts) - min(lifts):.0f} counts; "
          f"resting {(rows_after[-1][0] - t_done) if rows_after else 0:.1f} s after Done"
          if lifts else f"{limp_ms:.0f} ms")
    check("all ports let go", ports_free())


def flow_record_no_start(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out("\n== 6. Record a try with no start position: the follower must not drop")
    st.clear_start_position()
    smp = Sampler(follower)
    r, t_done = _record_try(st, leader, "pick red block", seconds=2.5)
    check("second try records", r.get("ok"), str(r))
    if not r.get("ok"):
        smp.stop.set()
        return
    ok = wait_for(lambda: st.mode == "idle", 60)
    time.sleep(0.5)
    smp.stop.set()
    check("the second try ends", ok)
    rows_after = smp.between(t_done, time.perf_counter())
    at_done = rows_after[0][1] if rows_after else follower.raws()
    off = [r for r in rows_after if not all(r[2])]
    worst = max((max(abs(a - b) for a, b in zip(r[1][:4], at_done[:4])) for r in rows_after),
                default=0)
    final = max(abs(a - b) for a, b in zip(follower.raws()[:4], at_done[:4]))
    check("after Done the follower never goes limp", not off,
          f"limp for {(off[-1][0] - off[0][0]) if off else 0:.2f} s, "
          f"from +{(off[0][0] - t_done) if off else 0:.2f} s")
    check("...and does not fold toward the table", worst < 40,
          f"fell up to {worst:.0f} counts ({worst * 360 / 4095:.0f} deg), "
          f"rests {final:.0f} counts from where the try ended")
    root = TMP / "datasets" / "so101_pick_red_block"
    check("the same job adds a second try to the same dataset", st._episodes_in(root) == 2,
          str(st._episodes_in(root)))
    check("all ports let go", ports_free())


def flow_replay(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out("\n== 7. Play it back")
    leader.grab()
    smp = Sampler(follower)
    t0 = time.perf_counter()
    r = st.start_replay("so101_pick_red_block", 0)
    check("replay of try 1 starts", r.get("ok"), str(r))
    if r.get("ok"):
        ok = wait_for(lambda: st.mode == "idle", 60)
        time.sleep(1.0)
        smp.stop.set()
        check("replay finishes", ok and log_has("Replay finished"), last_lines(st, 3))
        rows = smp.between(t0, time.perf_counter())
        pans = [r[1][0] for r in rows]
        check("the robot moved during replay", pans and max(pans) - min(pans) > 150,
              f"shoulder_pan span {max(pans) - min(pans):.0f} counts" if pans else "")
        t_end = rows[-1][0] if rows else 0
        tail = [r for r in rows if r[0] > t_end - 0.8]
        end_pose = rows[-1][1] if rows else follower.raws()
        check("after replay the follower stays up, resting on low power",
              tail and all(all(r[2]) for r in tail) and follower.limits_now() == [300] * 6,
              f"torque {follower.torques()} limits {follower.limits_now()}; "
              f"shoulder_lift ended at {end_pose[1]:.0f} (folded is {FOLLOWER_REST[1]})")
    smp.stop.set()
    bad = st.start_replay("so101_pick_red_block", 5)
    check("a try that does not exist is refused in the page's numbers",
          not bad.get("ok") and "1 to 2" in bad.get("why", ""), str(bad))
    check("all ports let go", ports_free())


def flow_estop_teleop(ctx, goal_enables_torque: bool):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    runs = int(os.environ.get("ESTOP_RUNS", "4"))
    tag = "a goal write switches torque on" if goal_enables_torque else "torque stays as written"
    out(f"\n== {ctx['n']}. EMERGENCY STOP while teleoperating ({tag}), {runs} times")
    for arm in (leader, follower):
        arm.goal_enables_torque = goal_enables_torque
    lat, reon, errors, never, stayed_on = [], 0, [], 0, []
    for n in range(runs):
        if not st.start_teleop().get("ok"):
            wait_for(lambda: not st._estop_busy, 15)
            st.start_teleop()
        mark = len(ALL_LOG)
        if not wait_for(lambda: log_has("Teleop running", mark), 20):
            errors.append(f"run {n}: never ran: {st.error}")
            st.stop()
            wait_for(lambda: st.mode == "idle", 10)
            continue
        leader.grab(swing({"shoulder_pan": 400, "elbow_flex": 300}, 3.0))
        time.sleep(0.7 + 0.13 * n)          # a different moment of the loop each time
        t0 = time.perf_counter()
        st.emergency_stop()
        got = wait_for(lambda: not any(follower.torques()), 2.0, step=0.001)
        t1 = time.perf_counter()
        if got:
            lat.append(t1 - t0)
        else:
            never += 1
        wait_for(lambda: st.mode == "idle" and not st._estop_busy, 15)
        cuts = []
        for servo in follower.servos.values():
            offs = [(t - t0, who) for t, on, who in servo.torque_log if not on and t >= t0]
            if offs:
                cuts.append(offs[0])
        print(f"ESTOP run {n}: follower off after {1000 * (t1 - t0):.0f} ms; first cut per motor: "
              + ", ".join(f"{1000 * dt:.0f}ms by {who}" for dt, who in cuts), flush=True)
        for servo in list(follower.servos.values()) + list(leader.servos.values()):
            if any(on and t > t1 for t, on, _ in servo.torque_log):
                reon += 1
                break
        if st.error:
            errors.append(f"run {n}: {st.error}")
        leader.grab()
        if any(follower.torques()) or any(leader.torques()):
            stayed_on.append(f"run {n + 1}: follower {follower.torques()} leader {leader.torques()}")
        follower.grab()                     # someone holds the floppy robot
        time.sleep(0.2)
    check("every motor of both arms is off after every stop", not stayed_on, "; ".join(stayed_on))
    if lat:
        check("the follower's motors go off fast", max(lat) < 0.25,
              f"{1000 * min(lat):.0f}-{1000 * max(lat):.0f} ms over {len(lat)} runs")
    check("the follower's motors always went off", never == 0, f"{never} runs never")
    check("nothing switched back on after the cut", reon == 0, f"{reon} runs switched back on")
    check("no error is reported for an emergency stop", not errors, "; ".join(errors)[:300])
    follower.let_go()
    for arm in (leader, follower):
        arm.goal_enables_torque = False
    check("all ports let go", ports_free())


def flow_estop_record(ctx, runs: int = 3):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out(f"\n== {ctx['n']}. EMERGENCY STOP while recording, {runs} times")
    root = TMP / "datasets" / "so101_estop_job"
    lat, errors = [], []
    for n in range(runs):
        before = st._episodes_in(root)
        wait_for(lambda: not st._estop_busy, 15)
        follower.let_go()
        r, t_done = _record_try(st, leader, "estop job", seconds=1.0 + 0.4 * n, done="estop")
        if not r.get("ok"):
            errors.append(f"run {n}: {r}")
            continue
        if wait_for(lambda: not any(follower.torques()), 2.0, step=0.001):
            lat.append(time.perf_counter() - t_done)
        follower.grab()
        wait_for(lambda: st.mode == "idle" and not st._estop_busy, 30)
        after = st._episodes_in(root)
        last = (labels(root) or [{}])[-1]
        check(f"run {n + 1}: the try is kept and marked deleted",
              after == before + 1 and last.get("status") == "deleted" and last.get("try") == str(after),
              f"episodes {before}->{after}, last label {last.get('try')}/{last.get('status')}")
        check(f"run {n + 1}: motors off on both arms afterwards",
              not any(follower.torques()) and not any(leader.torques()),
              f"{follower.torques()} {leader.torques()}")
        if st.error:
            errors.append(f"run {n}: {st.error}")
    if lat:
        check("the follower's motors go off fast while recording", max(lat) < 0.25,
              f"{1000 * min(lat):.0f}-{1000 * max(lat):.0f} ms")
    check("no error is reported for an emergency stop while recording", not errors,
          "; ".join(errors)[:300])
    follower.let_go()
    r, t_done = _record_try(st, leader, "estop job", seconds=1.5)
    ok = r.get("ok") and wait_for(lambda: st.mode == "idle", 60)
    check("a normal try after them adds to the same dataset",
          ok and st._episodes_in(root) == runs + 1, f"{st._episodes_in(root)} episodes; {st.error}")
    check("all ports let go", ports_free())


def flow_lost_replies(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out(f"\n== {ctx['n']}. One lost reply, at different moments")
    wait_for(lambda: not st._estop_busy, 15)
    follower.let_go()
    for who, arm in (("leader", leader), ("follower", follower)):
        st.error = None
        mark = len(ALL_LOG)
        st.start_teleop()
        running = wait_for(lambda: log_has("Teleop running", mark), 20)
        leader.grab(swing({"shoulder_pan": 300}, 2.0))
        time.sleep(1.0)
        fs.SIM.dropped = 0
        arm.drop_at = [time.perf_counter()]
        time.sleep(2.0)
        check(f"teleop survives one lost reply from the {who}", running and st.mode == "teleop",
              f"{fs.SIM.dropped} lost; {st.error or 'still running'}")
        st.stop()
        wait_for(lambda: st.mode == "idle", 10)
        leader.grab()
    # In the middle of a try.
    st.error = None
    root = TMP / "datasets" / "so101_flaky_job"
    before = st._episodes_in(root)
    fs.SIM.dropped = 0
    st.start_record("flaky job")
    wait_for(lambda: st.status.get("phase") == "recording", 30)
    leader.grab(swing({"shoulder_pan": 300}, 2.0))
    time.sleep(1.0)
    follower.drop_at = [time.perf_counter()]
    time.sleep(1.5)
    st.events["exit_early"] = True
    wait_for(lambda: st.mode == "idle", 60)
    leader.grab()
    check("a try survives one lost reply in the middle", st._episodes_in(root) == before + 1
          and not st.error, f"{fs.SIM.dropped} lost; {st.error or 'saved'}")
    st.error = None
    r, t_done = _record_try(st, leader, "flaky job", seconds=1.5)
    ok = r.get("ok") and wait_for(lambda: st.mode == "idle", 60)
    check("the next try of that job still saves", ok and st._episodes_in(root) == before + 2
          and not st.error, f"{st._episodes_in(root)} tries; {r.get('why') or st.error or ''}")
    # While the robot is connecting for a try.
    st.error = None
    st.start_record("flaky job")
    time.sleep(0.05)
    follower.drop_at = [time.perf_counter()]
    wait_for(lambda: st.status.get("phase") == "recording" or st.mode == "idle", 30)
    rec = st.status.get("phase") == "recording"
    if rec:
        time.sleep(0.5)
        st.events["exit_early"] = True
    wait_for(lambda: st.mode == "idle", 60)
    check("a lost reply while connecting does not stop the try", rec and not st.error,
          f"{st.error or 'recorded'}")
    for arm in (leader, follower):
        arm.drop_prob, arm.drop_at = 0.0, []
    st.error = None
    check("all ports let go", ports_free())


def flow_early_end(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out(f"\n== {ctx['n']}. A new job's first try ended before it started, then that job again")
    wait_for(lambda: not st._estop_busy, 15)
    follower.let_go()
    for how, job in (("done", "early done job"), ("stop", "early stop job"),
                     ("estop", "early estop job")):
        st.error = None
        mark = len(ALL_LOG)
        root = TMP / "datasets" / ("so101_" + job.replace(" ", "_"))
        st.start_record(job)
        # Pressed while the page still says "Getting the robot ready".
        wait_for(lambda: log_has("Getting ready to record", mark), 30)
        if how == "done":
            st.events["exit_early"] = True
        elif how == "stop":
            st.stop()
        else:
            st.emergency_stop()
        wait_for(lambda: st.mode == "idle" and not st._estop_busy, 60)
        follower.grab()
        check(f"{how} while getting ready: no error shown", not st.error, str(st.error))
        st.error = None
        follower.let_go()
        r, t_done = _record_try(st, leader, job, seconds=1.5)
        ok = r.get("ok") and wait_for(lambda: st.mode == "idle", 60)
        check("...and the next try of that job is saved as try 1",
              ok and st._episodes_in(root) == 1 and not st.error,
              f"{st._episodes_in(root)} tries; {r.get('why') or st.error or ''}")
    check("all ports let go", ports_free())


def flow_error_mid_later_try(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out(f"\n== {ctx['n']}. An error in the middle of try 2, then another try")
    wait_for(lambda: not st._estop_busy, 15)
    follower.let_go()
    job, root = "midway job", TMP / "datasets" / "so101_midway_job"
    _record_try(st, leader, job, seconds=1.5)
    wait_for(lambda: st.mode == "idle", 60)
    check("try 1 saved", st._episodes_in(root) == 1, str(st.error))
    st.error = None
    st.start_record(job)
    wait_for(lambda: st.status.get("phase") == "recording", 30)
    time.sleep(0.8)
    # The follower stops answering for 0.4 s: more than asking again covers.
    follower.drop_prob = 1.0
    time.sleep(0.4)
    follower.drop_prob = 0.0
    wait_for(lambda: st.mode == "idle", 60)
    check("an error mid-try is reported", bool(st.error), str(st.error))
    check("...and try 1 is still there", st._episodes_in(root) == 1, str(st._episodes_in(root)))
    st.error = None
    r, t_done = _record_try(st, leader, job, seconds=1.5)
    ok = r.get("ok") and wait_for(lambda: st.mode == "idle", 60)
    check("the next try after it is saved as try 2", ok and st._episodes_in(root) == 2
          and not st.error, f"{st._episodes_in(root)} tries; {r.get('why') or st.error or ''}")
    check("labels.csv numbers agree with the dataset",
          [x["try"] for x in labels(root)] == ["1", "2"], str([x["try"] for x in labels(root)]))
    check("all ports let go", ports_free())


def flow_release_and_cable(ctx):
    st, leader, follower = ctx["st"], ctx["leader"], ctx["follower"]
    out(f"\n== {ctx['n']}. Switch the motors off; a cable moved to another socket")
    wait_for(lambda: not st._estop_busy, 15)
    leader.servos[1].mem[40] = 1            # a motor on on each arm, to switch off
    follower.servos[2].mem[40] = 1
    r = st.release_motors()
    check("Switch the motors off: both arms floppy", r.get("ok") and not any(follower.torques())
          and not any(leader.torques()), f"{r} {follower.torques()} {leader.torques()}")
    # The follower's cable moves from COM10 to COM12; its adapter keeps its serial.
    arm = fs.SIM.arms.pop(FOLLOWER_PORT)
    arm.port = "COM12"
    fs.SIM.arms["COM12"] = arm
    fs.SIM.ports = ([p for p in fs.SIM.ports if p.device != FOLLOWER_PORT]
                    + [fs.FakePortInfo("COM12", arm.serial_number)])
    ok, why = st.usable()
    check("the moved arm is found again by its serial number, no wave test",
          ok and st.cfg.get("follower_port") == "COM12", f"{why} -> {st.cfg.get('follower_port')}")
    mark = len(ALL_LOG)
    st.start_teleop()
    running = wait_for(lambda: log_has("Teleop running", mark), 20)
    check("...and teleop works on the new port, without calibrating again",
          running and not log_has("halfway", mark), str(st.error))
    st.stop()
    wait_for(lambda: st.mode == "idle", 10)
    check("all ports let go", ports_free())


def flow_stop_during_calibration(ctx):
    out(f"\n== {ctx['n']}. Stop pressed while calibration waits on Next")
    leader, follower = build_arms()         # two arms this computer has never seen
    for f in (TMP / "calibration").rglob("*.json"):
        f.unlink()
    st = ctx["st"]
    st.cfg["follower_port"], st.cfg["leader_port"] = FOLLOWER_PORT, LEADER_PORT
    mark = len(ALL_LOG)
    st.start_teleop()
    asked = wait_for(lambda: st.prompt and "halfway" in st.prompt, 20)
    st.stop()
    ended = wait_for(lambda: st.mode == "idle", 10)
    check("Stop ends a calibration that is waiting",
          asked and ended and log_has("Stopped before", mark), last_lines(st, 2))
    check("...and lets go of the ports", ports_free())
    check("...with no motor left on", not any(follower.torques()) and not any(leader.torques()))
    check("...and no half-finished calibration file",
          not list((TMP / "calibration").rglob("*.json")))


# The first two always run: everything after them needs the arms calibrated.
FLOWS = {
    "setup": flow_setup,
    "teleop": flow_teleop,
    "gripper": flow_gripper,
    "start_stop": flow_start_and_stop,
    "record_start": flow_record_with_start,
    "record_nostart": flow_record_no_start,
    "replay": flow_replay,
    "estop_teleop": lambda c: flow_estop_teleop(c, False),
    "estop_teleop_goal": lambda c: flow_estop_teleop(c, True),
    "estop_record": flow_estop_record,
    "lost_replies": flow_lost_replies,
    "early_end": flow_early_end,
    "error_midway": flow_error_mid_later_try,
    "release_cable": flow_release_and_cable,
    "stop_calibration": flow_stop_during_calibration,
}


def main(argv) -> int:
    unknown = [a for a in argv if a not in FLOWS]
    if unknown:
        out(f"unknown flow(s) {unknown}; the flows are: {', '.join(FLOWS)}")
        return 2
    out(f"SO-101 simulation -- temp folder {TMP}")
    leader, follower = build_arms()
    stop = threading.Event()
    for a in (leader, follower):
        a.background(stop)
    ctx = {"leader": leader, "follower": follower, "n": 1}
    try:
        st = flow_setup(ctx)
        if st is None:
            return 1
        ctx["st"] = st
        for n, name in enumerate(list(FLOWS)[1:], start=2):
            if argv and name not in argv and name != "teleop":
                continue
            ctx["n"] = n
            try:
                result = FLOWS[name](ctx)
                if name == "teleop" and result is False:
                    break
            except Exception as exc:
                check(f"{name} ran to the end", False, f"{type(exc).__name__}: {exc}")
                traceback.print_exc()
            # Teleop keeps running into the gripper and start-position flows.
            if st.mode != "idle" and name not in ("teleop", "gripper"):
                st.stop()
                wait_for(lambda: st.mode == "idle", 20)
    finally:
        stop.set()
        try:
            if ctx.get("st"):
                ctx["st"].close_cameras()
        except Exception:
            pass
    bad = [r for r in RESULTS if not r[0]]
    out(f"\n{len(RESULTS) - len(bad)} passed, {len(bad)} failed   (log: {TMP / 'station.log'})")
    for ok, name, detail in bad:
        out(f"  FAIL {name}: {detail}")
    return 1 if bad else 0


if __name__ == "__main__":
    code = main(sys.argv[1:])
    LOG.flush()
    os._exit(code)        # the station's camera and worker threads are daemons; do not wait
