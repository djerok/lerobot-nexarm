#!/usr/bin/env python
"""Teleoperate NexArm with NO cameras.

examples/nexarm/teleoperate.py always builds a front+wrist camera pair and
follower.connect() opens them, so it dies before the arms ever move if the
cameras are not plugged in. This is the same loop with cameras={}.

Ports are NOT defaulted. COM numbers move when a cable moves, so a baked-in
default is a guess that silently drives the wrong arm. They come from
nexarm.json, which nexarm_autoconfig.py writes after physically proving which
arm is which. No config and no flags means this refuses to run.

Usage:
    .venv\\Scripts\\python.exe teleop_nocam.py
"""

import argparse
import json
import sys
import time
from pathlib import Path

from lerobot.robots.nexarm_follower import NexArmFollower, NexArmFollowerConfig
from lerobot.teleoperators.nexarm_leader import NexArmLeader, NexArmLeaderConfig
from lerobot.utils.robot_utils import precise_sleep

CONFIG_PATH = Path(__file__).resolve().parent / "nexarm.json"


def resolve_ports(args):
    """Explicit flags win. Otherwise nexarm.json. Otherwise stop."""
    if args.leader_port and args.follower_port:
        return args.leader_port, args.follower_port, "flags"

    if not CONFIG_PATH.exists():
        sys.exit(
            "No nexarm.json, so there is nothing to say which arm is which.\n"
            "Run this first:  python nexarm.py setup\n"
            "Or pass both ports:  --leader-port COMx --follower-port COMy"
        )

    cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    leader = args.leader_port or cfg.get("leader_port")
    follower = args.follower_port or cfg.get("follower_port")
    if not leader or not follower:
        sys.exit(f"{CONFIG_PATH.name} has no leader_port/follower_port. Re-run setup.")
    if leader == follower:
        sys.exit(f"{CONFIG_PATH.name} gives the same port for both arms. Re-run setup.")
    return leader, follower, CONFIG_PATH.name


def first_sync(follower, leader, seconds, fps):
    """Walk the follower to the leader's pose slowly, before live teleop starts.

    At startup the two arms are almost never in the same pose, so the very first
    send_action asks the follower to cross that whole gap at once. It slams, and
    it clicks. This eases across the gap instead, then hands over to the loop
    already matched.
    """
    target = leader.get_action()
    obs = follower.get_observation()
    start = {k: float(obs[k]) for k in target if k in obs}
    if not start:
        print("Could not read the follower's pose, so skipping the slow sync.")
        return

    gap = max(abs(target[k] - start[k]) for k in start)
    print(f"Slow first sync: {gap:.0f} counts to close, over {seconds:.0f}s.")

    steps = max(1, int(seconds * fps))
    for i in range(1, steps + 1):
        f = i / steps
        # ease in and out, so it does not jerk at either end
        e = f * f * (3.0 - 2.0 * f)
        t0 = time.perf_counter()
        follower.send_action({k: start[k] + (target[k] - start[k]) * e for k in start})
        precise_sleep(1.0 / fps - (time.perf_counter() - t0))
    print("Synced.")


def main():
    p = argparse.ArgumentParser(description="Teleoperate NexArm (no cameras)")
    p.add_argument("--follower-port", default=None)
    p.add_argument("--leader-port", default=None)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument(
        "--sync-seconds",
        type=float,
        default=4.0,
        help="how long the follower takes to reach the leader's pose at startup (0 disables)",
    )
    args = p.parse_args()

    leader_port, follower_port, source = resolve_ports(args)
    print(f"leader   {leader_port}\nfollower {follower_port}\n(from {source})")

    follower = NexArmFollower(NexArmFollowerConfig(port=follower_port, cameras={}))
    leader = NexArmLeader(NexArmLeaderConfig(port=leader_port))

    follower.connect()
    leader.connect()

    try:
        if args.sync_seconds > 0:
            first_sync(follower, leader, args.sync_seconds, args.fps)

        print("Teleoperation started. Move the leader arm. Ctrl+C to stop.")
        while True:
            start = time.perf_counter()
            follower.send_action(leader.get_action())
            precise_sleep(1.0 / args.fps - (time.perf_counter() - start))
    except KeyboardInterrupt:
        print("Stopping teleoperation.")
    finally:
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()
