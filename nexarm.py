#!/usr/bin/env python
"""One entry point for the arm. Reads nexarm.json, so no ports, no indices.

    python nexarm.py setup                          find the arms and cameras
    python nexarm.py teleop                         leader drives follower
    python nexarm.py teleop --no-display            arms only, no viewer
    python nexarm.py record --task "Pick up block"  record a dataset
    python nexarm.py info                           show the saved config

If nexarm.json is missing, setup runs first on its own.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "nexarm.json"


def run_setup(extra=()):
    return subprocess.run(
        [sys.executable, str(HERE / "nexarm_autoconfig.py"), *extra]
    ).returncode


def load_config(auto=True):
    if not CONFIG_PATH.exists():
        if not auto:
            print("No nexarm.json. Run: python nexarm.py setup")
            sys.exit(1)
        print("No nexarm.json yet, finding the arms first.\n")
        if run_setup() != 0:
            sys.exit(1)
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def cam_flags(cfg):
    cams = cfg.get("cameras", {})
    flags = []
    if "front" in cams:
        flags += ["--front-cam", str(cams["front"])]
    if "wrist" in cams:
        flags += ["--wrist-cam", str(cams["wrist"])]
    return flags


def main():
    parser = argparse.ArgumentParser(description="NexArm launcher")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("setup", add_help=False)
    sub.add_parser("info")

    t = sub.add_parser("teleop")
    t.add_argument("--no-display", action="store_true")
    t.add_argument("--fps", type=int)

    r = sub.add_parser("record")
    r.add_argument("--task", default="Pick up the object")
    r.add_argument("--repo-id", help="defaults to local_user/nexarm_<task>")
    r.add_argument("--num-episodes", type=int, default=20)
    r.add_argument("--episode-time", type=int, default=10)
    r.add_argument("--reset-time", type=int, default=10)
    r.add_argument("--push-to-hub", action="store_true")

    args, unknown = parser.parse_known_args()

    if args.cmd == "setup":
        return run_setup(unknown)

    cfg = load_config()

    if args.cmd == "info":
        print(json.dumps(cfg, indent=2))
        return 0

    base = [
        "--leader-port", cfg["leader_port"],
        "--follower-port", cfg["follower_port"],
    ]

    if args.cmd == "teleop":
        cmd = [sys.executable, str(HERE / "examples/nexarm/teleoperate.py"), *base]
        cmd += ["--fps", str(args.fps or cfg.get("fps", 30))]
        cams = cfg.get("cameras", {})
        # Rerun logs camera frames, so turn the viewer off when there are none.
        if args.no_display or not cams:
            cmd.append("--no-display")
        cmd += cam_flags(cfg)

    else:
        slug = "".join(c if c.isalnum() else "_" for c in args.task.lower())[:30]
        repo_id = args.repo_id or f"local_user/nexarm_{slug}"
        cmd = [sys.executable, str(HERE / "examples/nexarm/record.py"), *base]
        cmd += [
            "--repo-id", repo_id,
            "--task", args.task,
            "--num-episodes", str(args.num_episodes),
            "--episode-time", str(args.episode_time),
            "--reset-time", str(args.reset_time),
            "--fps", str(cfg.get("fps", 30)),
        ]
        cmd += cam_flags(cfg)
        if args.push_to_hub:
            cmd.append("--push-to-hub")

    print(" ".join(cmd) + "\n")
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    sys.exit(main())
