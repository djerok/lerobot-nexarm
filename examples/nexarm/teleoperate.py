#!/usr/bin/env python

# Teleoperate NexArm: leader arm controls follower arm in real time.
#
# Usage:
#   python examples/nexarm/teleoperate.py --follower-port COM19 --leader-port COM18
#   python examples/nexarm/teleoperate.py --follower-port /dev/ttyUSB1 --leader-port /dev/ttyUSB0
#
# Optional flags:
#   --fps            Control loop rate (default: 30)
#   --front-cam      OpenCV camera index for front camera (default: 0)
#   --wrist-cam      OpenCV camera index for wrist camera (default: 1)
#   --no-display     Disable Rerun visualization

import argparse
import time

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.nexarm_follower import NexArmFollower, NexArmFollowerConfig
from lerobot.teleoperators.nexarm_leader import NexArmLeader, NexArmLeaderConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


def parse_args():
    parser = argparse.ArgumentParser(description="Teleoperate NexArm")
    parser.add_argument("--follower-port", required=True, help="Serial port for follower ESP32 (e.g. COM19)")
    parser.add_argument("--leader-port", required=True, help="Serial port for leader ESP32 (e.g. COM18)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--front-cam", type=int, default=0)
    parser.add_argument("--wrist-cam", type=int, default=1)
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    camera_config = {
        "front": OpenCVCameraConfig(index_or_path=args.front_cam, width=640, height=480, fps=args.fps),
        "wrist": OpenCVCameraConfig(index_or_path=args.wrist_cam, width=640, height=480, fps=args.fps),
    }

    follower_config = NexArmFollowerConfig(port=args.follower_port, cameras=camera_config)
    leader_config = NexArmLeaderConfig(port=args.leader_port)

    follower = NexArmFollower(follower_config)
    leader = NexArmLeader(leader_config)

    follower.connect()
    leader.connect()

    if not args.no_display:
        init_rerun(session_name="nexarm_teleoperate")

    print("Teleoperation started. Press Ctrl+C to stop.")
    try:
        while True:
            start = time.perf_counter()

            leader_pos = leader.get_action()
            follower.send_action(leader_pos)
            obs = follower.get_observation()

            if not args.no_display:
                log_rerun_data(obs)

            precise_sleep(1.0 / args.fps - (time.perf_counter() - start))
    except KeyboardInterrupt:
        print("Stopping teleoperation.")
    finally:
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()
