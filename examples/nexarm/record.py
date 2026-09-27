#!/usr/bin/env python

# Record a demonstration dataset with NexArm.
#
# This is a convenience wrapper around the lerobot-record CLI.
# Replace YOUR_HF_USERNAME with your actual Hugging Face username
# (run `huggingface-cli whoami` to confirm).
#
# Usage:
#   python examples/nexarm/record.py \
#       --follower-port COM19 --leader-port COM18 \
#       --repo-id YOUR_HF_USERNAME/nexarm_pick \
#       --num-episodes 50 --episode-time 10 --reset-time 10
#
# Keys during recording:
#   Enter    start / confirm next episode
#   ←        redo last episode
#   ESC      finish early and save
#
# Alternatively, use the CLI directly:
#   lerobot-record \
#       --robot.type=nexarm_follower \
#       --robot.port=COM19 \
#       --robot.cameras='{"front":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":30},"wrist":{"type":"opencv","index_or_path":1,"width":640,"height":480,"fps":30}}' \
#       --teleop.type=nexarm_leader \
#       --teleop.port=COM18 \
#       --dataset.repo_id=YOUR_HF_USERNAME/nexarm_pick \
#       --dataset.single_task="Pick up the object" \
#       --dataset.num_episodes=50 \
#       --dataset.episode_time_s=10 \
#       --dataset.reset_time_s=10

import argparse
import subprocess
import sys


def parse_args():
    parser = argparse.ArgumentParser(description="Record NexArm dataset")
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--repo-id", required=True, help="e.g. my_hf_user/nexarm_pick")
    parser.add_argument("--task", default="Pick up the object", help="One-sentence task description")
    parser.add_argument("--num-episodes", type=int, default=50)
    parser.add_argument("--episode-time", type=int, default=10, help="Seconds per episode")
    parser.add_argument("--reset-time", type=int, default=10, help="Seconds to reset between episodes")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--front-cam", type=int, default=0)
    parser.add_argument("--wrist-cam", type=int, default=1)
    parser.add_argument("--push-to-hub", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    cameras_json = (
        f'{{"front":{{"type":"opencv","index_or_path":{args.front_cam},"width":640,"height":480,"fps":{args.fps}}},'
        f'"wrist":{{"type":"opencv","index_or_path":{args.wrist_cam},"width":640,"height":480,"fps":{args.fps}}}}}'
    )

    cmd = [
        sys.executable, "-m", "lerobot.scripts.lerobot_record",
        "--robot.type=nexarm_follower",
        f"--robot.port={args.follower_port}",
        f"--robot.cameras={cameras_json}",
        "--teleop.type=nexarm_leader",
        f"--teleop.port={args.leader_port}",
        f"--dataset.repo_id={args.repo_id}",
        f"--dataset.single_task={args.task}",
        f"--dataset.num_episodes={args.num_episodes}",
        f"--dataset.episode_time_s={args.episode_time}",
        f"--dataset.reset_time_s={args.reset_time}",
        "--display_data=true",
    ]

    if args.push_to_hub:
        cmd.append("--dataset.push_to_hub=true")

    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
