#!/usr/bin/env python

# Run inference with a trained policy on NexArm (no leader arm needed).
#
# Replace YOUR_HF_USERNAME with your actual Hugging Face username.
#
# Usage — local checkpoint:
#   python examples/nexarm/rollout.py \
#       --follower-port COM19 \
#       --policy-path outputs/train/nexarm_act/checkpoints/last/pretrained_model
#
# Usage — policy from Hugging Face Hub:
#   python examples/nexarm/rollout.py \
#       --follower-port COM19 \
#       --policy-path YOUR_HF_USERNAME/nexarm_act
#
# Alternatively, use the CLI directly:
#   lerobot-rollout \
#       --strategy.type=sentry \
#       --policy.path=outputs/train/nexarm_act/checkpoints/last/pretrained_model \
#       --robot.type=nexarm_follower \
#       --robot.port=COM19 \
#       --robot.cameras='{"front":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":30},"wrist":{"type":"opencv","index_or_path":1,"width":640,"height":480,"fps":30}}' \
#       --display_data=true

import argparse
import subprocess
import sys


def parse_args():
    parser = argparse.ArgumentParser(description="Run NexArm inference")
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--policy-path", required=True, help="Local checkpoint dir or HF Hub repo id")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--front-cam", type=int, default=0)
    parser.add_argument("--wrist-cam", type=int, default=1)
    parser.add_argument(
        "--strategy", default="sentry",
        choices=["base", "sentry", "highlight", "dagger"],
        help="Rollout strategy (default: sentry)"
    )
    parser.add_argument("--repo-id", default=None, help="Optional: save rollout dataset (e.g. YOUR_HF_USERNAME/eval_nexarm)")
    return parser.parse_args()


def main():
    args = parse_args()

    cameras_json = (
        f'{{"front":{{"type":"opencv","index_or_path":{args.front_cam},"width":640,"height":480,"fps":{args.fps}}},'
        f'"wrist":{{"type":"opencv","index_or_path":{args.wrist_cam},"width":640,"height":480,"fps":{args.fps}}}}}'
    )

    cmd = [
        sys.executable, "-m", "lerobot.scripts.lerobot_rollout",
        f"--strategy.type={args.strategy}",
        f"--policy.path={args.policy_path}",
        "--robot.type=nexarm_follower",
        f"--robot.port={args.follower_port}",
        f"--robot.cameras={cameras_json}",
        "--display_data=true",
    ]

    if args.repo_id:
        cmd.append(f"--dataset.repo_id={args.repo_id}")

    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
