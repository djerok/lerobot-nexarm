# Hiwonder NexArm LeRobot VLA Open-Source 6-Axis Robotic Arm

English | [中文](./README_cn.md)

[NexArm](https://www.hiwonder.com/products/nexarm6-axis) is an open-source, [🤗 LeRobot](https://github.com/huggingface/lerobot)-native robotic arm designed for embodied AI research and rapid validation of imitation and reinforcement learning policies. Its dual‑chip architecture (ESP32 + AT32) enables synchronous leader‑follower teleoperation with millisecond‑level tracking latency, generating clean demonstration data that feeds directly into LeRobot training pipelines.

The robotic arm features an all‑metal chassis driven by 65 kg·cm magnetic encoder servos, delivering ±2 mm repeatability and smooth, jitter‑free motion. Combined with advanced inverse kinematics and curve smoothing, it natively computes complex trajectories while minimizing start‑stop vibrations for fluid movement.

For on‑device perception, NexArm integrates a 6 TOPS K230 vision module, allowing you to run multimodal large models and computer vision pipelines—such as YOLO tracking and hand‑eye coordinated grasping—without a PC. All schematics, firmware, and code are open source, making NexArm a transparent and modifiable platform for academic research and real‑world robotic prototyping.

<p align="center">
  <img src="./media/readme/VLA_architecture.jpg" alt="nexarm" width="600"/>
</p>

## 🎯 Compared to SO‑ARM101:

- Payload: 500 g (150% improvement over SO‑ARM101)
- Repeatability: ±2 mm (33% improvement over SO‑ARM101)
- Architecture: ESP32 + AT32 dual‑chip design
- Build: All‑metal body with rotating base and parallel‑rail gripper
- Versatility: Supports multiple control methods out of the box

## End-to-End Robotic Arm Comparison

| Model | NexArm | SO-ARM101 |
|-------|--------|-----------|
| Payload | 500 g | 200 g |
| Repeatability | ±2 mm | ±3 mm |
| Workspace | 0.5 m | 0.4 m |
| Joint Servo | Dual-output-shaft magnetic encoder bus servo | Single-output-shaft magnetic encoder bus servo with fixed shaft |
| Body Material | Aerospace-grade metal structure | PLA 3D-printed structural parts |
| End Effector | Parallel rail gripper | 3D-printed vertical-opening gripper |
| Product Positioning | Advanced end-to-end development / advanced embodied AI applications | Entry-level end-to-end development / basic embodied AI applications |

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Installation](#installation)
- [Step 1: Find Serial Ports](#step-1-find-serial-ports)
- [Step 2: Find Cameras](#step-2-find-cameras)
- [Step 3: Teleoperation Test](#step-3-teleoperation-test)
- [Step 4: Collect a Dataset](#step-4-collect-a-dataset)
- [Step 5: Train a Policy](#step-5-train-a-policy)
- [Step 6: Run Inference](#step-6-run-inference)
- [Code Architecture](#code-architecture)
- [Troubleshooting](#troubleshooting)

---

## Hardware Overview

### Components

| Component | Description |
|-----------|-------------|
| **Leader arm** | ESP32 board driving 6 × HX-30HM servos. Operator freely moves this arm during teleoperation. |
| **Follower arm** | ESP32 + AT32F421 co-processor driving 6 × HX-30HM servos. Mirrors the leader or executes policy output. |
| **Servos** | HX-30HM serial bus servos — 12-bit resolution (0–4095), 1 Mbps, high torque. |
| **Cameras** | 2 × USB cameras: `front` (top-down workspace view) and `wrist` (end-effector close-up), 640×480 @ 30 FPS. |

### Joint Layout (6 DOF)

| Joint | Name | Notes |
|-------|------|-------|
| 1 | `shoulder_pan` | Base rotation |
| 2 | `shoulder_lift` | Mirrored between leader and follower (4096 − pos) |
| 3 | `elbow_flex` | Elbow |
| 4 | `wrist_flex` | Wrist pitch |
| 5 | `wrist_roll` | Wrist rotation |
| 6 | `gripper` | Open/close, mapped range [1195, 2833] |

### Communication Protocol

NexArm uses a custom CommProtocol over USB serial:

```
Frame: [0xFF][0xFF][ID][LEN][CMD][ARGS...][CHECKSUM]
```

| CMD | Function | Direction |
|-----|----------|-----------|
| 56 | Set motion speed and acceleration (`motion_speed`, `motion_acc`) | Host → Follower |
| 68 | Enter/exit LeRobot bridge mode (follower only) | Host → Follower |
| 96 | Read 6 servo positions (12-byte reply) | Host → Device → Host |
| 97 | Write 6 servo positions (12 bytes, no reply) | Host → Device |
| 98 | Enable/disable torque | Host → Device |

---

## Installation

### Option A — conda (recommended)

```bash
git clone https://github.com/Hiwonder-official/lerobot-nexarm.git
cd lerobot-nexarm

conda create -n nexarm python=3.12 -y
conda activate nexarm

pip install -e ".[nexarm]"
```

To enable real-time Rerun visualization during teleoperation:

```bash
pip install -e ".[nexarm,viz]"
```

### Option B — venv

```bash
git clone https://github.com/Hiwonder-official/lerobot-nexarm.git
cd lerobot-nexarm

python -m venv .venv

# Activate
# Windows:
.venv\Scripts\activate
# Linux / macOS:
source .venv/bin/activate

pip install -e ".[nexarm]"
```

### Verify

```bash
python -c "from lerobot.robots.nexarm_follower import NexArmFollower; print('OK')"
```

### Connect Hardware

1. Plug the **follower arm** ESP32 into the PC via USB.
2. Plug the **leader arm** ESP32 into the PC via USB.
3. Plug in both USB cameras (front + wrist).

### Platform Support

| Platform | Status | Notes |
|----------|--------|-------|
| Windows 10/11 | Verified | Install CH340 driver; port format `COM19` |
| Ubuntu 20.04+ | Verified | Port format `/dev/ttyUSB0`; add user to `dialout` group |
| macOS | Verified | Port format `/dev/tty.usbserial-xxx` |

---

## Step 1: Find Serial Ports

Identify which port corresponds to the leader and which to the follower.

```bash
python -m lerobot.scripts.lerobot_find_port
```

Typical output on Windows:

| Port | Device |
|------|--------|
| COM18 | Leader ESP32 |
| COM19 | Follower ESP32 |

On Linux these are typically `/dev/ttyUSB0` and `/dev/ttyUSB1`.

> Tip: plug in one arm at a time if you are unsure which port belongs to which arm.

---

## Step 2: Find Cameras

Identify which camera index is `front` and which is `wrist`.

```bash
python -m lerobot.scripts.lerobot_find_cameras opencv
```

Or scan manually and save images to compare:

```python
import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(f"cam_{i}.png", frame)
            print(f"Camera {i}: available")
        cap.release()
```

- **front**: top-down view of the entire workspace
- **wrist**: close-up view of the end-effector / gripper

> Note: camera indices can change when you unplug and replug USB devices. Re-scan after reconnecting.

---

## Step 3: Teleoperation Test

Verify the leader-follower link. The leader arm runs torque-free so the operator can move it freely; the follower mirrors every joint in real time.

Edit `examples/nexarm/teleoperate.py` with your actual port numbers and camera indices, then run:

```bash
python examples/nexarm/teleoperate.py \
  --follower-port COM19 \
  --leader-port COM18
```

Or pass everything on the command line directly:

```bash
python examples/nexarm/teleoperate.py \
  --follower-port COM19 --leader-port COM18 \
  --front-cam 0 --wrist-cam 1 --fps 30
```

### Motion Speed and Acceleration

The follower arm's motion profile is controlled by two parameters in `NexArmFollowerConfig` (or via the YAML / command line):

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `motion_speed` | `2000` | 0–3400 | Maximum servo speed in raw units/s. `0` = no limit. |
| `motion_acc` | `100` | 0–254 | Acceleration ramp. `0` = instant (max acceleration). Higher = smoother ramp. |

These are applied once at connection time via the firmware. You can override them in `examples/nexarm/teleoperate.py` by editing the `NexArmFollowerConfig`:

```python
follower_config = NexArmFollowerConfig(
    port="COM19",
    motion_speed=2000,   # adjust for faster or slower movement
    motion_acc=100,      # adjust for harder or softer acceleration
    cameras=camera_config,
)
```

Or on the command line:

```bash
python examples/nexarm/teleoperate.py \
  --follower-port COM19 --leader-port COM18
```

> Note: the firmware handles speed/acceleration limiting internally — no software-side delta clamping is applied.

**What to check:**
- Follower tracks the leader smoothly across all joints.
- `shoulder_lift` direction is automatically mirrored.
- Gripper open/close maps correctly.

---

## Step 4: Collect a Dataset

Record demonstration episodes via teleoperation for imitation learning.

Edit `examples/nexarm/record.py`, then run:

```bash
python examples/nexarm/record.py \
  --follower-port COM19 --leader-port COM18 \
  --repo-id YOUR_HF_USERNAME/nexarm_pick \
  --task "Pick up the red block" \
  --num-episodes 50 --episode-time 10 --reset-time 10
```

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dataset.repo_id` | `local/nexarm_pick` | Dataset name (saved locally) |
| `dataset.num_episodes` | 50 | Number of episodes to record |
| `dataset.episode_time_s` | 10 | Duration of each episode (seconds) |
| `dataset.reset_time_s` | 10 | Time between episodes to reset the scene |
| `dataset.fps` | 30 | Recording frame rate |
| `dataset.push_to_hub` | false | Upload to HuggingFace Hub |

**Recording flow:**
1. Script connects both arms and cameras automatically.
2. Per episode: press Enter to start → move the leader arm → recording stops after `episode_time_s` → reset the scene within `reset_time_s`.
3. Dataset is saved locally after all episodes complete.

**Data quality tips:**
- Record at least **50 episodes** for usable training results.
- Keep start positions consistent across episodes.
- Ensure clear camera views and stable lighting.
- Each episode should contain one complete task execution (reach → grasp → lift → place).

---

## Step 5: Train a Policy

Train an ACT (Action Chunking with Transformers) policy on the collected dataset.

Install training dependencies:

```bash
pip install -e ".[nexarm,training]"
```

Run training:

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/nexarm_pick \
  --policy.type=act \
  --output_dir=outputs/train/nexarm_act \
  --batch_size=32 \
  --steps=100000 \
  --save_freq=25000
```

**Training recommendations:**

| Item | Recommended | Notes |
|------|-------------|-------|
| Hardware | CUDA GPU | RTX 3090 or better preferred |
| Episodes | 50+ | More is better |
| Steps | 100,000 | Adjust based on loss curve |
| Batch size | 32 | Reduce to 16 if VRAM is limited |
| Save frequency | 25,000 | Checkpoint every 25 k steps |

Checkpoints are saved to:

```
outputs/train/nexarm_act/checkpoints/last/pretrained_model/
├── config.json
├── model.safetensors
├── policy_preprocessor.json
├── policy_postprocessor.json
└── train_config.json
```

---

## Step 6: Run Inference

Deploy the trained policy on the real robot. The follower arm executes actions predicted by the model — no leader arm needed.

```bash
python examples/nexarm/rollout.py \
  --follower-port COM19 \
  --policy-path outputs/train/nexarm_act/checkpoints/last/pretrained_model
```

**Notes:**
- No `--teleop` argument needed — the policy replaces the human operator.
- Each run appends a timestamp to `repo_id` automatically to avoid conflicts.
- The run is recorded as a dataset for later analysis.
- With a GPU the policy runs at 30 Hz; on CPU, ACT action chunking (chunk_size=100) keeps throughput at 20–30 Hz because model inference only fires when the action queue empties.

**Rollout strategies:**

| Strategy | Flag | Description |
|----------|------|-------------|
| `base` | `--strategy.type=base` | Inference only, no recording |
| `sentry` | `--strategy.type=sentry` | Continuous recording + auto-save (recommended for evaluation) |
| `highlight` | `--strategy.type=highlight` | Ring buffer, press key to save highlights |
| `dagger` | `--strategy.type=dagger` | Human-robot collaboration, requires leader arm |

---

## Code Architecture

```
src/lerobot/
├── motors/nexarm/
│   ├── __init__.py                  # Exports NexArmMotorsBus
│   └── nexarm.py                    # CommProtocol framing, position read/write, torque, bridge mode
├── robots/nexarm_follower/
│   ├── __init__.py
│   ├── config_nexarm_follower.py    # RobotConfig subclass (port, cameras, baudrate)
│   └── nexarm_follower.py           # Robot subclass (connect, observe, send_action)
└── teleoperators/nexarm_leader/
    ├── __init__.py
    ├── config_nexarm_leader.py      # TeleoperatorConfig subclass
    └── nexarm_leader.py             # Teleoperator subclass (read positions, leader→follower mapping)
```

**Modified upstream files:**

| File | Change |
|------|--------|
| `src/lerobot/robots/utils.py` | Added `nexarm_follower` branch in `make_robot_from_config()` |
| `src/lerobot/teleoperators/utils.py` | Added `nexarm_leader` branch in `make_teleoperator_from_config()` |
| `pyproject.toml` | Added `nexarm` optional dependency group |
| `src/lerobot/cameras/opencv/camera_opencv.py` | Fixed `stop_event` race condition on Linux |
| `src/lerobot/processor/normalize_processor.py` | Added device/dtype caching to avoid redundant `.to()` calls |

---

## Troubleshooting

**Serial port permission denied (Linux)**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Camera not found**
- Run `lerobot-find-cameras opencv` to scan available indices.
- Close other programs using the camera (OBS, browser, etc.).
- Re-scan after unplugging and replugging USB devices.

**Follower arm does not move during teleoperation**
1. Confirm the follower COM port is correct.
2. Confirm the follower ESP32 firmware supports CMD 68 (LeRobot bridge mode).
3. Try power-cycling the follower arm.

**TimeoutError during data collection**
```
TimeoutError: No position reply from NexArm
```
The leader firmware prints debug lines over Serial that corrupt protocol frames. The driver retries 3 times automatically. To eliminate the issue permanently, comment out the `Serial.printf` calls in `Nex_Arm.ino` inside the `lerobotMode == true` branch and reflash the firmware.

**Training loss does not decrease**
- Ensure you have at least 50 episodes.
- Verify camera frames are not black or blurry.
- Try increasing the learning rate: `--policy.optimizer_lr=1e-4`.

**Robot moves hesitantly / small motion amplitude**
The model collapsed to the mean. Try:
- Lower `kl_weight`: `--policy.kl_weight=5.0` or `1.0`
- Increase `batch_size`: `--batch_size=64`
- Record more consistent episodes (same start position, complete task each time)
- Train longer: `--steps=200000`

**Low inference FPS on CPU**
ACT uses action chunking (chunk_size=100) so CPU inference is normally 20–30 Hz. If slower:
- Check no background processes are saturating the CPU.
- Dual-camera capture adds ~45 ms per frame — this is expected.

**Checksum errors**
The leader firmware has a known checksum bug. The driver accepts both correct and incorrect checksums for compatibility.
