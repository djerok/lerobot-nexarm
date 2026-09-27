# 幻尔 NexArm LeRobot VLA 开源六轴机械臂

[English](./README.md) | 中文

[NexArm](https://www.hiwonder.com/products/nexarm6-axis) 是一款面向具身 AI 研究的开源六轴机械臂，基于 [🤗 LeRobot](https://github.com/huggingface/lerobot) 原生设计，适合快速验证模仿学习和强化学习策略。双芯片架构（ESP32 + AT32）实现毫秒级主从同步遥操作，采集的示教数据可直接输入 LeRobot 训练流程。

全金属机身，搭载 65 kg·cm 磁编码舵机，重复定位精度 ±2 mm，运动平滑无抖动。结合逆运动学和曲线平滑算法，可原生计算复杂轨迹，最大程度减少启停振动。

板载 6 TOPS K230 视觉模块，支持在无 PC 的情况下运行多模态大模型和计算机视觉流程（YOLO 目标跟踪、手眼协调抓取等）。所有原理图、固件和代码完全开源。

<p align="center">
  <img src="./media/readme/VLA_architecture.jpg" alt="nexarm" width="600"/>
</p>

## 与 SO-ARM101 对比

| 项目 | NexArm | SO-ARM101 |
|------|--------|-----------|
| 负载 | 500 g | 200 g |
| 重复定位精度 | ±2 mm | ±3 mm |
| 工作空间 | 0.5 m | 0.4 m |
| 关节舵机 | 双输出轴磁编码总线舵机 | 单输出轴磁编码总线舵机（固定轴） |
| 机身材质 | 航空级金属结构 | PLA 3D 打印结构件 |
| 末端执行器 | 平行导轨夹爪 | 3D 打印竖开式夹爪 |
| 定位 | 进阶端到端开发 / 高阶具身 AI 应用 | 入门级端到端开发 / 基础具身 AI 应用 |

---

## 目录

- [硬件概述](#硬件概述)
- [安装](#安装)
- [第 1 步：查找串口](#第-1-步查找串口)
- [第 2 步：查找摄像头](#第-2-步查找摄像头)
- [第 3 步：遥操作测试](#第-3-步遥操作测试)
- [第 4 步：采集数据集](#第-4-步采集数据集)
- [第 5 步：训练策略](#第-5-步训练策略)
- [第 6 步：推理部署](#第-6-步推理部署)
- [代码架构](#代码架构)
- [故障排查](#故障排查)

---

## 硬件概述

### 组件

| 组件 | 说明 |
|------|------|
| **主臂（Leader）** | ESP32 驱动 6 × HX-30HM 舵机，遥操作时操作员手持该臂自由拖动 |
| **从臂（Follower）** | ESP32 + AT32F421 协处理器，驱动 6 × HX-30HM 舵机，镜像主臂或执行策略输出 |
| **舵机** | HX-30HM 串行总线舵机，12 位分辨率（0–4095），1 Mbps |
| **摄像头** | 2 × USB 摄像头：`front`（俯视工作区）和 `wrist`（末端夹爪特写），640×480 @ 30 FPS |

### 关节布局（6 自由度）

| 关节 | 名称 | 说明 |
|------|------|------|
| 1 | `shoulder_pan` | 底座旋转 |
| 2 | `shoulder_lift` | 主从臂方向相反（4096 − pos） |
| 3 | `elbow_flex` | 肘部 |
| 4 | `wrist_flex` | 腕部俯仰 |
| 5 | `wrist_roll` | 腕部旋转 |
| 6 | `gripper` | 开合，映射范围 [1195, 2833] |

### 通信协议

NexArm 通过 USB 串口使用自定义 CommProtocol：

```
帧格式: [0xFF][0xFF][ID][LEN][CMD][ARGS...][CHECKSUM]
```

| CMD | 功能 | 方向 |
|-----|------|------|
| 56 | 设置运动速度和加速度 | 上位机 → 从臂 |
| 68 | 进入/退出 LeRobot 桥接模式（仅从臂） | 上位机 → 从臂 |
| 96 | 读取 6 个舵机位置（12 字节回复） | 上位机 → 设备 → 上位机 |
| 97 | 写入 6 个舵机位置（12 字节，无回复） | 上位机 → 设备 |
| 98 | 使能/失能扭矩 | 上位机 → 设备 |

---

## 安装

### 方式 A — conda（推荐）

```bash
git clone https://github.com/Hiwonder-official/lerobot-nexarm.git
cd lerobot-nexarm

conda create -n nexarm python=3.12 -y
conda activate nexarm

pip install -e ".[nexarm]"
```

开启 Rerun 实时可视化：

```bash
pip install -e ".[nexarm,viz]"
```

### 方式 B — venv

```bash
git clone https://github.com/Hiwonder-official/lerobot-nexarm.git
cd lerobot-nexarm

python -m venv .venv

# 激活虚拟环境
# Windows:
.venv\Scripts\activate
# Linux / macOS:
source .venv/bin/activate

pip install -e ".[nexarm]"
```

### 验证安装

```bash
python -c "from lerobot.robots.nexarm_follower import NexArmFollower; print('OK')"
```

### 连接硬件

1. 将**从臂** ESP32 通过 USB 连接到电脑
2. 将**主臂** ESP32 通过 USB 连接到电脑
3. 插入两个 USB 摄像头（front + wrist）

### 平台支持

| 平台 | 状态 | 说明 |
|------|------|------|
| Windows 10/11 | 已验证 | 安装 CH340 驱动；端口格式 `COM19` |
| Ubuntu 20.04+ | 已验证 | 端口格式 `/dev/ttyUSB0`；将用户加入 `dialout` 组 |
| macOS | 已验证 | 端口格式 `/dev/tty.usbserial-xxx` |

---

## 第 1 步：查找串口

确认哪个端口对应主臂，哪个对应从臂。

```bash
python -m lerobot.scripts.lerobot_find_port
```

Windows 典型输出：

| 端口 | 设备 |
|------|------|
| COM18 | 主臂 ESP32 |
| COM19 | 从臂 ESP32 |

Linux 通常为 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`。

> 提示：如不确定哪个端口对应哪条臂，可以每次只插一条臂来逐一确认。

---

## 第 2 步：查找摄像头

确认哪个摄像头索引对应 `front`，哪个对应 `wrist`。

```bash
python -m lerobot.scripts.lerobot_find_cameras opencv
```

或手动扫描并保存图片对比：

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

- **front**：俯视整个工作区
- **wrist**：末端夹爪特写

> 注意：拔插 USB 设备后摄像头索引可能变化，重新连接后需重新扫描。

---

## 第 3 步：遥操作测试

验证主从联动。主臂无扭矩，操作员可自由拖动；从臂实时镜像每个关节。

```bash
python examples/nexarm/teleoperate.py \
  --follower-port COM19 \
  --leader-port COM18
```

或指定摄像头和帧率：

```bash
python examples/nexarm/teleoperate.py \
  --follower-port COM19 --leader-port COM18 \
  --front-cam 0 --wrist-cam 1 --fps 30
```

### 运动速度与加速度

从臂运动参数在 `NexArmFollowerConfig` 中配置：

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `motion_speed` | `2000` | 0–3400 | 最大舵机速度（原始单位/秒），0 = 不限 |
| `motion_acc` | `100` | 0–254 | 加速度斜率，0 = 最大加速，越大越平滑 |

**检查要点：**
- 从臂跟随主臂所有关节平滑运动
- `shoulder_lift` 方向自动镜像
- 夹爪开合映射正确

---

## 第 4 步：采集数据集

通过遥操作录制示教片段，用于模仿学习。

```bash
python examples/nexarm/record.py \
  --follower-port COM19 --leader-port COM18 \
  --repo-id YOUR_HF_USERNAME/nexarm_pick \
  --task "拿起红色积木" \
  --num-episodes 50 --episode-time 10 --reset-time 10
```

**关键参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--repo-id` | — | 数据集名称（本地保存） |
| `--num-episodes` | 50 | 录制片段数 |
| `--episode-time` | 10 | 每段时长（秒） |
| `--reset-time` | 10 | 片段间重置时间（秒） |
| `--fps` | 30 | 录制帧率 |
| `--push-to-hub` | false | 上传到 HuggingFace Hub |

**录制流程：**
1. 脚本自动连接两臂和摄像头
2. 每段：按 Enter 开始 → 拖动主臂 → 达到 `episode-time` 后自动停止 → 在 `reset-time` 内重置场景
3. 全部片段录制完成后自动保存数据集

**数据质量建议：**
- 至少录制 **50 段**才能得到有效训练结果
- 每段起始姿态保持一致
- 确保摄像头视野清晰、光线稳定
- 每段包含一次完整任务（接近 → 抓取 → 提起 → 放置）

---

## 第 5 步：训练策略

在采集的数据集上训练 ACT（基于 Transformer 的动作分块）策略。

安装训练依赖：

```bash
pip install -e ".[nexarm,training]"
```

运行训练：

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/nexarm_pick \
  --policy.type=act \
  --output_dir=outputs/train/nexarm_act \
  --batch_size=32 \
  --steps=100000 \
  --save_freq=25000
```

**训练建议：**

| 项目 | 推荐值 | 说明 |
|------|--------|------|
| 硬件 | CUDA GPU | RTX 3090 或更好 |
| 片段数 | 50+ | 越多越好 |
| 步数 | 100,000 | 根据 loss 曲线调整 |
| 批大小 | 32 | VRAM 不足时可降至 16 |
| 保存频率 | 25,000 | 每 25k 步保存一个检查点 |

检查点保存路径：

```
outputs/train/nexarm_act/checkpoints/last/pretrained_model/
├── config.json
├── model.safetensors
├── policy_preprocessor.json
├── policy_postprocessor.json
└── train_config.json
```

---

## 第 6 步：推理部署

将训练好的策略部署到真实机器人上。从臂执行模型预测的动作，无需主臂。

```bash
python examples/nexarm/rollout.py \
  --follower-port COM19 \
  --policy-path outputs/train/nexarm_act/checkpoints/last/pretrained_model
```

使用 HuggingFace Hub 上的策略：

```bash
python examples/nexarm/rollout.py \
  --follower-port COM19 \
  --policy-path YOUR_HF_USERNAME/nexarm_act
```

**说明：**
- 不需要 `--leader-port`，策略替代人工操作
- 有 GPU 时策略以 30 Hz 运行；CPU 上 ACT 动作分块（chunk_size=100）可维持 20–30 Hz
- 加 `--repo-id YOUR_HF_USERNAME/eval_nexarm_pick` 可将本次推理保存为数据集

---

## 代码架构

```
src/lerobot/
├── motors/nexarm/
│   ├── __init__.py                  # 导出 NexArmMotorsBus
│   └── nexarm.py                    # CommProtocol 帧封装、位置读写、扭矩、桥接模式
├── robots/nexarm_follower/
│   ├── __init__.py
│   ├── config_nexarm_follower.py    # RobotConfig 子类（port、cameras、baudrate）
│   └── nexarm_follower.py           # Robot 子类（connect、observe、send_action）
└── teleoperators/nexarm_leader/
    ├── __init__.py
    ├── config_nexarm_leader.py      # TeleoperatorConfig 子类
    └── nexarm_leader.py             # Teleoperator 子类（读取位置、主从关节映射）
```

**修改的上游文件：**

| 文件 | 修改内容 |
|------|----------|
| `src/lerobot/robots/utils.py` | 在 `make_robot_from_config()` 中添加 `nexarm_follower` 分支 |
| `src/lerobot/teleoperators/utils.py` | 在 `make_teleoperator_from_config()` 中添加 `nexarm_leader` 分支 |
| `pyproject.toml` | 添加 `nexarm` 可选依赖组 |
| `src/lerobot/cameras/opencv/camera_opencv.py` | 修复 Linux 上 `stop_event` 竞态条件 |
| `src/lerobot/processor/normalize_processor.py` | 添加 device/dtype 缓存，避免重复 `.to()` 调用 |

---

## 故障排查

**Linux 串口权限被拒绝**
```bash
sudo usermod -a -G dialout $USER
# 重新登录生效
```

**找不到摄像头**
- 运行 `lerobot-find-cameras opencv` 扫描可用索引
- 关闭其他使用摄像头的程序（OBS、浏览器等）
- 拔插 USB 设备后重新扫描

**遥操作时从臂不动**
1. 确认从臂 COM 端口正确
2. 确认从臂 ESP32 固件支持 CMD 68（LeRobot 桥接模式）
3. 尝试重新给从臂断电重启

**采集数据时 TimeoutError**
```
TimeoutError: No position reply from NexArm
```
主臂固件在 `lerobotMode == true` 分支内有 `Serial.printf` 调试输出，会干扰协议帧。驱动已自动重试 3 次。彻底解决需注释掉 `Nex_Arm.ino` 中对应的 `Serial.printf` 并重新烧录固件。

**训练 loss 不下降**
- 确保至少有 50 段数据
- 检查摄像头画面是否黑屏或模糊
- 尝试调高学习率：`--policy.optimizer_lr=1e-4`

**机器人动作迟疑 / 幅度很小**
模型坍缩到均值。尝试：
- 降低 `kl_weight`：`--policy.kl_weight=5.0` 或 `1.0`
- 增大 `batch_size`：`--batch_size=64`
- 录制更一致的示教数据（相同起始姿态，每段完整执行任务）
- 延长训练：`--steps=200000`

**CPU 推理 FPS 过低**
ACT 使用动作分块（chunk_size=100），CPU 推理正常在 20–30 Hz。如更慢：
- 检查是否有后台进程占满 CPU
- 双摄像头采集每帧约增加 45 ms，属正常现象

**校验和错误**
主臂固件存在已知校验和 bug，驱动兼容性处理了正确和错误的校验和，无需额外操作。
