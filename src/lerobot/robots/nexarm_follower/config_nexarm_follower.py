# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("nexarm_follower")
@dataclass
class NexArmFollowerConfig(RobotConfig):
    # Serial port for the slave ESP32 (e.g. "COM19", "/dev/ttyUSB1")
    port: str

    # Serial baud rate — NexArm uses 1 Mbps
    baudrate: int = 1_000_000

    disable_torque_on_disconnect: bool = True

    # Motion parameters sent via CMD 56 after connect.
    # acc: 0–254. 0 = max (no ramp), higher = softer ramp.
    # motion_speed: 0–3400 raw units/s. 0 = no limit.
    motion_acc: int = 100
    motion_speed: int = 2000

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
