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

"""NexArm leader (master) teleoperator — LeRobot Teleoperator subclass.

Connects to the master ESP32 via USB serial. Reads servo positions via
CMD 96, maps them from leader space to follower space. The master runs
in torque-off mode so the operator can freely drag the arm.
"""

from __future__ import annotations

import logging
from typing import Any

from lerobot.motors import MotorCalibration
from lerobot.motors.nexarm import NexArmMotorsBus
from lerobot.motors.nexarm.nexarm import (
    JOINT_NAMES,
    POSITION_MAX,
    POSITION_MIN,
    map_leader_to_follower,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_nexarm_leader import NexArmLeaderConfig

logger = logging.getLogger(__name__)


class NexArmLeader(Teleoperator):
    config_class = NexArmLeaderConfig
    name = "nexarm_leader"

    def __init__(self, config: NexArmLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = NexArmMotorsBus(port=config.port, baudrate=config.baudrate)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @property
    def is_calibrated(self) -> bool:
        return len(self.calibration) > 0 or self.calibration_fpath.is_file()

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()

        if calibrate and not self.is_calibrated:
            self.calibrate()
        elif self.calibration_fpath.is_file() and not self.calibration:
            self._load_calibration()

        self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        for i, name in enumerate(JOINT_NAMES):
            self.calibration[name] = MotorCalibration(
                id=i + 1,
                drive_mode=0,
                homing_offset=2048,
                range_min=POSITION_MIN,
                range_max=POSITION_MAX,
            )
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.set_torque(False)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        leader_pos = self.bus.read_positions()
        follower_pos = map_leader_to_follower(leader_pos)
        return {f"{name}.pos": float(follower_pos[i]) for i, name in enumerate(JOINT_NAMES)}

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
