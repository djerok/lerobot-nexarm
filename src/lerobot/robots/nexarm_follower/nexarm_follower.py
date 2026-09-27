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

"""NexArm follower (slave) robot — LeRobot Robot subclass.

Connects to the slave ESP32 via USB serial. Uses CMD 68 to enter bridge
mode, then CMD 96/97/98 through the AT32 co-processor to read/write
6-DOF servo positions.
"""

from __future__ import annotations

import contextlib
import logging
import time
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import MotorCalibration
from lerobot.motors.nexarm import NexArmMotorsBus
from lerobot.motors.nexarm.nexarm import JOINT_NAMES, POSITION_MAX, POSITION_MIN
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_nexarm_follower import NexArmFollowerConfig

logger = logging.getLogger(__name__)


class NexArmFollower(Robot):
    """
    NexArm 6-DOF desktop robot arm (follower/slave).

    The follower connects to a slave ESP32 via USB serial at 1 Mbps. The ESP32
    forwards commands through an AT32F421 co-processor to HX-30HM serial bus
    servos. CMD 68 enters LeRobot bridge mode for direct servo access via
    CMD 96 (read positions) and CMD 97 (write positions).
    """

    config_class = NexArmFollowerConfig
    name = "nexarm_follower"

    def __init__(self, config: NexArmFollowerConfig):
        super().__init__(config)
        self.config = config
        self.bus = NexArmMotorsBus(port=config.port, baudrate=config.baudrate)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return len(self.calibration) > 0 or self.calibration_fpath.is_file()

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.bus.enter_lerobot_mode()

        if calibrate and not self.is_calibrated:
            self.calibrate()
        elif self.calibration_fpath.is_file() and not self.calibration:
            self._load_calibration()

        for cam in self.cameras.values():
            cam.connect()

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
        self.bus.write_motion_params(acc=self.config.motion_acc, speed=self.config.motion_speed)
        self.bus.set_torque(True)

    # Velocity watchdog: warn when a joint jumps more than this per tick.
    _MAX_POS_DELTA = 300

    # Positions suspiciously close to 0 or 4095 are treated as corrupt reads.
    # The firmware occasionally returns these on a dropped/corrupted packet.
    _CORRUPT_MARGIN = 5

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        raw = self.bus.read_positions()
        dt_ms = (time.perf_counter() - start) * 1e3

        prev = getattr(self, "_prev_positions", None)

        # Sanitize: replace corrupt reads (≤margin or ≥4095-margin) with prev value.
        positions = list(raw)
        for i, name in enumerate(JOINT_NAMES):
            p = raw[i]
            is_corrupt = p <= self._CORRUPT_MARGIN or p >= POSITION_MAX - self._CORRUPT_MARGIN
            if is_corrupt and prev is not None:
                logger.warning(
                    "servo %s corrupt read %d — using previous value %d",
                    name,
                    p,
                    prev[i],
                )
                positions[i] = prev[i]
            elif prev is not None:
                delta = abs(p - prev[i])
                if delta > self._MAX_POS_DELTA:
                    logger.warning(
                        "servo %s jumped %d raw units (prev=%d now=%d) - possible slip or lost packet",
                        name,
                        delta,
                        prev[i],
                        p,
                    )

        logger.debug(
            "servo_pos [%s]  read=%.1fms",
            " ".join(f"{name}={positions[i]}" for i, name in enumerate(JOINT_NAMES)),
            dt_ms,
        )

        self._prev_positions = positions
        obs_dict: RobotObservation = {
            f"{name}.pos": float(positions[i]) for i, name in enumerate(JOINT_NAMES)
        }

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Command arm to move to a target joint configuration.

        Args:
            action (RobotAction): The goal positions for the motors.

        Returns:
            RobotAction: The action sent to the motors.
        """
        goal = []
        for name in JOINT_NAMES:
            val = float(action[f"{name}.pos"])
            clamped = max(POSITION_MIN, min(POSITION_MAX, int(round(val))))
            goal.append(clamped)
        self.bus.write_positions(goal)
        return {f"{name}.pos": float(goal[i]) for i, name in enumerate(JOINT_NAMES)}

    @check_if_not_connected
    def disconnect(self) -> None:
        for cam in self.cameras.values():
            with contextlib.suppress(Exception):
                cam.disconnect()

        if self.config.disable_torque_on_disconnect:
            with contextlib.suppress(Exception):
                # Hold current position briefly before killing torque.
                # Without this the servos lose power mid-air and drop.
                try:
                    positions = self.bus.read_positions()
                    self.bus.write_positions(positions)
                    time.sleep(0.4)
                except Exception:
                    pass
                self.bus.set_torque(False)

        with contextlib.suppress(Exception):
            self.bus.exit_lerobot_mode()

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
