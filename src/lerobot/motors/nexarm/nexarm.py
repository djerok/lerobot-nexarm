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

"""NexArm serial motor bus driver.

NexArm uses a custom CommProtocol over USB serial at 1 Mbps.
Frame format: [0xFF][0xFF][ID][LEN][CMD][ARGS...][CHECKSUM]

The master ESP32 directly controls HX-30HM servos (leader arm).
The slave ESP32 forwards commands through an AT32F421 co-processor (follower arm).

Supported commands:
    CMD 68  — Enter/exit LeRobot bridge mode (slave only)
    CMD 96  — Read 6 servo positions (reply: 12 bytes, 6 × int16 LE)
    CMD 97  — Write 6 servo positions (12 bytes, sync-write, no reply)
    CMD 98  — Enable/disable torque on all 6 servos
"""

from __future__ import annotations

import contextlib
import logging
import struct
import threading
import time

import serial

logger = logging.getLogger(__name__)

SYSTEM_ID = 0xFF

CMD_LEROBOT_MODE = 68
CMD_READ_POS = 96
CMD_WRITE_POS = 97
CMD_TORQUE = 98
CMD_SET_MOTION_PARAMS = 56  # sets arm.move_acc only; NOT used by CMD_LR_WRITE_POS

# HX-30HM register layout (used by reg_write path, cmd=3)
REG_ACC = 41  # acceleration (1 byte)
REG_GOAL_POS = 42  # goal position (2 bytes LE)
REG_GOAL_SPEED = 46  # goal speed    (2 bytes LE)

JOINT_COUNT = 6
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

POSITION_MIN = 0
POSITION_MAX = 4095

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT = 0.05
REPLY_TIMEOUT = 0.15


def build_frame(device_id: int, cmd: int, args: bytes = b"") -> bytes:
    length = len(args) + 2
    data_raw = bytes([device_id & 0xFF, length & 0xFF, cmd & 0xFF]) + args
    checksum = (~sum(data_raw)) & 0xFF
    return b"\xff\xff" + data_raw + bytes([checksum])


def parse_frame(data: bytes) -> tuple[int, int, bytes] | None:
    """Parse a CommProtocol frame. Returns (id, cmd, args) or None.

    Accepts both the correct full-range checksum and the buggy short checksum
    from the master firmware (tx_packet_complete uses rx_packet.elements.length
    instead of tx_packet.elements.length).
    """
    if len(data) < 6 or data[0] != 0xFF or data[1] != 0xFF:
        return None
    device_id = data[2]
    length = data[3]
    total = 4 + length
    if len(data) < total:
        return None
    cmd = data[4]
    n = length - 2
    args = data[5 : 5 + n]
    checksum_byte = data[total - 1]
    expected_full = (~sum(data[2 : total - 1])) & 0xFF
    expected_short = (~sum(data[2:5])) & 0xFF
    if checksum_byte != expected_full and checksum_byte != expected_short:
        return None
    return (device_id, cmd, bytes(args))


def map_leader_to_follower(leader_pos: list[int | float]) -> list[int]:
    """Map leader (master) raw positions to follower (slave) positions.

    ID2 (shoulder_lift): mirrored → 4096 - pos
    ID6 (gripper): remapped → 2833 + (pos - 2048) * 4, clamped to [1195, 2833]
    """
    result = []
    for idx, pos in enumerate(leader_pos):
        p = max(POSITION_MIN, min(POSITION_MAX, int(round(pos))))
        if idx == 1:
            p = 4096 - p
        elif idx == 5:
            p = 2833 + (p - 2048) * 4
            p = max(1195, min(2833, p))
        result.append(max(POSITION_MIN, min(POSITION_MAX, p)))
    return result


class NexArmMotorsBus:
    """Serial bus driver for NexArm 6-DOF robot arms.

    Handles USB serial communication at 1 Mbps with CommProtocol framing.
    Thread-safe — all serial I/O goes through a lock.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.is_connected:
            return
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(0.1)
        self._serial.reset_input_buffer()
        logger.info(f"NexArmMotorsBus connected on {self.port}")

    def disconnect(self) -> None:
        if self._serial is not None:
            with contextlib.suppress(Exception):
                self._serial.close()
            self._serial = None
            logger.info(f"NexArmMotorsBus disconnected from {self.port}")

    def _send(
        self,
        frame: bytes,
        expect_reply: bool = True,
        reply_timeout: float = REPLY_TIMEOUT,
    ) -> tuple[int, bytes] | None:
        if self._serial is None:
            raise ConnectionError("Serial port not open")
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(frame)
            if not expect_reply:
                return None
            return self._read_reply(reply_timeout)

    def _read_reply(self, timeout: float) -> tuple[int, bytes] | None:
        assert self._serial is not None
        buf = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            avail = self._serial.in_waiting
            if avail:
                buf.extend(self._serial.read(avail))
                result = self._try_parse(buf)
                if result is not None:
                    return result
            else:
                time.sleep(0.001)
        return None

    def _try_parse(self, buf: bytearray) -> tuple[int, bytes] | None:
        while len(buf) >= 6:
            idx = bytes(buf).find(b"\xff\xff")
            if idx < 0:
                buf.clear()
                return None
            if idx > 0:
                del buf[:idx]
            if len(buf) < 4:
                return None
            length = buf[3]
            total = 4 + length
            if len(buf) < total:
                return None
            parsed = parse_frame(bytes(buf[:total]))
            if parsed is not None:
                del buf[:total]
                return (parsed[1], parsed[2])
            else:
                del buf[:2]
        return None

    def read_positions(self, retries: int = 3) -> list[int]:
        """Read 6 servo positions via CMD 96.

        Retries on timeout to handle firmware debug prints that corrupt the stream.
        """
        for attempt in range(retries):
            frame = build_frame(SYSTEM_ID, CMD_READ_POS)
            result = self._send(frame, reply_timeout=REPLY_TIMEOUT)
            if result is not None:
                cmd, args = result
                if len(args) >= JOINT_COUNT * 2:
                    return [
                        max(POSITION_MIN, min(POSITION_MAX, struct.unpack_from("<h", args, i * 2)[0]))
                        for i in range(JOINT_COUNT)
                    ]
            if attempt < retries - 1:
                time.sleep(0.005)
        raise TimeoutError("No position reply from NexArm")

    def write_positions(self, positions: list[int]) -> None:
        """Write 6 servo positions via CMD 97 (sync write, no reply)."""
        args = b""
        for p in positions:
            args += struct.pack("<h", max(POSITION_MIN, min(POSITION_MAX, int(p))))
        frame = build_frame(SYSTEM_ID, CMD_WRITE_POS, args)
        self._send(frame, expect_reply=False)

    def set_torque(self, enable: bool) -> None:
        """Enable/disable torque on all 6 servos."""
        frame = build_frame(SYSTEM_ID, CMD_TORQUE, bytes([1 if enable else 0]))
        self._send(frame, expect_reply=False)
        time.sleep(0.02)

    def enter_lerobot_mode(self) -> None:
        """Enter LeRobot bridge mode on slave ESP32 (CMD 68)."""
        frame = build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([1]))
        self._send(frame, expect_reply=False)
        time.sleep(0.1)

    def exit_lerobot_mode(self) -> None:
        """Exit LeRobot bridge mode on slave ESP32 (CMD 68)."""
        frame = build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([0]))
        self._send(frame, expect_reply=False)
        time.sleep(0.1)

    def write_motion_params(self, acc: int, speed: int = 0) -> None:
        """Set LeRobot-mode servo acceleration and speed limit (CMD 56).

        acc:   0-254, acceleration applied to every CMD 97 write.
               0 = max acceleration (no ramp), 50 is a smooth default.
        speed: 0-3400 raw units/s. 0 = no speed limit (servo runs at max).
               Stored in AT32 as arm.lerobot_speed and applied each CMD 97.
        """
        acc = max(0, min(254, int(acc)))
        speed = max(0, min(3400, int(speed)))
        args = bytes([acc & 0xFF, speed & 0xFF, (speed >> 8) & 0xFF])
        frame = build_frame(SYSTEM_ID, CMD_SET_MOTION_PARAMS, args)
        self._send(frame, expect_reply=False)
