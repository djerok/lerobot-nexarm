"""Find the arms and the cameras without asking anyone anything.

nexarm_autoconfig.py already does this, but it does it from a console: it prints
instructions, waits on input() for the camera assignment, and expects whoever ran
it to read the output. None of that survives being driven from a web page, and a
ten-year-old will not read it either.

So the same detection is broken into steps a server can call one at a time and
report progress on, and the one genuinely undecidable question -- which camera is
the wrist -- is guessed here and made fixable with a button instead of a prompt.

The role of each arm cannot be read off the hardware. Both boards enumerate as
the same CH340 with an empty USB serial number and both answer the same protocol
identically, so the role is decided by behaviour: the leader runs torque-off and
a person can drag it, the follower is held by its servos. Hence the drag test.
"""

from __future__ import annotations

import json
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "nexarm.json"
SHOTS_DIR = ROOT / "station" / "shots"

IS_WINDOWS = sys.platform.startswith("win")
IS_MAC = sys.platform == "darwin"


def camera_backend():
    """OpenCV capture backend for this OS.

    DirectShow is the one that works on Windows; passing it on macOS makes every
    index fail to open, which looks exactly like "no cameras are plugged in".
    AVFoundation is the macOS equivalent. Anywhere else, let OpenCV choose.
    """
    import cv2

    if IS_WINDOWS:
        return cv2.CAP_DSHOW
    if IS_MAC:
        return getattr(cv2, "CAP_AVFOUNDATION", cv2.CAP_ANY)
    return cv2.CAP_ANY


def open_capture(index: int):
    import cv2

    return cv2.VideoCapture(int(index), camera_backend())

SYSTEM_ID = 0xFF
CMD_LEROBOT_MODE = 68
CMD_READ_POS = 96
JOINT_COUNT = 6
BAUD = 1_000_000

# USB VID:PID of the CH340 bridge both NexArm ESP32 boards sit behind.
CH340 = (0x1A86, 0x7523)

# A real drag swings a joint by hundreds of counts; sensor noise is single digits.
# Both a floor and a clear gap are required before a result is trusted.
DRAG_FLOOR = 100
DRAG_RATIO = 3


def build_frame(device_id: int, cmd: int, args: bytes = b"") -> bytes:
    length = len(args) + 2
    data_raw = bytes([device_id & 0xFF, length & 0xFF, cmd & 0xFF]) + args
    checksum = (~sum(data_raw)) & 0xFF
    return b"\xff\xff" + data_raw + bytes([checksum])


def read_positions(ser, timeout: float = 0.25):
    """Send CMD 96 and parse the 6 int16 reply. None if no valid frame arrives."""
    ser.reset_input_buffer()
    ser.write(build_frame(SYSTEM_ID, CMD_READ_POS))
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
            idx = bytes(buf).find(b"\xff\xff")
            if idx >= 0 and len(buf) - idx >= 4:
                length = buf[idx + 3]
                total = idx + 4 + length
                if len(buf) >= total:
                    args = bytes(buf[idx + 5 : total - 1])
                    if len(args) >= JOINT_COUNT * 2:
                        return [
                            struct.unpack_from("<h", args, i * 2)[0]
                            for i in range(JOINT_COUNT)
                        ]
        else:
            time.sleep(0.001)
    return None


def candidate_ports() -> list[str]:
    """Every CH340 serial port. Falls back to all ports rather than refusing.

    macOS lists each USB serial adapter twice, as /dev/tty.* and /dev/cu.*. They
    are the same hardware, but opening the tty side blocks waiting for a carrier
    signal the arm never asserts, so it hangs instead of failing. Only the cu side
    is usable, and a duplicate here would also break the drag test, which needs
    exactly two ports to compare.
    """
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    found = [
        p.device for p in ports
        if (p.vid, p.pid) == CH340 or "CH340" in (p.description or "")
    ]
    if not found:
        found = [p.device for p in ports]
    if IS_MAC:
        callout = [d for d in found if "/cu." in d]
        if callout:
            found = callout
    return sorted(found)


def port_exists(port: str) -> bool:
    from serial.tools import list_ports

    return any(p.device == port for p in list_ports.comports())


class ArmSession:
    """Holds both serial ports open across the drag test, then puts them back.

    The drag test needs the ports open for as long as a person is waving an arm,
    which is many HTTP requests. Opening per request would fight the exclusive
    lock Windows puts on a COM port, so the session is kept and closed on exit.
    """

    def __init__(self):
        self.arms: dict[str, object] = {}
        self.baseline: dict[str, list[int] | None] = {}
        self.moved: dict[str, int] = {}

    def open_all(self, log=print) -> list[str]:
        import serial

        ports = candidate_ports()
        if not ports:
            log("No USB serial ports at all. Plug the arms in and switch them on.")
            return []

        log(f"Checking {len(ports)} serial port(s) at {BAUD} baud.")
        for port in ports:
            try:
                ser = serial.Serial(port, BAUD, timeout=0.3)
            except Exception as exc:
                log(f"  {port}: cannot open ({exc})")
                continue
            time.sleep(0.25)
            # CMD 68 puts the slave board into bridge mode. Harmless on the
            # master, and it is what makes a follower report positions at all.
            ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([1])))
            time.sleep(0.25)
            if read_positions(ser) is None:
                log(f"  {port}: no reply, not a NexArm board")
                ser.close()
                continue
            log(f"  {port}: NexArm board answering")
            self.arms[port] = ser

        for port, ser in self.arms.items():
            self.baseline[port] = read_positions(ser)
            self.moved[port] = 0
        return list(self.arms)

    def sample(self) -> dict[str, int]:
        """One pass over both arms, updating the largest movement seen so far."""
        for port, ser in self.arms.items():
            pos = read_positions(ser)
            base = self.baseline.get(port)
            if pos and base:
                delta = max(abs(a - b) for a, b in zip(pos, base))
                if delta > self.moved[port]:
                    self.moved[port] = delta
        return dict(self.moved)

    def verdict(self) -> tuple[str | None, str | None]:
        if len(self.moved) < 2:
            return None, None
        ranked = sorted(self.moved, key=lambda p: self.moved[p], reverse=True)
        top, rest = ranked[0], ranked[1]
        if self.moved[top] > DRAG_FLOOR and self.moved[top] > self.moved[rest] * DRAG_RATIO:
            return top, rest
        return None, None

    def close(self):
        for ser in self.arms.values():
            try:
                # Leave the slave board the way it was found.
                ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([0])))
                time.sleep(0.05)
                ser.close()
            except Exception:
                pass
        self.arms.clear()


def probe_cameras(limit: int = 6, log=print) -> list[int]:
    """Open each OpenCV index, keep the ones that hand back a frame, save a still."""
    import cv2

    SHOTS_DIR.mkdir(parents=True, exist_ok=True)
    working = []
    for idx in range(limit):
        cap = open_capture(idx)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                cv2.imwrite(str(SHOTS_DIR / f"camera_{idx}.jpg"), frame)
                h, w = frame.shape[:2]
                working.append(idx)
                log(f"  camera {idx}: {w}x{h}")
        cap.release()
    if not working:
        log("  no cameras found -- the arms still work, there is just no video.")
    return working


def guess_cameras(working: list[int]) -> dict[str, int]:
    """Assign front/wrist without asking.

    There is no way to tell from a USB camera which one is bolted to the gripper,
    so this is a guess, and the UI carries a Swap button for when it is wrong.
    Lowest index goes to front because the built-in laptop webcam usually takes
    index 0 and is usually the one pointing at the scene.
    """
    if not working:
        return {}
    if len(working) == 1:
        return {"front": working[0]}
    return {"front": working[0], "wrist": working[1]}


def load_config() -> dict | None:
    if not CONFIG_PATH.exists():
        return None
    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        return None


def save_config(cfg: dict) -> None:
    cfg = dict(cfg)
    cfg["detected_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


def config_is_usable(cfg: dict | None) -> tuple[bool, str]:
    """Is a saved config still true? COM numbers move when a cable moves."""
    if not cfg:
        return False, "no saved setup yet"
    leader, follower = cfg.get("leader_port"), cfg.get("follower_port")
    if not leader or not follower:
        return False, "saved setup has no arm ports"
    if leader == follower:
        return False, "saved setup names the same port twice"
    missing = [p for p in (leader, follower) if not port_exists(p)]
    if missing:
        return False, f"{', '.join(missing)} is gone -- a cable moved"
    return True, "saved setup still matches what is plugged in"
