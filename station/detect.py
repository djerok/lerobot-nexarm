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
a person can drag it, the follower is held by its servos. Hence the drag test,
which lives in robots.py now that it serves every kind of arm.
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
# The SO-100 / SO-101's Feetech bus boards sit behind a CH343 instead. The NexArm
# probes here never touch one; robots.py drives them through LeRobot.
CH343 = (0x1A86, 0x55D3)


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


def usb_serial_ports() -> list:
    """Serial ports that are real USB adapters.

    Bluetooth "Standard Serial over Bluetooth link" ports carry no USB VID, and
    opening one that belongs to a paired phone or headset blocks for about 13 s
    before failing with a semaphore timeout (measured on a laptop with two paired
    devices). Probing those is what made the page hang.
    """
    from serial.tools import list_ports

    return [p for p in list_ports.comports() if p.vid is not None]


def is_so101_board(p) -> bool:
    return (p.vid, p.pid) == CH343 or "CH343" in (p.description or "")


def candidate_ports() -> list[str]:
    """Every CH340 serial port. Falls back to other USB serial ports, never to all.

    It used to fall back to every port, Bluetooth included, whenever no CH340 was
    plugged in -- an SO-101 (CH343 boards), or simply before the cables went in --
    and then sat ~13 s on each paired Bluetooth device. That read as the station
    being unresponsive and never connecting.

    macOS lists each USB serial adapter twice, as /dev/tty.* and /dev/cu.*. They
    are the same hardware, but opening the tty side blocks waiting for a carrier
    signal the arm never asserts, so it hangs instead of failing. Only the cu side
    is usable, and a duplicate here would also break the drag test, which needs
    exactly two ports to compare.
    """
    usb = usb_serial_ports()
    found = [
        p.device for p in usb
        if (p.vid, p.pid) == CH340 or "CH340" in (p.description or "")
    ]
    if not found:
        # Never an SO-101 bus: the probe frames go to broadcast ID 0xFF, which
        # every Feetech servo on that bus would receive.
        found = [p.device for p in usb if not is_so101_board(p)]
    if IS_MAC:
        callout = [d for d in found if "/cu." in d]
        if callout:
            found = callout
    return sorted(found)


def port_exists(port: str) -> bool:
    from serial.tools import list_ports

    return any(p.device == port for p in list_ports.comports())


def remap_by_serial(cfg: dict) -> bool:
    """Follow arms to new COM numbers by their USB serial numbers.

    COM numbers move when a cable goes into a different socket; a USB adapter's
    serial number does not. Where the setup recorded one -- the SO boards' CH343
    has one, the NexArm's CH340 does not -- the port is looked up again instead of
    sending a child back to the wave test. Returns True if anything changed.
    """
    serials: dict[str, str | None] = {}
    for p in usb_serial_ports():
        if p.serial_number:
            # A serial shared by two adapters (clone boards) points nowhere.
            serials[p.serial_number] = None if p.serial_number in serials else p.device
    changed = False
    for role in ("leader", "follower"):
        serial = cfg.get(f"{role}_serial")
        port = serials.get(serial) if serial else None
        if port and cfg.get(f"{role}_port") != port:
            cfg[f"{role}_port"] = port
            changed = True
    return changed


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
