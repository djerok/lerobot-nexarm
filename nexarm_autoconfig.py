#!/usr/bin/env python
"""Detect the robot and write nexarm.json so nothing needs flags.

Finds every USB serial port, works out which arm is the leader and which is
the follower by talking to them, finds the cameras, then saves the answers to
nexarm.json next to this file. Run it again any time a cable moves.

    python nexarm_autoconfig.py
    python nexarm_autoconfig.py --cameras front=0,wrist=1   # skip camera prompt
    python nexarm_autoconfig.py --no-cameras                # arms only

Leader/follower cannot be read off the hardware. Both boards enumerate as the
same CH340 with an empty USB serial number, and both answer the same protocol
identically, so there is nothing to query. The role is decided by behaviour
instead: the leader runs torque-off and a person can drag it, the follower is
held by its servos. The script asks you to move one arm and watches.
"""

import argparse
import json
import struct
import sys
import time
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent / "nexarm.json"

SYSTEM_ID = 0xFF
CMD_LEROBOT_MODE = 68
CMD_READ_POS = 96
JOINT_COUNT = 6
BAUD = 1_000_000

# USB VID:PID of the CH340 bridge both NexArm ESP32 boards sit behind.
CH340 = (0x1A86, 0x7523)


def build_frame(device_id, cmd, args=b""):
    length = len(args) + 2
    data_raw = bytes([device_id & 0xFF, length & 0xFF, cmd & 0xFF]) + args
    checksum = (~sum(data_raw)) & 0xFF
    return b"\xff\xff" + data_raw + bytes([checksum])


def read_positions(ser, timeout=0.25):
    """Send CMD 96 and parse the 6 int16 reply. None if no valid frame."""
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


def candidate_ports():
    """Every CH340 serial port, newest Windows naming included."""
    from serial.tools import list_ports

    found = []
    for p in list_ports.comports():
        if (p.vid, p.pid) == CH340 or "CH340" in (p.description or ""):
            found.append(p.device)
    if not found:
        # Fall back to everything rather than refusing to try.
        found = [p.device for p in list_ports.comports()]
    return sorted(found)


def open_arm(port):
    """Open a port and confirm a NexArm board is answering on it."""
    import serial

    try:
        ser = serial.Serial(port, BAUD, timeout=0.3)
    except Exception as exc:
        print(f"  {port}: cannot open ({exc})")
        return None

    time.sleep(0.25)
    # CMD 68 puts the slave board into bridge mode. Harmless on the master,
    # and it is what makes a follower start reporting positions.
    ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([1])))
    time.sleep(0.25)

    if read_positions(ser) is None:
        print(f"  {port}: no position reply, not a NexArm board")
        ser.close()
        return None

    print(f"  {port}: NexArm board, responding")
    return ser


def drag_test(arms, seconds=15):
    """Work out which arm is the leader by which one a person can move.

    Both boards speak the same protocol and neither reports its own role, so
    identity comes from behaviour: the leader runs with torque disabled and
    can be dragged by hand, the follower is held in place by its servos.
    """
    baseline = {p: read_positions(s) for p, s in arms.items()}
    moved = {p: 0 for p in arms}

    print("\n" + "-" * 56)
    print(" Take hold of the LEADER arm, the one you move by hand,")
    print(f" and wave it around for {seconds} seconds. Starting now.")
    print("-" * 56)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for port, ser in arms.items():
            pos = read_positions(ser)
            if pos and baseline[port]:
                delta = max(abs(a - b) for a, b in zip(pos, baseline[port]))
                if delta > moved[port]:
                    moved[port] = delta
        time.sleep(0.05)

    for port, amount in moved.items():
        print(f"  {port}: moved {amount}")

    ranked = sorted(moved, key=lambda p: moved[p], reverse=True)
    top, rest = ranked[0], ranked[1]
    # A real drag swings a joint by hundreds of units. Sensor noise is single
    # digits, so require both a floor and a clear gap before trusting it.
    if moved[top] > 100 and moved[top] > moved[rest] * 3:
        return top, rest
    return None, None


def detect_arms():
    ports = candidate_ports()
    if not ports:
        print("No USB serial ports found. Plug the arms in and power them on.")
        return None, None

    print(f"Checking {len(ports)} serial port(s) at {BAUD} baud:")
    arms = {}
    for port in ports:
        ser = open_arm(port)
        if ser:
            arms[port] = ser

    try:
        if len(arms) < 2:
            return None, None
        leader, follower = drag_test(arms)
        return leader, follower
    finally:
        for ser in arms.values():
            # Leave the slave board as it was found.
            ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([0])))
            time.sleep(0.05)
            ser.close()


def probe_cameras(limit=6):
    """Open each OpenCV index and keep the ones that hand back a frame."""
    import cv2

    working = []
    shots = Path(__file__).resolve().parent / "camera_check"
    shots.mkdir(exist_ok=True)
    for idx in range(limit):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                out = shots / f"camera_{idx}.jpg"
                cv2.imwrite(str(out), frame)
                h, w = frame.shape[:2]
                working.append(idx)
                print(f"  index {idx}: {w}x{h}, saved {out.name}")
        cap.release()
    return working, shots


def choose_cameras(working, shots):
    if not working:
        print("  no cameras found.")
        return {}
    if len(working) == 1:
        print(f"  one camera, using index {working[0]} as 'front'.")
        return {"front": working[0]}

    print(f"\nOpen {shots} and look at the pictures.")
    print("The wrist camera is the one attached to the gripper.")

    def ask(name, default):
        raw = input(f"  which index is the {name} camera? {working} [{default}]: ")
        raw = raw.strip()
        return int(raw) if raw else default

    try:
        wrist = ask("WRIST", working[1])
        front = ask("FRONT", next(i for i in working if i != wrist))
    except (EOFError, ValueError):
        front, wrist = working[0], working[1]
        print(f"  no answer, defaulting front={front} wrist={wrist}")
    return {"front": front, "wrist": wrist}


def main():
    parser = argparse.ArgumentParser(description="Autoconfigure NexArm")
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument("--cameras", help="skip the prompt, e.g. front=0,wrist=1")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    print("=" * 56)
    print(" NexArm autoconfig")
    print("=" * 56)

    leader, follower = detect_arms()
    if not leader or not follower:
        print("\nCould not tell the arms apart.")
        print("  - both USB cables in, both arms powered on")
        print("  - close anything else using the ports (Arduino IDE, a terminal)")
        print("  - move only ONE arm, and move it a long way")
        print("  - if the leader will not budge, its torque is still on:")
        print("    power cycle it and run this again")
        return 1

    cameras = {}
    if args.cameras:
        for part in args.cameras.split(","):
            name, _, value = part.partition("=")
            cameras[name.strip()] = int(value)
    elif not args.no_cameras:
        print("\nLooking for cameras:")
        working, shots = probe_cameras()
        cameras = choose_cameras(working, shots)

    config = {
        "robot_type": "nexarm",
        "leader_port": leader,
        "follower_port": follower,
        "baudrate": BAUD,
        "fps": args.fps,
        "cameras": cameras,
        "detected_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    CONFIG_PATH.write_text(json.dumps(config, indent=2), encoding="utf-8")

    print("\n" + "=" * 56)
    print(f" leader   {leader}")
    print(f" follower {follower}")
    for name, idx in cameras.items():
        print(f" {name:<8} camera index {idx}")
    print(f"\n saved to {CONFIG_PATH.name}")
    print("=" * 56)
    print("\nNow run:")
    print("  python nexarm.py teleop")
    print('  python nexarm.py record --task "Pick up the block"')
    return 0


if __name__ == "__main__":
    sys.exit(main())
