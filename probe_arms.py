#!/usr/bin/env python
"""Identify which NexArm COM port is the leader and which is the follower.

The NexArm protocol has no device-identity query, so this uses behaviour:
the leader (master ESP32) runs torque-off and is draggable by hand, the
follower holds position. Read-only: only CMD 96 (read positions) is sent.

Usage:
    python.exe probe_arms.py COM11 COM12
Then move ONE arm by hand while it runs.
"""

import struct
import sys
import time

import serial

SYSTEM_ID = 0xFF
CMD_READ_POS = 96
JOINT_COUNT = 6
BAUD = 1_000_000


def build_frame(device_id: int, cmd: int, args: bytes = b"") -> bytes:
    length = len(args) + 2
    data_raw = bytes([device_id & 0xFF, length & 0xFF, cmd & 0xFF]) + args
    checksum = (~sum(data_raw)) & 0xFF
    return b"\xff\xff" + data_raw + bytes([checksum])


def read_positions(ser: serial.Serial, timeout: float = 0.2):
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


def main():
    ports = sys.argv[1:] or ["COM11", "COM12"]
    sers = {}
    for p in ports:
        try:
            sers[p] = serial.Serial(p, BAUD, timeout=0.2)
        except Exception as e:
            print(f"{p}: could not open ({e})")
    time.sleep(0.3)

    baseline, movement = {}, {}
    for p, s in sers.items():
        pos = read_positions(s)
        print(f"{p}: {pos if pos else 'NO REPLY'}")
        baseline[p] = pos
        movement[p] = 0

    print("\nMove ONE arm by hand now. Watching for 12 seconds...\n")
    end = time.monotonic() + 12
    while time.monotonic() < end:
        for p, s in sers.items():
            pos = read_positions(s)
            if pos and baseline[p]:
                movement[p] = max(
                    movement[p], max(abs(a - b) for a, b in zip(pos, baseline[p]))
                )
        time.sleep(0.05)

    print("max movement seen:")
    for p in sers:
        print(f"  {p}: {movement[p]}")

    moved = [p for p in sers if movement[p] > 40]
    if len(moved) == 1:
        leader = moved[0]
        follower = [p for p in sers if p != leader][0]
        print(f"\n==> LEADER   = {leader}  (it moved, so torque is off)")
        print(f"==> FOLLOWER = {follower}")
        print(
            f"\npython.exe examples/nexarm/teleoperate.py "
            f"--leader-port {leader} --follower-port {follower}"
        )
    elif not moved:
        print("\nNothing moved. Move the arm further, or check it is powered.")
    else:
        print("\nBoth moved — only drag one arm at a time.")

    for s in sers.values():
        s.close()


if __name__ == "__main__":
    main()
