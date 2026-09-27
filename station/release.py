#!/usr/bin/env python
"""Switch the motors off so an arm can be moved by hand again.

    python station/release.py            both CH340 ports
    python station/release.py COM12      one port

WARNING: with the motors off the arm is dead weight and a raised arm WILL fall.
Take hold of it before running this.

This sends CMD 98 with 0 and nothing else. No position is read and no position is
commanded, which is deliberate: the reason an arm needs releasing in the first
place is usually that something commanded it somewhere it should not have gone,
and the recovery tool is not the place to repeat that.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from station.detect import BAUD, build_frame, candidate_ports  # noqa: E402

CMD_TORQUE = 98
CMD_LEROBOT_MODE = 68
SYSTEM_ID = 0xFF


def release(port: str) -> bool:
    import serial

    try:
        ser = serial.Serial(port, BAUD, timeout=0.3)
    except Exception as exc:
        print(f"  {port}: cannot open ({exc})")
        return False
    try:
        time.sleep(0.2)
        # Bridge mode first: on the slave board the torque command is only
        # accepted once the board is listening on the lerobot channel.
        ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([1])))
        time.sleep(0.2)
        for _ in range(3):
            ser.write(build_frame(SYSTEM_ID, CMD_TORQUE, bytes([0])))
            time.sleep(0.1)
        # Leave the board out of bridge mode, the way the other tools find it.
        ser.write(build_frame(SYSTEM_ID, CMD_LEROBOT_MODE, bytes([0])))
        time.sleep(0.1)
        print(f"  {port}: motors off, arm should move freely by hand now")
        return True
    finally:
        ser.close()


def main() -> int:
    ports = sys.argv[1:] or candidate_ports()
    if not ports:
        print("No serial ports found.")
        return 1
    print("Hold the arm before it goes limp.")
    print(f"Switching motors off on: {', '.join(ports)}")
    ok = [release(p) for p in ports]
    if not any(ok):
        print("\nNothing could be released. If a port would not open, close anything")
        print("else using it -- the station, an Arduino IDE, another terminal.")
        return 1
    print("\nIf an arm is still stiff, switch it off at the power switch and on again.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
