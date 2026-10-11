"""Two simulated SO-101 arms -- Feetech STS3215 buses behind CH343 adapters -- for simtest.py.

No SO-101 had been run on this station when it was written, so the SO path was
checked on this instead. Everything above the serial port is the real code:
scservo_sdk's PortHandler and packet handler, LeRobot's FeetechMotorsBus,
SOFollower / SOLeader, and the station. Only ``serial.Serial`` (as scservo_sdk
sees it), the COM port list and the cameras are replaced. Nothing here can reach
a real port: install() also replaces the real ``serial.Serial`` with one that
refuses every port.

Servo model (STS3215, protocol 0):
  * 256-byte control table, little-endian, sign-magnitude where LeRobot says so
    (Homing_Offset bit 11, Goal/Present_Position bit 15).
  * Present_Position = (raw - Homing_Offset) mod 4096.
  * Torque on: raw moves toward clamp(Goal, Min, Max) + Homing_Offset at ~3000 c/s.
    Torque_Limit below GRAVITY_HOLD_LIMIT cannot hold a joint gravity pulls on.
  * Torque off: a scripted hand moves it, else gravity folds the follower's
    shoulder_lift / elbow_flex / wrist_flex toward a rest pose.
  * Optional: writing Goal_Position switches torque on (worst case for E-STOP).
  * A blocked gripper: the jaw cannot pass a given raw value.
  * Replies arrive after a USB latency, one motor after another, so the SDK's
    non-atomic is_using flag has the same windows it has on a real bus.

What it does not know: anything a real STS3215 does that its manual does not
say -- how far an arm really sags, how its firmware treats a goal written with
torque off. Those wait for real arms.
"""

from __future__ import annotations

import math
import random
import threading
import time
import types

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
GRAVITY_JOINTS = {"shoulder_lift", "elbow_flex", "wrist_flex"}
GRAVITY_HOLD_LIMIT = 150       # Torque_Limit below this sags under gravity
SPEED = 3000.0                 # counts / s with torque on
FALL_SPEED = 900.0             # counts / s an unpowered joint folds

# Reply timing. Measured with time.perf_counter throughout: time.monotonic()
# ticks every 15.6 ms on Windows before Python 3.13, so a reply "ready 0.8 ms
# from now" would wait for the next tick and every transaction took ~8 ms.
FIRST_REPLY_S = 0.0008         # USB latency to the first status byte
NEXT_REPLY_S = 0.00015         # each further motor in a sync read


def u16(mem, a):
    return mem[a] | (mem[a + 1] << 8)


def set16(mem, a, v):
    mem[a] = v & 0xFF
    mem[a + 1] = (v >> 8) & 0xFF


def sm_decode(v, bit):
    mag = v & ((1 << bit) - 1)
    return -mag if (v >> bit) & 1 else mag


def checksum(body) -> int:
    return (~sum(body)) & 0xFF


class Servo:
    def __init__(self, arm, sid: int, name: str, raw: float):
        self.arm, self.id, self.name = arm, sid, name
        self.mem = bytearray(256)
        m = self.mem
        m[0], m[1] = 3, 10                 # firmware 3.10
        set16(m, 3, 777)                   # STS3215
        m[5], m[6], m[7], m[8] = sid, 0, 250, 1
        set16(m, 9, 0)
        set16(m, 11, 4095)
        m[13], m[14], m[15] = 70, 140, 40
        set16(m, 16, 1000)                 # Max_Torque_Limit (EPROM)
        m[18] = 0x1C                       # Phase, bit 4 set as shipped on some
        m[21], m[22], m[23] = 32, 32, 0
        set16(m, 28, 500)
        m[33], m[36] = 0, 80
        m[40] = 0                          # torque off at power-on
        m[41] = 0
        set16(m, 48, 1000)                 # Torque_Limit (RAM) = Max at power-on
        m[55] = 1
        m[85] = 0
        self.raw = float(raw)
        set16(m, 42, int(self.raw) % 4096)  # power-on goal = where it is
        self.hand = None                   # callable(now) -> raw, while held by a person
        self.block = None                  # (raw, +1|-1): jaw cannot go past raw that way
        self.rest = None                   # gravity folds it here when unpowered
        self.torque_log: list[tuple[float, int, str]] = []

    # --- registers
    @property
    def offset(self) -> int:
        return sm_decode(u16(self.mem, 31), 11)

    @property
    def torque(self) -> bool:
        return self.mem[40] == 1

    @property
    def torque_limit(self) -> int:
        return u16(self.mem, 48)

    @property
    def goal(self) -> int:
        return sm_decode(u16(self.mem, 42), 15)

    @property
    def limits(self) -> tuple[int, int]:
        return u16(self.mem, 9), u16(self.mem, 11)

    def present(self) -> int:
        return int(round(self.raw - self.offset)) % 4096

    def target_raw(self) -> float:
        g = self.goal
        lo, hi = self.limits
        if not (lo == 0 and hi == 0):
            g = max(lo, min(hi, g))
        return float(g + self.offset)

    def read_mem(self, addr: int, n: int) -> bytes:
        set16(self.mem, 56, self.present())
        return bytes(self.mem[addr:addr + n])

    def write_mem(self, addr: int, data: bytes, who: str) -> None:
        before = self.torque
        for i, b in enumerate(data):
            if addr + i < 256:
                self.mem[addr + i] = b
        touched = range(addr, addr + len(data))
        if 40 in touched and self.mem[40] not in (0, 1):
            self.mem[40] = 1 if self.mem[40] else 0
        if (42 in touched or 43 in touched) and self.arm.goal_enables_torque:
            self.mem[40] = 1
        if self.torque != before:
            self.torque_log.append((time.perf_counter(), int(self.torque), who))

    # --- physics
    def advance(self, now: float, dt: float) -> None:
        if self.torque:
            target = self.target_raw()
            if self.name in GRAVITY_JOINTS and self.rest is not None \
                    and self.torque_limit < GRAVITY_HOLD_LIMIT:
                target = self.rest           # too weak to hold against gravity
                step = FALL_SPEED * 0.3 * dt
            else:
                step = SPEED * dt
            d = target - self.raw
            self.raw += max(-step, min(step, d))
        elif self.hand is not None:
            self.raw = float(self.hand(now))
        elif self.rest is not None and self.name in GRAVITY_JOINTS:
            d = self.rest - self.raw
            step = FALL_SPEED * dt
            self.raw += max(-step, min(step, d))
        if self.block is not None:
            at, way = self.block
            if way < 0 and self.raw < at:
                self.raw = at
            if way > 0 and self.raw > at:
                self.raw = at


class Arm:
    """One Feetech bus: six servos behind one USB adapter."""

    def __init__(self, port: str, serial_number: str, pose, rest=None):
        self.port, self.serial_number = port, serial_number
        self.servos = {i + 1: Servo(self, i + 1, n, pose[i]) for i, n in enumerate(JOINTS)}
        if rest is not None:
            for i, s in self.servos.items():
                s.rest = float(rest[i - 1])
        self.lock = threading.RLock()
        self.open_handle = None
        self.goal_enables_torque = False
        self.drop_prob = 0.0             # chance a motor's reply never comes
        self.drop_at: list[float] = []   # perf_counter times: lose the next reply after each
        self.powered = True
        self._t = time.perf_counter()
        self.opens = 0
        self.max_open_seconds = 0.0

    def by_name(self, name: str) -> Servo:
        return self.servos[JOINTS.index(name) + 1]

    def advance(self) -> None:
        with self.lock:
            now = time.perf_counter()
            dt = min(0.2, now - self._t)
            self._t = now
            if dt > 0:
                for s in self.servos.values():
                    s.advance(now, dt)

    def raws(self) -> list[float]:
        self.advance()
        return [self.servos[i].raw for i in range(1, 7)]

    def torques(self) -> list[int]:
        return [int(self.servos[i].torque) for i in range(1, 7)]

    def limits_now(self) -> list[int]:
        return [self.servos[i].torque_limit for i in range(1, 7)]

    def grab(self, script=None) -> None:
        """A person takes hold. script(t, base) -> raw per joint; None = hold still."""
        base = self.raws()
        t0 = time.perf_counter()
        for i, s in self.servos.items():
            j = s.name
            if script is None:
                s.hand = (lambda b: (lambda now: b))(base[i - 1])
            else:
                s.hand = (lambda j=j, b=base[i - 1]: (lambda now: script(now - t0, j, b)))()

    def let_go(self) -> None:
        for s in self.servos.values():
            s.hand = None

    def background(self, stop: threading.Event, hz: float = 200.0) -> threading.Thread:
        def run():
            while not stop.is_set():
                self.advance()
                time.sleep(1.0 / hz)
        t = threading.Thread(target=run, daemon=True, name=f"phys-{self.port}")
        t.start()
        return t


class FakeSerial:
    """What scservo_sdk's PortHandler opens: serial.Serial, for a simulated arm."""

    def __init__(self, port=None, baudrate=1_000_000, bytesize=8, timeout=0, **kw):
        import serial

        arm = SIM.arms.get(port)
        if arm is None or not arm.powered:
            raise serial.SerialException(f"could not open port '{port}': FileNotFoundError(2, "
                                         f"'The system cannot find the file specified.', None, 2)")
        with arm.lock:
            if arm.open_handle is not None:
                raise serial.SerialException(f"could not open port '{port}': PermissionError(13, "
                                             f"'Access is denied.', None, 5)")
            arm.open_handle = self
            arm.opens += 1
        self.arm, self.port, self.baudrate = arm, port, baudrate
        self.is_open = True
        self._tx = bytearray()
        self._rx: list[list] = []          # [ready_time, bytes]
        self._lock = threading.Lock()
        self._opened_at = time.perf_counter()

    # pyserial surface used by scservo_sdk
    @property
    def in_waiting(self) -> int:
        self._check()
        now = time.perf_counter()
        with self._lock:
            return sum(len(b) for t, b in self._rx if t <= now)

    def read(self, n: int = 1) -> bytes:
        self._check()
        now = time.perf_counter()
        out = bytearray()
        with self._lock:
            while self._rx and len(out) < n and self._rx[0][0] <= now:
                t, b = self._rx[0]
                take = b[:n - len(out)]
                out += take
                if len(take) == len(b):
                    self._rx.pop(0)
                else:
                    self._rx[0][1] = b[len(take):]
        return bytes(out)

    def write(self, data) -> int:
        self._check()
        self.arm.advance()
        data = bytes(data)
        with self._lock:
            self._tx += data
            packets = []
            while True:
                i = self._tx.find(b"\xff\xff")
                if i < 0:
                    if self._tx[-1:] != b"\xff":
                        self._tx.clear()
                    break
                del self._tx[:i]
                if len(self._tx) < 4:
                    break
                total = 4 + self._tx[3]
                if len(self._tx) < total:
                    break
                packets.append(bytes(self._tx[:total]))
                del self._tx[:total]
        for p in packets:
            self._handle(p)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        now = time.perf_counter()
        with self._lock:
            self._rx = [e for e in self._rx if e[0] > now]

    def close(self) -> None:
        if not self.is_open:
            return
        self.is_open = False
        with self.arm.lock:
            if self.arm.open_handle is self:
                self.arm.open_handle = None
        self.arm.max_open_seconds = max(self.arm.max_open_seconds,
                                        time.perf_counter() - self._opened_at)

    def _check(self) -> None:
        import serial

        if not self.is_open:
            raise serial.SerialException("Attempting to use a port that is not open")
        if not self.arm.powered:
            raise serial.SerialException("ClearCommError failed (device removed)")

    # protocol
    def _reply(self, sid: int, data: bytes, k: int) -> None:
        if random.random() < self.arm.drop_prob:
            SIM.dropped += 1
            return
        if self.arm.drop_at and time.perf_counter() >= self.arm.drop_at[0]:
            self.arm.drop_at.pop(0)
            SIM.dropped += 1
            return
        body = bytes([sid, len(data) + 2, 0]) + data
        pkt = b"\xff\xff" + body + bytes([checksum(body)])
        ready = time.perf_counter() + FIRST_REPLY_S + k * NEXT_REPLY_S + len(pkt) * 1e-5
        with self._lock:
            self._rx.append([ready, pkt])

    def _handle(self, pkt: bytes) -> None:
        if checksum(pkt[2:-1]) != pkt[-1]:
            SIM.bad_checksums += 1
            return
        sid, instr, params = pkt[2], pkt[4], pkt[5:-1]
        servos = self.arm.servos
        who = threading.current_thread().name
        SIM.packets += 1
        with self.arm.lock:
            if instr == PING:
                ids = sorted(servos) if sid == BROADCAST else ([sid] if sid in servos else [])
                for k, i in enumerate(ids):
                    self._reply(i, b"", k)
            elif instr == READ and sid in servos:
                self._reply(sid, servos[sid].read_mem(params[0], params[1]), 0)
            elif instr == WRITE:
                ids = sorted(servos) if sid == BROADCAST else ([sid] if sid in servos else [])
                for i in ids:
                    servos[i].write_mem(params[0], params[1:], who)
                if sid != BROADCAST and sid in servos:
                    self._reply(sid, b"", 0)
            elif instr == SYNC_WRITE:
                addr, n = params[0], params[1]
                rest = params[2:]
                for j in range(0, len(rest) - n, n + 1):
                    i = rest[j]
                    if i in servos:
                        servos[i].write_mem(addr, rest[j + 1:j + 1 + n], who)
            elif instr == SYNC_READ:
                addr, n = params[0], params[1]
                for k, i in enumerate(params[2:]):
                    if i in servos:
                        self._reply(i, servos[i].read_mem(addr, n), k)


class _Sim:
    def __init__(self):
        self.arms: dict[str, Arm] = {}
        self.ports: list = []
        self.dropped = 0
        self.bad_checksums = 0
        self.packets = 0


SIM = _Sim()


class FakePortInfo:
    def __init__(self, device, serial_number, vid=0x1A86, pid=0x55D3):
        self.device, self.name = device, device
        self.vid, self.pid = vid, pid
        self.serial_number = serial_number
        self.description = f"USB-Enhanced-SERIAL CH343 ({device})"
        self.hwid = f"USB VID:PID={vid:04X}:{pid:04X} SER={serial_number}"
        self.manufacturer, self.product, self.location, self.interface = "wch.cn", None, None, None

    def __getitem__(self, i):
        return (self.device, self.description, self.hwid)[i]


def install() -> None:
    """Point every serial port this process can open at the simulator."""
    import serial
    import serial.tools.list_ports as list_ports
    import scservo_sdk.port_handler as ph

    def no_real_ports(*a, **kw):
        raise serial.SerialException("simulator: real serial ports are switched off")

    serial.Serial = no_real_ports
    ph.serial = types.SimpleNamespace(Serial=FakeSerial, EIGHTBITS=8,
                                      SerialException=serial.SerialException)
    list_ports.comports = lambda include_links=False: [p for p in SIM.ports
                                                      if SIM.arms[p.device].powered]


# ------------------------------------------------------------------ cameras

class FakeCapture:
    """cv2.VideoCapture for a camera that is not there: 30 fps of moving pictures."""

    OPEN = {0, 1}

    def __init__(self, index):
        import numpy as np

        self.np = np
        self.index = int(index)
        self.opened = self.index in self.OPEN
        self.n = 0
        self.next_t = time.perf_counter()
        self.props = {}

    def isOpened(self):
        return self.opened

    def read(self):
        if not self.opened:
            time.sleep(0.05)
            return False, None
        now = time.perf_counter()
        if self.next_t > now:
            time.sleep(self.next_t - now)
        self.next_t = max(self.next_t + 1 / 30, time.perf_counter() - 0.05)
        f = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
        f[:, :, self.index % 3] = 60
        x = (self.n * 7) % 600
        f[200:280, x:x + 40, :] = 255
        f[0:8, 0:8, 1] = self.n % 256
        self.n += 1
        return True, f

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        import cv2

        if prop == cv2.CAP_PROP_FOURCC:
            return float(int.from_bytes(b"MJPG", "little"))
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return 640.0
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return 480.0
        return self.props.get(prop, 0.0)

    def release(self):
        self.opened = False


def sweep(amplitudes: dict, period: float = 1.6):
    """A hand script: each joint swings around where it was grabbed, one after another."""
    names = list(amplitudes)

    def script(t, joint, base):
        if joint not in amplitudes:
            return base
        k = names.index(joint)
        local = t - k * period
        if local < 0 or local > period:
            return base
        return base + amplitudes[joint] * math.sin(2 * math.pi * local / period)

    return script
