"""Every kind of arm the station can drive, behind one small interface.

Until October 2026 the station drove the NexArm and nothing else -- the NexArm
classes were named directly in every mode. A class with mixed hardware, NexArm on
one table and SO-101 on the next, needs one page for all of them, so the kind of
arm is now data rather than code.

Two families:

* **NexArm** keeps its own path, unchanged. Its actions are raw 0-4095 servo
  counts, and every safety number in hardware.py was tuned on it.
* **Every single-arm leader/follower pair LeRobot ships** -- SO-100 / SO-101,
  Koch, OpenManipulator-X, and whatever else is in its registry -- is driven
  through LeRobot's own classes, looked up by name. Their actions are calibrated
  units, degrees or percent of range, so the safety numbers are derived from each
  motor's normalisation mode instead of being written down per robot.

What LeRobot does not provide, and this file does:

* recognising an arm on a USB port without moving it: a guess from the USB chip,
  confirmed by actually talking to the motors, read-only;
* raw positions for the wave test that tells the leader from the follower;
* safety limits in whatever units a kind's actions are in;
* prompts a ten-year-old can follow, in place of LeRobot's terminal ones.
"""

from __future__ import annotations

import importlib
import math
import pkgutil
import time
from dataclasses import dataclass, fields

from . import detect

# ROBOTIS U2D2, an FTDI FT232H underneath: the usual Dynamixel adapter (Koch, OMX).
U2D2 = (0x0403, 0x6014)

# A real drag swings a joint by hundreds of counts; sensor noise is single digits.
# Both a floor and a clear gap are required before a result is trusted.
DRAG_FLOOR = 100
DRAG_RATIO = 3

# How long to wait for one motor to answer while finding out what is on a port.
# LeRobot's own default is a full second per motor, so asking an Arduino or a
# GPS dongle "are you a six-motor arm?" took six seconds per kind of arm -- the
# page froze for twenty. A real motor answers in a couple of milliseconds.
PROBE_TIMEOUT_MS = 100


class Cancelled(Exception):
    """Stop was pressed while the station was waiting on a person."""


@dataclass(frozen=True)
class Kind:
    key: str                  # short name; also the prefix of its dataset folders
    label: str                # what the page shows
    follower_type: str        # LeRobot RobotConfig name
    leader_type: str          # LeRobot TeleoperatorConfig name
    usb_ids: tuple = ()       # (vid, pid) of the USB bridge it usually sits behind
    usb_names: tuple = ()     # ...or words in the port description
    raw_counts: bool = False  # NexArm only: actions are raw 0-4095 counts
    auto: bool = True         # tried on an unrecognised USB chip when nothing claims it

    def usb_match(self, port) -> bool:
        return (port.vid, port.pid) in self.usb_ids or any(
            name in (port.description or "") for name in self.usb_names)


NEXARM = Kind("nexarm", "NexArm", "nexarm_follower", "nexarm_leader",
              usb_ids=(detect.CH340,), usb_names=("CH340",), raw_counts=True)

# Tried in this order. SO-100 and SO-101 are one entry: same motors, same LeRobot
# class, and nothing on the bus tells them apart. Some SO boards use a CH340, so a
# CH340 that does not answer as a NexArm is tried as an SO arm next.
KNOWN = (
    NEXARM,
    Kind("so101", "SO-100 / SO-101", "so101_follower", "so101_leader",
         usb_ids=(detect.CH343, detect.CH340), usb_names=("CH343",)),
    Kind("koch", "Koch", "koch_follower", "koch_leader", usb_ids=(U2D2,)),
    Kind("omx", "OpenManipulator-X", "omx_follower", "omx_leader", usb_ids=(U2D2,)),
    Kind("rebot", "reBot B601", "rebot_b601_follower", "rebot_102_leader", auto=False),
)

_kinds: list[Kind] | None = None


def load_lerobot_arms() -> None:
    """Import every LeRobot follower and leader package that will import.

    LeRobot registers a robot type only when its config module is imported. Only
    the ``*_follower`` and ``*_leader`` packages are touched -- not phones, gloves
    or humanoids -- and one whose SDK is missing is skipped, not fatal.
    """
    import lerobot.robots as robots_pkg
    import lerobot.teleoperators as teleop_pkg

    for pkg in (robots_pkg, teleop_pkg):
        for mod in pkgutil.iter_modules(pkg.__path__):
            if mod.ispkg and mod.name.endswith(("_follower", "_leader")):
                try:
                    importlib.import_module(f"{pkg.__name__}.{mod.name}")
                except Exception:
                    pass


def all_kinds() -> list[Kind]:
    """KNOWN, plus every other single-arm pair in LeRobot's registry.

    A pair is ``<name>_follower`` with a matching ``<name>_leader``, both taking a
    ``port``. Extras are never probed on their own -- an unknown bus type is not
    something to throw packets at -- but they can be picked on the page.
    """
    global _kinds
    if _kinds is not None:
        return _kinds
    kinds = list(KNOWN)
    try:
        load_lerobot_arms()
        from lerobot.robots.config import RobotConfig
        from lerobot.teleoperators.config import TeleoperatorConfig

        followers = RobotConfig.get_known_choices()
        leaders = TeleoperatorConfig.get_known_choices()
        taken = {k.follower_type for k in kinds} | {"so100_follower"}
        for name in sorted(followers):
            base = name.removesuffix("_follower")
            if name == base or name in taken or f"{base}_leader" not in leaders:
                continue
            if not all(_takes_port(c) for c in (followers[name], leaders[f"{base}_leader"])):
                continue
            kinds.append(Kind(base, base.replace("_", " ").title(), name,
                              f"{base}_leader", auto=False))
    except Exception:
        pass
    _kinds = kinds
    return kinds


def _takes_port(config_cls) -> bool:
    try:
        return "port" in {f.name for f in fields(config_cls)}
    except TypeError:
        return False


def by_key(key: str | None) -> Kind | None:
    for kind in all_kinds():
        if kind.key == key:
            return kind
    return None


_missing: dict[str, str | None] = {}


def missing_software(kind: Kind) -> str | None:
    """Why this kind cannot be driven on this computer, or None if it can.

    Building the leader is enough to find out: LeRobot checks for the motor SDK
    when it builds the bus, and building touches no hardware.
    """
    if kind.key not in _missing:
        try:
            make_leader(kind, "COM0" if detect.IS_WINDOWS else "/dev/null", "station_probe")
            _missing[kind.key] = None
        except ImportError as exc:
            _missing[kind.key] = str(exc)
        except Exception:
            _missing[kind.key] = None    # not a software problem; the hardware will say
    return _missing[kind.key]


def choices() -> list[dict]:
    """For the page's robot picker: every kind this computer can actually drive."""
    return [{"key": k.key, "label": k.label} for k in all_kinds() if missing_software(k) is None]


# --------------------------------------------------------------------- configs

def follower_config(kind: Kind, port: str, robot_id: str | None, cameras: dict):
    from lerobot.robots.config import RobotConfig

    all_kinds()
    return RobotConfig.get_choice_class(kind.follower_type)(
        port=port, id=robot_id, cameras=cameras)


def leader_config(kind: Kind, port: str, robot_id: str | None):
    from lerobot.teleoperators.config import TeleoperatorConfig

    all_kinds()
    return TeleoperatorConfig.get_choice_class(kind.leader_type)(port=port, id=robot_id)


def make_follower(kind: Kind, port: str, robot_id: str | None, cameras: dict | None = None):
    from lerobot.robots.utils import make_robot_from_config

    return make_robot_from_config(follower_config(kind, port, robot_id, cameras or {}))


def make_leader(kind: Kind, port: str, robot_id: str | None):
    from lerobot.teleoperators.utils import make_teleoperator_from_config

    return make_teleoperator_from_config(leader_config(kind, port, robot_id))


def arm_ids(kind: Kind, leader_port: str, follower_port: str, serials: dict) -> dict:
    """Names for LeRobot's calibration files, one per physical arm.

    LeRobot keeps a calibration per ``id``. Where the USB adapter has a serial
    number -- the SO boards' CH343 does -- the id carries it, so the file follows
    the arm from port to port and from one computer's cable to another. A NexArm
    needs no calibration file, so it gets no id, exactly as before.
    """
    if kind.raw_counts:
        return {}

    def one(role, port):
        serial = serials.get(port) or ""
        return f"{kind.key}_{role}_{serial}" if serial else f"{kind.key}_{role}"

    return {
        "leader_id": one("leader", leader_port),
        "follower_id": one("follower", follower_port),
        "leader_serial": serials.get(leader_port) or None,
        "follower_serial": serials.get(follower_port) or None,
    }


# --------------------------------------------------------------------- limits

@dataclass(frozen=True)
class Limits:
    """Safety numbers for one joint, in the units its actions use."""

    lo: float
    hi: float
    step: float        # most a target may move in one tick: the jump guard
    sync_step: float   # the same during the slow opening sync
    sync_rate: float   # units per second that sync covers

    def sane(self, value) -> bool:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        return math.isfinite(v) and self.lo <= v <= self.hi


def lerobot_limits(robot) -> dict[str, Limits]:
    """Limits for a LeRobot arm, from how each of its motors is normalised.

    The NexArm numbers in hardware.py, converted: 250 counts a tick is about 22
    degrees, the 12-count sync step about one degree, 320 counts a second about
    28 degrees a second. Percent-of-range joints get the same proportions of a
    200-unit range. The ranges are wide on purpose -- they exist to reject
    garbage, and a calibrated joint can legitimately read a little past its ends.
    """
    from lerobot.motors import MotorNormMode

    out = {}
    for name, motor in robot.bus.motors.items():
        if motor.norm_mode == MotorNormMode.DEGREES:
            lim = Limits(-270.0, 270.0, step=22.0, sync_step=1.0, sync_rate=28.0)
        elif motor.norm_mode == MotorNormMode.RANGE_0_100:
            lim = Limits(-10.0, 110.0, step=12.0, sync_step=0.6, sync_rate=16.0)
        else:
            lim = Limits(-110.0, 110.0, step=12.0, sync_step=0.6, sync_rate=16.0)
        out[f"{name}.pos"] = lim
    return out


# ------------------------------------------------------------ talking to motors

class NexArmReader:
    """Raw positions from a NexArm board over its own protocol."""

    def __init__(self, ser):
        self.ser = ser

    @classmethod
    def open(cls, port: str):
        import serial

        try:
            ser = serial.Serial(port, detect.BAUD, timeout=0.3)
        except Exception:
            return None
        time.sleep(0.25)
        # CMD 68 puts the slave board into bridge mode. Harmless on the master,
        # and it is what makes a follower report positions at all.
        ser.write(detect.build_frame(detect.SYSTEM_ID, detect.CMD_LEROBOT_MODE, bytes([1])))
        time.sleep(0.25)
        if detect.read_positions(ser) is None:
            ser.close()
            return None
        return cls(ser)

    def read(self):
        return detect.read_positions(self.ser)

    def close(self):
        try:
            # Leave the slave board the way it was found.
            self.ser.write(detect.build_frame(detect.SYSTEM_ID, detect.CMD_LEROBOT_MODE, bytes([0])))
            time.sleep(0.05)
            self.ser.close()
        except Exception:
            pass


def _role_bus(kind: Kind, port: str, role: str):
    """The motor bus of the leader's or the follower's class. Builds, never connects."""
    device = make_leader(kind, port, "station_probe") if role == "leader" \
        else make_follower(kind, port, "station_probe")
    return device.bus


def _answers_as(bus) -> bool:
    """Every motor this bus expects answers with the expected model. Read-only.

    The same check as LeRobot's handshake, but with a short timeout, because
    the handshake waits a full second for each motor that does not answer.
    """
    try:
        bus.set_timeout(PROBE_TIMEOUT_MS)
        for motor in bus.motors.values():
            if bus.ping(motor.id) != bus.model_number_table.get(motor.model):
                return False
        return True
    except Exception:
        return False
    finally:
        try:
            bus.set_timeout()
        except Exception:
            pass


class LeRobotReader:
    """Raw positions through a kind's own LeRobot bus, touching nothing else.

    Only ``bus.connect`` runs here -- never ``connect()`` on the device, which
    would start a calibration, and never ``configure()``, which on a follower
    switches the torque on. Every motor the arm should have is pinged and its
    model checked, which makes this a real identification and not a guess from
    the USB chip.

    The leader's motor list is tried first and then the follower's. They are
    the same for an SO arm, but not for every kind: a Koch leader has xl330-m077
    motors where its follower has xl430s, and an OpenManipulator-X follower
    numbers its motors 11-16 where the leader uses 1-6. Checking a follower port
    against the leader's list alone found no follower at all.
    """

    def __init__(self, bus, role: str):
        self.bus = bus
        self.role = role        # which motor list the port answered to

    @classmethod
    def open(cls, kind: Kind, port: str):
        for role in ("leader", "follower"):
            bus = _role_bus(kind, port, role)
            try:
                bus.connect(handshake=False)
            except Exception:
                return None             # the port itself would not open
            if _answers_as(bus):
                reader = cls(bus, role)
                if reader.read() is not None:
                    return reader
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
        return None

    def read(self):
        try:
            return [int(v) for v in self.bus.sync_read("Present_Position", normalize=False).values()]
        except Exception:
            return None

    def close(self):
        try:
            self.bus.disconnect(disable_torque=False)
        except Exception:
            pass


def open_reader(kind: Kind, port: str, log=print):
    if kind.raw_counts:
        return NexArmReader.open(port)
    try:
        return LeRobotReader.open(kind, port)
    except ImportError as exc:
        # The SDK for this kind is not installed. Say so once, plainly.
        log(f"  {kind.label} needs software that is not installed: {exc}")
        return None
    except Exception:
        return None


def kinds_to_try(port, force: Kind | None = None) -> list[Kind]:
    """Which kinds to try on a port, in order.

    A recognised USB chip is tried only as the kinds that use it. An unknown chip
    is tried as every kind marked ``auto``. A NexArm probe is never sent down an
    SO bus whatever was asked for: its frames go to broadcast ID 0xFF, which every
    Feetech servo on that bus would receive.
    """
    if force is not None:
        if force.raw_counts and detect.is_so101_board(port):
            return []
        return [force]
    matched = [k for k in KNOWN if k.usb_match(port)]
    if detect.is_so101_board(port):
        matched = [k for k in matched if not k.raw_counts]
    return matched or [k for k in KNOWN if k.auto and not (k.raw_counts and detect.is_so101_board(port))]


class ArmFinder:
    """Find two arms of one kind, then tell which one a person is holding.

    The role of each arm cannot be read off the hardware -- two NexArm boards are
    identical down to an empty USB serial number -- so it is decided by behaviour:
    a person waves the leader, and the arm that moved is the leader. That works
    for every kind, whether the follower is holding itself up or hanging loose.
    """

    def __init__(self, force: str | None = None):
        self.force = by_key(force) if force else None
        self.kind: Kind | None = None
        self.arms: dict[str, object] = {}
        self.serials: dict[str, str] = {}
        self.baseline: dict[str, list[int] | None] = {}
        self.moved: dict[str, int] = {}

    def open_all(self, log=print) -> list[str]:
        ports = detect.usb_serial_ports()
        if not ports:
            log("No arm cables found. Plug the arms in and switch them on.")
            return []
        log(f"Checking {len(ports)} USB port(s)"
            + (f" for a {self.force.label}." if self.force else "."))

        found: dict[str, dict[str, object]] = {}
        for p in ports:
            for kind in kinds_to_try(p, self.force):
                reader = open_reader(kind, p.device, log)
                if reader is not None:
                    found.setdefault(kind.key, {})[p.device] = reader
                    self.serials[p.device] = p.serial_number or ""
                    log(f"  {p.device}: {kind.label} arm answering")
                    break
            else:
                log(f"  {p.device}: no arm answering")

        # Clone USB adapters sometimes all carry the same serial number. A serial
        # that is not unique cannot tell two arms apart, so it is not kept.
        counts: dict[str, int] = {}
        for serial in self.serials.values():
            if serial:
                counts[serial] = counts.get(serial, 0) + 1
        for p, serial in self.serials.items():
            if serial and counts[serial] > 1:
                self.serials[p] = ""

        pairs = [k for k in found if len(found[k]) >= 2]
        choice = pairs[0] if pairs else max(found, key=lambda k: len(found[k]), default=None)
        if len(pairs) > 1:
            log(f"  More than one kind of robot is plugged in. Using the {by_key(choice).label}.")
        for key, readers in found.items():
            if key != choice:
                for reader in readers.values():
                    reader.close()
        if choice is None:
            return []

        self.kind = by_key(choice)
        self.arms = found[choice]
        for port, reader in self.arms.items():
            self.baseline[port] = reader.read()
            self.moved[port] = 0
        return list(self.arms)

    def sample(self) -> dict[str, int]:
        """One pass over both arms, updating the largest movement seen so far."""
        for port, reader in self.arms.items():
            pos = reader.read()
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
        for reader in self.arms.values():
            reader.close()
        self.arms.clear()


def release(kind: Kind, port: str, role: str = "follower") -> bool:
    """Switch one arm's motors off so it can be moved by hand.

    Uses the motor list of the arm's own role first: an OpenManipulator-X
    follower's motors are 11-16, and switching off 1-6 instead would have
    reported success while the stuck arm stayed stuck. Success means the motors
    were found and told -- not merely that the port opened.
    """
    if kind.raw_counts:
        from station import release as rel

        return rel.release(port)
    other = "leader" if role == "follower" else "follower"
    for which in (role, other):
        try:
            bus = _role_bus(kind, port, which)
            bus.connect(handshake=False)
        except Exception:
            return False
        try:
            if _answers_as(bus):
                bus.disable_torque()
                return True
        except Exception:
            pass
        finally:
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
    return False


def park_goal(bus) -> None:
    """Make the motors' goal the pose they are already in.

    Servos switch on aiming at whatever goal was last written to them, which
    after a previous session is a pose from minutes ago, and the arm lunges there
    the instant torque arrives. Writing the present pose as the goal first means
    switching on holds the arm still. Raw units on both sides, so no calibration
    is involved. Best effort: on failure nothing is worse than stock.
    """
    from .hardware import reading_is_sane

    try:
        if hasattr(bus, "read_positions"):               # NexArm
            current = bus.read_positions()
            if reading_is_sane(current):
                bus.write_positions(list(current))
                time.sleep(0.05)
        else:                                             # LeRobot MotorsBus
            current = bus.sync_read("Present_Position", normalize=False)
            if current:
                bus.sync_write("Goal_Position", current, normalize=False)
                time.sleep(0.05)
    except Exception:
        pass


# --------------------------------------------------------------------- prompts

def which_arm(text: str) -> str:
    """LeRobot names the device in its prompts; turn that into words for a child."""
    low = text.lower()
    if "leader" in low:
        return "the arm you hold (the leader)"
    if "follower" in low:
        return "the robot arm (the follower)"
    return ""


def friendly_prompt(text: str, arm: str) -> str:
    arm = arm or "the arm"
    if "middle of its range" in text:
        return (f"Setting up {arm} for the first time on this computer. Its motors are "
                f"off, so hold it -- it will not hold itself up. Put every joint about "
                f"halfway, then press Next.")
    return text.replace("press ENTER", "press Next").replace("Press ENTER", "Press Next")


def ranges_prompt(arm: str) -> str:
    arm = arm or "the arm"
    return (f"Now move every joint of {arm} slowly all the way one way and all the "
            f"way back, one joint at a time. When every joint has been to both ends, "
            f"press Next.")
