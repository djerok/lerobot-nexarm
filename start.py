#!/usr/bin/env python
"""Start the robot. This is the whole thing:

    python start.py

It finds the arms, finds the cameras, opens a page in the browser, and from there
a child can move the arm and record a dataset. No COM numbers, no camera indices,
no flags.

Two things it does that are easy to miss and cost an afternoon each:

* It re-runs itself inside this folder's .venv if the interpreter it was started
  with cannot import lerobot. Double-clicking a .py file on Windows hands it to
  whichever python is first on PATH, which here is a stub, and the traceback that
  produces says nothing useful.
* It never passes the recording on to Hugging Face. lerobot's own default for
  ``push_to_hub`` is True, and the dataset is video of the room.

No output is printed with characters outside ASCII: this runs in a Windows console
whose code page is cp1252, which raises on anything else rather than substituting.
"""

from __future__ import annotations

import importlib
import os
import shutil
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV_PY = ROOT / ".venv" / "Scripts" / "python.exe"     # Windows
VENV_PY_POSIX = ROOT / ".venv" / "bin" / "python"       # everywhere else


# What has to import before anything works, and the package that provides it.
REQUIRED = (
    ("serial", "pyserial", "talks to the arms"),
    ("cv2", "opencv-python", "reads the cameras"),
    ("lerobot", None, "drives the robot"),           # installed from this folder
)

# Recording pulls these in on top, and `pip install -e .` does NOT bring them.
# The versions are pinned rather than taken latest for two specific reasons:
# av 18 removed `av.option`, which lerobot imports at module level, and pandas 3
# is outside the range lerobot supports.
RECORD_PINS = (
    "av>=15.0.0,<16.0.0",
    "datasets>=4.7.0,<5.0.0",
    "pandas>=2.0.0,<3.0.0",
    "pyarrow>=21.0.0,<30.0.0",
    "jsonlines>=4.0.0,<5.0.0",
)

# The motor SDKs for every arm that is not a NexArm: Feetech for SO-100/SO-101,
# Dynamixel for Koch and OpenManipulator-X. Same ranges as lerobot's own extras.
# None of them is needed for a NexArm, so a missing one is installed if it can be
# and otherwise only means that kind of arm is not offered.
ARM_SDKS = (
    ("scservo_sdk", "feetech-servo-sdk>=1.0.0,<2.0.0", "SO-100 / SO-101"),
    ("dynamixel_sdk", "dynamixel-sdk>=3.7.31,<3.9.0", "Koch, OpenManipulator-X"),
    ("deepdiff", "deepdiff>=7.0.1,<9.0.0", "both of the above"),
) + ((
    # Windows: recognise each camera by identity, so an unplugged camera comes
    # back in its own slot instead of whichever camera took over its number.
    ("pygrabber", "pygrabber>=0.2,<1", "following cameras across unplugs"),
) if sys.platform.startswith("win") else ())


def missing_modules() -> list[tuple[str, str | None, str]]:
    out = []
    for module, package, why in REQUIRED:
        try:
            __import__(module)
        except Exception:
            out.append((module, package, why))
    return out


def recording_works() -> bool:
    """Can lerobot actually record, or only teleoperate?

    Worth a separate question: without the dataset extra, importing lerobot
    succeeds and teleop runs, and the failure only appears when a child presses
    Record -- with a message about `datasets` that says nothing about an extra.
    """
    try:
        from lerobot.scripts import lerobot_record, lerobot_replay  # noqa: F401
        return True
    except Exception:
        return False


def have_deps() -> tuple[bool, str]:
    gaps = missing_modules()
    if gaps:
        module, _, why = gaps[0]
        return False, f"{module} -- {why}"
    if not recording_works():
        return False, "the recording dependencies"
    return True, ""


def auto_install() -> bool:
    """Install what is missing, without making anyone read an error first.

    Uses uv against this folder's own .venv. If uv is not on the machine there
    is nothing sensible to do automatically, so it says which script to run.
    """
    venv = VENV_PY if VENV_PY.exists() else VENV_PY_POSIX
    if not venv.exists():
        return False
    uv = shutil.which("uv") or shutil.which(
        "uv.exe", path=str(Path.home() / ".local" / "bin"))
    if not uv:
        return False

    wanted = [pkg for _, pkg, _ in missing_modules() if pkg]
    if not recording_works():
        wanted += list(RECORD_PINS)
    if not wanted:
        return False

    print("Installing what is missing. This runs once and takes a few minutes.")
    for spec in wanted:
        print(f"  {spec}")
    result = subprocess.run(
        [uv, "pip", "install", "--python", str(venv), *wanted])
    return result.returncode == 0


def missing_arm_sdks() -> list[tuple[str, str, str]]:
    out = []
    for module, spec, arms in ARM_SDKS:
        try:
            __import__(module)
        except Exception:
            out.append((module, spec, arms))
    return out


def install_arm_sdks() -> None:
    """Fetch the SDKs for non-NexArm arms if any are missing. Never fatal."""
    missing = missing_arm_sdks()
    if not missing:
        return
    venv = VENV_PY if VENV_PY.exists() else VENV_PY_POSIX
    uv = shutil.which("uv") or shutil.which(
        "uv.exe", path=str(Path.home() / ".local" / "bin"))
    if not (venv.exists() and uv):
        return
    print("Adding support for more kinds of arm (once):")
    for _, spec, arms in missing:
        print(f"  {spec}  -- {arms}")
    subprocess.run([uv, "pip", "install", "--python", str(venv),
                    *[spec for _, spec, _ in missing]])
    importlib.invalidate_caches()


def can_spawn(python: Path) -> tuple[bool, str]:
    """Does this interpreter actually start?

    Existing on disk is not the same as working. A uv-built .venv is a trampoline
    holding the absolute path of one exact Python build, so a uv upgrade, a moved
    folder, or a copied project turns it into a file that exists and cannot run:

        uv trampoline failed to spawn Python child process ... entity not found

    That error names uv, not the robot, and it is the thing that most often makes
    a fresh machine look broken. So it is tested rather than assumed.
    """
    if not python.exists():
        return False, "not there"
    try:
        r = subprocess.run([str(python), "-c", "pass"],
                           capture_output=True, text=True, timeout=60)
    except Exception as exc:
        return False, str(exc)
    if r.returncode == 0:
        return True, ""
    return False, (r.stderr or r.stdout or f"exit code {r.returncode}").strip()


def uv_path() -> str | None:
    return shutil.which("uv") or shutil.which(
        "uv.exe", path=str(Path.home() / ".local" / "bin"))


def rebuild_venv() -> Path | None:
    """Recreate .venv in place, which is what fixes a dead trampoline.

    The installed packages live in .venv/Lib/site-packages and survive this; it
    is the launcher stubs that are rewritten. If they do not survive, the
    dependency check that follows reinstalls them anyway.
    """
    uv = uv_path()
    if not uv:
        return None
    print("This folder's Python cannot start. Rebuilding it, one moment.")
    r = subprocess.run([uv, "venv", "--python", "3.12", str(ROOT / ".venv"),
                        "--allow-existing"], capture_output=True, text=True)
    if r.returncode != 0:
        print((r.stderr or "").strip()[:400])
        return None
    for candidate in (VENV_PY, VENV_PY_POSIX):
        ok, _ = can_spawn(candidate)
        if ok:
            return candidate
    return None


def working_venv_python() -> Path | None:
    """The project interpreter, repaired if it needs repairing."""
    for candidate in (VENV_PY, VENV_PY_POSIX):
        if candidate.exists():
            ok, why = can_spawn(candidate)
            if ok:
                return candidate
            print(f"{candidate.name} will not start: {why.splitlines()[0][:200]}")
            return rebuild_venv()
    return None


def reexec_in_venv() -> None:
    """Hand over to the project venv, once, if this interpreter cannot do the job."""
    if os.environ.get("NEXARM_STATION_REEXEC"):
        return
    venv = working_venv_python()
    if venv is None or Path(sys.executable).resolve() == venv.resolve():
        return
    print("Switching to this folder's own Python.")
    env = dict(os.environ, NEXARM_STATION_REEXEC="1")
    raise SystemExit(subprocess.run([str(venv), str(Path(__file__).resolve()), *sys.argv[1:]],
                                    env=env).returncode)


def doctor() -> int:
    """Print what this machine actually has, for when something still will not run.

        python start.py --doctor
    """
    print("=" * 58)
    print(" Robot Station -- what this computer has")
    print("=" * 58)
    print(f"  platform          {sys.platform}")
    print(f"  python running    {sys.version.split()[0]}  {sys.executable}")
    print(f"  project folder    {ROOT}")

    uv = uv_path()
    print(f"  uv                {uv or 'NOT FOUND -- run the installer for your OS'}")

    for candidate in (VENV_PY, VENV_PY_POSIX):
        if candidate.exists():
            ok, why = can_spawn(candidate)
            print(f"  .venv python      {'works' if ok else 'BROKEN: ' + why.splitlines()[0][:160]}")
            break
    else:
        print("  .venv python      NOT BUILT -- run the installer for your OS")

    for module, package, why in REQUIRED:
        try:
            __import__(module)
            print(f"  {module:<17} ok")
        except Exception as exc:
            print(f"  {module:<17} MISSING ({package or 'this folder'}) -- {type(exc).__name__}")
    print(f"  recording         {'ok' if recording_works() else 'MISSING the record dependencies'}")
    for module, spec, arms in ARM_SDKS:
        try:
            __import__(module)
            state = "ok"
        except Exception:
            state = f"MISSING ({spec}) -- no {arms}"
        print(f"  {module:<17} {state}")

    try:
        from station import detect
        print(f"  serial ports      {detect.candidate_ports() or 'none -- arms unplugged or no driver'}")
    except Exception as exc:
        print(f"  serial ports      cannot check ({type(exc).__name__}: {exc})")

    try:
        import cv2
        found = []
        for i in range(4):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                found.append(i)
            cap.release()
        print(f"  cameras           {found or 'none -- that is allowed, the arms still work'}")
    except Exception as exc:
        print(f"  cameras           cannot check ({type(exc).__name__})")

    print()
    print("  Paste this whole block when asking for help.")
    return 0


def main() -> int:
    if "--doctor" in sys.argv:
        return doctor()

    ok, missing = have_deps()
    if not ok:
        reexec_in_venv()
        ok, missing = have_deps()
    if not ok:
        print("Something needed is not installed: " + missing)
        if auto_install():
            ok, missing = have_deps()
    if not ok:
        print()
        print("Could not install it automatically: " + missing)
        print("Run the one-time installer for your computer:")
        print("  Windows:  powershell -ExecutionPolicy Bypass -File setup.ps1")
        print("  macOS:    bash setup.sh")
        return 1

    install_arm_sdks()

    # Imported after the venv check, because these import lerobot.
    from station import robots, server
    from station.hardware import Station

    print("=" * 58)
    print(" Robot Station")
    print("=" * 58)

    # Loading LeRobot's list of arms takes a few seconds; do it before the page
    # asks for it rather than while a child waits on the first screen.
    print("  Arms this station can drive: "
          + ", ".join(c["label"] for c in robots.choices()))

    station = Station()
    usable, _ = station.load()
    if usable:
        station.open_cameras()

    port = server.free_port()
    httpd = server.serve(station, port)
    url = f"http://127.0.0.1:{port}/"

    print()
    print("  Open this page:  " + url)
    if not usable:
        print("  The page will help you find the arms. Plug both in, switch them on.")
    print()
    print("  Leave this window open. Press Ctrl+C here when you are finished.")
    print()

    webbrowser.open(url)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        station.stop()
        # Give a running teleop loop or record a moment to let go of the hardware,
        # otherwise the next run finds the ports still locked.
        for _ in range(20):
            if station.mode == "idle":
                break
            time.sleep(0.25)
        station.close_cameras()
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
