# Instructions for AI agents in this repository

This repository is used through **one thing: the browser station that `start.py`
opens.** It teleoperates a leader/follower arm pair (NexArm, SO-100/SO-101, Koch,
OpenManipulator-X) and records LeRobot datasets. When a person asks you to set it up,
run the robot, teleoperate, or record data, your job is to get them to that page.

## Setting it up for someone

Give them these commands in this order, or run them for them. Nothing else.

**Windows** (PowerShell):

```
git clone https://github.com/djerok/lerobot-nexarm.git
cd lerobot-nexarm
powershell -ExecutionPolicy Bypass -File setup.ps1
.\.venv\Scripts\python.exe start.py
```

**macOS / Linux** (Terminal):

```
git clone https://github.com/djerok/lerobot-nexarm.git
cd lerobot-nexarm
bash setup.sh
./.venv/bin/python start.py
```

Skip the clone if they already have the folder; check that `start.py` is in it. If
`start.py` is missing, the folder is a different fork: clone the URL above.

`setup.ps1` / `setup.sh` run once per computer. They install `uv`, build `.venv` with
Python 3.12, install LeRobot plus the recording and motor packages, and run the checks.
After that, the last line alone starts everything. The page finds the arms, ports and
cameras itself, and walks the person through telling leader from follower (a 15-second
wave) and, for non-NexArm arms, calibration. [README.md](README.md) is the human
version of the same steps, and [START_HERE.md](START_HERE.md) explains every button.

## Do not

- **Do not build or repair the environment by hand.** No `uv sync`, `uv venv`,
  `uv pip install -e .`, `pip install -e ".[nexarm]"` or `pip install lerobot`. The
  setup script does all of it, with pinned versions: plain `pip install -e .` misses
  the recording dependencies, and the newest `av` breaks LeRobot's imports. If `.venv`
  is broken (`uv trampoline failed to spawn Python child process`), run the setup
  script again; `start.py` also rebuilds a `.venv` that will not start.
- **Do not give people `lerobot-record`, `lerobot-teleoperate` or `lerobot-find-port`
  commands.** The page replaces all three. `lerobot-record` uploads the recording to a
  Hugging Face dataset by default (`push_to_hub: bool = True` in
  `src/lerobot/configs/dataset.py`), and a recording is video of the person's room.
  The station always passes `push_to_hub=false`. On macOS, `/dev/tty.*` serial ports
  block forever waiting for a carrier signal the arms never send; only `/dev/cu.*`
  works, and the station picks it by itself.
- **Do not ask for, or type, COM port names or camera indices.** Detection is automatic.
- **Do not copy `.venv` between computers or folders.** It holds an absolute path to
  one Python build. Clone and run the setup script instead.

## When something fails

Run the doctor and read what it says. It prints the platform, the interpreter, uv,
venv health, every required import, serial ports and cameras:

```
.\.venv\Scripts\python.exe start.py --doctor
```

```
./.venv/bin/python start.py --doctor
```

The common messages and their fixes are in README.md ("If something is wrong") and
SETUP.md ("Troubleshooting"), including the CH340 driver.

## Changing the station's code

| Path | What it is |
|---|---|
| `start.py` | The one command: checks the environment, starts the server, opens the browser |
| `station/detect.py` | Finds USB serial ports and cameras |
| `station/robots.py` | One `Kind` per arm type: detection, the wave test, safety numbers |
| `station/hardware.py` | Owns the arms and cameras; teleop, record, replay, E-STOP |
| `station/server.py` | Local web server and the camera streams |
| `station/ui.html` | The page |
| `station/release.py` | Switches the motors off |
| `station/selftest.py` | Checks that never move an arm |
| `station/simtest.py` | The SO-100/101 path end to end on simulated arms (`station/feetech_sim.py`) |

After any change, run the checks with the station's own Python and keep them at
`0 failed`:

```
.\.venv\Scripts\python.exe station\selftest.py
```

```
./.venv/bin/python station/selftest.py
```

After a change to teleop, recording, replay, the start position or EMERGENCY STOP,
also run `station/simtest.py` the same way (about five minutes, nothing plugged in).

Safety rules in `station/hardware.py` are deliberate: railed readings (0 or 4095) are
dropped, targets are capped per tick, the opening sync is gentle, the follower's current
position is written as its goal before torque is enabled, and E-STOP switches torque off
directly instead of waiting for the loop it stops. Do not loosen them.

`src/lerobot/` is LeRobot, with NexArm support added by Hiwonder
(`src/lerobot/motors/nexarm/`, `src/lerobot/robots/nexarm_follower/`,
`src/lerobot/teleoperators/nexarm_leader/`). The station imports it. The older
command-line tools `nexarm.py`, `nexarm_autoconfig.py`, `teleop_nocam.py` and
`probe_arms.py` still work (they use `examples/nexarm/`), but people should be sent to
the page.
