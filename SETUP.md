# NexArm setup — every step, from a clean machine

Works on **Windows** and **macOS**. You need the two arms, their USB cables, and
one or two USB cameras. You do **not** need to know which cable is which, which
COM port or `/dev` name anything landed on, or which camera is which number. The
software works all of that out.

If you only want the short version: install once, then `python start.py` forever.

---

## 0. What you need first

| | Windows | macOS |
|---|---|---|
| OS | Windows 10 or 11 | macOS 12 or newer |
| Python | **not needed** — the installer fetches 3.12 | **not needed** — same |
| Disk | about 6 GB, mostly PyTorch | same |
| Driver | usually automatic; see [CH340 driver](#ch340-driver) if no port appears | see [CH340 driver](#ch340-driver) |

Do **not** install Python from the Microsoft Store. On this kind of machine a
bare `python` is often a stub that silently does nothing, which produces errors
that look like the robot is broken when nothing is plugged in wrong. The
installer sidesteps it by building its own environment.

---

## 1. Get the code

```
git clone https://github.com/djerok/lerobot-nexarm.git
cd lerobot-nexarm
```

No git? Download the ZIP from the repository page, unzip it, and open a terminal
in the unzipped folder.

---

## 2. Install, once per machine

### Windows

Open **PowerShell** in the folder and run:

```
powershell -ExecutionPolicy Bypass -File setup.ps1
```

### macOS

Open **Terminal** in the folder and run:

```
bash setup.sh
```

Either way it installs `uv`, builds a Python 3.12 environment in `.venv`,
installs LeRobot and the recording dependencies, and then runs the checks.

**It takes several minutes the first time** because PyTorch is large. It is not
stuck.

Expect the checks to end with something like `82 passed, 0 failed`. A few camera
checks are skipped if the robot page is already open — that is normal and it
says so.

---

## 3. Plug the robot in

1. Both arms into USB.
2. Both arms switched **on** at their power switches.
3. Cameras into USB. One is enough; two is better. If a camera is bolted to the
   gripper, that is the **wrist** camera.

---

## 4. Run it

### Windows

```
.\.venv\Scripts\python.exe start.py
```

### macOS

```
./.venv/bin/python start.py
```

Your browser opens on the robot page. Leave the terminal window open — closing it
stops the robot. Press `Ctrl+C` there when you are finished.

---

## 5. The first run asks you to wave an arm

The page asks you to **take hold of the arm you move by hand and wave it around
for fifteen seconds.** Wave only that one, and move it a long way.

This is not a formality and it cannot be skipped. Both circuit boards are the
same CH340 chip with no serial number, and both answer the same protocol
identically, so nothing the computer can ask will tell it which arm is which. The
only real difference is physical: the leader's motors are off so a person can
drag it, and the follower's motors are holding it still. Watching which one moves
is the answer.

The result is written to `nexarm.json` and you are never asked again — unless a
cable moves, which changes the port names, and then the page notices and asks
again.

Cameras are found straight after and guessed. If front and wrist come out the
wrong way round, press **Swap front and wrist**. There is no way to tell from a
USB camera which one is bolted to the gripper, so a guess plus a button is the
honest design.

---

## 6. What the page does

| Button | What happens |
|---|---|
| **Find my robot** | The fifteen-second wave. Only shown when the setup is unknown. |
| **Start moving** | The arm you hold drives the other one. |
| **Stop** | Ends whatever is running. |
| **Save the recordings in this folder** | Choose where datasets are written. Empty means `datasets/` here. |
| **Start recording** | Records one try of a job you name. No timer. |
| **Done, save this try** | Saves it. Press Start again to add another try to the same job. |
| **That went wrong, do it again** | Throws this try away and starts it over. |
| **Throw it away and stop** | Ends the try without keeping it. |
| **Play it back** | Replays a saved try on the arm, with nobody holding the leader. |
| **Swap front and wrist** | The cameras were guessed the wrong way round. |
| **Switch the motors off** | Frees a stuck arm. Hold it first — it drops. |

---

## 7. Where recordings go, and where they do not go

Each press of **Start recording** captures exactly one try, for as long as it
takes -- there is no episode timer and no tidy-up timer. Pressing Start again with
the same job name adds another try to the same dataset, so a session of repeated
goes produces one trainable dataset rather than a pile of one-episode folders.

Recordings are written to `datasets/` in this folder unless you choose another
folder on the page. They are in standard LeRobot dataset format, so they can be
used for training directly.

**They are not uploaded anywhere.** This matters more than it sounds: LeRobot's
own `push_to_hub` default is `True`, so the plain `lerobot-record` command
publishes to a **public** Hugging Face dataset when it finishes. A recording is
video of the room and everyone in it. This station passes `push_to_hub=false`
explicitly on every recording.

If you ever use `lerobot-record` directly instead of this page, pass it yourself:

```
--dataset.push_to_hub=false
```

`datasets/` is in `.gitignore`, so recordings cannot be committed by accident.

---

## 8. Safety, and what the software will not do

- **Readings that look broken are ignored, never obeyed.** The boards sometimes
  return a railed value (0 or 4095) when a packet drops. Sent to the arm as a
  target, that drives it into its own end stop. It happened once during
  development, which is why the check exists.
- **A target that teleports is spread over several ticks.** Normal movement is
  untouched — the limit allows nearly twice the arm's full range per second, far
  more than a hand produces. It only catches jumps no human makes.
- **The arm is not throttled.** It runs at the firmware's normal speed, because a
  follower lagging behind the hand makes a worse demonstration, not a safer one.
- **The one exception is the opening sync.** When teleop starts, the follower has
  to cross from wherever it was parked to wherever the leader is being held. That
  single move is deliberately gentle, timed to the distance, and capped per tick.
  The follower also has its current position written as its goal before its motors
  switch on, so it holds still instead of snapping to a stale target from a
  previous session.
- **Replay asks before it moves**, because nobody is holding the leader.

---

## 9. Troubleshooting

### It will not start on a new computer

Run this first. It prints everything the machine has and everything it is
missing, and it is the fastest thing to send when asking for help:

```
python start.py --doctor
```

Windows: `.\.venv\Scripts\python.exe start.py --doctor`
macOS: `./.venv/bin/python start.py --doctor`

### "uv trampoline failed to spawn Python child process"

Or any variation of the project's own Python refusing to start. A `uv`
environment is a small launcher holding the **absolute path of one exact Python
build**, so it breaks when `uv` updates its Python, when the project folder is
moved or renamed, or when the folder is copied from another machine with `.venv`
included.

`start.py` now detects this and rebuilds `.venv` by itself, and both installers
check for it. To fix it by hand:

```
uv venv --python 3.12 .venv
```

then run the installer for your OS again. Never copy a `.venv` between
computers; clone the repository and install.

### CH340 driver

If no serial port shows up at all, the USB-to-serial chip has no driver.

- **Windows:** Device Manager → look for an unknown device or *USB-SERIAL CH340*
  under Ports. If it has a warning triangle, install the CH340 driver from
  WCH's site and replug.
- **macOS:** `ls /dev/cu.*` in Terminal. You want something like
  `/dev/cu.wchusbserial*`. Recent macOS includes the driver; if nothing appears,
  install WCH's CH34x driver and reboot.

### "found 1 arm(s), need 2"

A USB cable is out, or one arm is switched off at its own power switch. Both arms
need to be powered, not just plugged in.

### "Could not tell them apart"

Either two arms moved or neither did. Wave **one**, and move it far. If the arm
you hold will not budge, its motors are still on — hold it, press **Switch the
motors off**, and try again. Or power cycle that arm.

### An arm is stuck and will not move by hand

Its motors are holding position. Hold the arm, then press **Switch the motors
off** on the page, or run:

```
python station/release.py
```

It goes limp when the motors let go, so take its weight first. Still stiff? Use
the power switch.

### Cameras: none, one, or two

All three are supported. None means no video and working arms. One becomes the
front camera. Two become front and wrist; press **Swap front and wrist** if the
guess is backwards. More than two are ignored. If a configured camera stops
opening -- unplugged, or moved to another socket and renumbered -- the station
notices, looks again, and saves what is actually there.

### No picture

No camera found, or something else already has it. Close Zoom, Teams, Photo
Booth, and any other robot window. On macOS the first run asks for Camera
permission — if you missed it, System Settings → Privacy & Security → Camera, and
tick Terminal. The arms work fine without a camera.

### "COM7 is gone — a cable moved"

Correct. A cable went into a different socket, which renames the port. The page
asks for the fifteen-second wave again. Nothing is broken.

### "Something needed is not installed"

Run the installer from step 2 again.

### Recording fails on `datasets` or `av`

The recording dependencies are missing. This is what step 2 installs; older
copies of `setup.ps1` did not. Run:

```
uv pip install --python .venv/Scripts/python.exe "av>=15.0.0,<16.0.0" "datasets>=4.7.0,<5.0.0" "pandas>=2.0.0,<3.0.0" "pyarrow>=21.0.0,<30.0.0" "jsonlines>=4.0.0,<5.0.0"
```

On macOS the python path is `.venv/bin/python`. Do **not** install the newest
`av` — version 18 removed `av.option`, which LeRobot imports.

### `torchcodec is not available ... falling back to pyav`

Harmless. Video decoding uses a slower path. Nothing to fix.

### Port 8123 is busy

The station picks 8124 or 8125 instead. The URL it prints is the right one.

---

## 10. Checking without the robot

```
python station/selftest.py
```

134 checks: detection of every kind of arm, the page, the API, camera streams, the jump guard, the
railed-reading guard, the save folder, the replay guards, zero/one/two cameras,
cross-platform behaviour, and that recordings are not set to upload. It never
moves an arm.

Camera checks are skipped, with a note, when the station is already running (it
holds the cameras) or when no camera is plugged in. Neither is a failure.

---

## 11. The older command-line tools

Still present, still work, unchanged:

```
python nexarm.py setup                          find the arms and cameras
python nexarm.py teleop                         leader drives follower
python nexarm.py record --task "Pick up block"  record a dataset
python nexarm.py info                           show the saved config
python teleop_nocam.py                          teleop with no cameras at all
python probe_arms.py COM11 COM12                the original drag test
```

`nexarm.py record` goes through `examples/nexarm/record.py`, which does **not**
pass `push_to_hub=false`. The browser station does. Prefer the station, or pass
the flag yourself.

---

## 12. What each file is

| File | Does what |
|---|---|
| `start.py` | The one command. Finds things, serves the page, opens the browser. |
| `setup.ps1` / `setup.sh` | One-time install, Windows / macOS. |
| `station/detect.py` | Finds the arms and the cameras. Platform differences live here. |
| `station/hardware.py` | Owns the arms and cameras. Speed and safety limits live here. |
| `station/server.py` | Local web server, JSON API, MJPEG camera streams. |
| `station/ui.html` | The page. No frameworks, no CDN. |
| `station/release.py` | Switches the motors off. |
| `station/selftest.py` | The 134 checks. |
| `nexarm.json` | Which port is which arm, which camera is which. Machine-specific, not committed. |
| `datasets/` | Your recordings. Not committed. |
