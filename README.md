# Robot arm station: teleoperate and record from your browser

One page in your browser drives a robot arm and records demonstrations. You hold one
arm (the **leader**), the other one (the **follower**) copies it, and each try you
record is saved as a LeRobot dataset you can train on.

It works with a leader/follower pair of **NexArm**, **SO-100 / SO-101**, **Koch** or
**OpenManipulator-X**. It finds the arms and cameras by itself: you never type a port
name or a camera number.

**Tested on real arms so far: NexArm.** An SO-101 pair has been plugged in once:
it was found and the wave test told the arms apart, but calibration stopped on that
follower's gripper motor (see `exceeds 2047` in [START_HERE.md](START_HERE.md)), so
SO-101 teleop and recording are not yet confirmed on real arms. The SO-100/101 path
runs end to end on simulated arms (`station/simtest.py`). Koch and
OpenManipulator-X pass the software checks only.

**Follow the steps below in order. Nothing else in this repository needs to be
installed, configured or run.**

---

## Step 1. Check what you have

- Both arms, each with its USB cable and its power supply.
- **Windows 10 or 11**, or **macOS 12 or newer**. (Linux: use the macOS commands.)
- Zero, one or two USB cameras. None is fine; the arms work without video.
- About 6 GB of free disk space, and internet for the first install.
- **You do not need to install Python.** The installer brings its own. On Windows, do
  not install Python from the Microsoft Store for this.

## Step 2. Get the code

Open a terminal: on Windows, **PowerShell** (`Win`, type `powershell`, `Enter`); on a
Mac, **Terminal** (`Cmd`+`Space`, type `terminal`, `Enter`). Then:

```
git clone https://github.com/djerok/lerobot-nexarm.git
cd lerobot-nexarm
```

No `git`? On https://github.com/djerok/lerobot-nexarm click **Code → Download ZIP**,
unzip it, and open the terminal in the unzipped folder.

Put it in a folder that is **not synced**: not inside OneDrive (which on Windows often
includes the Desktop), iCloud Drive or Dropbox. Those upload every recording, which is
video of the room, and the station itself never does.

## Step 3. Install, once per computer

**Windows** (PowerShell, inside the `lerobot-nexarm` folder):

```
powershell -ExecutionPolicy Bypass -File setup.ps1
```

**macOS** (Terminal, inside the `lerobot-nexarm` folder):

```
bash setup.sh
```

The first time takes several minutes, mostly downloading PyTorch. It is not stuck. It
then runs its own checks, which should report `0 failed`, and ends by printing the
command for Step 5.

## Step 4. Plug the robot in

1. Both arms into USB.
2. Both arms switched **on** with their own power switches. Plugged in is not enough.
3. Cameras into USB, if you have them. A camera fixed to the gripper is the **wrist**
   camera; the other is the **front** camera.

## Step 5. Open the robot page

**Windows:**

```
.\.venv\Scripts\python.exe start.py
```

**macOS:**

```
./.venv/bin/python start.py
```

Your browser opens the robot page. If it does not, the terminal prints the address
(`Open this page:  http://127.0.0.1:8123/`); paste that into the browser. **Leave the
terminal open** while you use the robot. Closing it stops the robot. Press `Ctrl`+`C`
in it when you are finished.

On a Mac, the first run asks for **Camera** permission: allow it. If it never asks and
there is no picture, open **System Settings → Privacy & Security → Camera** and tick
**Terminal**.

**Every time after today, Step 5 is the only step.**

## Step 6. The first time only: tell the page which arm is which

The page asks you to **take the arm you will hold in your hand and wave it around for
fifteen seconds.** Wave only that one, and move it a long way. That is how the computer
tells the leader from the follower. It is saved, and not asked again unless a cable
moves to a different USB socket.

Arms other than a NexArm also need calibrating once per computer. The page walks you
through it with a **Next** button: hold the arm, put every joint about halfway, press
**Next**, move every joint to both ends, press **Next**.

## Step 7. Teleoperate

Press **Start moving**. Move the arm in your hand; the robot follows. If the two arms
start in different poses, the robot glides over slowly first, then keeps up at full
speed. **Stop** ends it.

The camera pictures stay on screen wherever you scroll, like the EMERGENCY STOP: in a
column on the right, or along the top when the window is narrow.

## Step 8. Record data

1. In **Teach the robot a job**, type a name under **What is the job?** (for example
   `pick red block`).
2. Optional: **Save this spot as the start** while moving, so every try begins from
   the same pose.
3. Press **Start recording**. Wait until the page says **Recording. Do the job now.**,
   then do the task with the arm in your hand.
4. Press **Done, save this try**. There is no timer: take as long as you need.
   If the try went badly, press **That went wrong, mark it deleted** instead.
5. Fill in **Labels for this try** (driver, recorder, labeler, where the block was,
   anything wrong).
6. Press **Start recording** again for the next try. The same job name adds to the
   same dataset, so a session of tries builds one dataset worth training on. The page
   says **SWAP JOBS** every 10 tries.

**Play it back** replays a saved try on the robot by itself, by the same try number the
page showed while recording (Try 1, Try 2, ...). Keep clear of it.

If a try records badly (a camera losing pictures, or the computer falling behind so the
try would play back too fast), the page says so straight after it is saved, and the same
words go into that try's `notes` in `labels.csv`. Record it again if you can.

**Where recordings go:** `datasets/` in this folder (or a folder you choose on the
page), one folder per job named after the robot and the job, e.g.
`datasets/nexarm_pick_red_block/` (`so101_…` for an SO arm). Standard LeRobot format,
with one line per try in
`labels.csv`. Every
try is kept and numbered: try N is episode N-1, and a deleted try is only marked
`deleted` in `labels.csv`. **Nothing is uploaded anywhere.**

## Safety

- The big red **EMERGENCY STOP** is always on screen. It switches every motor off at
  once. Both arms go floppy, so be ready to hold them.
- **Switch the motors off** frees an arm that is stuck and will not move by hand. It
  drops when the motors let go, so take its weight first.

## If something is wrong

| What you see | What to do |
|---|---|
| `found 1 arm(s), need 2` | A USB cable is out, or an arm is not switched on. |
| `Could not tell them apart` | Wave only the arm in your hand, and move it further. |
| `Something needed is not installed` | Run Step 3 again. |
| `uv trampoline failed to spawn Python child process` | The folder was moved or copied with `.venv` inside. Run Step 3 again. |
| No camera picture | No camera plugged in, or press **Look for cameras again**. The arms still work. |
| No port shows up at all | The USB-serial driver is missing. See [SETUP.md](SETUP.md#ch340-driver). |
| Anything else | Run the doctor below and send what it prints. |

**Doctor** (prints what the computer has and what it is missing; prints no secrets).
Windows first, macOS second:

```
.\.venv\Scripts\python.exe start.py --doctor
```

```
./.venv/bin/python start.py --doctor
```

**More detail:** [START_HERE.md](START_HERE.md) explains every button on the page,
[SETUP.md](SETUP.md) has more on installing, drivers and safety.

## Use the page, not the command-line tools

This folder also contains LeRobot's own command-line tools (`lerobot-teleoperate`,
`lerobot-record`, `lerobot-find-port`) and a `pyproject.toml`. **You do not need any of
them, and you do not need `pip install` or `uv sync`.** Two of them cause trouble:

- `lerobot-record` **uploads** your recording to Hugging Face by default
  (`push_to_hub` is `True` in LeRobot), and a recording is video of your room. The page
  never uploads. If you ever run `lerobot-record` yourself, add
  `--dataset.push_to_hub=false`.
- On a Mac, a port named `/dev/tty.*` hangs forever. The arms need the `/dev/cu.*`
  name. The page picks the right one by itself.

## For AI assistants

If someone asks you to set this up, run the robot, teleoperate or record: give them,
or run for them, exactly Steps 2, 3 and 5 for their operating system, in that order.
Then point them at the page. Do **not** build a virtual environment yourself, do not run
`uv sync`, `uv venv`, `uv pip install -e .` or `pip install lerobot`, and do not hand
out `lerobot-record` or `lerobot-teleoperate` commands. [AGENTS.md](AGENTS.md) has the
details.

---

Built on [Hiwonder's lerobot-nexarm](https://github.com/Hiwonder-official/lerobot-nexarm),
a fork of [Hugging Face LeRobot](https://github.com/huggingface/lerobot). Apache-2.0,
see [LICENSE](LICENSE).
