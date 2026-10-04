# Start here

One command runs the robot:

```
python start.py
```

That is the whole thing. It finds the arms, finds the cameras, and opens a page in
your browser. Everything else happens on that page: move the arm, record a job,
switch the motors off if something gets stuck.

You never type a COM port or a camera number.

## Which robots

Any leader/follower arm pair: **NexArm**, **SO-100 / SO-101**, **Koch**,
**OpenManipulator-X**, and any other single-arm pair LeRobot ships whose motor
software is installed. The page works out which one is plugged in from the USB
chip and by talking to the motors (it never moves them to find out). If it guesses
wrong, pick the robot from the list on the first screen.

**The first time a non-NexArm arm is used on a computer**, LeRobot needs to
calibrate it. The page walks through it with a **Next** button: hold the arm (its
motors are off), put every joint about halfway, press Next, then move every joint
to both ends and press Next. That is saved, and never asked again on that computer.
SO arms are remembered by their USB adapter's serial number, so moving a cable to
another socket does not mean doing the wave again.

## The first time

1. Plug both arms into the computer with their USB cables.
2. Switch both arms on.
3. Run `python start.py`.
4. The page asks you to **wave the arm you hold in your hand** for fifteen seconds.
   Wave only that one, and move it a long way.

That fifteen seconds is how the computer works out which arm is which. It cannot
be skipped and it cannot be guessed: both circuit boards look identical to the
computer, right down to having no serial number, and both answer the same
questions the same way. The only difference between them is that the one you hold
is loose and the other one holds itself still -- so the way to tell them apart is
to watch which one a person can move.

The answer is saved. Next time, `python start.py` goes straight to the page.

## On the page

| | |
|---|---|
| **Start moving** | The arm you hold is the leader. The other copies it. |
| **Stop** | Ends whatever is running. |
| **Start recording** | Do the job with the leader arm; the robot writes down what it sees and what you did. |
| **Done, save this try** | Saves the try. There is **no timer** -- take as long as you need. |
| **That went wrong, do it again** | Throws this try away and starts it over. |
| **Play it back** | Replays a saved try on the arm by itself. Keep clear. |
| **Swap front and wrist** | The two cameras were guessed the wrong way round. |
| **Switch the motors off** | An arm is stuck and will not move by hand. |

## Things worth knowing

**One try per press, and no clock.** Recording runs until you press Done. Press
Start again for the same job and the next try is added to the same dataset, so
repeated goes build up something worth training on.

**Any number of cameras from zero to two.** None is fine -- the arms still work,
there is just no video. One becomes the front camera. Two become front and wrist,
and if they are the wrong way round there is a Swap button. If a camera is
unplugged mid-session the station notices and looks again by itself.

**Recordings stay on this computer.** They are written to `datasets/` in this
folder, or a folder you choose on the page, and go nowhere else. This is not the default behaviour of the underlying
tool -- `lerobot` publishes to a public Hugging Face dataset unless told not to,
and a recording is video of the room. If you ever record with the plain
`lerobot-record` command instead of this page, pass
`--dataset.push_to_hub=false` yourself.

**The arm runs at normal speed, except for the first sync.** When teleop starts,
the follower has to cross from wherever it was parked to wherever you are holding
the leader. That one move is deliberately gentle and takes a few seconds. After
that it keeps up with your hand.

**A target that teleports is spread over several ticks.** Normal movement is
untouched. Only jumps no hand could make are caught.

**A reading that looks broken is ignored, never obeyed.** The boards occasionally
return a railed value (0 or 4095) on a dropped packet. Sent to the arm as a target
that drives it into its own end stop, so those readings are dropped instead.

**If an arm will not budge**, its motors are still holding position. Hold the arm,
then press *Switch the motors off* on the page, or run:

```
python station/release.py
```

It goes floppy when the motors let go, so take its weight first. If it is still
stiff after that, use the power switch.

**If a cable moves to a different USB socket**, the page notices and asks you to
do the fifteen-second wave again. Nothing is broken.

## When something will not work

| What you see | What it means |
|---|---|
| `found 1 arm(s), need 2` | A USB cable is out, or one arm is switched off. |
| `Could not tell them apart` | Two arms moved, or neither did. Wave one, further. |
| `COM7 is gone -- a cable moved` | Correct, and it is about to ask you to redo the wave. |
| No picture | No camera plugged in. The arms still work without one. |
| `Something needed is not installed` | Run `setup.ps1` first. |

## Checking the software without the robot

```
python station/selftest.py
```

Runs 134 checks: detection of every kind of arm, the page, the API, the camera streams, the speed limit,
the railed-reading guard, and that recordings are not set to upload. It needs the
cameras but never moves an arm.

## What is in here

| File | Does what |
|---|---|
| `start.py` | The one command. Finds things, serves the page, opens the browser. |
| `station/detect.py` | Finds the USB ports and the cameras. |
| `station/robots.py` | Every kind of arm: recognising it, the wave test, its safety numbers. |
| `station/hardware.py` | Owns the arms and cameras. Speed limits live here. |
| `station/server.py` | The local web server and the camera streams. |
| `station/ui.html` | The page. |
| `station/release.py` | Switches the motors off. |
| `station/selftest.py` | The checks above. |

The older command-line tools still work and are unchanged:
`nexarm.py`, `nexarm_autoconfig.py`, `teleop_nocam.py`, `probe_arms.py`.
