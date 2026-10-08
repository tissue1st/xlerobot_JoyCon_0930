# xlerobot_JoyCon_0930 — Joy-Con teleop for XLeRobot 0.4.0

Two Nintendo Switch Joy-Cons driving an **XLeRobot 0.4.0** (two SO101
follower arms, 2-motor head camera, 2-wheel differential base).
`xlerobot_JoyCon_0930.py` is the single script. It grew out of
`teleop_so101_joint_nudge.py` from the `tissue1st/JoyCon` repo (single
SO101, hardware-tuned 2026-09-04) via `xlerobot_JoyCon_0929.py` (dual arm
+ wheels, 2026-09-29), and runs on Ubuntu and Windows.

> **Versions:** every improved version is a new file
> (`xlerobot_JoyCon_0930.py`, `xlerobot_JoyCon_1001.py`, …) with the same
> setup and command-line options; older files are never edited. Each
> version's changes, reasons and hardware-check status are logged in
> [`CHANGELOG.md`](CHANGELOG.md) (Korean). Latest: **1006** (2026-10-06) —
> 1001's layout (stick = pan/lift, ZR/ZL = gripper, R + right stick = camera,
> **L3 alone toggles ARM ↔ WHEEL**) with a working plus/minus wrist reset,
> a faster camera and 80 % wheel speed; **offline-tested only** so far.
> The rest of this README describes the 0930 script.
>
> **Camera script:** `xlerobot_camera_1006.py` (2026-10-06) shows the head and
> both wrist cameras in one window and labels objects with YOLOE. It runs on
> its own venv and does not touch the robot — see
> [Camera + object detection](#camera--object-detection-xlerobot_camera_1006py) below.
> Latest camera version: **`xlerobot_camera_1007b.py`** (2026-10-07) — chosen
> after comparing models with `xlerobot_model_compare_1007.py`: YOLOE-26s in
> text-prompt mode only (15 words in `PROMPT_WORDS`, `--prompt` to change),
> `desk` and `chair` blacklisted (kept in the model, hidden on screen;
> `--block` to change), confidence ≥ 0.3. No more prompt-free mode, so no wall
> labels. **Offline-tested only** (on saved snapshots) — details in
> `CHANGELOG.md`. The previous `xlerobot_camera_1007.py` (prompt-free + big
> blacklist/whitelist) is on hold.
> Latest: **`xlerobot_camera_1008.py`** (2026-10-08) — objects are drawn as
> outlines that follow their shape (from the segmentation masks) instead of
> boxes (`m` switches back to boxes), `chair` is no longer blacklisted (it is a
> prompt word now, 16 words; only `desk` is hidden), up to 15 objects per
> camera are shown, and detection is capped at 15 Hz (`--detect-hz`, 0 = no
> cap; the view still refreshes with every camera frame). Offline-tested on
> saved snapshots only.
>
> **Teleop + cameras in one script:** `xlerobot_Final_1007.py` (2026-10-07) =
> teleop 1006 + camera 1006. Run it from `.venv` as usual
> (`python xlerobot_Final_1007.py --port1 COM5 --port2 COM6`); it starts the
> camera window as a second process with `.venv-vision`'s Python. `q` in the
> camera window closes only the cameras; Ctrl+C in the terminal stops both.
> Camera options are renamed `--cam-head/--cam-left/--cam-right/--cam-list`;
> `--no-camera` gives plain teleop. **Partly verified on hardware** (runs
> together fine, 2026-10-07) — details in `CHANGELOG.md` (Final 1007).
> `xlerobot_Final_1007b.py` adds a 1.5× faster gripper. Latest:
> **`xlerobot_Final_1007c.py`** — 1007b with camera 1007b's detection
> (YOLOE-26s text prompts, 15 words, `desk`/`chair` hidden; `--prompt` /
> `--block` to change). Offline-tested only.
> Control experiment: **`xlerobot_Final_1007_newcontrol.py`** — 1007c with
> position (IK) arm control like XLeRobot's official Joy-Con example: stick
> up/down moves the gripper forward/back, X/B (L: up/down) moves it up/down,
> the wrist follows so the gripper keeps its angle, home (L: capture) returns
> the arm to its start pose. Offline-tested only.
>
> **Motion model (no robot needed):** `xlerobot_JoyCon_1008.py` (2026-10-08) is
> not a runnable teleop but a pure-logic module for the next control scheme, to
> be imported by a later `xlerobot_Final_*`: turning the Joy-Con left/right
> (the heading of its long axis) drives shoulder_pan 1:1, the stick's vertical
> axis moves the gripper forward/back and its horizontal axis up/down (IK), so
> arm motors 1-3 need no buttons. The gyro bias is measured once at startup
> (Joy-Con on the desk); pan only starts following after the first stick push,
> so picking the Joy-Con up does not swing the arm. **Not tested yet** (code,
> synthetic IMU checks and review only) — details in `CHANGELOG.md` (1008).

**Not included in this repo — install them yourself (steps below):**
Python 3.12 (conda or a venv), [lerobot](https://github.com/huggingface/lerobot)
(pinned commit `bf31dd79`), [joycon-robotics](https://github.com/box2ai-robotics/joycon-robotics)
with [XLeRobot](https://github.com/Vector-Wangel/XLeRobot)'s modified
`joyconrobotics` files, and XLeRobot's `xlerobot_2wheels` robot class
copied into lerobot. The `.gitignore` expects the three clones and
`.venv` inside this folder.

Two modes:

- **ARM mode** (startup): each Joy-Con drives its own arm exactly like the
  tuned single-arm script — buttons nudge shoulder_pan/shoulder_lift/
  elbow_flex in joint space, the gyro drives wrist_roll/wrist_flex, home
  toggles the gripper. Wheels are held at 0.
- **WHEEL mode**: both arms freeze where they are; the **left Joy-Con's
  tilt** drives the base and the **right Joy-Con's face buttons** move the
  center (head) camera. ZL is a latched brake.

**Status (2026-09-30):**

| Step | Goal | Status |
|---|---|---|
| 0 | Standalone copy of `teleop_so101_joint_nudge.py` (helpers inlined) | done |
| 1 | Two Joy-Cons → two arms on XLeRobot's `xlerobot_2wheels` class | **works on hardware** (Windows, 2026-09-30: calibration, zeroing, ARM-mode control of both arms) |
| 2 | ARM / WHEEL modes (L3 / R3), left-tilt driving, latched ZL brake, head camera on right X/B/Y/A, watchdog, LEDs, `--wheel-dry-run` | **partly verified on hardware**: L3 → WHEEL and forward/back driving work. Turning was dead in the real grip → turn axis re-mapped from a measured log (untested since). Open: R3 → ARM didn't switch back in the first run, although a Joy-Con-only log shows every R3 press arriving |
| — | Wrist/center camera image streams | out of scope; this script doesn't touch cameras |

### Camera + object detection (`xlerobot_camera_1006.py`)

Separate from the teleop: opens the cameras only (no motor bus, no Joy-Cons).
Uses its own venv `.venv-vision` because it needs the CUDA build of torch,
while the teleop `.venv` has the CPU build that lerobot pins. Versions are kept
inside lerobot's limits (torch 2.11, numpy < 2.3, OpenCV < 4.14) so the two can
be merged into one venv later. RTX 50-series GPUs need a CUDA 12.8+ build.

```powershell
uv venv .venv-vision --python 3.12
uv pip install --python .venv-vision/Scripts/python.exe torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv-vision/Scripts/python.exe ultralytics "numpy==2.2.6" "opencv-python==4.13.0.92"

.venv-vision\Scripts\python xlerobot_camera_1006.py --list    # which index is which camera
.venv-vision\Scripts\python xlerobot_camera_1006.py           # 3 cameras + prompt-free detection
.venv-vision\Scripts\python xlerobot_camera_1006.py --prompt "cup,bottle"   # only these objects
.venv-vision\Scripts\python xlerobot_camera_1006.py --no-detect             # cameras only
```

- Default indices (dev PC, 2026-10-06): head = 4, left wrist = 2, right wrist = 3
  (three identical `USB2.0_CAM1`, 640x480). Windows renumbers cameras when they
  are re-plugged; run `--list`, cover one lens and see which brightness drops,
  then pass `--head/--left/--right`.
- Model: YOLOE-26s-seg (ultralytics), boxes + outlines. The weights download on
  the first run. About 35–40 ms for all three cameras on an RTX 5050 Laptop.
- Keys in the video window: `q` quit, `d` detection on/off, `m` outlines on/off,
  `s` snapshot to `captures/`.
- Display is de-cluttered: objects must be seen for a few frames before they
  appear and are held briefly when missed; confidence ≥ 0.4, at most 10 per
  camera, non-object words (`IGNORE_WORDS`) dropped. Details in `CHANGELOG.md`.

### Setup — Ubuntu

```bash
cd xlerobot_JoyCon_0930                      # this repo
conda create -y -n lerobot python=3.12 && conda activate lerobot
conda install -y ffmpeg -c conda-forge

git clone https://github.com/huggingface/lerobot.git
git -C lerobot checkout bf31dd794ffb4f87380aba3912f64421e8352d3c
git clone https://github.com/box2ai-robotics/joycon-robotics.git
git clone https://github.com/Vector-Wangel/XLeRobot.git   # must include 9f664e87 (2026-09-18 wheel fix)

pip install -e "./lerobot[feetech]" "pyzmq>=26.2.1,<28.0.0" hidapi pyglm scipy
pip install --no-deps -e ./joycon-robotics
cp XLeRobot/software/joyconrobotics/*.py joycon-robotics/joyconrobotics/
cp -r XLeRobot/software/src/robots/xlerobot_2wheels lerobot/src/lerobot/robots/

# Linux Joy-Con driver (from joycon-robotics' README)
sudo apt-get install -y dkms libevdev-dev libudev-dev cmake gcc-12 joycond
(cd joycon-robotics && make install)
sudo systemctl enable --now joycond
```

Notes:
- **XLeRobot must be recent**: it fixed the left-wheel direction on
  2026-09-18 (commit `9f664e87`); an older copy turns "forward" into
  spinning in place. The script checks this before connecting and refuses
  to run with an old copy (re-pull and re-copy).
- `xlerobot_2wheels/__init__.py` imports its ZMQ host/client, so `pyzmq`
  is required even though this script doesn't use ZMQ.
- joycon-robotics goes in with `--no-deps` because its `setup.py` lists
  both `hidapi` and `hid==1.0.4`, two packages that install the same
  `hid` module (the script works with either, but mixing them breaks).
- `make install` builds the `hid-nintendo` DKMS module with the compiler
  that built your kernel; if it fails with `gcc-12: not found`, install
  the matching `gcc-XX`.
- Serial ports default to `/dev/ttyACM0` (`--port1`) and `/dev/ttyACM1`
  (`--port2`); both boards look identical, so check which is which.
- If a Joy-Con script fails with `'NoneType' object is not subscriptable`
  inside joyconrobotics, the Joy-Con went to sleep / dropped Bluetooth —
  press any button on it and retry.

The script catches the missing-setup cases it can detect (no
`xlerobot_2wheels` in lerobot, no `zmq`, old XLeRobot copy, no
calibration file) and prints the exact command to run.

Motor layout it expects (from XLeRobot's `xlerobot_2wheels` code):
`port1` bus = left arm (ids 1–6) + head (7 = `head_motor_1` = pan,
8 = `head_motor_2` = tilt); `port2` bus = right arm (ids 1–6) + wheels
(9 = left, 10 = right). The pan/tilt roles follow the *code* of
XLeRobot's `7_xlerobot_2wheels_teleop_joycon.py` (its comments have 1/2
swapped).

Pair **both** Joy-Cons, then press L (left) + R (right) together until
each shows only its first LED lit (XLeRobot's recommended pairing check).

### Setup — Windows (tested on the dev PC 2026-09-30: Joy-Cons, calibration, arms, wheels)

Everything above except the Linux-only Joy-Con driver works on Windows:
lerobot/Feetech talk to the arms over COM ports, and joyconrobotics
talks to the Joy-Cons through `hidapi`, which Windows supports directly
(no `hid-nintendo` DKMS module, no `joycond`, no `make install`).

**Installed this way on the dev PC (2026-09-30)** — everything lives
inside this folder (clones + a Python 3.12 venv, all git-ignored by the
`.gitignore` here); nothing system-wide except `uv`. Run in PowerShell
from this folder:

```powershell
python -m pip install uv                              # any system Python; used only to make the venv
python -m uv venv --python 3.12 .venv                 # lerobot needs >= 3.12 (uv downloads it if missing)
git clone https://github.com/huggingface/lerobot.git
git -C lerobot checkout bf31dd794ffb4f87380aba3912f64421e8352d3c   # same lerobot as the Ubuntu setup / the reviewed one
git clone https://github.com/box2ai-robotics/joycon-robotics.git
git clone https://github.com/Vector-Wangel/XLeRobot.git          # must include 9f664e87 (2026-09-18 wheel fix)
python -m uv pip install --python .venv\Scripts\python.exe -e ".\lerobot[feetech]" "pyzmq>=26.2.1,<28.0.0" hidapi pyglm scipy
$env:PYTHONUTF8 = "1"                                 # joycon-robotics' setup.py can't read its README under cp949
python -m uv pip install --python .venv\Scripts\python.exe --no-deps -e .\joycon-robotics
Remove-Item Env:PYTHONUTF8
Copy-Item -Force XLeRobot\software\joyconrobotics\*.py joycon-robotics\joyconrobotics\
Copy-Item -Recurse -Force XLeRobot\software\src\robots\xlerobot_2wheels lerobot\src\lerobot\robots\
```

Why these choices:
- `lerobot[feetech]` only — `core_scripts` (datasets, rerun, …) isn't
  needed by this script. torch still comes with lerobot's base install.
- joycon-robotics with `--no-deps`: its `setup.py` lists **both**
  `hidapi` and `hid==1.0.4`, two packages that install the same `hid`
  module; the `hid` one also needs a separate `hidapi.dll` on Windows.
  Installing its real dependencies (`hidapi pyglm scipy`) by hand keeps
  the working one. (The script handles either `hid` module anyway.)

Run everything with the venv's Python — either activate it
(`.venv\Scripts\activate`) and use `python`, or call
`.venv\Scripts\python.exe xlerobot_JoyCon_0930.py ...` directly.

Then pair **both** Joy-Cons in Settings → Bluetooth (hold the sync
button until the lights run) and check them **before anything else**:

```bat
python xlerobot_JoyCon_0930.py --check-joycons      # with the venv activated
```

It lists each Joy-Con with its serial number and whether joyconrobotics
will accept it (`both Joy-Cons usable.` is what you want). On Windows the
serial comes back as the bare Bluetooth MAC (`a05a5fc68e84`), which
joyconrobotics rejects; the script re-inserts the colons
(`a0:5a:5f:c6:8e:84`, exactly what Linux reports) before joyconrobotics
sees it, so nothing in the library needs changing. Tested on real
Joy-Cons: found, accepted, IMU and buttons read, player LEDs set.

Joy-Cons go to sleep and drop off Bluetooth after a while idle — press
any button to reconnect before starting the script.

On Windows the serial ports have no default — pass them explicitly
(Device Manager → Ports shows which COM each board is; unplugging one
board and re-running a command shows which one disappeared):

```bat
python xlerobot_JoyCon_0930.py --port1 COM3 --port2 COM4 --wheel-dry-run
```

(A missing `--port1`/`--port2` prints the COM ports it can see.) The
calibration file lives under `%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\xlerobot_2wheels\`
— the script prints the exact path if it's missing. Stopping works the
same (Ctrl+C); Ctrl+Z/SIGHUP handling is Linux-only.

### Run (Ubuntu paths; on Windows add `--port1 COMx --port2 COMy`)

```bash
conda activate lerobot                       # Windows: .venv\Scriptsctivate
cd xlerobot_JoyCon_0930
python xlerobot_JoyCon_0930.py --check-joycons                   # Joy-Con pairing/serial check only
python xlerobot_JoyCon_0930.py --wheel-dry-run                   # FIRST RUN: wheels never move, commands are printed
python xlerobot_JoyCon_0930.py                                   # both arms + wheel mode
python xlerobot_JoyCon_0930.py --arms right                      # right arm only (no wheel mode)
python xlerobot_JoyCon_0930.py --port1 /dev/ttyACM0 --port2 /dev/ttyACM1 --robot-id my_xlerobot_2wheels
python xlerobot_JoyCon_0930.py --robot so101 --port /dev/ttyACM0 # old single-SO101 bench rig (no wheel mode)
python xlerobot_JoyCon_0930.py --robot-id <id> --calibrate       # no calibration file yet: run XLeRobot's calibration
# Windows example (dev PC): python xlerobot_JoyCon_0930.py --port1 COM5 --port2 COM6 --wheel-dry-run
```

**Calibration backup:** this robot's calibration (2026-09-30) is kept in
[`calibration/xlerobot_2wheels/my_xlerobot_2wheels.json`](calibration/xlerobot_2wheels/my_xlerobot_2wheels.json).
On a new PC (or after the cache was wiped), copy it to
`%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\xlerobot_2wheels\`
(Ubuntu: `~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/`)
and press ENTER at the restore prompt -- no manual calibration needed. It is
specific to this robot: after replacing a motor or re-assembling an arm,
recalibrate and commit the new file here.

Calibration: `--robot-id` names the file
`~/.cache/huggingface/lerobot/calibration/robots/xlerobot_2wheels/<id>.json`
(default id `my_xlerobot_2wheels`; XLeRobot's own direct-control
examples use `my_xlerobot_2wheels_lab`). If it exists, XLeRobot's class
prompts ENTER = restore from file, `c` = recalibrate. If it doesn't, the
class would go straight into a full manual calibration that rewrites
every arm/head motor's homing offset — so the script stops instead,
lists the ids it did find, and only calibrates with `--calibrate`. The
head's calibrated range midpoint becomes the camera center that right
plus returns to. XLeRobot's calibration writes each motor register
without retries, so one lost status packet (`There is no status
packet!`) used to abort the whole calibration before anything was saved
(happened once on 2026-09-30, with the wiring and 12.6 V supply fine);
the script now retries those writes up to 3 times. If it still fails,
the file isn't saved — just rerun the same command.

Then any leftover wheel velocity is cleared, each arm's nudge range is
printed from its calibration, the Joy-Cons connect (hold them still ~2s
each while the gyro calibrates — they ignore all input until teleop
starts), both arms move to zero, and the loop runs in ARM mode.

Stopping: Ctrl+C (also SIGTERM, a closed terminal/SIGHUP, and Ctrl+Z,
which stops instead of suspending) only sets a flag; the loop finishes
its current tick, then the base is stopped (clearing any stuck serial
port state first, with retries) and everything disconnects (which
releases arm torque, as before). Only SIGKILL or a power cut skips
this — the wheel servos keep their last velocity then.

### Controls

Mode switch (both Joy-Cons required):

| Input | Function |
|---|---|
| L3 (left stick click) | → WHEEL mode |
| R3 (right stick click) | → ARM mode |

Absolute, not a toggle: pressing either one again never flips back. If
both are pressed together, ARM wins. L3 is ignored while the right
Joy-Con isn't responding (R3 is the only way back).

**ARM mode** — per Joy-Con (right / left):

| Input (R / L) | Function |
|---|---|
| Y / A  ·  left / right | shoulder_pan − / + |
| B / X  ·  down / up | shoulder_lift + / − |
| ZR / R  ·  ZL / L | elbow_flex + / − |
| home  ·  capture | gripper toggle |
| plus  ·  minus | recenter wrist_flex/wrist_roll to 0 |
| gyro gx / gy | wrist_roll / wrist_flex |
| stick (tilt), SL, SR | unused |

**WHEEL mode** — arms, grippers and wrist values stay frozen and resume
from exactly there on return to ARM mode:

| Joy-Con | Input | Function |
|---|---|---|
| left | tilt forward / back | drive forward / back (`x.vel`) |
| left | tilt left / right | turn in place left / right (`theta.vel`); combined with forward/back = curve |
| left | ZL | brake toggle — latched; pressing again releases it and re-captures the neutral pose |
| right | X / B | center camera up / down (`head_motor_2`) |
| right | Y / A | center camera left / right (`head_motor_1`) |
| right | plus | center camera back to its calibrated center (0) |
| both | everything else | ignored |

How the tilt works: on entering WHEEL mode (and on brake release) the
script waits 0.3 s, then records the left Joy-Con's current pose as
**neutral** — hold it the way you hold it in ARM mode. Forward/back and
left/right are read from fixed Joy-Con axes, tuned on 2026-09-30 from a
60 s log of the real left Joy-Con in the user's grip: at neutral gravity
points along the Joy-Con's local **−Z** (g ≈ (−0.20, +0.09, −0.97)),
forward/back tilt rotates about local **Y** (−35° / +57° measured) and
left/right tilt about local **X** (left −60°, right +46°). So
`WHEEL_FORWARD_AXIS = 1` (Y) and `WHEEL_TURN_AXIS = 0` (X) with
`WHEEL_TURN_SIGN = -1` (left tilt → +theta = turn left); `WHEEL_FORWARD_SIGN = -1` too, since the measured forward tilt was −Y. The first
version read the turn from Z, which only moved ±12° in this grip — just
past the 10° deadzone — so turning looked dead. A very different grip
(e.g. with gravity along X) would need other axes. Tilt is measured from the
accelerometer's gravity direction relative to that pose (absolute, no
drift): within ±10° nothing moves, ±10–30° ramps linearly up to the max
speed (0.1 m/s, 30 °/s — XLeRobot's own "slow" level), and more than 75°
total (e.g. the controller put down, ~90°) stops the base. (Was 60°; real
left turns were tilted ~60° and got cut off.) Twisting the
controller about the vertical axis does nothing (gravity can't see that
rotation), which is why turning is a sideways tilt.

Safety:
- **Watchdog** — if a Joy-Con delivers no input report for 0.3 s, its arm
  input is paused (a dropped Bluetooth link otherwise leaves the last
  report, maybe with a button held, frozen in place), and in WHEEL mode
  the brake is engaged and **stays engaged** until ZL is pressed. That
  only works after a short stall: if the Joy-Con really disconnected,
  joyconrobotics never reopens it (its reader threads die with
  `OSError: read error` tracebacks), so re-pairing does NOT bring it
  back — stop with Ctrl+C and restart.
- LEDs: ARM = LED 1 on, WHEEL = all 4 on, WHEEL + brake = all 4
  flashing. (If an LED write ever fails, the script prints one warning
  and carries on without LEDs.)
- The `[dbg WHEEL]` line (~2×/s) shows the measured tilt, the wheel
  command and the camera targets.

### Known joyconrobotics bug the script works around (all platforms)

joyconrobotics opens each Joy-Con three times, and every handle receives
every subcommand reply — including the ACKs of the previous handle's
setup commands. Its calibration read takes the first reply it sees, so a
handle can load its IMU calibration shifted by one reply. On real
Joy-Cons this hit **3 of 10 connections** (e.g. left accel X = 144 g,
gyro X = 3×10⁷ rad/s — tilt driving and the wrists would be garbage).
The script reads the calibration itself with reply checking before
joyconrobotics connects, compares it with what the IMU handle loaded, and
on a mismatch loads the right values and re-measures the gyro bias —
you'll see `IMU calibration repaired` and a ~2 s pause; keep the Joy-Con
still then. Verified on real Joy-Cons: all 10 connections read correctly
afterwards.

### First hardware run checklist

Run with `--wheel-dry-run` first on a new setup and check, in this order:

1. **ARM mode** behaves like the single-arm script on the right arm, and
   the left arm mirrors it. If the left wrist moves the wrong way, flip
   the matching entry in `WRIST_GYRO_SIGN["left"]` (`(roll_sign,
   flex_sign)`). joyconrobotics already mirrors the left Joy-Con's Y/Z
   axes into the right one's frame, so `(+1, +1)` is the expected value.
2. **L3** → WHEEL mode (LEDs all on). Tilt the left Joy-Con **forward**:
   the `[dbg WHEEL]` line's `x.vel` must be **positive**; tilt it **left**:
   `theta.vel` must be **positive** (+ = turn left). If either is
   negative, flip `WHEEL_FORWARD_SIGN` / `WHEEL_TURN_SIGN`. If the
   forward tilt shows up in `turn` instead of `fwd`, swap
   `WHEEL_FORWARD_AXIS` / `WHEEL_TURN_AXIS`. If one direction stays near
   0 in both `fwd` and `turn` whatever you do, your grip puts that motion
   on the third axis — log it (see the 2026-09-30 note above) and set the
   axis index accordingly.
3. **Center camera**: X should tilt it up, Y pan it left. Flip
   `HEAD_TILT_UP_SIGN` / `HEAD_PAN_LEFT_SIGN` if the direction is
   reversed; if X/B pans instead of tilting, swap `HEAD_TILT_MOTOR` /
   `HEAD_PAN_MOTOR`.
4. **ZL** brake: LEDs flash, `[dbg WHEEL] BRAKE`. **R3** → ARM mode.
5. Then run without `--wheel-dry-run` — first with the base lifted
   (wheels off the ground): a forward tilt must spin both wheels
   forward, a left tilt must turn it left.

- `use_degrees=True` is set on `XLerobot2WheelsConfig` on purpose — the
  upstream default is −100..100 normalization, which would silently break
  every degree constant in the script (lerobot's own SO101 config defaults
  to degrees, which is what the single-arm tuning used). Don't remove it.
- All wheel/head/watchdog numbers (`WHEEL_*`, `HEAD_*`,
  `JOYCON_WATCHDOG_S`) are first guesses made without hardware — tune by
  feel. If the watchdog brakes spuriously on a noisy Bluetooth link, raise
  `JOYCON_WATCHDOG_S` (e.g. 0.5).
