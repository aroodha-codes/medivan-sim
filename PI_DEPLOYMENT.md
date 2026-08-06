# MediVan — Raspberry Pi Deployment Guide

From a Raspberry Pi 4 still in its box to a robot that maps, delivers and
docks. No prior Raspberry Pi experience assumed.

Work top to bottom. Each section ends with a check; if the check fails,
fix it before moving on. Skipping ahead makes later failures very hard to
attribute — one wrong step usually shows up three sections later as
something that looks unrelated.

**About this guide.** Every command was verified against the project files
as they stand. Commands that could only be verified on x86 — anything that
touches the camera, the IMU or the motors — are flagged as such where they
appear. Nothing here has been run on a physical Pi.

Companion documents, referenced rather than repeated:

| Document | Covers |
|---|---|
| `docs/HARDWARE_SETUP.md` | full wiring, pin map, dock geometry |
| `docs/CALIBRATION.md` | calibration theory, quality criteria, troubleshooting |
| `docs/DOCKING_TEST_PROCEDURE.md` | the eight-stage physical docking test |
| `STATUS.md` | what is measured, what is a known limitation |

---

## 1. Hardware requirements

| # | Component | Notes |
|---|---|---|
| 1 | Raspberry Pi 4 Model B, **4 GB** | 2 GB is tight once OpenCV is loaded |
| 2 | Pi Camera Module V2 (IMX219) | the only exteroceptive sensor — see §6 |
| 3 | MPU6050 IMU | yaw fusion; the one part of the pose estimate that does not drift |
| 4 | L298N motor driver | dual H-bridge |
| 5 | 2× 18650 Li-ion cells | with holder |
| 6 | TP4056 charging module | |
| 7 | 5 V buck converter | powers the Pi; **not** the L298N's onboard regulator |
| 8 | 2WD differential robot chassis | 2 gear motors, castor, wheels |
| 9 | microSD card, **32 GB**, class 10 / A1 | 16 GB works but leaves little room |
| 10 | USB-C power supply, 5 V 3 A | for bench work before the battery pack |
| 11 | Printed ArUco marker, 100 mm | generated in §10 |
| 12 | Pi Camera ribbon cable | the one in the camera box is usually short |
| 13 | Jumper wires (F–F, M–F) | |
| 14 | Checkerboard print | for §9 — plain paper is fine |

Useful but not required: a steel rule (to check the marker print), a USB
keyboard and HDMI cable for first boot if you'd rather not use SSH, and a
second computer to SSH from.

**The hardware list is fixed.** Do not substitute a different camera or add
wheel encoders — the software is built and measured against exactly this
set, and encoders in particular would change conclusions recorded in
`EVALUATION_REPORT.md`.

---

## 2. Flash Raspberry Pi OS

On your laptop, not the Pi.

1. Download **Raspberry Pi Imager** from
   <https://www.raspberrypi.com/software/> and install it.
2. Insert the microSD card.
3. Open Imager:
   - **Choose device** → Raspberry Pi 4
   - **Choose OS** → Raspberry Pi OS (64-bit)
   - **Choose storage** → your SD card
4. Click the **gear / Edit Settings** button before writing. This is the
   easy moment to set things up; doing it afterwards is fiddlier.

   - Set hostname: `medivan` (then the Pi answers to `medivan.local`)
   - Set username and password — **write these down**
   - Configure wireless LAN: your SSID, password, and the right country
     code. The country code matters; get it wrong and Wi-Fi may not
     associate at all.
   - Set locale and timezone
   - **Services** tab → tick **Enable SSH** → *Use password authentication*

   SSH and Wi-Fi are optional if you plan to work with a keyboard and
   monitor plugged into the Pi. For a robot that will be driving around,
   you want both.

5. **Write**, wait, then eject.

Use the 64-bit OS. Some Python wheels are no longer published for 32-bit
ARM, and you will hit that during §7.

---

## 3. First boot

Insert the card, connect the camera ribbon (Pi powered **off** — see §11),
and apply power. First boot takes a couple of minutes.

Connect over SSH from your laptop:

```bash
ssh <your-username>@medivan.local
```

If `medivan.local` does not resolve, find the Pi's IP address on your
router's admin page and use that instead.

Then, on the Pi:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

**Why this is required.** The OS image was built months before you
downloaded it. `apt update` refreshes the list of available packages;
`full-upgrade` installs the newer versions, including kernel and firmware
updates. On a Pi that matters more than on a desktop, because the camera
and I2C drivers live in the kernel and firmware — installing packages
against a stale kernel is a common cause of "the camera worked yesterday"
faults. Do this once, up front, and the rest of the guide sits on a known
base.

It can take 10–20 minutes on a fresh image. Let it finish.

---

## 4. Install system packages

```bash
sudo apt install -y \
  git \
  python3 \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-opencv \
  python3-picamera2 \
  python3-smbus \
  libatlas-base-dev \
  i2c-tools \
  build-essential
```

What each is for:

| Package | Why |
|---|---|
| `git` | cloning the project |
| `python3-venv`, `python3-dev` | virtual environment, and headers for building any wheel that has no ARM binary |
| `python3-opencv` | OpenCV from Debian. Much faster than pip, which may try to compile OpenCV from source and exhaust RAM on a 4 GB Pi |
| `python3-picamera2` | the supported camera stack on current Raspberry Pi OS — see §6 |
| `python3-smbus`, `i2c-tools` | I2C bus access and the `i2cdetect` diagnostic |
| `libatlas-base-dev` | BLAS runtime NumPy links against |
| `build-essential` | compiler toolchain for source builds |

**Check that OpenCV includes the ArUco module.** The docking code needs
`cv2.aruco`, which lives in OpenCV's *contrib* modules and is not in every
build:

```bash
python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'aruco'))"
```

You want a version number and `True`. If it prints `False`, the Debian
build lacks contrib and you must install from pip inside the virtual
environment instead:

```bash
pip install opencv-contrib-python
```

Note that `opencv-python` and `opencv-contrib-python` conflict — install
only the contrib one, never both, or you get an OpenCV whose `cv2.aruco`
is missing at random.

---

## 5. Enable interfaces

```bash
sudo raspi-config
```

Navigate with arrow keys, select with Enter, and use Tab to reach
`<Finish>`.

- **Interface Options → I2C → Yes** — required for the MPU6050.
- **Interface Options → Camera** — on older Raspberry Pi OS releases
  (Bullseye and earlier) this enables the legacy camera stack. On current
  releases (Bookworm) **this entry does not exist**, because the camera is
  detected automatically. Its absence is not a fault; move on.
- **Interface Options → SSH → Yes** if you did not enable it in Imager.

Then:

```bash
sudo reboot
```

---

## 6. Verify hardware

Do both checks now. Everything downstream depends on them.

### Camera

```bash
libcamera-hello --list-cameras
```

Expected: a block describing an `imx219` sensor with its available modes.

If the command is not found, try `rpicam-hello --list-cameras` — the tools
were renamed on Bookworm and both names may be present.

**Expected output** looks roughly like:

```
Available cameras
-----------------
0 : imx219 [3280x2464 10-bit RGGB] (/base/soc/i2c0mux/i2c@1/imx219@10)
```

**Common failures:**

| Symptom | Cause |
|---|---|
| `No cameras available` | ribbon not seated, or inserted backwards |
| Works then stops | ribbon partially unlatched by chassis vibration |
| Detected but images are black | lens cap, or a genuinely faulty module |

The ribbon has contacts on one side only. In the Pi's CSI connector the
contacts face **away** from the Ethernet port. Lift the plastic latch,
insert fully, press the latch back down. Always with the Pi powered off.

### An important caveat about the camera and this project

The project's camera code (`hardware/camera_hw.py`, and the `--camera`
modes of `calibration.py` and `detect_aruco.py`) uses
`cv2.VideoCapture(0)`. On current Raspberry Pi OS, CSI cameras are driven
by **libcamera**, and `cv2.VideoCapture` does not always see them — it was
written for V4L2 devices.

Test it explicitly before you rely on it:

```bash
python3 -c "import cv2; c=cv2.VideoCapture(0); print('opened:', c.isOpened()); print('frame:', c.read()[0]); c.release()"
```

If both print `True`, you are fine — continue.

If it prints `False`, you have three options, in order of preference:

1. Run camera scripts through libcamera's V4L2 compatibility shim:
   ```bash
   libcamerify python3 detect_aruco.py --camera
   ```
   Check whether the shim is installed with `which libcamerify`; if it is
   missing, search for it: `apt-cache search libcamera | grep -i v4l2`.
2. Use a **USB webcam** instead of the CSI camera for calibration and
   detection. It will appear as a normal V4L2 device. Note the intrinsics
   will then be that webcam's, not the Camera V2's, so recalibrate if you
   later switch back.
3. Adapt the capture calls to `picamera2`. This is a code change, so it is
   out of scope for this guide — but `python3-picamera2` is installed in
   §4 so the option is there.

This has not been tested on hardware from here, so treat it as the first
thing to check rather than a settled answer.

### I2C

```bash
i2cdetect -y 1
```

Expected — the MPU6050 answering at address `0x68`:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
...
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

**Common failures:**

| Symptom | Cause |
|---|---|
| Grid is entirely `--` | wiring, or I2C not enabled in §5 |
| `Could not open file /dev/i2c-1` | I2C not enabled; re-run §5 and reboot |
| Device appears at `0x69` | the AD0 pin is pulled high — fine, but note it |
| Permission denied | your user is not in the `i2c` group — see §16 |

An MPU6050 that does not answer means no heading fusion. Heading is the
only bounded part of the pose estimate; without it, the robot's angular
error grows without limit and nothing downstream works properly. Fix this
before continuing.

---

## 7. Clone the project and install Python dependencies

```bash
cd ~
git clone <your-repository-url> medivan_sim
cd medivan_sim
```

If you are copying the project across rather than cloning, put it at
`~/medivan_sim` so the paths in this guide match.

### Create the virtual environment

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

**`--system-site-packages` is not optional here.** It lets the virtual
environment see the apt-installed `python3-opencv` and `python3-picamera2`.
Without it, the venv is sealed off from them, `import cv2` fails, and you
end up trying to pip-install OpenCV on a Pi — which usually means a
multi-hour source build that runs out of memory. If you have already
created a plain venv, delete it (`rm -rf .venv`) and redo this step.

You will need to run `source .venv/bin/activate` in every new terminal.
The prompt shows `(.venv)` when it is active.

### Install requirements

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### Install the packages requirements.txt is missing

`requirements.txt` covers the simulator only. Three parts of the project
import packages that are not listed in it:

| Needed by | Packages |
|---|---|
| `backend/app.py` (the web backend) | `flask`, `flask-socketio`, `flask-cors` |
| `hardware/imu_hw.py` (MPU6050) | `smbus2` |
| `hardware/motor_driver_hw.py` (L298N) | `gpiozero` |

Install them:

```bash
pip install flask flask-socketio flask-cors smbus2 gpiozero
```

The hardware modules degrade gracefully — `imu_hw.py` prints
`[WARNING] smbus2 not found. Running IMUHW in stub mode.` and carries on
with no IMU, which is a confusing way to discover a missing package. Install
them now.

Consider adding these five lines to `requirements.txt` so the next person
does not have to find this out. That is an edit to a dependency list, not
to robotics code.

### Check

```bash
python3 -c "import cv2, numpy, flask; print('imports ok')"
python3 -m pytest test_ai.py test_modules.py test_slam.py -q
```

The test suite takes a few minutes on a Pi. All tests should pass.
`test_aruco_docking.py` is a standalone script rather than a pytest file —
run it directly:

```bash
python3 test_aruco_docking.py
```

Expected: `=== 33 passed, 0 failed ===`.

---

## 8. Project folder structure

```
medivan_sim/
├── main.py                     simulator with Pygame visualisation
├── config.py                   ALL tunable constants and GPIO pin numbers
├── requirements.txt
├── calibration.py              camera calibration utility        (§9)
├── generate_marker.py          ArUco marker generator            (§10)
├── detect_aruco.py             detection / pose / docking dry-run (§15)
│
├── modules/                    robotics — SLAM, EKF, A*, mission controller
│   ├── mission_controller.py       the single robotics interface
│   ├── slam_engine.py
│   ├── localizer.py                EKF
│   ├── path_planner.py             A*
│   └── ...
│
├── robot/
│   └── aruco_docking.py        marker detection and docking behaviour
│
├── hardware/                   physical drivers, used when HARDWARE_MODE=True
│   ├── camera_hw.py
│   ├── imu_hw.py
│   ├── motor_driver_hw.py
│   └── encoder_hw.py
│
├── backend/                    Flask web layer
│   ├── app.py                      entry point                   (§13)
│   ├── routes.py
│   └── mission_runner.py
│
├── frontend/                   dashboard, served by the backend  (§14)
│   ├── index.html
│   └── js/api.js
│
├── docs/
│   ├── HARDWARE_SETUP.md
│   ├── CALIBRATION.md
│   └── DOCKING_TEST_PROCEDURE.md
│
├── assets/                     map image, YOLO model, Q-table
│
├── calibration/                ← CREATED BY calibration.py IN §9
│   ├── camera_matrix.npy           camera intrinsics
│   ├── dist_coeffs.npy             lens distortion
│   └── calibration_meta.txt        when, how many views, quality
│
└── output/                     everything the robot writes
    ├── maps/                       saved occupancy grids
    ├── markers/                    generated marker PNG and PDF
    └── calib_frames/               calibration images
```

**`calibration/` does not exist in a fresh clone.** It is created the first
time a calibration is accepted and saved. No calibration file ships with
the project — anything you find there is one you produced.

---

## 9. Camera calibration

Full detail is in `docs/CALIBRATION.md`. The short version:

**Why it matters.** Until calibration exists, the docking code falls back
to datasheet placeholder values and reports `calibrated=False`. Range is
recovered from the marker's apparent size, so a 2% focal-length error is a
2% range error at every distance — a fixed bias that does not average out.
The robot then stops consistently short of, or drives into, the dock.

### Print a checkerboard

Any checkerboard works. The defaults expect **9 × 6 inner corners** with
**25 mm** squares — that is a 10 × 7 grid of squares; the tool counts the
*inner* corners where four squares meet, not the squares. Print at 100%
scale, measure one square with a rule, and mount it on stiff card. A board
that bends makes the distortion fit absorb the bend.

### Capture

From the project root, with the venv active:

```bash
cd ~/medivan_sim
source .venv/bin/activate
python3 calibration.py --capture
```

Move the board around the frame — **corners and edges matter most**, tilt
it 30–45°, vary distance across roughly 0.3–0.7 m. The tool only accepts a
view once the board has moved appreciably, because near-identical views
improve the reported error without adding information.

Frames are saved to `output/calib_frames/`, so a set can be reprocessed
without reshooting:

```bash
python3 calibration.py --from-dir output/calib_frames
```

If `--capture` cannot open the camera, revisit the `cv2.VideoCapture`
caveat in §6.

### Verify it succeeded

```bash
python3 calibration.py --validate
```

Expected:

```
  calibrated = True  -- real intrinsics in use
  fx=... fy=... cx=... cy=...
```

A calibration that fails the tool's quality checks is **not saved** — the
tool tells you why (too few views, error too high, board never visited the
frame edges). That is deliberate: saving it would set `calibrated=True` on
numbers nobody should trust, which is worse than the honest fallback.

Sanity checks on your own numbers: `cx, cy` near the image centre,
`fx ≈ fy`, and a horizontal field of view near the Camera V2's 62.2°.

Then check it against a tape measure, which is the only real test:

```bash
python3 detect_aruco.py --camera --expect-range 1.00
```

with the marker at exactly 1.00 m, square to the camera. A consistent bias
is a calibration or marker-size error; scatter is noise.

### Always start from the project root

`config.py` stores the calibration paths as **relative** paths, resolved
against the current working directory. Start the robot from anywhere else
and calibration is silently not found — no error, just placeholder
intrinsics and `calibrated=False`.

This was reproduced during development: with both files present, running
from a different directory reported `calibrated = False`.

So: always `cd ~/medivan_sim` first. `docs/CALIBRATION.md` gives a two-line
change to `config.py` that makes the paths absolute, if you would rather
fix it than remember it.

---

## 10. Generate and print the marker

```bash
cd ~/medivan_sim
python3 generate_marker.py
```

Writes:

```
output/markers/marker_id0_100mm.png
output/markers/marker_id0_100mm.pdf     ← print this one
```

Copy the PDF to a machine with a printer:

```bash
scp <username>@medivan.local:~/medivan_sim/output/markers/marker_id0_100mm.pdf .
```

**Print at 100% / "Actual size". Do NOT use "Fit to Page".** Fit-to-page
silently rescales — often to about 96% — and every range reading inherits
that error.

The PDF includes a 100 mm ruler beside the marker. **Measure it with a
steel rule after printing.** The black outer edge of the marker must be
100 mm. If it is not, either reprint or set `ARUCO_MARKER_SIZE_M` in
`config.py` to what you actually measured — but do not leave the two
disagreeing.

Verified from the PDF file itself: rasterised at 300 dpi it measures
99.991 × 100.076 mm, and still decodes as `DICT_4X4_50` id 0. That checks
the file. It does not check your printer.

Mount the marker **flat on rigid backing**. A curled marker moves the
corners, and the corners are the entire pose estimate.

---

## 11. Hardware wiring

Full pin map and rationale: `docs/HARDWARE_SETUP.md`. Summary:

**Power off the Pi before connecting or disconnecting anything.**

### Camera

Ribbon into the Pi's CSI port, contacts facing away from the Ethernet
port. Latch down. Mount the lens at **0.10 m** above the floor
(`CAMERA_HEIGHT_M`), pointing forward and level.

### MPU6050 (I2C)

| MPU6050 | Pi |
|---|---|
| VCC | 3.3 V (pin 1) |
| GND | GND |
| SDA | BCM 2 (pin 3) |
| SCL | BCM 3 (pin 5) |

### Motors (L298N)

| Signal | BCM | L298N |
|---|---|---|
| `PIN_MOTOR_ENA` | 18 | ENA — left PWM |
| `PIN_MOTOR_IN1` | 17 | IN1 — left dir A |
| `PIN_MOTOR_IN2` | 27 | IN2 — left dir B |
| `PIN_MOTOR_IN3` | 22 | IN3 — right dir A |
| `PIN_MOTOR_IN4` | 23 | IN4 — right dir B |
| `PIN_MOTOR_ENB` | 19 | ENB — right PWM |

**GPIO 18 and 19 are not arbitrary.** They are hardware PWM0 and PWM1,
which are independent channels. GPIO 12 and 18 share PWM channel 0 — use
that pair and both motors are forced to the same duty cycle, so the robot
cannot turn. Do not "tidy" the pin assignment.

### Power

- Battery pack (2× 18650) → L298N motor supply, and → 5 V buck converter →
  Pi.
- The Pi is powered from the **buck converter**, not from the L298N's
  onboard 5 V regulator. Motor current spikes on a shared rail brown out
  the Pi mid-mission, which looks like random crashes.
- TP4056 handles cell charging.

### Ground

**Pi ground, L298N logic ground and battery ground must all be common.**
Without a shared ground the direction pins float and the motors behave
erratically — a fault that looks like a software bug and is not.

### Before powering the motors

Put the **wheels off the ground** and confirm each motor turns in the
expected direction. Reversed polarity presents as a control bug and wastes
hours.

---

## 12. First startup (simulator)

This runs the full stack against the simulated world — no motors move. It
confirms the software works before hardware is in the loop.

`main.py` uses Pygame and wants a display. Over SSH there is none, so run
it headless:

```bash
cd ~/medivan_sim
source .venv/bin/activate
SDL_VIDEODRIVER=dummy python3 main.py
```

With a monitor attached, drop the `SDL_VIDEODRIVER=dummy` prefix to see
the window.

**Expected startup messages:**

```
[main] World loaded (physics only): 800x600
[main] Phase 1: SLAM MAPPING -- robot exploring environment...
[main] HARDWARE_MODE=False. Loading software simulators...
[AIDetector] YOLOv8n loaded from .../assets/yolov8n.onnx @ 320x320
[Camera] No webcam -- using synthetic corridor frames
[QLearning] Q-table loaded from .../assets/q_table.npy
[main] Simulation running. Press Q to quit, TAB for manual mode.
```

After roughly half a minute of exploring:

```
[SLAM] Map saved -> .../output/slam_map.png
[SLAM] Mapping complete! Coverage=94.9% Time=27s
[main] SLAM complete! Phase 2: NAVIGATION
[main] First path: 76 waypoints
```

Press `Q` to quit (or Ctrl-C over SSH). It prints a session summary with
distance travelled, frames logged and replan count.

ALSA warnings about `Unknown PCM default` are harmless — the Pi has no
audio device configured and the sound module falls back silently.

`Coverage` and timings will differ from the numbers above; they depend on
the run. What matters is that both phases are reached with no traceback.

### Switching to real hardware

Once §6 and §11 both pass, set in `config.py`:

```python
HARDWARE_MODE: bool = True
```

This swaps the simulated drivers for `hardware/*.py`. Wheels off the
ground the first time.

---

## 13. Run the backend

```bash
cd ~/medivan_sim
source .venv/bin/activate
python3 backend/app.py
```

Expected:

```
[backend] MissionController stepping at 30 Hz
[backend] REST on http://0.0.0.0:5000/api/telemetry
[backend] Socket.IO enabled
```

Useful options:

| Option | Effect |
|---|---|
| `--port 5000` | change the port |
| `--hz 30` | control-loop rate |
| `--deliveries 0` | deliveries seeded at startup; 0 waits for the dashboard |
| `--no-saved-map` | ignore the stored map and explore from cold |
| `--start-paused` | come up held, so you can watch before it moves |
| `--map-name NAME` | use a scratch map store instead of the operational one |

Check it from the Pi:

```bash
curl http://localhost:5000/api/telemetry
```

You should get a JSON object including `state`, `x`, `y`, `battery_pct`
and `coverage_pct`.

A Werkzeug warning about production deployment is expected — this is the
Flask development server. For a permanent installation, put it behind a
proper WSGI server and a systemd unit; see the `WorkingDirectory` note in
`docs/HARDWARE_SETUP.md`, which matters because of the relative-path issue
in §9.

---

## 14. Open the dashboard

The backend serves the dashboard itself. From any machine on the same
network:

```
http://medivan.local:5000/
```

or `http://<pi-ip-address>:5000/`.

### Switch it to live data

**The dashboard starts in demo mode.** Out of the box it shows a simulated
robot generated in the browser — convincing, and completely disconnected
from your Pi. The header shows `DEMO DATA`.

To connect it:

1. Go to the **Settings** page.
2. **Data source** → *Live robot*.
3. **Robot address** → `http://medivan.local:5000` (or the Pi's IP).
4. **Apply**.

The header indicator changes to `LIVE`.

### Verify telemetry is real

Confirm the dashboard is showing your robot rather than the demo:

- The header reads **LIVE**, not `DEMO DATA`.
- **Overview** → position, heading and battery change as the robot moves,
  and match what `curl http://localhost:5000/api/telemetry` reports.
- **Live map** → the occupancy grid fills in as exploration proceeds.
- **Camera** → shows no frame and no detections. **This is correct.** The
  vision pipeline is deliberately not run inside the mission loop; the
  demo mode fabricates detections, the live backend does not.
- **Activity feed** → real events (`Mapping complete`, `Delivery
  assigned`).

A note on the **Drift** reading: it is the EKF's own 1σ position
uncertainty, not measured drift. Measured over a full mission, the filter
*underestimates* its true error — 111 px reported against 171 px actual at
one point. Treat it as a lower bound.

---

## 15. Docking test

The full procedure is `docs/DOCKING_TEST_PROCEDURE.md` — eight staged
tests, each isolating one failure mode. Do not skip to the end; a docking
failure with no per-stage numbers is very hard to diagnose.

Prerequisites: §9 complete with `calibrated = True`, marker printed and
measured, dock built.

Outline:

**Marker detection** (Stages 1–3). Wheels off the ground. Marker at a
tape-measured 1.00 m:

```bash
python3 detect_aruco.py --camera --expect-range 1.00
```

Record detection rate, mean range, bias, latency. Then repeat across
0.90 m down to 0.12 m to map your usable envelope, then vary lighting and
marker angle. Expect range accuracy to be **worst at the far end** — in
noise-free synthetic frames the reading was +5.2% at 0.90 m and +3.5% at
0.80 m, improving to ~0.3% from 0.60 m in. The pre-dock standoff sits in
the least accurate part of that curve.

**Alignment** (Stage 4). Still wheels-up, run the real docking state
machine with motors idle:

```bash
python3 detect_aruco.py --camera --dock-dry-run
```

Watch the phase sequence `search_aruco → align_dock → docking` and check
that moving the marker sideways changes the motor PWMs in the *correct*
direction. A sign error here is trivially visible now and expensive later.

**Docking** (Stages 5–6). Wheels down, someone on the e-stop. Approach
from the pre-dock waypoint, square and then offset. Ten runs; target ≥8
mechanical engagements. If vision reports 0.15 m but the contacts miss,
the fault is mechanical tolerance — measure before changing code.

**Charging** (Stage 7). Connect the charging circuit and let it dock.

Charging **stops at 95%**, deliberately: the 18650 pack has no balancing
circuitry and holding Li-ion at 100% accelerates ageing. Stopping short is
correct behaviour, not a fault.

**Full mission** (Stage 8). The one that matters. In simulation the return
to dock currently fails, timing out 87–93 px short — it is the longest and
last traverse and runs on the most degraded pose estimate (drift
0.941 px/m, unbounded, a documented limitation). ArUco is the first
correction in this project with a reference genuinely independent of the
robot's own map, so this is where you learn whether it closes the gap.

Record the pose error at the pre-dock waypoint. If drift puts the robot
somewhere the marker is not in frame, docking never starts — and that is a
localisation problem, not a docking one.

---

## 16. Troubleshooting

### Camera not detected

```bash
libcamera-hello --list-cameras     # or rpicam-hello
```

- Nothing listed → power off, reseat the ribbon (contacts away from the
  Ethernet port), latch down, power on.
- Still nothing → try the other ribbon; they fail more often than the
  cameras do.
- Listed by `libcamera-hello` but `cv2.VideoCapture(0)` returns `False` →
  this is the libcamera/V4L2 gap, not a broken camera. See §6.
- Worked, then stopped → chassis vibration unlatching the connector.

### I2C device missing

```bash
i2cdetect -y 1
```

- All `--` → check wiring: SDA to BCM 2, SCL to BCM 3, VCC to **3.3 V**
  (not 5 V), grounds common.
- `Could not open file /dev/i2c-1` → I2C not enabled. Re-run §5, reboot.
- Appears at `0x69` instead of `0x68` → AD0 is high. Harmless, but note it.
- `[WARNING] smbus2 not found. Running IMUHW in stub mode.` → the package
  is missing; `pip install smbus2` inside the venv (§7).

### Marker not detected

- Check the calibration is loaded: `python3 calibration.py --validate`.
- Check the printed size with a rule. A rescaled print is the most common
  cause of consistently wrong ranges.
- Too close? Below ~0.12 m a 100 mm marker overflows the frame and stops
  decoding. This is expected and is why contact is declared at 0.15 m.
- Too far? Detection was measured reliable to about 0.90 m.
- Glare, low light, or a marker at a steep angle. Try Stage 3 conditions.
- Curled or creased marker — mount it flat on rigid backing.
- Confirm the marker itself is valid:
  `python3 generate_marker.py --verify output/markers/marker_id0_100mm.png`

### Calibration missing

Symptom: `calibrated = False`, or the docking module printing that
placeholders are in use.

- Have you run §9 at all? No calibration ships with the project.
- Did the calibration *save*? The tool refuses to save a fit that fails
  its quality checks and says why. Re-shoot with more views spread across
  the frame.
- Check the files exist: `ls calibration/`

### Wrong working directory

The single most confusing failure in this project. The calibration paths
in `config.py` are relative, so they resolve against wherever you started
the program. Launch from the wrong directory and calibration is silently
not found — no error, just quietly wrong intrinsics.

```bash
cd ~/medivan_sim        # ALWAYS
python3 calibration.py --validate
```

`--validate` prints both the repo-anchored paths and what would actually
be loaded from where you are standing. In a systemd unit, set
`WorkingDirectory=/home/<user>/medivan_sim`.

### Battery issues

- **Pi reboots when the motors start** → the Pi is sharing a rail with the
  motors. It must be fed from the 5 V buck converter. Check for a rainbow
  square / undervoltage warning in `dmesg`.
- **Charging stops at 95%** → correct, deliberate, not a fault.
- **Robot refuses new deliveries after a low-battery event** → also
  correct. `BatteryManager` sets a lockout so it will not accept work
  until it has charged.
- **Cells at different voltages** → the pack has no balancing. Check them
  individually with a multimeter; a badly mismatched pair should be
  replaced.

### Permission errors

Add your user to the hardware groups (once, then log out and back in):

```bash
sudo usermod -aG i2c,video,gpio,dialout $USER
```

- `Permission denied: '/dev/i2c-1'` → the `i2c` group.
- Camera permission errors → the `video` group.
- GPIO errors → the `gpio` group.
- `error: externally-managed-environment` from pip → you are outside the
  virtual environment. Run `source .venv/bin/activate`. Do **not** reach
  for `--break-system-packages` on a Pi you care about.
- Port 5000 already in use → something else is bound; use
  `--port 5001`, or find it with `sudo lsof -i :5000`.

---

## 17. Deployment checklist

Print this. Tick as you go — and do not tick ahead.

**System**

- ☐ Raspberry Pi OS (64-bit) flashed
- ☐ SSH and/or Wi-Fi working, Pi reachable
- ☐ `sudo apt update && sudo apt full-upgrade -y` completed, rebooted
- ☐ System packages installed (§4)
- ☐ `cv2.aruco` present — `python3 -c "import cv2; print(hasattr(cv2,'aruco'))"` → `True`
- ☐ Camera enabled (or auto-detected on Bookworm)
- ☐ I2C enabled
- ☐ Rebooted after `raspi-config`

**Hardware verification**

- ☐ `libcamera-hello --list-cameras` lists the imx219
- ☐ `cv2.VideoCapture(0)` opens and reads a frame — or a documented workaround chosen (§6)
- ☐ `i2cdetect -y 1` shows `0x68`
- ☐ Motors wired per §11, grounds common
- ☐ Pi powered from the buck converter, not the L298N regulator
- ☐ Motor directions verified with wheels off the ground

**Software**

- ☐ Repository cloned to `~/medivan_sim`
- ☐ Virtual environment created **with `--system-site-packages`**
- ☐ `pip install -r requirements.txt` completed
- ☐ Extra packages installed: `flask flask-socketio flask-cors smbus2 gpiozero`
- ☐ `python3 -m pytest test_ai.py test_modules.py test_slam.py -q` passes
- ☐ `python3 test_aruco_docking.py` → 33 passed

**Marker and calibration**

- ☐ Marker generated
- ☐ Marker printed at 100%, **not** fit-to-page
- ☐ Printed marker measured with a rule = 100 mm (or `ARUCO_MARKER_SIZE_M` updated to match)
- ☐ Marker mounted flat on rigid backing
- ☐ Checkerboard printed and mounted flat
- ☐ Calibration captured (≥12 views, spread across the frame)
- ☐ `python3 calibration.py --validate` → `calibrated = True`
- ☐ Range checked against a tape measure at 1.00 m

**Bring-up**

- ☐ Simulator runs: `SDL_VIDEODRIVER=dummy python3 main.py` reaches Phase 2
- ☐ `HARDWARE_MODE = True` set in `config.py` (when ready)
- ☐ Backend starts: `python3 backend/app.py`
- ☐ `curl http://localhost:5000/api/telemetry` returns JSON
- ☐ Dashboard reachable at `http://medivan.local:5000/`
- ☐ Dashboard switched to **Live robot** — header shows `LIVE`
- ☐ Telemetry on the dashboard matches the robot

**Docking** (see `docs/DOCKING_TEST_PROCEDURE.md`)

- ☐ Stage 1 — marker detected, bias < 3%
- ☐ Stage 2 — detection envelope mapped
- ☐ Stage 3 — lighting and angle robustness recorded
- ☐ Stage 4 — dry run, phases advance, PWM signs correct
- ☐ Stage 5 — powered approach on open floor
- ☐ Stage 6 — docking successful, ≥8/10
- ☐ Stage 7 — charging starts on contact and stops at 95%
- ☐ Stage 8 — full mission with return to dock

---

## What this guide does not cover

Honesty about the boundaries, so you know where you are on your own:

- **None of it has been executed on a physical Pi.** Commands were checked
  against the project files and, where possible, run on x86 Linux. Anything
  touching the camera, IMU or motors is unverified on hardware.
- The `cv2.VideoCapture` / libcamera question in §6 is the most likely
  place to lose time, and the one I am least able to settle from here.
- Timings quoted anywhere in the project were measured on x86. The Pi's
  Cortex-A72 will be slower — materially so for the YOLO path. Re-run
  `evaluate_models.py` on the Pi for deployment numbers.
- Docking has never been tested physically. Stage 8 in particular tests a
  known open failure, not a settled feature.
- `STATUS.md` lists what is measured versus what is a documented
  limitation. Read it before quoting any number from this project.
