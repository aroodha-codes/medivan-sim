# MediVan — Hardware Setup Guide

Covers bringing the physical robot up to the point where the docking test
procedure can be run. Nothing in this document has been executed: this
container has no Pi, no camera and no dock. Treat it as instructions to
follow and verify, not as a record of something that worked.

Hardware is fixed. Do not add to it.

| Part | Notes |
|---|---|
| Raspberry Pi 4 Model B, 4 GB | the compute |
| Pi Camera Module V2 (IMX219) | 62.2° HFOV; the only exteroceptive sensor |
| MPU6050 IMU | yaw fusion; the bounded part of the pose estimate |
| L298N motor driver | 2 channels |
| 2WD differential chassis | |
| 2× 18650 + TP4056 + 5 V buck | no cell balancing — see the charging note |

---

## 1. Wiring

GPIO numbering is **BCM**, matching `config.py`.

| Signal | BCM | Goes to |
|---|---|---|
| `PIN_MOTOR_ENA` | 18 | L298N ENA (left PWM) |
| `PIN_MOTOR_IN1` | 17 | L298N IN1 (left dir A) |
| `PIN_MOTOR_IN2` | 27 | L298N IN2 (left dir B) |
| `PIN_MOTOR_IN3` | 22 | L298N IN3 (right dir A) |
| `PIN_MOTOR_IN4` | 23 | L298N IN4 (right dir B) |
| `PIN_MOTOR_ENB` | 19 | L298N ENB (right PWM) |
| `PIN_I2C_SDA` | 2 | MPU6050 SDA |
| `PIN_I2C_SCL` | 3 | MPU6050 SCL |

**GPIO 18 and 19 are not arbitrary.** On the BCM2711 they are hardware
PWM0 and PWM1, which are independent. GPIO 12 and 18 both sit on PWM
channel 0, so using that pair makes the two motors share a duty cycle and
the robot cannot turn under PWM control. `config.py` documents this; do
not "tidy" the pin assignment.

Grounds: the L298N logic ground, the Pi ground and the battery ground must
be common, or the direction pins float and the motors behave erratically.

The Pi is powered from the 5 V buck converter, not from the L298N's on-board
regulator. Motor current spikes on the same rail will brown out the Pi
mid-mission.

## 2. Pi OS configuration

```bash
sudo raspi-config          # Interface Options -> enable Camera and I2C
sudo reboot

libcamera-hello --list-cameras   # camera is seen
i2cdetect -y 1                   # MPU6050 answers, usually at 0x68
```

If `i2cdetect` shows nothing, the IMU wiring is wrong and heading fusion —
the one part of localisation that does *not* drift — will be dead.

## 3. Software

```bash
git clone <repo> && cd medivan_sim
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` pulls `opencv-contrib-python`, which is what supplies
`cv2.aruco`. Plain `opencv-python` does not, and the docking module will
fail to import.

On a 4 GB Pi 4 the OpenCV wheel build can exhaust RAM. If pip tries to
build from source rather than fetching a wheel, use the distro package
instead:

```bash
sudo apt install python3-opencv
```

Then set `HARDWARE_MODE = True` in `config.py` to switch the simulated
drivers for the real ones.

### Always start from the repo root

`CAMERA_CALIB_MATRIX_PATH` is a **relative** path
(`calibration/camera_matrix.npy`), resolved against the working directory.
Start the robot from anywhere else and calibration is silently not found —
the code falls back to datasheet placeholders, reports `calibrated=False`,
and every range carries a scale error.

Verified in this container: the same files, read from a different working
directory, produced `calibrated = False`. Use absolute paths in any
systemd unit:

```ini
WorkingDirectory=/home/pi/medivan_sim
ExecStart=/home/pi/medivan_sim/.venv/bin/python3 -m backend.app
```

Check it at any time with `python3 calibration.py --validate`, which prints
both the repo-anchored paths and what the docking module would actually
load from where you are standing.

## 4. Marker

```bash
python3 generate_marker.py
```

Writes `output/markers/marker_id0_100mm.pdf`. Print **at 100 % / actual
size** — not "fit to page". The PDF carries a 100 mm ruler beside the
marker; measure it with a steel rule before use.

Verified here: rasterised at 300 dpi, the marker measures 99.991 × 100.076
mm against a 100 mm target — 0.076 mm out, which is one pixel at that
resolution, i.e. at the measurement floor. It also still decodes as
`DICT_4X4_50` id 0 after the PDF round-trip. That checks the file. It does
not check your printer.

Range is recovered from apparent size against assumed size, so a size error
becomes a range error of the same proportion. If your print measures 97 mm,
either reprint or set `ARUCO_MARKER_SIZE_M = 0.097`. Do not leave them
disagreeing.

Mount it **flat on rigid backing**. A curled or creased marker moves the
corners, and the corners are the entire pose estimate.

## 5. Dock geometry

| Parameter | Value | Meaning |
|---|---|---|
| `ARUCO_MARKER_SIZE_M` | 0.10 | printed side |
| `PRE_DOCK_DISTANCE_M` | 0.80 | standoff where vision docking takes over |
| `DOCK_CONTACT_DISTANCE_M` | 0.15 | contacts declared engaged |
| `CAMERA_HEIGHT_M` | 0.10 | lens height above floor |
| `DOCK_TIMEOUT_S` | 90.0 | attempt abandoned |

Mount the marker centre at the camera's height (0.10 m) and square to the
approach path. Tilting it introduces a yaw bias that `ALIGN_DOCK` will
faithfully null onto the wrong heading.

Detection envelope previously measured at fx = 265, 320×240: reliable from
0.90 m in to 0.12 m, so the 0.80 m standoff sits inside it. Two caveats
this round's testing adds:

- Below ~0.12 m a 100 mm marker overflows a 320×240 frame and stops
  decoding. `DOCK_CONTACT_DISTANCE_M = 0.15` is above that on purpose. The
  last few centimetres are the dock funnel's job, not vision's.
- Range accuracy is **worst at the far end**. In noise-free synthetic
  frames the reading was +5.2 % at 0.90 m and +3.5 % at 0.80 m, falling to
  ~0.3 % from 0.60 m in. The marker is only ~29 px across at 0.90 m, so
  corner quantisation dominates. Expect the first fix at the pre-dock
  waypoint to be the least accurate one of the approach; it improves as the
  robot closes.

Clearance: the corridor cells the chassis can occupy are narrow (205 of
1100 free cells). Leave the pre-dock waypoint on a cell the vehicle can
actually reach, or A* will fail before docking is ever attempted.

## 6. Charging

`BatteryManager` stops charging at **95 %**, deliberately. The 18650 pack
has no balancing circuitry and holding Li-ion at 100 % accelerates ageing.
A charge that "stops early" is correct behaviour, not a fault.

## 7. Before the first run

1. Wheels off the ground. Confirm both motors respond and turn in opposite
   senses when commanded to rotate. Reversed polarity looks like a control
   bug and wastes hours.
2. `i2cdetect -y 1` sees the IMU.
3. `python3 calibration.py --capture` — see `CALIBRATION.md`.
4. `python3 detect_aruco.py --camera --expect-range 1.00` with the marker
   at a tape-measured 1.00 m.
5. Then `DOCKING_TEST_PROCEDURE.md`.

Do not skip 3. Everything downstream inherits its error.
