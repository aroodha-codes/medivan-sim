# MediVan — Final Status

Written at hand-off. Every claim below is backed by a benchmark or test in
`EVALUATION_REPORT.md`; anything unverified is marked as such.

## Verified by measurement

| Subsystem | Evidence |
|---|---|
| Perception (`PerceptionSource`) | 93–97 % occupancy accuracy vs ground truth; pose-dependent scans |
| Occupancy mapping | Coverage metric bounded and reproducible (std ~28 cells over 5 seeds) |
| EKF heading | 1.52–1.62 deg RMSE, stable across straight / turn / mixed trajectories |
| Covariance consistency | NEES 0.87–1.10 (3-DOF target ~3.0; slightly conservative = safe) |
| A* planner | 31.15 ms mean vs 90.97 ms before (2.9x), identical routes |
| Map persistence | Cold boot 301 explore ticks → warm boot 0; map 96.4 % coverage, 460 B |
| Battery manager | All six rules unit-tested, charging stops at exactly 95.0 % |
| Mission execution | 2/2 deliveries completed autonomously, 0 planning failures |
| ArUco docking | 33/33 tests on real rendered markers; 1.019 m read at true 1.00 m |
| Hardware abstraction | 60/60 control-loop ticks clean in `HARDWARE_MODE` |

45 Python files compile; 17/17 non-Pygame modules import cleanly.

## Documented limitations

1. **Translational drift, 0.941 px/m (R^2 = 0.948), unbounded.** Four
   correction methods were implemented, benchmarked and rejected: junction
   landmarks (23x worse), live-map scan matching (3x), frozen-map scan
   matching (6–9x), and a zero-match configuration that was baseline in
   disguise. All shared one flaw — the observation derived from the robot's
   own map. Retained but disabled; see ADDENDUM 9/10.
2. **Return-to-dock fails in simulation**, caused by (1) on the mission's
   longest and last traverse. ArUco docking is the fix and is the first
   method with a genuinely independent reference.
3. **Exploration reaches ~40 % reachable coverage** in the runs performed;
   coverage rises monotonically with distance and does not plateau, so this
   is a time budget rather than a defect.

## Not verified in this environment

* `main.py` — requires Pygame, which cannot be installed (proxy returns
  `host_not_allowed` for all hosts). Every benchmark ran against the modules
  directly via `modules/mission_controller.py`.
* Pytest suites `test_ai.py`, `test_modules.py`, `test_slam.py` — need pytest.
* ArUco → MissionController integration (`GO_TO_PREDOCK`, `SEARCH_ARUCO`,
  `ALIGN_DOCK`, `DOCKING` states). The module is complete and tested
  standalone; the simulator cannot render a marker from the vehicle camera
  pose, so end-to-end docking would be a mock feeding a mock.
* Dashboard rendering — `frontend/` is syntax-checked (`node --check`) and all
  49 element IDs resolve, but it has never been opened in a browser.
* Flask backend, WebSocket layer, login page, mission history UI.

## Run it

```bash
# Mission (headless, no Pygame needed)
python3 -c "from modules.mission_controller import MissionController; \
            print(MissionController(seed=1, n_deliveries=2).run().events)"

# ArUco docking tests
python3 test_aruco_docking.py

# Model evaluation harness — run this ON the Pi for real numbers
python3 evaluate_models.py --json results.json

# Dashboard (demo data)
cd frontend && python3 -m http.server 8080
```

## First three things to do on hardware

1. **Calibrate the camera.** `robot/aruco_docking.py` falls back to
   datasheet placeholders and reports `calibrated=False`; ranges carry a
   systematic scale error until `calibration/camera_matrix.npy` and
   `dist_coeffs.npy` exist.
2. **Re-run `evaluate_models.py` on the Pi.** The 93.99 ms YOLO latency was
   measured on x86 and will be materially slower on the Cortex-A72.
3. **Test ArUco docking physically.** Print marker ID 0 at 100 mm. Detection
   envelope measured at fx=265: reliable from 0.90 m in to 0.12 m.

## Known report discrepancy

Figure 6.1 in the project report lists `Wheelchair` and `Medical_Cart` with
per-class precision/recall. Neither is a COCO class, no script in this
repository produces those numbers, and the shipped `yolov8n.onnx` cannot
output them. There is no labelled hospital dataset here, so detection
precision/recall/mAP cannot currently be computed at all.
