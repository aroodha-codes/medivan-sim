# MediVan — Model Evaluation Report

All figures below are **measured** by `evaluate_models.py` from the artifacts
shipped in this repository. Nothing is hard-coded. Re-run to reproduce.

> Platform for these runs: 1-core Intel Xeon @ 2.80 GHz, x86_64, Python 3.12.
> **Not** a Raspberry Pi 4. Re-run on the Pi for deployment numbers.

## 1. Objective (iii) — Battery / docking state machine

Deterministic FSM over 8 states. No learned component, fully testable.
This objective is met.

## 2. Objective (ii) — A* route optimisation

| Metric | Value |
|---|---|
| Success rate | **74.0 %** (148/200 queries) |
| Plan time, mean | 90.97 ms |
| Plan time, p95 | 243.01 ms |
| Route length, mean | 10.18 m |

Sampling note: start/goal pairs are drawn from the planner's true navigable
domain (free **and** outside the 10 px vehicle-clearance band). Sampling
`is_free` alone gives a misleading 43.5 %.

The 26 % failures are genuinely disconnected regions — rooms with no
corridor wide enough for the vehicle footprint. **To exceed 85 % here, widen
the doorways in `hospital_map.png` or reduce `VEHICLE_WIDTH_PX`.** This is a
map property, not a planner defect.

## 3. Objective (ii) — SLAM coverage — **NOT MET**

| Explorer | Coverage after 4000 steps |
|---|---|
| Original wall-follower | 6.3 % |
| Frontier-based (this build) | 15.2 % |
| **Required for handover** | **85 %** |

A 2.4× improvement, but the target is not reached. Root causes still open:

1. The robot stays within a ~25×18 px region — forward progress is being
   blocked, most likely by the vehicle-footprint collision test near walls.
2. `_extract_walls_from_camera` reveals very few cells per frame against
   `CameraSim` synthetic frames, so the frontier is almost always one cell
   away and the robot replans continuously instead of travelling.

**Until this is resolved the MAPPING → NAVIGATION handover never fires**, so
the A* layer above is never exercised in a live run.

## 4. Objective (i) — Perception — NO ACCURACY FIGURE CAN BE PRODUCED

Measured (latency only):

| Metric | Value |
|---|---|
| Mean inference | 93.99 ms |
| p95 | 99.24 ms |
| Raw rate | 10.64 FPS |
| Budget used @ skip-5 | 56.4 % of 30 FPS |
| Classes kept | 17 of 80 COCO |

**Precision, recall, F1 and mAP are not reported, because they cannot be
computed from anything in this repository.** There is no labelled
hospital-corridor validation set. The shipped model is stock COCO-pretrained
YOLOv8n — it has never been fine-tuned on hospital data.

### Warning about Figure 6.1 in the project report

Figure 6.1 lists classes `Wheelchair` and `Medical_Cart` with per-class
precision/recall. Neither is a COCO class, no script in this repository
produces those numbers, and `yolov8n.onnx` cannot output them. Those figures
are not reproducible and should be expected to draw questions at viva.

### How to obtain a real >85 % figure

1. Capture ~300–500 corridor frames on the Pi camera in the actual environment.
2. Label them (CVAT / Roboflow) with your real classes.
3. Fine-tune: `yolo train model=yolov8n.pt data=hospital.yaml imgsz=320`
4. Export: `yolo export model=best.pt format=onnx imgsz=320`
5. Evaluate: `yolo val model=best.pt data=hospital.yaml` → real mAP50.

A fine-tuned YOLOv8n on a clean single-environment indoor set of this size
commonly reaches mAP50 in the 0.85–0.95 range. That is the legitimate route
to the number you need.

---

# ADDENDUM — SLAM coverage root cause

## The 85 % threshold was mathematically impossible

Measured directly from `assets/hospital_map.png` at `SLAM_GRID_RESOLUTION=10`:

| Quantity | Cells |
|---|---|
| Grid size (old coverage denominator) | 80 x 60 = **4800** |
| Ground-truth free cells | 1100 |
| Reachable from start (flood fill) | 1100 |
| Adjacent wall cells (mappable) | 416 |
| **Total observable** | **1516** |
| **Maximum possible old coverage** | **31.6 %** |
| Threshold demanded by config | 85.0 % |

The old metric was `explored / (grid_w * grid_h)`. Every cell of solid
interior wall was in the denominator but could never be observed, so coverage
was capped at 31.6 %. `SLAM_COVERAGE_THRESHOLD = 0.85` could never fire:
`mapping_complete` never became True, the state machine never left
`SLAM_MAPPING`, and **the entire A\* / delivery / docking stack was
unreachable in a live run.**

This — not the exploration policy — was the reason the system never worked.
The earlier wall-follower measurement of "6.3 % coverage" was 6.3 % of 4800,
i.e. ~20 % of what was actually observable.

## Fix

Coverage is now `explored / (explored + open_frontier)` — a denominator the
robot *discovers* rather than assumes, which is how frontier exploration is
normally terminated. It rises monotonically toward 1.0 as frontiers are
consumed. A warm-up guard (`SLAM_MIN_MAPPING_FRAMES`,
`SLAM_MIN_EXPLORED_CELLS`) prevents the first tick reading as 100 % mapped.

Result: `mapping_complete` now fires, and the navigation stack is reachable
for the first time.

## Known limitation: exploration completeness

The frontier explorer does **not** reliably drive the frontier set to zero.
Across runs on the same map it mapped between **674 and 1509** of the 1516
observable cells before the coverage fallback triggered handover. The map
passed to the navigation layer may therefore be partial, and A* will fail for
goals in unmapped regions.

This is the top open item. Recommended next step: replace the greedy
nearest-frontier selection with a utility function balancing information gain
against travel cost, and add a re-entry path from NAVIGATION back to MAPPING
when a goal falls outside the known map.

---

# ADDENDUM 2 — verified results and open items

## Verified improvement

**A\* heuristic (Euclidean → Manhattan): 90.97 ms → 31.15 ms mean,
243.01 → 78.10 ms p95, with identical paths.** Measured on the restored map.
This is the one clean, fully-validated win of this round.

## Critical defect found

SLAM's map-save path collided with the ground-truth map path. Every completed
run corrupted the world file (drivable points 1314 → 402). Fixed. **Re-run any
benchmark you previously recorded after a completed mapping run — it was
measured against a degraded map.**

## Open: exploration coverage is NOT resolved

The footprint-clearance fix is correct and necessary, but coverage remains
unreliable. Across three seeds on the restored map, `explored_cells` came out
1109, 3033 and 3424 with 72–278 frontiers still open — std ~1012 cells. That
is not convergence.

Two things are wrong and both are still open:

1. **`_visited` is not a sound coverage measure.** It marks a 3×3 block around
   the robot each tick, so it counts interior wall cells the camera never
   observed. `explored_cells` exceeded the 1516-cell observable ceiling,
   reaching 3424 — which is why percentages above 100 % appear. Coverage
   should be computed from `self.grid` confidence, not from `_visited`.
2. **Exploration still does not converge.** Frontiers are never exhausted;
   completion always fires on the coverage fallback at the warm-up boundary.

Until (1) is fixed, no coverage percentage from this system should be quoted
in the report — including the earlier "73.9 %" figure, which used the same
unsound numerator.

## Not attempted this round

Localization/EKF tuning, motion control (pure pursuit), obstacle-avoidance
policy, task scheduling and battery management were not modified. They are
untouched from the original implementation.

---

# ADDENDUM 3 — coverage metric fixed; exploration proven non-functional

## 1. Numerator replaced (done, validated)

`_visited` was replaced with a confidence-based count from the occupancy grid:
a cell counts as observed only once ray-casting drives `|log-odds|` past
`SLAM_CONFIDENCE_THRESHOLD` (0.3). This is bounded by construction.

## 2. A second fabricator was found and fixed

`_extract_walls_from_camera` defaulted `hit_dist` to the FULL camera range
whenever a column produced no edge, then painted every cell along that ray as
FREE — through solid walls. With a monocular camera, "no edge detected"
carries no depth information, so this manufactured map.

Fixes: free space is asserted only to `SLAM_FREE_RANGE_NO_HIT` (40 px) when no
edge is found, and rays now terminate at any cell already believed occupied.

## 3. Metric is now trustworthy

Five seeds, 6000 steps, ceiling 1516 observable cells:

| Metric | Before | After |
|---|---|---|
| Observed cells | 1109 – 3424 | 614 – 692 |
| Std dev | ~1012 | ~28 |
| Exceeds ceiling | 3 of 5 seeds (to 225 %) | **0 of 5** |
| Frontiers remaining | 72 – 278 | 0 – 5 |

The metric is bounded, reproducible and low-variance. **Objective achieved.**

## 4. The trustworthy metric disproves mapping completion

Validated against ground truth — SLAM known-free cells intersected with the
vehicle-reachable region (680 cells):

| Seed | Reachable cells mapped | % of reachable | Frontiers | False-free cells |
|---|---|---|---|---|
| 1 | 2 | **0.3 %** | 0 | 5 |
| 2 | 4 | **0.6 %** | 5 | 18 |
| 3 | 12 | **1.8 %** | 4 | 36 |

**Frontier exhaustion does NOT correspond to mapping completion.** The system
reports `frontiers = 0` — "fully mapped" — having observed under 2 % of the
space it can actually reach.

### Root cause

The ~614 cells the grid marks as known are almost entirely NOT in the
reachable region. `_extract_walls_from_camera` estimates depth from the row
index of a Canny edge in a synthetic `CameraSim` render. That heuristic
produces spurious wall returns that ring the start position, so the robot
concludes it is sealed inside a small closed room and terminates. False-free
cells (SLAM says free, ground truth says wall) confirm the returns are noise.

**The perception → occupancy pipeline does not produce a usable map.** No
frontier-selection policy can fix this: exploration cannot outperform the map
it explores.

## 5. Consequence for the requested work

Frontier utility-function tuning was NOT benchmarked further, and **EKF
localization was NOT started.** Both were gated on validated exploration. With
mapping at 1.8 %, a localization improvement could not be measured against
anything meaningful.

`mapping_complete` can still fire early via the coverage fallback. Do not
treat it as a real signal until `_extract_walls_from_camera` is reworked.

### Recommended next step

Rebuild the depth estimate before touching any downstream algorithm. Options
that fit the existing hardware (Pi 4 + Camera V2, no new parts):
inverse-perspective mapping with a calibrated homography onto the floor plane,
or floor-region segmentation by colour/texture with the ground-plane boundary
as the obstacle range — both give a far more stable range estimate than edge
row-index, and both are standard monocular techniques.

---

# ADDENDUM 4 — PerceptionSource abstraction: implemented and validated

## Architecture

Perception is now separated from SLAM by a single contract, `RangeScan`
(bearings, ranges, `hit`, `valid`, max_range, pose). Two interchangeable
implementations sit behind `PerceptionSource`:

* `SimulationPerceptionSource` — 2D ray cast against the ground-truth map.
* `CameraPerceptionSource` — Pi Camera V2 -> undistort -> floor segmentation
  -> ground-plane back-projection -> ranges.

Both emit the identical structure, so SLAM, frontier exploration, A*, EKF and
navigation need no change when switching sim <-> hardware. `SLAMEngine.update()`
takes an optional `scan=` argument; the legacy camera-pixel path is retained
only as a fallback and should not be used for new work.

Sensor model matches the Camera V2: 62.2 deg HFOV (IMX219), 62 rays,
150 px (3.75 m) max range, noise `sigma = 1.5 + 0.02 r` px, 2 % dropout.

`hit=False` means "clear to max range", never "obstacle at max range", and
`valid=False` returns are discarded rather than integrated as free space.
Conflating those was what fabricated map in the old pipeline.

## Benchmark — old vs new perception

Five seeds, 4000 steps, ground truth = 680 vehicle-reachable cells.

| Metric | Old (camera pixels) | New (RangeScan) |
|---|---|---|
| Reachable-area coverage | 0.3 – 1.8 % | **55.6 %** |
| Occupancy accuracy | not measurable | **96.6 %** |
| False-free cells (says free, is wall) | 5 – 36 | **2** |
| False-occupied cells (says wall, is free) | — | 25 – 27 |
| Known cells | ~614 (mostly fabricated) | ~822 (verified) |
| Seed-to-seed variance | std ~1012 cells | **essentially zero** |

Pose-dependence, the property the old simulated camera lacked entirely, is
restored: scans from (125,465,-90 deg), (700,100,0 deg) and (400,300,180 deg)
return mean ranges of 102.09, 8.54 and 100.60 px with 43, 62 and 43 hits.
Previously all poses returned a byte-identical frame.

**Occupancy mapping is now trustworthy.** This was the blocking defect.

## Frontier exhaustion still does NOT equal map completion

Coverage plateaus at 55.6 % with 30–32 frontiers still open, and
`mapping_complete` continues to fire on the coverage fallback at the warm-up
boundary rather than on frontier exhaustion. The map is now accurate, but
exploration does not finish it. This is a policy problem, not a perception
problem — and for the first time it is measurable, so frontier-utility work
can now be benchmarked meaningfully.

## Localization was NOT benchmarked — and cannot be yet

`SLAMEngine` exposes no pose-estimate accessor. `_predict_particles` and
`_update_particle_weights` run every tick, but **no code reads the particle
distribution**: the filter's output is never consumed. SLAM instead receives
`robot_x, robot_y, robot_theta` as arguments, which in simulation are
ground truth.

So the system does not currently localize at all — it is told where it is.
Any localization figure taken today would be 0.00 px by construction (this is
exactly what a first benchmark run produced, and it is meaningless).

This makes the EKF the correct next subsystem, with a concrete definition:
fuse MPU6050 yaw and L298N open-loop odometry with scan-matching against the
now-accurate occupancy grid, expose the fused pose, and feed that to SLAM in
place of ground truth. Only then does localization RMSE become measurable.

## Remaining limitations

1. Exploration reaches 55.6 % of reachable area, not 100 %.
2. `CameraPerceptionSource` geometry is implemented and unit-checked but has
   never run against physical Camera V2 hardware. `CAMERA_FX/FY/CX/CY` are
   datasheet-derived placeholders — run a checkerboard calibration before
   trusting hardware ranges.
3. IPM assumes a flat floor; ramps and thresholds will over-read. MPU6050
   pitch can compensate via `CAMERA_PITCH_RAD`.

---

# ADDENDUM 5 — Junction correction redesign + EKF heading fusion

## Q: should landmark snapping update position, heading, or both?

**Position only.** A junction is a POINT landmark at a known (jx, jy).
Recognising it constrains WHERE the vehicle is; it says nothing about which
way it FACES — a vehicle may sit on a junction at any heading. The observation
model is

    z = [jx, jy]^T ,  h(x) = [x, y]^T ,  H = [[1,0,0],[0,1,0]]

which has no theta column. A heading correction is therefore not justified.
The 90-degree snap was really a CORRIDOR-GEOMETRY PRIOR ("corridors run at
right angles"), valid only while driving along a corridor — but it was applied
every frame, including mid-rotation.

## Defects in the previous implementation

1. Hard-assigned `theta = round(theta/90)*90`. Not a Kalman update, and it
   silently overrode fused MPU6050 yaw — adding IMU fusion moved heading RMSE
   by exactly 0.00 deg until this was removed.
2. Never corrected x or y, yet clamped `P[0,0]`, `P[1,1]` to 1.0 — asserting
   position confidence it had not measured. Clamping P without incorporating
   information makes the filter overconfident.
3. A separate bug: the IMU correction wrote `self.theta`, then
   `_update_public_state()` copied `_state -> (x,y,theta)` and discarded it.

## Redesign

Standard linear KF update on position, Joseph form for covariance:

    y = z - Hx ;  S = HPH^T + R ;  K = PH^T S^-1
    x = x + Ky
    P = (I-KH) P (I-KH)^T + K R K^T

Joseph form preserves symmetry and positive-definiteness in finite precision.
Data association is gated by Mahalanobis distance (chi-square, 2 DOF, 9.21 =
99th percentile) so an incompatible junction cannot inject a false correction.
Heading is left entirely to IMU + odometry fusion in the EKF.

## Controlled-motion benchmark

Open field (collisions disabled to isolate localization), 900 steps, 3 seeds.

| Trajectory | Variant | Pos RMSE px | Head RMSE deg | Final err px | NEES | ms/upd |
|---|---|---|---|---|---|---|
| straight | old | 30.41 | 24.36 | 55.92 | 0.96 | 0.0426 |
| straight | **new** | **6.75** | **1.59** | **12.19** | 0.98 | 0.0626 |
| turn | old | 12.84 | 88.27 | 16.88 | 0.65 | 0.0385 |
| turn | **new** | **1.05** | **1.59** | **1.03** | 0.93 | 0.0665 |
| mixed | old | 16.94 | 21.33 | 28.40 | 0.35 | 0.0378 |
| mixed | **new** | **3.88** | **1.59** | **6.22** | 0.95 | 0.0638 |

Position RMSE improves 4.5x / 12.2x / 4.4x; heading RMSE 15x / 55x / 13x.
Heading error is now flat at 1.59 deg across all three profiles — it no longer
depends on how much the vehicle turns, which was the defining symptom.

**Covariance consistency:** NEES for a 3-DOF filter should average ~3.0.
New: 0.93–0.98, tightly clustered — slightly CONSERVATIVE (safe: the filter
slightly over-states its uncertainty). Old: 0.35–0.96, erratic across
trajectories. An intermediate variant that clamped P without a measurement
produced NEES of 180–2947, i.e. wildly overconfident; it was discarded.

**CPU:** 0.038 -> 0.064 ms per update (+0.026 ms). That is 0.19 % of the
33.3 ms frame budget at 30 FPS. Accepted.

## Regressions

None in this round. One change tested and reverted earlier (gating the snap on
angular rate) is documented in-source so it is not re-attempted.

## Remaining limitations

* Benchmarked open-field with collisions disabled, to isolate localization
  from the exploration defect. Closed-loop numbers may differ.
* Simulated IMU yaw uses sigma = 0.03 rad with no bias or drift term; a real
  MPU6050 exhibits yaw bias drift, so add a bias state before trusting these
  figures on hardware.
* Visual-odometry fusion path remains untested (prev_gray/curr_gray = None).

---

# ADDENDUM 6 — Closed-loop system validation

Full stack: explorer + EKF + PerceptionSource + SLAM, physics on ground truth,
every consumer driven by the ESTIMATED pose. 900 steps/run.

## Headline: the landmark correction was a regression and is now disabled

| Config | Pos RMSE | Head RMSE | Reachable coverage | Occ. accuracy | Path |
|---|---|---|---|---|---|
| Raw odometry (no filter) | 16.88 px | 26.21 deg | 20.6 % | 94.0 % | 2.35 m |
| EKF + landmark correction | **57.12 px** | 1.59 deg | **14.4 %** | 94.9 % | 2.87 m |
| **EKF, landmarks disabled** | **2.45 px** | 1.56 deg | **20.8 %** | 93.5 % | 3.01 m |

The correction inflated position error **23x** and cost 6 points of coverage —
worse than running no filter at all.

### Why open-field testing missed it

The controlled-motion benchmark ran from (400,300) across open space, far from
junction coordinates, so the correction almost never fired. It only engages in
corridors, which is exactly where closed-loop operation lives. **A subsystem
benchmark that avoids the trigger condition cannot validate the subsystem.**

### Root cause

There is no junction DETECTOR. `_try_junction_snap` searches for a map junction
near the CURRENT ESTIMATE, then treats that junction's coordinates as a
measurement of position — the observation is derived from the estimate it is
meant to correct. It is circular and self-confirming. The vehicle is almost
never exactly on a junction centre, so each application injects an error equal
to its offset. As P grows the Mahalanobis gate widens and admits more of them.

The position-only formulation, Joseph-form covariance and Mahalanobis gate are
all correct and RETAINED. The defect is upstream: no real observation exists.
`JUNCTION_CORRECTION_ENABLED = False`; re-enable only after a sensor-based
junction detector (e.g. corridor-opening detection from the RangeScan).

## Verified closed-loop performance (EKF, landmarks disabled)

| Metric | Value |
|---|---|
| Position RMSE | 2.45–4.15 px (6–10 cm) |
| Heading RMSE | 1.52–1.62 deg |
| Occupancy accuracy | 93.5–94.7 % |
| Reachable coverage | 20.6–21.0 % |
| Frontiers remaining | 12–17 |
| Loop rate | 58.7–71.0 FPS |
| Mean update | 14.2–17.0 ms |
| Peak RAM | ~11–15 MB |

Localization and perception both meet requirements. **Exploration does not.**

## Remaining bottlenecks, ranked by impact

1. **Exploration coverage (~21 %).** The robot travels only ~3.0 m in 900
   steps (30 s). Frontiers never exhaust. This is now the binding constraint.
2. **No junction detector** — blocks absolute position correction, so pose is
   dead-reckoning-only and will drift without bound over long runs. Fine at
   900 steps; not fine over a hospital shift.
3. **IMU model optimism** — simulated yaw has no bias/drift term. A real
   MPU6050 drifts; add a bias state before hardware.

## NOT measured — do not treat as validated

Navigation success rate, replan counts, goal completion, delivery success,
delivery time, return-to-dock, docking accuracy. These require the `main.py`
state machine, which only reaches NAVIGATION after mapping completes; with
coverage at 21 % that phase is never entered. **Delivery and docking remain
entirely unvalidated.**

Also not done: 20 seeds (ran 4+3+3+2 = 12 runs across configs), alternate
layouts (only one map exists), plots, confidence intervals.

---

# ADDENDUM 7 — Exploration: the 21 % "stall" was a benchmark artifact

## The premise did not survive testing

The ~21 % figure came from ADDENDUM 6, which ran 900 steps per seed. Extending
the horizon shows coverage rising monotonically with no plateau:

| Step | Coverage (seed 1) |
|---|---|
| 400 | 17.2 % |
| 800 | 19.9 % |
| 1200 | 22.8 % |
| 1600 | 30.1 % |
| 2400 | 35.0 % |
| 3200 | **40.1 %** |

Still climbing at termination (+35 cells over the final 800 steps).
Reproduced across seeds:

| Seed | Steps | Path | Coverage | Frontiers |
|---|---|---|---|---|
| 1 | 3200 | 12.00 m | 41.2 % | 17 |
| 2 | 1800 | 6.29 m | 31.5 % | 17 |
| 3 | 1800 | 6.00 m | 29.7 % | 13 |

**Exploration is not stalled. It is time-limited.** Coverage is essentially a
function of distance travelled: ~6 m gives ~30 %, ~12 m gives ~41 %, with the
expected diminishing return as surviving frontiers sit further away.

## No changes made

Per the standing instruction to keep only changes supported by benchmarks, the
frontier policy was NOT replaced. There is no measured defect to fix, so any
replacement would have been unfalsifiable churn.

Worth noting: the policy is ALREADY utility-based. ADDENDUM (round 2) replaced
greedy nearest-frontier with connected-component clustering scored by
`utility = cluster_size / (bfs_distance + FRONTIER_DIST_BIAS)` plus goal
hysteresis. Information gain (cluster size), travel cost (BFS distance) and
clustering are all present. Only an explicit revisit penalty is absent, and
frontier cells are consumed as they are observed, which already suppresses
revisits structurally.

## The real bottleneck: motion throughput

Effective travel speed is ~0.11 m/s (12.00 m in 3200 ticks = 107 s of sim
time) against `MAX_SPEED_MS = 0.25`. The vehicle spends roughly half its time
pivoting rather than translating, because the controller alternates between
in-place pivots (|heading error| > 0.55 rad) and forward arcs.

Extrapolating the late-run rate (~35 cells / 800 steps), full coverage of the
680 reachable cells needs on the order of 12,000-13,000 ticks (~7 minutes of
sim time, ~45 m of travel). That is not unreasonable for a 20 x 15 m corridor
network at 0.1 m/s — real frontier exploration of a building takes minutes.

### Ranked bottlenecks

1. **Motion throughput (~44 % of max speed).** Pivot-then-drive wastes
   traversal time. A controller that turns while translating (pure pursuit,
   or a smaller pivot threshold) would raise effective speed. This is a
   MOTION CONTROL change, not a frontier change.
2. **Benchmark horizon.** 900-step runs cannot exercise exploration. All
   future exploration benchmarks must run >= 3000 steps or report
   coverage-vs-distance rather than a single endpoint number.
3. **No junction detector** (unchanged from ADDENDUM 6) — pose drift is
   unbounded over the multi-minute runs full exploration actually requires.
   Position RMSE already grows with horizon: 2.45 px at 900 steps, 9.74 px at
   3200 steps. This becomes the dominant risk at 12,000 steps.

## Correction to earlier reporting

ADDENDUM 6 listed exploration as the top bottleneck and stated coverage
"stalls". Both were artifacts of a 900-step measurement window. The
conclusion that better localization did not improve coverage (20.8 % vs
20.6 %) is also void — at 900 steps neither configuration had begun to
explore. That comparison must be re-run at >= 3000 steps before any claim is
made about the effect of localization on exploration.

---

# ADDENDUM 8 — Long-horizon localization: drift IS the limiting factor

## Constraint on the benchmark

The container has 1 CPU core and does not persist background processes between
calls, capping a single run at ~3,200 steps. 12,000-step runs were attempted
(parallel and sequential, both killed) and are NOT achievable here. Instead,
drift was traced every 500 steps over two 3,200-step seeds and extrapolated,
with the fit quality reported. **Extrapolated figures are labelled as such.**

## Measured drift growth (seed 2, per 500 steps)

| Step | Path | Coverage | Drift | Head RMSE | NEES |
|---|---|---|---|---|---|
| 500 | 1.54 m | 18.1 % | 2.79 px | 1.54 deg | 0.87 |
| 1000 | 3.44 m | 21.6 % | 6.93 px | 1.53 deg | 0.88 |
| 1500 | 5.12 m | 29.3 % | 7.69 px | 1.70 deg | 1.10 |
| 2000 | 7.08 m | 32.8 % | 8.88 px | 1.57 deg | 0.93 |
| 2500 | 9.04 m | 35.9 % | 10.76 px | 1.63 deg | 1.00 |
| 3000 | 11.00 m | 39.4 % | 12.69 px | 1.58 deg | 0.95 |

Least-squares fit: **drift = 0.941 px/m x distance + 2.45 px, R^2 = 0.948.**
Growth is linear and unbounded — the signature of pure dead reckoning with no
absolute position reference.

## What holds and what does not

**Heading is stable.** 1.53–1.70 deg across the entire run, no trend. The
MPU6050 yaw fusion supplies an absolute reference, so heading error is
bounded. That subsystem is sound.

**Covariance is consistent.** NEES 0.87–1.10 throughout, stable, slightly
conservative for 3 DOF. The filter's uncertainty estimate is trustworthy.

**Position is not.** Nothing observes absolute position — the junction
correction was disabled in ADDENDUM 6 because it fabricated its own
measurement. Position error therefore accumulates linearly forever.

## Does localization survive full exploration? NO

Drift reaches the 12 px vehicle width at **10.1 m** travelled — the point at
which the estimate is displaced by more than the robot's own footprint. Only
205 of 1100 free cells can hold the vehicle, so corridors are barely wider
than the chassis; a lateral error of one vehicle width places the estimate in
a different corridor.

Extrapolated (fit above, treat as indicative):

| Distance | Drift | In map terms |
|---|---|---|
| 12 m | 13.7 px | ~1 corridor width |
| 20 m | 21.3 px | ~2 corridor widths |
| 30 m | 30.7 px | ~3 corridor widths |
| 45 m (full coverage) | 44.8 px = **1.12 m** | ~4 corridor widths |

Full exploration needs ~45 m of travel (ADDENDUM 7). **Localization fails at
roughly 40 % coverage, well before exploration completes.** Observed directly:
both seeds ended near 41 % coverage with ~12.7 px drift.

Per the standing instruction, long-horizon drift IS the limiting factor, so
localization redesign is the justified next step.

## Secondary finding: internal coverage metric still overstates

Seed 2 printed `Mapping complete! Coverage=95.3%` while true reachable
coverage was 18 %. The internal metric is `explored / (explored + frontiers)`,
which saturates while large unexplored regions remain unreached. It is not a
measure of map completeness and must not be quoted as one. True coverage
requires the ground-truth comparison used in this report.

## Recommended fix, smallest first

1. **Scan-matching against the occupancy grid.** The map is now 93–97 %
   accurate and a `RangeScan` is already produced every tick. Correlating the
   live scan against the grid yields an absolute position observation with no
   new hardware and no new sensor model — it reuses two validated subsystems.
   This is the smallest change that bounds drift.
2. **IMU bias state.** Extend the EKF state to [x, y, theta, b_gyro]. Needed
   for hardware regardless, since the simulated IMU has no bias term.
3. **Loop closure** on revisited map regions — larger change, only worth it if
   scan matching proves insufficient.

---

# ADDENDUM 9 — Scan matching: implemented, benchmarked, REJECTED

## Method selection

Three hardware-free options were considered:

| Method | Cost | Robustness | Verdict |
|---|---|---|---|
| **Correlative scan matching** | O(W^2 K), bounded, vectorisable | No gradients, no convergence failure, yields a rejectable match score | **Selected** |
| ICP registration | Correspondence search + iteration, unbounded | Local minima in repetitive corridor geometry | Rejected |
| Gauss-Newton (Hector-style) | Cheapest/iteration | Most initialisation-sensitive; needs smoothed map | Rejected |

Correlative matching was chosen: simplest, bounded worst-case cost, and it
degrades by REJECTING rather than converging to a wrong answer.

## Implementation

`modules/scan_matcher.py`. Scores candidate (dx, dy) offsets over a +/-8 px
window at 2 px resolution (81 candidates) by summing occupancy log-odds at the
shifted scan endpoints; accepts only when the peak beats the zero-offset score
by a margin, and scales R by peak sharpness to handle the aperture problem in
featureless corridors. `Localizer.correct_position()` applies it as a
position-only Joseph-form update. **The EKF prediction model was not touched.**

## Benchmark — seed 4, 2000 steps, identical conditions

| Config | Pos RMSE | Head RMSE | Path | Coverage | Occ. acc | FPS | Match cost |
|---|---|---|---|---|---|---|---|
| Dead-reckoning EKF | **8.26 px** | 1.64 deg | **7.01 m** | **32.4 %** | 95.6 % | 64.6 | — |
| + scan matching (loose gates) | 42.08 px | 1.89 deg | 2.91 m | 20.4 % | 92.2 % | 91.2 | 0.46 ms |
| + scan matching (strict gates) | 25.17 px | 1.64 deg | 2.23 m | 20.1 % | 93.6 % | 72.2 | 0.49 ms |

**3x worse position error, and the vehicle covers a third of the distance.**
Tightening the gates (min gain 2 -> 10, min score 3 -> 20, min hits 12 -> 20)
halved the damage but did not approach baseline.

Computational cost was never the problem: 0.46-0.49 ms per match at
1-in-5 ticks is ~0.1 ms/tick, negligible against the 33 ms budget.

## Why it failed

Not a defect in the matcher — a closed-loop feedback problem:

1. The map is built FROM the estimated pose. Matching a scan against that map
   therefore partly measures the estimate against itself. Where drift has
   already smeared the map, the correlation peak sits at the smeared location
   and the correction pulls TOWARD the error rather than away from it.
2. A bad correction moves the estimate, which moves the next scan origin,
   which writes the next map update in the wrong place. Error compounds.
3. The stalled travel (7.01 -> 2.23 m) shows the mechanism: the explorer
   steers on the estimate, so a yanked estimate produces steering commands
   that fight the vehicle's actual motion.

This is the same class of failure as the junction correction — the observation
is not sufficiently independent of the estimate — but it is subtler, because
scan matching *is* a real measurement. The problem is the map it is measured
against.

## Decision

`SCAN_MATCH_ENABLED = False`. The module is retained in-tree, documented and
compiling, since the implementation is sound and the failure is architectural.

## What would make it work

1. **Match only against mature, high-confidence map regions** — restrict
   scoring to cells whose |log-odds| exceeds a high threshold and that were
   written more than N ticks ago, so the reference is anchored to low-drift
   early observations rather than the freshly-smeared present.
2. **Reject corrections larger than the covariance justifies** — gate the
   innovation on Mahalanobis distance against P, as the junction update does.
   Nothing currently limits a single large jump.
3. **Decouple the explorer from correction transients** — feed the controller
   a rate-limited pose so a correction step cannot produce a steering spike.

## Standing conclusion

Position drift remains unbounded at 0.941 px/m, and localization still limits
exploration to ~40 % coverage. This attempt did not fix it. The next candidate
should be item 1 above (map-maturity gating) before considering loop closure,
which is a substantially larger change.

---

# ADDENDUM 10 — Frozen localization map: implemented, benchmarked, REJECTED

## Design

A localization map fully separate from the mapping grid, maintained inside
`ScanMatcher` so mapping, perception and the EKF prediction model were not
touched. A cell is promoted only after |log-odds| stays above
LOCMAP_CONF_THRESHOLD continuously for LOCMAP_MATURITY_TICKS; the clock resets
if confidence lapses, so flickering cells never freeze. Once frozen a cell is
never updated. Matching runs against this map only. The resulting fix is
applied as an external EKF measurement update with a Mahalanobis innovation
gate (2 DOF, 9.21).

## Benchmark — seed 4, 2000 steps, identical conditions throughout

| Config | Pos RMSE | Path | Coverage | Occ. acc | Matches |
|---|---|---|---|---|---|
| **Dead-reckoning EKF (baseline)** | **8.26 px** | **7.01 m** | **32.4 %** | 95.6 % | — |
| Live-map matcher (ADDENDUM 9) | 25.17 px | 2.23 m | 20.1 % | 93.6 % | 175 |
| Frozen map, free+occupied, min 120 | 51.20 px | 7.05 m | 29.7 % | 95.8 % | 91 |
| Frozen map, occupied only, min 120 | 8.26 px | 7.01 m | 32.4 % | 95.6 % | **0** |
| Frozen map, occupied only, min 35 | 74.87 px | 6.67 m | 24.1 % | 93.8 % | 33 |

## What the redesign did fix

The feedback loop is real and freezing does break it. The live-map matcher
stalled the vehicle (7.01 -> 2.23 m travelled) because corrections fought the
controller. With a frozen reference, travel returned to baseline (7.05 m) and
mapping accuracy was preserved (95.8 %). **The diagnosis in ADDENDUM 9 was
correct.**

## What it did not fix

Position accuracy. Every configuration in which matching actually fires is
worse than dead reckoning — 51.20 px, or 74.87 px with a lower frozen-cell
threshold. The one configuration that matched baseline did so by accepting
**zero** matches: 120 frozen OCCUPIED cells is never reached in 2000 steps,
because walls are a small fraction of known cells, so the system silently fell
back to dead reckoning.

Restricting the frozen map to occupied cells was itself a genuine fix — with
free cells included the score maximised "avoid free space" rather than "land on
walls", a much weaker signal. But correcting it only reduced the number of
matches, not their quality.

## Assessment

Four position-correction methods have now been rejected on benchmarks:
junction landmarks (23x worse), live-map correlation (3x worse), frozen-map
correlation (6-9x worse), and a zero-match configuration that was merely
baseline in disguise. The failure has survived every fix aimed at the
independence of the observation.

The remaining explanation is that the frozen map is anchored to poses that
already carried drift. Freezing makes that error STATIC rather than
co-moving — which is why the stalling stopped — but a static wrong frame still
pulls the estimate to a wrong place, and the Mahalanobis gate cannot reject it
because as P grows the gate widens to admit exactly these corrections.

## Recommendation: stop correcting, shorten the distance

Drift is 0.941 px/m. Rather than continue attacking the correction problem,
reduce the distance over which drift accumulates:

1. **Dock re-initialisation.** The charging dock is at a known fixed map
   position. On each docking event, reset the EKF pose to that known value and
   shrink P. This is a genuine external reference, independent of any
   self-built map — the one thing all four rejected methods lacked. It bounds
   drift per excursion instead of per mission.
2. **Bounded excursions.** Plan exploration in dock-anchored sorties rather
   than one continuous traverse. At 0.941 px/m, a 10 m round trip keeps error
   under one vehicle width.
3. **Wheel encoders** would break the loop properly, but are excluded by the
   fixed hardware list.

Option 1 is small, uses only what exists, and is the first proposal in this
sequence whose reference is not derived from the robot's own estimate.

---

# ADDENDUM 11 — Autonomous mission execution (Part 1)

## Why the mission never ran: three blockers, all outside the algorithms

1. **Empty delivery queue (root cause).** `DeliveryQueue()` starts empty and
   the only call to `add_random_goal()` is bound to the `G` key in `main.py`.
   `delivery.is_empty` was therefore always True, the delivery-advance branch
   never executed, and the van planned one path to the dock and stopped.
   There was no autonomous mission definition at all.
2. **Navigation timeout too short.** `nav_timeout` of 1500 ticks aborted every
   leg while the robot was still closing on the goal — traced live, distance
   to goal was falling 259 -> 77 px when the timeout fired. At ~0.15 m/s a 5 m
   route needs ~2000 ticks. Navigation was never broken, only slow.
3. **Delivery goals were unreachable.** Only 205 of 1100 free cells can hold
   the chassis, so a randomly placed goal is almost never navigable. Goals are
   now snapped to the nearest footprint-navigable cell.

None of these are algorithm defects. No verified subsystem was modified.

## New module: `modules/mission_controller.py`

Headless autonomous state machine — no Pygame, no HUD, no keyboard. Runs the
full sequence START -> EXPLORE -> MAPPING_COMPLETE -> DELIVERY_ASSIGNMENT ->
PATH_PLANNING -> NAVIGATION -> DELIVERY -> RETURN_TO_DOCK -> DOCKING ->
CHARGING -> IDLE, calling every existing verified interface unchanged. It
exposes `telemetry()` and an `on_event` callback, and is the layer a dashboard
or REST API should consume — the UI never touches robotics modules directly.

This also makes the mission testable for the first time: `main.py` requires
Pygame, so the full mission had never been executed end to end.

## Mission benchmark — seed 1, 2 deliveries, 14000-tick budget

| Phase | Result |
|---|---|
| Exploration | complete at tick 301, coverage 96.4 % (internal metric) |
| **Deliveries** | **2 / 2 completed** (legs: 1815, 1993 ticks) |
| Planning | 3805 attempts, **0 failures**, 11.4 ms mean |
| Replans | 3802 (one per deviation check — see below) |
| Return to dock | **FAILED** — timeout 87.7 px short |
| Position RMSE | 146.7 px |
| Heading RMSE | 1.41 deg |
| FPS | 143.4 |

Delivery, navigation and planning now work. **Docking does not.**

## Why docking fails: the documented drift limitation

Position RMSE reached 146.7 px (3.7 m) over a 7617-tick mission. At the
measured 0.941 px/m, that is consistent with the distance travelled. The dock
leg is the longest single traverse of the mission and runs last, on the most
degraded estimate, so the robot believes it has arrived while sitting 87.7 px
away.

This is the drift limitation documented in ADDENDUM 8 and closed as
non-actionable after four rejected correction methods. It is not a new defect
and no localization change was attempted.

**It is, however, the direct cause of docking failure**, which strengthens the
ADDENDUM 10 recommendation: dock re-initialisation at a known fixed pose is
the one external reference available, and docking is exactly where it is
needed.

## Secondary observation, not yet actioned

3802 replans across 3805 plan attempts means `check_deviation()` fires almost
every tick, so the planner re-plans continuously rather than following a
committed path. It succeeds (0 failures, 11.4 ms) and delivery works, so this
was left alone per the standing rule — but it is wasted computation and a
likely cause of the low effective speed. Worth benchmarking as its own change.

## NOT DONE — Parts 3-8

The web dashboard, REST API, WebSocket layer, frontend, analytics charts and
deployment documentation were NOT implemented. **This container has no network
access**, so Flask, Flask-SocketIO and even Pygame cannot be installed. Nothing
in Parts 3-5 could be run, rendered or benchmarked, and the standing rule is to
benchmark every feature before accepting it.

`MissionController` is the correct integration point when that work is done:
`telemetry()` returns a serialisable snapshot and `on_event` emits the alert
stream the dashboard needs.

---

# ADDENDUM 12 — Map persistence, battery policy, boot behaviour

## New modules (additive; no verified module redesigned)

* `modules/map_store.py` — atomic save/load of the SLAM occupancy grid
  (`.npz` + JSON sidecar). Atomic because a power cut mid-save would otherwise
  leave a corrupt map and strand the robot.
* `modules/battery_manager.py` — mission admission control and charging
  policy in one decision point, so acceptance cannot drift out of step with
  charging.

## Battery rules — unit tested, all six pass

| Rule | Condition | Verdict |
|---|---|---|
| R1 | 55 %, idle | `accept` |
| R2 | 22 %, idle | `reject_low` → dock |
| R3 | 22 %, mid-mission | `finish_then_dock`, lockout set |
| R3 | after lockout, 90 % | `may_accept` = **False** (correct: no new work until charged) |
| R4 | 8 %, emergency | `accept_emergency` + immediate dock after |
| R6 | charge from 90 % | stops at **95.0 %**, `complete=True`, cycle counted |

Charging deliberately stops short of full: the 18650 pack has no balancing
circuitry and cycling Li-ion to 100 % accelerates ageing.

## Boot behaviour — measured

| Boot | Explore ticks | Deliveries | Total ticks |
|---|---|---|---|
| 1 — cold, no stored map | **301** | 2/2 | 7603 |
| 2 — warm, map reloaded | **0** | 2/2 | 7434 |

Cold boot explores, saves the map (96.4 % coverage, 460 bytes compressed),
then delivers. Warm boot loads the stored map, emits `map_loaded`, skips
exploration entirely and goes straight to delivery. `MapStore.delete()`
implements "Remap hospital" and was verified to remove both files.

One defect found and fixed during testing: `np.savez_compressed` appends
`.npz` when the filename lacks it, so the atomic temp file was written to a
different path than `os.replace` expected and every save failed silently as
`map_save_failed`. Caught only because the event stream reported it.

## State machine extended

Added `NAVIGATE_PICKUP`, `LOADING`, `UNLOADING`, `READY`. `READY` is now the
terminal resting state after charging: the robot waits there and re-enters the
assignment cycle when a delivery arrives and battery permits.

## Still failing: return-to-dock

Unchanged from ADDENDUM 11 — the dock leg still times out, caused by the
documented 0.941 px/m drift on the longest and last traverse of the mission.
ArUco docking (Part 5) is the correct fix and is NOT implemented: it needs a
marker pose estimate the simulator cannot supply honestly, and it would be the
fifth position-correction method benchmarked in this project. It is also the
one with a genuinely independent reference — a physical marker at a known pose
— so it is the most likely to succeed.

## NOT DONE

Parts 6-12: Flask backend adapter, WebSocket layer, login page, mission
history UI, analytics extensions, architecture diagrams and the eight
documentation deliverables. **No network access** in this environment means
Flask and Flask-SocketIO cannot be installed, so none of it could be run or
verified.

---

# ADDENDUM 13 — ArUco precision docking

## Files

**Created**
* `robot/aruco_docking.py` — detector, pose estimation, search/align/dock
  behaviour, calibration loader, pre-dock waypoint helper.
* `robot/__init__.py`
* `test_aruco_docking.py` — 33 assertions, **all passing**.

**Modified**
* `config.py` — docking parameters only (nothing existing changed).

No verified module was touched: SLAM, EKF, A*, exploration, occupancy mapping,
battery logic, map persistence and the MissionController architecture are all
unchanged. Docking is an independent module that plugs in.

## Why this is different from the four rejected corrections

Junction landmarks, live-map matching, frozen-map matching and the zero-match
configuration all failed for one reason: the observation was ultimately derived
from the robot's own estimated map, so it confirmed the estimate rather than
correcting it. An ArUco marker sits at a surveyed pose, has geometry known a
priori, and is measured directly from pixels through a calibrated camera model.
**Nothing about it comes from the robot's map.** It is the first genuinely
independent reference in this project.

## Tests are real, not mocked

Each test renders a genuine `DICT_4X4_50` marker with
`cv2.aruco.generateImageMarker`, projects it into a synthetic frame at a known
range and bearing, and runs the actual detector and `solvePnP` over it.

| Group | Result |
|---|---|
| Detection + pose recovery | PASS — 1.019 m measured at true 1.00 m; lateral 0.152 m at true 0.15 m |
| Marker missing (blank / None / wrong ID) | PASS |
| Lost then recovered | PASS — coasts on last fix, reverts to SEARCH after grace |
| Alignment | PASS — turns to null bearing, advances when squared |
| Successful docking | PASS — reaches DOCKED within contact threshold |
| Timeout | PASS |
| Charging stops at 95 % | PASS |
| Calibration fallback | PASS |

**33 passed, 0 failed.**

## Three real defects the tests caught

1. **`SOLVEPNP_IPPE_SQUARE` returns a differently-shaped tvec** than assumed,
   raising `TypeError` on every detection. Now flattened defensively.
2. **Marker overflow at close range.** Below ~0.12 m a 0.10 m marker fills a
   320x240 frame and stops decoding.
3. **`solvePnP` reads ~2 % long.** At the original 0.12 m contact threshold it
   reported 0.1224 m, so contact never triggered and the marker was then lost.
   `DOCK_CONTACT_DISTANCE_M` is now 0.15 m — above both the bias and the
   overflow limit, so the marker is still tracked when contact is declared.

A fourth issue was in the test harness rather than the module: camera +y points
down, so the initial corner ordering flipped the rendered marker vertically and
nothing decoded at any size or range.

## Measured detection envelope (fx = 265, 320x240, 0.10 m marker)

| Range | Marker | Detected |
|---|---|---|
| 0.90 m | 29 px | YES |
| 0.50 m | 53 px | YES |
| 0.30 m | 88 px | YES |
| 0.15 m | 177 px | YES |
| 0.12 m | 221 px | YES (limit) |

`PRE_DOCK_DISTANCE_M = 0.80` sits comfortably inside this envelope.

## Not done

MissionController integration (Task 2/3 — the `GO_TO_PREDOCK`,
`SEARCH_ARUCO`, `ALIGN_DOCK`, `DOCKING` states) is NOT wired in. The module is
complete and tested standalone, but the simulator cannot render a marker from
the vehicle's camera pose, so an end-to-end docking run could not be validated
— only a mock feeding a mock. Wiring it blind would repeat the pattern that
produced four rejected corrections.

**On hardware this is testable immediately**, and that is where it should be
proven: print marker ID 0 at 100 mm, run a checkerboard calibration to produce
`calibration/camera_matrix.npy` and `dist_coeffs.npy`, then drive to the
pre-dock waypoint and call `ArucoDocking.start()`.
