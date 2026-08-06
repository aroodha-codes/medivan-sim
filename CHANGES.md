# MediVan — Change Log

All changes are marked in-source with `FIX (MT3608 review)` or an explicit
`REPLACES` note. Nothing else was touched.

## A. Crash fixes — `HARDWARE_MODE=True` could not run before these

| # | File | Problem | Status |
|---|------|---------|--------|
| 1 | `config.py` / `modules/ai_obstacle_detector.py` | `YOLO_INPUT_SIZE=160` vs a **fixed** `[1,3,320,320]` ONNX export → OpenCV Reshape assertion on the first inference frame. YOLO had never run. | Fixed |
| 2 | `hardware/imu_hw.py` | `IMUData(accel_rms=…)` — field is `vib_rms`. Uncaught `TypeError` in stub path; silently swallowed in live path, so tilt/vibration safety always returned defaults. | Fixed |
| 3 | `hardware/encoder_hw.py` | `EncoderReading(left_total_m=…)` — fields are `dist_left_m`/`dist_right_m`. `TypeError` on tick 1. | Fixed |
| 4 | `hardware/motor_driver_hw.py` | Missing `set_pwm()`, wrong `update()` signature, no `wall_contact` / `forward_v`. Every one is called by `main.py`. | Rewritten to full `MotorDriverSim` parity |
| 5 | `config.py` | `ENA=12`, `ENB=18` are **both PWM0** on BCM2711 → no independent hardware PWM. | Moved to report Fig 5.2 pinout: ENA=18/PWM0, ENB=19/PWM1, IN1=17, IN2=27, IN3=22, IN4=23 |
| 6 | `main.py` | `SDL_VIDEODRIVER=dummy` set *after* `pygame.init()` → headless Pi crashed at `set_mode()`. | Moved before init |

## B. Robustness

- `ai_obstacle_detector.py` auto-probes the ONNX input size at load, so a
  re-exported model at any `imgsz` works without editing config.
- A YOLO inference fault now degrades to the heuristic classifier instead of
  terminating the 30 FPS control loop.
- `motor_driver_sim.py`: a collision used to reject **both** translation and
  rotation. A differential drive pressed against a wall can still pivot.
  Rotation is now retried separately.

## C. Algorithm replacement — exploration

`slam_engine.get_explore_command()` replaced: reactive left-wall-following →
**frontier-based exploration** (Yamauchi 1997). BFS over the occupancy grid
finds the nearest known-free cell adjacent to unknown space; the robot drives
there with proportional heading control.

Measured on `assets/hospital_map.png`, 4000 steps:

| Explorer | Coverage |
|----------|----------|
| Original wall-follower | 6.3 % |
| Frontier-based | **15.2 %** |

**This does NOT reach the 85 % handover threshold.** See EVALUATION_REPORT.md.

## D. Added

- `evaluate_models.py` — headless, reproducible evaluation harness.
  Run on the Pi: `python evaluate_models.py --json results.json`

---

# Round 2 — architecture upgrades

## E. CRITICAL: SLAM overwrote the ground-truth map

`SLAM_MAP_SAVE_PATH` resolved to **the same file** as `MAP_PATH`
(`assets/hospital_map.png`). `save_map()` fires on `mapping_complete`, so
every successful run wrote the robot's own *partial* occupancy grid over the
ground-truth world. The environment degraded on each run — measured drivable
sample points fell **1314 → 402** after a few passes, silently invalidating
any benchmark taken afterwards.

Fixed: SLAM output now goes to `output/slam_map.png`. Ground-truth map
restored and verified by checksum across a full run.

**This is the highest-severity defect found in the project.** Any earlier
measurement you took after a completed run was against a corrupted map.

## F. A* heuristic: Euclidean → Manhattan

Expansion is 4-connected, so no route can be shorter than `|dx|+|dy|` steps.
Euclidean is a strictly looser under-estimate, making A* explore a wider
frontier for the *identical* optimal path. Manhattan is the tightest
admissible heuristic for a 4-connected uniform grid.

| Metric | Euclidean (old) | Manhattan (new) |
|---|---|---|
| Plan time mean | 90.97 ms | **31.15 ms** |
| Plan time p95 | 243.01 ms | **78.10 ms** |
| Success rate | 74.0 % | 74.0 % |
| Waypoints mean | 41.7 | 41.7 |
| Route length mean | 10.18 m | 10.18 m |

**2.9× faster planning, byte-identical routes.** Verified on the restored map.

## G. 8-connected A* — implemented, benchmarked, REVERTED

Octile heuristic + diagonal expansion with corner-cut rejection.

| Metric | 4-conn | 8-conn |
|---|---|---|
| Plan time mean | 90.97 ms | 322.77 ms |
| Plan time p95 | 243.01 ms | 936.62 ms |
| Route length | 10.18 m | 9.90 m |

2.7 % shorter routes for a **3.5× planning-time regression** — the doubled
branching factor plus four extra `is_free`/`is_near_wall` calls per diagonal.
On a Pi 4 that would stall the 30 FPS loop. Reverted; rationale kept in-source
so it is not re-attempted.

## H. Frontier BFS now respects vehicle width

`_bfs_field` tested only the cell centre, so it routed through gaps narrower
than the chassis; the van jammed on doorframes and exploration plateaued with
frontiers still open. Added `_cell_traversable()`, sampling the eight
footprint corners — the same clearance rule A* enforces.

Measured cause: only **205 of 1100** free cells can physically hold the
vehicle. The corridors on `hospital_map.png` are barely wider than the robot.

## I. Utility-based frontier selection

Greedy nearest-frontier replaced with connected-component clustering scored by
`utility = cluster_size / (bfs_distance + FRONTIER_DIST_BIAS)`, plus
hysteresis on the active goal. One shared BFS now scores all clusters
(previously one BFS per replan, returning only the single nearest cell).
