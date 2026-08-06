# 🏥 MediVan — AI-Based Indoor Navigation Simulator

**An autonomous hospital delivery robot simulator powered by YOLO, Q-Learning, and A\* pathfinding.**

| Feature | Detail |
|---------|--------|
| **Platform** | Raspberry Pi 4 (ARM Cortex-A72, 4 GB RAM) |
| **Language** | Python 3.10+ |
| **Stack** | OpenCV · Pygame · NumPy · Matplotlib |
| **AI** | YOLOv8-Nano · Q-Learning RL · A\* Search · Visual SLAM |
| **Modules** | 16 interconnected subsystems |

---

## 🚀 Quick Start

```bash
# 1. Clone & enter
cd medivan_sim

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run the simulation
python main.py
```

The simulator auto-generates a default hospital map on first run.

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| **Q** | Quit |
| **TAB** | Toggle autonomous / manual mode |
| **W/A/S/D** | Manual drive (in manual mode) |
| **SPACE** | Emergency stop |
| **E** | Release emergency stop |
| **C** | Force return to dock |
| **G** | Add delivery goal (random) |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 7: Orchestrator  (main.py — 30 FPS loop)            │
├────────────┬────────────────────────────────────────────────┤
│  Layer 6   │  HUD Compositor  ·  Data Logger               │
├────────────┼────────────────────────────────────────────────┤
│  Layer 5   │  Charging Dock FSM  ·  Audio Alerts           │
│            │  Delivery Queue                                │
├────────────┼────────────────────────────────────────────────┤
│  Layer 4   │  A* Path Planner  ·  Q-Learning Agent         │
│            │  YOLO Detector  ·  Visual SLAM  ·  Localizer  │
├────────────┼────────────────────────────────────────────────┤
│  Layer 3   │  Motor Driver  ·  Bump Switches               │
├────────────┼────────────────────────────────────────────────┤
│  Layer 2   │  Encoder Sim  ·  IMU Sim  ·  Camera Sim       │
├────────────┼────────────────────────────────────────────────┤
│  Layer 1   │  Map Loader  ·  Map Editor                    │
├────────────┼────────────────────────────────────────────────┤
│  Layer 0   │  config.py (constants, enums, dataclasses)    │
└────────────┴────────────────────────────────────────────────┘
```

---

## 🧠 AI Components

### 1. YOLOv8-Nano Object Detection
- **Purpose**: Detect and classify dynamic obstacles (people, carts, equipment)
- **Mode**: ONNX inference via OpenCV DNN (production) or heuristic classifier (simulation)
- **Optimization**: 320×320 input, skip-frame (every 5th frame), ~6 FPS effective on RPi4
- **Download model**: `python scripts/download_model.py`

### 2. Q-Learning Reinforcement Learning
- **Purpose**: Learn optimal junction navigation decisions through experience
- **State space**: 54 discrete states (distance × obstacle × speed × battery)
- **Action space**: 4 actions (PROCEED, WAIT, SLOW, REROUTE)
- **Storage**: 54×4 NumPy array (~1.7 KB), <1μs per decision

### 3. A\* Path Planning
- **Purpose**: Find optimal collision-free paths through hospital corridors
- **Grid**: 10px cell resolution, 4-directional movement
- **Dynamic obstacles**: Temporarily blocked but never written to static map

### 4. Visual SLAM
- **Purpose**: Explore and map unknown environments using camera + encoders
- **Method**: Particle filter localization + log-odds occupancy grid
- **Transition**: Auto-switches to A\* navigation at 85% map coverage

---

## 📁 Project Structure

```
medivan_sim/
├── main.py                    # Orchestrator (30 FPS main loop)
├── config.py                  # All constants, enums, dataclasses
├── map_editor.py              # Standalone map drawing tool
├── profiler.py                # Performance benchmarking tool
├── replay_viewer.py           # Visual log replay application
├── requirements.txt           # Python dependencies
├── conftest.py                # Shared pytest fixtures
├── test_ai.py                 # AI module tests
├── test_slam.py               # SLAM convergence tests
├── test_modules.py            # Comprehensive module tests
│
├── assets/
│   ├── hospital_map.png       # Preloaded hospital map
│   ├── yolov8n.onnx           # [Optional] YOLO model
│   └── q_table.npy            # [Auto-saved] Q-table
│
├── modules/
│   ├── map_loader.py          # Map parser + A* cost grid
│   ├── encoder_sim.py         # Wheel encoder simulation
│   ├── imu_sim.py             # IMU daemon thread (30 Hz)
│   ├── camera_sim.py          # Camera + obstacle integration
│   ├── ai_obstacle_detector.py # YOLOv8-Nano detector
│   ├── motor_driver_sim.py    # L298N motor physics
│   ├── bump_switch_sim.py     # Perimeter contact safety
│   ├── localizer.py           # Sensor fusion localization
│   ├── path_planner.py        # A* + Pure Pursuit + Q-Learning
│   ├── q_learning_agent.py    # Reinforcement Learning agent
│   ├── slam_engine.py         # Visual SLAM engine
│   ├── charging_dock_sim.py   # 8-state dock FSM
│   ├── audio_sim.py           # Buzzer tone alerts
│   ├── delivery_queue.py      # Multi-goal delivery dispatch
│   ├── hud.py                 # 4-quadrant HUD display
│   └── data_logger.py         # JSONL logging + replay
│
├── scripts/
│   └── download_model.py      # Download YOLOv8n ONNX model
│
├── generate_report.py         # Full project PDF report
└── generate_run_report.py     # Simulation run PDF report
```

---

## 🔋 Two-Phase Operation

1. **Phase 1 — SLAM Mapping**: The robot explores the environment using wall-following, builds an occupancy grid from camera + encoder data. No prior map knowledge.

2. **Phase 2 — AI Navigation**: Once SLAM coverage reaches 85%, the robot auto-switches to A\* pathfinding with Q-Learning junction decisions and YOLO obstacle avoidance.

---

## 📊 Tools

| Tool | Command | Purpose |
|------|---------|---------|
| Simulation | `python main.py` | Run the full simulator |
| Map Editor | `python map_editor.py` | Draw/edit hospital maps |
| Profiler | `python profiler.py` | Benchmark per-module timing |
| Replay Viewer | `python replay_viewer.py` | Visual log playback |
| Model Download | `python scripts/download_model.py` | Get YOLOv8n ONNX |
| Tests | `pytest -v` | Run all tests |
| Project Report | `python generate_report.py` | Generate project PDF |
| Run Report | `python generate_run_report.py` | Generate simulation PDF |

---

## 🍓 Raspberry Pi 4 Deployment

```bash
# On your PC: export YOLO model
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320

# Transfer to RPi4
scp -r medivan_sim/ pi@raspberrypi:~/
scp yolov8n.onnx pi@raspberrypi:~/medivan_sim/assets/

# On RPi4: setup & run
cd ~/medivan_sim
pip install -r requirements.txt
python main.py
```

---

## 📜 License

This project was developed as a Major Project by **Karthik N**.

Technology Stack: Python 3.10+ | OpenCV | Pygame | NumPy  
Target Platform: Raspberry Pi 4 (ARM Cortex-A72)
