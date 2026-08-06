"""
config.py — Central configuration for the MediVan Indoor Navigation Simulator.

Every tunable constant, enum, and shared dataclass lives here.
No other module defines magic numbers — they import from config.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List, Tuple

# ════════════════════════════════════════════════
# DEPLOYMENT MODE
# ════════════════════════════════════════════════
HARDWARE_MODE: bool = False                 # Set to True when running on the physical Raspberry Pi

# ════════════════════════════════════════════════
# RASPBERRY PI GPIO CONFIGURATION (BCM Numbering)
# ════════════════════════════════════════════════
# L298N Motor Driver Pins
# ── FIX (MT3608 review): the previous map used ENA=12 and ENB=18. On the
#    BCM2711 (Pi 4) GPIO12 and GPIO18 are BOTH on PWM channel 0, so the two
#    motors could never be driven by independent hardware PWM — gpiozero
#    silently fell back to software PWM (jittery, CPU-bound).
#    GPIO18 = PWM0 and GPIO19 = PWM1 are a valid independent pair, and this
#    map now matches Figure 5.2 of the project report exactly.
PIN_MOTOR_ENA: int = 18                     # Left PWM  -> hardware PWM0
PIN_MOTOR_IN1: int = 17                     # Left Dir A
PIN_MOTOR_IN2: int = 27                     # Left Dir B
PIN_MOTOR_IN3: int = 22                     # Right Dir A
PIN_MOTOR_IN4: int = 23                     # Right Dir B
PIN_MOTOR_ENB: int = 19                     # Right PWM -> hardware PWM1

# I2C (IMU / ADC)
PIN_I2C_SDA: int = 2
PIN_I2C_SCL: int = 3
IMU_I2C_ADDR: int = 0x68                    # Standard MPU6050 address

# ════════════════════════════════════════════════
# MAP
# ════════════════════════════════════════════════
MAP_PATH: str = "assets/hospital_map.png"
MAP_SCALE_M_PER_PX: float = 0.025          # 1 px = 2.5 cm
MAP_FREE_COLOR: int = 255                   # white = drivable
MAP_WALL_COLOR: int = 0                     # black = blocked
MAP_NOGO_COLOR: Tuple[int, int, int] = (0, 0, 255)   # red (BGR in OpenCV)
CELL_SIZE_PX: int = 10                      # A* grid resolution
MAP_WIDTH: int = 800                        # default map canvas width
MAP_HEIGHT: int = 600                       # default map canvas height

# ════════════════════════════════════════════════
# VEHICLE DIMENSIONS (on-map pixel scale)
# ════════════════════════════════════════════════
VEHICLE_WIDTH_PX: int = 12
VEHICLE_LENGTH_PX: int = 18
WHEEL_DIAMETER_M: float = 0.065
WHEEL_BASE_M: float = 0.180
PULSES_PER_REV: int = 20
DIST_PER_PULSE: float = 3.14159 * WHEEL_DIAMETER_M / PULSES_PER_REV
MAX_SPEED_MS: float = 0.25                  # 25 cm/s (Slowed down for Pi AI lag)
PWM_DEADBAND: int = 30

# ════════════════════════════════════════════════
# LOCALIZATION
# ════════════════════════════════════════════════
# ── EKF heading fusion ───────────────────────────────────────────
# Measurement variance for the MPU6050 yaw observation (rad^2). The DMP-free
# complementary-filter yaw on this part is good to roughly 1-2 deg short-term.
EKF_R_IMU_YAW: float = 0.0009            # (~1.7 deg)^2
# Junction 90-degree snapping is only valid when travelling straight.
# Junction landmark measurement noise (px^2). Reflects how precisely the
# vehicle can tell it is ON a junction rather than near one.
JUNCTION_R_POS: float = 9.0              # (3 px)^2
# Chi-square gate, 2 DOF. 9.21 = 99th percentile: reject associations that
# are statistically incompatible with the current estimate.
JUNCTION_MAHALANOBIS_GATE: float = 9.21

# ── JUNCTION LANDMARK CORRECTION: DISABLED BY BENCHMARK ──────────
# Closed-loop validation, 900 steps, 3-4 seeds each:
#
#   config                        pos RMSE   head RMSE   coverage
#   raw odometry                   16.88 px    26.21 deg    20.6 %
#   EKF + landmark correction      57.12 px     1.59 deg    14.4 %
#   EKF, landmarks disabled         2.45 px     1.56 deg    20.8 %
#
# The correction inflates position error 23x (2.45 -> 57.12 px) and costs
# 6 points of exploration coverage. It is worse than using NO filter at all.
#
# ROOT CAUSE: there is no junction DETECTOR. _try_junction_snap searches for a
# map junction near the CURRENT ESTIMATE and then treats that junction's
# coordinates as a measurement of position. The "observation" is derived from
# the estimate it is meant to correct -- circular, and self-confirming. The
# vehicle is almost never exactly on a junction centre, so every application
# injects a systematic error equal to its offset from that centre. As P grows,
# the Mahalanobis gate widens and admits ever more of these false updates.
#
# The position-only redesign, Joseph-form covariance and Mahalanobis gate are
# all correct and retained -- the defect is upstream, in the absence of a real
# observation. Re-enable ONLY after implementing a sensor-based junction
# detector (e.g. corridor-opening detection from the RangeScan), which would
# supply a genuine measurement independent of the estimate.
JUNCTION_CORRECTION_ENABLED: bool = False

# ── ARUCO PRECISION DOCKING ──────────────────────────────────────
# ArUco is used ONLY for terminal docking, never for exploration,
# navigation or deliveries. A marker is visible from a few metres at best,
# so it cannot help elsewhere, and running the detector every frame would
# spend Pi CPU for nothing.
ARUCO_DICT_NAME: str = "DICT_4X4_50"
ARUCO_MARKER_ID: int = 0
ARUCO_MARKER_SIZE_M: float = 0.10        # printed side length

PRE_DOCK_DISTANCE_M: float = 0.80        # standoff where ArUco is enabled
DOCK_SPEED_PWM: int = 70                 # slow creep on final approach
SEARCH_ROTATION_PWM: int = 65            # on-the-spot sweep speed
ALIGNMENT_TOLERANCE_RAD: float = 0.07    # ~4 deg bearing error accepted
LATERAL_TOLERANCE_M: float = 0.04
# Contact threshold. Measured: solvePnP reads ~2 % long (1.019 m at a true
# 1.00 m), and below ~0.12 m a 0.10 m marker overflows a 320x240 frame and
# stops decoding. 0.15 m sits above both effects, so the marker is still
# tracked at the moment contact is declared; the last few centimetres are
# covered by the physical dock funnel rather than by vision.
DOCK_CONTACT_DISTANCE_M: float = 0.15    # charging contacts engage
DOCK_TIMEOUT_S: float = 90.0             # abort the attempt after this
MARKER_LOST_GRACE_FRAMES: int = 12       # coast on last fix before re-search

# Intrinsics live in .npy files produced by a checkerboard calibration run.
# Never hardcode them: the CAMERA_FX/FY/CX/CY constants are datasheet-derived
# placeholders used only when these files are absent, and ranges carry a
# systematic scale error until a real calibration is done.
CAMERA_CALIB_MATRIX_PATH: str = "calibration/camera_matrix.npy"
CAMERA_CALIB_DIST_PATH: str = "calibration/dist_coeffs.npy"

# ── SCAN MATCHING (absolute position observation) ────────────────
# DISABLED BY BENCHMARK -- see EVALUATION_REPORT.md ADDENDUM 9.
# Seed 4, 2000 steps, identical conditions:
#   dead-reckoning EKF : pos RMSE  8.26 px | path 7.01 m | coverage 32.4 %
#   + scan matching    : pos RMSE 25.17 px | path 2.23 m | coverage 20.1 %
# 3x worse position error and the vehicle stalls. Retained in-tree because the
# implementation is sound; the failure is a feedback-loop problem, not a bug in
# the matcher. Re-enable only with the fixes listed in the report.
SCAN_MATCH_ENABLED: bool = False
SCAN_MATCH_EVERY_N: int = 5          # ticks between matches (CPU control)
SCAN_MATCH_WINDOW_PX: int = 8        # +/- search half-window
SCAN_MATCH_STEP_PX: int = 2          # search resolution -> 9x9 = 81 candidates
SCAN_MATCH_MIN_HITS: int = 20        # need enough returns to constrain a fit
SCAN_MATCH_MIN_GAIN: float = 10.0    # peak must beat zero-offset by this
SCAN_MATCH_MIN_SCORE: float = 20.0   # absolute peak floor (mature map)
SCAN_MATCH_R_BASE: float = 4.0       # (2 px)^2 best case
SCAN_MATCH_R_MAX: float = 400.0      # (20 px)^2 when the peak is ambiguous
# ── FROZEN LOCALIZATION MAP ──────────────────────────────────────
LOCMAP_CONF_THRESHOLD: float = 1.5   # |log-odds| for a cell to count as solid
LOCMAP_MATURITY_TICKS: int = 150     # sustained confidence before freezing
LOCMAP_MIN_FROZEN_CELLS: int = 120   # do not match until the frame is real
# Mahalanobis gate on the scan-match innovation (2 DOF, 99th pct = 9.21)
SCAN_MATCH_GATE: float = 9.21

ODOM_ENCODER_WEIGHT: float = 0.20           # Low trust in open-loop estimator
ODOM_VISUAL_WEIGHT: float = 0.80            # High trust in camera (since no wheel encoders)
OPTICAL_FLOW_STRIDE: int = 3              # run optical flow every N frames
SLIP_FACTOR_RANGE: Tuple[float, float] = (0.85, 0.98)
OPTICAL_FLOW_SCALE: float = 0.005           # px-flow → metres
JUNCTION_SNAP_DIST_PX: int = 8

# ════════════════════════════════════════════════
# IMU
# ════════════════════════════════════════════════
IMU_ALPHA: float = 0.98
IMU_DT: float = 0.033                       # ~30 Hz
GYRO_BIAS: float = 0.01

# ════════════════════════════════════════════════
# VIBRATION THRESHOLDS (m/s²)
# ════════════════════════════════════════════════
VIB_SAFE_RMS: float = 1.5
VIB_WARNING_RMS: float = 2.5
VIB_DANGER_RMS: float = 4.0

# ════════════════════════════════════════════════
# CAMERA / OBSTACLE DETECTION
# ════════════════════════════════════════════════
FRAME_W: int = 320
FRAME_H: int = 240
FPS: int = 30
OBS_ROI_TOP: float = 0.30
OBS_ROI_BOTTOM: float = 0.85
OBS_MIN_AREA_PX: int = 1200
OBS_SLOW_AREA_PX: int = 4000
OBS_STOP_AREA_PX: int = 8000

# ════════════════════════════════════════════════
# BATTERY
# ════════════════════════════════════════════════
BATTERY_START_PCT: float = 100.0
LOW_BAT_THRESHOLD: float = 25.0              # return to dock immediately
EMERGENCY_BAT: float = 18.0                  # critical - force stop if not docked
CHARGE_COMPLETE: float = 95.0
DISCHARGE_MOVING: float = 0.003             # %/frame under load
DISCHARGE_STANDBY: float = 0.001            # %/frame motors off
CHARGE_RATE: float = 0.008                  # %/frame docked

# ════════════════════════════════════════════════
# DOCK
# ════════════════════════════════════════════════
DOCK_APPROACH_PWM: int = 45
DOCK_TOLERANCE_PX: int = 5
DOCK_HEADING_TOL: float = 5.0               # degrees
DOCK_MAX_TIME_S: int = 7200
ARUCO_ID: int = 0

# ════════════════════════════════════════════════
# A* PATH COST WEIGHTS
# ════════════════════════════════════════════════
COST_FREE: int = 10
COST_NEAR_WALL: int = 25
COST_JUNCTION: int = 20
COST_BUMP_ZONE: int = 15

# ════════════════════════════════════════════════
# PATH PLANNER TUNING
# ════════════════════════════════════════════════
LOOKAHEAD_DIST_PX: int = 20
PURSUIT_KP: float = 1.4
BASE_PWM: int = 160
JUNCTION_SLOW_PWM_FACTOR: float = 0.30
BUMP_SLOW_PWM_FACTOR: float = 0.35
JUNCTION_SLOW_DIST_PX: int = 20
BUMP_SLOW_DIST_PX: int = 15
JUNCTION_CLEAR_TIME_S: float = 1.0
JUNCTION_RECHECK_TIME_S: float = 2.0
REPLAN_DEVIATION_PX: int = 15

# ════════════════════════════════════════════════
# AI OBSTACLE DETECTION (YOLOv8-Nano)
# ════════════════════════════════════════════════
YOLO_MODEL_PATH: str = "assets/yolov8n.onnx"
# ── FIX (MT3608 review): this was 160. The shipped assets/yolov8n.onnx is
#    exported with a FIXED input shape of [1, 3, 320, 320], so a 160x160 blob
#    made cv2.dnn forward() throw a Reshape assertion on the very first
#    inference frame — the perception pipeline crashed the moment the model
#    was present. The detector now auto-detects this value from the model at
#    load time; this constant is the fallback and must match the export.
#    To genuinely run at 160, re-export the model:
#        yolo export model=yolov8n.pt format=onnx imgsz=160
YOLO_INPUT_SIZE: int = 320                  # must match the ONNX export size
YOLO_CONF_THRESHOLD: float = 0.35
YOLO_NMS_THRESHOLD: float = 0.45
YOLO_SKIP_FRAMES: int = 5                   # run inference every Nth frame

# ════════════════════════════════════════════════
# LOGGING
# ════════════════════════════════════════════════
LOG_EVERY_N_FRAMES: int = 2                 # reduce disk writes on Pi
YOLO_CLASSES_OF_INTEREST: dict = {
    # ── Indoor hospital-relevant COCO classes only ──
    # People (primary dynamic obstacles)
    0: "person",
    # Wheeled objects (carts, wheelchairs)
    1: "bicycle",        # proxy for wheelchair
    # Furniture & static obstacles
    56: "chair",
    57: "couch",          # waiting area seating
    59: "bed",            # hospital bed / gurney / stretcher
    60: "dining table",   # nurses station / desk / table
    13: "bench",          # corridor bench
    # Medical & portable equipment
    24: "backpack",       # bags, IV pole proxy
    26: "handbag",        # staff bags
    28: "suitcase",       # rolling equipment cases
    39: "bottle",         # IV bottle / fluid containers
    63: "laptop",         # workstations on wheels
    62: "tv",             # corridor monitors / display screens
    58: "potted plant",   # decorative plants in hallways
    73: "book",           # charts / clipboards
    74: "clock",          # wall clocks (static landmark)
    67: "cell phone",     # not obstacle but indoor indicator
}

# ════════════════════════════════════════════════
# Q-LEARNING (junction decisions)
# ════════════════════════════════════════════════
Q_TABLE_PATH: str = "assets/q_table.npy"
Q_LEARNING_RATE: float = 0.10
Q_DISCOUNT_FACTOR: float = 0.95
Q_EPSILON_START: float = 1.0
Q_EPSILON_DECAY: float = 0.995
Q_EPSILON_MIN: float = 0.01

# ════════════════════════════════════════════════
# VISUAL SLAM (camera-based mapping)
# ════════════════════════════════════════════════
SLAM_NUM_PARTICLES: int = 30                # particle filter count
# Minimum exploration before the mapping-complete test is allowed to fire.
# Without these the first tick reads as 100 % mapped (explored>0, frontiers=0).
# Frontier-exploration utility tuning.
# utility = cluster_size / (bfs_distance + FRONTIER_DIST_BIAS)
# Larger bias -> flatter distance discount -> favours big distant openings.
# A grid cell counts as OBSERVED once |log-odds| exceeds this. Must match the
# band the frontier test treats as unknown (abs(grid) <= 0.3) so that
# "observed" and "not a frontier candidate" stay consistent.
# ── PERCEPTION SOURCE ────────────────────────────────────────────
# Sensor model shared by SimulationPerceptionSource and
# CameraPerceptionSource so behaviour tuned in sim transfers to hardware.
# HFOV matches the Pi Camera Module V2 (Sony IMX219) horizontal field of view.
PERCEPTION_HFOV_DEG: float = 62.2
PERCEPTION_NUM_RAYS: int = 62            # ~1 ray per degree
PERCEPTION_MAX_RANGE_PX: int = 150       # 3.75 m at 0.025 m/px
PERCEPTION_MIN_RANGE_PX: int = 8
PERCEPTION_RANGE_STEP_PX: float = 2.0    # ray-march resolution
# Range noise: sigma = SIGMA_PX + PROPORTIONAL * range. Monocular ground-plane
# range error grows with distance because dv = (v - cy) shrinks.
PERCEPTION_NOISE_SIGMA_PX: float = 1.5
PERCEPTION_NOISE_PROPORTIONAL: float = 0.02
PERCEPTION_DROPOUT_PROB: float = 0.02

# ── CAMERA INTRINSICS (Pi Camera V2) ─────────────────────────────
# PLACEHOLDERS derived from the IMX219 datasheet (f=3.04 mm, 1.12 um pixels)
# scaled to FRAME_W x FRAME_H. Replace with values from a real checkerboard
# calibration before trusting hardware ranges.
CAMERA_FX: float = 265.0
CAMERA_FY: float = 265.0
CAMERA_CX: float = 160.0
CAMERA_CY: float = 120.0
CAMERA_HEIGHT_M: float = 0.10            # lens height above floor
CAMERA_PITCH_RAD: float = 0.0            # 0 = optical axis parallel to floor

SLAM_CONFIDENCE_THRESHOLD: float = 0.3
# When a camera column yields NO wall edge, free space is only asserted out to
# this range (px), not to SLAM_CAMERA_RANGE_PX. A monocular camera cannot
# distinguish "open corridor" from "featureless wall", so marking free to full
# range fabricates map. Keep well below SLAM_CAMERA_RANGE_PX.
SLAM_FREE_RANGE_NO_HIT: int = 40
FRONTIER_DIST_BIAS: float = 12.0
# Bonus applied to the currently-selected goal so the robot commits to it
# instead of oscillating between two clusters of similar utility.
FRONTIER_HYSTERESIS: float = 1.6
SLAM_MIN_MAPPING_FRAMES: int = 300
SLAM_MIN_EXPLORED_CELLS: int = 150
SLAM_COVERAGE_THRESHOLD: float = 0.85       # stop mapping at 85% coverage
# CRITICAL FIX - SLAM map save path collided with MAP_PATH.
# save_map() fires on mapping_complete and wrote the robot's own PARTIAL
# occupancy grid straight over assets/hospital_map.png - the ground-truth
# world. Every completed run therefore corrupted the environment for the
# next run: measured drivable sample points fell 1314 -> 402 after a few
# benchmark passes. The SLAM output now goes to a separate directory.
SLAM_MAP_SAVE_PATH: str = "output/slam_map.png"
SLAM_GRID_RESOLUTION: int = 10              # px per grid cell (matches A*)
SLAM_CAMERA_FOV_DEG: float = 60.0           # camera field of view
SLAM_CAMERA_RANGE_PX: int = 150             # max camera detection range
SLAM_WALL_FOLLOW_PWM: int = 120             # PWM during wall-following exploration
SLAM_LOG_ODDS_FREE: float = -0.4            # log-odds decrement for free cells
SLAM_LOG_ODDS_OCC: float = 0.85             # log-odds increment for occupied cells
SLAM_LOG_ODDS_PRIOR: float = 0.0            # initial log-odds (unknown)

# ════════════════════════════════════════════════
# MAP EDITOR COLORS (RGB for Pygame / PNG)
# ════════════════════════════════════════════════
COLOR_FREE: Tuple[int, int, int] = (255, 255, 255)
COLOR_WALL: Tuple[int, int, int] = (0, 0, 0)
COLOR_DOCK: Tuple[int, int, int] = (0, 0, 180)
COLOR_JUNCTION: Tuple[int, int, int] = (255, 255, 0)
COLOR_BUMP_ZONE: Tuple[int, int, int] = (255, 140, 0)
COLOR_NOGO: Tuple[int, int, int] = (180, 0, 0)
COLOR_START: Tuple[int, int, int] = (0, 180, 0)

# ════════════════════════════════════════════════
# ENUMS
# ════════════════════════════════════════════════

class CellType(enum.Enum):
    """Types of cells on the preloaded hospital map."""
    FREE = "free"
    WALL = "wall"
    DOCK = "dock"
    JUNCTION = "junction"
    BUMP_ZONE = "bump_zone"
    NOGO = "nogo"
    START = "start"


class DockState(enum.Enum):
    """Charging dock finite-state-machine states."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    ALIGNING = "aligning"
    SLOW_APPROACH = "slow_approach"
    CONTACT = "contact"
    CHARGING = "charging"
    CHARGED = "charged"
    UNDOCKING = "undocking"
    DOCK_FAULT = "dock_fault"


class VibrationLevel(enum.Enum):
    """Vibration severity classification."""
    SAFE = "safe"
    WARNING = "warning"
    DANGER = "danger"


class ObstacleAction(enum.Enum):
    """Camera obstacle response action."""
    NOMINAL = "nominal"
    SLOW = "slow"
    STOP = "stop"


class MotorDirection(enum.Enum):
    """L298N motor direction states."""
    FWD = 1
    REV = -1
    BRAKE = 0


class DriveMode(enum.Enum):
    """Autonomous or manual keyboard control."""
    AUTONOMOUS = "autonomous"
    MANUAL = "manual"


class SimMode(enum.Enum):
    """Simulation operating mode."""
    MAPPING = "mapping"          # SLAM builds the map
    NAVIGATION = "navigation"    # Uses preloaded map


class ObstacleClass(enum.Enum):
    """AI-classified indoor obstacle category for hospital environment."""
    PERSON = "person"
    CART = "cart"                          # wheelchair, trolley, wheeled object
    EQUIPMENT = "equipment"               # portable medical equipment
    FURNITURE = "furniture"               # chairs, couches, benches, tables
    MEDICAL_EQUIPMENT = "medical_equip"   # beds, gurneys, IV stands
    UNKNOWN = "unknown"


class JunctionAction(enum.Enum):
    """Q-Learning agent actions at junctions."""
    PROCEED = 0
    WAIT = 1
    SLOW = 2
    REROUTE = 3


# ════════════════════════════════════════════════
# DATACLASSES
# ════════════════════════════════════════════════

@dataclass
class VehicleState:
    """Current estimated state of the MediVan on the map."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0                      # radians
    speed_ms: float = 0.0
    odometry_confidence: float = 1.0


@dataclass
class IMUData:
    """Latest IMU reading (complementary-filtered)."""
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 9.81
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    vib_rms: float = 0.0
    vib_level: VibrationLevel = VibrationLevel.SAFE
    tilt_fault: bool = False
    slope_warning: bool = False


@dataclass
class MotorCommand:
    """PWM + direction command for L298N driver."""
    pwm_a: int = 0                          # left motor pair
    pwm_b: int = 0                          # right motor pair
    dir_a: MotorDirection = MotorDirection.BRAKE
    dir_b: MotorDirection = MotorDirection.BRAKE


@dataclass
class ObstacleResult:
    """Single detected dynamic obstacle from camera."""
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)   # x, y, w, h
    area: int = 0
    proximity: float = 0.0                  # 0-1, 1 = right in front
    action: ObstacleAction = ObstacleAction.NOMINAL
    classification: ObstacleClass = ObstacleClass.UNKNOWN
    confidence: float = 0.0                 # AI detection confidence


@dataclass
class DockResult:
    """ArUco dock marker detection result."""
    found: bool = False
    lateral_offset: float = 0.0
    distance_px: float = 0.0


@dataclass
class BumpState:
    """Perimeter bump switch states."""
    front: bool = False
    rear: bool = False
    left: bool = False
    right: bool = False

    @property
    def any_triggered(self) -> bool:
        """Return True if any switch is triggered."""
        return self.front or self.rear or self.left or self.right


@dataclass
class MapCell:
    """Metadata for a single map grid cell."""
    cell_type: CellType = CellType.FREE
    cost: int = COST_FREE
    x: int = 0
    y: int = 0
