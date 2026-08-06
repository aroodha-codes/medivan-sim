"""
test_modules.py — Comprehensive pytest tests for all MediVan modules.

Covers imports, encoder, map loader, path planner, motor driver,
bump switches, charging dock, localizer, HUD, and data logger.
"""
import math, os, sys, json, tempfile
import numpy as np, cv2, pytest

sys.path.insert(0, os.path.dirname(__file__))

from config import (
    MotorDirection, MotorCommand, VehicleState, IMUData,
    BumpState, DockState, DriveMode, CellType, COST_FREE,
)


class TestImports:
    """Every module should import without error."""

    def test_config(self):
        import config

    def test_map_loader(self):
        from modules.map_loader import MapLoader

    def test_encoder_sim(self):
        from modules.encoder_sim import EncoderSim

    def test_imu_sim(self):
        from modules.imu_sim import IMUSim

    def test_camera_sim(self):
        from modules.camera_sim import CameraSim

    def test_motor_driver_sim(self):
        from modules.motor_driver_sim import MotorDriverSim

    def test_bump_switch_sim(self):
        from modules.bump_switch_sim import BumpSwitchSim

    def test_localizer(self):
        from modules.localizer import Localizer

    def test_path_planner(self):
        from modules.path_planner import PathPlanner

    def test_q_learning_agent(self):
        from modules.q_learning_agent import QLearningAgent

    def test_ai_obstacle_detector(self):
        from modules.ai_obstacle_detector import AIObstacleDetector

    def test_slam_engine(self):
        from modules.slam_engine import SLAMEngine

    def test_charging_dock_sim(self):
        from modules.charging_dock_sim import ChargingDockSim

    def test_audio_sim(self):
        from modules.audio_sim import AudioSim

    def test_hud(self):
        from modules.hud import HUD

    def test_data_logger(self):
        from modules.data_logger import DataLogger

    def test_delivery_queue(self):
        from modules.delivery_queue import DeliveryQueue


class TestEncoder:
    """Encoder simulator: PWM -> displacement."""

    def test_forward_gives_positive_displacement(self, encoder):
        r = encoder.update(160, 160, MotorDirection.FWD, MotorDirection.FWD,
                           dt=1/30, theta=0.0)
        # Moving forward at theta=0 should give positive dx
        assert abs(r.dx_px) > 0 or abs(r.dy_px) > 0

    def test_brake_gives_zero(self, encoder):
        r = encoder.update(0, 0, MotorDirection.BRAKE, MotorDirection.BRAKE,
                           dt=1/30, theta=0.0)
        assert abs(r.dx_px) < 2  # small noise tolerance
        assert abs(r.dy_px) < 2

    def test_turning_gives_dtheta(self, encoder):
        r = encoder.update(200, 100, MotorDirection.FWD, MotorDirection.FWD,
                           dt=1/30, theta=0.0)
        # Different PWMs should produce rotation
        assert r.dtheta != 0.0

    def test_cumulative_distance(self, encoder):
        for _ in range(10):
            encoder.update(160, 160, MotorDirection.FWD, MotorDirection.FWD,
                           dt=1/30, theta=0.0)
        assert encoder.total_distance_m > 0


class TestMapLoader:
    """Map loader: parse map, extract landmarks."""

    def test_map_dimensions(self, map_loader):
        assert map_loader.width > 0
        assert map_loader.height > 0

    def test_start_position(self, map_loader):
        assert map_loader.start_position is not None
        sx, sy = map_loader.start_position
        assert 0 <= sx < map_loader.width
        assert 0 <= sy < map_loader.height

    def test_dock_position(self, map_loader):
        assert map_loader.dock_position is not None

    def test_is_free_corridor(self, map_loader):
        sx, sy = map_loader.start_position
        assert map_loader.is_free(sx, sy) is True

    def test_wall_not_free(self, map_loader):
        # (0,0) should be wall in default map
        assert map_loader.is_free(0, 0) is False

    def test_get_cost_returns_int(self, map_loader):
        sx, sy = map_loader.start_position
        cost = map_loader.get_cost(sx, sy)
        assert isinstance(cost, (int, float))

    def test_junctions_detected(self, map_loader):
        assert len(map_loader.junctions) > 0


class TestPathPlanner:
    """A* path planner + pure pursuit."""

    def test_finds_valid_path(self, planner, map_loader):
        path = planner.plan_path(
            start=map_loader.start_position,
            goal=map_loader.dock_position,
            get_cost_fn=map_loader.get_cost,
            is_free_fn=map_loader.is_free,
            is_near_wall_fn=map_loader.is_near_wall,
            map_width=map_loader.width,
            map_height=map_loader.height,
        )
        assert len(path) > 0

    def test_path_starts_near_start(self, planner, map_loader):
        path = planner.plan_path(
            start=map_loader.start_position,
            goal=map_loader.dock_position,
            get_cost_fn=map_loader.get_cost,
            is_free_fn=map_loader.is_free,
            is_near_wall_fn=map_loader.is_near_wall,
            map_width=map_loader.width,
            map_height=map_loader.height,
        )
        sx, sy = map_loader.start_position
        px, py = path[0]
        assert abs(px - sx) < 20 and abs(py - sy) < 20

    def test_path_no_walls(self, planner, map_loader):
        path = planner.plan_path(
            start=map_loader.start_position,
            goal=map_loader.dock_position,
            get_cost_fn=map_loader.get_cost,
            is_free_fn=map_loader.is_free,
            is_near_wall_fn=map_loader.is_near_wall,
            map_width=map_loader.width,
            map_height=map_loader.height,
        )
        for wx, wy in path:
            assert map_loader.is_free(wx, wy), f"Wall at ({wx},{wy})"

    def test_replan_count_increments(self, planner, map_loader):
        assert planner.replan_count == 0
        planner.plan_path(
            start=map_loader.start_position,
            goal=map_loader.dock_position,
            get_cost_fn=map_loader.get_cost,
            is_free_fn=map_loader.is_free,
            is_near_wall_fn=map_loader.is_near_wall,
            map_width=map_loader.width,
            map_height=map_loader.height,
        )
        assert planner.replan_count == 1

    def test_follow_path_empty(self, planner):
        cmd = planner.follow_path(VehicleState(x=100, y=200, theta=0))
        assert isinstance(cmd, MotorCommand)
        assert cmd.dir_a == MotorDirection.BRAKE


class TestHUD:
    """HUD compositor: should produce valid frames."""

    def test_render_shape(self):
        from modules.hud import HUD, HUD_W, HUD_H
        hud = HUD()
        frame = hud.render(
            camera_frame=None, display_map=None,
            vehicle_state=VehicleState(x=200, y=300, theta=0.3),
            imu_data=IMUData(), motor_cmd=MotorCommand(),
            bump_state=BumpState(), dock_state=DockState.IDLE,
            battery_pct=75.0, mode=DriveMode.AUTONOMOUS,
        )
        assert frame.shape == (HUD_H, HUD_W, 3)

    def test_alert_sets(self):
        from modules.hud import HUD
        hud = HUD()
        hud.set_alert("TEST", 2.0)
        assert hud._alert_text == "TEST"


class TestDataLogger:
    """Data logger: JSONL output."""

    def test_write_and_read(self):
        from modules.data_logger import DataLogger
        with tempfile.TemporaryDirectory() as td:
            logger = DataLogger(log_dir=td)
            logger.open()
            vs = VehicleState(x=100, y=200)
            logger.log(vs, IMUData(), MotorCommand(), BumpState(),
                       DockState.IDLE, 95.0, DriveMode.AUTONOMOUS)
            logger.log(vs, IMUData(), MotorCommand(), BumpState(),
                       DockState.IDLE, 95.0, DriveMode.AUTONOMOUS)
            logger.close()
            log_path = os.path.join(td, "medivan_log.jsonl")
            assert os.path.exists(log_path)
            with open(log_path) as f:
                lines = f.readlines()
            assert len(lines) >= 1
            record = json.loads(lines[0])
            assert "map_x" in record


class TestDeliveryQueue:
    """Multi-goal delivery queue."""

    def test_add_and_get(self):
        from modules.delivery_queue import DeliveryQueue
        dq = DeliveryQueue()
        dq.add_goal((300, 200), "Ward A")
        assert dq.pending_count == 1
        goal = dq.current_goal
        assert goal == (300, 200)

    def test_complete_advances(self):
        from modules.delivery_queue import DeliveryQueue
        dq = DeliveryQueue()
        dq.add_goal((300, 200), "Ward A")
        dq.add_goal((500, 100), "Ward B")
        assert dq.current_goal == (300, 200)
        dq.mark_complete()
        assert dq.current_goal == (500, 100)

    def test_empty_queue(self):
        from modules.delivery_queue import DeliveryQueue
        dq = DeliveryQueue()
        assert dq.current_goal is None
        assert dq.is_empty is True
