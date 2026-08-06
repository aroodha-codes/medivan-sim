"""
main.py -- MediVan Indoor Navigation Simulator entry point.

Two-phase autonomous operation:
  Phase 1 - MAPPING:  SLAM explores the environment using camera +
            encoder, builds an occupancy grid map from scratch.
  Phase 2 - NAVIGATION:  A* + Q-Learning + YOLO navigates using
            the SLAM-built map.

The robot has NO prior knowledge of the environment. It always
starts by mapping, then auto-switches to navigation once coverage
reaches the threshold (85%).

All modules wrapped in try/except -- single failure degrades gracefully.
Q or window close -> stop motors, flush logs, plot trajectory.
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Optional

import cv2
import numpy as np
import pygame

# ── Ensure project root is on path ──────────────
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from config import (
    MAP_PATH, FPS, MAP_SCALE_M_PER_PX, LOW_BAT_THRESHOLD,
    DriveMode, DockState, CellType, SimMode, HARDWARE_MODE,
    VehicleState, IMUData, MotorCommand, BumpState,
    ObstacleAction, VibrationLevel,
)
from modules.map_loader import MapLoader
from modules.encoder_sim import EncoderSim
from modules.imu_sim import IMUSim
from modules.camera_sim import CameraSim
from modules.motor_driver_sim import MotorDriverSim
from modules.bump_switch_sim import BumpSwitchSim
from modules.localizer import Localizer
from modules.path_planner import PathPlanner
from modules.charging_dock_sim import ChargingDockSim
from modules.audio_sim import AudioSim, AudioEvent
from modules.hud import HUD
from modules.data_logger import DataLogger
from modules.slam_engine import SLAMEngine
from modules.delivery_queue import DeliveryQueue


def _ensure_map_exists(map_path: str) -> str:
    """Auto-generate the default hospital map if it doesn't exist."""
    full_path = os.path.join(PROJECT_ROOT, map_path)
    if not os.path.exists(full_path):
        from map_editor import generate_default_map
        generate_default_map(full_path)
    return full_path


def main() -> None:
    """Run the MediVan simulation."""

    # ════════════════════════════════════════════
    # INITIALISATION
    # ════════════════════════════════════════════
    # FIX (MT3608 review): SDL reads SDL_VIDEODRIVER when the video subsystem
    # is initialised, which happens inside pygame.init().  The old code set it
    # ~55 lines later, so a headless Pi (SSH, no DISPLAY) crashed on
    # pygame.display.set_mode() before the variable ever took effect.
    # It must be set BEFORE pygame.init().
    if not os.environ.get("DISPLAY") and not os.environ.get("SDL_VIDEODRIVER"):
        print("[main] No DISPLAY detected — using headless SDL dummy driver.")
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    pygame.init()
    clock = pygame.time.Clock()

    # -- Ground-truth world (for simulation physics only)
    # The robot does NOT see this map -- it builds its own via SLAM
    map_path = _ensure_map_exists(MAP_PATH)
    map_loader = MapLoader()
    map_loader.load_map(map_path)
    print(f"[main] World loaded (physics only): {map_loader.width}x{map_loader.height}")

    # -- SLAM engine: robot always starts by mapping
    sim_mode = SimMode.MAPPING
    slam: Optional[SLAMEngine] = SLAMEngine(ground_truth_free_fn=map_loader.is_free)
    print("[main] Phase 1: SLAM MAPPING -- robot exploring environment...")

    # ── Modules (Factory Pattern) ───────────────
    if HARDWARE_MODE:
        print("[main] HARDWARE_MODE=True. Loading physical RPi drivers...")
        from hardware.motor_driver_hw import MotorDriverHW
        from hardware.camera_hw import CameraHW
        from hardware.imu_hw import IMUHW
        from hardware.encoder_hw import EncoderHW
        
        encoder = EncoderHW()
        imu = IMUHW()
        camera = CameraHW()
        motor = MotorDriverHW()
        bump = BumpSwitchSim()  # Fallback to sim for now if no physical bump switch
    else:
        print("[main] HARDWARE_MODE=False. Loading software simulators...")
        encoder = EncoderSim()
        imu = IMUSim()
        camera = CameraSim()
        motor = MotorDriverSim()
        bump = BumpSwitchSim()

    # Shared generic modules
    localizer = Localizer()
    planner = PathPlanner()
    dock = ChargingDockSim(dock_position=map_loader.dock_position)
    audio = AudioSim()
    hud = HUD()
    logger = DataLogger(log_dir=PROJECT_ROOT)
    delivery = DeliveryQueue()

    # -- Initial state ───────────────────────────
    start_pos = map_loader.start_position or (125, 465)
    localizer.initialize(start_pos, start_theta=-math.pi / 2)
    motor.set_position(start_pos[0], start_pos[1], -math.pi / 2)
    slam.initialize(start_pos[0], start_pos[1], -math.pi / 2)

    # Navigation goal (used after SLAM completes)
    goal = map_loader.dock_position or (700, 295)

    # -- Pygame display ──────────────────────────
    # (SDL_VIDEODRIVER is now selected before pygame.init() — see above.)
    screen = pygame.display.set_mode((640, 520))
    pygame.display.set_caption("MediVan -- SLAM MAPPING")

    # ── Start background threads ────────────────
    imu.start()
    logger.open()

    # ── Camera greyscale buffers ────────────────
    prev_gray: Optional[np.ndarray] = None
    curr_gray: Optional[np.ndarray] = None

    # ── Run state ───────────────────────────────
    running = True
    dt = 1.0 / FPS
    frame_count = 0

    print("[main] Simulation running. Press Q to quit, TAB for manual mode.")

    # ════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════
    while running:
        frame_count += 1

        # ── Pygame events ───────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_TAB:
                    motor.mode = (DriveMode.MANUAL
                                  if motor.mode == DriveMode.AUTONOMOUS
                                  else DriveMode.AUTONOMOUS)
                    print(f"[main] Mode -> {motor.mode.value}")
                elif event.key == pygame.K_SPACE:
                    motor.emergency_stop()
                    audio.play(AudioEvent.EMERGENCY_STOP)
                    hud.set_alert("EMERGENCY STOP", 3.0)
                elif event.key == pygame.K_c:
                    dock.force_return_to_dock()
                    print("[main] Force return to dock")
                elif event.key == pygame.K_e:
                    motor.release_emergency()
                elif event.key == pygame.K_g:
                    pos = delivery.add_random_goal(
                        map_loader.width, map_loader.height,
                        map_loader.is_free)
                    goal = delivery.current_goal or goal
                    hud.set_alert(f"DELIVERY ADDED: {delivery.current_label}", 2.0)
                    audio.play(AudioEvent.JUNCTION_APPROACH)

        keys = pygame.key.get_pressed()

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 1. MAP LOADER — serve static map
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        display_map = map_loader.get_display_map()

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 2. IMU — get latest from background thread
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            imu_data = imu.get_latest()
        except Exception:
            imu_data = IMUData()

        # Tilt safety
        if imu_data.tilt_fault:
            motor.emergency_stop()
            hud.set_alert("TILT FAULT — MOTORS STOPPED", 2.0)
            audio.play(AudioEvent.EMERGENCY_STOP)

        # Vibration danger → cap PWM
        vib_pwm_cap = 255
        if imu_data.vib_level == VibrationLevel.DANGER:
            vib_pwm_cap = int(255 * 0.60)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 3. CAMERA — dynamic obstacle detection
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            obstacles, dock_result, cam_frame = camera.process_frame(
                vehicle_x=motor.x,
                vehicle_y=motor.y,
                vehicle_theta=motor.theta,
                dock_mode=dock.dock_active_for_camera,
                dock_x=float(map_loader.dock_position[0]) if map_loader.dock_position else 0,
                dock_y=float(map_loader.dock_position[1]) if map_loader.dock_position else 0,
            )
        except Exception:
            obstacles, dock_result, cam_frame = [], None, None
            from config import DockResult
            dock_result = DockResult()

        # Update greyscale buffers for optical flow
        if cam_frame is not None:
            curr_gray = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2GRAY)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 4. LOCALIZER — position estimation
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            enc_reading = encoder.update(
                motor.pwm_a, motor.pwm_b,
                motor.dir_a, motor.dir_b,
                dt=dt, theta=motor.theta,
            )
            vehicle_state = localizer.update(
                enc_dx=enc_reading.dx_px,
                enc_dy=enc_reading.dy_px,
                enc_dtheta=enc_reading.dtheta,
                prev_gray=prev_gray,
                curr_gray=curr_gray,
                is_free_fn=map_loader.is_free,
                junctions=map_loader.junctions,
            )
        except Exception:
            vehicle_state = VehicleState(x=motor.x, y=motor.y, theta=motor.theta)

        # Sync motor position from localizer (authoritative)
        motor.x = vehicle_state.x
        motor.y = vehicle_state.y
        motor.theta = vehicle_state.theta

        prev_gray = curr_gray

        # Junction snap → reset IMU yaw
        if localizer.junction_snap_occurred:
            snapped = round(math.degrees(vehicle_state.theta) / 90) * 90
            imu.snap_yaw(math.radians(snapped))
            audio.play(AudioEvent.JUNCTION_APPROACH)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 4b. SLAM UPDATE (mapping mode only)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if slam is not None and cam_frame is not None:
            try:
                # Reuse enc_reading from step 4 — no second encoder.update()
                slam.update(
                    cam_frame,
                    enc_dx=enc_reading.dx_px,
                    enc_dy=enc_reading.dy_px,
                    enc_dtheta=enc_reading.dtheta,
                    robot_x=motor.x, robot_y=motor.y, robot_theta=motor.theta,
                    ai_results=obstacles,
                )
                if slam.mapping_complete:
                    print("[main] SLAM complete! Phase 2: NAVIGATION")
                    sim_mode = SimMode.NAVIGATION
                    pygame.display.set_caption("MediVan -- AI Navigation")
                    # Reload the SLAM-built map for A* navigation
                    map_loader.load_map(map_path)
                    # Plan first path using the SLAM-built map
                    planner.plan_path(
                        start=(int(vehicle_state.x), int(vehicle_state.y)),
                        goal=goal,
                        get_cost_fn=map_loader.get_cost,
                        is_free_fn=map_loader.is_free,
                        is_near_wall_fn=map_loader.is_near_wall,
                        map_width=map_loader.width,
                        map_height=map_loader.height,
                    )
                    print(f"[main] First path: {len(planner.path)} waypoints")
                    slam = None  # free memory
            except Exception as e:
                print(f"[main] SLAM error: {e}")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 5. PATH PLANNER -- follow path / re-plan
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        motor_cmd = MotorCommand()

        try:
            # Dynamic obstacle blocking
            needs_replan = planner.set_dynamic_obstacles(
                obstacles, motor.x, motor.y, motor.theta)

            # Deviation check
            if planner.check_deviation(vehicle_state):
                needs_replan = True

            # Delivery queue: advance to next goal on path completion
            if planner.path_complete and not delivery.is_empty:
                delivery.mark_complete()
                if not delivery.is_empty:
                    goal = delivery.current_goal
                    needs_replan = True
                    hud.set_alert(f"NEXT: {delivery.current_label}", 2.0)
                else:
                    goal = map_loader.dock_position or (700, 295)
                    needs_replan = True
                    hud.set_alert("ALL DELIVERED — RETURNING TO DOCK", 2.0)

            # Dock wants a path
            if dock.wants_dock_path and map_loader.dock_position:
                goal = map_loader.dock_position
                needs_replan = True

            # Re-plan if needed
            if needs_replan and goal:
                planner.plan_path(
                    start=(int(vehicle_state.x), int(vehicle_state.y)),
                    goal=goal,
                    get_cost_fn=map_loader.get_cost,
                    is_free_fn=map_loader.is_free,
                    is_near_wall_fn=map_loader.is_near_wall,
                    map_width=map_loader.width,
                    map_height=map_loader.height,
                )

            # Grid cell type lookup
            def grid_fn(x: int, y: int) -> CellType:
                if 0 <= y < map_loader.height and 0 <= x < map_loader.width:
                    return map_loader.grid[y][x]
                return CellType.WALL

            # Manual mode override
            manual_cmd = motor.handle_keyboard(keys)
            if manual_cmd is not None:
                motor_cmd = manual_cmd
            elif sim_mode == SimMode.MAPPING and slam is not None:
                # SLAM exploration: wall-following
                motor_cmd = slam.get_explore_command(
                    motor.x, motor.y, motor.theta, map_loader.is_free)
            elif motor.mode == DriveMode.AUTONOMOUS and not motor.emergency_stopped:
                motor_cmd = planner.follow_path(
                    vehicle_state, grid_fn=grid_fn, obstacles=obstacles,
                    battery_pct=dock.battery_pct)

                # Obstacle action override
                worst_action = ObstacleAction.NOMINAL
                for obs in obstacles:
                    if obs.action.value == "stop":
                        worst_action = ObstacleAction.STOP
                        break
                    elif obs.action.value == "slow":
                        worst_action = ObstacleAction.SLOW

                if worst_action == ObstacleAction.STOP:
                    from config import MotorDirection
                    motor_cmd = MotorCommand(0, 0,
                                             MotorDirection.BRAKE, MotorDirection.BRAKE)
                    audio.play(AudioEvent.OBSTACLE_WARNING)
                elif worst_action == ObstacleAction.SLOW:
                    motor_cmd.pwm_a = int(motor_cmd.pwm_a * 0.4)
                    motor_cmd.pwm_b = int(motor_cmd.pwm_b * 0.4)

        except Exception as e:
            print(f"[main] Planner error: {e}")

        # Vibration cap
        motor_cmd.pwm_a = min(motor_cmd.pwm_a, vib_pwm_cap)
        motor_cmd.pwm_b = min(motor_cmd.pwm_b, vib_pwm_cap)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 6. MOTOR DRIVER — set PWM + wall collision
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            motor.set_pwm(motor_cmd)
            motor.update(dt, map_loader.is_free)

            if motor.wall_contact:
                hud.set_alert("WALL CONTACT", 1.5)
                audio.play(AudioEvent.BUMP_CONTACT)
        except Exception as e:
            print(f"[main] Motor error: {e}")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 7. BUMP SWITCHES — safety override
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            bump_state = bump.check(motor.x, motor.y, motor.theta,
                                     map_loader.is_free)
            if bump.is_reversing:
                rev_cmd = bump.get_reverse_command()
                motor.set_pwm(rev_cmd)
                audio.play(AudioEvent.BUMP_CONTACT)
        except Exception:
            bump_state = BumpState()

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 8. CHARGING DOCK — FSM update
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            dock_cmd = dock.update(
                vehicle_state=vehicle_state,
                dock_result=dock_result,
                motors_active=(motor.forward_v != 0),
                dt=dt,
            )
            if dock_cmd is not None:
                motor.set_pwm(dock_cmd)

            # Battery alerts
            if dock.battery_pct < LOW_BAT_THRESHOLD:
                audio.play(AudioEvent.LOW_BATTERY)
                hud.set_alert(f"LOW BATTERY {dock.battery_pct:.0f}% - RETURNING TO DOCK", 2.0)
            if dock.state == DockState.CHARGED:
                audio.play(AudioEvent.DOCK_COMPLETE)
        except Exception as e:
            print(f"[main] Dock error: {e}")

        # IMU bump injection at bump zones
        try:
            for bz in map_loader.bump_zones:
                dist = math.sqrt((motor.x - bz[0])**2 + (motor.y - bz[1])**2)
                if dist < 15:
                    imu.inject_bump(2.0)
                    break
        except Exception:
            pass

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 9. AUDIO — already played inline above
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 10. HUD — render composite display
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            # Use SLAM display in mapping mode
            if slam is not None:
                display_map = slam.get_slam_display()

            hud_frame = hud.render(
                camera_frame=cam_frame,
                display_map=display_map,
                vehicle_state=vehicle_state,
                imu_data=imu_data,
                motor_cmd=motor_cmd,
                bump_state=bump_state,
                dock_state=dock.state,
                battery_pct=dock.battery_pct,
                mode=motor.mode,
                path=planner.path,
                wall_flash=motor.is_wall_flash_active,
                delivery_status=f"SEMANTIC MAPPING | Objects found: {len(slam.semantic_landmarks)}" if sim_mode == SimMode.MAPPING and slam else delivery.status_text,
            )

            # Convert BGR → RGB for Pygame
            hud_rgb = cv2.cvtColor(hud_frame, cv2.COLOR_BGR2RGB)
            surf = pygame.surfarray.make_surface(
                np.transpose(hud_rgb, (1, 0, 2))
            )
            screen.blit(surf, (0, 0))
        except Exception as e:
            print(f"[main] HUD error: {e}")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 11. DATA LOGGER — log frame
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        try:
            worst_obs_action = "nominal"
            for obs in obstacles:
                if obs.action == ObstacleAction.STOP:
                    worst_obs_action = "stop"
                    break
                elif obs.action == ObstacleAction.SLOW:
                    worst_obs_action = "slow"

            logger.log(
                vehicle_state=vehicle_state,
                imu_data=imu_data,
                motor_cmd=motor_cmd,
                bump_state=bump_state,
                dock_state=dock.state,
                battery_pct=dock.battery_pct,
                mode=motor.mode,
                obstacle_count=len(obstacles),
                obstacle_action=worst_obs_action,
                contact_quality=dock.contact_quality,
                junction_snap=localizer.junction_snap_occurred,
                replan_count=planner.replan_count,
                audio_event=audio.current_event.value if audio.current_event else "",
            )
            audio.clear_event()
        except Exception as e:
            print(f"[main] Logger error: {e}")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 12. DISPLAY FLIP + CLOCK TICK
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        pygame.display.flip()
        clock.tick(FPS)

    # ════════════════════════════════════════════
    # SHUTDOWN
    # ════════════════════════════════════════════
    print("\n[main] Shutting down...")
    motor.emergency_stop()
    
    if HARDWARE_MODE:
        motor.cleanup()
        imu.cleanup()
        camera.cleanup()
    else:
        imu.stop()
        camera.release()

    planner.save_agent()
    print(f"[main] Q-Learning stats: {planner.q_agent.get_stats()}")
    print(f"[main] Delivery stats: {delivery.get_summary()}")
    logger.close()
    logger.print_summary(MAP_SCALE_M_PER_PX)
    logger.plot_trajectory(MAP_PATH)
    pygame.quit()
    print("[main] Done.")


if __name__ == "__main__":
    main()
