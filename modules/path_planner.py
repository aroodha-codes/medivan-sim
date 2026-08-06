"""
path_planner.py -- A* path planning on the preloaded map grid.

Plans the shortest drivable path between two map positions using A*,
then follows it with a pure-pursuit controller.  Dynamic obstacles
from the camera are temporarily blocked in the search space but
NEVER written to the static map.

Junction decisions are made by a Q-Learning agent that learns
optimal behaviour (proceed/wait/slow/reroute) from experience.
Bump zones trigger speed reductions to protect cargo.
"""

from __future__ import annotations

import heapq
import math
import os
import sys
import time
from typing import Callable, List, Optional, Tuple

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import (
    CELL_SIZE_PX, VEHICLE_WIDTH_PX, PWM_DEADBAND,
    LOOKAHEAD_DIST_PX, PURSUIT_KP, BASE_PWM,
    JUNCTION_SLOW_PWM_FACTOR, BUMP_SLOW_PWM_FACTOR,
    JUNCTION_SLOW_DIST_PX, BUMP_SLOW_DIST_PX,
    JUNCTION_CLEAR_TIME_S, JUNCTION_RECHECK_TIME_S,
    REPLAN_DEVIATION_PX, COST_FREE,
    CellType, MotorCommand, MotorDirection, VehicleState, ObstacleResult,
    JunctionAction,
)
from modules.q_learning_agent import QLearningAgent


class PathPlanner:
    """A* path planner with pure-pursuit path following.

    The planner operates on the preloaded map grid — dynamic obstacles
    are injected as temporary blocked cells for each planning cycle
    and discarded afterwards.
    """

    def __init__(self) -> None:
        self.path: list[Tuple[int, int]] = []
        self.path_index: int = 0
        self.replan_count: int = 0
        self._dynamic_blocked: set[Tuple[int, int]] = set()

        # Q-Learning agent for junction decisions
        self.q_agent = QLearningAgent()

        # Junction behaviour state
        self._junction_waiting: bool = False
        self._junction_wait_start: float = 0.0
        self._junction_wait_frames: int = 0
        self._junction_action: Optional[JunctionAction] = None
        self._junction_reroute_requested: bool = False

    # ── A* search ───────────────────────────────

    def plan_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        get_cost_fn: Callable[[int, int], int],
        is_free_fn: Callable[[int, int], bool],
        is_near_wall_fn: Callable[[int, int, int], bool],
        map_width: int,
        map_height: int,
    ) -> list[Tuple[int, int]]:
        """Run A* from *start* to *goal* on the map grid.

        Parameters
        ----------
        start, goal : (x, y) pixel coordinates.
        get_cost_fn : map_loader.get_cost(x, y)
        is_free_fn  : map_loader.is_free(x, y)
        is_near_wall_fn : map_loader.is_near_wall(x, y, margin)
        map_width, map_height : map dimensions for bounds checking.

        Returns
        -------
        list of (x, y) waypoints from start to goal (inclusive).
        """
        step = CELL_SIZE_PX
        margin = VEHICLE_WIDTH_PX // 2 + 4

        # Quantise to grid cells
        sx, sy = start[0] // step, start[1] // step
        gx, gy = goal[0] // step, goal[1] // step
        max_cx = map_width // step
        max_cy = map_height // step

        if not (0 <= gx < max_cx and 0 <= gy < max_cy):
            self.path = []
            return self.path

        # A* open set: (f_score, counter, (cx, cy))
        counter = 0
        open_set: list[Tuple[float, int, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, counter, (sx, sy)))
        came_from: dict[Tuple[int, int], Optional[Tuple[int, int]]] = {(sx, sy): None}
        g_score: dict[Tuple[int, int], float] = {(sx, sy): 0.0}

        # MANHATTAN heuristic (replaces Euclidean).
        # Expansion is 4-connected, so no route can ever be shorter than
        # |dx| + |dy| steps. Euclidean (the straight line) is a strictly
        # looser under-estimate of that, which makes A* explore a wider
        # frontier for the identical optimal path. Manhattan is the tightest
        # admissible heuristic for a 4-connected uniform grid, so the result
        # is unchanged but fewer nodes are expanded.
        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return (abs(a[0] - b[0]) + abs(a[1] - b[1])) * COST_FREE

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                break

            # 4-CONNECTED expansion retained.
            # An 8-connected variant with an octile heuristic was implemented
            # and benchmarked: routes were only 2.7 % shorter (9.90 m vs
            # 10.18 m) but planning cost 322.77 ms mean / 936.62 ms p95
            # against 90.97 / 243.01 for 4-connected -- a 3.5x regression,
            # caused by the doubled branching factor plus four extra
            # is_free/is_near_wall calls per diagonal for corner-cut
            # rejection. On a Pi 4 that would stall the 30 FPS control loop,
            # so the change was reverted. See EVALUATION_REPORT.md.
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = current[0] + dx, current[1] + dy
                if not (0 <= nx < max_cx and 0 <= ny < max_cy):
                    continue

                # Pixel position of this cell centre
                px, py = nx * step + step // 2, ny * step + step // 2

                # Skip non-free or dynamically blocked
                if not is_free_fn(px, py):
                    continue
                if (nx, ny) in self._dynamic_blocked:
                    continue

                # Wall-clearance check — van has physical width
                if is_near_wall_fn(px, py, margin):
                    continue

                move_cost = get_cost_fn(px, py)
                tentative_g = g_score[current] + move_cost

                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + heuristic((nx, ny), (gx, gy))
                    counter += 1
                    heapq.heappush(open_set, (f, counter, (nx, ny)))
                    came_from[(nx, ny)] = current

        # Reconstruct path
        path_cells: list[Tuple[int, int]] = []
        node: Optional[Tuple[int, int]] = (gx, gy)
        if node not in came_from:
            self.path = []
            return self.path

        while node is not None:
            path_cells.append(node)
            node = came_from.get(node)
        path_cells.reverse()

        # Convert back to pixel coordinates
        self.path = [(c[0] * step + step // 2,
                       c[1] * step + step // 2) for c in path_cells]
        self.path_index = 0
        self.replan_count += 1
        return self.path

    # ── dynamic obstacle blocking ───────────────

    def set_dynamic_obstacles(
        self,
        obstacles: List[ObstacleResult],
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
    ) -> bool:
        """Convert camera obstacles to temporary blocked cells.

        Returns True if any obstacle blocks the current path
        (triggering a re-plan).
        """
        self._dynamic_blocked.clear()
        blocks_path = False

        for obs in obstacles:
            # Approximate map position from camera bbox + vehicle pose
            bx, by, bw, bh = obs.bbox
            cam_cx = bx + bw / 2.0
            cam_cy = by + bh / 2.0

            # Camera → map: rough forward projection
            forward_dist = (1.0 - obs.proximity) * 80  # px ahead on map
            cos_t = math.cos(vehicle_theta)
            sin_t = math.sin(vehicle_theta)
            map_x = int(vehicle_x + forward_dist * cos_t)
            map_y = int(vehicle_y + forward_dist * sin_t)

            # Block a small area around this position
            step = CELL_SIZE_PX
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    cell = (map_x // step + dx, map_y // step + dy)
                    self._dynamic_blocked.add(cell)

            # Check if any blocked cell is on the current path
            for wx, wy in self.path[self.path_index:]:
                cx, cy = wx // step, wy // step
                if (cx, cy) in self._dynamic_blocked:
                    blocks_path = True

        return blocks_path

    # ── path following (pure pursuit) ───────────

    def follow_path(
        self,
        vehicle_state: VehicleState,
        grid_fn: Optional[Callable[[int, int], CellType]] = None,
        obstacles: Optional[List[ObstacleResult]] = None,
        battery_pct: float = 100.0,
    ) -> MotorCommand:
        """Pure-pursuit controller with Q-Learning junction decisions.

        Parameters
        ----------
        vehicle_state : current vehicle state.
        grid_fn : optional callable(x,y) -> CellType for junction/bump detection.
        obstacles : camera obstacle list for junction cross-traffic check.
        battery_pct : current battery percentage (for Q-Learning state).

        Returns
        -------
        MotorCommand for the motor driver.
        """
        if not self.path or self.path_index >= len(self.path):
            return MotorCommand(0, 0, MotorDirection.BRAKE, MotorDirection.BRAKE)

        vx, vy, vtheta = vehicle_state.x, vehicle_state.y, vehicle_state.theta

        # Advance path index past visited waypoints
        while self.path_index < len(self.path) - 1:
            wx, wy = self.path[self.path_index]
            if math.sqrt((vx - wx) ** 2 + (vy - wy) ** 2) < LOOKAHEAD_DIST_PX:
                self.path_index += 1
            else:
                break

        # Find lookahead point
        la_idx = min(self.path_index + 1, len(self.path) - 1)
        la_x, la_y = self.path[la_idx]

        # Angle to lookahead
        angle_to_la = math.atan2(la_y - vy, la_x - vx) - vtheta
        # Normalise to [-π, π]
        angle_to_la = math.atan2(math.sin(angle_to_la), math.cos(angle_to_la))

        # PWM differential
        pwm_diff = PURSUIT_KP * angle_to_la * (255 / math.pi)
        base = BASE_PWM

        # -- Junction behaviour (Q-Learning) -----
        if grid_fn is not None:
            for check_idx in range(self.path_index,
                                   min(self.path_index + 3, len(self.path))):
                cx, cy = self.path[check_idx]
                cell = grid_fn(cx, cy)
                dist = math.sqrt((vx - cx) ** 2 + (vy - cy) ** 2)

                if cell == CellType.JUNCTION and dist < JUNCTION_SLOW_DIST_PX:
                    # Determine state for Q-Learning
                    has_obstacle = bool(obstacles and any(
                        o.proximity > 0.5 for o in obstacles))

                    # Ask Q-Learning agent for action
                    if self._junction_action is None:
                        self._junction_action = self.q_agent.choose_action(
                            junction_dist_px=dist,
                            obstacle_nearby=has_obstacle,
                            speed_ms=vehicle_state.speed_ms,
                            battery_pct=battery_pct,
                        )
                        self._junction_wait_frames = 0
                        self._junction_wait_start = time.time()

                    self._junction_wait_frames += 1

                    # Execute the chosen action
                    if self._junction_action == JunctionAction.WAIT:
                        # Full stop, wait
                        elapsed = time.time() - self._junction_wait_start
                        if has_obstacle or elapsed < JUNCTION_CLEAR_TIME_S:
                            return MotorCommand(0, 0, MotorDirection.BRAKE,
                                                MotorDirection.BRAKE)
                        else:
                            # Junction cleared -- give reward and reset
                            reward = self.q_agent.compute_reward(
                                self._junction_action, collision=False,
                                obstacle_present=has_obstacle,
                                time_spent_frames=self._junction_wait_frames,
                                safely_passed=True,
                            )
                            self.q_agent.learn(
                                reward, dist, has_obstacle,
                                vehicle_state.speed_ms, battery_pct, done=True,
                            )
                            self._junction_action = None

                    elif self._junction_action == JunctionAction.SLOW:
                        base = int(BASE_PWM * JUNCTION_SLOW_PWM_FACTOR)
                        # Learn after passing
                        if dist > JUNCTION_SLOW_DIST_PX:
                            reward = self.q_agent.compute_reward(
                                self._junction_action, collision=False,
                                obstacle_present=has_obstacle,
                                time_spent_frames=self._junction_wait_frames,
                                safely_passed=True,
                            )
                            self.q_agent.learn(
                                reward, dist, has_obstacle,
                                vehicle_state.speed_ms, battery_pct, done=True,
                            )
                            self._junction_action = None

                    elif self._junction_action == JunctionAction.REROUTE:
                        self._junction_reroute_requested = True
                        reward = self.q_agent.compute_reward(
                            self._junction_action, collision=False,
                            obstacle_present=has_obstacle,
                            time_spent_frames=self._junction_wait_frames,
                            safely_passed=True,
                        )
                        self.q_agent.learn(
                            reward, dist, has_obstacle,
                            vehicle_state.speed_ms, battery_pct, done=True,
                        )
                        self._junction_action = None

                    else:  # PROCEED
                        base = int(BASE_PWM * 0.7)  # slight caution
                        if dist > JUNCTION_SLOW_DIST_PX:
                            # Passed junction -- learn
                            collision = has_obstacle and dist < 5
                            reward = self.q_agent.compute_reward(
                                self._junction_action, collision=collision,
                                obstacle_present=has_obstacle,
                                time_spent_frames=self._junction_wait_frames,
                                safely_passed=not collision,
                            )
                            self.q_agent.learn(
                                reward, dist, has_obstacle,
                                vehicle_state.speed_ms, battery_pct, done=True,
                            )
                            self._junction_action = None

                    break

                elif cell == CellType.BUMP_ZONE and dist < BUMP_SLOW_DIST_PX:
                    base = int(BASE_PWM * BUMP_SLOW_PWM_FACTOR)
                    break
            else:
                # Not near any junction -- reset
                if self._junction_action is not None:
                    self._junction_action = None

        # Compute final PWMs
        pwm_left = int(base - pwm_diff)
        pwm_right = int(base + pwm_diff)
        pwm_left = max(PWM_DEADBAND, min(255, pwm_left))
        pwm_right = max(PWM_DEADBAND, min(255, pwm_right))

        return MotorCommand(pwm_left, pwm_right,
                            MotorDirection.FWD, MotorDirection.FWD)

    # ── deviation check ─────────────────────────

    def check_deviation(self, vehicle_state: VehicleState) -> bool:
        """Return True if the van has deviated too far from path."""
        if not self.path or self.path_index >= len(self.path):
            return False
        wx, wy = self.path[min(self.path_index, len(self.path) - 1)]
        dist = math.sqrt((vehicle_state.x - wx) ** 2 +
                          (vehicle_state.y - wy) ** 2)
        return dist > REPLAN_DEVIATION_PX

    @property
    def path_complete(self) -> bool:
        """True when the van has reached the end of the current path."""
        return len(self.path) > 0 and self.path_index >= len(self.path) - 1

    @property
    def remaining_waypoints(self) -> int:
        return max(0, len(self.path) - self.path_index)

    @property
    def reroute_requested(self) -> bool:
        """True when Q-Learning agent wants to avoid a junction."""
        if self._junction_reroute_requested:
            self._junction_reroute_requested = False
            return True
        return False

    def save_agent(self) -> None:
        """Save the Q-Learning agent's Q-table to disk."""
        self.q_agent.save()


# ── Standalone test ─────────────────────────────
if __name__ == "__main__":
    planner = PathPlanner()

    # Mock map functions
    def mock_cost(x, y): return COST_FREE
    def mock_free(x, y): return 50 < x < 750 and 50 < y < 550
    def mock_near_wall(x, y, m): return x < 60 or x > 740 or y < 60 or y > 540

    path = planner.plan_path(
        start=(125, 465), goal=(700, 295),
        get_cost_fn=mock_cost,
        is_free_fn=mock_free,
        is_near_wall_fn=mock_near_wall,
        map_width=800, map_height=600,
    )
    print(f"Path found: {len(path)} waypoints")
    if path:
        print(f"Start: {path[0]}, End: {path[-1]}")
