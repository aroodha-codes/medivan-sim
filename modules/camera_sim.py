"""
camera_sim.py -- Simulated forward-facing camera for dynamic obstacle detection.

The camera has ONE job: detect dynamic obstacles (people, carts) that
are NOT on the static map.  The map already encodes walls, rooms, and
permanent obstacles -- the camera only needs to find things that appear
or move at runtime.

Uses AI-powered detection (YOLOv8-Nano when ONNX model available,
heuristic classifier as fallback).  Also handles ArUco marker
detection for dock alignment when in DOCKING state.
"""

from __future__ import annotations

import math
import os
import random
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import (
    FRAME_W, FRAME_H, FPS, ARUCO_ID,
    OBS_ROI_TOP, OBS_ROI_BOTTOM,
    OBS_MIN_AREA_PX, OBS_SLOW_AREA_PX, OBS_STOP_AREA_PX,
    ObstacleAction, ObstacleResult, DockResult,
)
from modules.ai_obstacle_detector import AIObstacleDetector


# ── Dynamic obstacle sprite ────────────────────

class _DynObstacle:
    """A moving obstacle sprite (person / cart) in the camera frame."""

    def __init__(self) -> None:
        self.x: int = random.randint(50, FRAME_W - 50)
        self.y: int = random.randint(
            int(FRAME_H * OBS_ROI_TOP) + 20,
            int(FRAME_H * OBS_ROI_BOTTOM) - 20,
        )
        self.w: int = random.randint(40, 100)
        self.h: int = random.randint(60, 140)
        self.vx: int = random.choice([-3, -2, -1, 1, 2, 3])
        self.vy: int = random.choice([-1, 0, 0, 1])
        self.colour: Tuple[int, int, int] = (
            random.randint(60, 200),
            random.randint(60, 200),
            random.randint(60, 200),
        )
        self.ttl: int = random.randint(60, 300)  # frames to live

    def update(self) -> bool:
        """Move the sprite; return False when expired."""
        self.x += self.vx
        self.y += self.vy
        self.ttl -= 1
        return self.ttl > 0 and 0 <= self.x < FRAME_W

    def draw(self, frame: np.ndarray) -> None:
        cv2.rectangle(
            frame,
            (self.x - self.w // 2, self.y - self.h // 2),
            (self.x + self.w // 2, self.y + self.h // 2),
            self.colour, -1,
        )


class CameraSim:
    """Camera module with real webcam support.

    If a laptop/USB camera is detected, uses live video frames for
    AI obstacle detection. Otherwise falls back to synthetic corridor
    frames with spawned obstacle sprites.
    """

    def __init__(self) -> None:
        # AI obstacle detector (YOLO or heuristic fallback)
        self._ai_detector = AIObstacleDetector()

        # ArUco setup
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(
            self._aruco_dict, self._aruco_params
        )

        # Dynamic obstacle sprites (synthetic mode only)
        self._obstacles: list[_DynObstacle] = []
        self._frame_count: int = 0

        # Temporal blending buffers
        self._prev1: Optional[np.ndarray] = None
        self._prev2: Optional[np.ndarray] = None

        # -- Try to open real camera ---------------------
        self._cap: Optional[cv2.VideoCapture] = None
        self.use_real_camera: bool = False
        try:
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                cap.set(cv2.CAP_PROP_FPS, FPS)
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    self._cap = cap
                    self.use_real_camera = True
                    print("[Camera] REAL WEBCAM detected -- using live video")
                else:
                    cap.release()
            else:
                cap.release()
        except Exception:
            pass

        if not self.use_real_camera:
            print("[Camera] No webcam -- using synthetic corridor frames")

        # Corridor background (generated once, synthetic mode)
        self._corridor_bg = self._generate_corridor_background()

    # ── public API ──────────────────────────────

    def process_frame(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
        dock_mode: bool = False,
        dock_x: float = 0.0,
        dock_y: float = 0.0,
    ) -> Tuple[List[ObstacleResult], DockResult, np.ndarray]:
        """Generate and analyse one camera frame.

        Parameters
        ----------
        vehicle_x, vehicle_y, vehicle_theta : float
            Current van pose on the map (used for perspective hint).
        dock_mode : bool
            If True, draw and detect ArUco marker for dock alignment.
        dock_x, dock_y : float
            Map-pixel position of the dock (for ArUco placement).

        Returns
        -------
        obstacles : list[ObstacleResult]
        dock_result : DockResult
        annotated_frame : np.ndarray (BGR, 640×480)
        """
        self._frame_count += 1

        # 1. Build raw frame
        frame = self._build_frame(vehicle_x, vehicle_y, vehicle_theta,
                                  dock_mode, dock_x, dock_y)

        # 2. Temporal blend
        frame = self._temporal_blend(frame)

        # 3. Fluorescent flicker
        frame = self._flicker(frame)

        # 4. Detect dynamic obstacles (AI-powered)
        obstacles = self._ai_detector.detect(frame)

        # 5. Dock ArUco detection
        dock_result = DockResult()
        if dock_mode:
            dock_result = self._detect_dock_marker(frame)

        # 6. Annotate
        annotated = self._annotate(frame, obstacles, dock_result)

        return obstacles, dock_result, annotated

    # ── frame generation ────────────────────────

    def _generate_corridor_background(self) -> np.ndarray:
        """Create a static corridor-like background image."""
        bg = np.full((FRAME_H, FRAME_W, 3), (200, 195, 185), dtype=np.uint8)

        # Floor (grey-beige)
        cv2.rectangle(bg, (0, int(FRAME_H * 0.55)), (FRAME_W, FRAME_H),
                      (180, 175, 165), -1)

        # Ceiling
        cv2.rectangle(bg, (0, 0), (FRAME_W, int(FRAME_H * 0.15)),
                      (210, 210, 215), -1)

        # Left wall edge
        pts_left = np.array([
            [0, int(FRAME_H * 0.15)], [120, int(FRAME_H * 0.20)],
            [120, int(FRAME_H * 0.55)], [0, FRAME_H]
        ], np.int32)
        cv2.fillPoly(bg, [pts_left], (190, 185, 175))

        # Right wall edge
        pts_right = np.array([
            [FRAME_W, int(FRAME_H * 0.15)], [FRAME_W - 120, int(FRAME_H * 0.20)],
            [FRAME_W - 120, int(FRAME_H * 0.55)], [FRAME_W, FRAME_H]
        ], np.int32)
        cv2.fillPoly(bg, [pts_right], (190, 185, 175))

        # Fluorescent lights on ceiling
        for lx in range(160, FRAME_W - 100, 200):
            cv2.rectangle(bg, (lx, int(FRAME_H * 0.05)),
                          (lx + 80, int(FRAME_H * 0.08)),
                          (240, 240, 245), -1)

        # Floor tiles (subtle grid)
        for tx in range(0, FRAME_W, 80):
            cv2.line(bg, (tx, int(FRAME_H * 0.55)), (tx, FRAME_H),
                     (170, 165, 155), 1)
        for ty in range(int(FRAME_H * 0.55), FRAME_H, 40):
            cv2.line(bg, (0, ty), (FRAME_W, ty), (170, 165, 155), 1)

        return bg

    def _build_frame(self, vx: float, vy: float, vtheta: float,
                     dock_mode: bool, dx: float, dy: float) -> np.ndarray:
        """Compose one raw camera frame.

        Uses real webcam if available, otherwise synthetic corridor.
        """
        # -- Real camera mode: capture live frame --------
        if self.use_real_camera and self._cap is not None:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H))
                # Draw ArUco if dock mode
                if dock_mode:
                    marker_size = 60
                    marker_img = cv2.aruco.generateImageMarker(
                        self._aruco_dict, ARUCO_ID, marker_size
                    )
                    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
                    mx = FRAME_W // 2 - marker_size // 2
                    my = int(FRAME_H * 0.25)
                    y1, y2 = my, my + marker_size
                    x1, x2 = mx, mx + marker_size
                    if 0 <= y1 and y2 < FRAME_H and 0 <= x1 and x2 < FRAME_W:
                        frame[y1:y2, x1:x2] = marker_bgr
                return frame

        # -- Synthetic mode: corridor + sprites ----------
        frame = self._corridor_bg.copy()

        # Spawn / update dynamic obstacle sprites
        if random.random() < 0.02 and len(self._obstacles) < 4:
            self._obstacles.append(_DynObstacle())

        self._obstacles = [o for o in self._obstacles if o.update()]
        for obs in self._obstacles:
            obs.draw(frame)

        # Draw ArUco marker if in dock mode and within range
        if dock_mode:
            marker_size = 60
            marker_img = cv2.aruco.generateImageMarker(
                self._aruco_dict, ARUCO_ID, marker_size
            )
            marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
            mx = FRAME_W // 2 - marker_size // 2 + random.randint(-8, 8)
            my = int(FRAME_H * 0.25)
            y1, y2 = my, my + marker_size
            x1, x2 = mx, mx + marker_size
            if 0 <= y1 and y2 < FRAME_H and 0 <= x1 and x2 < FRAME_W:
                frame[y1:y2, x1:x2] = marker_bgr

        return frame

    def release(self) -> None:
        """Release the webcam resource."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            print("[Camera] Webcam released")

    def _temporal_blend(self, frame: np.ndarray) -> np.ndarray:
        """Apply temporal smoothing: 0.5×curr + 0.3×prev1 + 0.2×prev2."""
        blended = frame.astype(np.float32) * 0.5
        if self._prev1 is not None:
            blended += self._prev1.astype(np.float32) * 0.3
        else:
            blended += frame.astype(np.float32) * 0.3
        if self._prev2 is not None:
            blended += self._prev2.astype(np.float32) * 0.2
        else:
            blended += frame.astype(np.float32) * 0.2

        self._prev2 = self._prev1
        self._prev1 = frame.copy()
        return blended.astype(np.uint8)

    def _flicker(self, frame: np.ndarray) -> np.ndarray:
        """Simulate fluorescent light flicker ±8% every 30 frames."""
        if self._frame_count % 30 < 3:
            factor = 1.0 + random.uniform(-0.08, 0.08)
            frame = np.clip(frame.astype(np.float32) * factor, 0, 255).astype(np.uint8)
        return frame

    # ── obstacle detection ──────────────────────

    def _detect_obstacles(self, frame: np.ndarray) -> List[ObstacleResult]:
        """Legacy MOG2 detection -- now delegated to AIObstacleDetector."""
        return self._ai_detector.detect(frame)

    # ── dock marker detection ───────────────────

    def _detect_dock_marker(self, frame: np.ndarray) -> DockResult:
        """Detect ArUco marker ID=0 and compute lateral offset."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)

        if ids is None:
            return DockResult(found=False)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == ARUCO_ID:
                c = corners[i][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                lateral = cx - FRAME_W / 2.0
                # Approximate distance from marker size
                marker_w = float(np.linalg.norm(c[0] - c[1]))
                return DockResult(
                    found=True,
                    lateral_offset=lateral,
                    distance_px=marker_w,
                )

        return DockResult(found=False)

    # ── annotation ──────────────────────────────

    def _annotate(
        self,
        frame: np.ndarray,
        obstacles: List[ObstacleResult],
        dock: DockResult,
    ) -> np.ndarray:
        """Draw bounding boxes and labels on the frame."""
        out = frame.copy()

        # ROI lines
        roi_top = int(FRAME_H * OBS_ROI_TOP)
        roi_bot = int(FRAME_H * OBS_ROI_BOTTOM)
        cv2.line(out, (0, roi_top), (FRAME_W, roi_top), (255, 255, 0), 1)
        cv2.line(out, (0, roi_bot), (FRAME_W, roi_bot), (255, 255, 0), 1)

        for obs in obstacles:
            x, y, w, h = obs.bbox
            colour = (0, 0, 255) if obs.action == ObstacleAction.STOP else \
                     (0, 165, 255) if obs.action == ObstacleAction.SLOW else \
                     (0, 255, 0)
            cv2.rectangle(out, (x, y), (x + w, y + h), colour, 2)
            cls_name = obs.classification.value.upper()
            conf_str = f"{obs.confidence:.0%}" if obs.confidence > 0 else ""
            label = f"{cls_name} {obs.action.value} {conf_str}"
            cv2.putText(out, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, colour, 1)

        if dock.found:
            cx = int(FRAME_W / 2 + dock.lateral_offset)
            cv2.drawMarker(out, (cx, FRAME_H // 3),
                           (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(out, f"DOCK off={dock.lateral_offset:.0f}px",
                        (cx - 60, FRAME_H // 3 - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        # Label with detection mode
        mode_label = f"CAMERA - AI DETECTION [{self._ai_detector.mode}]"
        cv2.putText(out, mode_label, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

        return out


# ── Standalone test ─────────────────────────────
if __name__ == "__main__":
    cam = CameraSim()
    for i in range(90):
        obs, dock, frame = cam.process_frame(100, 100, 0.0,
                                              dock_mode=(i > 60))
        if obs:
            print(f"Frame {i}: {len(obs)} obstacles — "
                  f"{[o.action.value for o in obs]}")
        if dock.found:
            print(f"Frame {i}: Dock found, offset={dock.lateral_offset:.1f}")
        cv2.imshow("CameraSim Test", frame)
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
