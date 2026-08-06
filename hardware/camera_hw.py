"""
camera_hw.py -- Physical Pi Camera Module V2 Interface.

Captures frames using cv2.VideoCapture(0) and processes them
through the AI detector.
"""

from __future__ import annotations

import os
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import FRAME_W, FRAME_H
from modules.ai_obstacle_detector import AIObstacleDetector, ObstacleResult


class CameraHW:
    """Interfaces with the physical Pi Camera Module V2."""

    def __init__(self) -> None:
        self.detector = AIObstacleDetector()
        
        # Initialize physical camera (usually index 0 on RPi)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            print("[Hardware] WARNING: Cannot open camera index 0. Check connection.")
        else:
            # Set resolution to match simulation/YOLO requirements
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            print("[Hardware] Pi Camera initialized.")

    def process_frame(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
        dock_mode: bool = False,
        dock_x: float = 0.0,
        dock_y: float = 0.0,
    ) -> Tuple[List[ObstacleResult], Optional[dict], Optional[np.ndarray]]:
        """Capture a physical frame and run YOLO inference."""
        
        if not self.cap.isOpened():
            return [], None, None
            
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return [], None, None

        # Optionally flip frame if camera is mounted upside down
        # frame = cv2.flip(frame, -1)

        # 1. AI Obstacle Detection
        obstacles = self.detector.detect(frame)

        # 2. ArUco / Docking marker detection (stub for physical)
        dock_result = None
        if dock_mode:
            # On physical hardware, you'd run cv2.aruco.detectMarkers here
            pass

        return obstacles, dock_result, frame

    def cleanup(self) -> None:
        """Release the camera hardware."""
        if self.cap.isOpened():
            self.cap.release()
