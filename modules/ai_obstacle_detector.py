"""
ai_obstacle_detector.py -- Indoor hospital obstacle detection using YOLOv8-Nano.

Detects and classifies obstacles specific to indoor hospital environments:
  - PERSON:           staff, patients, visitors
  - CART:             wheelchairs, trolleys, wheeled carts
  - FURNITURE:        chairs, couches, benches, tables
  - MEDICAL_EQUIPMENT: hospital beds, gurneys, IV stands
  - EQUIPMENT:        bags, monitors, portable devices

Only indoor-relevant COCO classes are kept; all outdoor detections
(cars, trucks, animals, sports equipment, etc.) are filtered out.

Operates in dual mode:
  - YOLO:      loads YOLOv8n ONNX model via OpenCV DNN (320x320 input)
  - HEURISTIC: fallback classifier from MOG2 contours when no model present

RPi4 optimizations:
  - 320x320 input (4x fewer pixels than 640x640)
  - Skip-frame inference (every 5th frame, reuse last result)
  - Aggressive NMS (0.45) and confidence threshold (0.35)
  - Pre-allocated numpy buffers (zero per-frame allocation)
  - Indoor-only class filter (17 classes from 80 COCO classes)
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import (
    FRAME_W, FRAME_H,
    OBS_ROI_TOP, OBS_ROI_BOTTOM,
    OBS_MIN_AREA_PX, OBS_SLOW_AREA_PX, OBS_STOP_AREA_PX,
    YOLO_MODEL_PATH, YOLO_INPUT_SIZE,
    YOLO_CONF_THRESHOLD, YOLO_NMS_THRESHOLD,
    YOLO_SKIP_FRAMES, YOLO_CLASSES_OF_INTEREST,
    ObstacleAction, ObstacleClass, ObstacleResult,
)

# COCO class ID -> Indoor ObstacleClass mapping
# Only maps classes relevant to hospital corridor navigation
_CLASS_MAP: Dict[int, ObstacleClass] = {
    # ── People (dynamic, highest priority) ───────
    0:  ObstacleClass.PERSON,           # person (staff, patient, visitor)
    # ── Wheeled objects (carts, wheelchairs) ─────
    1:  ObstacleClass.CART,             # bicycle -> wheelchair/cart proxy
    # ── Furniture (static indoor obstacles) ──────
    56: ObstacleClass.FURNITURE,        # chair
    57: ObstacleClass.FURNITURE,        # couch / waiting area seating
    60: ObstacleClass.FURNITURE,        # dining table -> desk / nurses station
    13: ObstacleClass.FURNITURE,        # bench
    58: ObstacleClass.FURNITURE,        # potted plant (hallway decoration)
    # ── Medical equipment (hospital-specific) ────
    59: ObstacleClass.MEDICAL_EQUIPMENT,  # bed -> hospital bed / gurney
    39: ObstacleClass.MEDICAL_EQUIPMENT,  # bottle -> IV fluid container
    # ── Portable equipment ───────────────────────
    24: ObstacleClass.EQUIPMENT,        # backpack -> bags / carried items
    26: ObstacleClass.EQUIPMENT,        # handbag -> staff bags
    28: ObstacleClass.EQUIPMENT,        # suitcase -> rolling equipment case
    63: ObstacleClass.EQUIPMENT,        # laptop -> mobile workstation
    62: ObstacleClass.EQUIPMENT,        # tv -> corridor display / monitor
    73: ObstacleClass.EQUIPMENT,        # book -> charts / clipboard
    74: ObstacleClass.EQUIPMENT,        # clock (static, landmark)
    67: ObstacleClass.EQUIPMENT,        # cell phone (small, usually ignored)
}


class AIObstacleDetector:
    """YOLOv8-Nano obstacle detector with RPi4-optimized inference.

    When an ONNX model file is available at YOLO_MODEL_PATH, full
    deep-learning inference is used via OpenCV DNN.  Otherwise, a
    lightweight heuristic classifier assigns categories to MOG2
    detections based on size and aspect ratio.

    Skip-frame logic reuses the last detection result for N-1 out
    of every N frames, drastically reducing CPU load on RPi4.
    """

    def __init__(self) -> None:
        self._net: Optional[cv2.dnn.Net] = None
        self._use_yolo: bool = False
        self._input_size: int = YOLO_INPUT_SIZE
        self._frame_count: int = 0
        self._cached_results: List[ObstacleResult] = []

        # MOG2 fallback
        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history=120, varThreshold=40, detectShadows=False,
        )
        self._kern_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self._kern_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        # Pre-allocated input blob buffer (320x320x3)
        self._blob_buffer = np.zeros(
            (1, 3, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE), dtype=np.float32
        )

        # Try to load ONNX model
        self._try_load_model()

    @property
    def mode(self) -> str:
        """Return current detection mode: 'YOLO' or 'HEURISTIC'."""
        return "YOLO" if self._use_yolo else "HEURISTIC"

    # -- public API -----------------------------------------

    def detect(self, frame: np.ndarray) -> List[ObstacleResult]:
        """Run obstacle detection on a BGR camera frame.

        Uses skip-frame logic: full inference only every YOLO_SKIP_FRAMES
        frames, returns cached results for intermediate frames.

        Parameters
        ----------
        frame : np.ndarray
            BGR image (640x480).

        Returns
        -------
        List[ObstacleResult]
            Detected obstacles with classification and confidence.
        """
        self._frame_count += 1

        # Skip-frame: reuse cached results for non-inference frames
        if self._frame_count % YOLO_SKIP_FRAMES != 1 and self._cached_results:
            return self._cached_results

        if self._use_yolo:
            # FIX (MT3608 review): a single inference fault used to propagate
            # straight out of detect() and terminate the 30 FPS control loop.
            # Perception now degrades to the heuristic classifier instead of
            # stopping the vehicle's brain mid-corridor.
            try:
                results = self._detect_yolo(frame)
            except Exception as e:
                print(f"[AIDetector] YOLO inference failed ({e}); "
                      f"falling back to heuristic classifier.")
                self._use_yolo = False
                results = self._detect_heuristic(frame)
        else:
            results = self._detect_heuristic(frame)

        self._cached_results = results
        return results

    # -- YOLO inference -------------------------------------

    def _try_load_model(self) -> None:
        """Attempt to load the YOLOv8n ONNX model."""
        # Try both relative to project root and current dir
        paths_to_try = [
            os.path.join(os.path.dirname(__file__), "..", YOLO_MODEL_PATH),
            YOLO_MODEL_PATH,
        ]
        for path in paths_to_try:
            if os.path.exists(path):
                try:
                    self._net = cv2.dnn.readNetFromONNX(path)
                    # Prefer CPU backend (RPi4 has no GPU)
                    self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

                    # FIX (MT3608 review): YOLOv8 ONNX exports carry a FIXED
                    # input shape. If config.YOLO_INPUT_SIZE disagrees with it,
                    # forward() throws a Reshape assertion on the first frame.
                    # Probe the model once and adopt its real size.
                    self._input_size = self._probe_input_size(path)
                    if self._input_size != YOLO_INPUT_SIZE:
                        print(f"[AIDetector] NOTE: model expects "
                              f"{self._input_size}x{self._input_size}, config "
                              f"says {YOLO_INPUT_SIZE}. Using the model's size.")

                    self._use_yolo = True
                    print(f"[AIDetector] YOLOv8n loaded from {path} "
                          f"@ {self._input_size}x{self._input_size}")
                    return
                except Exception as e:
                    print(f"[AIDetector] Failed to load ONNX: {e}")
                    break

        print("[AIDetector] No ONNX model found -- using heuristic classifier")

    def _probe_input_size(self, path: str) -> int:
        """Determine the ONNX model's expected square input size.

        Tries the graph metadata first, then falls back to trial inference
        over the common YOLOv8 export sizes.
        """
        try:
            import onnxruntime as ort
            so = ort.SessionOptions()
            so.log_severity_level = 3
            sess = ort.InferenceSession(
                path, so, providers=["CPUExecutionProvider"])
            shape = sess.get_inputs()[0].shape
            if isinstance(shape[-1], int) and shape[-1] > 0:
                return int(shape[-1])
        except Exception:
            pass

        # Fallback: probe candidate sizes with a dummy forward pass.
        for size in (YOLO_INPUT_SIZE, 320, 640, 160, 256, 416):
            try:
                dummy = np.zeros((size, size, 3), dtype=np.uint8)
                blob = cv2.dnn.blobFromImage(
                    dummy, 1.0 / 255.0, (size, size), swapRB=True, crop=False)
                self._net.setInput(blob)
                self._net.forward()
                return size
            except Exception:
                continue
        return YOLO_INPUT_SIZE

    def _detect_yolo(self, frame: np.ndarray) -> List[ObstacleResult]:
        """Run YOLOv8n inference via OpenCV DNN."""
        assert self._net is not None

        # 1. Preprocess: letterbox resize + normalize
        size = getattr(self, "_input_size", YOLO_INPUT_SIZE)
        blob = cv2.dnn.blobFromImage(
            frame, scalefactor=1.0 / 255.0,
            size=(size, size),
            swapRB=True, crop=False,
        )

        # 2. Forward pass
        self._net.setInput(blob)
        outputs = self._net.forward()

        # 3. Postprocess
        return self._postprocess_yolo(outputs, frame.shape)

    def _postprocess_yolo(
        self, outputs: np.ndarray, frame_shape: Tuple[int, ...]
    ) -> List[ObstacleResult]:
        """Parse YOLOv8 output tensor into ObstacleResult list.

        YOLOv8 output shape: (1, 84, N) where 84 = 4 bbox + 80 classes
        Transposed to (N, 84) for processing.
        """
        h, w = frame_shape[:2]
        size = getattr(self, "_input_size", YOLO_INPUT_SIZE)
        scale_x = w / size
        scale_y = h / size

        # Handle different output shapes
        if len(outputs.shape) == 3:
            preds = outputs[0].T  # (N, 84)
        else:
            preds = outputs

        if preds.shape[1] < 5:
            return []

        boxes: List[List[int]] = []
        confidences: List[float] = []
        class_ids: List[int] = []

        for detection in preds:
            # bbox: cx, cy, w, h
            cx, cy, bw, bh = detection[:4]
            class_scores = detection[4:]

            max_score = float(np.max(class_scores))
            class_id = int(np.argmax(class_scores))

            if max_score < YOLO_CONF_THRESHOLD:
                continue

            # Only keep classes of interest
            if class_id not in YOLO_CLASSES_OF_INTEREST:
                continue

            # Convert to pixel coordinates
            x1 = int((cx - bw / 2) * scale_x)
            y1 = int((cy - bh / 2) * scale_y)
            bw_px = int(bw * scale_x)
            bh_px = int(bh * scale_y)

            boxes.append([x1, y1, bw_px, bh_px])
            confidences.append(max_score)
            class_ids.append(class_id)

        if not boxes:
            return []

        # NMS
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, YOLO_CONF_THRESHOLD, YOLO_NMS_THRESHOLD
        )

        results: List[ObstacleResult] = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, bw_px, bh_px = boxes[i]
                area = bw_px * bh_px
                bottom_y = y + bh_px
                proximity = bottom_y / h

                # Action based on area
                if area >= OBS_STOP_AREA_PX:
                    action = ObstacleAction.STOP
                elif area >= OBS_SLOW_AREA_PX:
                    action = ObstacleAction.SLOW
                else:
                    action = ObstacleAction.NOMINAL

                classification = _CLASS_MAP.get(
                    class_ids[i], ObstacleClass.UNKNOWN
                )

                results.append(ObstacleResult(
                    bbox=(x, y, bw_px, bh_px),
                    area=area,
                    proximity=proximity,
                    action=action,
                    classification=classification,
                    confidence=confidences[i],
                ))

        return results

    # -- Heuristic fallback ---------------------------------

    def _detect_heuristic(self, frame: np.ndarray) -> List[ObstacleResult]:
        """MOG2 background subtraction + indoor-specific classifier.

        Classifies detected blobs for indoor hospital environment:
          - Tall + narrow (aspect > 1.5)       -> PERSON (standing staff/patient)
          - Wide + short (aspect < 0.6)        -> CART (wheelchair/trolley)
          - Large + medium aspect (area > 8000) -> MEDICAL_EQUIPMENT (bed/gurney)
          - Medium + square-ish               -> FURNITURE (chair/table)
          - Otherwise                          -> EQUIPMENT (bags/devices)
        """
        roi_top = int(FRAME_H * OBS_ROI_TOP)
        roi_bot = int(FRAME_H * OBS_ROI_BOTTOM)
        roi = frame[roi_top:roi_bot, :]

        mask = self._bg_sub.apply(roi, learningRate=0.005)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kern_close)
        mask = cv2.dilate(mask, self._kern_dilate, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        results: List[ObstacleResult] = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            if area < OBS_MIN_AREA_PX:
                continue

            full_y = y + roi_top
            bottom_y = full_y + h
            proximity = bottom_y / FRAME_H

            # Action
            if area >= OBS_STOP_AREA_PX:
                action = ObstacleAction.STOP
            elif area >= OBS_SLOW_AREA_PX:
                action = ObstacleAction.SLOW
            else:
                action = ObstacleAction.NOMINAL

            # Indoor heuristic classification by aspect ratio + area
            aspect = h / max(w, 1)
            if aspect > 1.5:
                # Tall, narrow → standing person
                classification = ObstacleClass.PERSON
                conf = 0.70 + min(0.2, area / 20000)
            elif aspect < 0.6:
                # Wide, flat → wheelchair / cart / trolley
                classification = ObstacleClass.CART
                conf = 0.60 + min(0.2, area / 25000)
            elif area > 8000 and 0.6 <= aspect <= 1.2:
                # Large, roughly square → hospital bed / gurney
                classification = ObstacleClass.MEDICAL_EQUIPMENT
                conf = 0.55 + min(0.2, area / 30000)
            elif area > 3000:
                # Medium → furniture (chair, table)
                classification = ObstacleClass.FURNITURE
                conf = 0.50 + min(0.2, area / 25000)
            else:
                # Small → portable equipment
                classification = ObstacleClass.EQUIPMENT
                conf = 0.45 + min(0.2, area / 30000)

            results.append(ObstacleResult(
                bbox=(x, full_y, w, h),
                area=area,
                proximity=proximity,
                action=action,
                classification=classification,
                confidence=min(0.99, conf),
            ))

        return results


# -- Standalone test ----------------------------------------
if __name__ == "__main__":
    detector = AIObstacleDetector()
    print(f"Detection mode: {detector.mode}")

    # Generate a test frame with a synthetic obstacle
    frame = np.full((FRAME_H, FRAME_W, 3), (180, 175, 165), dtype=np.uint8)
    cv2.rectangle(frame, (250, 200), (320, 380), (80, 120, 160), -1)

    # Run detection multiple times to train MOG2
    for i in range(150):
        if i == 130:
            cv2.rectangle(frame, (250, 200), (320, 380), (80, 120, 160), -1)
        results = detector.detect(frame)
        if results:
            for r in results:
                print(f"  Frame {i}: {r.classification.value} "
                      f"conf={r.confidence:.2f} area={r.area} "
                      f"action={r.action.value}")
            break

    print(f"Final mode: {detector.mode}")
    print("Test complete.")
