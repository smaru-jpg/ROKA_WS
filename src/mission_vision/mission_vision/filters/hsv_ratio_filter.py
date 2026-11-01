"""Measure an HSV color ratio inside selected YOLO bounding boxes."""

import cv2
import numpy as np


class HSVRatioFilter:
    """Annotate detections with an HSV ratio and optionally reject weak matches."""

    def __init__(
        self,
        target_classes=None,
        h_min=0,
        h_max=179,
        s_min=0,
        s_max=255,
        v_min=0,
        v_max=255,
        min_ratio=0.0,
        reject_below=False,
        result_name="hsv",
    ):
        if isinstance(target_classes, str):
            target_classes = [target_classes]
        self.target_classes = {
            str(value).strip().lower()
            for value in (target_classes or [])
        }
        self.h_min = self._bounded_integer("h_min", h_min, 0, 179)
        self.h_max = self._bounded_integer("h_max", h_max, 0, 179)
        self.s_min = self._bounded_integer("s_min", s_min, 0, 255)
        self.s_max = self._bounded_integer("s_max", s_max, 0, 255)
        self.v_min = self._bounded_integer("v_min", v_min, 0, 255)
        self.v_max = self._bounded_integer("v_max", v_max, 0, 255)
        if self.s_min > self.s_max or self.v_min > self.v_max:
            raise ValueError("HSV saturation/value minimum cannot exceed maximum")

        self.min_ratio = float(min_ratio)
        if not 0.0 <= self.min_ratio <= 1.0:
            raise ValueError("min_ratio must be in [0, 1]")
        self.reject_below = bool(reject_below)
        self.result_name = str(result_name).strip()
        if not self.result_name:
            raise ValueError("result_name must not be empty")

    @staticmethod
    def _bounded_integer(name, value, minimum, maximum):
        parsed = int(value)
        if not minimum <= parsed <= maximum:
            raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
        return parsed

    def _mask(self, hsv):
        """Support both ordinary and hue-wrap intervals such as red 170..10."""
        if self.h_min <= self.h_max:
            return cv2.inRange(
                hsv,
                np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8),
                np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8),
            )
        high_hue = cv2.inRange(
            hsv,
            np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8),
            np.array([179, self.s_max, self.v_max], dtype=np.uint8),
        )
        low_hue = cv2.inRange(
            hsv,
            np.array([0, self.s_min, self.v_min], dtype=np.uint8),
            np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8),
        )
        return cv2.bitwise_or(high_hue, low_hue)

    def process(self, detections, context):
        frame = context.get("frame")
        if frame is None:
            return detections
        if not isinstance(frame, np.ndarray) or frame.ndim != 3:
            raise ValueError("HSVRatioFilter requires a BGR image")

        frame_height, frame_width = frame.shape[:2]
        filtered = []
        for detection in detections:
            class_name = str(detection["class_name"]).strip().lower()
            if self.target_classes and class_name not in self.target_classes:
                filtered.append(detection)
                continue

            x1, y1, x2, y2 = (int(value) for value in detection["bbox"])
            x1 = max(0, min(x1, frame_width))
            x2 = max(0, min(x2, frame_width))
            y1 = max(0, min(y1, frame_height))
            y2 = max(0, min(y2, frame_height))
            ratio = 0.0
            if x2 > x1 and y2 > y1:
                roi = frame[y1:y2, x1:x2]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                matched_pixels = cv2.countNonZero(self._mask(hsv))
                ratio = matched_pixels / float(roi.shape[0] * roi.shape[1])

            detection.setdefault("hsv_results", {})[self.result_name] = ratio
            if not self.reject_below or ratio >= self.min_ratio:
                filtered.append(detection)

        return filtered
