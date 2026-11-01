"""Shared low-overhead Ultralytics detector used by every ROKA mission."""

from hashlib import sha256
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO


def select_device(requested="auto"):
    requested = str(requested).strip().lower()
    if requested == "auto":
        return "0" if torch.cuda.is_available() else "cpu"
    return requested


class YoloDetector:
    def __init__(
        self,
        model_path,
        confidence=0.25,
        device="auto",
        image_size=640,
        expected_sha256="",
        expected_classes=None,
    ):
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")
        if expected_sha256:
            digest = sha256(self.model_path.read_bytes()).hexdigest()
            if digest != str(expected_sha256):
                raise RuntimeError(
                    f"YOLO checksum mismatch: expected {expected_sha256}, got {digest}"
                )

        self.confidence = float(confidence)
        self.image_size = int(image_size)
        self.device = select_device(device)
        if self.device == "cpu":
            torch.set_num_threads(min(2, torch.get_num_threads()))
        # OpenCV otherwise creates a worker pool even though display and frame
        # conversion are lightweight at 640x480.
        cv2.setNumThreads(1)
        self.model = YOLO(str(self.model_path))
        self.names = dict(self.model.names)
        if expected_classes is not None:
            normalized = {int(key): str(value) for key, value in expected_classes.items()}
            if self.names != normalized:
                raise RuntimeError(
                    f"YOLO class contract mismatch: expected {normalized}, got {self.names}"
                )

    def predict_result(self, frame):
        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )
        return results[0]

    def detections(self, frame):
        result = self.predict_result(frame)
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0])
                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
                detections.append({
                    "class_id": class_id,
                    "class_name": str(self.names[class_id]),
                    "confidence": float(box.conf[0]),
                    "bbox": [x1, y1, x2, y2],
                })
        return result, detections
