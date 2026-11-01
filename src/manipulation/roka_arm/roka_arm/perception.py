import cv2
import numpy as np
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO

from . import arm_config as cfg


class Perception:
    """
    Provide color/depth conversion and box detection.

    현재는 arm 패키지 내부 helper.
    나중에 필요하면 별도 perception ROS2 node/package로 분리하기 쉽게 구성.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO(cfg.YOLO_MODEL_PATH)
        self.latest_depth = None
        self.latest_depth_stamp_ns = None
        if cfg.INFERENCE_DEVICE == "auto":
            self.inference_device = "0" if torch.cuda.is_available() else "cpu"
        else:
            self.inference_device = cfg.INFERENCE_DEVICE
        if self.inference_device == "cpu":
            torch.set_num_threads(min(2, torch.get_num_threads()))

    def update_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
        except Exception as e:
            print(f"[DEPTH ERROR] CvBridge 변환 실패: {e}")
            return False

        self.latest_depth = depth
        self.latest_depth_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        return True

    def image_to_bgr(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            print(f"[CAMERA ERROR] CvBridge 변환 실패: {e}")
            return None

    def detect_box(self, frame):
        try:
            results = self.model.predict(
                source=frame,
                conf=cfg.YOLO_CONFIDENCE,
                verbose=False,
                device=self.inference_device,
            )
        except Exception as e:
            print(f"[YOLO ERROR] 추론 실패: {e}")
            return None

        if not results:
            return None

        return self.select_box_detection(results[0])

    def select_box_detection(self, result):
        if result.boxes is None or len(result.boxes) == 0:
            return None

        best = None
        best_conf = -1.0

        for box in result.boxes:
            conf = float(box.conf[0])
            class_id = int(box.cls[0])

            if (
                conf < cfg.YOLO_CONFIDENCE
                or class_id not in cfg.TARGET_CLASS_IDS
            ):
                continue

            if conf > best_conf:
                best = box
                best_conf = conf

        if best is None:
            return None

        x1, y1, x2, y2 = best.xyxy[0].tolist()

        return {
            "x1": int(x1),
            "y1": int(y1),
            "x2": int(x2),
            "y2": int(y2),
            "cx": int((x1 + x2) / 2.0),
            "cy": int((y1 + y2) / 2.0),
            "conf": best_conf,
            "class_id": int(best.cls[0]),
            "class_name": self.model.names[int(best.cls[0])],
        }

    def get_box_depth_mm(self, detection, image_stamp_ns=None):
        if self.latest_depth is None or detection is None:
            return None

        if image_stamp_ns is not None and self.latest_depth_stamp_ns is not None:
            age_sec = abs(image_stamp_ns - self.latest_depth_stamp_ns) / 1.0e9
            if age_sec > cfg.MAXIMUM_DEPTH_AGE_SEC:
                return None

        depth = self.latest_depth

        if depth.ndim != 2:
            return None

        h, w = depth.shape[:2]
        cx = max(0, min(w - 1, detection["cx"]))
        cy = max(0, min(h - 1, detection["cy"]))

        r = cfg.DEPTH_SAMPLE_RADIUS

        x1 = max(0, cx - r)
        x2 = min(w, cx + r + 1)
        y1 = max(0, cy - r)
        y2 = min(h, cy + r + 1)

        roi = depth[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        valid = roi[np.isfinite(roi) & (roi > 0)]

        if valid.size == 0:
            return None

        median_value = float(np.median(valid))

        if np.issubdtype(depth.dtype, np.floating):
            return median_value * 1000.0

        return median_value

    def draw_overlay(self, frame, detection, depth_mm, servo_state, auto_control):
        h, w = frame.shape[:2]

        roi_left = max(0, cfg.TARGET_X - cfg.DEAD_ZONE_X)
        roi_right = min(w - 1, cfg.TARGET_X + cfg.DEAD_ZONE_X)
        roi_top = max(0, cfg.TARGET_Y - cfg.DEAD_ZONE_Y)
        roi_bottom = min(h - 1, cfg.TARGET_Y + cfg.DEAD_ZONE_Y)

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (roi_left, roi_top),
            (roi_right, roi_bottom),
            (0, 255, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.18, frame, 0.82, 0, frame)

        cv2.rectangle(
            frame,
            (roi_left, roi_top),
            (roi_right, roi_bottom),
            (0, 255, 0),
            2,
        )

        cv2.line(
            frame,
            (cfg.TARGET_X, 0),
            (cfg.TARGET_X, h - 1),
            (0, 255, 255),
            1,
        )
        cv2.line(
            frame,
            (0, cfg.TARGET_Y),
            (w - 1, cfg.TARGET_Y),
            (0, 255, 255),
            1,
        )

        if detection is not None:
            x1 = detection["x1"]
            y1 = detection["y1"]
            x2 = detection["x2"]
            y2 = detection["y2"]
            cx = detection["cx"]
            cy = detection["cy"]
            conf = detection["conf"]
            class_name = detection["class_name"]

            cv2.rectangle(
                frame, (x1, y1), (x2, y2), (255, 0, 0), 2
            )
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

            ex = cx - cfg.TARGET_X
            ey = cy - cfg.TARGET_Y

            depth_text = (
                f"{depth_mm:.0f} mm"
                if depth_mm is not None
                else "N/A"
            )

            cv2.putText(
                frame,
                f"{class_name} cx={cx} cy={cy} ex={ex} ey={ey} conf={conf:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"Depth={depth_text} Target={cfg.TARGET_DEPTH_MM} mm",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )
        else:
            cv2.putText(
                frame,
                "BOX NOT DETECTED",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        auto_text = "ON" if auto_control else "OFF"

        cv2.putText(
            frame,
            f"AUTO: {auto_text} STATE: {servo_state}",
            (10, h - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            (
                f"ROI X:{roi_left}~{roi_right} "
                f"Y:{roi_top}~{roi_bottom} [a:auto, q:quit]"
            ),
            (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
