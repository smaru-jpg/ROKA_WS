
from pathlib import Path
import time

import cv2
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from mission_vision.cameras.camera_manager import CameraManager
from mission_vision.detector import YoloDetector
from mission_vision.display import OpenCvDisplay
from mission_vision.filters.filter_manager import FilterManager


def format_detection_label(detection, passed):
    """Build the field-tuning overlay including all measured HSV ratios."""
    label = (
        f"{detection['class_name']} "
        f"{float(detection['confidence']):.2f}"
    )
    for name, ratio in sorted(detection.get("hsv_results", {}).items()):
        label += f" {name}:{float(ratio):.2f}"
    return f"{label} {'PASS' if passed else 'REJECT'}"


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        # The only ROS parameter needed for normal use.
        # Relative paths are resolved inside this installed package.
        self.declare_parameter(
            "config_file",
            "config/spring.yaml",
        )

        package_share = Path(
            get_package_share_directory("mission_vision")
        )

        config_param = str(
            self.get_parameter("config_file").value
        )

        config_path = Path(config_param)
        if not config_path.is_absolute():
            config_path = package_share / config_path

        if not config_path.exists():
            raise FileNotFoundError(
                f"Mission config not found: {config_path}"
            )

        with config_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            self.config = yaml.safe_load(file) or {}

        self.mission_name = str(
            self.config.get("mission", {}).get(
                "name",
                "unknown",
            )
        )
        self.declare_parameter("enabled", True)
        self.enabled = bool(self.get_parameter("enabled").value)

        # ------------------------------------------------------------
        # Camera
        # ------------------------------------------------------------
        camera_config = dict(self.config.get("camera", {}))
        self.declare_parameter(
            "image_topic",
            str(camera_config.get("topic", "/roka/arm_camera/color/image_raw")),
        )
        camera_config["topic"] = str(self.get_parameter("image_topic").value)
        self.camera = CameraManager(camera_config, self)

        # ------------------------------------------------------------
        # YOLO model
        # ------------------------------------------------------------
        model_config = self.config.get("model", {})
        model_path_param = str(
            model_config.get("path", "")
        ).strip()
        if not model_path_param:
            raise ValueError("model.path is required; implicit best.pt fallback is forbidden")
        model_sha256 = str(model_config.get("sha256", "")).strip().lower()
        expected_classes = model_config.get("expected_classes")
        if len(model_sha256) != 64 or any(
            character not in "0123456789abcdef" for character in model_sha256
        ):
            raise ValueError("model.sha256 must be a complete 64-character digest")
        if not isinstance(expected_classes, dict) or not expected_classes:
            raise ValueError("model.expected_classes is required")

        model_path = Path(model_path_param)
        if not model_path.is_absolute():
            model_path = package_share / model_path

        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO model not found: {model_path}"
            )

        self.model_path = model_path
        self.yolo_conf_threshold = float(
            model_config.get(
                "yolo_conf_threshold",
                0.25,
            )
        )
        if not 0.0 < self.yolo_conf_threshold <= 1.0:
            raise ValueError("model.yolo_conf_threshold must be in (0, 1]")
        self.declare_parameter(
            "inference_device", str(model_config.get("device", "auto"))
        )
        self.detector = YoloDetector(
            self.model_path,
            confidence=self.yolo_conf_threshold,
            device=str(self.get_parameter("inference_device").value),
            image_size=int(model_config.get("image_size", 640)),
            expected_sha256=model_sha256,
            expected_classes=expected_classes,
        )
        self.model = self.detector.model

        # ------------------------------------------------------------
        # Inference/result settings
        # ------------------------------------------------------------
        inference_config = self.config.get(
            "inference",
            {},
        )

        self.detection_mode = str(
            inference_config.get(
                "detection_mode",
                "single",
            )
        ).lower()

        if self.detection_mode not in (
            "single",
            "multiple",
        ):
            raise ValueError(
                "inference.detection_mode must be "
                "'single' or 'multiple'"
            )

        self.timer_period_sec = float(
            inference_config.get(
                "timer_period_sec",
                0.1,
            )
        )
        if self.timer_period_sec <= 0.0:
            raise ValueError("inference.timer_period_sec must be positive")

        display_config = self.config.get(
            "display",
            {},
        )
        self.declare_parameter(
            "display_enabled", bool(display_config.get("enabled", True))
        )
        self.display_enabled = bool(self.get_parameter("display_enabled").value)
        self.display = OpenCvDisplay(
            self.display_enabled, "Mission Vision", self.get_logger()
        )

        ros_config = self.config.get(
            "ros",
            {},
        )
        self.result_topic = str(
            ros_config.get(
                "result_topic",
                "/roka/vision/result",
            )
        )

        # ------------------------------------------------------------
        # Dynamic filter pipeline
        # ------------------------------------------------------------
        self.filter_manager = FilterManager(
            self.config.get("pipeline", {}),
            logger=self.get_logger(),
        )

        # ------------------------------------------------------------
        # ROS publisher
        # ------------------------------------------------------------
        self.publisher = self.create_publisher(
            String,
            self.result_topic,
            10,
        )
        self.enable_service_name = f"/roka/vision/{self.mission_name}/enable"
        self.enable_service = self.create_service(
            SetBool,
            self.enable_service_name,
            self._set_enabled,
        )
        self.last_camera_warning = 0.0

        # ------------------------------------------------------------
        # Startup information
        # ------------------------------------------------------------
        self.get_logger().info(
            f"Mission: {self.mission_name}"
        )
        self.get_logger().info(
            f"Config: {config_path}"
        )
        self.get_logger().info(
            f"Camera type: {self.camera.camera_type}"
        )
        self.get_logger().info(
            f"Model: {self.model_path}"
        )
        self.get_logger().info(
            f"Model classes: {self.detector.names}"
        )
        self.get_logger().info(
            f"YOLO threshold: {self.yolo_conf_threshold}"
        )
        self.get_logger().info(
            f"Detection mode: {self.detection_mode}"
        )
        self.get_logger().info(
            f"Filters: {self.filter_manager.describe()}"
        )
        self.get_logger().info(
            f"Result topic: {self.result_topic}"
        )
        self.get_logger().info(
            f"Inference enabled: {self.enabled}; service={self.enable_service_name}"
        )

        self.timer = self.create_timer(
            self.timer_period_sec,
            self.detect_callback,
        )

        self.get_logger().info(
            "Mission Vision Node started"
        )

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = (
            f"{self.mission_name} inference enabled"
            if self.enabled
            else f"{self.mission_name} inference disabled"
        )
        self.get_logger().info(response.message)
        return response

    def detect_callback(self):
        if not self.enabled:
            return
        frame = self.camera.read()

        if frame is None:
            now = time.monotonic()
            frame_age = self.camera.age_seconds()
            camera_stale = frame_age is None or frame_age >= 2.0
            if camera_stale and now - self.last_camera_warning >= 5.0:
                detail = "no frame received" if frame_age is None else f"last frame age={frame_age:.1f}s"
                self.get_logger().warning(
                    f"Waiting for camera on {self.camera.camera.topic}; {detail}"
                )
                self.last_camera_warning = now
            return

        height, width = frame.shape[:2]

        context = {
            "mission_name": self.mission_name,
            "detection_mode": self.detection_mode,
            "frame": frame,
            "frame_width": width,
            "frame_height": height,
            "timestamp": time.monotonic(),
        }

        # ------------------------------------------------------------
        # Stage 1: frame preprocessing
        # ------------------------------------------------------------
        processed_frame = self.filter_manager.apply(
            "preprocess",
            frame,
            context,
        )

        context["frame"] = processed_frame
        height, width = processed_frame.shape[:2]
        context["frame_width"] = width
        context["frame_height"] = height

        # ------------------------------------------------------------
        # YOLO
        # ------------------------------------------------------------
        result, raw_detections = self.detector.detections(processed_frame)

        context["raw_detections"] = raw_detections

        # ------------------------------------------------------------
        # Stage 2: detection filters
        # ------------------------------------------------------------
        filtered_detections = self.filter_manager.apply(
            "detection",
            raw_detections,
            context,
        )

        context["filtered_detections"] = filtered_detections

        # ------------------------------------------------------------
        # Convert detections into a class-level result
        # ------------------------------------------------------------
        preliminary_result = self._make_preliminary_result(
            filtered_detections
        )

        context["preliminary_result"] = preliminary_result

        # ------------------------------------------------------------
        # Stage 3: result filters
        # ------------------------------------------------------------
        final_result = self.filter_manager.apply(
            "result",
            preliminary_result,
            context,
        )

        vision_result = self._format_result(
            final_result
        )

        # ------------------------------------------------------------
        # ROS publish
        # ------------------------------------------------------------
        msg = String()
        msg.data = vision_result
        self.publisher.publish(msg)

        # ------------------------------------------------------------
        # Debug display
        # ------------------------------------------------------------
        if self.display.enabled:
            annotated = processed_frame.copy()
            filtered_ids = {
                id(detection)
                for detection in filtered_detections
            }

            for detection in raw_detections:
                x1, y1, x2, y2 = (
                    int(value)
                    for value in detection["bbox"]
                )
                passed = id(detection) in filtered_ids
                color = (0, 255, 0) if passed else (0, 0, 255)
                label = format_detection_label(detection, passed)
                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    color,
                    2,
                )
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )

            raw_text = f"RAW: {len(raw_detections)}"
            filtered_text = (
                f"FILTERED: {len(filtered_detections)}"
            )
            final_text = f"FINAL: {vision_result}"

            cv2.putText(
                annotated,
                raw_text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                filtered_text,
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                final_text,
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            self.display.show(annotated)

    def _make_preliminary_result(
        self,
        detections,
    ):
        if self.detection_mode == "single":
            if not detections:
                return None

            best = max(
                detections,
                key=lambda detection: detection["confidence"],
            )
            return best["class_name"]

        # multiple mode: keep each class at most once per frame,
        # prioritizing the higher-confidence detection.
        ordered = sorted(
            detections,
            key=lambda detection: detection["confidence"],
            reverse=True,
        )

        class_names = []
        seen = set()

        for detection in ordered:
            name = detection["class_name"]
            if name not in seen:
                seen.add(name)
                class_names.append(name)

        return class_names

    @staticmethod
    def _format_result(result):
        if result is None:
            return "NONE"

        if isinstance(result, str):
            return result.upper()

        if isinstance(result, (list, tuple, set)):
            values = [
                str(value).upper()
                for value in result
                if value is not None
            ]
            if not values:
                return "NONE"
            return ",".join(values)

        return str(result).upper()

    def destroy_node(self):
        try:
            self.camera.close()
        finally:
            self.display.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"[MISSION-VISION][ERROR] {error}", flush=True)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ros2 launch may forward SIGINT while the parent terminal is
                # already interrupting this cleanup path.
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
