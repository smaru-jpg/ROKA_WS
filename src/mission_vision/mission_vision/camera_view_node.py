#!/usr/bin/env python3
"""Low-latency operator camera view for sections without a YOLO profile."""

from __future__ import annotations

import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from mission_vision.display import OpenCvDisplay


class CameraViewNode(Node):
    def __init__(self):
        super().__init__("section_camera_view")
        self.image_topic = str(self.declare_parameter(
            "image_topic", "/roka/arm_camera/color/image_raw"
        ).value)
        self.section = str(self.declare_parameter("section", "section4").value)
        self.display_enabled = bool(self.declare_parameter(
            "display_enabled", True
        ).value)
        self.maximum_display_rate_hz = float(self.declare_parameter(
            "maximum_display_rate_hz", 30.0
        ).value)
        if self.maximum_display_rate_hz <= 0.0:
            raise ValueError("maximum_display_rate_hz must be positive")
        self.bridge = CvBridge()
        self.display = OpenCvDisplay(
            self.display_enabled,
            f"ROKA {self.section} Manual Camera",
            self.get_logger(),
        )
        self.last_display_time = 0.0
        self.last_frame_time = 0.0
        self.warned_waiting = False
        self.create_subscription(
            Image, self.image_topic, self._image, qos_profile_sensor_data
        )
        self.create_timer(2.0, self._watchdog)
        self.get_logger().info(
            f"manual camera view: section={self.section}, topic={self.image_topic}"
        )

    def _image(self, message: Image) -> None:
        now = time.monotonic()
        self.last_frame_time = now
        self.warned_waiting = False
        if now - self.last_display_time < 1.0 / self.maximum_display_rate_hz:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().warning(f"camera conversion failed: {error}")
            return
        self.last_display_time = now
        cv2.putText(
            frame,
            f"{self.section.upper()} MANUAL CAMERA",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
        )
        self.display.show(frame)

    def _watchdog(self) -> None:
        age = time.monotonic() - self.last_frame_time
        if (self.last_frame_time == 0.0 or age > 2.0) and not self.warned_waiting:
            self.get_logger().warning(
                f"waiting for operator camera: {self.image_topic}"
            )
            self.warned_waiting = True

    def destroy_node(self):
        self.display.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
