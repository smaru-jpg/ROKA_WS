"""Publish a saved route and the real vehicle-width envelope for RViz replay."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy
import yaml

from roka_nav2_bringup.route_geometry import corridor_boundaries
from roka_robot_description.hardware_config import load_hardware_config


class RecordedRouteVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("recorded_route_visualizer")
        self.declare_parameter("profile_file", "")
        profile_file = Path(
            str(self.get_parameter("profile_file").value)
        ).expanduser().resolve()
        if not profile_file.is_file():
            raise ValueError(f"recorded route is missing: {profile_file}")
        document = yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {}
        self.frame_id = str(document.get("frame_id", "map")).lstrip("/")
        source = document.get("points")
        if not isinstance(source, list) or len(source) < 2:
            raise ValueError("recorded route needs at least two points")
        self.points: list[dict[str, float]] = []
        for index, entry in enumerate(source):
            if not isinstance(entry, dict):
                raise ValueError(f"recorded route point {index} is not a mapping")
            point = {"x": float(entry["x"]), "y": float(entry["y"])}
            yaw = entry.get("yaw")
            point["yaw"] = float(yaw) if yaw is not None else math.nan
            if not math.isfinite(point["x"]) or not math.isfinite(point["y"]):
                raise ValueError(f"recorded route point {index} is non-finite")
            self.points.append(point)
        for index, point in enumerate(self.points):
            if math.isfinite(point["yaw"]):
                continue
            before = self.points[max(0, index - 1)]
            after = self.points[min(len(self.points) - 1, index + 1)]
            point["yaw"] = math.atan2(
                after["y"] - before["y"], after["x"] - before["x"]
            )

        hardware = load_hardware_config()
        self.left, self.right = corridor_boundaries(
            self.points, hardware.navigation_half_width_m
        )
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.route_pub = self.create_publisher(
            PathMessage, "/replay/final_path", qos
        )
        self.left_pub = self.create_publisher(
            PathMessage, "/replay/vehicle_left", qos
        )
        self.right_pub = self.create_publisher(
            PathMessage, "/replay/vehicle_right", qos
        )
        self.growing_pub = self.create_publisher(
            PathMessage, "/replay/growing_path_latched", qos
        )
        live_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.growing_subscription = self.create_subscription(
            PathMessage,
            "/replay/growing_path",
            self.growing_pub.publish,
            live_qos,
        )
        self.timer = self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            "saved route visualization ready: %d points, vehicle width %.3f m"
            % (len(self.points), 2.0 * hardware.navigation_half_width_m)
        )

    def _message(self, points: list[dict[str, float]], z: float) -> PathMessage:
        message = PathMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point["x"]
            pose.pose.position.y = point["y"]
            pose.pose.position.z = z
            pose.pose.orientation.z = math.sin(0.5 * point["yaw"])
            pose.pose.orientation.w = math.cos(0.5 * point["yaw"])
            message.poses.append(pose)
        return message

    def _publish(self) -> None:
        self.route_pub.publish(self._message(self.points, 0.08))
        self.left_pub.publish(self._message(self.left, 0.07))
        self.right_pub.publish(self._message(self.right, 0.07))


def main() -> None:
    rclpy.init()
    node = RecordedRouteVisualizer()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
