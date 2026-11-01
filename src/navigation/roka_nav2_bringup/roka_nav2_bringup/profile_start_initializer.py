"""Seed the selected map localizer from the recorded route start."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import yaml


def load_profile_start(path: Path) -> tuple[str, float, float, float]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    points = document.get("points", [])
    if not isinstance(points, list) or not points:
        raise ValueError("track profile has no recorded start point")
    first = points[0]
    frame = str(document.get("frame_id", "map"))
    values = (float(first["x"]), float(first["y"]), float(first.get("yaw", 0.0)))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("track profile start contains a non-finite value")
    return frame, *values


class ProfileStartInitializer(Node):
    def __init__(self) -> None:
        super().__init__("profile_start_initializer")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("scan_topic", "/scan_stabilized")
        self.declare_parameter("require_scan", True)
        self.declare_parameter("startup_delay_sec", 2.0)
        self.declare_parameter("maximum_wait_sec", 45.0)
        self.declare_parameter("publish_count", 3)
        self.declare_parameter("position_stddev_m", 0.50)
        self.declare_parameter("yaw_stddev_deg", 60.0)

        profile = Path(str(self.get_parameter("profile_file").value)).expanduser()
        if not profile.is_file():
            raise ValueError(f"recorded start profile does not exist: {profile}")
        self.frame, self.x, self.y, self.yaw = load_profile_start(profile)
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.startup_delay = float(self.get_parameter("startup_delay_sec").value)
        self.maximum_wait = float(self.get_parameter("maximum_wait_sec").value)
        self.remaining = int(self.get_parameter("publish_count").value)
        self.position_variance = float(
            self.get_parameter("position_stddev_m").value
        ) ** 2
        self.yaw_variance = math.radians(float(
            self.get_parameter("yaw_stddev_deg").value
        )) ** 2
        if min(self.startup_delay, self.maximum_wait) < 0.0 or self.remaining <= 0:
            raise ValueError("start initializer timing and publish count are invalid")

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            10,
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_publisher = self.create_publisher(
            String, "/localization/start_seed_status", status_qos
        )
        self.scan_received = False
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.started = self.get_clock().now()
        self.last_publish = None
        self.done = False
        self.create_timer(0.20, self.update)
        self.publish_status(
            "waiting to seed localization from the recorded start; place the robot near "
            "the marked start with approximately the same heading"
        )

    def publish_status(self, value: str, *, error: bool = False) -> None:
        message = String()
        message.data = value
        self.status_publisher.publish(message)
        if error:
            self.get_logger().error(value)
        else:
            print(f"[LOCALIZATION][STATUS] {value}", flush=True)

    def scan_callback(self, _message: LaserScan) -> None:
        self.scan_received = True

    def update(self) -> None:
        if self.done:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds * 1.0e-9
        if elapsed > self.maximum_wait:
            self.done = True
            self.publish_status(
                "localization start seed timed out: no live scan or subscriber; "
                "set RViz 2D Pose Estimate manually",
                error=True,
            )
            return
        if elapsed < self.startup_delay:
            return
        if self.require_scan and not self.scan_received:
            return
        if self.publisher.get_subscription_count() <= 0:
            return
        if self.last_publish is not None and (
            now - self.last_publish
        ).nanoseconds * 1.0e-9 < 0.50:
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.frame
        # This is an approximate map-frame seed, not a time-synchronised
        # measurement. Request the latest odom transform to avoid startup
        # extrapolation while wheel TF is a few milliseconds behind wall time.
        message.header.stamp = rclpy.time.Time().to_msg()
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        message.pose.covariance[0] = self.position_variance
        message.pose.covariance[7] = self.position_variance
        message.pose.covariance[35] = self.yaw_variance
        self.publisher.publish(message)
        self.last_publish = now
        self.remaining -= 1
        if self.remaining == 0:
            self.done = True
            self.publish_status(
                f"localization seeded near recorded start x={self.x:.2f}, y={self.y:.2f}; "
                "wait for localization and the safety preflight to converge; "
                "approved auto-start proceeds only after both are ready"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProfileStartInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
