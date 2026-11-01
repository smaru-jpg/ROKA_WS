#!/usr/bin/env python3
"""One-shot live VLP-16/EBIMU/wheel/TF contract checker."""

from collections import defaultdict
import math
from pathlib import Path
import statistics

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import yaml


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class LiveSensorContract(Node):
    def __init__(self):
        super().__init__("roka_sensor_contract")
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("output", "")
        self.declare_parameter("minimum_lidar_hz", 9.0)
        self.declare_parameter("minimum_imu_hz", 90.0)
        self.declare_parameter("minimum_wheel_hz", 10.0)
        self.stamps = defaultdict(list)
        self.tf_parents = defaultdict(set)
        self.point_fields = {}
        self.point_time_min = None
        self.point_time_max = None
        self.invalid = defaultdict(int)
        self.imu_acceleration_norms = []
        self.imu_quaternion_norms = []
        self.imu_covariance_valid = []
        self.wheel_pose_covariance_valid = []
        self.wheel_twist_covariance_valid = []
        self.create_subscription(
            PointCloud2, "/velodyne_points", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/wheel/odom", self._wheel, qos_profile_sensor_data
        )
        self.create_subscription(TFMessage, "/tf", self._tf, 100)
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TFMessage, "/tf_static", self._tf, static_tf_qos)
        self.create_timer(float(self.get_parameter("duration_sec").value), self._finish)
        self.finished = False
        self.report = None

    def _remember(self, topic, message):
        self.stamps[topic].append(_stamp_seconds(message.header.stamp))

    def _cloud(self, message):
        self._remember("/velodyne_points", message)
        self.point_fields = {field.name: field.datatype for field in message.fields}
        try:
            values = point_cloud2.read_points(
                message, field_names=["time"], skip_nans=False
            )["time"]
            finite = [float(value) for value in values if math.isfinite(float(value))]
            if finite:
                self.point_time_min = min(finite)
                self.point_time_max = max(finite)
        except (AssertionError, KeyError, ValueError):
            self.point_time_min = None
            self.point_time_max = None

    def _imu(self, message):
        self._remember("/imu/data", message)
        values = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid["/imu/data"] += 1
        else:
            self.imu_acceleration_norms.append(
                math.sqrt(sum(value * value for value in values[3:]))
            )
        quaternion = message.orientation
        quaternion_values = (
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )
        if all(math.isfinite(value) for value in quaternion_values):
            self.imu_quaternion_norms.append(
                math.sqrt(sum(value * value for value in quaternion_values))
            )
        self.imu_covariance_valid.append(
            message.orientation_covariance[0] >= 0.0
            and any(abs(value) > 0.0 for value in message.orientation_covariance)
            and any(abs(value) > 0.0 for value in message.angular_velocity_covariance)
            and any(abs(value) > 0.0 for value in message.linear_acceleration_covariance)
        )

    def _wheel(self, message):
        self._remember("/wheel/odom", message)
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid["/wheel/odom"] += 1
        self.wheel_pose_covariance_valid.append(
            any(abs(value) > 0.0 for value in message.pose.covariance)
        )
        self.wheel_twist_covariance_valid.append(
            any(abs(value) > 0.0 for value in message.twist.covariance)
        )

    def _tf(self, message):
        for transform in message.transforms:
            self.tf_parents[transform.child_frame_id].add(transform.header.frame_id)

    def _finish(self):
        if self.finished:
            return
        self.finished = True
        failures = []
        topics = {}
        limits = {
            "/velodyne_points": float(self.get_parameter("minimum_lidar_hz").value),
            "/imu/data": float(self.get_parameter("minimum_imu_hz").value),
            "/wheel/odom": float(self.get_parameter("minimum_wheel_hz").value),
        }
        for topic, limit in limits.items():
            stamps = self.stamps[topic]
            intervals = [b - a for a, b in zip(stamps, stamps[1:])]
            duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
            rate = (len(stamps) - 1) / duration if duration > 0.0 else 0.0
            backwards = sum(interval < 0.0 for interval in intervals)
            topics[topic] = {
                "count": len(stamps),
                "rate_hz": round(rate, 3),
                "header_backwards": backwards,
                "invalid_values": self.invalid[topic],
            }
            if rate < limit:
                failures.append(f"{topic} rate {rate:.2f}Hz is below {limit:.2f}Hz")
            if backwards:
                failures.append(f"{topic} has {backwards} backwards timestamps")
            if self.invalid[topic]:
                failures.append(f"{topic} has non-finite values")

        required_fields = {
            "x": PointField.FLOAT32,
            "y": PointField.FLOAT32,
            "z": PointField.FLOAT32,
            "intensity": PointField.FLOAT32,
            "ring": PointField.UINT16,
            "time": PointField.FLOAT32,
        }
        wrong_fields = {
            name: {"expected": datatype, "actual": self.point_fields.get(name)}
            for name, datatype in required_fields.items()
            if self.point_fields.get(name) != datatype
        }
        if wrong_fields:
            failures.append(f"VLP-16 PointCloud2 fields are invalid: {wrong_fields}")
        point_time_valid = (
            self.point_time_min is not None
            and self.point_time_max is not None
            and self.point_time_min >= -0.002
            and 0.0 < self.point_time_max <= 0.2
            and self.point_time_max >= self.point_time_min
        )
        if not point_time_valid:
            failures.append(
                "VLP-16 point time is not a 0-based scan offset: "
                f"min={self.point_time_min}, max={self.point_time_max}"
            )

        multiple_parents = {
            child: sorted(parents)
            for child, parents in self.tf_parents.items()
            if len(parents) > 1
        }
        if multiple_parents:
            failures.append(f"TF children have multiple parents: {multiple_parents}")

        imu_quality = {
            "acceleration_norm_mps2_median": (
                round(statistics.median(self.imu_acceleration_norms), 4)
                if self.imu_acceleration_norms else None
            ),
            "quaternion_norm_minimum": (
                round(min(self.imu_quaternion_norms), 6)
                if self.imu_quaternion_norms else None
            ),
            "quaternion_norm_maximum": (
                round(max(self.imu_quaternion_norms), 6)
                if self.imu_quaternion_norms else None
            ),
            "covariance_valid_fraction": (
                round(statistics.mean(self.imu_covariance_valid), 4)
                if self.imu_covariance_valid else None
            ),
        }
        if self.imu_quaternion_norms and max(
            abs(value - 1.0) for value in self.imu_quaternion_norms
        ) > 0.05:
            failures.append("IMU quaternion norm differs from one by more than 0.05")
        if imu_quality["covariance_valid_fraction"] != 1.0:
            failures.append("IMU covariance is missing or invalid")

        wheel_covariance = {
            "pose_valid_fraction": (
                round(statistics.mean(self.wheel_pose_covariance_valid), 4)
                if self.wheel_pose_covariance_valid else None
            ),
            "twist_valid_fraction": (
                round(statistics.mean(self.wheel_twist_covariance_valid), 4)
                if self.wheel_twist_covariance_valid else None
            ),
        }
        if wheel_covariance["pose_valid_fraction"] != 1.0:
            failures.append("wheel odometry pose covariance is missing")
        if wheel_covariance["twist_valid_fraction"] != 1.0:
            failures.append("wheel odometry twist covariance is missing")

        lidar_stamps = sorted(self.stamps["/velodyne_points"])
        imu_stamps = np.asarray(sorted(self.stamps["/imu/data"]), dtype=np.float64)
        nearest_absolute = []
        if lidar_stamps and len(imu_stamps):
            for stamp in lidar_stamps:
                index = int(np.searchsorted(imu_stamps, stamp))
                candidates = []
                if index < len(imu_stamps):
                    candidates.append(abs(float(imu_stamps[index]) - stamp))
                if index > 0:
                    candidates.append(abs(float(imu_stamps[index - 1]) - stamp))
                nearest_absolute.append(min(candidates))
        nearest_p95 = (
            float(np.percentile(nearest_absolute, 95))
            if nearest_absolute else None
        )
        if nearest_p95 is not None and nearest_p95 > 0.020:
            failures.append("LiDAR/IMU nearest timestamp difference p95 exceeds 20 ms")

        publisher_counts = {
            topic: len(self.get_publishers_info_by_topic(topic))
            for topic in ["/velodyne_points", "/imu/data", "/wheel/odom", "/tf"]
        }
        for topic in ("/velodyne_points", "/imu/data", "/wheel/odom"):
            if publisher_counts[topic] != 1:
                failures.append(
                    f"{topic} must have exactly one publisher, got {publisher_counts[topic]}"
                )
        self.report = {
            "schema_version": 2,
            "result": "PASS" if not failures else "FAIL",
            "failures": failures,
            "topics": topics,
            "velodyne_fields": self.point_fields,
            "velodyne_point_time_sec": {
                "minimum": self.point_time_min,
                "maximum": self.point_time_max,
                "valid": point_time_valid,
            },
            "publisher_counts": publisher_counts,
            "imu_quality": imu_quality,
            "wheel_odometry_covariance": wheel_covariance,
            "lidar_imu_nearest_timestamp_p95_ms": (
                round(nearest_p95 * 1000.0, 3)
                if nearest_p95 is not None else None
            ),
            "tf_parents": {
                child: sorted(parents) for child, parents in sorted(self.tf_parents.items())
            },
        }
        rendered = yaml.safe_dump(self.report, sort_keys=False, allow_unicode=True)
        print("[SENSOR-CONTRACT]\n" + rendered, flush=True)
        output = str(self.get_parameter("output").value).strip()
        if output:
            path = Path(output).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(rendered, encoding="utf-8")


def main():
    rclpy.init()
    node = LiveSensorContract()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.report and node.report["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
