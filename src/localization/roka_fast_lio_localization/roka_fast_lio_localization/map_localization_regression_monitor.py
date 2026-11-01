#!/usr/bin/env python3
"""Record a deterministic result for fixed-PCD localization bag replay."""

import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
import yaml


class MapLocalizationRegressionMonitor(Node):
    def __init__(self):
        super().__init__("map_localization_regression_monitor")
        self.declare_parameter("output_file", "/tmp/map_localization_regression.yaml")
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("minimum_pose_count", 80)
        self.declare_parameter("minimum_observation_sec", 20.0)
        self.started = time.monotonic()
        self.finished = False
        self.exit_success = False
        self.ready = False
        self.status = "waiting"
        self.poses = []
        self.first_pose_wall = None
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, "/map_localization/ready", self._ready, state_qos)
        self.create_subscription(String, "/map_localization/status", self._status, state_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/map_localization/pose", self._pose, 10
        )
        self.create_timer(0.25, self._tick)

    def _ready(self, message):
        self.ready = bool(message.data)

    def _status(self, message):
        self.status = message.data

    def _pose(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        self.poses.append((stamp, p.x, p.y, p.z, yaw))
        if self.first_pose_wall is None:
            self.first_pose_wall = time.monotonic()

    def _finish(self, passed, reason):
        if self.finished:
            return
        self.finished = True
        steps = [
            math.dist(first[1:4], second[1:4])
            for first, second in zip(self.poses, self.poses[1:])
        ]
        yaw_steps = [
            abs(math.atan2(math.sin(second[4] - first[4]), math.cos(second[4] - first[4])))
            for first, second in zip(self.poses, self.poses[1:])
        ]
        periods = [
            second[0] - first[0]
            for first, second in zip(self.poses, self.poses[1:])
        ]
        linear_rates = [
            step / period for step, period in zip(steps, periods) if period > 1.0e-4
        ]
        yaw_rates = [
            step / period for step, period in zip(yaw_steps, periods) if period > 1.0e-4
        ]
        passed = (
            passed
            and self.ready
            and len(self.poses) >= int(self.get_parameter("minimum_pose_count").value)
            and max(steps, default=0.0) <= 0.30
            # The localizer intentionally updates at 0.35 s; evaluate motion
            # per timestamp instead of treating normal movement between two
            # registrations as an instantaneous pose jump.
            and max(linear_rates, default=0.0) <= 1.50
            and max(yaw_rates, default=0.0) <= 2.50
        )
        report = {
            "schema_version": 1,
            "result": "PASS" if passed else "FAIL",
            "ready": self.ready,
            "status": self.status,
            "pose_count": len(self.poses),
            "maximum_step_m": max(steps, default=0.0),
            "maximum_yaw_step_rad": max(yaw_steps, default=0.0),
            "maximum_linear_rate_mps": max(linear_rates, default=0.0),
            "maximum_yaw_rate_radps": max(yaw_rates, default=0.0),
            "reason": reason,
            "observation_sec": (
                time.monotonic() - self.first_pose_wall
                if self.first_pose_wall is not None else 0.0
            ),
        }
        output = Path(str(self.get_parameter("output_file").value)).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        print("[MAP-LOCALIZATION-REGRESSION]\n" + yaml.safe_dump(report, sort_keys=False), flush=True)
        self.exit_success = passed

    def _tick(self):
        minimum_count = int(self.get_parameter("minimum_pose_count").value)
        minimum_duration = float(
            self.get_parameter("minimum_observation_sec").value
        )
        observed_for = (
            time.monotonic() - self.first_pose_wall
            if self.first_pose_wall is not None else 0.0
        )
        if self.ready and len(self.poses) >= minimum_count and observed_for >= minimum_duration:
            self._finish(
                True,
                f"{len(self.poses)} localized poses remained stable for {observed_for:.1f}s",
            )
        elif time.monotonic() - self.started > float(
            self.get_parameter("timeout_sec").value
        ):
            self._finish(False, "localization timeout")


def main():
    rclpy.init()
    node = MapLocalizationRegressionMonitor()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.exit_success else 1)


if __name__ == "__main__":
    main()
