#!/usr/bin/env python3
"""Finalize an offline FAST-LIO/SC-PGO replay and write deterministic metrics."""

import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
import yaml

from .archive_format import validate_archive_pair


def _yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
    return math.atan2(siny, cosy)


def _angle_difference(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


class LioRegressionMonitor(Node):
    def __init__(self):
        super().__init__("lio_regression_monitor")
        self.declare_parameter("output_file", "/tmp/lio_regression.yaml")
        self.declare_parameter("idle_timeout_sec", 2.0)
        self.declare_parameter("startup_timeout_sec", 45.0)
        self.declare_parameter("maximum_step_m", 0.10)
        self.declare_parameter("maximum_yaw_step_deg", 8.0)
        self.declare_parameter("maximum_z_span_m", 0.15)
        self.declare_parameter("validation_context", "unspecified")
        self.start_wall = time.monotonic()
        self.last_odom_wall = None
        self.poses = []
        self.cloud_count = 0
        self.latest_pgo_path = None
        self.future = None
        self.finished = False
        self.exit_success = False
        self.create_subscription(
            Odometry, "/lio/odom", self._odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/lio/cloud_body", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(NavPath, "/pgo/path", self._path, 10)
        self.client = self.create_client(Trigger, "/pgo/save_map")
        self.create_timer(0.25, self._tick)

    def _odom(self, message):
        position = message.pose.pose.position
        self.poses.append((
            position.x,
            position.y,
            position.z,
            _yaw(message.pose.pose.orientation),
        ))
        self.last_odom_wall = time.monotonic()

    def _cloud(self, _message):
        self.cloud_count += 1

    def _path(self, message):
        self.latest_pgo_path = message

    def _archive_metrics(self):
        directory = Path(
            str(self.get_parameter("output_file").value)
        ).expanduser().resolve().parent
        return validate_archive_pair(directory)

    def _metrics(self, save_success, save_message):
        steps = [
            math.dist(first[:3], second[:3])
            for first, second in zip(self.poses, self.poses[1:])
        ]
        turns = [
            abs(_angle_difference(first[3], second[3]))
            for first, second in zip(self.poses, self.poses[1:])
        ]
        report = {
            "schema_version": 1,
            "result": "PASS" if save_success and len(self.poses) >= 10 else "FAIL",
            "validation": {
                "context": str(self.get_parameter("validation_context").value),
                "maximum_step_m": float(self.get_parameter("maximum_step_m").value),
                "maximum_yaw_step_deg": float(
                    self.get_parameter("maximum_yaw_step_deg").value
                ),
                "maximum_z_span_m": float(
                    self.get_parameter("maximum_z_span_m").value
                ),
            },
            "lio": {
                "odom_count": len(self.poses),
                "cloud_count": self.cloud_count,
                "path_length_m": sum(steps),
                "maximum_step_m": max(steps, default=0.0),
                "maximum_yaw_step_rad": max(turns, default=0.0),
                "z_span_m": (
                    max(pose[2] for pose in self.poses) - min(pose[2] for pose in self.poses)
                    if self.poses
                    else 0.0
                ),
                "displacement_m": (
                    math.dist(self.poses[0][:3], self.poses[-1][:3])
                    if self.poses
                    else 0.0
                ),
            },
            "pgo": {
                "path_pose_count": (
                    len(self.latest_pgo_path.poses) if self.latest_pgo_path else 0
                ),
                "save_success": bool(save_success),
                "save_message": save_message,
            },
        }
        try:
            report["pgo"]["archives"] = self._archive_metrics()
        except (OSError, ValueError) as error:
            report["result"] = "FAIL"
            report.setdefault("failures", []).append(str(error))
        maximum_step = float(self.get_parameter("maximum_step_m").value)
        maximum_yaw_step = math.radians(
            float(self.get_parameter("maximum_yaw_step_deg").value)
        )
        maximum_z_span = float(self.get_parameter("maximum_z_span_m").value)
        if report["lio"]["maximum_step_m"] > maximum_step:
            report["result"] = "FAIL"
            report.setdefault("failures", []).append(
                f"FAST-LIO pose step exceeds {maximum_step:.3f} m"
            )
        if report["lio"]["maximum_yaw_step_rad"] > maximum_yaw_step:
            report["result"] = "FAIL"
            report.setdefault("failures", []).append(
                "FAST-LIO yaw step exceeds "
                f"{math.degrees(maximum_yaw_step):.1f} degrees"
            )
        if report["lio"]["z_span_m"] > maximum_z_span:
            report["result"] = "FAIL"
            report.setdefault("failures", []).append(
                f"FAST-LIO Z span exceeds {maximum_z_span:.3f} m"
            )
        return report

    def _finish(self, save_success, save_message):
        if self.finished:
            return
        self.finished = True
        report = self._metrics(save_success, save_message)
        output = Path(str(self.get_parameter("output_file").value)).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        print("[LIO-REGRESSION]\n" + yaml.safe_dump(report, sort_keys=False), flush=True)
        self.exit_success = report["result"] == "PASS"

    def _tick(self):
        if self.finished:
            return
        now = time.monotonic()
        startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        if self.last_odom_wall is None:
            if now - self.start_wall > startup_timeout:
                self._finish(False, "no FAST-LIO odometry before startup timeout")
            return
        if self.future is None:
            idle_timeout = float(self.get_parameter("idle_timeout_sec").value)
            if now - self.last_odom_wall < idle_timeout:
                return
            if not self.client.wait_for_service(timeout_sec=0.1):
                if now - self.last_odom_wall > idle_timeout + 10.0:
                    self._finish(False, "SC-PGO save service unavailable")
                return
            self.future = self.client.call_async(Trigger.Request())
            return
        if self.future.done():
            result = self.future.result()
            if result is None:
                self._finish(False, "SC-PGO save service returned no response")
            else:
                self._finish(result.success, result.message)


def main():
    rclpy.init()
    node = LioRegressionMonitor()
    try:
        # Calling rclpy.shutdown() from a timer callback can wait on the
        # callback that invoked it.  Let the main loop observe ``finished``
        # instead so offline regressions terminate deterministically.
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
