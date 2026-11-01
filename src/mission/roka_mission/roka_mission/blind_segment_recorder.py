#!/usr/bin/env python3
"""Record only Xbox velocity and attitude for a short fog straightaway."""

import math
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
import yaml

from .blind_profile import route_length, wrap_angle


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


class BlindSegmentRecorder(Node):
    def __init__(self):
        super().__init__("blind_segment_recorder")
        self.declare_parameter("output_file", "")
        self.declare_parameter("route_profile_file", "")
        self.declare_parameter("exit_progress_ratio", 0.30)
        self.declare_parameter("distance_odom_topic", "/wheel/odom")
        self.declare_parameter("maximum_input_age_sec", 0.50)
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("maximum_duration_sec", 60.0)
        filename = str(self.get_parameter("output_file").value)
        if not filename:
            raise ValueError("output_file is required")
        self.output_file = Path(filename).expanduser().resolve()
        route_filename = str(self.get_parameter("route_profile_file").value)
        if not route_filename:
            raise ValueError("route_profile_file is required")
        self.route_profile_file = Path(route_filename).expanduser().resolve()
        route_document = yaml.safe_load(
            self.route_profile_file.read_text(encoding="utf-8")
        ) or {}
        self.route_points = list(route_document.get("points", []))
        self.route_length_m = route_length(self.route_points)
        self.exit_ratio = float(self.get_parameter("exit_progress_ratio").value)
        self.distance_odom_topic = str(
            self.get_parameter("distance_odom_topic").value
        ).strip()
        self.maximum_input_age = float(
            self.get_parameter("maximum_input_age_sec").value
        )
        self.sample_rate = float(self.get_parameter("sample_rate_hz").value)
        self.maximum_duration = float(self.get_parameter("maximum_duration_sec").value)
        if not any(
            math.isclose(self.exit_ratio, choice, abs_tol=1.0e-9)
            for choice in (0.20, 0.30, 0.40)
        ) or not self.distance_odom_topic or self.maximum_input_age <= 0.0 or self.sample_rate <= 0.0:
            raise ValueError("invalid blind segment recording parameters")
        self.target_distance_m = self.route_length_m * self.exit_ratio

        self.recording = False
        self.buttons_armed = False
        self.previous_buttons = []
        self.latest_twist = Twist()
        self.latest_yaw = None
        self.imu_received_nanoseconds = None
        self.start_yaw = None
        self.latest_xy = None
        self.odom_received_nanoseconds = None
        self.last_xy = None
        self.distance = 0.0
        self.started = None
        self.samples = []
        self.command_pub = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self.create_subscription(Twist, "/cmd_vel_joy_raw", self._twist, 20)
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry,
            self.distance_odom_topic,
            self._odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(Joy, "/joy", self._joy, qos_profile_sensor_data)
        self.create_timer(1.0 / self.sample_rate, self._sample)
        self.get_logger().info(
            "[MISSION-1] start-origin fog recorder ready: "
            f"exit={self.exit_ratio:.0%}, wheel target={self.target_distance_m:.2f}m; "
            "release buttons, A=start, LB+stick=drive, automatic save at target"
        )

    def _twist(self, message):
        self.latest_twist = message
        self.command_pub.publish(message if self.recording else Twist())

    def _imu(self, message):
        self.latest_yaw = yaw_from_quaternion(message.orientation)
        self.imu_received_nanoseconds = self.get_clock().now().nanoseconds

    def _odom(self, message):
        self.latest_xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.recording and self.last_xy is not None:
            self.distance += math.hypot(
                self.latest_xy[0] - self.last_xy[0], self.latest_xy[1] - self.last_xy[1]
            )
        self.last_xy = self.latest_xy
        self.odom_received_nanoseconds = self.get_clock().now().nanoseconds

    def _input_age(self, received_nanoseconds):
        if received_nanoseconds is None:
            return math.inf
        return (
            self.get_clock().now().nanoseconds - received_nanoseconds
        ) / 1.0e9

    def _inputs_are_fresh(self):
        return (
            self._input_age(self.imu_received_nanoseconds) <= self.maximum_input_age
            and self._input_age(self.odom_received_nanoseconds) <= self.maximum_input_age
        )

    def _pressed(self, buttons, index):
        previous = self.previous_buttons[index] if index < len(self.previous_buttons) else 0
        current = buttons[index] if index < len(buttons) else 0
        return bool(current and not previous)

    def _joy(self, message):
        buttons = list(message.buttons)
        if not self.buttons_armed:
            self.buttons_armed = not any(buttons)
            self.previous_buttons = buttons
            return
        if self._pressed(buttons, 0) and not self.recording:
            self._start()
        elif self._pressed(buttons, 1) and self.recording:
            self._save(manual=True)
        self.previous_buttons = buttons

    def _start(self):
        if self.latest_yaw is None or self.latest_xy is None or not self._inputs_are_fresh():
            self.get_logger().error(
                "[MISSION-1] A ignored: fresh IMU/wheel odometry is not ready"
            )
            return
        self.recording = True
        self.started = self.get_clock().now()
        self.start_yaw = self.latest_yaw
        self.last_xy = self.latest_xy
        self.distance = 0.0
        self.samples = []
        self.get_logger().info("[MISSION-1] fog straight recording STARTED")

    def _elapsed(self):
        return (self.get_clock().now() - self.started).nanoseconds / 1.0e9

    def _sample(self):
        if not self.recording:
            return
        if not self._inputs_are_fresh():
            self.recording = False
            self.samples = []
            self.command_pub.publish(Twist())
            self.get_logger().error(
                "[MISSION-1] recording aborted: IMU/wheel odometry became stale"
            )
            return
        elapsed = self._elapsed()
        if elapsed >= self.maximum_duration:
            self.recording = False
            self.samples = []
            self.command_pub.publish(Twist())
            self.get_logger().error(
                "[MISSION-1] recording aborted: target distance was not reached in time"
            )
            return
        yaw_delta = 0.0 if self.latest_yaw is None else wrap_angle(
            self.latest_yaw - self.start_yaw
        )
        self.samples.append({
            "elapsed_sec": round(elapsed, 4),
            "linear_mps": round(float(self.latest_twist.linear.x), 5),
            "angular_radps": round(float(self.latest_twist.angular.z), 5),
            "yaw_delta_rad": round(yaw_delta, 6),
        })
        if self.distance >= self.target_distance_m:
            self._save(manual=False)

    def _save(self, *, manual):
        if manual and self.distance < 0.95 * self.target_distance_m:
            self.get_logger().warning(
                "[MISSION-1] B ignored: wheel distance "
                f"{self.distance:.2f}/{self.target_distance_m:.2f}m; continue driving"
            )
            return
        self.recording = False
        self.command_pub.publish(Twist())
        duration = self._elapsed()
        if duration < 0.5 or len(self.samples) < 5:
            self.get_logger().error("[MISSION-1] recording discarded: segment is too short")
            return
        document = {
            "format": "roka_blind_v2",
            "purpose": "section_1_start_origin_fog_command_replay",
            "entry_progress_ratio": 0.0,
            "exit_progress_ratio": self.exit_ratio,
            "route_length_m": round(self.route_length_m, 4),
            "target_distance_m": round(self.target_distance_m, 4),
            "duration_sec": round(duration, 4),
            "distance_m": round(self.distance, 4),
            "sample_rate_hz": self.sample_rate,
            "preserves_recorded_stops": True,
            "commands": self.samples,
        }
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_file.with_suffix(self.output_file.suffix + ".tmp")
        temporary.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        temporary.replace(self.output_file)
        self.get_logger().info(
            f"[MISSION-1] fog profile SAVED: {self.output_file}; "
            f"exit={self.exit_ratio:.0%}, {duration:.2f}s, {self.distance:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BlindSegmentRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
