"""Ground-truth pure-pursuit controller for hardware-free PK-3A testing."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import yaml

from .safe_path import GroundTruthGrid, _candidate_metrics


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_rpy(quaternion: object) -> tuple[float, float, float]:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def pure_pursuit_angular_velocity(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
    linear_speed: float,
    maximum_angular_speed: float,
) -> float:
    dx = target_x - x
    dy = target_y - y
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_squared = max(dx * dx + dy * dy, 1.0e-6)
    curvature = 2.0 * local_y / distance_squared
    return max(
        -maximum_angular_speed,
        min(maximum_angular_speed, linear_speed * curvature),
    )


def lateral_motion_speed_limit(
    nominal_speed: float,
    lateral_speed: float,
    soft_limit: float,
    hard_limit: float,
    crawl_speed: float,
) -> float:
    """Reduce drive speed before sustained lateral motion becomes unsafe."""
    magnitude = abs(lateral_speed)
    if magnitude <= soft_limit:
        return nominal_speed
    ratio = min(1.0, (magnitude - soft_limit) / (hard_limit - soft_limit))
    return max(crawl_speed, nominal_speed * (1.0 - 0.75 * ratio))


def terrain_relative_roll_error_deg(roll_rad: float, cross_slope_deg: float) -> float:
    """Return chassis roll relative to the locally banked wheel plane."""
    return math.degrees(roll_rad) - cross_slope_deg


class SafePathController(Node):
    def __init__(self) -> None:
        super().__init__("sim_safe_path_controller")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("ground_truth_map_file", "")
        self.declare_parameter("ground_truth_metadata_file", "")
        self.declare_parameter("odom_topic", "/simulator/floating_base_state")
        self.declare_parameter("cmd_vel_topic", "/sim/cmd_vel_nav")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("target_laps", 0)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("lookahead_distance_m", 0.35)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("maximum_linear_speed_mps", 0.30)
        self.declare_parameter("maximum_angular_speed_radps", 0.45)
        self.declare_parameter("linear_acceleration_mps2", 0.18)
        self.declare_parameter("linear_deceleration_mps2", 0.30)
        self.declare_parameter("maximum_cross_track_error_m", 0.20)
        self.declare_parameter("required_wheel_edge_m", 0.0841)
        self.declare_parameter("maximum_roll_deg", 65.0)
        self.declare_parameter("maximum_roll_error_deg", 15.0)
        self.declare_parameter("maximum_pitch_deg", 38.0)
        self.declare_parameter("lateral_speed_soft_limit_mps", 0.04)
        self.declare_parameter("maximum_lateral_speed_mps", 0.10)
        self.declare_parameter("lateral_motion_hold_sec", 0.35)
        self.declare_parameter("minimum_crawl_speed_mps", 0.04)
        self.declare_parameter("slip_ratio", 0.20)
        self.declare_parameter("slip_hold_sec", 1.50)
        self.declare_parameter("odom_timeout_sec", 0.30)

        profile_path = Path(str(self.get_parameter("profile_file").value)).expanduser()
        map_path = Path(str(self.get_parameter("ground_truth_map_file").value)).expanduser()
        metadata_path = Path(
            str(self.get_parameter("ground_truth_metadata_file").value)
        ).expanduser()
        if not profile_path.is_file():
            raise FileNotFoundError(f"safe path profile not found: {profile_path}")
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        self.points = list(document.get("points", []))
        if not document.get("closed_loop") or len(self.points) < 10:
            raise ValueError("safe path must be a closed loop with at least 10 points")
        self.path_xy = np.asarray(
            [[float(point["x"]), float(point["y"])] for point in self.points],
            dtype=float,
        )
        self.grid = GroundTruthGrid(map_path, metadata_path)
        self.lookahead = float(self.get_parameter("lookahead_distance_m").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.maximum_linear = float(self.get_parameter("maximum_linear_speed_mps").value)
        self.maximum_angular = float(self.get_parameter("maximum_angular_speed_radps").value)
        self.acceleration = float(self.get_parameter("linear_acceleration_mps2").value)
        self.deceleration = float(self.get_parameter("linear_deceleration_mps2").value)
        self.maximum_cross_track = float(
            self.get_parameter("maximum_cross_track_error_m").value
        )
        self.required_wheel_edge = float(
            self.get_parameter("required_wheel_edge_m").value
        )
        self.maximum_roll = math.radians(float(self.get_parameter("maximum_roll_deg").value))
        self.maximum_roll_error = float(
            self.get_parameter("maximum_roll_error_deg").value
        )
        self.maximum_pitch = math.radians(float(self.get_parameter("maximum_pitch_deg").value))
        self.lateral_speed_soft_limit = float(
            self.get_parameter("lateral_speed_soft_limit_mps").value
        )
        self.maximum_lateral_speed = float(
            self.get_parameter("maximum_lateral_speed_mps").value
        )
        self.lateral_motion_hold = float(
            self.get_parameter("lateral_motion_hold_sec").value
        )
        self.minimum_crawl_speed = float(
            self.get_parameter("minimum_crawl_speed_mps").value
        )
        self.slip_ratio = float(self.get_parameter("slip_ratio").value)
        self.slip_hold = float(self.get_parameter("slip_hold_sec").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.target_laps = int(self.get_parameter("target_laps").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        if min(
            rate,
            self.lookahead,
            self.speed_scale,
            self.maximum_linear,
            self.maximum_angular,
            self.acceleration,
            self.deceleration,
            self.maximum_cross_track,
            self.required_wheel_edge,
            self.odom_timeout,
            self.lateral_speed_soft_limit,
            self.lateral_motion_hold,
            self.minimum_crawl_speed,
            self.maximum_roll_error,
        ) <= 0.0:
            raise ValueError("safe controller parameters must be positive")
        if self.lateral_speed_soft_limit >= self.maximum_lateral_speed:
            raise ValueError("lateral soft limit must be below the hard limit")

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_publisher = self.create_publisher(
            PathMessage, "/track/safe_path", path_qos
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        self.create_service(Trigger, "~/start", self.start_callback)
        self.create_service(Trigger, "~/stop", self.stop_callback)
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.period = 1.0 / rate
        self.active = bool(self.get_parameter("auto_start").value)
        self.fault_reason = ""
        self.odom: Odometry | None = None
        self.last_odom_time = None
        self.current_index: int | None = None
        self.previous_index: int | None = None
        self.completed_laps = 0
        self.commanded_speed = 0.0
        self.slip_since = None
        self.lateral_motion_since = None
        self.publish_path()
        self.get_logger().info(
            f"safe path controller: {len(self.points)} points, auto_start={self.active}, "
            f"target_laps={self.target_laps}"
        )

    def publish_path(self) -> None:
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for point in self.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.position.z = float(point.get("elevation_m", 0.0))
            yaw = float(point["yaw"])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def odom_callback(self, message: Odometry) -> None:
        self.odom = message
        self.last_odom_time = self.get_clock().now()

    def start_callback(self, _request: object, response: object) -> object:
        self.fault_reason = ""
        self.active = True
        self.slip_since = None
        self.lateral_motion_since = None
        response.success = True
        response.message = "MuJoCo safe path enabled"
        return response

    def stop_callback(self, _request: object, response: object) -> object:
        self.active = False
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())
        response.success = True
        response.message = "MuJoCo safe path stopped"
        return response

    def nearest_index(self, x: float, y: float) -> tuple[int, float]:
        if self.current_index is None:
            differences = self.path_xy - np.array([x, y])
            distances = np.einsum("ij,ij->i", differences, differences)
            index = int(np.argmin(distances))
            return index, float(math.sqrt(distances[index]))
        count = len(self.points)
        indices = np.asarray(
            [(self.current_index + offset) % count for offset in range(-12, 61)],
            dtype=int,
        )
        differences = self.path_xy[indices] - np.array([x, y])
        distances = np.einsum("ij,ij->i", differences, differences)
        selected = int(np.argmin(distances))
        return int(indices[selected]), float(math.sqrt(distances[selected]))

    def target_index(self, start: int) -> int:
        count = len(self.points)
        distance = 0.0
        index = start
        while distance < self.lookahead:
            following = (index + 1) % count
            distance += float(np.linalg.norm(self.path_xy[following] - self.path_xy[index]))
            index = following
            if index == start:
                break
        return index

    def set_fault(self, reason: str) -> None:
        if not self.fault_reason:
            self.get_logger().error(f"safe autonomy stopped: {reason}")
        self.fault_reason = reason
        self.active = False
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())

    def update(self) -> None:
        if self.odom is None or self.last_odom_time is None:
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1.0e-9
        if age > self.odom_timeout:
            self.set_fault(f"floating-base odometry timeout ({age:.2f}s)")
            return
        if not self.active:
            self.command_publisher.publish(Twist())
            return

        position = self.odom.pose.pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.odom.pose.pose.orientation)
        index, cross_track = self.nearest_index(float(position.x), float(position.y))
        if cross_track > self.maximum_cross_track:
            self.set_fault(f"cross-track error {cross_track:.3f}m")
            return
        if abs(roll) > self.maximum_roll:
            self.set_fault(f"absolute roll {math.degrees(roll):.1f}deg")
            return
        if abs(pitch) > self.maximum_pitch:
            self.set_fault(f"pitch {math.degrees(pitch):.1f}deg")
            return
        terrain = _candidate_metrics(
            self.grid,
            float(position.x),
            float(position.y),
            yaw,
            self.required_wheel_edge,
        )
        if terrain is None:
            self.set_fault("one or more wheels left the rideable safety corridor")
            return
        roll_error = terrain_relative_roll_error_deg(
            roll, float(terrain["cross_slope_deg"])
        )
        if abs(roll_error) > self.maximum_roll_error:
            self.set_fault(
                f"terrain-relative roll error {roll_error:.1f}deg "
                f"(body={math.degrees(roll):.1f}deg, "
                f"terrain={terrain['cross_slope_deg']:.1f}deg)"
            )
            return

        velocity = self.odom.twist.twist.linear
        forward_speed = math.cos(yaw) * float(velocity.x) + math.sin(yaw) * float(velocity.y)
        lateral_speed = -math.sin(yaw) * float(velocity.x) + math.cos(yaw) * float(velocity.y)
        now = self.get_clock().now()
        if abs(lateral_speed) > self.maximum_lateral_speed:
            if self.lateral_motion_since is None:
                self.lateral_motion_since = now
            elif (
                (now - self.lateral_motion_since).nanoseconds * 1.0e-9
                >= self.lateral_motion_hold
            ):
                self.set_fault(
                    f"sustained lateral motion {lateral_speed:.3f}m/s for "
                    f"{self.lateral_motion_hold:.2f}s"
                )
                return
        else:
            self.lateral_motion_since = None
        if self.commanded_speed > 0.07 and abs(forward_speed) < self.slip_ratio * self.commanded_speed:
            if self.slip_since is None:
                self.slip_since = now
            elif (now - self.slip_since).nanoseconds * 1.0e-9 >= self.slip_hold:
                self.set_fault(
                    f"longitudinal slip/stall: command={self.commanded_speed:.3f}, "
                    f"measured={forward_speed:.3f}m/s"
                )
                return
        else:
            self.slip_since = None

        if self.previous_index is not None:
            count = len(self.points)
            if self.previous_index > int(0.85 * count) and index < int(0.15 * count):
                self.completed_laps += 1
                self.get_logger().info(f"completed safe lap {self.completed_laps}")
                if self.target_laps > 0 and self.completed_laps >= self.target_laps:
                    self.active = False
                    self.commanded_speed = 0.0
                    self.command_publisher.publish(Twist())
                    return
        self.previous_index = index
        self.current_index = index
        target = self.points[self.target_index(index)]
        desired_speed = min(
            self.maximum_linear,
            self.speed_scale
            * float(self.points[index].get("speed_mps", self.maximum_linear)),
        )
        heading_error = abs(wrap_angle(float(target["yaw"]) - yaw))
        if heading_error > 0.55:
            desired_speed = min(desired_speed, 0.07)
        desired_speed = lateral_motion_speed_limit(
            desired_speed,
            lateral_speed,
            self.lateral_speed_soft_limit,
            self.maximum_lateral_speed,
            self.minimum_crawl_speed,
        )
        speed_delta = desired_speed - self.commanded_speed
        limit = (self.acceleration if speed_delta >= 0.0 else self.deceleration) * self.period
        self.commanded_speed += max(-limit, min(limit, speed_delta))
        angular = pure_pursuit_angular_velocity(
            float(position.x),
            float(position.y),
            yaw,
            float(target["x"]),
            float(target["y"]),
            self.commanded_speed,
            self.maximum_angular,
        )
        command = Twist()
        command.linear.x = self.commanded_speed
        command.angular.z = angular
        self.command_publisher.publish(command)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafePathController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # ROS 2 Humble can invalidate the subscription context while launch is
        # delivering SIGINT.  This is a shutdown race, not a controller fault.
        pass
    finally:
        if rclpy.ok():
            try:
                node.command_publisher.publish(Twist())
            except (RuntimeError, rclpy.exceptions.RCLError):
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
