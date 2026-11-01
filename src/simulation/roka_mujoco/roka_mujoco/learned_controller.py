"""Run the Xbox behavior-cloning policy in the PK-3A MuJoCo scene."""

from __future__ import annotations

import json
import math
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from roka_robot_description.workspace import workspace_path

from .demonstration_learning import (
    FEATURE_NAMES,
    _path_features,
    _predict,
)
from .safe_path import TIRE_HALF_WIDTH_M, GroundTruthGrid, _candidate_metrics
from .safe_path_controller import quaternion_to_rpy, wrap_angle


def _pure_pursuit_angular(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
    linear_speed: float,
) -> float:
    """Return differential-drive angular speed toward a mapped look-ahead point."""
    dx = target_x - x
    dy = target_y - y
    distance = max(math.hypot(dx, dy), 0.10)
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    return 2.0 * linear_speed * math.sin(alpha) / distance


class LearnedController(Node):
    def __init__(self) -> None:
        super().__init__("sim_learned_controller")
        self.declare_parameter(
            "model_file",
            str(workspace_path("models", "learned", "xbox_behavior_policy_v1.npz")),
        )
        self.declare_parameter(
            "profile_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_safe_path.yaml")),
        )
        self.declare_parameter(
            "ground_truth_map_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.npz")),
        )
        self.declare_parameter(
            "ground_truth_metadata_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.json")),
        )
        self.declare_parameter("odom_topic", "/simulator/floating_base_state")
        self.declare_parameter("cmd_vel_topic", "/sim/cmd_vel_nav")
        self.declare_parameter("status_topic", "/sim/learned_controller/status")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("target_laps", 0)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("minimum_linear_speed_mps", 0.12)
        self.declare_parameter("maximum_linear_speed_mps", 0.30)
        self.declare_parameter("maximum_angular_speed_radps", 0.60)
        self.declare_parameter("linear_acceleration_mps2", 0.40)
        self.declare_parameter("linear_deceleration_mps2", 0.60)
        self.declare_parameter("angular_acceleration_radps2", 1.50)
        self.declare_parameter("lateral_reference_gain_radps_per_m", 2.40)
        self.declare_parameter("heading_reference_gain_radps_per_rad", 1.20)
        self.declare_parameter("minimum_tracking_speed_scale", 0.70)
        self.declare_parameter("minimum_tire_clearance_m", 0.0)
        self.declare_parameter("recovery_enter_lateral_error_m", 0.12)
        self.declare_parameter("recovery_exit_lateral_error_m", 0.06)
        self.declare_parameter("recovery_enter_heading_error_deg", 25.0)
        self.declare_parameter("recovery_exit_heading_error_deg", 12.0)
        self.declare_parameter("recovery_lookahead_m", 0.55)
        self.declare_parameter("recovery_linear_speed_mps", 0.16)
        self.declare_parameter("maximum_recovery_lateral_error_m", 0.55)
        self.declare_parameter("minimum_reference_height_delta_m", -0.06)
        self.declare_parameter("absolute_roll_limit_deg", 85.0)
        self.declare_parameter("absolute_pitch_limit_deg", 65.0)
        self.declare_parameter("odom_timeout_sec", 0.30)

        model_path = Path(str(self.get_parameter("model_file").value)).expanduser()
        archive = np.load(model_path)
        names = tuple(str(value) for value in archive["feature_names"])
        if names != FEATURE_NAMES:
            raise ValueError("learned model feature schema does not match controller")
        self.parameters = {
            key: archive[key]
            for key in ("w1", "b1", "w2", "b2", "feature_mean", "feature_std")
        }
        self.output_scales = np.asarray(archive["output_scales"], dtype=float)
        self.reference_lateral = np.asarray(
            archive["reference_lateral_offset_m"], dtype=float
        )
        self.reference_heading = np.asarray(
            archive["reference_heading_error_rad"], dtype=float
        )
        profile_path = Path(str(self.get_parameter("profile_file").value)).expanduser()
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        self.points = list(document.get("points", []))
        if not document.get("closed_loop") or not self.points:
            raise ValueError("learned controller requires a closed path profile")
        if (
            self.reference_lateral.shape != (len(self.points),)
            or self.reference_heading.shape != (len(self.points),)
        ):
            raise ValueError("learned model reference path size does not match profile")
        self.path_xy = np.asarray(
            [[float(point["x"]), float(point["y"])] for point in self.points]
        )
        path_yaw = np.asarray([float(point["yaw"]) for point in self.points])
        path_normal = np.column_stack((-np.sin(path_yaw), np.cos(path_yaw)))
        self.reference_xy = self.path_xy + path_normal * self.reference_lateral[:, None]
        segment_lengths = np.linalg.norm(
            np.roll(self.reference_xy, -1, axis=0) - self.reference_xy, axis=1
        )
        self.path_features = _path_features(profile_path)
        self.grid = GroundTruthGrid(
            Path(str(self.get_parameter("ground_truth_map_file").value)),
            Path(str(self.get_parameter("ground_truth_metadata_file").value)),
        )
        self.maximum_linear = float(
            self.get_parameter("maximum_linear_speed_mps").value
        )
        self.minimum_linear = float(
            self.get_parameter("minimum_linear_speed_mps").value
        )
        self.maximum_angular = float(
            self.get_parameter("maximum_angular_speed_radps").value
        )
        self.linear_acceleration = float(
            self.get_parameter("linear_acceleration_mps2").value
        )
        self.linear_deceleration = float(
            self.get_parameter("linear_deceleration_mps2").value
        )
        self.angular_acceleration = float(
            self.get_parameter("angular_acceleration_radps2").value
        )
        self.lateral_reference_gain = float(
            self.get_parameter("lateral_reference_gain_radps_per_m").value
        )
        self.heading_reference_gain = float(
            self.get_parameter("heading_reference_gain_radps_per_rad").value
        )
        self.minimum_tracking_speed_scale = float(
            self.get_parameter("minimum_tracking_speed_scale").value
        )
        self.minimum_tire_clearance = float(
            self.get_parameter("minimum_tire_clearance_m").value
        )
        self.recovery_enter_lateral = float(
            self.get_parameter("recovery_enter_lateral_error_m").value
        )
        self.recovery_exit_lateral = float(
            self.get_parameter("recovery_exit_lateral_error_m").value
        )
        self.recovery_enter_heading = math.radians(float(
            self.get_parameter("recovery_enter_heading_error_deg").value
        ))
        self.recovery_exit_heading = math.radians(float(
            self.get_parameter("recovery_exit_heading_error_deg").value
        ))
        recovery_lookahead = float(
            self.get_parameter("recovery_lookahead_m").value
        )
        self.recovery_linear = float(
            self.get_parameter("recovery_linear_speed_mps").value
        )
        self.maximum_recovery_lateral = float(
            self.get_parameter("maximum_recovery_lateral_error_m").value
        )
        self.minimum_reference_height_delta = float(
            self.get_parameter("minimum_reference_height_delta_m").value
        )
        median_spacing = max(float(np.median(segment_lengths)), 0.01)
        self.recovery_lookahead_steps = max(
            1, int(round(recovery_lookahead / median_spacing))
        )
        self.roll_limit = math.radians(
            float(self.get_parameter("absolute_roll_limit_deg").value)
        )
        self.pitch_limit = math.radians(
            float(self.get_parameter("absolute_pitch_limit_deg").value)
        )
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.target_laps = int(self.get_parameter("target_laps").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        if min(
            rate,
            self.minimum_linear,
            self.maximum_linear,
            self.maximum_angular,
            self.linear_acceleration,
            self.linear_deceleration,
            self.angular_acceleration,
            self.lateral_reference_gain,
            self.heading_reference_gain,
            self.recovery_enter_lateral,
            self.recovery_exit_lateral,
            self.recovery_enter_heading,
            self.recovery_exit_heading,
            recovery_lookahead,
            self.recovery_linear,
            self.maximum_recovery_lateral,
            self.odom_timeout,
        ) <= 0.0:
            raise ValueError("learned controller limits must be positive")
        if self.minimum_linear > self.maximum_linear:
            raise ValueError("minimum linear speed cannot exceed maximum")
        if not (
            self.recovery_exit_lateral < self.recovery_enter_lateral
            < self.maximum_recovery_lateral
        ):
            raise ValueError("recovery lateral thresholds are inconsistent")
        if self.recovery_exit_heading >= self.recovery_enter_heading:
            raise ValueError("recovery heading exit must be below entry")
        if not self.minimum_linear <= self.recovery_linear <= self.maximum_linear:
            raise ValueError("recovery speed must be within linear speed limits")
        if not 0.0 < self.minimum_tracking_speed_scale <= 1.0:
            raise ValueError("minimum_tracking_speed_scale must be in (0, 1]")
        self.period = 1.0 / rate
        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            30,
        )
        self.create_service(Trigger, "~/start", self._start_service)
        self.create_service(Trigger, "~/stop", self._stop_service)
        self.timer = self.create_timer(self.period, self._update)
        self.active = bool(self.get_parameter("auto_start").value)
        self.odom: Odometry | None = None
        self.last_odom_time = None
        self.current_index: int | None = None
        self.previous_index: int | None = None
        self.completed_laps = 0
        self.lap_armed = False
        self.recovery_active = False
        self.recovery_stable_cycles = 0
        self.recovery_count = 0
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.fault_reason = ""
        self.episode_sequence = 1 if self.active else 0
        self.episode_start_sim_sec = self._sim_time_sec()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0
        self.last_reference_lateral_error = 0.0
        self.last_reference_heading_error_deg = 0.0
        self.last_reference_height_delta = 0.0
        self.last_tire_clearance: float | None = None
        self.last_corridor_available = True
        self.get_logger().info(
            f"Xbox behavior policy loaded: {model_path}, auto_start={self.active}"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message
        self.last_odom_time = self.get_clock().now()

    def _sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _reset_episode_state(self) -> None:
        self.current_index = None
        self.previous_index = None
        self.completed_laps = 0
        self.lap_armed = False
        self.recovery_active = False
        self.recovery_stable_cycles = 0
        self.recovery_count = 0
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.fault_reason = ""
        self.episode_sequence += 1
        self.episode_start_sim_sec = self._sim_time_sec()

    def _start_service(self, _request: object, response: object) -> object:
        self._reset_episode_state()
        self.active = True
        response.success = True
        response.message = (
            f"learned MuJoCo controller enabled for episode "
            f"{self.episode_sequence}"
        )
        return response

    def _stop_service(self, _request: object, response: object) -> object:
        self.active = False
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.command_publisher.publish(Twist())
        response.success = True
        response.message = "learned MuJoCo controller stopped"
        return response

    def _fault(self, reason: str) -> None:
        if not self.fault_reason:
            self.get_logger().error(f"learned autonomy stopped: {reason}")
        self.fault_reason = reason
        self.active = False
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.command_publisher.publish(Twist())
        self._publish_status()

    def _publish_status(self) -> None:
        now = self._sim_time_sec()
        index = self.current_index if self.current_index is not None else -1
        progress = 0.0 if index < 0 else 100.0 * index / len(self.points)
        document = {
            "schema_version": 1,
            "sim_time_sec": now,
            "episode_elapsed_sim_sec": max(0.0, now - self.episode_start_sim_sec),
            "episode_sequence": self.episode_sequence,
            "active": self.active,
            "fault_reason": self.fault_reason,
            "completed_laps": self.completed_laps,
            "path_index": index,
            "path_progress_pct": progress,
            "recovery_active": self.recovery_active,
            "recovery_count": self.recovery_count,
            "corridor_available": self.last_corridor_available,
            "x_m": self.last_x,
            "y_m": self.last_y,
            "z_m": self.last_z,
            "roll_deg": self.last_roll_deg,
            "pitch_deg": self.last_pitch_deg,
            "reference_lateral_error_m": self.last_reference_lateral_error,
            "reference_heading_error_deg": self.last_reference_heading_error_deg,
            "reference_height_delta_m": self.last_reference_height_delta,
            "tire_clearance_m": self.last_tire_clearance,
            "command_linear_mps": self.linear_command,
            "command_angular_radps": self.angular_command,
        }
        message = String()
        message.data = json.dumps(document, separators=(",", ":"), allow_nan=False)
        self.status_publisher.publish(message)

    def _nearest_index(self, x: float, y: float) -> int:
        if self.current_index is None:
            candidates = np.arange(len(self.points))
        else:
            count = len(self.points)
            candidates = np.asarray([
                (self.current_index + offset) % count for offset in range(-15, 81)
            ])
        differences = self.reference_xy[candidates] - np.asarray([x, y])
        distances = np.einsum("ij,ij->i", differences, differences)
        return int(candidates[int(np.argmin(distances))])

    @staticmethod
    def _approach(current: float, target: float, up: float, down: float) -> float:
        delta = target - current
        limit = up if delta >= 0.0 else down
        return current + max(-limit, min(limit, delta))

    def _update(self) -> None:
        if self.odom is None or self.last_odom_time is None:
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1.0e-9
        if age > self.odom_timeout:
            self._fault(f"floating-base odometry timeout ({age:.2f}s)")
            return
        if not self.active:
            self.command_publisher.publish(Twist())
            self._publish_status()
            return
        position = self.odom.pose.pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.odom.pose.pose.orientation)
        self.last_x = float(position.x)
        self.last_y = float(position.y)
        self.last_z = float(position.z)
        self.last_roll_deg = math.degrees(roll)
        self.last_pitch_deg = math.degrees(pitch)
        if abs(roll) >= self.roll_limit or abs(pitch) >= self.pitch_limit:
            self._fault(
                f"overturn attitude roll={math.degrees(roll):.1f}deg, "
                f"pitch={math.degrees(pitch):.1f}deg"
            )
            return
        x, y = float(position.x), float(position.y)
        index = self._nearest_index(x, y)
        point = self.points[index]
        path_yaw = float(point["yaw"])
        dx = x - float(point["x"])
        dy = y - float(point["y"])
        lateral_error = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
        heading_error = wrap_angle(yaw - path_yaw)
        velocity = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        forward_speed = math.cos(yaw) * float(velocity.x) + math.sin(yaw) * float(velocity.y)
        lateral_speed = -math.sin(yaw) * float(velocity.x) + math.cos(yaw) * float(velocity.y)
        reference_lateral_error = lateral_error - self.reference_lateral[index]
        reference_heading_error = wrap_angle(
            heading_error - self.reference_heading[index]
        )
        reference_height_delta = float(position.z) - float(point["elevation_m"])
        self.current_index = index
        self.last_reference_lateral_error = reference_lateral_error
        self.last_reference_heading_error_deg = math.degrees(
            reference_heading_error
        )
        self.last_reference_height_delta = reference_height_delta
        if abs(reference_lateral_error) >= self.maximum_recovery_lateral:
            self._fault(
                "robot is outside the mapped-path recovery envelope "
                f"at path={index}/{len(self.points)} "
                f"(lateral error={reference_lateral_error:.3f}m)"
            )
            return
        if reference_height_delta < self.minimum_reference_height_delta:
            self._fault(
                "robot dropped below the mapped track surface "
                f"at path={index}/{len(self.points)} "
                f"(height delta={reference_height_delta:.3f}m)"
            )
            return
        required_edge = TIRE_HALF_WIDTH_M + self.minimum_tire_clearance
        strict_terrain = _candidate_metrics(self.grid, x, y, yaw, required_edge)
        terrain = strict_terrain
        if terrain is None:
            terrain = _candidate_metrics(self.grid, x, y, yaw, 0.0)
        corridor_available = strict_terrain is not None
        self.last_corridor_available = corridor_available
        if terrain is None:
            # The 2.5 cm raster can reject a wheel-centre pixel even while the
            # MuJoCo tyre still visibly contacts the track. Use mapped path
            # terrain for the policy feature and let recovery steer inward.
            actual_cross_slope = float(point["cross_slope_deg"])
            actual_longitudinal_slope = float(point["longitudinal_slope_deg"])
            tire_clearance = self.minimum_tire_clearance - self.grid.resolution
            self.last_tire_clearance = None
        else:
            actual_cross_slope = float(terrain["cross_slope_deg"])
            actual_longitudinal_slope = float(terrain["longitudinal_slope_deg"])
            tire_clearance = (
                float(terrain["minimum_wheel_edge_m"]) - TIRE_HALF_WIDTH_M
            )
            self.last_tire_clearance = tire_clearance
        feature = np.asarray([[
            lateral_error,
            math.sin(heading_error),
            math.cos(heading_error),
            float(point["cross_slope_deg"]),
            float(point["longitudinal_slope_deg"]),
            actual_cross_slope,
            actual_longitudinal_slope,
            math.degrees(roll),
            math.degrees(pitch),
            forward_speed,
            lateral_speed,
            float(angular.z),
            tire_clearance,
            float(point["lateral_offset_m"]),
            float(self.path_features["curvature"][index]),
            float(self.path_features["lookahead_cross"][index]),
        ]])
        normalized = (
            feature - self.parameters["feature_mean"]
        ) / self.parameters["feature_std"]
        learned = _predict(self.parameters, normalized)[0] * self.output_scales
        recovery_requested = (
            not corridor_available
            or abs(reference_lateral_error) >= self.recovery_enter_lateral
            or abs(reference_heading_error) >= self.recovery_enter_heading
        )
        if not self.recovery_active and recovery_requested:
            self.recovery_active = True
            self.recovery_stable_cycles = 0
            self.recovery_count += 1
            self.get_logger().warning(
                "mapped-path recovery engaged "
                f"at path={index}/{len(self.points)} "
                f"(lateral={reference_lateral_error:.3f}m, "
                f"heading={math.degrees(reference_heading_error):.1f}deg, "
                f"corridor_pixel={corridor_available})"
            )
        if self.recovery_active:
            recovered = (
                corridor_available
                and abs(reference_lateral_error) <= self.recovery_exit_lateral
                and abs(reference_heading_error) <= self.recovery_exit_heading
            )
            self.recovery_stable_cycles = (
                self.recovery_stable_cycles + 1 if recovered else 0
            )
            if self.recovery_stable_cycles >= 15:
                self.recovery_active = False
                self.recovery_stable_cycles = 0
                self.get_logger().info(
                    f"mapped-path recovery complete at path={index}/{len(self.points)}"
                )
        if self.recovery_active:
            target_index = (
                index + self.recovery_lookahead_steps
            ) % len(self.points)
            target_x, target_y = self.reference_xy[target_index]
            target_linear = self.recovery_linear
            recovery_angular = _pure_pursuit_angular(
                x,
                y,
                yaw,
                float(target_x),
                float(target_y),
                target_linear,
            ) - 0.50 * reference_heading_error
            target_angular = max(
                -self.maximum_angular,
                min(self.maximum_angular, recovery_angular),
            )
        else:
            tracking_scale = max(
                self.minimum_tracking_speed_scale,
                1.0
                - 1.5 * abs(reference_lateral_error)
                - 0.35 * abs(reference_heading_error),
            )
            target_linear = max(
                self.minimum_linear,
                min(self.maximum_linear, float(learned[0]) * tracking_scale),
            )
            stabilized_angular = (
                float(learned[1])
                - self.lateral_reference_gain * reference_lateral_error
                - self.heading_reference_gain * reference_heading_error
            )
            target_angular = max(
                -self.maximum_angular,
                min(self.maximum_angular, stabilized_angular),
            )
        self.linear_command = self._approach(
            self.linear_command,
            target_linear,
            self.linear_acceleration * self.period,
            self.linear_deceleration * self.period,
        )
        self.angular_command = self._approach(
            self.angular_command,
            target_angular,
            self.angular_acceleration * self.period,
            self.angular_acceleration * self.period,
        )
        if self.previous_index is not None:
            count = len(self.points)
            if int(0.35 * count) <= index <= int(0.65 * count):
                self.lap_armed = True
            if (
                self.lap_armed
                and self.previous_index > int(0.85 * count)
                and index < int(0.15 * count)
            ):
                self.completed_laps += 1
                self.lap_armed = False
                self.get_logger().info(f"learned policy completed lap {self.completed_laps}")
                if self.target_laps > 0 and self.completed_laps >= self.target_laps:
                    self.active = False
                    self.linear_command = 0.0
                    self.angular_command = 0.0
        self.previous_index = index
        self.current_index = index
        command = Twist()
        command.linear.x = self.linear_command
        command.angular.z = self.angular_command
        self.command_publisher.publish(command)
        self._publish_status()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LearnedController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
