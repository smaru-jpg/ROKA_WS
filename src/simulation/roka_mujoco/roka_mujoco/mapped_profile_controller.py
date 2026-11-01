"""Follow a real mapping-session profile with the MuJoCo robot on level ground."""

from __future__ import annotations

import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from roka_robot_description.workspace import workspace_path

from .safe_path_controller import quaternion_to_rpy, wrap_angle


def mapping_profile_in_start_frame(
    points: list[dict[str, object]],
) -> list[dict[str, object]]:
    """Return a copy of map-frame points expressed from the recorded start pose."""
    if len(points) < 2:
        raise ValueError("mapped profile requires at least two points")
    start_x = float(points[0]["x"])
    start_y = float(points[0]["y"])
    start_yaw = float(points[0]["yaw"])
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    transformed: list[dict[str, object]] = []
    for source in points:
        point = dict(source)
        dx = float(source["x"]) - start_x
        dy = float(source["y"]) - start_y
        point["x"] = cosine * dx + sine * dy
        point["y"] = -sine * dx + cosine * dy
        point["yaw"] = wrap_angle(float(source["yaw"]) - start_yaw)
        transformed.append(point)
    return transformed


def open_path_nearest_index(
    path_xy: np.ndarray,
    x: float,
    y: float,
    current_index: int | None,
    backward_window: int = 4,
    forward_window: int = 20,
) -> tuple[int, float]:
    """Find the closest point without wrapping an open path end to its start."""
    if current_index is None:
        candidates = np.arange(len(path_xy), dtype=int)
    else:
        begin = max(0, current_index - backward_window)
        end = min(len(path_xy), current_index + forward_window + 1)
        candidates = np.arange(begin, end, dtype=int)
    differences = path_xy[candidates] - np.asarray([x, y], dtype=float)
    squared = np.einsum("ij,ij->i", differences, differences)
    selected = int(np.argmin(squared))
    return int(candidates[selected]), float(math.sqrt(squared[selected]))


class MappedProfileController(Node):
    """Dynamics test controller for a path recorded by the real mapping stack."""

    def __init__(self) -> None:
        super().__init__("sim_mapped_profile_controller")
        self.declare_parameter(
            "profile_file",
            str(workspace_path("maps", "section_1", "track_profile_test.yaml")),
        )
        self.declare_parameter("odom_topic", "/simulator/floating_base_state")
        self.declare_parameter("cmd_vel_topic", "/sim/cmd_vel_nav")
        self.declare_parameter("status_topic", "/sim/mapped_controller/status")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("lookahead_distance_m", 0.50)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("maximum_linear_speed_mps", 0.30)
        self.declare_parameter("maximum_angular_speed_radps", 0.80)
        self.declare_parameter("linear_acceleration_mps2", 0.25)
        self.declare_parameter("linear_deceleration_mps2", 0.40)
        self.declare_parameter("maximum_cross_track_error_m", 0.28)
        self.declare_parameter("maximum_roll_deg", 30.0)
        self.declare_parameter("maximum_pitch_deg", 30.0)
        self.declare_parameter("goal_tolerance_m", 0.14)
        self.declare_parameter("odom_timeout_sec", 0.50)

        profile_path = Path(str(self.get_parameter("profile_file").value)).expanduser()
        if not profile_path.is_file():
            raise FileNotFoundError(f"mapped profile not found: {profile_path}")
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        if bool(document.get("closed_loop", False)):
            raise ValueError("mapped-profile test expects an open field path")
        if not bool(document.get("autonomy_approved", False)):
            raise ValueError("mapped profile is not approved for controlled autonomy")
        self.points = mapping_profile_in_start_frame(list(document.get("points", [])))
        self.path_xy = np.asarray(
            [[float(point["x"]), float(point["y"])] for point in self.points],
            dtype=float,
        )
        self.segment_lengths = np.linalg.norm(np.diff(self.path_xy, axis=0), axis=1)
        self.cumulative_length = np.concatenate(
            (np.asarray([0.0]), np.cumsum(self.segment_lengths))
        )
        self.path_length = float(self.cumulative_length[-1])
        if self.path_length < 0.50:
            raise ValueError("mapped path is too short for a dynamics test")

        self.lookahead = float(self.get_parameter("lookahead_distance_m").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.maximum_linear = float(
            self.get_parameter("maximum_linear_speed_mps").value
        )
        self.maximum_angular = float(
            self.get_parameter("maximum_angular_speed_radps").value
        )
        self.acceleration = float(
            self.get_parameter("linear_acceleration_mps2").value
        )
        self.deceleration = float(
            self.get_parameter("linear_deceleration_mps2").value
        )
        self.maximum_cross_track = float(
            self.get_parameter("maximum_cross_track_error_m").value
        )
        self.maximum_roll = math.radians(
            float(self.get_parameter("maximum_roll_deg").value)
        )
        self.maximum_pitch = math.radians(
            float(self.get_parameter("maximum_pitch_deg").value)
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
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
            self.maximum_roll,
            self.maximum_pitch,
            self.goal_tolerance,
            self.odom_timeout,
        ) <= 0.0:
            raise ValueError("mapped controller parameters must be positive")
        self.period = 1.0 / rate

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_publisher = self.create_publisher(
            PathMessage, "/sim/mapped_path", path_qos
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )
        self.create_service(Trigger, "~/start", self._start_callback)
        self.create_service(Trigger, "~/stop", self._stop_callback)
        self.create_timer(self.period, self._update)

        self.active = bool(self.get_parameter("auto_start").value)
        self.odom: Odometry | None = None
        self.last_odom_time = None
        self.episode_start_time = self.get_clock().now()
        self.current_index: int | None = None
        self.completed_laps = 0
        self.commanded_speed = 0.0
        self.fault_reason = ""
        self.last_status: dict[str, object] = {}
        self._publish_path()
        self.get_logger().info(
            f"mapped profile ready: {len(self.points)} points, "
            f"length={self.path_length:.2f}m, source={profile_path}"
        )

    def _publish_path(self) -> None:
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for point in self.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            yaw = float(point["yaw"])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message
        self.last_odom_time = self.get_clock().now()

    def _reset_episode(self) -> None:
        self.active = True
        self.current_index = None
        self.completed_laps = 0
        self.commanded_speed = 0.0
        self.fault_reason = ""
        self.last_status = {}
        self.episode_start_time = self.get_clock().now()

    def _start_callback(self, _request: object, response: object) -> object:
        self._reset_episode()
        response.success = True
        response.message = "mapped profile following enabled"
        return response

    def _stop_callback(self, _request: object, response: object) -> object:
        self.active = False
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())
        response.success = True
        response.message = "mapped profile following stopped"
        return response

    def _target_index(self, start: int) -> int:
        target_distance = self.cumulative_length[start] + self.lookahead
        return min(
            len(self.points) - 1,
            int(np.searchsorted(self.cumulative_length, target_distance)),
        )

    def _fault(self, reason: str) -> None:
        if not self.fault_reason:
            self.get_logger().error(f"mapped autonomy stopped: {reason}")
        self.fault_reason = reason
        self.active = False
        self.commanded_speed = 0.0
        self.command_publisher.publish(Twist())

    def _publish_status(self) -> None:
        document = {
            "active": self.active,
            "fault_reason": self.fault_reason,
            "completed_laps": self.completed_laps,
            "episode_elapsed_sim_sec": max(
                0.0,
                (self.get_clock().now() - self.episode_start_time).nanoseconds
                * 1.0e-9,
            ),
            **self.last_status,
        }
        message = String()
        message.data = json.dumps(document, separators=(",", ":"), allow_nan=False)
        self.status_publisher.publish(message)

    def _update(self) -> None:
        if self.odom is None or self.last_odom_time is None:
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1.0e-9
        if age > self.odom_timeout:
            self._fault(f"floating-base odometry timeout ({age:.2f}s)")
            self._publish_status()
            return
        if not self.active:
            self.command_publisher.publish(Twist())
            self._publish_status()
            return

        position = self.odom.pose.pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.odom.pose.pose.orientation)
        x = float(position.x)
        y = float(position.y)
        index, cross_track = open_path_nearest_index(
            self.path_xy, x, y, self.current_index
        )
        point = self.points[index]
        path_yaw = float(point["yaw"])
        dx = x - float(point["x"])
        dy = y - float(point["y"])
        signed_lateral = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
        heading_error = wrap_angle(yaw - path_yaw)
        progress = float(self.cumulative_length[index] / self.path_length)
        self.last_status = {
            "path_progress_pct": 100.0 * progress,
            "reference_lateral_error_m": signed_lateral,
            "reference_heading_error_deg": math.degrees(heading_error),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "reference_height_delta_m": float(position.z) - 0.103,
            "tire_clearance_m": float(point.get("tire_edge_clearance_m", 0.0)),
            "recovery_count": 0,
            "recovery_active": False,
            "corridor_available": cross_track <= self.maximum_cross_track,
            "command_linear_mps": self.commanded_speed,
            "command_angular_radps": 0.0,
        }
        if cross_track > self.maximum_cross_track:
            self._fault(f"mapped corridor departure ({cross_track:.3f}m)")
            self._publish_status()
            return
        if abs(roll) > self.maximum_roll or abs(pitch) > self.maximum_pitch:
            self._fault(
                f"flat-test attitude roll={math.degrees(roll):.1f}deg, "
                f"pitch={math.degrees(pitch):.1f}deg"
            )
            self._publish_status()
            return

        goal_distance = float(np.linalg.norm(self.path_xy[-1] - np.asarray([x, y])))
        if index >= len(self.points) - 3 and goal_distance <= self.goal_tolerance:
            self.completed_laps = 1
            self.active = False
            self.commanded_speed = 0.0
            self.last_status["path_progress_pct"] = 100.0
            self.command_publisher.publish(Twist())
            self._publish_status()
            self.get_logger().info("mapped profile completed")
            return

        self.current_index = max(index, self.current_index or 0)
        target = self.points[self._target_index(self.current_index)]
        desired_speed = min(
            self.maximum_linear,
            self.speed_scale * float(point.get("speed_mps", 0.10)),
        )
        risk = max(0.0, min(1.0, float(point.get("traversability_cost", 0.0))))
        desired_speed *= max(0.70, 1.0 - 0.35 * risk)
        if abs(heading_error) > 0.45:
            desired_speed = min(desired_speed, 0.06)
        delta = desired_speed - self.commanded_speed
        rate_limit = (
            self.acceleration if delta >= 0.0 else self.deceleration
        ) * self.period
        self.commanded_speed += max(-rate_limit, min(rate_limit, delta))

        target_dx = float(target["x"]) - x
        target_dy = float(target["y"]) - y
        local_y = -math.sin(yaw) * target_dx + math.cos(yaw) * target_dy
        distance_squared = max(target_dx * target_dx + target_dy * target_dy, 1.0e-6)
        angular = self.commanded_speed * 2.0 * local_y / distance_squared
        angular -= 0.35 * heading_error
        angular = max(-self.maximum_angular, min(self.maximum_angular, angular))
        command = Twist()
        command.linear.x = self.commanded_speed
        command.angular.z = angular
        self.command_publisher.publish(command)
        self.last_status["command_linear_mps"] = self.commanded_speed
        self.last_status["command_angular_radps"] = angular
        self._publish_status()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappedProfileController()
    try:
        rclpy.spin(node)
    except (
        KeyboardInterrupt,
        RuntimeError,
        rclpy.executors.ExternalShutdownException,
    ):
        pass
    finally:
        try:
            if rclpy.ok():
                node.command_publisher.publish(Twist())
            node.destroy_node()
        except RuntimeError:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
