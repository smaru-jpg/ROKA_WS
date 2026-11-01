"""Join and continuously follow an approved mapped ROKA route."""

from __future__ import annotations

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from roka_nav2_bringup.profile_validation import is_profile_allowed
from roka_nav2_bringup.route_geometry import (
    corridor_boundaries,
    generate_join_path,
    prepare_centerline,
    project_pose,
    route_completion,
    wrap_angle,
)
from roka_robot_description.hardware_config import load_hardware_config


def path_progress_metrics(points, index, lookahead_distance):
    """Return target index, progress ratio, travelled and remaining distance."""
    if not points:
        raise ValueError("path progress requires at least one point")
    if lookahead_distance < 0.0:
        raise ValueError("visualization lookahead distance must not be negative")
    index = max(0, min(int(index), len(points) - 1))
    segment_lengths = [
        math.hypot(
            float(points[position]["x"]) - float(points[position - 1]["x"]),
            float(points[position]["y"]) - float(points[position - 1]["y"]),
        )
        for position in range(1, len(points))
    ]
    total = sum(segment_lengths)
    travelled = sum(segment_lengths[:index])
    remaining = max(0.0, total - travelled)
    progress = 1.0 if total <= 1.0e-9 else min(1.0, travelled / total)
    target = index
    forward = 0.0
    while target < len(points) - 1 and forward < lookahead_distance:
        forward += segment_lengths[target]
        target += 1
    return target, progress, travelled, remaining


def bounded_start_distance(requested_distance_m, route_length_m):
    """Keep a short demonstration from accepting a join near its far end."""
    requested = float(requested_distance_m)
    route_length = float(route_length_m)
    if not math.isfinite(requested) or requested <= 0.0:
        raise ValueError("maximum start distance must be positive and finite")
    if not math.isfinite(route_length) or route_length <= 0.0:
        raise ValueError("route length must be positive and finite")
    return min(requested, max(0.30, 0.5 * route_length))


class TrackPathFollower(Node):
    """A topology-aware route follower with a curvature-limited join."""

    def __init__(self):
        super().__init__("track_path_follower")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("loop", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_start_timeout_sec", 120.0)
        self.declare_parameter("allow_test_profile", False)
        self.declare_parameter("operation_profile", "inline_track")
        self.declare_parameter("speed_publish_rate_hz", 10.0)
        self.declare_parameter("visualization_publish_rate_hz", 5.0)
        self.declare_parameter("visualization_lookahead_distance_m", 0.50)
        self.declare_parameter("maximum_start_distance_m", 0.75)
        self.declare_parameter("maximum_start_heading_error_deg", 90.0)
        self.declare_parameter("route_spacing_m", 0.05)
        self.declare_parameter("route_maximum_smoothing_deviation_m", 0.08)
        self.declare_parameter("preserve_recorded_geometry", False)
        self.declare_parameter("minimum_turn_radius_m", 0.50)
        self.declare_parameter("join_minimum_lookahead_m", 0.50)
        self.declare_parameter("join_maximum_lookahead_m", 1.50)
        self.declare_parameter("enforce_join_corridor", True)
        self.declare_parameter("corridor_safety_margin_m", 0.01)
        self.declare_parameter("corridor_reference_mode", "track_remaining_width")
        self.declare_parameter("corridor_warning_ratio", 0.65)
        self.declare_parameter("corridor_recovery_ratio", 0.80)
        self.declare_parameter("corridor_hard_ratio", 1.00)
        self.declare_parameter("enforce_corridor_hard_stop", True)
        self.declare_parameter("require_join_occupancy_clear", True)
        self.declare_parameter("join_allow_unknown_occupancy", False)
        self.declare_parameter("occupancy_threshold", 65)
        self.declare_parameter("open_endpoint_tolerance_m", 0.30)
        self.declare_parameter("open_completion_heading_deg", 180.0)
        self.declare_parameter("closed_completion_progress_ratio", 0.95)
        self.declare_parameter("closed_completion_heading_deg", 60.0)
        self.declare_parameter("require_localization", True)
        self.declare_parameter("require_field_preflight", False)
        self.declare_parameter("maximum_localization_age_sec", 2.0)
        self.declare_parameter("maximum_position_stddev_m", 1.0)
        self.declare_parameter("maximum_yaw_stddev_deg", 45.0)
        self.declare_parameter("minimum_localization_stable_sec", 2.0)
        self.declare_parameter("maximum_localization_stable_position_span_m", 0.15)
        self.declare_parameter("maximum_localization_stable_yaw_span_deg", 10.0)
        self.declare_parameter(
            "required_active_lifecycle_nodes",
            ["/controller_server", "/terrain_velocity_smoother", "/collision_monitor"],
        )

        filename = str(self.get_parameter("profile_file").value)
        if not filename:
            raise ValueError("profile_file must point to track_profile.yaml")
        self.profile_file = Path(filename).expanduser()
        document = yaml.safe_load(self.profile_file.read_text(encoding="utf-8")) or {}
        self.frame_id = str(document.get("frame_id", "map"))
        self.closed_loop = bool(document.get("closed_loop", False))
        self.profile_allowed = is_profile_allowed(
            document,
            allow_test_profile=bool(self.get_parameter("allow_test_profile").value),
        )
        self.operation_profile = str(
            self.get_parameter("operation_profile").value
        ).strip().lower()
        if self.operation_profile not in {"flat_commissioning", "inline_track"}:
            raise ValueError(
                "operation_profile must be flat_commissioning or inline_track"
            )
        source_points = list(document.get("points", []))
        if len(source_points) < 2:
            raise ValueError("track profile requires at least two points")
        self.route_spacing = float(self.get_parameter("route_spacing_m").value)
        preserve_recorded_geometry = bool(
            self.get_parameter("preserve_recorded_geometry").value
        )
        prepared_geometry = document.get("centerline_geometry") or (
            document.get("test_path_conversion") or {}
        ).get("centerline")
        self.points, self.route_metadata = prepare_centerline(
            source_points,
            closed=self.closed_loop,
            spacing_m=self.route_spacing,
            maximum_deviation_m=float(
                self.get_parameter("route_maximum_smoothing_deviation_m").value
            ),
            # Prepared test/safe profiles were already smoothed before their
            # exact terrain/occupancy approval. Runtime must not move them.
            smoothing_iterations=(
                0 if prepared_geometry or preserve_recorded_geometry else 3
            ),
        )
        if preserve_recorded_geometry:
            self.route_metadata["runtime_geometry_policy"] = (
                "recorded_polyline_resample_only"
            )
        elif prepared_geometry:
            self.route_metadata["runtime_geometry_policy"] = "resample_only_preapproved"
        for point in self.points:
            values = [float(point["x"]), float(point["y"]), float(point["yaw"])]
            if "speed_mps" in point and float(point["speed_mps"]) <= 0.0:
                raise ValueError("track speed_mps must be positive")
            if not all(math.isfinite(value) for value in values):
                raise ValueError("track profile contains a non-finite point")

        self.hardware = load_hardware_config()
        self.track_width = float(document.get("track_width_m", 0.9144))
        corridor_reference_mode = str(
            self.get_parameter("corridor_reference_mode").value
        ).strip().lower()
        if corridor_reference_mode == "track_remaining_width":
            corridor_reference = 0.5 * (
                self.track_width - self.hardware.navigation_width_m
            )
        elif corridor_reference_mode == "vehicle_half_width":
            corridor_reference = self.hardware.navigation_half_width_m
        else:
            raise ValueError(
                "corridor_reference_mode must be track_remaining_width or "
                "vehicle_half_width"
            )
        self.corridor_reference_mode = corridor_reference_mode
        self.corridor_half_width = corridor_reference - float(
            self.get_parameter("corridor_safety_margin_m").value
        )
        if self.corridor_half_width <= 0.02:
            raise ValueError(
                "approved track width cannot contain the conservative vehicle footprint"
            )
        ratios = [
            float(self.get_parameter("corridor_warning_ratio").value),
            float(self.get_parameter("corridor_recovery_ratio").value),
            float(self.get_parameter("corridor_hard_ratio").value),
        ]
        if not 0.0 < ratios[0] < ratios[1] < ratios[2] <= 1.0:
            raise ValueError("corridor warning/recovery/hard ratios are invalid")
        self.corridor_warning = ratios[0] * self.corridor_half_width
        self.corridor_recovery = ratios[1] * self.corridor_half_width
        self.corridor_hard = ratios[2] * self.corridor_half_width
        self.enforce_corridor_hard_stop = bool(
            self.get_parameter("enforce_corridor_hard_stop").value
        )
        self.minimum_turn_radius = float(
            self.get_parameter("minimum_turn_radius_m").value
        )
        self.join_minimum_lookahead = float(
            self.get_parameter("join_minimum_lookahead_m").value
        )
        self.join_maximum_lookahead = float(
            self.get_parameter("join_maximum_lookahead_m").value
        )
        self.enforce_join_corridor = bool(
            self.get_parameter("enforce_join_corridor").value
        )
        requested_start_distance = float(
            self.get_parameter("maximum_start_distance_m").value
        )
        # A join allowance larger than a short demonstrated route can select
        # its far endpoint and send the robot in an unintuitive direction.
        # Long tracks retain their configured tolerance; short commissioning
        # routes are bounded by half their demonstrated length.
        self.maximum_start_distance = bounded_start_distance(
            requested_start_distance,
            self.route_metadata["length_m"],
        )
        self.maximum_start_heading = math.radians(float(
            self.get_parameter("maximum_start_heading_error_deg").value
        ))
        self.require_join_occupancy = bool(
            self.get_parameter("require_join_occupancy_clear").value
        )
        self.join_allow_unknown = bool(
            self.get_parameter("join_allow_unknown_occupancy").value
        )
        self.occupancy_threshold = int(self.get_parameter("occupancy_threshold").value)
        self.open_endpoint_tolerance = float(
            self.get_parameter("open_endpoint_tolerance_m").value
        )
        self.open_completion_heading = math.radians(float(
            self.get_parameter("open_completion_heading_deg").value
        ))
        self.closed_completion_progress = float(
            self.get_parameter("closed_completion_progress_ratio").value
        )
        self.closed_completion_heading = math.radians(float(
            self.get_parameter("closed_completion_heading_deg").value
        ))
        self.default_speed = float(document.get("default_speed_mps", 0.30))
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.loop = bool(self.get_parameter("loop").value) and self.closed_loop

        self.require_localization = bool(self.get_parameter("require_localization").value)
        self.require_field_preflight = bool(
            self.get_parameter("require_field_preflight").value
        )
        self.field_preflight_received = False
        self.field_preflight_ready = False
        self.maximum_localization_age = float(
            self.get_parameter("maximum_localization_age_sec").value
        )
        self.maximum_position_stddev = float(
            self.get_parameter("maximum_position_stddev_m").value
        )
        self.maximum_yaw_stddev = math.radians(float(
            self.get_parameter("maximum_yaw_stddev_deg").value
        ))
        self.minimum_localization_stable = float(
            self.get_parameter("minimum_localization_stable_sec").value
        )
        self.maximum_localization_position_span = float(
            self.get_parameter(
                "maximum_localization_stable_position_span_m"
            ).value
        )
        self.maximum_localization_yaw_span = math.radians(float(
            self.get_parameter("maximum_localization_stable_yaw_span_deg").value
        ))
        if (
            self.minimum_localization_stable <= 0.0
            or self.maximum_localization_position_span <= 0.0
            or self.maximum_localization_yaw_span <= 0.0
        ):
            raise ValueError("localization stability limits must be positive")
        self.localization_received_at = None
        self.localization_position_stddev = math.inf
        self.localization_yaw_stddev = math.inf
        self.localization_stability_samples = []

        self.required_lifecycle_nodes = [
            str(value).rstrip("/")
            for value in self.get_parameter("required_active_lifecycle_nodes").value
        ]
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"{name}/get_state")
            for name in self.required_lifecycle_nodes
        }
        self.lifecycle_futures = {name: None for name in self.required_lifecycle_nodes}
        self.lifecycle_states = {name: None for name in self.required_lifecycle_nodes}
        self.create_timer(0.25, self.update_lifecycle_states)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.action_client = ActionClient(
            self, FollowPath, str(self.get_parameter("follow_path_action").value)
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(PathMessage, "/track/centerline", latched_qos)
        self.corridor_left_pub = self.create_publisher(
            PathMessage, "/track/corridor_left", latched_qos
        )
        self.corridor_right_pub = self.create_publisher(
            PathMessage, "/track/corridor_right", latched_qos
        )
        self.join_path_pub = self.create_publisher(
            PathMessage, "/track/join_path", latched_qos
        )
        self.traversed_path_pub = self.create_publisher(
            PathMessage, "/track/traversed_path", latched_qos
        )
        self.remaining_path_pub = self.create_publisher(
            PathMessage, "/track/remaining_path", latched_qos
        )
        self.actual_path_pub = self.create_publisher(
            PathMessage, "/track/actual_path", latched_qos
        )
        self.target_pub = self.create_publisher(PoseStamped, "/track/current_target", latched_qos)
        self.localized_pose_pub = self.create_publisher(
            PoseStamped, "/track/localized_pose", latched_qos
        )
        self.progress_marker_pub = self.create_publisher(
            Marker, "/track/progress_marker", latched_qos
        )
        self.status_pub = self.create_publisher(String, "/track/follower_status", latched_qos)
        self.speed_pub = self.create_publisher(Float32, "/track/reference_speed", 10)
        self.progress_pub = self.create_publisher(Float32, "/track/progress_ratio", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.localization_callback, 10
        )
        self.create_subscription(
            Bool, "/field/preflight_ready", self.field_preflight_callback, latched_qos
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, latched_qos)
        self.create_service(Trigger, "~/start", self.start_callback)
        self.create_service(Trigger, "~/stop", self.stop_callback)

        self.active = False
        self.goal_handle = None
        self.active_points = list(self.points)
        self.join_point_count = 0
        self.progress_index = 0
        self.progress_unwrapped_index = 0
        self.actual_points: list[dict[str, float]] = []
        self.map_message: OccupancyGrid | None = None
        self.completion_cancel_pending = False
        self.last_lateral_error = 0.0
        self.follower_state = "BOOT"
        self.follower_detail = "route geometry loaded"
        self.last_status_text = ""

        self.visualization_lookahead = float(
            self.get_parameter("visualization_lookahead_distance_m").value
        )
        speed_rate = float(self.get_parameter("speed_publish_rate_hz").value)
        visualization_rate = float(
            self.get_parameter("visualization_publish_rate_hz").value
        )
        if speed_rate <= 0.0 or visualization_rate <= 0.0:
            raise ValueError("track publication rates must be positive")
        self.create_timer(1.0 / speed_rate, self.update_speed)
        self.create_timer(1.0 / visualization_rate, self.update_visualization)

        self.auto_start_enabled = bool(self.get_parameter("auto_start").value)
        self.auto_start_timeout = float(
            self.get_parameter("auto_start_timeout_sec").value
        )
        if self.auto_start_timeout <= 0.0:
            raise ValueError("auto_start_timeout_sec must be positive")
        self.auto_start_deadline = self.get_clock().now() + Duration(
            seconds=self.auto_start_timeout
        )
        self.auto_start_accepted = False
        self.last_auto_start_wait_reason = ""
        self.start_pending = self.auto_start_enabled
        self.create_timer(0.5, self.auto_start_timer)

        self.publish_static_route_visualization()
        self.set_follower_state("PREFLIGHT", "waiting for localization and control chain")
        self.get_logger().info(
            "[ROUTE] loaded %s: mode=%s, source=%d, smooth=%d, length=%.2fm, "
            "corridor_hard=%.3fm (%s), closed=%s"
            % (
                self.profile_file,
                self.operation_profile,
                int(self.route_metadata["source_point_count"]),
                len(self.points),
                float(self.route_metadata["length_m"]),
                self.corridor_hard,
                self.corridor_reference_mode,
                self.closed_loop,
            )
        )

    @staticmethod
    def quaternion_from_yaw(yaw):
        return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def make_path(self, points):
        message = PathMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.position.z = float(point.get("z", 0.0))
            qx, qy, qz, qw = self.quaternion_from_yaw(float(point.get("yaw", 0.0)))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        return message

    def make_pose(self, point):
        return self.make_path([point]).poses[0]

    def publish_static_route_visualization(self):
        self.path_pub.publish(self.make_path(self.points))
        left, right = corridor_boundaries(self.points, self.corridor_hard)
        self.corridor_left_pub.publish(self.make_path(left))
        self.corridor_right_pub.publish(self.make_path(right))

    def set_follower_state(self, state, detail=""):
        state, detail = str(state), str(detail)
        changed = state != self.follower_state or detail != self.follower_detail
        self.follower_state, self.follower_detail = state, detail
        self.publish_follower_status()
        if changed:
            self.get_logger().info(f"[ROUTE][{state}] {detail}")

    def publish_follower_status(self):
        text = self.follower_state
        if self.follower_detail:
            text += f": {self.follower_detail}"
        if text == self.last_status_text:
            return
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.last_status_text = text

    def publish_progress_visualization(self, index):
        points = self.active_points
        if not points:
            return
        index = max(0, min(int(index), len(points) - 1))
        target_index, progress, _, remaining = path_progress_metrics(
            points, index, self.visualization_lookahead
        )
        self.traversed_path_pub.publish(self.make_path(points[:index + 1]))
        self.remaining_path_pub.publish(self.make_path(points[index:]))
        self.target_pub.publish(self.make_pose(points[target_index]))
        progress_message = Float32()
        progress_message.data = float(progress)
        self.progress_pub.publish(progress_message)

        speed = float(points[index].get("speed_mps", self.default_speed))
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "track_progress"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(points[index]["x"])
        marker.pose.position.y = float(points[index]["y"])
        marker.pose.position.z = 0.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 1.0 if self.follower_state in {"RECOVERING", "FAULT"} else 0.10
        marker.color.g = 0.45 if self.follower_state in {"RECOVERING", "FAULT"} else 1.0
        marker.color.b = 1.0
        marker.text = (
            f"{self.follower_state}  {100.0 * progress:.0f}%\n"
            f"cross {self.last_lateral_error:+.2f} m | "
            f"remain {remaining:.2f} m | ref {speed:.2f} m/s"
        )
        self.progress_marker_pub.publish(marker)

    def localization_callback(self, message):
        covariance = message.pose.covariance
        self.localization_received_at = self.get_clock().now()
        self.localization_position_stddev = math.sqrt(
            max(0.0, float(covariance[0]), float(covariance[7]))
        )
        self.localization_yaw_stddev = math.sqrt(max(0.0, float(covariance[35])))
        now_seconds = self.get_clock().now().nanoseconds * 1.0e-9
        pose = message.pose.pose
        self.localization_stability_samples.append((
            now_seconds,
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        ))
        oldest = now_seconds - self.minimum_localization_stable
        # Keep one sample immediately before the window boundary so the
        # measured span actually covers the requested duration.
        while (
            len(self.localization_stability_samples) > 2
            and self.localization_stability_samples[1][0] < oldest
        ):
            self.localization_stability_samples.pop(0)
        localized_pose = PoseStamped()
        localized_pose.header = message.header
        localized_pose.pose = message.pose.pose
        self.localized_pose_pub.publish(localized_pose)

    def field_preflight_callback(self, message):
        self.field_preflight_received = True
        self.field_preflight_ready = bool(message.data)

    def map_callback(self, message):
        self.map_message = message

    def require_ready_field_hardware(self):
        if not self.require_field_preflight:
            return
        if not self.field_preflight_received:
            raise RuntimeError("field hardware preflight has not completed")
        if not self.field_preflight_ready:
            raise RuntimeError("field hardware preflight is NOT READY")

    def require_fresh_localization(self):
        if not self.require_localization:
            return
        if self.localization_received_at is None:
            raise RuntimeError("map localization has no live pose")
        age = (
            self.get_clock().now() - self.localization_received_at
        ).nanoseconds / 1.0e9
        if age > self.maximum_localization_age:
            raise RuntimeError(f"map localization pose is stale ({age:.1f}s)")
        if self.localization_position_stddev > self.maximum_position_stddev:
            raise RuntimeError(
                "map localization position uncertainty is "
                f"{self.localization_position_stddev:.2f}m"
            )
        if self.localization_yaw_stddev > self.maximum_yaw_stddev:
            raise RuntimeError(
                "map localization heading uncertainty is "
                f"{math.degrees(self.localization_yaw_stddev):.1f}deg"
            )
        samples = self.localization_stability_samples
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < self.minimum_localization_stable:
            raise RuntimeError(
                "map localization pose has not remained stable for "
                f"{self.minimum_localization_stable:.1f}s"
            )
        origin = samples[0]
        position_span = max(
            math.hypot(sample[1] - origin[1], sample[2] - origin[2])
            for sample in samples
        )
        yaw_span = max(
            abs(wrap_angle(sample[3] - origin[3])) for sample in samples
        )
        if position_span > self.maximum_localization_position_span:
            raise RuntimeError(
                f"map localization is moving/jumping while stopped ({position_span:.2f}m)"
            )
        if yaw_span > self.maximum_localization_yaw_span:
            raise RuntimeError(
                "map localization heading is unstable while stopped "
                f"({math.degrees(yaw_span):.1f}deg)"
            )

    def update_lifecycle_states(self):
        for name, client in self.lifecycle_clients.items():
            future = self.lifecycle_futures[name]
            if future is not None:
                if not future.done():
                    continue
                try:
                    response = future.result()
                    self.lifecycle_states[name] = int(response.current_state.id)
                except Exception:
                    self.lifecycle_states[name] = None
                self.lifecycle_futures[name] = None
            if client.service_is_ready() and self.lifecycle_futures[name] is None:
                self.lifecycle_futures[name] = client.call_async(GetState.Request())

    def require_active_control_chain(self):
        unavailable, inactive = [], []
        for name in self.required_lifecycle_nodes:
            state = self.lifecycle_states.get(name)
            if state is None:
                unavailable.append(name)
            elif state != State.PRIMARY_STATE_ACTIVE:
                inactive.append(name)
        if unavailable:
            raise RuntimeError("control lifecycle unavailable: " + ", ".join(unavailable))
        if inactive:
            raise RuntimeError("control lifecycle inactive: " + ", ".join(inactive))

    def current_pose(self):
        transform = self.buffer.lookup_transform(
            self.frame_id, self.base_frame, rclpy.time.Time()
        )
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            self.yaw_from_quaternion(transform.transform.rotation),
        )

    def _occupancy_value(self, x, y):
        message = self.map_message
        if message is None:
            return None
        origin = message.info.origin
        origin_yaw = self.yaw_from_quaternion(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        column = int(math.floor(local_x / message.info.resolution))
        row = int(math.floor(local_y / message.info.resolution))
        if not 0 <= column < message.info.width or not 0 <= row < message.info.height:
            return -1
        return int(message.data[row * message.info.width + column])

    def require_join_occupancy_clearance(self, points):
        if not self.require_join_occupancy:
            return
        if self.map_message is None:
            raise RuntimeError("approved occupancy map has not been received")
        offsets = [
            (0.0, 0.0),
            (self.hardware.navigation_front_m, self.hardware.navigation_half_width_m),
            (self.hardware.navigation_front_m, -self.hardware.navigation_half_width_m),
            (-self.hardware.navigation_rear_m, self.hardware.navigation_half_width_m),
            (-self.hardware.navigation_rear_m, -self.hardware.navigation_half_width_m),
            (self.hardware.navigation_front_m, 0.0),
            (-self.hardware.navigation_rear_m, 0.0),
            (0.0, self.hardware.navigation_half_width_m),
            (0.0, -self.hardware.navigation_half_width_m),
        ]
        stride = max(1, int(round(0.10 / self.route_spacing)))
        for path_index in range(0, len(points), stride):
            point = points[path_index]
            yaw = float(point.get("yaw", 0.0))
            cosine, sine = math.cos(yaw), math.sin(yaw)
            for forward, lateral in offsets:
                x = float(point["x"]) + cosine * forward - sine * lateral
                y = float(point["y"]) + sine * forward + cosine * lateral
                value = self._occupancy_value(x, y)
                if value is None:
                    raise RuntimeError("approved occupancy map has not been received")
                if value < 0 and not self.join_allow_unknown:
                    raise RuntimeError(
                        f"join footprint enters unknown occupancy near path sample {path_index}"
                    )
                if value >= self.occupancy_threshold:
                    raise RuntimeError(
                        f"join footprint intersects occupied map near path sample {path_index}"
                    )

    def build_active_route(self):
        x, y, yaw = self.current_pose()
        projection = project_pose(self.points, x, y, yaw, closed=self.closed_loop)
        if projection.distance_m > self.maximum_start_distance:
            raise RuntimeError(
                f"start is {projection.distance_m:.2f}m from the recorded route; "
                f"limit={self.maximum_start_distance:.2f}m"
            )
        if projection.heading_error_rad > self.maximum_start_heading:
            raise RuntimeError(
                "start heading differs from the recorded route by "
                f"{math.degrees(projection.heading_error_rad):.1f}deg; "
                f"limit={math.degrees(self.maximum_start_heading):.1f}deg"
            )
        # If the robot is already on and aligned with the demonstrated route,
        # do not manufacture a 0.5 m join that skips the beginning of an open
        # test section. A join is only for genuine placement error.
        if (
            projection.distance_m <= 0.05
            and projection.heading_error_rad <= math.radians(12.0)
        ):
            start_index = projection.index
            if projection.segment_ratio >= 0.75:
                start_index = min(start_index + 1, len(self.points) - 1)
            if self.closed_loop:
                route = self.points[start_index:] + self.points[:start_index + 1]
            else:
                route = self.points[start_index:]
            points = [dict(point) for point in route]
            self.require_join_occupancy_clearance(points)
            self.join_point_count = 0
            self.join_path_pub.publish(self.make_path([{
                "x": x, "y": y, "yaw": yaw,
            }]))
            self.get_logger().info(
                "[ROUTE] robot already aligned: error=%.3fm, heading=%.1fdeg, start=%d"
                % (
                    projection.distance_m,
                    math.degrees(projection.heading_error_rad),
                    start_index,
                )
            )
            return points
        join = generate_join_path(
            self.points,
            x,
            y,
            yaw,
            projection,
            closed=self.closed_loop,
            spacing_m=self.route_spacing,
            minimum_turn_radius_m=self.minimum_turn_radius,
            minimum_lookahead_m=self.join_minimum_lookahead,
            maximum_lookahead_m=self.join_maximum_lookahead,
            corridor_half_width_m=(
                self.corridor_half_width if self.enforce_join_corridor else None
            ),
        )
        if self.closed_loop:
            route = self.points[join.target_index:] + self.points[:join.target_index + 1]
        else:
            route = self.points[join.target_index:]
        join_prefix = join.points[:-1]
        points = list(join_prefix) + [dict(point) for point in route]
        if len(points) < 2:
            raise RuntimeError("route join produced fewer than two path points")
        self.require_join_occupancy_clearance(points)
        self.join_point_count = len(join_prefix)
        self.join_path_pub.publish(self.make_path(join.points))
        self.get_logger().info(
            "[ROUTE] join accepted: start_error=%.3fm, heading=%.1fdeg, "
            "length=%.2fm, max_curvature=%.2frad/m, target=%d"
            % (
                projection.distance_m,
                math.degrees(projection.heading_error_rad),
                join.length_m,
                join.maximum_abs_curvature_radpm,
                join.target_index,
            )
        )
        return points

    def auto_start_timer(self):
        if not self.start_pending or self.active:
            return
        if not self.auto_start_accepted and self.get_clock().now() > self.auto_start_deadline:
            self.start_pending = False
            self.set_follower_state(
                "FAULT",
                "automatic start timed out: "
                f"{self.last_auto_start_wait_reason or 'stack not ready'}",
            )
            return
        try:
            self.start_following()
        except Exception as error:
            reason = str(error)
            if reason != self.last_auto_start_wait_reason:
                self.last_auto_start_wait_reason = reason
                state = (
                    "LOCALIZING"
                    if "localization" in reason or "pose" in reason
                    else "PREFLIGHT"
                )
                self.set_follower_state(state, reason)

    def start_callback(self, _request, response):
        try:
            self.start_following()
            response.success = True
            response.message = "continuous route FollowPath request started"
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.set_follower_state("ROUTE_READY", response.message)
        return response

    def stop_callback(self, _request, response):
        self.start_pending = False
        self.active = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.set_follower_state("STOPPED", "manual stop requested")
        response.success = True
        response.message = "route following cancelled"
        return response

    def start_following(self):
        if self.active:
            raise RuntimeError("route follower is already active")
        if not self.profile_allowed:
            raise RuntimeError("route profile approval scope is not allowed")
        self.set_follower_state("PREFLIGHT", "checking field hardware and Nav2")
        self.require_ready_field_hardware()
        self.require_active_control_chain()
        self.require_fresh_localization()
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Nav2 /follow_path action is unavailable")
        self.set_follower_state("ROUTE_READY", "validating join and vehicle footprint")
        points = self.build_active_route()
        self.active_points = points
        self.progress_index = 0
        self.progress_unwrapped_index = 0
        self.actual_points = []
        self.completion_cancel_pending = False
        self.publish_progress_visualization(0)
        goal = FollowPath.Goal()
        goal.path = self.make_path(points)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        speed = Float32()
        speed.data = float(points[0].get("speed_mps", self.default_speed))
        self.speed_pub.publish(speed)
        self.active = True
        self.set_follower_state("JOINING", "FollowPath goal pending")
        self.action_client.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        self.goal_handle = future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.active = False
            self.set_follower_state("FAULT", "Nav2 rejected continuous FollowPath goal")
            if self.auto_start_enabled and not self.auto_start_accepted:
                self.start_pending = True
            return
        self.auto_start_accepted = True
        self.last_auto_start_wait_reason = ""
        if self.join_point_count <= 1:
            self.set_follower_state("TRACKING", "following approved centerline")
        else:
            self.set_follower_state("JOINING", "following curvature-limited join")
        self.goal_handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result()
        self.goal_handle = None
        was_active = self.active
        self.active = False
        if result.status == GoalStatus.STATUS_SUCCEEDED or self.completion_cancel_pending:
            self.progress_index = len(self.active_points) - 1
            self.set_follower_state("COMPLETED", "route completion criteria reached")
        elif self.follower_state != "FAULT":
            self.set_follower_state("FAULT", f"FollowPath status={result.status}")
        if was_active and self.loop and (
            result.status == GoalStatus.STATUS_SUCCEEDED or self.completion_cancel_pending
        ):
            self.start_pending = True

    def _record_actual_pose(self, x, y, yaw):
        if self.actual_points and math.hypot(
            x - self.actual_points[-1]["x"], y - self.actual_points[-1]["y"]
        ) < 0.02:
            return
        self.actual_points.append({"x": x, "y": y, "yaw": yaw})
        if len(self.actual_points) > 5000:
            self.actual_points = self.actual_points[-5000:]
        self.actual_path_pub.publish(self.make_path(self.actual_points))

    def update_visualization(self):
        if not self.active:
            self.publish_progress_visualization(self.progress_index)
            return
        try:
            x, y, yaw = self.current_pose()
            projection = project_pose(
                self.active_points,
                x,
                y,
                yaw,
                closed=False,
                previous_unwrapped_index=self.progress_unwrapped_index,
                backward_window=4,
                forward_window=max(20, int(2.0 / self.route_spacing)),
            )
        except (TransformException, ValueError):
            return
        self.progress_unwrapped_index = max(
            self.progress_unwrapped_index, projection.unwrapped_index
        )
        self.progress_index = min(len(self.active_points) - 1, self.progress_unwrapped_index)
        self.last_lateral_error = projection.signed_lateral_m
        self._record_actual_pose(x, y, yaw)

        if self.follower_state == "JOINING" and self.progress_index >= max(
            0, self.join_point_count - 1
        ):
            self.set_follower_state("TRACKING", "joined approved centerline")
        if self.progress_index >= self.join_point_count:
            lateral = abs(projection.signed_lateral_m)
            if self.enforce_corridor_hard_stop and lateral > self.corridor_hard:
                self.active = False
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.set_follower_state(
                    "FAULT",
                    "vehicle center left approved corridor "
                    f"({lateral:.3f}m > {self.corridor_hard:.3f}m)",
                )
            elif (
                lateral > self.corridor_recovery
                and self.follower_state != "RECOVERING"
            ):
                self.set_follower_state(
                    "RECOVERING", f"controlled centerline recovery ({lateral:.3f}m)"
                )
            elif (
                self.follower_state == "RECOVERING"
                and lateral < self.corridor_warning
            ):
                self.set_follower_state("TRACKING", "centerline recovery complete")

        endpoint = self.active_points[-1]
        endpoint_distance = math.hypot(x - float(endpoint["x"]), y - float(endpoint["y"]))
        endpoint_heading = abs(wrap_angle(float(endpoint.get("yaw", 0.0)) - yaw))
        route_progress_index = max(0, self.progress_unwrapped_index - self.join_point_count)
        route_count = max(2, len(self.active_points) - self.join_point_count - 1)
        if not self.completion_cancel_pending and route_completion(
            closed=self.closed_loop,
            unwrapped_index=route_progress_index,
            route_point_count=route_count,
            distance_to_endpoint_m=endpoint_distance,
            heading_error_rad=endpoint_heading,
            endpoint_tolerance_m=self.open_endpoint_tolerance,
            open_heading_tolerance_rad=self.open_completion_heading,
            closed_progress_ratio=self.closed_completion_progress,
            closed_heading_tolerance_rad=self.closed_completion_heading,
        ):
            self.completion_cancel_pending = True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.set_follower_state("COMPLETED", "progress and endpoint region confirmed")
        self.publish_progress_visualization(self.progress_index)

    def update_speed(self):
        if not self.active:
            return
        index = max(0, min(self.progress_index, len(self.active_points) - 1))
        speed_value = float(self.active_points[index].get("speed_mps", self.default_speed))
        if self.follower_state == "RECOVERING":
            speed_value *= 0.50
        elif index >= self.join_point_count and self.corridor_recovery > 0.0:
            ratio = min(1.0, abs(self.last_lateral_error) / self.corridor_recovery)
            speed_value *= 1.0 - 0.35 * ratio
        speed = Float32()
        speed.data = max(0.03, speed_value)
        self.speed_pub.publish(speed)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPathFollower()
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
