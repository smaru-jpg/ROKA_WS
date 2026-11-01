#!/usr/bin/env python3
"""Replay a bounded straight command profile, then relocalize and resume Nav2."""

from enum import Enum, auto
import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

from .blind_profile import (
    project_route_progress,
    route_length,
    sample_at,
    validate_profile,
    wrap_angle,
)
from .blind_segment_recorder import yaw_from_quaternion


class State(Enum):
    WAIT_ORIGIN_READY = auto()
    REPLAY = auto()
    SETTLE = auto()
    RELOCALIZE = auto()
    WAIT_START = auto()
    DONE = auto()
    FAILED = auto()


class BlindSegmentController(Node):
    def __init__(self):
        super().__init__("blind_segment_controller")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("route_profile_file", "")
        self.declare_parameter("yaw_correction_gain", 0.8)
        self.declare_parameter("maximum_yaw_correction_radps", 0.25)
        self.declare_parameter("maximum_yaw_error_deg", 30.0)
        self.declare_parameter("maximum_roll_deg", 50.0)
        self.declare_parameter("maximum_pitch_deg", 55.0)
        self.declare_parameter("distance_overshoot_m", 0.50)
        self.declare_parameter("distance_completion_ratio", 0.95)
        self.declare_parameter("minimum_duration_distance_ratio", 0.85)
        self.declare_parameter("distance_odom_topic", "/wheel/odom")
        self.declare_parameter("maximum_imu_age_sec", 0.50)
        self.declare_parameter("maximum_odom_age_sec", 0.50)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("origin_ready_timeout_sec", 120.0)
        self.declare_parameter("exit_settle_sec", 0.50)
        self.declare_parameter("localization_timeout_sec", 8.0)
        self.declare_parameter("maximum_exit_progress_error_ratio", 0.05)
        self.declare_parameter("maximum_exit_route_distance_m", 0.50)
        filename = Path(str(self.get_parameter("profile_file").value)).expanduser()
        self.profile = validate_profile(yaml.safe_load(filename.read_text(encoding="utf-8")))
        route_filename = str(self.get_parameter("route_profile_file").value)
        if not route_filename:
            raise ValueError("route_profile_file is required")
        self.route_profile_file = Path(route_filename).expanduser().resolve()
        route_document = yaml.safe_load(
            self.route_profile_file.read_text(encoding="utf-8")
        ) or {}
        self.route_points = list(route_document.get("points", []))
        self.route_length_m = route_length(self.route_points)
        self.yaw_gain = float(self.get_parameter("yaw_correction_gain").value)
        self.max_yaw_correction = float(
            self.get_parameter("maximum_yaw_correction_radps").value
        )
        self.max_yaw_error = math.radians(
            float(self.get_parameter("maximum_yaw_error_deg").value)
        )
        self.max_roll = math.radians(float(self.get_parameter("maximum_roll_deg").value))
        self.max_pitch = math.radians(float(self.get_parameter("maximum_pitch_deg").value))
        self.distance_overshoot = float(self.get_parameter("distance_overshoot_m").value)
        self.distance_completion_ratio = float(
            self.get_parameter("distance_completion_ratio").value
        )
        self.minimum_duration_distance_ratio = float(
            self.get_parameter("minimum_duration_distance_ratio").value
        )
        self.distance_odom_topic = str(
            self.get_parameter("distance_odom_topic").value
        ).strip()
        self.maximum_imu_age = float(
            self.get_parameter("maximum_imu_age_sec").value
        )
        self.maximum_odom_age = float(
            self.get_parameter("maximum_odom_age_sec").value
        )
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.origin_ready_timeout = float(
            self.get_parameter("origin_ready_timeout_sec").value
        )
        self.exit_settle = float(self.get_parameter("exit_settle_sec").value)
        self.localization_timeout = float(
            self.get_parameter("localization_timeout_sec").value
        )
        self.maximum_exit_progress_error = float(
            self.get_parameter("maximum_exit_progress_error_ratio").value
        )
        self.maximum_exit_route_distance = float(
            self.get_parameter("maximum_exit_route_distance_m").value
        )
        if (
            not self.distance_odom_topic
            or self.maximum_imu_age <= 0.0
            or self.maximum_odom_age <= 0.0
            or self.service_timeout <= 0.0
            or self.origin_ready_timeout <= 0.0
            or not 0.0 < self.maximum_exit_progress_error < 0.5
            or self.maximum_exit_route_distance <= 0.0
            or not 0.5 <= self.minimum_duration_distance_ratio <= (
                self.distance_completion_ratio
            ) <= 1.0
        ):
            raise ValueError("invalid fog-segment freshness/service parameters")
        recorded_route_length = float(self.profile.get("route_length_m", 0.0))
        expected_distance = self.route_length_m * self.profile["exit_progress_ratio"]
        if recorded_route_length <= 0.0 or abs(
            recorded_route_length - self.route_length_m
        ) > 0.05 * self.route_length_m:
            raise ValueError("blind profile belongs to a different route map; re-record it")
        if not 0.85 * expected_distance <= self.profile["distance_m"] <= (
            1.15 * expected_distance
        ):
            raise ValueError("blind profile distance does not match its selected exit")

        self.state = State.WAIT_ORIGIN_READY
        self.state_entered = self.get_clock().now()
        self.pending = None
        self.entry_yaw = None
        self.yaw = None
        self.roll = None
        self.pitch = None
        self.imu_received_nanoseconds = None
        self.latest_xy = None
        self.last_xy = None
        self.odom_received_nanoseconds = None
        self.travelled = 0.0
        self.replay_started = None
        self.localization_after_exit = False
        self.field_preflight_ready = False
        self.last_exit_rejection = "no post-fog localization"
        self.command_pub = self.create_publisher(Twist, "/cmd_vel_mission", 10)
        self.status_pub = self.create_publisher(String, "/roka/mission/fog/status", 10)
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry,
            self.distance_odom_topic,
            self._odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._localization, 10
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.blind_active_pub = self.create_publisher(
            Bool, "/roka/mission/fog/blind_active", latched_qos
        )
        self.create_subscription(
            Bool, "/field/preflight_ready", self._field_preflight, latched_qos
        )
        self.start_client = self.create_client(Trigger, "/track_path_follower/start")
        self.create_timer(0.02, self._tick)
        self.create_timer(1.0, self._status)
        self.get_logger().info(
            f"[MISSION-1] start-origin fog controller ready: blind range "
            f"0..{self.profile['exit_progress_ratio']:.0%}; "
            f"replay {self.profile['duration_sec']:.2f}s/{self.profile['distance_m']:.2f}m"
        )

    def _transition(self, state, detail=""):
        self.state = state
        self.state_entered = self.get_clock().now()
        self.pending = None
        self.get_logger().info(
            f"[MISSION-1] state={state.name}" + (f"; {detail}" if detail else "")
        )
        self.blind_active_pub.publish(
            Bool(data=state in {State.REPLAY, State.SETTLE})
        )

    def _elapsed_state(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds / 1.0e9

    def _field_preflight(self, message):
        self.field_preflight_ready = bool(message.data)

    def _imu(self, message):
        q = message.orientation
        self.yaw = yaw_from_quaternion(q)
        sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
        cos_roll = 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)
        self.roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.asin(sin_pitch)
        self.imu_received_nanoseconds = self.get_clock().now().nanoseconds

    def _odom(self, message):
        self.latest_xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.state == State.REPLAY and self.last_xy is not None:
            self.travelled += math.hypot(
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
            self._input_age(self.imu_received_nanoseconds) <= self.maximum_imu_age
            and self._input_age(self.odom_received_nanoseconds) <= self.maximum_odom_age
        )

    def _localization(self, message):
        if self.state != State.RELOCALIZE:
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        if age < 0.0 or age > 0.50:
            return
        covariance = message.pose.covariance
        position_stddev = math.sqrt(max(0.0, covariance[0], covariance[7]))
        yaw_stddev = math.sqrt(max(0.0, covariance[35]))
        if position_stddev <= 1.0 and yaw_stddev <= math.radians(45.0):
            position = message.pose.pose.position
            progress, route_distance = project_route_progress(
                self.route_points, position.x, position.y
            )
            progress_error = abs(progress - self.profile["exit_progress_ratio"])
            if (
                progress_error <= self.maximum_exit_progress_error
                and route_distance <= self.maximum_exit_route_distance
            ):
                self.localization_after_exit = True
                self.last_exit_rejection = ""
            else:
                self.last_exit_rejection = (
                    f"localized at {progress:.1%}, {route_distance:.2f}m from route; "
                    f"expected {self.profile['exit_progress_ratio']:.0%} exit"
                )

    def _publish_zero(self):
        self.command_pub.publish(Twist())

    def _call(self, client, label):
        if self.pending is not None or not client.service_is_ready():
            return
        self.pending = client.call_async(Trigger.Request())
        self.pending.add_done_callback(lambda future: self._service_done(future, label))

    def _service_done(self, future, label):
        self.pending = None
        try:
            response = future.result()
            if not response.success:
                self._fail(f"{label} rejected: {response.message}")
                return
        except Exception as error:
            self._fail(f"{label} failed: {error}")
            return
        if self.state == State.WAIT_START:
            self._transition(State.DONE, "LiDAR localization accepted; route resumed")

    def _fail(self, reason):
        self._transition(State.FAILED, reason)
        self._publish_zero()

    def _replay_elapsed(self):
        return (self.get_clock().now() - self.replay_started).nanoseconds / 1.0e9

    def _tick(self):
        if (
            self.state == State.WAIT_START
            and self._elapsed_state() > self.service_timeout
        ):
            self._fail(f"{self.state.name} service timeout")
            return
        if self.state == State.WAIT_ORIGIN_READY:
            self._publish_zero()
            if self._elapsed_state() > self.origin_ready_timeout:
                self._fail("field preflight/IMU/wheel odometry start timeout")
            elif (
                self.field_preflight_ready
                and self.yaw is not None
                and self.latest_xy is not None
                and self._inputs_are_fresh()
            ):
                self.entry_yaw = self.yaw
                self.last_xy = self.latest_xy
                self.travelled = 0.0
                self.replay_started = self.get_clock().now()
                self._transition(
                    State.REPLAY,
                    "field ready; route remains stopped; IMU/wheel replay active",
                )
        elif self.state == State.REPLAY:
            elapsed = self._replay_elapsed()
            if not self._inputs_are_fresh():
                self._fail("IMU/wheel odometry became stale in the blind segment")
                return
            if abs(self.roll or 0.0) > self.max_roll or abs(self.pitch or 0.0) > self.max_pitch:
                self._fail("roll/pitch rollover guard reached")
                return
            sample = sample_at(self.profile, elapsed)
            desired_yaw = wrap_angle(self.entry_yaw + sample["yaw_delta_rad"])
            yaw_error = wrap_angle(desired_yaw - self.yaw)
            if abs(yaw_error) > self.max_yaw_error:
                self._fail("straight-segment yaw deviation exceeded")
                return
            if self.profile["distance_m"] > 0.0 and self.travelled > (
                self.profile["distance_m"] + self.distance_overshoot
            ):
                self._fail("blind-segment odometry overshoot")
                return
            if self.travelled >= (
                self.distance_completion_ratio * self.profile["distance_m"]
            ):
                self._publish_zero()
                self._transition(State.SETTLE, "wheel odometry exit distance reached")
                return
            if elapsed >= self.profile["duration_sec"]:
                if self.travelled < (
                    self.minimum_duration_distance_ratio * self.profile["distance_m"]
                ):
                    self._fail("blind-segment wheel distance underrun")
                    return
                self._publish_zero()
                self._transition(
                    State.SETTLE,
                    "recorded duration complete with wheel distance verified",
                )
                return
            command = Twist()
            command.linear.x = sample["linear_mps"]
            correction = max(
                -self.max_yaw_correction,
                min(self.max_yaw_correction, self.yaw_gain * yaw_error),
            )
            command.angular.z = sample["angular_radps"] + correction
            self.command_pub.publish(command)
        elif self.state == State.SETTLE:
            self._publish_zero()
            if self._elapsed_state() >= self.exit_settle:
                self.localization_after_exit = False
                self._transition(State.RELOCALIZE, "waiting for post-fog LiDAR localization")
        elif self.state == State.RELOCALIZE:
            self._publish_zero()
            if self.localization_after_exit:
                self._transition(State.WAIT_START, "post-fog localization valid")
            elif self._elapsed_state() > self.localization_timeout:
                self._fail(
                    "post-fog localization timeout; "
                    f"{self.last_exit_rejection}; operator recovery required"
                )
        elif self.state == State.WAIT_START:
            self._publish_zero()
            self._call(self.start_client, "resume mapped route")
        elif self.state == State.FAILED:
            self._publish_zero()

    def _status(self):
        self.status_pub.publish(String(data=(
            f"state={self.state.name};travelled_m={self.travelled:.3f};"
            f"profile_distance_m={self.profile['distance_m']:.3f};"
            f"blind_exit={self.profile['exit_progress_ratio']:.0%}"
        )))


def main(args=None):
    rclpy.init(args=args)
    node = BlindSegmentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
