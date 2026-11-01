#!/usr/bin/env python3
"""Coordinate Section 2 base stop/hold and arm pickup without race conditions."""

from enum import Enum, auto
import math

from geometry_msgs.msg import Twist
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from roka_interfaces.msg import BoxObservation
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class State(Enum):
    IDLE = auto()
    WAIT_TARGET = auto()
    STOP_ROUTE = auto()
    REQUEST_HOLD = auto()
    WAIT_HOLD = auto()
    REQUEST_ARM = auto()
    ARM_ACTIVE = auto()
    FAIL_DISABLE_ARM = auto()
    DISABLE_ARM = auto()
    DISABLE_BOX_VISION = auto()
    ENABLE_TRAFFIC_VISION = auto()
    RELEASE_HOLD = auto()
    RESUME_ROUTE = auto()
    COMPLETE = auto()
    FAILED = auto()


def observation_is_eligible(message, age_sec, maximum_age_sec, maximum_depth_mm):
    """Return true for a fresh, reachable target anywhere in the image."""
    return bool(
        message.detected
        and message.depth_valid
        and 0.0 < message.depth_mm <= maximum_depth_mm
        and 0.0 <= age_sec <= maximum_age_sec
    )


def conservative_body_speed(samples, now_nanoseconds, maximum_age_sec):
    """Return the largest fresh body-speed estimate, or infinity if none exists."""
    fresh = [
        speed
        for stamp_nanoseconds, speed in samples.values()
        if 0.0 <= (now_nanoseconds - stamp_nanoseconds) / 1.0e9 <= maximum_age_sec
    ]
    return max(fresh) if fresh else math.inf


class BoxPickCoordinator(Node):
    """Only arm a pickup after the base reports a stable zero-RPM hold."""

    def __init__(self):
        super().__init__("box_pick_coordinator")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("required_detection_frames", 4)
        self.declare_parameter("maximum_observation_age_sec", 0.40)
        self.declare_parameter("maximum_target_depth_mm", 1800.0)
        self.declare_parameter("maximum_stopped_speed_mps", 0.02)
        self.declare_parameter("body_odom_topics", ["/lio/odom", "/odom"])
        self.declare_parameter("body_odom_timeout_sec", 0.50)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("hold_timeout_sec", 5.0)
        self.declare_parameter("resume_route_after_grasp", True)
        self.declare_parameter("operator_trigger_required", False)
        self.declare_parameter("control_recorded_route", True)
        self.declare_parameter(
            "manual_base_lock_topic", "/roka/mission/box_pick/base_lock"
        )
        self.declare_parameter(
            "box_perception_service", "/roka/arm/set_perception_enabled"
        )
        self.declare_parameter(
            "traffic_vision_service", "/roka/vision/traffic/enable"
        )

        self.required_frames = int(self.get_parameter("required_detection_frames").value)
        self.maximum_observation_age = float(
            self.get_parameter("maximum_observation_age_sec").value
        )
        self.maximum_depth = float(self.get_parameter("maximum_target_depth_mm").value)
        self.maximum_stopped_speed = float(
            self.get_parameter("maximum_stopped_speed_mps").value
        )
        self.body_odom_topics = [
            str(topic).strip()
            for topic in self.get_parameter("body_odom_topics").value
            if str(topic).strip()
        ]
        self.body_odom_timeout = float(
            self.get_parameter("body_odom_timeout_sec").value
        )
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.hold_timeout = float(self.get_parameter("hold_timeout_sec").value)
        self.resume_route = bool(self.get_parameter("resume_route_after_grasp").value)
        self.operator_trigger_required = bool(
            self.get_parameter("operator_trigger_required").value
        )
        self.control_recorded_route = bool(
            self.get_parameter("control_recorded_route").value
        )
        self.manual_base_lock_topic = str(
            self.get_parameter("manual_base_lock_topic").value
        ).strip()
        self.box_perception_service = str(
            self.get_parameter("box_perception_service").value
        ).strip()
        self.traffic_vision_service = str(
            self.get_parameter("traffic_vision_service").value
        ).strip()
        if (
            self.required_frames < 1
            or self.maximum_observation_age <= 0.0
            or not self.body_odom_topics
            or self.body_odom_timeout <= 0.0
            or self.service_timeout <= 0.0
            or not self.box_perception_service
            or not self.traffic_vision_service
            or not self.manual_base_lock_topic
        ):
            raise ValueError("invalid Section 2 coordinator parameters")

        self.state = State.WAIT_TARGET if bool(
            self.get_parameter("auto_start").value
        ) else State.IDLE
        self.state_entered = self.get_clock().now()
        self.detection_frames = 0
        self.hold_active = False
        self.hold_ready = False
        self.body_speed_samples = {}
        self.arm_status = ""
        self.pending = None
        self.last_status = ""

        self.status_pub = self.create_publisher(String, "/roka/mission/box_pick/status", 10)
        self.command_pub = self.create_publisher(Twist, "/cmd_vel_mission", 10)
        lock_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.manual_lock_pub = self.create_publisher(
            Bool, self.manual_base_lock_topic, lock_qos
        )
        self.create_subscription(
            BoxObservation,
            "/roka/arm/box_observation",
            self._observation,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/md/hold_active", self._hold_active, 10)
        self.create_subscription(Bool, "/md/hold_ready", self._hold_ready, 10)
        for topic in self.body_odom_topics:
            self.create_subscription(
                Odometry,
                topic,
                lambda message, source=topic: self._odom(message, source),
                qos_profile_sensor_data,
            )
        self.create_subscription(String, "/roka/arm/status", self._arm_status, 10)

        self.hold_client = self.create_client(SetBool, "/md/set_hold")
        self.arm_client = self.create_client(SetBool, "/roka/arm/set_auto_control")
        self.box_perception_client = self.create_client(
            SetBool, self.box_perception_service
        )
        self.traffic_vision_client = self.create_client(
            SetBool, self.traffic_vision_service
        )
        self.route_stop_client = self.create_client(Trigger, "/track_path_follower/stop")
        self.route_start_client = self.create_client(Trigger, "/track_path_follower/start")
        self.create_service(SetBool, "/roka/mission/box_pick/enable", self._enable)
        self.create_service(
            Trigger, "/roka/mission/box_pick/trigger", self._operator_trigger
        )
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "[MISSION-2] ready: target detected -> operator/auto trigger -> "
            "zero-RPM hold -> arm"
        )
        self._publish_manual_lock()

    @staticmethod
    def state_requires_base_lock(state):
        return state in {
            State.STOP_ROUTE,
            State.REQUEST_HOLD,
            State.WAIT_HOLD,
            State.REQUEST_ARM,
            State.ARM_ACTIVE,
            State.FAIL_DISABLE_ARM,
            State.DISABLE_ARM,
            State.DISABLE_BOX_VISION,
            State.ENABLE_TRAFFIC_VISION,
            State.RELEASE_HOLD,
            State.RESUME_ROUTE,
            State.FAILED,
        }

    def _publish_manual_lock(self):
        self.manual_lock_pub.publish(Bool(
            data=self.state_requires_base_lock(self.state)
        ))

    def _transition(self, state, detail=""):
        self.state = state
        self.state_entered = self.get_clock().now()
        self.pending = None
        # Every state transition starts a fresh detection streak.  In particular,
        # re-enabling the mission must not reuse frames accumulated before an
        # operator stop or a completed pick.
        self.detection_frames = 0
        self._publish_manual_lock()
        suffix = f"; {detail}" if detail else ""
        self.get_logger().info(f"[MISSION-2] state={state.name}{suffix}")

    def _elapsed(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds / 1.0e9

    def _enable(self, request, response):
        if request.data:
            if self.state == State.WAIT_TARGET:
                response.success = True
                response.message = self.state.name
                return response
            if self.state not in {State.IDLE, State.COMPLETE}:
                response.success = False
                response.message = (
                    "pickup is active or failed; disable/recover it before re-enabling"
                )
                return response
            self._transition(State.WAIT_TARGET, "operator enabled")
        else:
            if self.state in {
                State.REQUEST_ARM,
                State.ARM_ACTIVE,
                State.FAIL_DISABLE_ARM,
                State.DISABLE_ARM,
            }:
                response.success = False
                response.message = "cannot release base until arm shutdown completes"
                return response
            self._request_set_bool(self.hold_client, False, "release hold")
            self._transition(State.IDLE, "operator disabled")
        response.success = True
        response.message = self.state.name
        return response

    def _operator_trigger(self, _request, response):
        if not self.operator_trigger_required:
            response.success = False
            response.message = "operator trigger is disabled in automatic mode"
            return response
        if self.state != State.WAIT_TARGET:
            response.success = False
            response.message = f"pickup is not ready: state={self.state.name}"
            return response
        if self.detection_frames < self.required_frames:
            response.success = False
            response.message = (
                f"BOX NOT READY: {self.detection_frames}/{self.required_frames} "
                "fresh depth-confirmed frames"
            )
            return response
        self._transition(State.STOP_ROUTE, "operator B trigger accepted")
        response.success = True
        response.message = "box pickup accepted; base locked"
        return response

    def _observation(self, message):
        if self.state != State.WAIT_TARGET:
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        valid = observation_is_eligible(
            message, age, self.maximum_observation_age, self.maximum_depth
        )
        self.detection_frames = self.detection_frames + 1 if valid else 0
        if self.detection_frames >= self.required_frames:
            if not self.operator_trigger_required:
                self._transition(State.STOP_ROUTE, "fresh reachable target confirmed")

    def _hold_active(self, message):
        self.hold_active = bool(message.data)

    def _hold_ready(self, message):
        self.hold_ready = bool(message.data)

    def _odom(self, message, source):
        linear = message.twist.twist.linear
        speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        self.body_speed_samples[source] = (
            self.get_clock().now().nanoseconds,
            speed,
        )

    def _base_speed(self):
        return conservative_body_speed(
            self.body_speed_samples,
            self.get_clock().now().nanoseconds,
            self.body_odom_timeout,
        )

    def _publish_zero(self):
        self.command_pub.publish(Twist())

    def _fail(self, reason):
        if self.state in {
            State.REQUEST_ARM,
            State.ARM_ACTIVE,
            State.DISABLE_ARM,
        }:
            self._transition(State.FAIL_DISABLE_ARM, reason)
        else:
            self._transition(State.FAILED, reason)

    def _arm_status(self, message):
        self.arm_status = message.data

    def _call(self, client, request, label):
        if self.pending is not None:
            return False
        if not client.service_is_ready():
            return False
        self.pending = client.call_async(request)
        self.pending.add_done_callback(lambda future: self._service_done(future, label))
        return True

    def _request_set_bool(self, client, value, label):
        request = SetBool.Request()
        request.data = bool(value)
        return self._call(client, request, label)

    def _request_trigger(self, client, label):
        return self._call(client, Trigger.Request(), label)

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
        if self.state == State.STOP_ROUTE:
            self._transition(State.REQUEST_HOLD, "route command stopped")
        elif self.state == State.REQUEST_HOLD:
            self._transition(State.WAIT_HOLD, "MD hold requested")
        elif self.state == State.REQUEST_ARM:
            self._transition(State.ARM_ACTIVE, "base hold verified; arm enabled")
        elif self.state == State.FAIL_DISABLE_ARM:
            self._transition(
                State.FAILED,
                "arm auto control disabled; base zero/driver lock retained",
            )
        elif self.state == State.DISABLE_ARM:
            self._transition(
                State.DISABLE_BOX_VISION,
                "arm auto control disabled; switching vision profiles",
            )
        elif self.state == State.DISABLE_BOX_VISION:
            self._transition(
                State.ENABLE_TRAFFIC_VISION,
                "box YOLO disabled",
            )
        elif self.state == State.ENABLE_TRAFFIC_VISION:
            self._transition(
                State.RELEASE_HOLD,
                "traffic-light YOLO enabled",
            )
        elif self.state == State.RELEASE_HOLD:
            if self.resume_route:
                self._transition(State.RESUME_ROUTE, "base hold released")
            else:
                self._transition(State.COMPLETE, "pickup complete")
        elif self.state == State.RESUME_ROUTE:
            self._transition(State.COMPLETE, "pickup complete; route resumed")

    def _tick(self):
        stop_states = {
            State.STOP_ROUTE,
            State.REQUEST_HOLD,
            State.WAIT_HOLD,
            State.REQUEST_ARM,
            State.ARM_ACTIVE,
            State.FAIL_DISABLE_ARM,
            State.DISABLE_ARM,
            State.DISABLE_BOX_VISION,
            State.ENABLE_TRAFFIC_VISION,
            State.RELEASE_HOLD,
            State.RESUME_ROUTE,
            State.FAILED,
        }
        service_states = {
            State.STOP_ROUTE,
            State.REQUEST_HOLD,
            State.REQUEST_ARM,
            State.FAIL_DISABLE_ARM,
            State.DISABLE_ARM,
            State.DISABLE_BOX_VISION,
            State.ENABLE_TRAFFIC_VISION,
            State.RELEASE_HOLD,
            State.RESUME_ROUTE,
        }
        if self.state in stop_states:
            # Priority 80 in twist_mux: stop the navigation command immediately,
            # even before its action cancellation or HOLD service completes.
            self._publish_zero()
        if self.state in service_states and self._elapsed() > self.service_timeout:
            if self.state == State.FAIL_DISABLE_ARM:
                self._transition(
                    State.FAILED,
                    "arm disable service timeout; HOLD and zero command retained",
                )
            else:
                self._fail(f"{self.state.name} service timeout")
            return

        if self.state == State.STOP_ROUTE:
            if self.control_recorded_route:
                self._request_trigger(self.route_stop_client, "stop route")
            else:
                self._transition(State.REQUEST_HOLD, "manual base command locked")
        elif self.state == State.REQUEST_HOLD:
            self._request_set_bool(self.hold_client, True, "enable hold")
        elif self.state == State.WAIT_HOLD:
            if self._elapsed() > self.hold_timeout:
                self._fail("zero-RPM hold timeout; hold remains active")
            elif (
                self.hold_active
                and self.hold_ready
                and self._base_speed() <= self.maximum_stopped_speed
            ):
                self._transition(
                    State.REQUEST_ARM,
                    "base hold verified; requesting arm enable",
                )
        elif self.state == State.REQUEST_ARM:
            if (
                not self.hold_active
                or not self.hold_ready
                or self._base_speed() > self.maximum_stopped_speed
            ):
                self._fail("base HOLD/motion verification lost while enabling arm")
            else:
                self._request_set_bool(self.arm_client, True, "enable arm")
        elif self.state == State.ARM_ACTIVE:
            if (
                not self.hold_active
                or not self.hold_ready
                or self._base_speed() > self.maximum_stopped_speed
            ):
                self._fail("base HOLD/motion verification lost while arm was active")
            elif "state=GRASP_COMPLETE" in self.arm_status:
                self._transition(State.DISABLE_ARM, "grasp complete")
            elif "state=GRASP_FAILED" in self.arm_status:
                self._fail("grasp failed; hold remains active")
            # Deliberately no elapsed-time cutoff here. Visual alignment and
            # the arm's internal retry sequence may take an arbitrary amount
            # of time. Keep the base in verified zero-RPM HOLD until the arm
            # explicitly reports GRASP_COMPLETE or GRASP_FAILED.
        elif self.state == State.FAIL_DISABLE_ARM:
            self._request_set_bool(self.arm_client, False, "emergency disable arm")
        elif self.state == State.DISABLE_ARM:
            self._request_set_bool(self.arm_client, False, "disable arm")
        elif self.state == State.DISABLE_BOX_VISION:
            self._request_set_bool(
                self.box_perception_client, False, "disable box YOLO"
            )
        elif self.state == State.ENABLE_TRAFFIC_VISION:
            self._request_set_bool(
                self.traffic_vision_client, True, "enable traffic-light YOLO"
            )
        elif self.state == State.RELEASE_HOLD:
            self._request_set_bool(self.hold_client, False, "release hold")
        elif self.state == State.RESUME_ROUTE:
            if self.control_recorded_route:
                self._request_trigger(self.route_start_client, "resume route")
            else:
                self._transition(State.COMPLETE, "manual drive ready after LB release")

    def _publish_status(self):
        base_speed = self._base_speed()
        detail = (
            f"state={self.state.name};detections={self.detection_frames};"
            f"hold_active={self.hold_active};hold_ready={self.hold_ready};"
            f"base_speed_mps={base_speed:.3f}"
        )
        self.status_pub.publish(String(data=detail))
        if detail != self.last_status and self.state in {State.COMPLETE, State.FAILED}:
            self.get_logger().info(f"[MISSION-2] {detail}")
        self.last_status = detail


def main(args=None):
    rclpy.init(args=args)
    node = BoxPickCoordinator()
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
            rclpy.shutdown()
