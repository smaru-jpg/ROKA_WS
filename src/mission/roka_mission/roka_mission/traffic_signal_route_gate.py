#!/usr/bin/env python3
"""Latch a mapped route at RED and resume it only after confirmed BLUE."""

from enum import Enum, auto
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SignalDecision(Enum):
    NONE = auto()
    STOP = auto()
    GO = auto()


def parse_result_labels(value):
    """Return normalized class names from the mission-vision String payload."""
    return {
        token.strip().upper()
        for token in str(value).split(",")
        if token.strip() and token.strip().upper() != "NONE"
    }


class SignalDecisionLatch:
    """Convert noisy class results into red-stop/blue-go edge decisions.

    A missing detection never changes the last accepted traffic state.  RED has
    priority if a future multi-class detector reports both labels in one frame.
    """

    def __init__(
        self,
        stop_labels=("RED",),
        go_labels=("BLUE",),
        stop_confirmations=1,
        go_confirmations=2,
    ):
        self.stop_labels = {str(value).strip().upper() for value in stop_labels}
        self.go_labels = {str(value).strip().upper() for value in go_labels}
        self.stop_confirmations = int(stop_confirmations)
        self.go_confirmations = int(go_confirmations)
        if not self.stop_labels or not self.go_labels:
            raise ValueError("stop_labels and go_labels must not be empty")
        if self.stop_labels & self.go_labels:
            raise ValueError("stop_labels and go_labels must not overlap")
        if self.stop_confirmations < 1 or self.go_confirmations < 1:
            raise ValueError("signal confirmation counts must be positive")
        self.stop_streak = 0
        self.go_streak = 0
        self.stop_latched = False

    def update(self, value):
        labels = parse_result_labels(value)
        if labels & self.stop_labels:
            self.stop_streak += 1
            self.go_streak = 0
            if self.stop_streak >= self.stop_confirmations and not self.stop_latched:
                self.stop_latched = True
                return SignalDecision.STOP
            return SignalDecision.NONE
        if labels & self.go_labels:
            self.go_streak += 1
            self.stop_streak = 0
            if self.go_streak >= self.go_confirmations and self.stop_latched:
                self.stop_latched = False
                return SignalDecision.GO
            return SignalDecision.NONE
        self.stop_streak = 0
        self.go_streak = 0
        return SignalDecision.NONE


def requests_manual_takeover(status):
    """Recognize the latched B-stop status without matching the startup hint."""
    normalized = str(status).strip().lower()
    return normalized.startswith("b stop:") or normalized.startswith(
        "manual takeover stopped:"
    ) or normalized.startswith("manual takeover active:")


class TrafficSignalRouteGate(Node):
    """Own the high-priority zero command while a traffic RED is latched."""

    def __init__(self):
        super().__init__("traffic_signal_route_gate")
        self.declare_parameter("result_topic", "/roka/vision/result")
        self.declare_parameter("command_topic", "/cmd_vel_mission")
        self.declare_parameter("status_topic", "/roka/mission/traffic/status")
        self.declare_parameter("stop_active_topic", "/roka/mission/traffic/stop_active")
        self.declare_parameter("route_stop_service", "/track_path_follower/stop")
        self.declare_parameter("route_start_service", "/track_path_follower/start")
        self.declare_parameter("autonomy_status_topic", "/autonomy/control_status")
        self.declare_parameter("stop_labels", ["RED"])
        # The deployed model intentionally calls the competition go lamp BLUE.
        self.declare_parameter("go_labels", ["BLUE"])
        self.declare_parameter("stop_confirmations", 1)
        self.declare_parameter("go_confirmations", 2)
        self.declare_parameter("zero_publish_rate_hz", 20.0)
        self.declare_parameter("service_retry_sec", 0.50)
        self.declare_parameter("service_timeout_sec", 2.0)

        zero_rate = float(self.get_parameter("zero_publish_rate_hz").value)
        self.service_retry = float(self.get_parameter("service_retry_sec").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        if zero_rate <= 0.0 or self.service_retry <= 0.0 or self.service_timeout <= 0.0:
            raise ValueError("traffic route-gate timing parameters must be positive")

        self.decision_latch = SignalDecisionLatch(
            stop_labels=self.get_parameter("stop_labels").value,
            go_labels=self.get_parameter("go_labels").value,
            stop_confirmations=int(self.get_parameter("stop_confirmations").value),
            go_confirmations=int(self.get_parameter("go_confirmations").value),
        )
        self.desired_stop = False
        self.motion_blocked = False
        self.stop_service_confirmed = False
        self.stop_request_issued = False
        self.manual_takeover = False
        self.pending = None
        self.pending_action = ""
        self.pending_started = 0.0
        self.last_request = 0.0
        self.request_serial = 0
        self.last_status = ""

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), latched_qos
        )
        self.stop_active_pub = self.create_publisher(
            Bool, str(self.get_parameter("stop_active_topic").value), latched_qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter("result_topic").value),
            self._vision_result,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("autonomy_status_topic").value),
            self._autonomy_status,
            latched_qos,
        )
        self.stop_client = self.create_client(
            Trigger, str(self.get_parameter("route_stop_service").value)
        )
        self.start_client = self.create_client(
            Trigger, str(self.get_parameter("route_start_service").value)
        )
        self.create_timer(1.0 / zero_rate, self._tick)
        self._publish_status("DRIVE", "waiting for a confirmed RED")
        self.get_logger().info(
            "[TRAFFIC] ready: RED stops and latches; BLUE resumes after confirmation"
        )

    def _publish_status(self, state, detail):
        text = f"state={state};{detail}"
        self.stop_active_pub.publish(Bool(data=self.motion_blocked))
        if text == self.last_status:
            return
        self.last_status = text
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[TRAFFIC] {text}")

    def _vision_result(self, message):
        decision = self.decision_latch.update(message.data)
        if decision == SignalDecision.STOP:
            self.desired_stop = True
            self.motion_blocked = True
            self.stop_service_confirmed = False
            self.stop_request_issued = False
            self.request_serial += 1
            self._publish_zero()
            self._publish_status("STOPPING", "confirmed=RED; route cancellation pending")
        elif decision == SignalDecision.GO:
            self.desired_stop = False
            self.stop_service_confirmed = False
            self.request_serial += 1
            if self.manual_takeover:
                self._publish_status(
                    "MANUAL",
                    "confirmed=BLUE; autonomous restart suppressed by operator B-stop",
                )
            elif not self.stop_request_issued and self.pending_action != "stop":
                # RED may disappear before the route-stop service is available.
                # The high-priority zero was still applied, but an already-active
                # route does not need (and rejects) a duplicate start request.
                self.motion_blocked = False
                self._publish_status("DRIVE", "BLUE confirmed before route cancellation")
            else:
                self._publish_status("STARTING", "confirmed=BLUE; route restart pending")

    def _autonomy_status(self, message):
        if not requests_manual_takeover(message.data):
            return
        if self.manual_takeover:
            return
        self.manual_takeover = True
        # A traffic BLUE that races with the operator's B press must never
        # restart the route after the Xbox controller's own stop request.
        self.desired_stop = True
        self.motion_blocked = True
        self.stop_service_confirmed = False
        self.request_serial += 1
        self._publish_zero()
        self._publish_status(
            "MANUAL", "operator B-stop latched; traffic signal cannot restart autonomy"
        )

    def _publish_zero(self):
        self.command_pub.publish(Twist())

    def _request(self, action):
        now = time.monotonic()
        if self.pending is not None or now - self.last_request < self.service_retry:
            return
        client = self.stop_client if action == "stop" else self.start_client
        if not client.service_is_ready():
            return
        self.last_request = now
        self.pending_action = action
        self.pending_started = now
        if action == "stop":
            self.stop_request_issued = True
        serial = self.request_serial
        self.pending = client.call_async(Trigger.Request())
        self.pending.add_done_callback(
            lambda future: self._service_done(future, action, serial)
        )

    def _service_done(self, future, action, serial):
        # A timed-out request may complete after its replacement was sent.
        # Never let the old callback clear or complete the newer request.
        if future is not self.pending:
            return
        self.pending = None
        self.pending_action = ""
        stale = serial != self.request_serial
        try:
            response = future.result()
            success = response is not None and bool(response.success)
            detail = response.message if response is not None else "no response"
        except Exception as error:
            success = False
            detail = str(error)

        if stale:
            # A RED may arrive while a start request is in flight.  Keep the
            # high-priority zero active and let the timer issue a fresh stop.
            return
        if not success:
            self._publish_status(
                "STOPPING" if self.desired_stop else "STARTING",
                f"route {action} rejected ({detail}); retrying",
            )
            return
        if action == "stop":
            self.motion_blocked = True
            self.stop_service_confirmed = True
            if self.manual_takeover:
                self._publish_status(
                    "MANUAL", "operator B-stop confirmed; LB manual drive has priority"
                )
            else:
                self._publish_status("RED_HOLD", "route stopped; waiting for BLUE")
        elif not self.desired_stop and not self.manual_takeover:
            self.motion_blocked = False
            self.stop_request_issued = False
            self._publish_status("DRIVE", "BLUE confirmed; route resumed")

    def _tick(self):
        now = time.monotonic()
        if self.pending is not None and now - self.pending_started >= self.service_timeout:
            try:
                self.pending.cancel()
            except Exception:
                pass
            action = self.pending_action
            self.pending = None
            self.pending_action = ""
            self.request_serial += 1
            self._publish_status(
                "STOPPING" if self.desired_stop else "STARTING",
                f"route {action} service timed out; retrying",
            )

        if self.motion_blocked:
            self._publish_zero()
        if self.desired_stop and not self.stop_service_confirmed:
            self._request("stop")
        elif self.motion_blocked and not self.manual_takeover:
            self._request("start")

    def destroy_node(self):
        if self.motion_blocked:
            self._publish_zero()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignalRouteGate()
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
