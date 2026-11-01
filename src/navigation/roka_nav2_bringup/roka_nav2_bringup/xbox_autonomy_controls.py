"""Xbox start/stop controls for an approved PK-3A autonomous path."""

from functools import partial
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from roka_nav2_bringup.xbox_mapping_controls import button_pressed, rising_edge


class XboxAutonomyControls(Node):
    def __init__(self):
        super().__init__("xbox_autonomy_controls")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("mode_button", 3)  # Y
        self.declare_parameter("start_button", 0)  # A
        self.declare_parameter("stop_button", 1)  # B, always accepted
        self.declare_parameter("drive_enable_button", 4)  # LB must be released
        self.declare_parameter("minimum_action_interval_sec", 0.50)
        self.declare_parameter("manual_input_topic", "/cmd_vel_joy_raw")
        self.declare_parameter("manual_command_topic", "/cmd_vel_joy")
        self.declare_parameter("gate_manual_commands", False)
        self.declare_parameter("allow_autonomy_start_controls", True)
        self.declare_parameter("track_stop_service_enabled", True)
        self.declare_parameter("mission5_stop_service_enabled", False)

        self.mode_button = int(self.get_parameter("mode_button").value)
        self.start_button = int(self.get_parameter("start_button").value)
        self.stop_button = int(self.get_parameter("stop_button").value)
        self.drive_enable_button = int(
            self.get_parameter("drive_enable_button").value
        )
        self.minimum_interval = float(
            self.get_parameter("minimum_action_interval_sec").value
        )
        self.allow_start_controls = bool(
            self.get_parameter("allow_autonomy_start_controls").value
        )
        self.track_stop_enabled = bool(
            self.get_parameter("track_stop_service_enabled").value
        )
        self.mission5_stop_enabled = bool(
            self.get_parameter("mission5_stop_service_enabled").value
        )
        self.gate_manual_commands = bool(
            self.get_parameter("gate_manual_commands").value
        )
        self.previous_buttons = []
        self.drive_enable_pressed = False
        self.manual_takeover = False
        self.manual_enable_armed = False
        self.mode_enabled = False
        self.pending_action = ""
        self.action_serial = 0
        self.last_action_time = 0.0
        self.last_status_message = ""

        self.start_client = self.create_client(Trigger, "/track_path_follower/start")
        self.stop_client = self.create_client(Trigger, "/track_path_follower/stop")
        self.mission5_stop_client = self.create_client(
            SetBool, "/mission5/set_enabled"
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String, "/autonomy/control_status", status_qos
        )
        self.manual_stop_pub = self.create_publisher(
            Twist, str(self.get_parameter("manual_command_topic").value), 10
        )
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self.joy_callback, 10
        )
        if self.gate_manual_commands:
            self.create_subscription(
                Twist,
                str(self.get_parameter("manual_input_topic").value),
                self.manual_command_callback,
                10,
            )
        self.create_timer(0.05, self.manual_stop_watchdog)
        if self.allow_start_controls:
            self.publish_status(
                "autonomy stopped; use Y then A to start, or hold LB for manual drive"
            )
        else:
            self.publish_status("B emergency/manual takeover ready; Y/A owned by mission")

    def publish_status(self, message, error=False):
        # Button autorepeat or repeated operator presses must not flood the
        # field terminal with the same line. A changed state/error is still
        # printed immediately and retained on the status topic.
        if message == self.last_status_message:
            return
        self.last_status_message = message
        status = String()
        status.data = message
        self.status_pub.publish(status)
        if error:
            self.get_logger().error(message)
        else:
            print(f"[XBOX-AUTONOMY][STATUS] {message}", flush=True)

    def joy_callback(self, message):
        current = list(message.buttons)
        now = time.monotonic()
        mode_rising = rising_edge(self.previous_buttons, current, self.mode_button)
        start_rising = rising_edge(self.previous_buttons, current, self.start_button)
        stop_rising = rising_edge(self.previous_buttons, current, self.stop_button)
        self.previous_buttons = current
        self.drive_enable_pressed = button_pressed(current, self.drive_enable_button)

        # B is an unconditional, latched software stop even if the mode is not
        # armed.  Re-publishing zero at the joystick mux priority prevents a
        # blind-segment or mission publisher from taking control back while an
        # asynchronous cancellation is still propagating.
        if stop_rising:
            self.manual_takeover = True
            self.manual_enable_armed = False
            self.mode_enabled = False
            self.action_serial += 1
            self.pending_action = ""
            self.manual_stop_pub.publish(Twist())
            requested = False
            if self.track_stop_enabled and self.stop_client.service_is_ready():
                requested = True
                future = self.stop_client.call_async(Trigger.Request())
                future.add_done_callback(self.takeover_stop_finished)
            if self.mission5_stop_enabled and self.mission5_stop_client.service_is_ready():
                requested = True
                request = SetBool.Request()
                request.data = False
                future = self.mission5_stop_client.call_async(request)
                future.add_done_callback(self.takeover_stop_finished)
            suffix = "cancellation requested" if requested else "zero command latched"
            self.publish_status(
                f"B STOP: {suffix}; manual drive ready: hold LB and use the left stick"
            )
            return
        if not mode_rising and not start_rising:
            return
        if not self.allow_start_controls:
            self.publish_status("Y/A ignored: this section is controlled by its mission node")
            return
        if self.drive_enable_pressed:
            self.publish_status("autonomy button ignored: release LB first")
            return
        if self.pending_action:
            self.publish_status(
                f"autonomy button ignored: {self.pending_action} is still running"
            )
            return
        if now - self.last_action_time < self.minimum_interval:
            return
        self.last_action_time = now

        if mode_rising:
            # Y is an idempotent ARM command. Repeated Y presses previously
            # alternated enabled/disabled and made the following A look broken.
            # Only B disarms autonomous control.
            if not self.mode_enabled:
                self.mode_enabled = True
                self.publish_status("autonomy buttons enabled")
            return
        if start_rising:
            if not self.mode_enabled:
                self.publish_status("autonomy start rejected: press Y to enable first")
                return
            self.call_service(self.start_client, "start", self.start_finished)

    def call_service(self, client, label, callback, replace=False):
        if not client.service_is_ready():
            self.publish_status(f"autonomy {label} failed: service unavailable", True)
            return
        if self.pending_action and not replace:
            return
        self.action_serial += 1
        action_serial = self.action_serial
        self.pending_action = label
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            partial(
                self.service_finished,
                label=label,
                callback=callback,
                action_serial=action_serial,
            )
        )

    def service_finished(self, future, label, callback, action_serial):
        # B may replace an in-flight A/start request.  Ignore the stale start
        # response so it cannot announce or re-arm autonomy after cancellation.
        if action_serial != self.action_serial:
            return
        if self.pending_action == label:
            self.pending_action = ""
        try:
            response = future.result()
        except Exception as error:
            self.publish_status(f"autonomy {label} failed: {error}", True)
            return
        if response is None or not response.success:
            detail = response.message if response is not None else "no response"
            self.publish_status(f"autonomy {label} failed: {detail}", True)
            return
        callback(response.message)

    def start_finished(self, detail):
        self.manual_takeover = False
        self.publish_status(f"autonomy running; {detail}; press B to stop")

    def stop_finished(self, detail):
        self.mode_enabled = False
        self.publish_status(
            f"autonomy cancelled; {detail}; manual drive ready: hold LB and use the left stick"
        )

    def takeover_stop_finished(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.publish_status(
                f"B STOP remains latched; cancellation response failed: {error}", True
            )
            return
        if response is not None and not response.success:
            self.publish_status(
                f"B STOP remains latched; cancellation rejected: {response.message}", True
            )

    def manual_stop_watchdog(self):
        if not self.manual_takeover:
            return
        if self.manual_enable_armed and self.drive_enable_pressed:
            self.publish_status(
                "manual takeover active: LB + stick; hold LT for 1.00 m/s turbo"
            )
            return
        if not self.drive_enable_pressed:
            self.manual_enable_armed = True
        self.manual_stop_pub.publish(Twist())
        self.publish_status(
            "manual takeover stopped: release then hold LB to drive manually"
        )

    def manual_command_callback(self, message):
        """Own the mux input after B; never race a teleop publisher on it."""
        if not self.gate_manual_commands:
            return
        if not (
            self.manual_takeover
            and self.manual_enable_armed
            and self.drive_enable_pressed
        ):
            return
        self.manual_stop_pub.publish(message)
        self.publish_status(
            "manual takeover active: LB + stick; hold LT for 1.00 m/s turbo"
        )


def main(args=None):
    rclpy.init(args=args)
    node = XboxAutonomyControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
