"""Edge-triggered ABXY controls for one manual PK-3A mapping lap."""

from functools import partial
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def button_pressed(buttons, index):
    return 0 <= index < len(buttons) and bool(buttons[index])


def rising_edge(previous, current, index):
    return button_pressed(current, index) and not button_pressed(previous, index)


def mapping_drive_block_reason(
    state, require_preflight, preflight_ready, base_connected
):
    if state not in {"recording", "complete"}:
        return "mapping recording is neither active nor completely saved"
    if require_preflight and preflight_ready is not True:
        return "hardware preflight is NOT READY"
    if base_connected is not True:
        return "MD200T feedback is not connected"
    return ""


class XboxMappingControls(Node):
    def __init__(self):
        super().__init__("xbox_mapping_controls")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("mode_button", 3)  # Y
        self.declare_parameter("reset_button", 2)  # X
        self.declare_parameter("start_button", 0)  # A
        self.declare_parameter("finish_button", 1)  # B
        self.declare_parameter("drive_enable_button", 4)  # LB
        self.declare_parameter("minimum_action_interval_sec", 0.50)
        self.declare_parameter("joy_command_topic", "/cmd_vel_joy_raw")
        self.declare_parameter("motor_command_topic", "/cmd_vel")
        self.declare_parameter("record_sensor_bag", True)
        self.declare_parameter("start_3d_session", False)
        self.declare_parameter("require_preflight", True)
        self.declare_parameter("exit_after_completion", True)

        self.mode_button = int(self.get_parameter("mode_button").value)
        self.reset_button = int(self.get_parameter("reset_button").value)
        self.start_button = int(self.get_parameter("start_button").value)
        self.finish_button = int(self.get_parameter("finish_button").value)
        self.drive_enable_button = int(
            self.get_parameter("drive_enable_button").value
        )
        self.minimum_interval = float(
            self.get_parameter("minimum_action_interval_sec").value
        )
        self.motor_command_topic = str(
            self.get_parameter("motor_command_topic").value
        )
        self.joy_command_topic = str(
            self.get_parameter("joy_command_topic").value
        )
        self.record_sensor_bag = bool(
            self.get_parameter("record_sensor_bag").value
        )
        self.start_3d_session = bool(
            self.get_parameter("start_3d_session").value
        )
        self.require_preflight = bool(self.get_parameter("require_preflight").value)
        self.exit_after_completion = bool(
            self.get_parameter("exit_after_completion").value
        )
        self.previous_buttons = []
        self.previous_lb_pressed = False
        self.mode_enabled = False
        self.state = "idle"
        self.pending_action = ""
        self.last_action_time = 0.0
        self.last_lb_warning_time = 0.0
        self.joy_drive_active = False
        self.motor_drive_active = False
        self.manual_drive_authorized = False
        self.drive_ready_reported = False
        self.joy_drive_started_at = None
        self.drive_block_reported = False
        self.base_connected = None
        self.preflight_ready = None
        self.shutdown_pending = False

        self.start_client = self.create_client(Trigger, "/track_path_recorder/start")
        self.stop_client = self.create_client(
            Trigger, "/track_path_recorder/stop_and_save"
        )
        self.reset_client = self.create_client(Trigger, "/track_path_recorder/reset")
        self.bundle_client = self.create_client(Trigger, "/map_bundle_server/save")
        self.bag_start_client = self.create_client(
            Trigger, "/mapping_session_recorder/start"
        )
        self.bag_stop_client = self.create_client(
            Trigger, "/mapping_session_recorder/stop"
        )
        self.pgo_session_client = self.create_client(
            Trigger, "/pgo/start_session"
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String, "/mapping/control_status", status_qos
        )
        self.motor_command_pub = self.create_publisher(
            Twist, self.motor_command_topic, 10
        )
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self.joy_callback, 10
        )
        self.create_subscription(
            Twist, self.joy_command_topic, self.joy_command_callback, 10
        )
        self.create_subscription(
            Twist, self.motor_command_topic, self.motor_command_callback, 10
        )
        self.create_subscription(
            Bool, "/md/base_connected", self.base_connected_callback, 10
        )
        self.create_subscription(
            Bool,
            "/field/preflight_ready",
            self.preflight_callback,
            status_qos,
        )
        self.create_timer(0.10, self.check_drive_pipeline)
        self.publish_status("mapping buttons disabled; release LB and press Y to enable")

    def publish_status(self, message, error=False):
        status = String()
        status.data = message
        self.status_pub.publish(status)
        if error:
            self.get_logger().error(message)
        else:
            # Operator state changes stay visible with launch log_level=warn,
            # but are not duplicated in rosout as warning messages.
            print(f"[XBOX-MAPPING][STATUS] {message}", flush=True)

    @staticmethod
    def command_active(message):
        return abs(message.linear.x) > 1.0e-3 or abs(message.angular.z) > 1.0e-3

    def drive_block_reason(self):
        return mapping_drive_block_reason(
            self.state,
            self.require_preflight,
            self.preflight_ready,
            self.base_connected,
        )

    def publish_motor_stop(self):
        self.motor_command_pub.publish(Twist())

    def revoke_manual_drive(self, reason=""):
        was_authorized = self.manual_drive_authorized
        self.manual_drive_authorized = False
        self.publish_motor_stop()
        if was_authorized and reason:
            print(f"[XBOX-DRIVE][STOP] {reason}", flush=True)

    def joy_command_callback(self, message):
        active = self.command_active(message)
        if active != self.joy_drive_active:
            self.joy_drive_active = active
            if active:
                self.joy_drive_started_at = time.monotonic()
                self.drive_block_reported = False
            else:
                self.joy_drive_started_at = None
                self.drive_block_reported = False
        # This callback is a command gate, not only a state monitor.  Every
        # 20 Hz Twist must be forwarded; returning when the active boolean did
        # not change froze the first tiny dead-zone command at the MD input.
        block_reason = self.drive_block_reason()
        if self.manual_drive_authorized and not block_reason:
            self.motor_command_pub.publish(message)
        else:
            self.publish_motor_stop()
            if active and not self.drive_block_reported:
                self.drive_block_reported = True
                reason = block_reason or "release and press LB after hardware READY"
                print(f"[XBOX-DRIVE][BLOCKED] {reason}", flush=True)

    def motor_command_callback(self, message):
        active = self.command_active(message)
        if active == self.motor_drive_active:
            return
        self.motor_drive_active = active
        if active and not self.drive_ready_reported:
            self.drive_ready_reported = True
            print(
                f"[XBOX-DRIVE][READY] MD feedback connected; command forwarded: "
                f"linear={message.linear.x:.3f} m/s, angular={message.angular.z:.3f} rad/s",
                flush=True,
            )

    def base_connected_callback(self, message):
        connected = bool(message.data)
        if connected == self.base_connected:
            return
        self.base_connected = connected
        state = "connected" if connected else "disconnected"
        print(f"[XBOX-DRIVE][MD] base {state}", flush=True)
        if not connected:
            self.revoke_manual_drive("MD200T feedback disconnected; command set to zero")

    def preflight_callback(self, message):
        ready = bool(message.data)
        if ready == self.preflight_ready:
            return
        self.preflight_ready = ready
        if ready:
            self.publish_status("hardware preflight READY")
        else:
            self.revoke_manual_drive("hardware preflight became NOT READY")
            # FIELD-CHECK already prints the exact failed devices and values.
            # Publish the lock state for UI consumers without duplicating a
            # second generic terminal error.
            status = String()
            status.data = "hardware preflight NOT READY"
            self.status_pub.publish(status)

    def check_drive_pipeline(self):
        if not self.manual_drive_authorized:
            return
        if (
            self.joy_drive_active
            and not self.motor_drive_active
            and not self.drive_block_reported
            and self.joy_drive_started_at is not None
            and time.monotonic() - self.joy_drive_started_at >= 0.50
        ):
            self.drive_block_reported = True
            print(
                "[XBOX-DRIVE][BLOCKED] authorized Xbox command did not reach "
                f"the MD input {self.motor_command_topic}",
                flush=True,
            )

    def joy_callback(self, message):
        current = list(message.buttons)
        now = time.monotonic()
        lb_pressed = button_pressed(current, self.drive_enable_button)
        mode_rising = rising_edge(self.previous_buttons, current, self.mode_button)
        reset_rising = rising_edge(self.previous_buttons, current, self.reset_button)
        start_rising = rising_edge(self.previous_buttons, current, self.start_button)
        finish_rising = rising_edge(self.previous_buttons, current, self.finish_button)
        self.previous_buttons = current

        if lb_pressed != self.previous_lb_pressed:
            if lb_pressed:
                self.drive_ready_reported = False
                reason = self.drive_block_reason()
                self.manual_drive_authorized = not bool(reason)
                if self.manual_drive_authorized:
                    state = "pressed: manual drive enabled"
                else:
                    state = f"pressed: manual drive BLOCKED; {reason}"
                    self.publish_motor_stop()
            else:
                self.revoke_manual_drive()
                self.drive_ready_reported = False
                state = "released: manual drive stopped"
            print(f"[XBOX-DRIVE][LB] {state}", flush=True)
            self.previous_lb_pressed = lb_pressed

        any_control = mode_rising or reset_rising or start_rising or finish_rising
        if not any_control:
            return
        if lb_pressed:
            if now - self.last_lb_warning_time >= 2.0:
                self.publish_status("mapping button ignored: release LB first")
                self.last_lb_warning_time = now
            return
        if self.pending_action:
            self.publish_status(
                f"mapping button ignored: {self.pending_action} is still running"
            )
            return
        if now - self.last_action_time < self.minimum_interval:
            wait_time = self.minimum_interval - (now - self.last_action_time)
            self.publish_status(
                f"button ignored: wait {max(0.0, wait_time):.1f}s before the next action"
            )
            return
        self.last_action_time = now

        # The first B press finishes and saves the map.  Once the bundle is
        # complete, a second B press is the operator's explicit request to stop
        # manual driving and close the field stack.  This remains available
        # even if Y mapping-button mode was toggled off after saving.
        if finish_rising and self.state == "complete":
            self.revoke_manual_drive("operator requested mapping-session exit")
            self.publish_status("saved map retained; closing after final motor stop")
            self.shutdown_pending = True
            self.create_timer(0.50, self.shutdown_after_completion)
            return

        if mode_rising:
            self.mode_enabled = not self.mode_enabled
            state = "enabled" if self.mode_enabled else "disabled"
            self.publish_status(f"mapping buttons {state}")
            return
        if not self.mode_enabled:
            return
        if self.state == "complete" and (reset_rising or start_rising):
            self.publish_status(
                "mapping is already saved; use LB to drive or press B to exit"
            )
            return
        if reset_rising:
            if self.state == "recording":
                self.publish_status("reset rejected: press B to finish recording first")
                return
            self.call_service(self.reset_client, "reset", self.reset_finished)
        elif start_rising:
            if self.state == "recording":
                self.publish_status("start ignored: path recording is already active")
                return
            if self.require_preflight and self.preflight_ready is not True:
                self.publish_status(
                    "start rejected: hardware preflight has not passed", True
                )
                return
            if self.start_3d_session:
                # FAST-LIO/SC-PGO is already warm when A is pressed. Mark the
                # exact mapping epoch before rosbag and centerline recording so
                # warm-up drift cannot be committed as driven track geometry.
                self.call_service(
                    self.pgo_session_client,
                    "start_3d_session",
                    self.pgo_session_started,
                )
            elif self.record_sensor_bag:
                self.call_service(
                    self.bag_start_client, "start_sensor_bag", self.bag_started
                )
            else:
                self.start_path_recording()
        elif finish_rising:
            if self.state == "recording":
                self.call_service(self.stop_client, "finish_path", self.path_finished)
            elif self.state == "path_saved":
                self.publish_status(
                    "finish rejected: sensor finalization failed; restart mapping"
                )
            else:
                self.publish_status("finish rejected: press A to start mapping first")

    def call_service(self, client, label, callback):
        if not client.service_is_ready():
            self.publish_status(f"{label} failed: required service is unavailable", True)
            return
        self.pending_action = label
        future = client.call_async(Trigger.Request())
        future.add_done_callback(partial(self.service_finished, callback=callback))

    def service_finished(self, future, callback):
        label = self.pending_action
        self.pending_action = ""
        try:
            response = future.result()
        except Exception as error:
            self.publish_status(f"{label} failed: {error}", True)
            self.action_failed(label)
            return
        if response is None or not response.success:
            detail = response.message if response is not None else "no response"
            self.publish_status(f"{label} failed: {detail}", True)
            self.action_failed(label)
            return
        callback(response.message)

    def action_failed(self, label):
        self.revoke_manual_drive("mapping action failed; command set to zero")
        # If rosbag started but the path recorder did not, stop/finalize it
        # immediately rather than recreating the former unbounded recorder.
        if label == "start_path" and self.record_sensor_bag:
            self.call_service(
                self.bag_stop_client,
                "rollback_sensor_bag",
                lambda detail: self.publish_status(
                    f"failed mapping start was finalized; {detail}"
                ),
            )

    def reset_finished(self, detail):
        self.revoke_manual_drive()
        self.state = "idle"
        self.publish_status(f"mapping path reset; {detail}")

    def start_finished(self, detail):
        self.state = "recording"
        self.publish_status(f"mapping path recording started; {detail}")

    def bag_started(self, detail):
        self.publish_status(f"sensor recording started; {detail}")
        self.start_path_recording()

    def pgo_session_started(self, detail):
        self.publish_status(f"3D mapping epoch started; {detail}")
        if self.record_sensor_bag:
            self.call_service(
                self.bag_start_client, "start_sensor_bag", self.bag_started
            )
        else:
            self.start_path_recording()

    def start_path_recording(self):
        self.call_service(self.start_client, "start_path", self.start_finished)

    def path_finished(self, detail):
        self.revoke_manual_drive("mapping recording finished; command set to zero")
        self.state = "path_saved"
        if self.record_sensor_bag:
            self.publish_status(f"mapping path saved; {detail}; finalizing sensor bag")
            self.call_service(
                self.bag_stop_client, "stop_sensor_bag", self.bag_stopped
            )
        else:
            self.publish_status(f"mapping path saved; {detail}; saving map bundle")
            self.start_bundle_save()

    def bag_stopped(self, detail):
        self.publish_status(f"sensor recording finalized; {detail}; validating map bundle")
        self.start_bundle_save()

    def start_bundle_save(self):
        self.call_service(self.bundle_client, "save_bundle", self.bundle_finished)

    def bundle_finished(self, detail):
        self.state = "complete"
        if self.exit_after_completion:
            self.mode_enabled = False
            self.publish_status(f"mapping complete; {detail}")
            self.shutdown_pending = True
            self.create_timer(0.50, self.shutdown_after_completion)
            return
        self.publish_status(
            f"mapping complete; {detail}; arm hold remains locked; "
            "LB manual drive remains available; release LB and press B again to exit"
        )

    def shutdown_after_completion(self):
        if not self.shutdown_pending:
            return
        self.shutdown_pending = False
        print("[XBOX-MAPPING][STATUS] closing completed mapping launch", flush=True)
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = XboxMappingControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
