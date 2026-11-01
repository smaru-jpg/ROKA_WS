"""Fail-closed Xbox base teleop with an LT turbo mode."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


def shaped_axis(value, deadzone):
    value = float(value)
    deadzone = max(0.0, min(0.95, float(deadzone)))
    magnitude = abs(value)
    if not math.isfinite(value) or magnitude <= deadzone:
        return 0.0
    scaled = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    return math.copysign(scaled, value)


def xbox_twist_values(
    axes,
    buttons,
    *,
    linear_axis=1,
    angular_axis=0,
    enable_button=4,
    turbo_axis=2,
    turbo_pressed_threshold=-0.5,
    normal_linear=0.30,
    turbo_linear=1.00,
    angular_scale=0.60,
    deadzone=0.05,
):
    enabled = 0 <= enable_button < len(buttons) and bool(buttons[enable_button])
    if not enabled:
        return 0.0, 0.0, False, False
    if not (0 <= linear_axis < len(axes) and 0 <= angular_axis < len(axes)):
        return 0.0, 0.0, False, False
    turbo = (
        0 <= turbo_axis < len(axes)
        and math.isfinite(float(axes[turbo_axis]))
        and float(axes[turbo_axis]) <= turbo_pressed_threshold
    )
    linear_limit = turbo_linear if turbo else normal_linear
    linear = shaped_axis(axes[linear_axis], deadzone) * linear_limit
    angular = shaped_axis(axes[angular_axis], deadzone) * angular_scale
    return linear, angular, True, turbo


class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__("roka_xbox_teleop")
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.cmd_topic = str(
            self.declare_parameter("cmd_vel_topic", "/cmd_vel_joy_raw").value
        )
        self.linear_axis = int(self.declare_parameter("linear_axis", 1).value)
        self.angular_axis = int(self.declare_parameter("angular_axis", 0).value)
        self.enable_button = int(self.declare_parameter("enable_button", 4).value)
        self.turbo_axis = int(self.declare_parameter("turbo_axis", 2).value)
        self.turbo_threshold = float(
            self.declare_parameter("turbo_pressed_threshold", -0.5).value
        )
        self.normal_linear = float(
            self.declare_parameter("normal_linear_speed_mps", 0.30).value
        )
        self.turbo_linear = float(
            self.declare_parameter("turbo_linear_speed_mps", 1.00).value
        )
        self.angular_scale = float(
            self.declare_parameter("angular_speed_rps", 0.60).value
        )
        self.deadzone = float(self.declare_parameter("axis_deadzone", 0.05).value)
        self.timeout = float(self.declare_parameter("joy_timeout_sec", 0.30).value)
        self.rate = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        if (
            min(self.linear_axis, self.angular_axis, self.enable_button, self.turbo_axis) < 0
            or self.normal_linear <= 0.0
            or self.turbo_linear < self.normal_linear
            or self.angular_scale <= 0.0
            or self.timeout <= 0.0
            or self.rate <= 0.0
        ):
            raise ValueError("invalid Xbox teleop parameters")
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self._joy, 20)
        self.create_timer(1.0 / self.rate, self._publish)
        self.latest = None
        self.latest_time = 0.0
        self.last_mode = ""
        self.get_logger().info(
            f"Xbox base: LB normal {self.normal_linear:.2f} m/s, "
            f"LB+LT turbo {self.turbo_linear:.2f} m/s, output={self.cmd_topic}"
        )

    def _joy(self, message):
        self.latest = message
        self.latest_time = time.monotonic()

    def _publish(self):
        message = Twist()
        mode = "STOP"
        if self.latest is not None and time.monotonic() - self.latest_time <= self.timeout:
            linear, angular, enabled, turbo = xbox_twist_values(
                self.latest.axes,
                self.latest.buttons,
                linear_axis=self.linear_axis,
                angular_axis=self.angular_axis,
                enable_button=self.enable_button,
                turbo_axis=self.turbo_axis,
                turbo_pressed_threshold=self.turbo_threshold,
                normal_linear=self.normal_linear,
                turbo_linear=self.turbo_linear,
                angular_scale=self.angular_scale,
                deadzone=self.deadzone,
            )
            message.linear.x = linear
            message.angular.z = angular
            mode = "TURBO" if enabled and turbo else ("NORMAL" if enabled else "STOP")
        self.publisher.publish(message)
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(f"Xbox drive mode: {mode}")

    def stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy's SIGINT handler may invalidate the context before spin()
        # returns. The periodic timeout already fails closed while running;
        # publish the final zero only while the context can still accept it.
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
