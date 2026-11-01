#!/usr/bin/env python3
"""Forward base commands only while the arm transport hold is confirmed."""

from __future__ import annotations

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class ArmHoldGate(Node):
    def __init__(self):
        super().__init__("arm_hold_cmd_gate")
        self.input_topic = str(self.declare_parameter(
            "input_topic", "/cmd_vel_arm_pending"
        ).value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/cmd_vel"
        ).value)
        self.require_field = bool(self.declare_parameter(
            "require_field_preflight", False
        ).value)
        self.input_timeout = float(self.declare_parameter(
            "input_timeout_sec", 0.30
        ).value)
        self.external_lock_topic = str(self.declare_parameter(
            "external_lock_topic", ""
        ).value).strip()
        self.require_enable_release = bool(self.declare_parameter(
            "require_enable_release", False
        ).value)
        self.joy_topic = str(self.declare_parameter(
            "joy_topic", "/joy"
        ).value)
        self.enable_button = int(self.declare_parameter(
            "enable_button", 4
        ).value)
        if self.input_timeout <= 0.0:
            raise ValueError("input_timeout_sec must be positive")
        if self.enable_button < 0:
            raise ValueError("enable_button cannot be negative")
        self.arm_ready = False
        self.field_ready = False
        self.external_locked = False
        self.release_required = self.require_enable_release
        self.last_input = 0.0
        self.forwarding = False
        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, "/roka/arm/hold_ready", self._arm_ready, latched
        )
        if self.require_field:
            self.create_subscription(
                Bool, "/field/preflight_ready", self._field_ready, latched
            )
        if self.external_lock_topic:
            self.create_subscription(
                Bool, self.external_lock_topic, self._external_lock, latched
            )
        if self.require_enable_release:
            self.create_subscription(Joy, self.joy_topic, self._joy, 20)
        self.create_subscription(Twist, self.input_topic, self._command, 10)
        self.create_timer(0.10, self._watchdog)
        print("[ARM-HOLD][DRIVE] 시작 자세 잠금 확인 전 베이스 명령 차단", flush=True)

    def _gate_ready(self) -> bool:
        return bool(
            self.arm_ready
            and (self.field_ready or not self.require_field)
            and not self.external_locked
            and not self.release_required
        )

    def _zero(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        try:
            self.publisher.publish(Twist())
        except RCLError:
            if rclpy.ok(context=self.context):
                raise

    def _arm_ready(self, message: Bool) -> None:
        self.arm_ready = bool(message.data)
        if not self.arm_ready and self.require_enable_release:
            self.release_required = True
        self._update_state()

    def _field_ready(self, message: Bool) -> None:
        self.field_ready = bool(message.data)
        self._update_state()

    def _external_lock(self, message: Bool) -> None:
        self.external_locked = bool(message.data)
        if self.external_locked and self.require_enable_release:
            self.release_required = True
        self._update_state()

    def _joy(self, message: Joy) -> None:
        pressed = (
            self.enable_button < len(message.buttons)
            and bool(message.buttons[self.enable_button])
        )
        if not pressed and self.release_required:
            self.release_required = False
            self._update_state()

    def _update_state(self) -> None:
        ready = self._gate_ready()
        if ready == self.forwarding:
            return
        self.forwarding = ready
        if ready:
            print("[ARM-HOLD][DRIVE] 팔 잠금/LB 재활성 확인, 베이스 명령 허용", flush=True)
        else:
            self._zero()
            print("[ARM-HOLD][DRIVE] 팔/하드웨어 준비 해제, 베이스 정지", flush=True)

    def _command(self, message: Twist) -> None:
        self.last_input = time.monotonic()
        if self._gate_ready():
            self.publisher.publish(message)
        else:
            self._zero()

    def _watchdog(self) -> None:
        if time.monotonic() - self.last_input > self.input_timeout:
            self._zero()

    def close(self) -> None:
        self._zero()


def main(args=None):
    rclpy.init(args=args)
    node = ArmHoldGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
