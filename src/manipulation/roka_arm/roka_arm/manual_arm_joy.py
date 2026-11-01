#!/usr/bin/env python3
"""Optional Xbox position control for the existing ROKA Dynamixel arm.

This node does not replace the proven vision/grasp controller.  It owns the
same serial buses only in the explicit ``roka manual`` mode, starts from the
existing transport pose, and leaves Dynamixel Torque Enable latched at the
last commanded position when the node exits.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from . import arm_config as cfg
from .dynamixel_driver import DynamixelDriver


@dataclass(frozen=True)
class ManualTargets:
    joint_13: int
    joint_11: int
    wrist_4: int


def shaped_axis(value: float, deadzone: float) -> float:
    """Apply a continuous deadzone and keep the result in [-1, 1]."""
    value = max(-1.0, min(1.0, float(value)))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return scaled if value > 0.0 else -scaled


def _clamp_int(value: float, limits: tuple[int, int]) -> int:
    return int(round(max(limits[0], min(limits[1], value))))


def integrate_targets(
    targets: ManualTargets,
    *,
    joint_13_axis: float,
    joint_11_axis: float,
    wrist_axis: float,
    dt: float,
    deadzone: float,
    joint_13_rate: float,
    joint_11_rate: float,
    wrist_rate: float,
    joint_13_sign: float = 1.0,
    joint_11_sign: float = 1.0,
    wrist_sign: float = 1.0,
) -> ManualTargets:
    """Integrate joystick speed into bounded absolute position targets."""
    dt = max(0.0, min(float(dt), 0.20))
    joint_13_limits = cfg.ARM_POSITION_LIMITS[cfg.JOINT_3_ID]
    joint_11_limits = cfg.ARM_POSITION_LIMITS[cfg.JOINT_1_ID]
    if joint_13_limits is None or joint_11_limits is None:
        raise ValueError("manual arm axes require explicit position limits")
    return ManualTargets(
        joint_13=_clamp_int(
            targets.joint_13
            + shaped_axis(joint_13_axis, deadzone) * joint_13_sign * joint_13_rate * dt,
            joint_13_limits,
        ),
        joint_11=_clamp_int(
            targets.joint_11
            + shaped_axis(joint_11_axis, deadzone) * joint_11_sign * joint_11_rate * dt,
            joint_11_limits,
        ),
        wrist_4=_clamp_int(
            targets.wrist_4
            + shaped_axis(wrist_axis, deadzone) * wrist_sign * wrist_rate * dt,
            (cfg.WRIST_MIN, cfg.WRIST_MAX),
        ),
    )


class ManualArmJoy(Node):
    """RB-deadman absolute position controller for ID13, ID11, and wrist 4/6."""

    def __init__(self):
        super().__init__("roka_manual_arm_joy")
        self._read_parameters()
        self.driver = DynamixelDriver()
        self.closed = False
        self.faulted = False
        self.latest_joy: Joy | None = None
        self.latest_joy_time = 0.0
        self.last_update_time = time.monotonic()

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.ready_pub = self.create_publisher(Bool, "/roka/arm/hold_ready", latched)
        self.status_pub = self.create_publisher(String, "/roka/arm/hold_status", latched)
        self._publish(False, "manual arm initializing")

        if not self.driver.initialize(
            enable_motion=True,
            allow_initial_recovery=True,
        ):
            self.driver.close_ports()
            raise RuntimeError("manual Dynamixel initialization failed")
        initial_pose = {
            **cfg.START_POSE,
            "wrist": cfg.WRIST_START_POSITION,
            "gripper": cfg.GRIPPER_OPEN_POSITION,
        }
        if not self.driver.wait_until_pose_values_reached(
            "MANUAL START HOLD", initial_pose
        ):
            self.driver.close_ports()
            raise RuntimeError("arm did not reach its initial transport pose")
        pose = self.driver.read_current_pose()
        if pose is None or not self.driver.all_torque_enabled():
            self.driver.close_ports()
            raise RuntimeError("manual arm pose/torque validation failed")
        self.targets = ManualTargets(
            joint_13=int(pose[cfg.JOINT_3_ID]),
            joint_11=int(pose[cfg.JOINT_1_ID]),
            wrist_4=int(pose["wrist"]),
        )

        self.create_subscription(Joy, self.joy_topic, self._joy, 20)
        self.create_timer(1.0 / self.control_rate_hz, self._control)
        self._publish(True, "initial pose held; optional manual control ready")
        print(
            "[ARM-MANUAL][READY] 초기자세 HOLD | RB+오른쪽 Y=ID13, "
            "RB+오른쪽 X=ID11, RB+D-pad 상하=손목 ID4/6 동기",
            flush=True,
        )

    def _read_parameters(self) -> None:
        cfg.DEVICENAME_ARM = str(self.declare_parameter(
            "arm_serial_port", cfg.DEVICENAME_ARM
        ).value)
        cfg.DEVICENAME_WRIST = str(self.declare_parameter(
            "wrist_serial_port", cfg.DEVICENAME_WRIST
        ).value)
        # A manual-node shutdown must close only the serial handles.  Dynamixel
        # Torque Enable and the last Goal Position stay active while powered.
        cfg.TORQUE_OFF_ON_EXIT = bool(self.declare_parameter(
            "torque_off_on_exit", False
        ).value)
        if cfg.TORQUE_OFF_ON_EXIT:
            raise ValueError("manual mode requires torque_off_on_exit=false")
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.enable_button = int(self.declare_parameter("enable_button", 5).value)
        self.joint_13_axis = int(self.declare_parameter("joint_13_axis", 4).value)
        self.joint_11_axis = int(self.declare_parameter("joint_11_axis", 3).value)
        self.wrist_axis = int(self.declare_parameter("wrist_axis", 7).value)
        self.deadzone = float(self.declare_parameter("deadzone", 0.18).value)
        self.control_rate_hz = float(self.declare_parameter(
            "control_rate_hz", 20.0
        ).value)
        self.joy_timeout_sec = float(self.declare_parameter(
            "joy_timeout_sec", 0.35
        ).value)
        self.joint_13_rate = float(self.declare_parameter(
            "joint_13_rate_counts_sec", 8000.0
        ).value)
        self.joint_11_rate = float(self.declare_parameter(
            "joint_11_rate_counts_sec", 5000.0
        ).value)
        self.wrist_rate = float(self.declare_parameter(
            "wrist_rate_counts_sec", 300.0
        ).value)
        self.joint_13_sign = float(self.declare_parameter("joint_13_sign", 1.0).value)
        self.joint_11_sign = float(self.declare_parameter("joint_11_sign", 1.0).value)
        self.wrist_sign = float(self.declare_parameter("wrist_sign", 1.0).value)
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if self.control_rate_hz <= 0.0 or self.joy_timeout_sec <= 0.0:
            raise ValueError("control rate and joy timeout must be positive")
        if min(self.enable_button, self.joint_13_axis, self.joint_11_axis, self.wrist_axis) < 0:
            raise ValueError("manual Xbox button/axis indexes cannot be negative")
        if min(self.joint_13_rate, self.joint_11_rate, self.wrist_rate) <= 0.0:
            raise ValueError("manual arm position rates must be positive")

    def _publish(self, ready: bool, detail: str) -> None:
        if not rclpy.ok(context=self.context):
            return
        try:
            ready_message = Bool()
            ready_message.data = bool(ready)
            self.ready_pub.publish(ready_message)
            status = String()
            status.data = f"ready={str(bool(ready)).lower()};detail={detail}"
            self.status_pub.publish(status)
        except RCLError:
            if rclpy.ok(context=self.context):
                raise

    def _joy(self, message: Joy) -> None:
        self.latest_joy = message
        self.latest_joy_time = time.monotonic()

    @staticmethod
    def _axis(message: Joy, index: int) -> float:
        return float(message.axes[index]) if index < len(message.axes) else 0.0

    def _enabled(self, message: Joy) -> bool:
        return self.enable_button < len(message.buttons) and bool(
            message.buttons[self.enable_button]
        )

    def _control(self) -> None:
        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now
        message = self.latest_joy
        if (
            self.faulted
            or message is None
            or now - self.latest_joy_time > self.joy_timeout_sec
            or not self._enabled(message)
        ):
            # No zero/torque-off command is sent: the last absolute Goal
            # Position remains actively held by each Dynamixel.
            return

        updated = integrate_targets(
            self.targets,
            joint_13_axis=self._axis(message, self.joint_13_axis),
            joint_11_axis=self._axis(message, self.joint_11_axis),
            wrist_axis=self._axis(message, self.wrist_axis),
            dt=dt,
            deadzone=self.deadzone,
            joint_13_rate=self.joint_13_rate,
            joint_11_rate=self.joint_11_rate,
            wrist_rate=self.wrist_rate,
            joint_13_sign=self.joint_13_sign,
            joint_11_sign=self.joint_11_sign,
            wrist_sign=self.wrist_sign,
        )
        ok = True
        if updated.joint_13 != self.targets.joint_13:
            ok = self.driver.write_arm_position(
                cfg.JOINT_3_ID, updated.joint_13
            ) and ok
        if updated.joint_11 != self.targets.joint_11:
            ok = self.driver.write_arm_position(
                cfg.JOINT_1_ID, updated.joint_11
            ) and ok
        if updated.wrist_4 != self.targets.wrist_4:
            # Existing driver always sends IDs 4 and 6 in one GroupSyncWrite.
            ok = self.driver.write_wrist_position(updated.wrist_4) and ok
        if not ok:
            self.faulted = True
            self._publish(False, "manual arm command failed; base stopped")
            print("[ARM-MANUAL][FAULT] 위치 명령 실패, 베이스 허용 해제", flush=True)
            return
        self.targets = updated

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if rclpy.ok(context=self.context):
            self._publish(False, "manual node stopped; final pose remains torque-held")
        # Reissue the final absolute goals before dropping serial ownership.
        # There is deliberately no Torque Disable write here.
        if hasattr(self, "targets") and not self.faulted:
            self.driver.write_arm_position(cfg.JOINT_3_ID, self.targets.joint_13)
            self.driver.write_arm_position(cfg.JOINT_1_ID, self.targets.joint_11)
            self.driver.write_wrist_position(self.targets.wrist_4)
        self.driver.close_ports()
        print(
            "[ARM-MANUAL][LATCHED] 노드 종료, 마지막 자세 Goal Position/토크 유지",
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ManualArmJoy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"[ARM-MANUAL][ERROR] {error}", flush=True)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
