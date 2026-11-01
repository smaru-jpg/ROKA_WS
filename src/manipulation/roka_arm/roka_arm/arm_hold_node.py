#!/usr/bin/env python3
"""Move once to the selected section pose and keep Dynamixel torque enabled."""

from __future__ import annotations

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from . import arm_config as cfg
from .dynamixel_driver import DynamixelDriver


def section_hold_pose():
    return {
        **cfg.START_POSE,
        "wrist": cfg.WRIST_START_POSITION,
        "wrist_reverse": 4096 - cfg.WRIST_START_POSITION,
        "gripper": cfg.GRIPPER_OPEN_POSITION,
    }


def pose_is_section_hold_locked(pose, target) -> bool:
    if not pose or not target:
        return False
    if any(key not in pose for key in target):
        return False
    return bool(
        all(
            abs(int(pose[motor_id]) - int(target[motor_id]))
            <= cfg.H54_POSITION_TOLERANCE
            for motor_id in (cfg.JOINT_1_ID, cfg.JOINT_2_ID, cfg.JOINT_3_ID)
        )
        and abs(int(pose["wrist"]) - int(target["wrist"]))
        <= cfg.WRIST_POSITION_TOLERANCE
        and abs(int(pose["wrist_reverse"]) - int(target["wrist_reverse"]))
        <= cfg.WRIST_POSITION_TOLERANCE
        and abs(int(pose["gripper"]) - int(target["gripper"]))
        <= cfg.GRIPPER_POSITION_TOLERANCE
    )


def pro_pose_is_section_hold_locked(pose, target) -> bool:
    """Check only the 24 V PRO axes used by the PRO-only drive mode."""
    motor_ids = (cfg.JOINT_1_ID, cfg.JOINT_2_ID, cfg.JOINT_3_ID)
    if not pose or not target or any(motor_id not in pose for motor_id in motor_ids):
        return False
    return all(
        abs(int(pose[motor_id]) - int(target[motor_id]))
        <= cfg.H54_POSITION_TOLERANCE
        for motor_id in motor_ids
    )


class ArmHoldNode(Node):
    def __init__(self):
        super().__init__("roka_arm_hold")
        cfg.DEVICENAME_ARM = str(self.declare_parameter(
            "arm_serial_port", cfg.DEVICENAME_ARM
        ).value)
        cfg.DEVICENAME_WRIST = str(self.declare_parameter(
            "wrist_serial_port", cfg.DEVICENAME_WRIST
        ).value)
        cfg.TORQUE_OFF_ON_EXIT = bool(self.declare_parameter(
            "torque_off_on_exit", cfg.TORQUE_OFF_ON_EXIT
        ).value)
        self.start_pose_name = str(self.declare_parameter(
            "start_pose_name", f"section{cfg.DEFAULT_SECTION}"
        ).value)
        self.hold_scope = str(self.declare_parameter(
            "hold_scope", "all"
        ).value).strip().lower()
        if self.hold_scope not in {"all", "pro_only"}:
            raise ValueError("hold_scope must be 'all' or 'pro_only'")
        self.pro_only = self.hold_scope == "pro_only"
        cfg.activate_start_pose(self.start_pose_name)
        self.verify_period = float(self.declare_parameter(
            "verify_period_sec", 2.0
        ).value)
        self.communication_grace_count = int(self.declare_parameter(
            "communication_grace_count", 2
        ).value)
        if self.verify_period <= 0.0:
            raise ValueError("verify_period_sec must be positive")
        if self.communication_grace_count < 0:
            raise ValueError("communication_grace_count cannot be negative")

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.ready_pub = self.create_publisher(Bool, "/roka/arm/hold_ready", latched)
        self.status_pub = self.create_publisher(String, "/roka/arm/hold_status", latched)
        self.driver = DynamixelDriver()
        self.ready = False
        self.closed = False
        self.communication_misses = 0
        self.recovery_warning_emitted = False
        self.hold_deviation_warning_emitted = False
        self.hold_pose = section_hold_pose()
        self._publish(False, "initializing")

        # A readable current encoder value may start outside a normal motion
        # limit. Only this initial recovery to the already validated section
        # target is exempt; every ordinary arm command keeps the old limits.
        if not self.driver.initialize(
            enable_motion=True,
            allow_initial_recovery=True,
            include_wrist=not self.pro_only,
        ):
            self.driver.close_ports()
            raise RuntimeError("Dynamixel section-pose hold initialization failed")
        pose_target = self._driver_pose(self.hold_pose)
        wait_ok = (
            self.driver.wait_until_arm_pose_values_reached(
                "SECTION HOLD PRO-ONLY", pose_target
            )
            if self.pro_only
            else self.driver.wait_until_pose_values_reached(
                "SECTION HOLD", pose_target
            )
        )
        if not wait_ok:
            self.driver.close_ports()
            raise RuntimeError("arm did not reach the section hold pose")
        torque_ready = (
            self.driver.read_arm_torque_state() is True
            if self.pro_only
            else self.driver.all_torque_enabled()
        )
        if not torque_ready:
            self.driver.close_ports()
            raise RuntimeError("one or more Dynamixel torque locks are not enabled")
        self._publish(
            True,
            f"{self.start_pose_name} reached; {self.hold_scope} torque locked",
        )
        print(
            f"[ARM-HOLD][READY] {self.start_pose_name} 시작 자세 도달, "
            f"토크 유지 scope={self.hold_scope}",
            flush=True,
        )
        self.create_timer(self.verify_period, self._verify_hold)

    def _publish(self, ready: bool, detail: str) -> None:
        self.ready = bool(ready)
        if not rclpy.ok(context=self.context):
            return
        ready_message = Bool()
        ready_message.data = self.ready
        try:
            self.ready_pub.publish(ready_message)
            status = String()
            status.data = f"ready={str(self.ready).lower()};detail={detail}"
            self.status_pub.publish(status)
        except RCLError:
            # SIGINT may invalidate the context between the explicit check and
            # publish.  The serial ports still get closed by close().
            if rclpy.ok(context=self.context):
                raise

    def _verify_hold(self) -> None:
        pro_only = bool(getattr(self, "pro_only", False))
        pose = (
            self.driver.read_arm_pose()
            if pro_only
            else self.driver.read_current_pose()
        )
        if pose is not None and not pro_only:
            reverse = self.driver.read_xm_position(
                cfg.WRIST_REVERSE_ID
            )
            if reverse is None:
                pose = None
            else:
                pose["wrist_reverse"] = reverse
        torque_state = (
            self.driver.read_arm_torque_state()
            if pro_only
            else self.driver.read_all_torque_state()
        )
        if pose is None or torque_state is None:
            self.communication_misses += 1
            if self.communication_misses <= self.communication_grace_count:
                if not self.recovery_warning_emitted:
                    print(
                        "[ARM-HOLD][WARN] 일시적 상태 패킷 누락; "
                        "기존 토크 잠금을 유지하며 재확인",
                        flush=True,
                    )
                    self.recovery_warning_emitted = True
                return
            if self.ready:
                self._publish(False, "Dynamixel feedback unavailable beyond grace window")
            return

        self.communication_misses = 0
        self.recovery_warning_emitted = False
        torque_ready = torque_state is True
        pose_locked = (
            pro_pose_is_section_hold_locked(pose, self.hold_pose)
            if pro_only
            else pose_is_section_hold_locked(pose, self.hold_pose)
        )
        if torque_ready and pose_locked:
            self.hold_deviation_warning_emitted = False
            if not self.ready:
                self._publish(True, "section hold pose recovered; torque locked")
            return

        self._publish(False, "torque/section hold pose lost; restoring hold")
        if not self.hold_deviation_warning_emitted:
            print("[ARM-HOLD][RECOVER] 토크/섹션 시작 자세 잠금 복구", flush=True)
            self.hold_deviation_warning_emitted = True
        if not torque_ready:
            torque_recovered = (
                self.driver.enable_arm_torque()
                if pro_only
                else self.driver.enable_all_torque()
            )
            if not torque_recovered:
                print("[ARM-HOLD][WAIT] 토크 복구 실패; 다음 주기에 재시도", flush=True)
                return
        pose_target = self._driver_pose(self.hold_pose)
        move_ok = (
            self.driver.move_arm_pose_values(
                "SECTION HOLD RECOVERY PRO-ONLY", pose_target
            )
            if pro_only
            else self.driver.move_pose_values("SECTION HOLD RECOVERY", pose_target)
        )
        if not move_ok:
            print("[ARM-HOLD][WAIT] 자세 복구 명령 실패; 다음 주기에 재시도", flush=True)
            return
        wait_ok = (
            self.driver.wait_until_arm_pose_values_reached(
                "SECTION HOLD RECOVERY PRO-ONLY", pose_target
            )
            if pro_only
            else self.driver.wait_until_pose_values_reached(
                "SECTION HOLD RECOVERY", pose_target
            )
        )
        if not wait_ok:
            print("[ARM-HOLD][WAIT] 자세 복구 대기 실패; 다음 주기에 재시도", flush=True)
            return
        self.hold_deviation_warning_emitted = False
        self._publish(True, "section hold pose recovered; torque locked")

    @staticmethod
    def _driver_pose(pose):
        """Strip the verification-only reverse-wrist value for driver moves."""
        return {
            **{motor_id: pose[motor_id] for motor_id in cfg.START_POSE},
            "wrist": pose["wrist"],
            "gripper": pose["gripper"],
        }

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        # A launch-wide shutdown can invalidate the ROS context before this
        # node's finally block runs.  Releasing the serial buses is still
        # required, but publishing through an invalid context only creates a
        # misleading RCLError after an otherwise successful map/calibration.
        if rclpy.ok(context=self.context):
            detail = (
                "stopped; torque is being released"
                if cfg.TORQUE_OFF_ON_EXIT
                else "stopped; serial ports closing, section pose torque remains enabled"
            )
            self._publish(False, detail)
        self.driver.close_ports()
        if not cfg.TORQUE_OFF_ON_EXIT:
            print(
                "[ARM-HOLD][LATCHED] 터미널 종료 후에도 섹션 자세 토크 유지",
                flush=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ArmHoldNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"[ARM-HOLD][ERROR] {error}", flush=True)
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
