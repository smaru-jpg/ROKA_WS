import fcntl
import os
import time

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

from . import arm_config as cfg


def make_position_param(position):
    return [
        DXL_LOBYTE(DXL_LOWORD(position)),
        DXL_HIBYTE(DXL_LOWORD(position)),
        DXL_LOBYTE(DXL_HIWORD(position)),
        DXL_HIBYTE(DXL_HIWORD(position)),
    ]


class DynamixelDriver:
    def __init__(self):
        self.arm_port = PortHandler(cfg.DEVICENAME_ARM)
        self.wrist_port = PortHandler(cfg.DEVICENAME_WRIST)
        self.packet = PacketHandler(cfg.PROTOCOL_VERSION)

        self.arm_opened = False
        self.wrist_opened = False
        self.group_sync_write = None
        self.motion_enabled = False
        self.torque_armed = False
        self._lock_files = []

    def _acquire_port_locks(self, include_wrist=True):
        """Prevent diagnostics and the controller from sharing a serial bus."""
        names = ["roka_arm_rs485"]
        if include_wrist:
            names.append("roka_arm_ttl")
        for name in names:
            path = f"/tmp/{name}.lock"
            handle = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(handle)
                self._release_port_locks()
                print(f"[ARM][ERROR] serial bus already in use: {path}")
                return False
            self._lock_files.append(handle)
        return True

    def _release_port_locks(self):
        while self._lock_files:
            handle = self._lock_files.pop()
            try:
                fcntl.flock(handle, fcntl.LOCK_UN)
            finally:
                os.close(handle)

    def _allow_position_write(self):
        if self.motion_enabled:
            return True
        print("[ARM][BLOCKED] position command rejected: motion is disabled")
        return False

    # ========================================================
    # 공통 통신 결과 검사
    # ========================================================
    def check_result(self, motor_id, comm_result, dxl_error, action):
        if comm_result != COMM_SUCCESS:
            print(
                f"[FAIL] ID {motor_id} {action}: "
                f"{self.packet.getTxRxResult(comm_result)}"
            )
            return False

        if dxl_error != 0:
            print(
                f"[FAIL] ID {motor_id} {action}: "
                f"{self.packet.getRxPacketError(dxl_error)}"
            )
            return False

        return True

    # ========================================================
    # Limit
    # ========================================================
    def validate_arm_position(self, motor_id, position):
        if motor_id not in cfg.ARM_POSITION_LIMITS:
            print(f"[ERROR] 잘못된 Arm Motor ID: {motor_id}")
            return False

        limit = cfg.ARM_POSITION_LIMITS[motor_id]

        if limit is None:
            if not -2147483648 <= position <= 2147483647:
                print(f"[LIMIT] ID {motor_id}: {position} (32bit 범위 초과)")
                return False
            return True

        min_position, max_position = limit

        if not min_position <= position <= max_position:
            print(
                f"[LIMIT] ID {motor_id}: {position} "
                f"(허용범위 {min_position} ~ {max_position})"
            )
            return False

        return True

    def validate_wrist_position(self, position):
        if not cfg.WRIST_MIN <= position <= cfg.WRIST_MAX:
            print(
                f"[LIMIT] Wrist ID 4 기준 위치: {position} "
                f"(허용범위 {cfg.WRIST_MIN} ~ {cfg.WRIST_MAX})"
            )
            return False
        return True

    def validate_gripper_position(self, position):
        if not cfg.GRIPPER_MIN <= position <= cfg.GRIPPER_MAX:
            print(
                f"[LIMIT] Gripper Position: {position} "
                f"(허용범위 {cfg.GRIPPER_MIN} ~ {cfg.GRIPPER_MAX})"
            )
            return False
        return True

    # ========================================================
    # Read
    # ========================================================
    def read_h54_position(self, motor_id):
        position, comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_PRESENT_POSITION,
        )

        if not self.check_result(
            motor_id, comm_result, dxl_error, "현재 위치 읽기"
        ):
            return None

        if position > 0x7FFFFFFF:
            position -= 0x100000000

        return position

    def read_xm_position(self, motor_id):
        position, comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.wrist_port,
            motor_id,
            cfg.XM_ADDR_PRESENT_POSITION,
        )

        if not self.check_result(
            motor_id, comm_result, dxl_error, "현재 위치 읽기"
        ):
            return None

        return position

    def read_gripper_current_mA(self):
        current_raw, comm_result, dxl_error = self.packet.read2ByteTxRx(
            self.wrist_port,
            cfg.GRIPPER_ID,
            cfg.XM_ADDR_PRESENT_CURRENT,
        )

        if not self.check_result(
            cfg.GRIPPER_ID,
            comm_result,
            dxl_error,
            "Gripper Present Current 읽기",
        ):
            return None

        if current_raw > 32767:
            current_raw -= 65536

        return current_raw * cfg.CURRENT_UNIT_MA

    def read_h54_torque_enabled(self, motor_id):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_TORQUE_ENABLE,
        )
        if not self.check_result(
            motor_id, comm_result, dxl_error, "Torque Enable 읽기"
        ):
            return None
        return int(value) == cfg.TORQUE_ON

    def read_xm_torque_enabled(self, motor_id):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(
            self.wrist_port,
            motor_id,
            cfg.XM_ADDR_TORQUE_ENABLE,
        )
        if not self.check_result(
            motor_id, comm_result, dxl_error, "Torque Enable 읽기"
        ):
            return None
        return int(value) == cfg.TORQUE_ON

    def read_all_torque_state(self):
        """Return True/False for torque state, or None on communication loss."""
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            enabled = self.read_h54_torque_enabled(motor_id)
            if enabled is None:
                return None
            if not enabled:
                return False
        for motor_id in (
            cfg.WRIST_FORWARD_ID,
            cfg.WRIST_REVERSE_ID,
            cfg.GRIPPER_ID,
        ):
            enabled = self.read_xm_torque_enabled(motor_id)
            if enabled is None:
                return None
            if not enabled:
                return False
        return True

    def read_arm_torque_state(self):
        """Return the 24 V PRO torque state without touching the 12 V bus."""
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            enabled = self.read_h54_torque_enabled(motor_id)
            if enabled is None:
                return None
            if not enabled:
                return False
        return True

    def all_torque_enabled(self):
        """Confirm every arm, wrist, and gripper actuator is energized."""
        return self.read_all_torque_state() is True

    def enable_all_torque(self):
        """Re-enable all actuators after a transport-hold dropout."""
        ok = True
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            ok = self.set_h54_torque(motor_id, cfg.TORQUE_ON) and ok
        for motor_id in (
            cfg.WRIST_FORWARD_ID,
            cfg.WRIST_REVERSE_ID,
            cfg.GRIPPER_ID,
        ):
            ok = self.set_xm_torque(motor_id, cfg.TORQUE_ON) and ok
        if ok:
            self.torque_armed = True
            self.motion_enabled = True
        return ok

    def enable_arm_torque(self):
        """Re-enable only the 24 V PRO joints."""
        ok = True
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            ok = self.set_h54_torque(motor_id, cfg.TORQUE_ON) and ok
        if ok:
            self.torque_armed = True
            self.motion_enabled = True
        return ok

    # ========================================================
    # Write
    # ========================================================
    def write_arm_position(self, motor_id, goal_position):
        if not self._allow_position_write():
            return False
        if not self.validate_arm_position(motor_id, goal_position):
            return False

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_GOAL_POSITION,
            goal_position & 0xFFFFFFFF,
        )

        return self.check_result(
            motor_id,
            comm_result,
            dxl_error,
            f"Goal Position {goal_position}",
        )

    def move_arm_delta(self, motor_id, delta):
        present = self.read_h54_position(motor_id)

        if present is None:
            print(f"[SAFETY STOP] ID {motor_id} 현재 위치 읽기 실패")
            return False

        goal = present + delta

        if not self.validate_arm_position(motor_id, goal):
            return False

        print(f"[ARM] ID {motor_id}: {present} -> {goal} (delta={delta})")
        return self.write_arm_position(motor_id, goal)

    def write_wrist_position(self, goal_position_4):
        if not self._allow_position_write():
            return False
        if not self.validate_wrist_position(goal_position_4):
            return False

        goal_position_6 = 4096 - goal_position_4
        goal_position_6 = max(0, min(4095, goal_position_6))

        param_4 = make_position_param(goal_position_4)
        param_6 = make_position_param(goal_position_6)

        if not self.group_sync_write.addParam(
            cfg.WRIST_FORWARD_ID, param_4
        ):
            print("[ERROR] ID 4 SyncWrite 등록 실패")
            self.group_sync_write.clearParam()
            return False

        if not self.group_sync_write.addParam(
            cfg.WRIST_REVERSE_ID, param_6
        ):
            print("[ERROR] ID 6 SyncWrite 등록 실패")
            self.group_sync_write.clearParam()
            return False

        result = self.group_sync_write.txPacket()
        self.group_sync_write.clearParam()

        if result != COMM_SUCCESS:
            print(
                "[FAIL] Wrist SyncWrite: "
                f"{self.packet.getTxRxResult(result)}"
            )
            return False

        print(
            f"[WRIST] ID 4 -> {goal_position_4} | "
            f"ID 6 -> {goal_position_6}"
        )
        return True

    def move_wrist_delta(self, delta):
        present_4 = self.read_xm_position(cfg.WRIST_FORWARD_ID)

        if present_4 is None:
            print("[SAFETY STOP] Wrist ID 4 현재 위치 읽기 실패")
            return False

        goal_4 = present_4 + delta

        if not self.validate_wrist_position(goal_4):
            return False

        print(
            f"[WRIST DELTA] {present_4} -> {goal_4} "
            f"(delta={delta})"
        )

        return self.write_wrist_position(goal_4)

    def write_gripper_position(self, goal_position):
        if not self._allow_position_write():
            return False
        if not self.validate_gripper_position(goal_position):
            return False

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.wrist_port,
            cfg.GRIPPER_ID,
            cfg.XM_ADDR_GOAL_POSITION,
            goal_position,
        )

        return self.check_result(
            cfg.GRIPPER_ID,
            comm_result,
            dxl_error,
            f"Gripper Goal {goal_position}",
        )

    def open_gripper(self):
        print(f"[GRIPPER] OPEN -> {cfg.GRIPPER_OPEN_POSITION}")
        return self.write_gripper_position(cfg.GRIPPER_OPEN_POSITION)

    def close_gripper(self):
        print(f"[GRIPPER] CLOSE -> {cfg.GRIPPER_CLOSE_POSITION}")
        return self.write_gripper_position(cfg.GRIPPER_CLOSE_POSITION)

    # ========================================================
    # Profile / Torque
    # ========================================================
    def set_h54_motion_profile(self, motor_id, velocity, acceleration):
        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_GOAL_VELOCITY,
            velocity,
        )
        if not self.check_result(
            motor_id, comm_result, dxl_error,
            f"Goal Velocity = {velocity}",
        ):
            return False

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_GOAL_ACCELERATION,
            acceleration,
        )
        return self.check_result(
            motor_id, comm_result, dxl_error,
            f"Goal Acceleration = {acceleration}",
        )

    def set_xm_profile(self, motor_id, acceleration, velocity):
        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.wrist_port,
            motor_id,
            cfg.XM_ADDR_PROFILE_ACCELERATION,
            acceleration,
        )
        if not self.check_result(
            motor_id, comm_result, dxl_error,
            f"Profile Acceleration = {acceleration}",
        ):
            return False

        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.wrist_port,
            motor_id,
            cfg.XM_ADDR_PROFILE_VELOCITY,
            velocity,
        )
        return self.check_result(
            motor_id, comm_result, dxl_error,
            f"Profile Velocity = {velocity}",
        )

    def set_h54_torque(self, motor_id, enable):
        comm_result, dxl_error = self.packet.write1ByteTxRx(
            self.arm_port,
            motor_id,
            cfg.H54_ADDR_TORQUE_ENABLE,
            enable,
        )
        return self.check_result(
            motor_id,
            comm_result,
            dxl_error,
            "Torque ON" if enable else "Torque OFF",
        )

    def set_xm_torque(self, motor_id, enable):
        comm_result, dxl_error = self.packet.write1ByteTxRx(
            self.wrist_port,
            motor_id,
            cfg.XM_ADDR_TORQUE_ENABLE,
            enable,
        )
        return self.check_result(
            motor_id,
            comm_result,
            dxl_error,
            "Torque ON" if enable else "Torque OFF",
        )

    # ========================================================
    # Pose
    # ========================================================
    def read_current_pose(self):
        pose = {}

        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            position = self.read_h54_position(motor_id)
            if position is None:
                return None
            pose[motor_id] = position

        wrist = self.read_xm_position(cfg.WRIST_FORWARD_ID)
        if wrist is None:
            return None

        gripper = self.read_xm_position(cfg.GRIPPER_ID)
        if gripper is None:
            return None

        pose["wrist"] = wrist
        pose["gripper"] = gripper
        return pose

    def read_arm_pose(self):
        """Read only the three 24 V PRO joints."""
        pose = {}
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            position = self.read_h54_position(motor_id)
            if position is None:
                return None
            pose[motor_id] = position
        return pose

    def move_arm_pose_values(self, pose_name, pose):
        """Move only the three 24 V PRO joints."""
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            if not self.validate_arm_position(motor_id, pose[motor_id]):
                return False
        for motor_id in (
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ):
            print(f"[POSE][PRO-ONLY] ID {motor_id} -> {pose[motor_id]}")
            if not self.write_arm_position(motor_id, pose[motor_id]):
                return False
        print(f"[COMPLETE][PRO-ONLY] {pose_name} command sent")
        return True

    def wait_until_arm_pose_values_reached(self, pose_name, pose):
        """Wait for a PRO-only pose without reading the 12 V bus."""
        start_time = time.time()
        while True:
            present = self.read_arm_pose()
            if present is None:
                return False
            if all(
                abs(int(present[motor_id]) - int(pose[motor_id]))
                <= cfg.H54_POSITION_TOLERANCE
                for motor_id in (
                    cfg.JOINT_1_ID,
                    cfg.JOINT_2_ID,
                    cfg.JOINT_3_ID,
                )
            ):
                print(f"[REACHED][PRO-ONLY] {pose_name}")
                return True
            if time.time() - start_time > cfg.POSE_REACHED_TIMEOUT:
                print(f"[TIMEOUT][PRO-ONLY] {pose_name}")
                return False
            time.sleep(cfg.POSE_CHECK_PERIOD)

    def move_pose_values(self, pose_name, pose):
        print()
        print("=" * 60)
        print(f"{pose_name} 이동 시작")
        print("=" * 60)

        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            if not self.validate_arm_position(
                motor_id, pose[motor_id]
            ):
                return False

        if not self.validate_wrist_position(pose["wrist"]):
            return False

        if not self.validate_gripper_position(pose["gripper"]):
            return False

        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            print(f"[POSE] ID {motor_id} -> {pose[motor_id]}")
            if not self.write_arm_position(
                motor_id, pose[motor_id]
            ):
                return False

        if not self.write_wrist_position(pose["wrist"]):
            return False

        print(f"[GRIPPER] ID 1 -> {pose['gripper']}")
        if not self.write_gripper_position(pose["gripper"]):
            return False

        print(f"[COMPLETE] {pose_name} 명령 전송 완료")
        return True

    def wait_until_pose_values_reached(self, pose_name, pose):
        target_wrist_4 = pose["wrist"]
        target_wrist_6 = 4096 - target_wrist_4
        target_gripper = pose["gripper"]
        start_time = time.time()

        print(f"[WAIT] {pose_name} 도달 대기 중...")

        while True:
            arm_reached = True

            for motor_id in [
                cfg.JOINT_1_ID,
                cfg.JOINT_2_ID,
                cfg.JOINT_3_ID,
            ]:
                present = self.read_h54_position(motor_id)
                if present is None:
                    return False

                if (
                    abs(present - pose[motor_id])
                    > cfg.H54_POSITION_TOLERANCE
                ):
                    arm_reached = False

            present_4 = self.read_xm_position(cfg.WRIST_FORWARD_ID)
            present_6 = self.read_xm_position(cfg.WRIST_REVERSE_ID)
            present_1 = self.read_xm_position(cfg.GRIPPER_ID)

            if (
                present_4 is None
                or present_6 is None
                or present_1 is None
            ):
                return False

            wrist_4_reached = (
                abs(present_4 - target_wrist_4)
                <= cfg.WRIST_POSITION_TOLERANCE
            )
            wrist_6_reached = (
                abs(present_6 - target_wrist_6)
                <= cfg.WRIST_POSITION_TOLERANCE
            )

            if target_gripper == cfg.GRIPPER_CLOSE_POSITION:
                gripper_reached = True
            else:
                gripper_reached = (
                    abs(present_1 - target_gripper)
                    <= cfg.GRIPPER_POSITION_TOLERANCE
                )

            if (
                arm_reached
                and wrist_4_reached
                and wrist_6_reached
                and gripper_reached
            ):
                print(f"[REACHED] {pose_name} 도달 완료")
                return True

            if time.time() - start_time > cfg.POSE_REACHED_TIMEOUT:
                print(f"[TIMEOUT] {pose_name} 도달 실패")
                return False

            time.sleep(cfg.POSE_CHECK_PERIOD)

    # ========================================================
    # Startup
    # ========================================================
    def check_arm_positions(self, allow_initial_recovery=False):
        """Validate only the 24 V PRO feedback."""
        print()
        print("=" * 60)
        print("Dynamixel PRO Present Position Check")
        print("=" * 60)

        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            position = self.read_h54_position(motor_id)
            if position is None:
                return False

            limit = cfg.ARM_POSITION_LIMITS[motor_id]
            if limit is None:
                print(
                    f"ID {motor_id:2d} | Position = {position} | Limit = NONE"
                )
            else:
                min_position, max_position = limit
                print(
                    f"ID {motor_id:2d} | Position = {position} | "
                    f"Limit = {min_position} ~ {max_position}"
                )

            if limit is not None and not limit[0] <= position <= limit[1]:
                if not allow_initial_recovery:
                    print(
                        f"[LIMIT] ID {motor_id}: {position} "
                        f"(허용범위 {limit[0]} ~ {limit[1]})"
                    )
                    return False
                print(
                    f"[ARM-START][NOTICE] ID {motor_id}={position} starts "
                    f"outside {limit[0]}..{limit[1]}; one-time recovery to "
                    "the validated section target is allowed"
                )

        print("=" * 60)
        return True

    def check_all_positions(self, allow_initial_recovery=False):
        if not self.check_arm_positions(
            allow_initial_recovery=allow_initial_recovery
        ):
            return False

        position_4 = self.read_xm_position(cfg.WRIST_FORWARD_ID)
        if position_4 is None:
            return False
        print(
            f"ID {cfg.WRIST_FORWARD_ID:2d} | "
            f"Position = {position_4} (Wrist Forward)"
        )
        if not cfg.WRIST_MIN <= position_4 <= cfg.WRIST_MAX:
            if not allow_initial_recovery:
                print(
                    f"[LIMIT] Wrist ID 4 기준 위치: {position_4} "
                    f"(허용범위 {cfg.WRIST_MIN} ~ {cfg.WRIST_MAX})"
                )
                return False
            if not 0 <= int(position_4) <= 4095:
                print("[SAFETY STOP] Wrist ID 4 encoder feedback is invalid")
                return False
            print(
                "[ARM-START][NOTICE] Wrist starts outside its motion range; "
                "one-time recovery to a validated start target is allowed"
            )

        position_6 = self.read_xm_position(cfg.WRIST_REVERSE_ID)
        if position_6 is None:
            return False
        print(
            f"ID {cfg.WRIST_REVERSE_ID:2d} | "
            f"Position = {position_6} (Wrist Reverse)"
        )
        if not 0 <= int(position_6) <= 4095:
            print("[SAFETY STOP] Wrist ID 6 encoder feedback is invalid")
            return False

        gripper_position = self.read_xm_position(cfg.GRIPPER_ID)
        if gripper_position is None:
            return False
        print(
            f"ID {cfg.GRIPPER_ID:2d} | "
            f"Position = {gripper_position} (Gripper)"
        )
        if not cfg.GRIPPER_MIN <= gripper_position <= cfg.GRIPPER_MAX:
            if not allow_initial_recovery:
                print(
                    f"[LIMIT] Gripper Position: {gripper_position} "
                    f"(허용범위 {cfg.GRIPPER_MIN} ~ {cfg.GRIPPER_MAX})"
                )
                return False
            if not 0 <= int(gripper_position) <= 4095:
                print("[SAFETY STOP] Gripper encoder feedback is invalid")
                return False
            print(
                "[ARM-START][NOTICE] Gripper starts outside its motion range; "
                "one-time recovery to a validated start target is allowed"
            )

        print("=" * 60)
        return True

    def move_arm_to_start_pose(self):
        print()
        print("=" * 60)
        print("ARM START POSE 이동")
        print("=" * 60)

        for motor_id, goal_position in cfg.START_POSE.items():
            if not self.validate_arm_position(motor_id, goal_position):
                print(
                    "[SAFETY STOP] START POSE가 Position Limit을 "
                    "벗어났습니다."
                )
                return False

        for motor_id, goal_position in cfg.START_POSE.items():
            print(f"[START] ID {motor_id} -> {goal_position}")
            if not self.write_arm_position(motor_id, goal_position):
                return False

        print("[OK] ARM START POSE 명령 전송 완료")
        return True

    def initialize(
        self,
        enable_motion=False,
        allow_initial_recovery=False,
        include_wrist=True,
    ):
        """Initialize all actuators or the 24 V PRO bus only.

        ``include_wrist=False`` never opens, reads, configures, or commands the
        12 V wrist/gripper bus.  The default preserves the deployed full-arm
        behavior.
        """
        include_wrist = bool(include_wrist)
        if not self._acquire_port_locks(include_wrist=include_wrist):
            return False

        if not self.arm_port.openPort():
            print(f"[ERROR] Arm Port Open 실패: {cfg.DEVICENAME_ARM}")
            return False
        self.arm_opened = True

        if not self.arm_port.setBaudRate(cfg.ARM_BAUDRATE):
            print(f"[ERROR] Arm Baudrate 설정 실패: {cfg.ARM_BAUDRATE}")
            return False
        print(f"[OK] Arm Port/Baudrate: {cfg.DEVICENAME_ARM} / {cfg.ARM_BAUDRATE}")

        if include_wrist:
            if not self.wrist_port.openPort():
                print(f"[ERROR] Wrist Port Open 실패: {cfg.DEVICENAME_WRIST}")
                return False
            self.wrist_opened = True

            if not self.wrist_port.setBaudRate(cfg.WRIST_BAUDRATE):
                print(
                    f"[ERROR] Wrist Baudrate 설정 실패: "
                    f"{cfg.WRIST_BAUDRATE}"
                )
                return False
            print(
                f"[OK] Wrist Port/Baudrate: "
                f"{cfg.DEVICENAME_WRIST} / {cfg.WRIST_BAUDRATE}"
            )

        position_check = self.check_all_positions if include_wrist else self.check_arm_positions
        if not position_check(allow_initial_recovery=allow_initial_recovery):
            print("[SAFETY STOP] Dynamixel 상태 확인 실패")
            return False

        if not enable_motion:
            print("[ARM][READY] IDs and positions valid; torque and motion remain OFF")
            return True

        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            if not self.set_h54_motion_profile(
                motor_id,
                cfg.H54_GOAL_VELOCITY,
                cfg.H54_GOAL_ACCELERATION,
            ):
                return False

        if include_wrist:
            for motor_id in [
                cfg.WRIST_FORWARD_ID,
                cfg.WRIST_REVERSE_ID,
            ]:
                if not self.set_xm_profile(
                    motor_id,
                    cfg.WRIST_PROFILE_ACCELERATION,
                    cfg.WRIST_PROFILE_VELOCITY,
                ):
                    return False

            if not self.set_xm_profile(
                cfg.GRIPPER_ID,
                cfg.GRIPPER_PROFILE_ACCELERATION,
                cfg.GRIPPER_PROFILE_VELOCITY,
            ):
                return False

        # From this point a partial failure may leave at least one actuator
        # energized, so close_ports() must attempt a full torque-off cleanup.
        self.torque_armed = True
        for motor_id in [
            cfg.JOINT_1_ID,
            cfg.JOINT_2_ID,
            cfg.JOINT_3_ID,
        ]:
            if not self.set_h54_torque(motor_id, cfg.TORQUE_ON):
                return False

        if include_wrist:
            for motor_id in [
                cfg.WRIST_FORWARD_ID,
                cfg.WRIST_REVERSE_ID,
                cfg.GRIPPER_ID,
            ]:
                if not self.set_xm_torque(motor_id, cfg.TORQUE_ON):
                    return False

        self.motion_enabled = True

        if include_wrist:
            self.group_sync_write = GroupSyncWrite(
                self.wrist_port,
                self.packet,
                cfg.XM_ADDR_GOAL_POSITION,
                4,
            )

        if not self.move_arm_to_start_pose():
            return False

        if include_wrist:
            if not self.write_wrist_position(cfg.WRIST_START_POSITION):
                return False

            if not self.open_gripper():
                return False

        print()
        print("=" * 60)
        scope = "FULL" if include_wrist else "PRO-ONLY"
        print(f"Dynamixel 초기화 완료 ({scope})")
        print("=" * 60)
        return True

    def disable_torque(self):
        ok = True
        if self.arm_opened:
            for motor_id in [
                cfg.JOINT_1_ID,
                cfg.JOINT_2_ID,
                cfg.JOINT_3_ID,
            ]:
                ok = self.set_h54_torque(motor_id, cfg.TORQUE_OFF) and ok
        if self.wrist_opened:
            for motor_id in [
                cfg.WRIST_FORWARD_ID,
                cfg.WRIST_REVERSE_ID,
                cfg.GRIPPER_ID,
            ]:
                ok = self.set_xm_torque(motor_id, cfg.TORQUE_OFF) and ok
        self.motion_enabled = False
        self.torque_armed = False
        return ok

    def close_ports(self):
        if self.torque_armed and cfg.TORQUE_OFF_ON_EXIT:
            self.disable_torque()

        if self.arm_opened:
            self.arm_port.closePort()
            self.arm_opened = False
            print("[OK] Arm Port Closed")

        if self.wrist_opened:
            self.wrist_port.closePort()
            self.wrist_opened = False
            print("[OK] Wrist Port Closed")
        self._release_port_locks()
