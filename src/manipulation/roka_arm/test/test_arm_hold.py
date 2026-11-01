from roka_arm import arm_config as cfg
from roka_arm.arm_hold_node import (
    ArmHoldNode,
    pose_is_section_hold_locked,
    pro_pose_is_section_hold_locked,
)
from roka_arm.arm_hold_gate import ArmHoldGate
from roka_arm.dynamixel_driver import DynamixelDriver


def sampled_pose():
    return {
        cfg.JOINT_1_ID: 0,
        cfg.JOINT_2_ID: 647,
        cfg.JOINT_3_ID: 149702,
        "wrist": 840,
        "wrist_reverse": 3245,
        "gripper": 2297,
    }


def test_section_hold_requires_every_arm_and_wrist_axis():
    target = sampled_pose()
    assert pose_is_section_hold_locked(target, target)

    for key in (
        cfg.JOINT_1_ID,
        cfg.JOINT_2_ID,
        cfg.JOINT_3_ID,
        "wrist",
        "wrist_reverse",
        "gripper",
    ):
        drifted = dict(target)
        tolerance = (
            cfg.H54_POSITION_TOLERANCE
            if isinstance(key, int)
            else (
                cfg.GRIPPER_POSITION_TOLERANCE
                if key == "gripper"
                else cfg.WRIST_POSITION_TOLERANCE
            )
        )
        drifted[key] += tolerance + 1
        assert not pose_is_section_hold_locked(drifted, target)


def test_missing_section_hold_pose_is_not_locked():
    target = sampled_pose()
    assert not pose_is_section_hold_locked(None, target)
    assert not pose_is_section_hold_locked({}, target)
    assert not pose_is_section_hold_locked(target, None)
    partial = dict(target)
    partial.pop("wrist_reverse")
    assert not pose_is_section_hold_locked(partial, target)


def test_pro_only_hold_ignores_12v_wrist_and_gripper_axes():
    target = sampled_pose()
    present = dict(target)
    present["wrist"] += 1000
    present["wrist_reverse"] -= 1000
    present["gripper"] = 0
    assert pro_pose_is_section_hold_locked(present, target)
    present[cfg.JOINT_3_ID] += cfg.H54_POSITION_TOLERANCE + 1
    assert not pro_pose_is_section_hold_locked(present, target)


def test_transient_feedback_loss_is_graceful_and_recovers_without_exit():
    class Driver:
        available = False

        def read_current_pose(self):
            pose = sampled_pose()
            pose.pop("wrist_reverse")
            return pose if self.available else None

        def read_xm_position(self, _motor_id):
            return sampled_pose()["wrist_reverse"] if self.available else None

        def read_all_torque_state(self):
            return True if self.available else None

    node = object.__new__(ArmHoldNode)
    node.driver = Driver()
    node.ready = True
    node.communication_grace_count = 2
    node.communication_misses = 0
    node.recovery_warning_emitted = False
    node.hold_deviation_warning_emitted = False
    node.hold_pose = sampled_pose()
    published = []
    node._publish = lambda ready, detail: (
        setattr(node, "ready", ready), published.append((ready, detail))
    )

    node._verify_hold()
    node._verify_hold()
    assert published == []
    node._verify_hold()
    assert published[-1][0] is False

    node.driver.available = True
    node._verify_hold()
    assert published[-1][0] is True
    assert node.communication_misses == 0


def test_torque_loss_recovers_the_validated_section_pose():
    class Driver:
        def read_current_pose(self):
            pose = sampled_pose()
            pose[cfg.JOINT_1_ID] = -18500
            pose.pop("wrist_reverse")
            return pose

        def read_xm_position(self, _motor_id):
            return sampled_pose()["wrist_reverse"]

        def read_all_torque_state(self):
            return False

        def enable_all_torque(self):
            return True

        def move_pose_values(self, _name, pose):
            self.commanded = dict(pose)
            return True

        def wait_until_pose_values_reached(self, _name, _pose):
            return True

    node = object.__new__(ArmHoldNode)
    node.driver = Driver()
    node.ready = True
    node.communication_grace_count = 2
    node.communication_misses = 0
    node.recovery_warning_emitted = False
    node.hold_deviation_warning_emitted = False
    node.hold_pose = sampled_pose()
    published = []
    node._publish = lambda ready, detail: (
        setattr(node, "ready", ready), published.append((ready, detail))
    )

    node._verify_hold()

    assert node.driver.commanded[cfg.JOINT_1_ID] == sampled_pose()[cfg.JOINT_1_ID]
    assert node.hold_pose[cfg.JOINT_1_ID] == sampled_pose()[cfg.JOINT_1_ID]
    assert published[-1][0] is True


def test_one_time_start_recovery_accepts_readable_outside_limit_feedback():
    driver = object.__new__(DynamixelDriver)
    h54 = {
        cfg.JOINT_1_ID: -17933,
        cfg.JOINT_2_ID: 647,
        cfg.JOINT_3_ID: 149702,
    }
    xm = {
        cfg.WRIST_FORWARD_ID: 840,
        cfg.WRIST_REVERSE_ID: 3245,
        cfg.GRIPPER_ID: 2297,
    }
    driver.read_h54_position = lambda motor_id: h54[motor_id]
    driver.read_xm_position = lambda motor_id: xm[motor_id]

    assert driver.check_all_positions(allow_initial_recovery=True)
    assert not driver.check_all_positions(allow_initial_recovery=False)


def test_normal_motion_limit_is_unchanged_after_start_recovery_exception():
    driver = object.__new__(DynamixelDriver)
    assert not driver.validate_arm_position(cfg.JOINT_1_ID, -17933)
    assert driver.validate_arm_position(cfg.JOINT_1_ID, -10000)


def test_pro_only_initialization_never_opens_or_commands_the_12v_bus():
    class Port:
        def __init__(self, forbidden=False):
            self.forbidden = forbidden
            self.opened = False

        def openPort(self):
            assert not self.forbidden
            self.opened = True
            return True

        def setBaudRate(self, _baud):
            assert not self.forbidden
            return True

    driver = object.__new__(DynamixelDriver)
    driver.arm_port = Port()
    driver.wrist_port = Port(forbidden=True)
    driver.arm_opened = False
    driver.wrist_opened = False
    driver.motion_enabled = False
    driver.torque_armed = False
    driver.group_sync_write = None
    driver._lock_files = []
    driver._acquire_port_locks = lambda include_wrist: not include_wrist
    driver.check_arm_positions = lambda **_kwargs: True
    driver.check_all_positions = lambda **_kwargs: (_ for _ in ()).throw(
        AssertionError("full-arm position check must not run")
    )
    driver.set_h54_motion_profile = lambda *_args: True
    driver.set_h54_torque = lambda *_args: True
    driver.move_arm_to_start_pose = lambda: True
    driver.set_xm_profile = lambda *_args: (_ for _ in ()).throw(
        AssertionError("12V profile write must not run")
    )
    driver.set_xm_torque = lambda *_args: (_ for _ in ()).throw(
        AssertionError("12V torque write must not run")
    )
    driver.write_wrist_position = lambda *_args: (_ for _ in ()).throw(
        AssertionError("wrist command must not run")
    )
    driver.open_gripper = lambda: (_ for _ in ()).throw(
        AssertionError("gripper command must not run")
    )

    assert driver.initialize(
        enable_motion=True,
        allow_initial_recovery=True,
        include_wrist=False,
    )
    assert driver.arm_opened
    assert not driver.wrist_opened
    assert driver.motion_enabled


def test_section_manual_gate_requires_arm_lock_external_unlock_and_lb_release():
    gate = object.__new__(ArmHoldGate)
    gate.arm_ready = True
    gate.field_ready = False
    gate.require_field = False
    gate.external_locked = False
    gate.release_required = False
    assert gate._gate_ready()

    gate.external_locked = True
    assert not gate._gate_ready()
    gate.external_locked = False
    gate.release_required = True
    assert not gate._gate_ready()
    gate.release_required = False
    gate.arm_ready = False
    assert not gate._gate_ready()
