from roka_arm import arm_config as cfg
from roka_arm.manual_arm_joy import ManualTargets, integrate_targets, shaped_axis


def test_deadzone_is_continuous_and_symmetric():
    assert shaped_axis(0.10, 0.18) == 0.0
    assert shaped_axis(-0.10, 0.18) == 0.0
    assert shaped_axis(1.0, 0.18) == 1.0
    assert shaped_axis(-1.0, 0.18) == -1.0


def test_manual_targets_map_id13_id11_and_paired_wrist_reference():
    targets = ManualTargets(joint_13=125480, joint_11=0, wrist_4=1100)
    updated = integrate_targets(
        targets,
        joint_13_axis=1.0,
        joint_11_axis=-1.0,
        wrist_axis=1.0,
        dt=0.1,
        deadzone=0.18,
        joint_13_rate=8000.0,
        joint_11_rate=5000.0,
        wrist_rate=300.0,
    )
    assert updated == ManualTargets(
        joint_13=126280,
        joint_11=-500,
        wrist_4=1130,
    )
    # The node sends this single wrist_4 reference through the existing
    # DynamixelDriver.write_wrist_position GroupSyncWrite for IDs 4 and 6.
    assert 4096 - updated.wrist_4 == 2966


def test_manual_targets_never_cross_existing_hardware_limits():
    targets = ManualTargets(
        joint_13=cfg.ARM_POSITION_LIMITS[cfg.JOINT_3_ID][1],
        joint_11=cfg.ARM_POSITION_LIMITS[cfg.JOINT_1_ID][0],
        wrist_4=cfg.WRIST_MAX,
    )
    updated = integrate_targets(
        targets,
        joint_13_axis=1.0,
        joint_11_axis=-1.0,
        wrist_axis=1.0,
        dt=0.2,
        deadzone=0.18,
        joint_13_rate=8000.0,
        joint_11_rate=5000.0,
        wrist_rate=300.0,
    )
    assert updated == targets
