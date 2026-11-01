from pathlib import Path

from roka_arm import arm_config as cfg
from roka_arm.preflight import assignment_check


def test_active_split_control_values_are_preserved():
    assert cfg.TARGET_X == 355
    assert cfg.TARGET_Y == 360
    assert cfg.GRIPPER_CLOSE_POSITION == 2300
    assert cfg.Y_DELTA == 15
    assert cfg.X_COMMAND_INTERVAL == 0.15
    assert cfg.Y_COMMAND_INTERVAL == 0.25
    assert cfg.Z_COMMAND_INTERVAL == 0.15
    assert cfg.ACTIVATION_MIN_Y_RATIO == 0.25


def test_deployment_yaml_keeps_torque_latched_when_runtime_closes():
    package = Path(__file__).parents[1]
    # The pasted standalone default releases torque, while all deployed launch
    # paths explicitly retain the pose through their ROS parameter files.
    assert cfg.TORQUE_OFF_ON_EXIT is True
    for filename in ("arm_hold.yaml", "roka_arm.yaml"):
        text = (package / "config" / filename).read_text(encoding="utf-8")
        assert "torque_off_on_exit: false" in text


def test_all_section_start_poses_are_named_and_within_limits():
    assert cfg.get_named_start_pose("section_1") == cfg.SECTION1_POSE
    assert cfg.get_named_start_pose("section2") == cfg.SECTION2_START_POSE
    assert cfg.get_named_start_pose("section_3") == cfg.SECTION3_POSE
    assert cfg.get_named_start_pose("section4") == cfg.SECTION4_POSE
    assert cfg.get_named_start_pose("section_5") == cfg.SECTION5_POSE
    assert cfg.get_named_start_pose("auto") == cfg.SECTION2_START_POSE


def test_no_raw_ttyusb_or_foreign_home_in_runtime_sources():
    package = Path(__file__).parents[1] / "roka_arm"
    source = "\n".join(
        path.read_text(errors="replace") for path in package.glob("*.py")
    )
    assert "/dev/ttyUSB" not in source
    assert "/home/ksmr" not in source


def test_motion_requires_hardware():
    values = {
        "arm_serial_port": "/dev/roka_arm_rs485",
        "wrist_serial_port": "/dev/roka_arm_ttl",
        "connect_hardware": False,
        "enable_motion": True,
        "torque_off_on_exit": True,
        "color_topic": "/color",
        "depth_topic": "/depth",
        "model_path": "/model.pt",
        "visualize": False,
        "inference_device": "cpu",
        "inference_rate_hz": 10.0,
        "maximum_depth_age_sec": 0.12,
        "target_class_ids": [0, 1],
        "activation_min_y_ratio": 0.0,
        "auto_control_start": False,
        "target_x": 355,
        "target_y": 360,
        "target_depth_mm": 250,
        "y_delta": 15,
        "x_command_interval": 0.15,
        "y_command_interval": 0.25,
        "z_command_interval": 0.15,
    }
    try:
        cfg.apply_runtime_overrides(values)
    except ValueError as error:
        assert "requires connect_hardware" in str(error)
    else:
        raise AssertionError("motion was accepted without hardware")


def test_serial_assignment_rejects_duplicate_and_reserved_device(tmp_path):
    first = tmp_path / "tty0"
    second = tmp_path / "tty1"
    first.touch()
    second.touch()
    arm = tmp_path / "arm"
    wrist = tmp_path / "wrist"
    arm.symlink_to(first)
    wrist.symlink_to(first)
    valid, detail = assignment_check((arm, wrist), ())
    assert not valid
    assert "same device" in detail

    wrist.unlink()
    wrist.symlink_to(second)
    valid, _ = assignment_check((arm, wrist), (second,))
    assert not valid

    valid, _ = assignment_check((arm, wrist), ())
    assert valid
