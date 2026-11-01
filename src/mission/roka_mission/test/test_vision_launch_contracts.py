from pathlib import Path


def source(name):
    return (
        Path(__file__).resolve().parents[1] / "launch" / name
    ).read_text(encoding="utf-8")


def test_section2_exposes_hardware_free_validation_arguments():
    text = source("section2_box_pick.launch.py")
    for argument in (
        "start_realsense",
        "connect_arm_hardware",
        "enable_arm_motion",
        "visualize_arm",
        "start_navigation",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in text
        assert f'LaunchConfiguration("{argument}")' in text


def test_each_vision_mission_has_exactly_one_camera_owner():
    section1 = source("section1_fog_hybrid.launch.py")
    section2 = source("section2_box_pick.launch.py")
    section3 = source("section3_marker_vision.launch.py")
    assert section1.count('"spring.launch.py"') == 1
    assert section2.count('"roka_arm.launch.py"') == 1
    assert "summer.launch.py" not in section2
    assert section2.count('"traffic.launch.py"') == 1
    assert '"enabled": "false"' in section2
    assert '"start_realsense": "false"' in section2
    assert section3.count('"autumn.launch.py"') == 1
    for text in (section1, section2, section3):
        assert text.count('DeclareLaunchArgument("start_realsense"') == 1


def test_section_start_poses_are_wired_to_their_runtime_launches():
    section2 = source("section2_box_pick.launch.py")
    section3 = source("section3_marker_vision.launch.py")
    section5 = source("section5_leader_follow.launch.py")
    assert '"start_pose_name": "section2_start"' in section2
    assert '"start_pose_name": "section3"' in section3
    assert '"start_pose_name": "section5"' in section5


def test_section3_and_section4_add_route_drive_without_inventing_task_logic():
    section3 = source("section3_marker_vision.launch.py")
    section4 = source("section4_route.launch.py")
    for text, section in ((section3, "section_3"), (section4, "section_4")):
        assert f'DeclareLaunchArgument("map_id", default_value="{section}")' in text
        assert '"pk3a_field.launch.py"' in text
        assert '"recorded_route_mode": "true"' in text
        assert '"autonomy_optional_xbox": "true"' in text
        assert 'LaunchConfiguration("start_navigation")' in text
    assert '"autumn.launch.py"' in section3
    assert '"camera_view.launch.py"' in section4


def test_mission_critical_controllers_shutdown_their_parent_launch():
    for name in ("section1_fog_hybrid.launch.py", "section2_box_pick.launch.py"):
        text = source(name)
        assert "RegisterEventHandler(OnProcessExit(" in text
        assert "EmitEvent(event=Shutdown(" in text


def test_section2_traffic_result_controls_the_recorded_route():
    text = source("section2_box_pick.launch.py")
    assert 'executable="traffic_signal_route_gate"' in text
    assert '"config", "traffic_stop_go.yaml"' in text
    assert "target_action=traffic_gate" in text


def test_section5_has_locked_target_and_fail_closed_command_chain():
    text = source("section5_leader_follow.launch.py")
    assert 'default_value="1.20"' in text
    assert 'default_value="112.0"' in text
    assert 'default_value="180.0"' in text
    assert '"curvature_speed_gain": 0.40' in text
    assert 'executable="section5_lifecycle_gate"' in text
    assert '"output_topic": "/cmd_vel_mission5_lifecycle_safe"' in text
    assert '"input_topic": "/cmd_vel_mission5_lifecycle_safe"' in text
    assert 'name="section5_takeover_mux"' in text
    assert 'executable="roka_xbox_teleop"' in text
    assert '"manual_input_topic": "/cmd_vel_joy_raw"' in text
    assert '"gate_manual_commands": True' in text
    assert 'DeclareLaunchArgument("driver_maximum_speed_mps", default_value="1.20")' in text
    assert '("cmd_vel", "/cmd_vel_mission5_selected")' in text
    assert '"mission5_stop_service_enabled": True' in text
    assert '"state_request_timeout_sec": 1.0' in text
    assert '"/cmd_vel_mission5_lifecycle_safe"' in text
    assert '"/cmd_vel_arm_locked"' in text
    assert '"/md/commanded_rpm"' in text
    assert '"final_surface_clearance_m": 0.10' in text
    assert '"initial_maximum_abs_y_m": 1.0' in text
    assert '"maximum_target_extent_x_m": 1.2' in text
    assert '"maximum_target_extent_y_m": 0.6' in text
    assert '"maximum_target_extent_z_m": 2.0' in text
    assert '"fixed_frame": "mission5_odom"' in text
    assert '"detection_timeout_sec": 0.70' in text
    assert '"reacquisition_confirmations": 3' in text
    assert '"predicted_tail_repair_threshold_m": 0.20' in text
    assert '"curve_mode_enter_curvature_inv_m": 0.16' in text
    assert '"curve_mode_exit_curvature_inv_m": 0.10' in text
    assert '"straight_lookahead_m": 1.50' in text
    assert '"straight_maximum_angular_rps": 0.35' in text
    assert '"maximum_turn_linear_speed_mps": 0.45' in text
    assert '"rotate_to_heading_angle_rad": 0.65' in text
    assert 'DeclareLaunchArgument("record_diagnostics", default_value="true")' in text
    assert '"/mission5/tracking_odom"' in text
    assert '"/mission5/control_mode"' in text
    assert text.count("RegisterEventHandler(OnProcessExit(") >= 2


def test_section5_operator_command_is_profile_independent_flat_mission():
    launcher = Path(__file__).resolve().parents[4] / "roka"
    text = launcher.read_text(encoding="utf-8")
    mission5_case = text.split("  mission5)", 1)[1].split("  drive)", 1)[0]
    assert "require_profile" not in mission5_case
    assert "1.0 m/s 이상은 실제 section_5 트랙 전용" not in mission5_case
    assert "1.2 m/s는 실제 section_5 트랙 전용" not in mission5_case
    assert 'run_filtered "평지 섹션5 선두추종' in mission5_case
    assert "roka mission5 1.00" in text
