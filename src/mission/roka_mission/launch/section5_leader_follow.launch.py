#!/usr/bin/env python3
"""Section 5: mapless VLP-16 leader trail following with occlusion recovery."""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_path
from roka_md_driver.xbox_identity import resolve_assigned_sdl_device_id


def _optional_xbox_takeover(context):
    try:
        device_id = resolve_assigned_sdl_device_id(
            LaunchConfiguration("joy_device_id").perform(context)
        )
    except RuntimeError as error:
        return [LogInfo(msg=f"[MISSION5] Optional Xbox takeover unavailable: {error}")]
    xbox_config = PathJoinSubstitution([
        FindPackageShare("roka_md_driver"), "config", "xbox_base_teleop.yaml",
    ])
    return [
        Node(
            package="joy",
            executable="joy_node",
            name="section5_joy_node",
            output="screen",
            parameters=[{
                "device_id": device_id,
                "deadzone": 0.12,
                "autorepeat_rate": 20.0,
                "coalesce_interval_ms": 1,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_md_driver",
            executable="roka_xbox_teleop",
            # Match the YAML node key so the LB/LT scaling parameters apply.
            name="roka_xbox_teleop",
            output="screen",
            parameters=[xbox_config, {"cmd_vel_topic": "/cmd_vel_joy_raw"}],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="xbox_autonomy_controls",
            name="section5_xbox_takeover",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "manual_input_topic": "/cmd_vel_joy_raw",
                "manual_command_topic": "/cmd_vel_joy",
                "gate_manual_commands": True,
                "allow_autonomy_start_controls": False,
                "track_stop_service_enabled": False,
                "mission5_stop_service_enabled": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ]


def generate_launch_description():
    hardware = load_hardware_config()
    diagnostic_root = workspace_path("reports", "mission5_runs")
    diagnostic_root.mkdir(parents=True, exist_ok=True)
    diagnostic_bag = diagnostic_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    footprint_lidar_z = hardware.footprint_to_base.xyz[2] + hardware.base_to_lidar.xyz[2]
    maximum_speed = LaunchConfiguration("maximum_speed_mps")
    driver_maximum_speed = LaunchConfiguration("driver_maximum_speed_mps")
    maximum_rpm = LaunchConfiguration("maximum_wheel_rpm")
    feedback_guard = LaunchConfiguration("feedback_overspeed_limit_rpm")
    tracker = Node(
        package="roka_terrain_mapping",
        executable="leader_follower_node",
        name="section5_leader_follower",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "maximum_linear_speed_mps": ParameterValue(maximum_speed, value_type=float),
            "desired_distance_m": 2.0,
            "fixed_frame": "mission5_odom",
            "odom_frame": "odom",
            "detection_timeout_sec": 0.70,
            "cloud_timeout_sec": 0.40,
            "association_gate_m": 2.0,
            "association_gate_sigma": 7.0,
            "initial_maximum_abs_y_m": 1.0,
            "maximum_target_extent_x_m": 1.2,
            "maximum_target_extent_y_m": 0.6,
            "maximum_target_extent_z_m": 2.0,
            "initial_minimum_speed_mps": 0.15,
            "maximum_candidate_speed_mps": 2.0,
            "maximum_candidate_acceleration_mps2": 4.0,
            "maximum_direction_change_rate_degps": 120.0,
            "reacquisition_confirmations": 3,
            "predicted_tail_repair_threshold_m": 0.20,
            "maximum_prediction_uncertainty_m": 5.0,
            "curvature_speed_gain": 0.40,
            "minimum_curve_speed_scale": 0.45,
            "curve_detection_distance_m": 2.0,
            "curve_mode_enter_curvature_inv_m": 0.16,
            "curve_mode_exit_curvature_inv_m": 0.10,
            "straight_lookahead_m": 1.50,
            "turn_minimum_lookahead_m": 0.60,
            "turn_maximum_lookahead_m": 0.80,
            "straight_heading_gain": 1.20,
            "straight_cross_track_gain": 0.90,
            "straight_heading_deadband_deg": 3.0,
            "straight_cross_track_deadband_m": 0.05,
            "straight_maximum_angular_rps": 0.35,
            "maximum_turn_linear_speed_mps": 0.45,
            "rotate_to_heading_angle_rad": 0.65,
            "minimum_rotate_angular_rps": 0.35,
            "wall_minimum_horizontal_span_m": 2.0,
            "footprint_front_m": hardware.navigation_front_m,
            "footprint_rear_m": hardware.navigation_rear_m,
            "footprint_half_width_m": hardware.navigation_half_width_m,
            "final_surface_clearance_m": 0.10,
            "clearance_release_hysteresis_m": 0.15,
            "braking_pipeline_latency_sec": 0.30,
            "braking_deceleration_mps2": 1.0,
            # The nominal screened straight is about 5 m. At the slower
            # 0.8 m/s leader speed it lasts about 6.25 s.
            "occlusion_coast_timeout_sec": 6.5,
            "lost_lock_timeout_sec": 10.0,
            "auto_enable": True,
        }],
        # Target-state messages are emitted only when the state changes. Keep
        # this one node at INFO so the operator can distinguish acquisition,
        # locked tracking and a sensor/attitude stop without every driver log.
        arguments=["--ros-args", "--log-level", "info"],
    )
    lifecycle_gate = Node(
        package="roka_mission",
        executable="section5_lifecycle_gate",
        name="section5_lifecycle_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": "/cmd_vel_mission5_safe",
            "output_topic": "/cmd_vel_mission5_lifecycle_safe",
            "input_timeout_sec": 0.30,
            "check_period_sec": 0.25,
            "state_request_timeout_sec": 1.0,
        }],
        # Always expose ACTIVE/retry state because this is the final Nav2
        # safety gate before the arm interlock and motor driver.
        arguments=["--ros-args", "--log-level", "info"],
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("maximum_speed_mps", default_value="1.20"),
        DeclareLaunchArgument("driver_maximum_speed_mps", default_value="1.20"),
        DeclareLaunchArgument("maximum_wheel_rpm", default_value="112.0"),
        DeclareLaunchArgument(
            "feedback_overspeed_limit_rpm", default_value="180.0"
        ),
        DeclareLaunchArgument("imu_serial_port", default_value="auto"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("record_diagnostics", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--output", str(diagnostic_bag),
                "--storage", "sqlite3",
                "--max-bag-size", str(512 * 1024 * 1024),
                "--log-level", "warn",
                "/odom", "/mission5/tracking_odom", "/imu/data",
                "/md/left/raw_state", "/md/right/raw_state",
                "/mission5/status", "/mission5/gap_m",
                "/mission5/leader_trail", "/mission5/control_mode",
                "/mission5/local_curvature", "/mission5/heading_error_rad",
                "/mission5/cross_track_error_m",
                "/cmd_vel_mission5_raw", "/cmd_vel_mission5_selected",
                "/cmd_vel_mission5_smoothed", "/cmd_vel_mission5_safe",
                "/cmd_vel_mission5_lifecycle_safe", "/cmd_vel_arm_locked",
                "/cmd_vel_joy_raw", "/cmd_vel_joy", "/md/commanded_rpm",
                "/tf", "/tf_static",
            ],
            output="log",
            condition=IfCondition(LaunchConfiguration("record_diagnostics")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"), "launch", "arm_hold.launch.py",
            ])),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level"),
                "start_pose_name": "section5",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"), "launch", "lidar_md_base.launch.py",
            ])),
            launch_arguments={
                "use_lidar_driver": "true",
                "use_md_driver": "true",
                "md_simulation_mode": "false",
                "md_allow_motion": LaunchConfiguration("md_allow_motion"),
                "md_cmd_vel_topic": "/cmd_vel_arm_locked",
                "md_max_linear_speed": driver_maximum_speed,
                "md_max_wheel_rpm": maximum_rpm,
                "md_feedback_overspeed_limit_rpm": feedback_guard,
                "md_odom_topic": "/odom",
                "md_base_frame": "base_footprint",
                "md_publish_tf": "true",
                "base_frame": "base_footprint",
                "lidar_z": str(footprint_lidar_z),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_imu_driver"), "launch", "ebimu.launch.py",
            ])),
            launch_arguments={
                "serial_port": LaunchConfiguration("imu_serial_port"),
                "allow_sensor_configuration": "false",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="section5_base_to_imu_tf",
            output="screen",
            arguments=[
                "--x", str(hardware.base_to_imu.xyz[0]),
                "--y", str(hardware.base_to_imu.xyz[1]),
                "--z", str(hardware.footprint_to_base.xyz[2] + hardware.base_to_imu.xyz[2]),
                "--roll", str(hardware.base_to_imu.rpy[0]),
                "--pitch", str(hardware.base_to_imu.rpy[1]),
                "--yaw", str(hardware.base_to_imu.rpy[2]),
                "--frame-id", "base_footprint",
                "--child-frame-id", hardware.imu_frame,
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="pk3a_field_preflight",
            name="section5_field_preflight",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "duration_sec": 8.0,
                "failed_check_retry_sec": 5.0,
                "retry_until_ready": True,
                "require_xbox": False,
                "require_arm_hold": True,
                "require_lio_stability": False,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_arm",
            executable="roka_arm_hold_gate",
            name="section5_arm_hold_cmd_gate",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_topic": "/cmd_vel_mission5_lifecycle_safe",
                "output_topic": "/cmd_vel_arm_locked",
                "require_field_preflight": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="mission5_velocity_smoother",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("roka_mission"), "config", "section5_safety.yaml",
            ])],
            remappings=[
                ("cmd_vel", "/cmd_vel_mission5_selected"),
                ("cmd_vel_smoothed", "/cmd_vel_mission5_smoothed"),
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="mission5_collision_monitor",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("roka_mission"), "config", "section5_safety.yaml",
            ])],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="mission5_safety_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": [
                    "mission5_velocity_smoother",
                    "mission5_collision_monitor",
                ],
                "bond_timeout": 4.0,
                "attempt_respawn_reconnection": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        lifecycle_gate,
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="section5_takeover_mux",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("roka_mission"), "config", "section5_takeover_mux.yaml",
            ])],
            remappings=[("cmd_vel_out", "/cmd_vel_mission5_selected")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        OpaqueFunction(function=_optional_xbox_takeover),
        RegisterEventHandler(OnProcessExit(
            target_action=lifecycle_gate,
            on_exit=[EmitEvent(event=Shutdown(reason="Section-5 lifecycle gate stopped"))],
        )),
        tracker,
        RegisterEventHandler(OnProcessExit(
            target_action=tracker,
            on_exit=[EmitEvent(event=Shutdown(reason="Section-5 tracker stopped"))],
        )),
        Node(
            package="rviz2",
            executable="rviz2",
            name="section5_leader_rviz",
            output="screen",
            arguments=[
                "-d", PathJoinSubstitution([
                    FindPackageShare("roka_terrain_mapping"), "rviz", "leader_follow.rviz",
                ]),
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
