#!/usr/bin/env python3
"""Run the real Nav2/safety command chain against the MuJoCo field vehicle."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    hardware = load_hardware_config()
    simulation_share = get_package_share_directory("roka_mujoco")
    navigation_share = get_package_share_directory("roka_nav2_bringup")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch", "pk3a_track.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "with_robot": "true",
            "controllers_file": os.path.join(
                simulation_share, "config", "mujoco_safe_controllers.yaml"
            ),
            "scene_file": os.path.join(
                simulation_share,
                "description",
                "pk3a",
                "models",
                "pk3a_mapped_flat_scene.xml",
            ),
            "headless": LaunchConfiguration("headless"),
            "show_ui": LaunchConfiguration("show_ui"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "render_fps": LaunchConfiguration("render_fps"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "camera_publish_rate": "5",
        }.items(),
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "sim_imu_broadcaster",
            "sim_diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
            "--activate-as-group",
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output="screen",
    )
    bridge = Node(
        package="roka_mujoco",
        executable="sim_nav2_bridge",
        name="sim_nav2_bridge",
        output="screen",
        parameters=[{
            # Keep the real navigation stack on wall time. The bridge replaces
            # simulator stamps, matching actual field timing and watchdogs.
            "use_sim_time": False,
            "profile_file": LaunchConfiguration("profile_file"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_footprint_to_base_link",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", str(hardware.wheel_radius_m),
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_footprint", "--child-frame-id", "base_link",
            "--ros-args", "--log-level", LaunchConfiguration("log_level"),
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "lidar_md_navigation.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "track_profile": LaunchConfiguration("profile_file"),
            "use_sim_time": "false",
            "use_lidar_driver": "false",
            "use_lidar_static_tf": "false",
            "use_stabilized_scan": "false",
            "use_md_driver": "false",
            "use_imu_driver": "false",
            "use_imu_ekf": "false",
            "use_dry_run_map_tf": "true",
            "use_terrain_mapping": "true",
            "use_safety_pipeline": "true",
            "use_collision_monitor": "true",
            "collision_source_timeout": "0.60",
            "supervisor_require_terrain": "false",
            "supervisor_allow_level_fallback": "true",
            "supervisor_require_imu": "true",
            "supervisor_require_odom": "true",
            "supervisor_enable_slip_detection": "false",
            "supervisor_body_odom_topic": "/odom",
            "supervisor_wheel_odom_topic": "/sim_diff_drive_controller/odom",
            "use_track_follower": "true",
            "track_auto_start": LaunchConfiguration("auto_start"),
            "require_field_preflight": "false",
            "initialize_from_profile_start": "false",
            "track_loop": "false",
            "allow_test_profile": "true",
            "direct_recorded_route": LaunchConfiguration("direct_recorded_route"),
            "operation_profile": "flat_commissioning",
            "maximum_start_distance_m": "0.75",
            "maximum_start_heading_error_deg": "90.0",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("show_ui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("window_width", default_value="1600"),
        DeclareLaunchArgument("window_height", default_value="900"),
        DeclareLaunchArgument("render_fps", default_value="30"),
        DeclareLaunchArgument("sim_speed_factor", default_value="1.0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("direct_recorded_route", default_value="false"),
        DeclareLaunchArgument(
            "map",
            default_value=str(workspace_path("maps", "section_1", "occupancy.yaml")),
        ),
        DeclareLaunchArgument(
            "profile_file",
            default_value=str(workspace_path(
                "maps", "section_1", "track_profile_test.yaml"
            )),
        ),
        simulator,
        spawner,
        bridge,
        footprint_tf,
        navigation,
    ])
