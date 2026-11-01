#!/usr/bin/env python3
"""Run hardware-free low-slope PK-3A closed-loop autonomy in MuJoCo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    share = get_package_share_directory("roka_mujoco")
    track_launch = os.path.join(share, "launch", "pk3a_track.launch.py")
    controller_file = os.path.join(share, "config", "mujoco_safe_controllers.yaml")
    autonomy_file = os.path.join(share, "config", "pk3a_safe_autonomy.yaml")
    mux_file = os.path.join(share, "config", "mujoco_twist_mux.yaml")
    scene_file = os.path.join(
        share, "description", "pk3a", "models", "pk3a_robot_scene.xml"
    )
    profile_default = str(workspace_path(
        "maps", "sim_pk3a_ground_truth", "pk3a_safe_path.yaml"
    ))
    map_default = str(workspace_path(
        "maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.npz"
    ))
    metadata_default = str(workspace_path(
        "maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.json"
    ))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(track_launch),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "with_robot": "true",
            "controllers_file": controller_file,
            "scene_file": scene_file,
            "headless": LaunchConfiguration("headless"),
            "show_ui": LaunchConfiguration("show_ui"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "render_fps": LaunchConfiguration("render_fps"),
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
    mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace="sim",
        name="twist_mux",
        parameters=[mux_file, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/sim_diff_drive_controller/cmd_vel_unstamped")],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )
    controller = Node(
        package="roka_mujoco",
        executable="sim_safe_path_controller",
        name="sim_safe_path_controller",
        parameters=[
            autonomy_file,
            {
                "profile_file": LaunchConfiguration("profile_file"),
                "ground_truth_map_file": LaunchConfiguration("ground_truth_map_file"),
                "ground_truth_metadata_file": LaunchConfiguration("ground_truth_metadata_file"),
                "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
                "target_laps": ParameterValue(LaunchConfiguration("target_laps"), value_type=int),
                "speed_scale": ParameterValue(LaunchConfiguration("speed_scale"), value_type=float),
            },
        ],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )
    sensor_adapter = Node(
        package="roka_mujoco",
        executable="sim_sensor_adapter",
        name="sim_sensor_adapter",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("show_ui", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="1600"),
        DeclareLaunchArgument("window_height", default_value="900"),
        DeclareLaunchArgument("render_fps", default_value="30"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("target_laps", default_value="0"),
        DeclareLaunchArgument("speed_scale", default_value="2.0"),
        DeclareLaunchArgument("profile_file", default_value=profile_default),
        DeclareLaunchArgument("ground_truth_map_file", default_value=map_default),
        DeclareLaunchArgument("ground_truth_metadata_file", default_value=metadata_default),
        simulator,
        spawner,
        mux,
        sensor_adapter,
        controller,
    ])
