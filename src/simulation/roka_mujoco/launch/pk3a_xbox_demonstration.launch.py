#!/usr/bin/env python3
"""Run Xbox MuJoCo teleoperation and record a learning demonstration."""

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
    xbox_launch = os.path.join(share, "launch", "pk3a_robot_xbox.launch.py")
    default_controller_config = os.path.join(
        share, "config", "mujoco_xbox_controllers.yaml"
    )
    default_teleop_config = os.path.join(
        share, "config", "mujoco_xbox_teleop.yaml"
    )
    xbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xbox_launch),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "joy_device_id": LaunchConfiguration("joy_device_id"),
            "camera_view": LaunchConfiguration("camera_view"),
            "controller_config_file": LaunchConfiguration(
                "controller_config_file"
            ),
            "teleop_config_file": LaunchConfiguration("teleop_config_file"),
            "headless": LaunchConfiguration("headless"),
            "show_ui": LaunchConfiguration("show_ui"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "render_fps": LaunchConfiguration("render_fps"),
            "camera_publish_rate": LaunchConfiguration("camera_publish_rate"),
        }.items(),
    )
    recorder = Node(
        package="roka_mujoco",
        executable="record_xbox_demonstration",
        name="sim_demonstration_recorder",
        parameters=[{
            "use_sim_time": True,
            "output_directory": LaunchConfiguration("output_directory"),
            "session_name": LaunchConfiguration("session_name"),
            "sample_rate_hz": ParameterValue(
                LaunchConfiguration("sample_rate_hz"), value_type=float
            ),
            "auto_record": True,
        }],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("camera_view", default_value="true"),
        DeclareLaunchArgument(
            "controller_config_file",
            default_value=default_controller_config,
        ),
        DeclareLaunchArgument(
            "teleop_config_file",
            default_value=default_teleop_config,
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("show_ui", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="1600"),
        DeclareLaunchArgument("window_height", default_value="900"),
        DeclareLaunchArgument("render_fps", default_value="30"),
        DeclareLaunchArgument("camera_publish_rate", default_value="15"),
        DeclareLaunchArgument("sample_rate_hz", default_value="30"),
        DeclareLaunchArgument(
            "output_directory",
            default_value=str(workspace_path("datasets", "xbox_demonstrations")),
        ),
        DeclareLaunchArgument("session_name", default_value=""),
        xbox,
        recorder,
    ])
