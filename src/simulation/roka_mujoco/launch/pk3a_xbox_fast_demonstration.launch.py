#!/usr/bin/env python3
"""Record staged 0.40/0.55 m/s Xbox demonstrations in MuJoCo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    share = get_package_share_directory("roka_mujoco")
    demonstration_launch = os.path.join(
        share, "launch", "pk3a_xbox_demonstration.launch.py"
    )
    arguments = {
        "log_level": LaunchConfiguration("log_level"),
        "joy_device_id": LaunchConfiguration("joy_device_id"),
        "camera_view": LaunchConfiguration("camera_view"),
        "headless": LaunchConfiguration("headless"),
        "show_ui": LaunchConfiguration("show_ui"),
        "window_width": LaunchConfiguration("window_width"),
        "window_height": LaunchConfiguration("window_height"),
        "render_fps": LaunchConfiguration("render_fps"),
        "camera_publish_rate": LaunchConfiguration("camera_publish_rate"),
        "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
        "output_directory": LaunchConfiguration("output_directory"),
        "session_name": LaunchConfiguration("session_name"),
        "controller_config_file": os.path.join(
            share, "config", "mujoco_xbox_fast_controllers.yaml"
        ),
        "teleop_config_file": os.path.join(
            share, "config", "mujoco_xbox_fast_teleop.yaml"
        ),
    }
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("camera_view", default_value="true"),
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
        DeclareLaunchArgument("session_name", default_value="fast_lap"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demonstration_launch),
            launch_arguments=arguments.items(),
        ),
    ])
