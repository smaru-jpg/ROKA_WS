#!/usr/bin/env python3
"""Repeat the section_1 mapped path from five controlled start errors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    share = get_package_share_directory("roka_mujoco")
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "pk3a_mapped_autonomy.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "headless": "true",
            "show_ui": "false",
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "auto_start": "false",
            "profile_file": LaunchConfiguration("profile_file"),
            "speed_scale": LaunchConfiguration("speed_scale"),
        }.items(),
    )
    benchmark = Node(
        package="roka_mujoco",
        executable="sim_learned_benchmark",
        name="sim_mapped_benchmark",
        parameters=[{
            "use_sim_time": False,
            "runs": ParameterValue(LaunchConfiguration("runs"), value_type=int),
            "output_file": LaunchConfiguration("output_file"),
            "status_topic": "/sim/mapped_controller/status",
            "controller_start_service": "/sim_mapped_profile_controller/start",
            "controller_stop_service": "/sim_mapped_profile_controller/stop",
            "settle_wall_sec": 0.75,
            "episode_timeout_sim_sec": 90.0,
            "reset_keyframes": [
                "mapped_nominal",
                "mapped_left_8cm",
                "mapped_right_8cm",
                "mapped_yaw_left_8deg",
                "mapped_yaw_right_8deg",
            ],
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        on_exit=Shutdown(reason="mapped-path benchmark finished"),
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("runs", default_value="5"),
        DeclareLaunchArgument("output_file", default_value=""),
        DeclareLaunchArgument("sim_speed_factor", default_value="2.0"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "profile_file",
            default_value=str(workspace_path(
                "maps", "section_1", "track_profile_test.yaml"
            )),
        ),
        autonomy,
        benchmark,
    ])
