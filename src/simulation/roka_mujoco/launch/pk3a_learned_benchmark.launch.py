#!/usr/bin/env python3
"""Run independent reset-to-reset learned-policy benchmark laps."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("roka_mujoco")
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "pk3a_learned_autonomy.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "headless": LaunchConfiguration("headless"),
            "show_ui": LaunchConfiguration("show_ui"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "render_fps": LaunchConfiguration("render_fps"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "auto_start": "false",
            "target_laps": "1",
        }.items(),
    )
    benchmark = Node(
        package="roka_mujoco",
        executable="sim_learned_benchmark",
        name="sim_learned_benchmark",
        parameters=[{
            "use_sim_time": False,
            "runs": ParameterValue(LaunchConfiguration("runs"), value_type=int),
            "output_file": LaunchConfiguration("output_file"),
            "settle_wall_sec": ParameterValue(
                LaunchConfiguration("settle_wall_sec"), value_type=float
            ),
            "episode_timeout_sim_sec": ParameterValue(
                LaunchConfiguration("episode_timeout_sim_sec"), value_type=float
            ),
        }],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
        on_exit=Shutdown(reason="PK-3A learned benchmark finished"),
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("runs", default_value="10"),
        DeclareLaunchArgument("output_file", default_value=""),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("show_ui", default_value="false"),
        DeclareLaunchArgument("window_width", default_value="1600"),
        DeclareLaunchArgument("window_height", default_value="900"),
        DeclareLaunchArgument("render_fps", default_value="30"),
        # Three-times real time occasionally starves the 50 Hz floating-base
        # publisher long enough to trip the controller's 0.30 s simulation-time
        # watchdog. Two-times real time remained deterministic without relaxing
        # the odometry safety timeout.
        DeclareLaunchArgument("sim_speed_factor", default_value="2.0"),
        DeclareLaunchArgument("settle_wall_sec", default_value="0.75"),
        DeclareLaunchArgument("episode_timeout_sim_sec", default_value="600.0"),
        autonomy,
        benchmark,
    ])
