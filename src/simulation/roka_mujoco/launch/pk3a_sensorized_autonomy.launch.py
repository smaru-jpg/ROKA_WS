#!/usr/bin/env python3
"""Run proven MuJoCo autonomy with an isolated IMU/wheel EKF observer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            "auto_start": LaunchConfiguration("auto_start"),
            "target_laps": LaunchConfiguration("target_laps"),
        }.items(),
    )
    observer = Node(
        package="robot_localization",
        executable="ekf_node",
        name="sim_sensor_ekf",
        output="screen",
        parameters=[os.path.join(share, "config", "sim_imu_wheel_ekf.yaml")],
        remappings=[("odometry/filtered", "/sim/odom/filtered")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("show_ui", default_value="true"),
            DeclareLaunchArgument("window_width", default_value="1600"),
            DeclareLaunchArgument("window_height", default_value="900"),
            DeclareLaunchArgument("render_fps", default_value="30"),
            DeclareLaunchArgument("sim_speed_factor", default_value="1.0"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("target_laps", default_value="0"),
            autonomy,
            observer,
        ]
    )
