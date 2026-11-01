#!/usr/bin/env python3
"""Record a bounded Section 1 fog straightaway with the Xbox."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_file"),
        DeclareLaunchArgument("route_profile_file"),
        DeclareLaunchArgument("exit_progress_ratio", default_value="0.30"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"),
                "launch",
                "arm_locked_xbox_drive.launch.py",
            ])),
            launch_arguments={
                "allow_motion": LaunchConfiguration("md_allow_motion"),
                "joy_device_id": "auto",
                "cmd_vel_topic": "/cmd_vel_joy",
                "pending_cmd_vel_topic": "/cmd_vel_joy_raw",
                "require_field_preflight": "false",
                "log_level": "info",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_imu_driver"), "launch", "ebimu.launch.py"
            ])),
            launch_arguments={
                "imu_config": PathJoinSubstitution([
                    FindPackageShare("roka_imu_driver"), "config", "ebimu_v5.yaml",
                ]),
                "serial_port": "auto",
                "allow_sensor_configuration": "false",
                "log_level": "warn",
            }.items(),
        ),
        Node(
            package="roka_mission",
            executable="blind_segment_recorder",
            output="screen",
            parameters=[{
                "output_file": LaunchConfiguration("output_file"),
                "route_profile_file": LaunchConfiguration("route_profile_file"),
                "exit_progress_ratio": LaunchConfiguration("exit_progress_ratio"),
            }],
        ),
    ])
