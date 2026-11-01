#!/usr/bin/env python3
"""Xbox base drive gated by a torque-held section start pose."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("allow_motion", default_value="false"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("pending_cmd_vel_topic", default_value="/cmd_vel_arm_pending"),
        DeclareLaunchArgument("require_field_preflight", default_value="false"),
        DeclareLaunchArgument("hold_scope", default_value="all"),
        DeclareLaunchArgument("max_linear_speed", default_value="1.00"),
        DeclareLaunchArgument("max_wheel_rpm", default_value="112.0"),
        DeclareLaunchArgument(
            "feedback_overspeed_limit_rpm", default_value="130.0"
        ),
        DeclareLaunchArgument("log_level", default_value="warn"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"), "launch", "arm_hold.launch.py",
            ])),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level"),
                "hold_scope": LaunchConfiguration("hold_scope"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_md_driver"), "launch",
                "mdh250_md200t_xbox.launch.py",
            ])),
            launch_arguments={
                "simulation_mode": "false",
                "allow_motion": LaunchConfiguration("allow_motion"),
                "joy_device_id": LaunchConfiguration("joy_device_id"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "teleop_cmd_vel_topic": LaunchConfiguration("pending_cmd_vel_topic"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_wheel_rpm": LaunchConfiguration("max_wheel_rpm"),
                "feedback_overspeed_limit_rpm": LaunchConfiguration(
                    "feedback_overspeed_limit_rpm"
                ),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        Node(
            package="roka_arm",
            executable="roka_arm_hold_gate",
            name="arm_hold_cmd_gate",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_topic": LaunchConfiguration("pending_cmd_vel_topic"),
                "output_topic": LaunchConfiguration("cmd_vel_topic"),
                "require_field_preflight": ParameterValue(
                    LaunchConfiguration("require_field_preflight"), value_type=bool
                ),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
