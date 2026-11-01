"""Record post-mapping real Xbox traversability without owning cmd_vel."""

import logging

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.workspace import workspace_path


def _configure_launch_logging(_context):
    launch.logging.launch_config.level = logging.INFO
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_configure_launch_logging),
        DeclareLaunchArgument(
            "profile_file",
            default_value=str(workspace_path("maps", "track_v1", "track_profile.yaml")),
        ),
        DeclareLaunchArgument(
            "output_directory",
            default_value=str(workspace_path("datasets", "real_traversability")),
        ),
        DeclareLaunchArgument("session_name", default_value=""),
        DeclareLaunchArgument("sample_rate_hz", default_value="20"),
        DeclareLaunchArgument("body_odom_topic", default_value="/lio/odom"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odom"),
        Node(
            package="roka_nav2_bringup",
            executable="real_traversability_recorder",
            name="real_traversability_recorder",
            parameters=[{
                "profile_file": LaunchConfiguration("profile_file"),
                "output_directory": LaunchConfiguration("output_directory"),
                "session_name": LaunchConfiguration("session_name"),
                "odom_topic": LaunchConfiguration("body_odom_topic"),
                "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                "sample_rate_hz": ParameterValue(
                    LaunchConfiguration("sample_rate_hz"), value_type=float
                ),
                "auto_record": True,
            }],
            output="screen",
            arguments=[
                "--ros-args", "--log-level", LaunchConfiguration("log_level")
            ],
        ),
    ])
