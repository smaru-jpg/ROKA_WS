#!/usr/bin/env python3
"""Section 1 hybrid: mapped route except deterministic fog straight replay."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    blind_controller = Node(
        package="roka_mission",
        executable="blind_segment_controller",
        name="blind_segment_controller",
        output="screen",
        parameters=[{
            "profile_file": LaunchConfiguration("profile_file"),
            "route_profile_file": LaunchConfiguration("route_profile_file"),
            "localization_timeout_sec": LaunchConfiguration(
                "localization_timeout_sec"
            ),
            "maximum_exit_progress_error_ratio": LaunchConfiguration(
                "maximum_exit_progress_error_ratio"
            ),
            "maximum_exit_route_distance_m": LaunchConfiguration(
                "maximum_exit_route_distance_m"
            ),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_id", default_value="section_1"),
        DeclareLaunchArgument("map_root", default_value=str(workspace_path("maps"))),
        DeclareLaunchArgument("profile_file"),
        DeclareLaunchArgument("route_profile_file"),
        # Flat/low-hill substitution tests cannot reproduce fog returns.  Give
        # the live localizer time and geometric room to reacquire after a
        # deliberate LiDAR input interruption while keeping the exit bounded.
        DeclareLaunchArgument("localization_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument(
            "maximum_exit_progress_error_ratio", default_value="0.10"
        ),
        DeclareLaunchArgument(
            "maximum_exit_route_distance_m", default_value="0.75"
        ),
        DeclareLaunchArgument("competition_track_mode", default_value="false"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("display_enabled", default_value="true"),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("enable_led", default_value="false"),
        DeclareLaunchArgument("led_serial_port", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"), "launch", "pk3a_field.launch.py"
            ])),
            launch_arguments={
                "mode": "autonomy",
                "map_id": LaunchConfiguration("map_id"),
                "map_root": LaunchConfiguration("map_root"),
                "recorded_route_mode": "true",
                "competition_track_mode": LaunchConfiguration(
                    "competition_track_mode"
                ),
                "localization_backend": "auto",
                "md_allow_motion": LaunchConfiguration("md_allow_motion"),
                # The fog controller owns the first motion.  Nav2 must remain
                # stopped until the live LiDAR pose is accepted at the exit.
                "autonomy_auto_start": "false",
                "autonomy_external_start_control": "true",
                "joy_device_id": "auto",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        blind_controller,
        RegisterEventHandler(OnProcessExit(
            target_action=blind_controller,
            on_exit=[EmitEvent(event=Shutdown(reason="Section-1 controller stopped"))],
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("mission_vision"), "launch", "spring.launch.py"
            ])),
            launch_arguments={
                "start_realsense": LaunchConfiguration("start_realsense"),
                "camera_serial": LaunchConfiguration("camera_serial"),
                "inference_device": LaunchConfiguration("inference_device"),
                "display_enabled": LaunchConfiguration("display_enabled"),
                "enable_led": LaunchConfiguration("enable_led"),
                "led_serial_port": LaunchConfiguration("led_serial_port"),
            }.items(),
        ),
    ])
