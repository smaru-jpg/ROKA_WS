#!/usr/bin/env python3
"""Section 3 recorded-route drive with marker observation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_id", default_value="section_3"),
        DeclareLaunchArgument("map_root", default_value=str(workspace_path("maps"))),
        DeclareLaunchArgument("competition_track_mode", default_value="false"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_navigation", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("display_enabled", default_value="true"),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("hold_arm", default_value="true"),
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
                "hold_arm": LaunchConfiguration("hold_arm"),
                "autonomy_auto_start": "true",
                "autonomy_optional_xbox": "true",
                "joy_device_id": "auto",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_navigation")),
        ),
        # Vision-only validation still holds the section pose. During route
        # driving pk3a_field owns the same hold node and its command interlock.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"), "launch", "arm_hold.launch.py"
            ])),
            launch_arguments={"start_pose_name": "section3"}.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("hold_arm"), "' == 'true' and '",
                LaunchConfiguration("start_navigation"), "' != 'true'",
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("mission_vision"), "launch", "autumn.launch.py"
            ])),
            launch_arguments={
                "start_realsense": LaunchConfiguration("start_realsense"),
                "camera_serial": LaunchConfiguration("camera_serial"),
                "inference_device": LaunchConfiguration("inference_device"),
                "display_enabled": LaunchConfiguration("display_enabled"),
            }.items(),
        ),
    ])
