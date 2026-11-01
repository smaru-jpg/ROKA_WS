#!/usr/bin/env python3
"""Section 2: recorded-route drive, box trigger, base hold, arm pickup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    mission_share = FindPackageShare("roka_mission")
    coordinator = Node(
        package="roka_mission",
        executable="box_pick_coordinator",
        name="box_pick_coordinator",
        output="screen",
        parameters=[PathJoinSubstitution([
            mission_share, "config", "box_pick.yaml"
        ])],
    )
    traffic_gate = Node(
        package="roka_mission",
        executable="traffic_signal_route_gate",
        name="traffic_signal_route_gate",
        output="screen",
        emulate_tty=True,
        parameters=[PathJoinSubstitution([
            mission_share, "config", "traffic_stop_go.yaml"
        ])],
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_id", default_value="section_2"),
        DeclareLaunchArgument("map_root", default_value=str(workspace_path("maps"))),
        DeclareLaunchArgument("competition_track_mode", default_value="false"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("connect_arm_hardware", default_value="true"),
        DeclareLaunchArgument("enable_arm_motion", default_value="true"),
        DeclareLaunchArgument("visualize_arm", default_value="true"),
        DeclareLaunchArgument("start_navigation", default_value="true"),
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
                # The full Section-2 arm controller owns both Dynamixel buses,
                # recovers to its validated start pose and publishes hold_ready.
                "hold_arm": "false",
                "autonomy_auto_start": "true",
                "joy_device_id": "auto",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_navigation")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"), "launch", "roka_arm.launch.py"
            ])),
            launch_arguments={
                "start_realsense": LaunchConfiguration("start_realsense"),
                "connect_hardware": LaunchConfiguration("connect_arm_hardware"),
                "enable_motion": LaunchConfiguration("enable_arm_motion"),
                "visualize": LaunchConfiguration("visualize_arm"),
                "camera_serial": LaunchConfiguration("camera_serial"),
                "inference_device": LaunchConfiguration("inference_device"),
                "start_pose_name": "section2_start",
            }.items(),
        ),
        # Load the traffic model once at startup, but keep inference gated off.
        # The coordinator enables it only after the box has been placed and the
        # arm has reached SECTION2_DRIVE_POSE.  The arm launch remains the sole
        # RealSense owner; traffic reuses its color topic.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("mission_vision"), "launch", "traffic.launch.py"
            ])),
            launch_arguments={
                "start_realsense": "false",
                "inference_device": LaunchConfiguration("inference_device"),
                "display_enabled": LaunchConfiguration("visualize_arm"),
                "enabled": "false",
            }.items(),
        ),
        # The model is enabled after a successful box placement.  RED then
        # cancels the recorded route and owns the high-priority zero command;
        # confirmed BLUE rejoins the same route from the current pose.
        traffic_gate,
        coordinator,
        RegisterEventHandler(OnProcessExit(
            target_action=traffic_gate,
            on_exit=[EmitEvent(event=Shutdown(reason="Section-2 traffic gate stopped"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=coordinator,
            on_exit=[EmitEvent(event=Shutdown(reason="Section-2 coordinator stopped"))],
        )),
    ])
