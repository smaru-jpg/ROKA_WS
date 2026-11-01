#!/usr/bin/env python3
"""Recover to and hold the selected section pose without camera or YOLO."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("roka_arm"))
    hold = Node(
        package="roka_arm",
        executable="roka_arm_hold",
        name="roka_arm_hold",
        output="screen",
        emulate_tty=True,
        parameters=[
            str(share / "config" / "arm_hold.yaml"),
            {
                "start_pose_name": LaunchConfiguration("start_pose_name"),
                "hold_scope": LaunchConfiguration("hold_scope"),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("start_pose_name", default_value="section2"),
        DeclareLaunchArgument("hold_scope", default_value="all"),
        hold,
        RegisterEventHandler(OnProcessExit(
            target_action=hold,
            on_exit=[EmitEvent(event=Shutdown(
                reason="arm transport hold stopped"
            ))],
        )),
    ])
