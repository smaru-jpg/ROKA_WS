#!/usr/bin/env python3
"""RealSense color stream and operator view without YOLO inference."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mission_vision.launch_support import realsense_group


def generate_launch_description():
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    viewer = Node(
        package="mission_vision",
        executable="camera_view_node",
        name="section_camera_view",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "section": LaunchConfiguration("section"),
            "display_enabled": ParameterValue(
                LaunchConfiguration("display_enabled"), value_type=bool
            ),
            "image_topic": "/roka/arm_camera/color/image_raw",
        }],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    return LaunchDescription([
        DeclareLaunchArgument("section", default_value="section4"),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("display_enabled", default_value="true"),
        OpaqueFunction(
            function=realsense_group,
            kwargs={"realsense_share": realsense_share},
        ),
        viewer,
        RegisterEventHandler(OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason="manual camera view stopped"))],
        )),
    ])
