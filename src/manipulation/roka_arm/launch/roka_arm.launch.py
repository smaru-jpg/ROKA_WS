#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _realsense_group(context, realsense_share):
    if not _enabled(LaunchConfiguration("start_realsense").perform(context)):
        return []
    camera_serial = LaunchConfiguration("camera_serial").perform(context).strip()
    if camera_serial and not camera_serial.replace("_", "").replace("-", "").isalnum():
        raise RuntimeError(f"invalid RealSense serial: {camera_serial!r}")
    normalized_serial = camera_serial.lstrip("_")
    realsense_serial = "''" if not normalized_serial else f"'_{normalized_serial}'"
    return [
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(realsense_share / "launch" / "rs_launch.py")
                    ),
                    launch_arguments={
                        "camera_namespace": "roka",
                        "camera_name": "arm_camera",
                        "serial_no": realsense_serial,
                        "device_type": "d435",
                        "enable_color": "true",
                        "enable_depth": "true",
                        "enable_infra": "false",
                        "enable_infra1": "false",
                        "enable_infra2": "false",
                        # Section 2 needs RGB-D for box distance and grasping.
                        # Both streams use the D435I's supported 30 FPS profile.
                        "rgb_camera.color_profile": "640,480,30",
                        "depth_module.depth_profile": "640,480,30",
                        "enable_sync": "true",
                        "align_depth.enable": "true",
                        "pointcloud.enable": "false",
                        "enable_motion": "false",
                        "publish_tf": "true",
                        "wait_for_device_timeout": "5.0",
                        "reconnect_timeout": "5.0",
                        "log_level": "warn",
                    }.items(),
                )
            ],
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("roka_arm"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))

    connect_hardware = LaunchConfiguration("connect_hardware")
    enable_motion = LaunchConfiguration("enable_motion")
    visualize = LaunchConfiguration("visualize")
    inference_device = LaunchConfiguration("inference_device")

    arm_node = Node(
        package="roka_arm",
        executable="roka_arm_controller",
        name="roka_arm_controller",
        output="screen",
        parameters=[
            str(package_share / "config" / "roka_arm.yaml"),
            {
                "model_path": str(
                    package_share / "models" / "box_marker_best.pt"
                ),
                "connect_hardware": connect_hardware,
                "enable_motion": enable_motion,
                "visualize": visualize,
                "start_pose_name": LaunchConfiguration("start_pose_name"),
                "perception_enabled": ParameterValue(
                    LaunchConfiguration("perception_enabled"), value_type=bool
                ),
                "manual_joy_enabled": ParameterValue(
                    LaunchConfiguration("manual_joy_enabled"), value_type=bool
                ),
                # CUDA indexes such as "0" must remain strings; an untyped
                # LaunchConfiguration is otherwise inferred as an integer.
                "inference_device": ParameterValue(
                    inference_device, value_type=str
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("connect_hardware", default_value="false"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("start_pose_name", default_value="section2_start"),
        DeclareLaunchArgument("perception_enabled", default_value="true"),
        DeclareLaunchArgument("manual_joy_enabled", default_value="false"),
        OpaqueFunction(
            function=_realsense_group,
            kwargs={"realsense_share": realsense_share},
        ),
        arm_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="ROKA arm controller exited")
                    )
                ],
            )
        ),
    ])
