"""Shared launch construction for all mission-vision profiles."""

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def realsense_group(context, realsense_share):
    if not _enabled(LaunchConfiguration("start_realsense").perform(context)):
        return []
    serial = LaunchConfiguration("camera_serial").perform(context).strip()
    if serial and not serial.replace("_", "").replace("-", "").isalnum():
        raise RuntimeError(f"invalid RealSense serial: {serial!r}")
    normalized = serial.lstrip("_")
    serial_argument = "''" if not normalized else f"'_{normalized}'"
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
                        "serial_no": serial_argument,
                        "device_type": "d435",
                        "enable_color": "true",
                        # IFF, marker and traffic models only consume color.
                        # Keeping depth/alignment off avoids host-side work and
                        # lets the D435I sustain its requested color rate.
                        "enable_depth": "false",
                        "enable_infra": "false",
                        "enable_infra1": "false",
                        "enable_infra2": "false",
                        "rgb_camera.color_profile": "640,480,30",
                        "enable_sync": "false",
                        "align_depth.enable": "false",
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


def mission_launch_description(config_name, include_led=False):
    package_share = Path(get_package_share_directory("mission_vision"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    config_path = str(package_share / "config" / config_name)
    vision_node = Node(
        package="mission_vision",
        executable="vision_node",
        name="mission_vision",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": config_path,
            "image_topic": "/roka/arm_camera/color/image_raw",
            "inference_device": ParameterValue(
                LaunchConfiguration("inference_device"), value_type=str
            ),
            "display_enabled": ParameterValue(
                LaunchConfiguration("display_enabled"), value_type=bool
            ),
            "enabled": ParameterValue(
                LaunchConfiguration("enabled"), value_type=bool
            ),
        }],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    actions = [
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("display_enabled", default_value="true"),
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument("enable_led", default_value="false"),
        DeclareLaunchArgument("led_serial_port", default_value=""),
        OpaqueFunction(
            function=realsense_group,
            kwargs={"realsense_share": realsense_share},
        ),
        vision_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=vision_node,
                on_exit=[EmitEvent(event=Shutdown(reason="mission vision stopped"))],
            )
        ),
    ]
    if include_led:
        led_node = Node(
            package="mission_vision",
            executable="neopixel_controller",
            name="mission_neopixel",
            output="screen",
            parameters=[{
                "config_file": config_path,
                "enabled": True,
                "serial_port": LaunchConfiguration("led_serial_port"),
            }],
            condition=IfCondition(LaunchConfiguration("enable_led")),
            arguments=["--ros-args", "--log-level", "warn"],
        )
        actions.extend([
            led_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=led_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="mission LED stopped"))],
                )
            ),
        ])
    return LaunchDescription(actions)
