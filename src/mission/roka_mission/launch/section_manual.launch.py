#!/usr/bin/env python3
"""One-command section manual drive with full-arm pose and camera display."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
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


def _normalize_section(value):
    section = str(value).strip().lower().replace("-", "_")
    if section.startswith("section_"):
        section = "section" + section[len("section_"):]
    if section not in {f"section{number}" for number in range(1, 6)}:
        raise RuntimeError("section must be section1..section5")
    return section


def _include(package, launch_name, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / "launch" / launch_name)
        ),
        launch_arguments=arguments.items(),
    )


def _section_actions(context):
    section = _normalize_section(LaunchConfiguration("section").perform(context))
    section_number = int(section[-1])
    start_pose = "section2_start" if section_number == 2 else section
    start_realsense = _enabled(
        LaunchConfiguration("start_realsense").perform(context)
    )
    camera_enabled = "true" if start_realsense else "false"
    camera_serial = LaunchConfiguration("camera_serial")
    inference_device = LaunchConfiguration("inference_device")
    display_enabled = LaunchConfiguration("display_enabled")
    enable_arm_motion = LaunchConfiguration("enable_arm_motion")
    start_base = _enabled(LaunchConfiguration("start_base").perform(context))
    manual_lock_topic = (
        "/roka/mission/box_pick/base_lock" if section_number == 2 else ""
    )

    actions = [
        # The full arm controller is the only owner of both Dynamixel buses in
        # section-manual mode.  It moves every PRO/wrist/gripper axis to the
        # selected section pose before publishing hold_ready.
        _include("roka_arm", "roka_arm.launch.py", {
            "start_realsense": camera_enabled if section_number == 2 else "false",
            "connect_hardware": LaunchConfiguration("connect_arm_hardware"),
            "enable_motion": enable_arm_motion,
            "visualize": display_enabled,
            "camera_serial": camera_serial,
            "inference_device": inference_device,
            "start_pose_name": start_pose,
            "perception_enabled": "true" if section_number == 2 else "false",
            "manual_joy_enabled": enable_arm_motion,
        }),
    ]
    if start_base:
        actions.append(_include("roka_md_driver", "mdh250_md200t_xbox.launch.py", {
                "simulation_mode": LaunchConfiguration("simulation_mode"),
                "allow_motion": LaunchConfiguration("md_allow_motion"),
                "joy_device_id": LaunchConfiguration("joy_device_id"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "teleop_cmd_vel_topic": LaunchConfiguration("pending_cmd_vel_topic"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_wheel_rpm": LaunchConfiguration("max_wheel_rpm"),
                "feedback_overspeed_limit_rpm": LaunchConfiguration(
                    "feedback_overspeed_limit_rpm"
                ),
                "log_level": LaunchConfiguration("log_level"),
            }))

    if section_number == 1:
        actions.append(_include("mission_vision", "spring.launch.py", {
            "start_realsense": camera_enabled,
            "camera_serial": camera_serial,
            "inference_device": inference_device,
            "display_enabled": display_enabled,
            "enabled": "true",
            "enable_led": "false",
        }))
    elif section_number == 2:
        # The arm node owns RGB-D and box YOLO. Traffic inference is loaded but
        # stays off until the verified pickup/place sequence has completed.
        actions.append(_include("mission_vision", "traffic.launch.py", {
            "start_realsense": "false",
            "camera_serial": camera_serial,
            "inference_device": inference_device,
            "display_enabled": display_enabled,
            "enabled": "false",
        }))
        actions.append(Node(
            package="roka_mission",
            executable="box_pick_coordinator",
            name="box_pick_coordinator",
            output="screen",
            emulate_tty=True,
            parameters=[
                str(
                    Path(get_package_share_directory("roka_mission"))
                    / "config" / "box_pick.yaml"
                ),
                {
                    "auto_start": True,
                    "operator_trigger_required": True,
                    "control_recorded_route": False,
                    "resume_route_after_grasp": False,
                    "manual_base_lock_topic": manual_lock_topic,
                },
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ))
    elif section_number == 3:
        actions.append(_include("mission_vision", "autumn.launch.py", {
            "start_realsense": camera_enabled,
            "camera_serial": camera_serial,
            "inference_device": inference_device,
            "display_enabled": display_enabled,
            "enabled": "true",
        }))
    else:
        actions.append(_include("mission_vision", "camera_view.launch.py", {
            "section": section,
            "start_realsense": camera_enabled,
            "camera_serial": camera_serial,
            "display_enabled": display_enabled,
        }))

    router = Node(
        package="roka_mission",
        executable="section_manual_joy_router",
        name="section_manual_joy_router",
        output="screen",
        emulate_tty=True,
        parameters=[{"section": section}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    if start_base:
        actions.append(Node(
            package="roka_arm",
            executable="roka_arm_hold_gate",
            name="section_manual_arm_gate",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_topic": LaunchConfiguration("pending_cmd_vel_topic"),
                "output_topic": LaunchConfiguration("cmd_vel_topic"),
                "require_field_preflight": False,
                "external_lock_topic": manual_lock_topic,
                "require_enable_release": True,
                "joy_topic": "/joy",
                "enable_button": 4,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ))
    actions.extend([
        router,
        RegisterEventHandler(OnProcessExit(
            target_action=router,
            on_exit=[EmitEvent(event=Shutdown(
                reason="section manual Xbox router stopped"
            ))],
        )),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("section", default_value="section1"),
        DeclareLaunchArgument("simulation_mode", default_value="false"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("connect_arm_hardware", default_value="true"),
        DeclareLaunchArgument("enable_arm_motion", default_value="true"),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("inference_device", default_value="auto"),
        DeclareLaunchArgument("display_enabled", default_value="true"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "pending_cmd_vel_topic", default_value="/cmd_vel_section_manual_pending"
        ),
        DeclareLaunchArgument("max_linear_speed", default_value="1.00"),
        DeclareLaunchArgument("max_wheel_rpm", default_value="112.0"),
        DeclareLaunchArgument("feedback_overspeed_limit_rpm", default_value="130.0"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_section_actions),
    ])
