#!/usr/bin/env python3
"""Run the PK-3A MuJoCo robot with an isolated Xbox control path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from roka_md_driver.xbox_identity import (
    resolve_assigned_sdl_device_id,
)


VIEWER_ARGUMENT_DEFAULTS = {
    "headless": "false",
    "show_ui": "true",
    "show_right_ui": "false",
    "show_profiler": "false",
    "show_sensor": "false",
    "window_width": "1600",
    "window_height": "900",
    "render_fps": "60",
    "camera_publish_rate": "15",
    "render_vsync": "false",
    "ui_font_scale": "100",
    "render_device": "nvidia",
}


def _select_roka_xbox(context):
    device_id = resolve_assigned_sdl_device_id(
        LaunchConfiguration("joy_device_id").perform(context),
    )
    context.launch_configurations["joy_device_id"] = str(device_id)
    return []


def generate_launch_description():
    package_share = get_package_share_directory("roka_mujoco")
    track_launch = os.path.join(
        package_share, "launch", "pk3a_track.launch.py"
    )
    default_controller_config = os.path.join(
        package_share, "config", "mujoco_xbox_controllers.yaml"
    )
    default_teleop_config = os.path.join(
        package_share, "config", "mujoco_xbox_teleop.yaml"
    )
    mux_config = os.path.join(
        package_share, "config", "mujoco_twist_mux.yaml"
    )
    driver_scene = os.path.join(
        package_share,
        "description",
        "pk3a",
        "models",
        "pk3a_robot_drive_scene.xml",
    )

    declarations = [
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument(
            "joy_device_id",
            default_value="auto",
            description="Fail-closed selection of the assigned ROKA Xbox.",
        ),
        OpaqueFunction(function=_select_roka_xbox),
        DeclareLaunchArgument(
            "camera_view",
            default_value="true",
            description="Open a separate driver-camera image window.",
        ),
        DeclareLaunchArgument(
            "controller_config_file",
            default_value=default_controller_config,
            description="Diff-drive controller limits for MuJoCo Xbox driving.",
        ),
        DeclareLaunchArgument(
            "teleop_config_file",
            default_value=default_teleop_config,
            description="Xbox axes, buttons, and speed scales.",
        ),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value=default)
        for name, default in VIEWER_ARGUMENT_DEFAULTS.items()
    )

    simulator_arguments = {
        "with_robot": "true",
        "controllers_file": LaunchConfiguration("controller_config_file"),
        "scene_file": driver_scene,
        "log_level": LaunchConfiguration("log_level"),
    }
    simulator_arguments.update(
        {
            name: LaunchConfiguration(name)
            for name in VIEWER_ARGUMENT_DEFAULTS
        }
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(track_launch),
        launch_arguments=simulator_arguments.items(),
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "sim_imu_broadcaster",
            "sim_diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
            "--activate-as-group",
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output="screen",
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        namespace="sim",
        name="joy_node",
        parameters=[{
            "use_sim_time": True,
            "device_id": ParameterValue(
                LaunchConfiguration("joy_device_id"), value_type=int
            ),
            "deadzone": 0.12,
            "autorepeat_rate": 20.0,
            "coalesce_interval_ms": 1,
            "sticky_buttons": False,
        }],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )

    teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        namespace="sim",
        name="teleop_twist_joy_node",
        parameters=[
            ParameterFile(
                LaunchConfiguration("teleop_config_file"), allow_substs=True
            ),
            {"use_sim_time": True},
        ],
        remappings=[("cmd_vel", "cmd_vel_joy")],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace="sim",
        name="twist_mux",
        parameters=[mux_config, {"use_sim_time": True}],
        remappings=[
            (
                "cmd_vel_out",
                "/sim_diff_drive_controller/cmd_vel_unstamped",
            )
        ],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )

    camera_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="driver_camera_view",
        arguments=[
            "--on-top", "/driver_camera/color",
            "--ros-args", "--log-level", LaunchConfiguration("log_level"),
        ],
        condition=IfCondition(LaunchConfiguration("camera_view")),
        output="screen",
    )

    sensor_adapter = Node(
        package="roka_mujoco",
        executable="sim_sensor_adapter",
        name="sim_sensor_adapter",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
        output="screen",
    )

    return LaunchDescription(
        declarations
        + [
            simulator,
            controller_spawner,
            joy,
            teleop,
            twist_mux,
            sensor_adapter,
            camera_view,
        ]
    )
