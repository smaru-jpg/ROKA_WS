#!/usr/bin/env python3
"""Launch the PK-3A track in capstone's native C++ MuJoCo runtime."""

import logging
import os

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from roka_mujoco.runtime_environment import ensure_mujoco_runtime
from roka_mujoco.track_description import build_track_description


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError(f"Expected true or false, got {value!r}")
    return normalized == "true"


def _shutdown_after_simulator_exit(context):
    if context.is_shutdown:
        return []
    return [Shutdown(reason="PK-3A MuJoCo simulator exited")]


def _configure_launch_logging(context):
    level = LaunchConfiguration("log_level").perform(context).upper()
    launch.logging.launch_config.level = getattr(logging, level, logging.WARNING)
    return []


def _launch_setup(context):
    ensure_mujoco_runtime()
    package_share = get_package_share_directory("roka_mujoco")
    with_robot = _as_bool(LaunchConfiguration("with_robot").perform(context))
    source_urdf = os.path.join(
        package_share,
        "description",
        "roka_robot.urdf" if with_robot else "pk3a_track.urdf",
    )

    scene = LaunchConfiguration("scene_file").perform(context).strip()
    if not scene:
        scene = os.path.join(
            package_share,
            "description",
            "pk3a",
            "models",
            "pk3a_robot_scene.xml" if with_robot else "pk3a_closed_loop.xml",
        )
    if not os.path.isfile(scene):
        raise FileNotFoundError(f"MuJoCo scene file not found: {scene}")

    controllers = LaunchConfiguration("controllers_file").perform(
        context
    ).strip()
    if not controllers:
        controllers = os.path.join(
            package_share,
            "config",
            "roka_robot_controllers.yaml"
            if with_robot
            else "pk3a_track_controllers.yaml",
        )
    if not os.path.isfile(controllers):
        raise FileNotFoundError(
            f"ros2_control parameter file not found: {controllers}"
        )
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    show_ui = _as_bool(LaunchConfiguration("show_ui").perform(context))
    show_right_ui = _as_bool(
        LaunchConfiguration("show_right_ui").perform(context)
    )
    show_profiler = _as_bool(
        LaunchConfiguration("show_profiler").perform(context)
    )
    show_sensor = _as_bool(
        LaunchConfiguration("show_sensor").perform(context)
    )
    render_vsync = _as_bool(
        LaunchConfiguration("render_vsync").perform(context)
    )
    render_fps = float(LaunchConfiguration("render_fps").perform(context))
    camera_publish_rate = float(
        LaunchConfiguration("camera_publish_rate").perform(context)
    )
    sim_speed_factor = float(
        LaunchConfiguration("sim_speed_factor").perform(context)
    )
    if camera_publish_rate <= 0.0:
        raise ValueError("camera_publish_rate must be positive")
    window_width = int(LaunchConfiguration("window_width").perform(context))
    window_height = int(LaunchConfiguration("window_height").perform(context))
    ui_font_scale = int(
        LaunchConfiguration("ui_font_scale").perform(context)
    )
    render_device = LaunchConfiguration("render_device").perform(
        context
    ).strip().lower()
    if render_device not in {"default", "nvidia"}:
        raise ValueError("render_device must be 'default' or 'nvidia'")
    render_environment = {}
    if render_device == "nvidia":
        render_environment = {
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        }

    robot_description = {
        "robot_description": build_track_description(
            source_urdf,
            scene,
            sim_speed_factor=sim_speed_factor,
            headless=headless,
            render_fps=render_fps,
            render_vsync=render_vsync,
            window_width=window_width,
            window_height=window_height,
            ui_font_scale=ui_font_scale,
            camera_publish_rate=camera_publish_rate,
        )
    }
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[
                "--ros-args", "--log-level", LaunchConfiguration("log_level")
            ],
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="screen",
            arguments=[
                "--ros-args", "--log-level", LaunchConfiguration("log_level")
            ],
            additional_env=render_environment,
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers),
            ],
            remappings=[("~/robot_description", "/robot_description")],
            on_exit=OpaqueFunction(function=_shutdown_after_simulator_exit),
        ),
    ]
    if not headless:
        ui_arguments = [
            "--width",
            str(window_width),
            "--height",
            str(window_height),
        ]
        if show_ui:
            ui_arguments.append("--left")
            if show_right_ui:
                ui_arguments.extend(["--right", "--joint"])
            if show_profiler:
                ui_arguments.append("--profiler")
            if show_sensor:
                ui_arguments.append("--sensor")
        actions.append(
            Node(
                package="roka_mujoco",
                executable="mujoco_ui_setup",
                arguments=ui_arguments + [
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="ROS console level: warn for operation, info for diagnostics.",
        ),
        OpaqueFunction(function=_configure_launch_logging),
        DeclareLaunchArgument(
            "scene_file",
            default_value="",
            description="Absolute MJCF path; empty selects the packaged scene.",
        ),
        DeclareLaunchArgument(
            "with_robot",
            default_value="false",
            description="Load the delivered ROKA robot on the PK-3A start section.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description=(
                "Absolute ros2_control parameter file; empty selects the "
                "packaged track or robot defaults."
            ),
        ),
        DeclareLaunchArgument(
            "sim_speed_factor",
            default_value="1.0",
            description="MuJoCo simulation speed relative to wall clock.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run the native C++ MuJoCo backend without a window.",
        ),
        DeclareLaunchArgument(
            "show_ui",
            default_value="true",
            description="Open the native MuJoCo left control panel.",
        ),
        DeclareLaunchArgument(
            "show_right_ui",
            default_value="false",
            description="Also open the native right joint panel.",
        ),
        DeclareLaunchArgument(
            "show_profiler",
            default_value="false",
            description="Open MuJoCo's F3 solver profiler overlay.",
        ),
        DeclareLaunchArgument(
            "show_sensor",
            default_value="false",
            description="Open MuJoCo's F4 sensor plot overlay.",
        ),
        DeclareLaunchArgument(
            "window_width",
            default_value="1600",
            description="Native MuJoCo viewer width in pixels.",
        ),
        DeclareLaunchArgument(
            "window_height",
            default_value="900",
            description="Native MuJoCo viewer height in pixels.",
        ),
        DeclareLaunchArgument(
            "render_fps",
            default_value="60",
            description="Software viewer FPS cap, independent of monitor Hz.",
        ),
        DeclareLaunchArgument(
            "camera_publish_rate",
            default_value="5",
            description="ROS RGB-D camera publication rate in Hz.",
        ),
        DeclareLaunchArgument(
            "render_vsync",
            default_value="false",
            description="Synchronize buffer swaps with monitor VBlank.",
        ),
        DeclareLaunchArgument(
            "ui_font_scale",
            default_value="100",
            description="Native UI scale: 100 to 300 in 50 percent steps.",
        ),
        DeclareLaunchArgument(
            "render_device",
            default_value="nvidia",
            description=(
                "OpenGL renderer: nvidia enables PRIME offload; "
                "default uses the current X11 renderer."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
