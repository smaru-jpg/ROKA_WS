#!/usr/bin/env python3
"""Run a real mapping-session path with the MuJoCo vehicle on level ground."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    share = get_package_share_directory("roka_mujoco")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "pk3a_track.launch.py")),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "with_robot": "true",
            "controllers_file": os.path.join(
                share, "config", "mujoco_safe_controllers.yaml"
            ),
            "scene_file": os.path.join(
                share,
                "description",
                "pk3a",
                "models",
                "pk3a_mapped_flat_scene.xml",
            ),
            "headless": LaunchConfiguration("headless"),
            "show_ui": LaunchConfiguration("show_ui"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "render_fps": LaunchConfiguration("render_fps"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "camera_publish_rate": "5",
        }.items(),
    )
    spawner = Node(
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
    mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace="sim",
        name="twist_mux",
        parameters=[
            os.path.join(share, "config", "mujoco_twist_mux.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("cmd_vel_out", "/sim_diff_drive_controller/cmd_vel_unstamped")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )
    controller = Node(
        package="roka_mujoco",
        executable="sim_mapped_profile_controller",
        name="sim_mapped_profile_controller",
        parameters=[
            os.path.join(share, "config", "pk3a_mapped_autonomy.yaml"),
            {
                "profile_file": LaunchConfiguration("profile_file"),
                "auto_start": ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool
                ),
                "speed_scale": ParameterValue(
                    LaunchConfiguration("speed_scale"), value_type=float
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )
    sensor_adapter = Node(
        package="roka_mujoco",
        executable="sim_sensor_adapter",
        name="sim_sensor_adapter",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("show_ui", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="1600"),
        DeclareLaunchArgument("window_height", default_value="900"),
        DeclareLaunchArgument("render_fps", default_value="30"),
        DeclareLaunchArgument("sim_speed_factor", default_value="1.0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "profile_file",
            default_value=str(workspace_path(
                "maps", "section_1", "track_profile_test.yaml"
            )),
        ),
        simulator,
        spawner,
        mux,
        sensor_adapter,
        controller,
    ])
