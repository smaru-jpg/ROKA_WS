#!/usr/bin/env python3
"""View real 2D occupancy with a low-confidence MuJoCo-derived 3D prior."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    output = Path(
        LaunchConfiguration("output_directory").perform(context)
    ).expanduser().resolve()
    occupancy = Path(
        LaunchConfiguration("occupancy_yaml").perform(context)
    ).expanduser().resolve()
    required = [
        output / "hybrid_map.npz",
        output / "track_profile_sim_assisted.yaml",
        output / "manifest.yaml",
        occupancy,
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        return [EmitEvent(event=Shutdown(
            reason="simulation-assisted view is missing: " + ", ".join(missing)
        ))]
    log_level = LaunchConfiguration("log_level")
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="sim_assisted_occupancy_server",
            output="screen",
            parameters=[{"yaml_filename": str(occupancy)}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="sim_assisted_map_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["sim_assisted_occupancy_server"],
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="roka_mujoco",
            executable="sim_assisted_map_visualizer",
            name="sim_assisted_map_visualizer",
            output="screen",
            parameters=[{"output_directory": str(output)}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_directory"),
        DeclareLaunchArgument("occupancy_yaml"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_nodes),
        Node(
            package="rviz2",
            executable="rviz2",
            name="sim_assisted_map_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("roka_mujoco"),
                    "description",
                    "pk3a",
                    "rviz",
                    "sim_assisted_map.rviz",
                ]),
                "--ros-args",
                "--log-level",
                "fatal",
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
