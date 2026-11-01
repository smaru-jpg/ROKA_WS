#!/usr/bin/env python3
"""Review a saved (or staging) real FAST-LIO/SC-PGO track bundle in RViz."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    directory = Path(
        LaunchConfiguration("bundle_directory").perform(context)
    ).expanduser().resolve()
    pcd = directory / "map.pcd"
    profile = directory / "track_profile.yaml"
    occupancy = directory / "occupancy.yaml"
    missing = [str(path) for path in (pcd, profile, occupancy) if not path.is_file()]
    if missing:
        return [EmitEvent(event=Shutdown(
            reason="real track view is missing: " + ", ".join(missing)
        ))]
    log_level = LaunchConfiguration("log_level")
    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="track_review_occupancy_server",
            output="screen",
            parameters=[{"yaml_filename": str(occupancy)}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="track_review_map_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["track_review_occupancy_server"],
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="track_mapping_visualizer",
            name="track_mapping_visualizer",
            output="screen",
            parameters=[{"bundle_directory": str(directory)}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]
    # Prefer the offline full-map terrain product.  The live terrain_map.bag is
    # only the final rolling local window and therefore cannot represent the
    # whole recorded route in RViz.
    terrain_bag = directory / "terrain_global.bag"
    if not terrain_bag.is_dir():
        terrain_bag = directory / "terrain_map.bag"
    if terrain_bag.is_dir():
        actions.extend([
            Node(
                package="roka_terrain_mapping",
                executable="terrain_map_server_node",
                name="track_review_terrain_map_server",
                output="screen",
                parameters=[{
                    "bag_path": str(terrain_bag),
                    "bag_topic": "/terrain/static_map",
                    "output_topic": "/terrain/static_map",
                    "zero_timestamp": True,
                }],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="roka_terrain_mapping",
                executable="terrain_review_visualizer_node",
                name="track_review_terrain_visualizer",
                output="screen",
                parameters=[{
                    "input_topic": "/terrain/static_map",
                    "output_topic": "/terrain/visual/classes",
                    "audit_file": str(directory / "terrain_review_annotations.yaml"),
                    "enable_click_review": True,
                }],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bundle_directory"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_nodes),
        Node(
            package="rviz2",
            executable="rviz2",
            name="track_map_review_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("roka_nav2_bringup"),
                    "rviz",
                    "track_mapping_3d.rviz",
                ]),
                "--ros-args",
                "--log-level",
                "fatal",
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
