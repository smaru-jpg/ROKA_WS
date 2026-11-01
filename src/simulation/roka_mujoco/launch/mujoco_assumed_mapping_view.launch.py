#!/usr/bin/env python3
"""Review the manual-demonstration-based assumed MuJoCo mapping bundle."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from roka_robot_description.workspace import workspace_root


def generate_launch_description():
    default_bundle = workspace_root() / "maps" / "sim_pk3a_assumed_mapping"
    return LaunchDescription([
        DeclareLaunchArgument(
            "bundle_directory", default_value=str(default_bundle)
        ),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        LogInfo(msg=(
            "MuJoCo assumed mapping: geometry from the virtual track, route from "
            "the successful Xbox manual lap; never authorized for field motion"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"),
                "launch", "track_map_view.launch.py",
            ])),
            launch_arguments={
                "bundle_directory": LaunchConfiguration("bundle_directory"),
                "use_rviz": "false",
                "log_level": "warn",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mujoco_assumed_mapping_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("roka_mujoco"), "description", "pk3a",
                    "rviz", "mujoco_assumed_mapping.rviz",
                ]),
                "--ros-args", "--log-level", "fatal",
            ],
            condition=IfCondition(LaunchConfiguration("show_rviz")),
        ),
    ])
