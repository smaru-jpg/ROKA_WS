#!/usr/bin/env python3
"""Launch the PK-3A scene with the delivered ROKA robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("roka_mujoco")
    common_launch = os.path.join(
        package_share, "launch", "pk3a_track.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common_launch),
            launch_arguments={
                "with_robot": "true",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        )
    ])
