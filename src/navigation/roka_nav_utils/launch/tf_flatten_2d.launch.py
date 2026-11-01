from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("roka_nav_utils")) / "config" / "tf_flatten_2d.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="warn"),
            Node(
                package="roka_nav_utils",
                executable="tf_flatten_2d",
                name="tf_flatten_2d",
                parameters=[str(config)],
                output="screen",
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
            )
        ]
    )
