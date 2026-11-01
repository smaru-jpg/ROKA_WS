"""Run the fixed-PCD relocalizer against a SC-PGO optimized map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_pcd", default_value=""),
        DeclareLaunchArgument("initial_pose_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        Node(
            package="roka_map_localization",
            executable="fixed_map_localizer",
            name="fixed_map_localizer",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("roka_map_localization"),
                    "config", "roka_fixed_map.yaml",
                ]),
                {
                    "map_pcd": LaunchConfiguration("map_pcd"),
                    "initial_pose_file": LaunchConfiguration("initial_pose_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_tf": LaunchConfiguration("publish_tf"),
                },
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
