"""Review a saved 2D map in RViz without modifying the source map."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _actions(context):
    bundle = Path(LaunchConfiguration("map_directory").perform(context)).expanduser().resolve()
    return [
        Node(
            package="roka_nav2_bringup",
            executable="occupancy_cleanup_editor",
            output="screen",
            parameters=[{
                "input_yaml": str(bundle / "occupancy.yaml"),
                "output_yaml": str(bundle / "occupancy_cleaned.yaml"),
                "audit_file": str(bundle / "occupancy_cleanup.yaml"),
                "profile_file": str(bundle / "track_profile.yaml"),
                "clear_radius_m": float(LaunchConfiguration("clear_radius_m").perform(context)),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", str(Path(__file__).parents[1] / "rviz" / "occupancy_cleanup.rviz")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_directory"),
        DeclareLaunchArgument("clear_radius_m", default_value="0.20"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_actions),
    ])
