from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("roka_terrain_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("terrain_type", default_value="uphill"),
        DeclareLaunchArgument("slope_deg", default_value="15.0"),
        DeclareLaunchArgument(
            "map_bundle_directory", default_value="/tmp/roka_terrain_bundle_test"
        ),
        Node(
            package="roka_terrain_mapping",
            executable="synthetic_terrain_node",
            name="synthetic_terrain",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[{
                "terrain_type": LaunchConfiguration("terrain_type"),
                "slope_deg": LaunchConfiguration("slope_deg"),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, "launch", "terrain_mapping.launch.py"])
            ),
            launch_arguments={
                "map_bundle_directory": LaunchConfiguration("map_bundle_directory"),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
    ])
