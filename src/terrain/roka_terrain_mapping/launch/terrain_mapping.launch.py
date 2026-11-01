from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from roka_robot_description.workspace import workspace_path


def generate_launch_description():
    share = FindPackageShare("roka_terrain_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument(
            "terrain_config",
            default_value=PathJoinSubstitution([share, "config", "terrain_mapping.yaml"]),
        ),
        DeclareLaunchArgument("use_obstacle_filter", default_value="true"),
        DeclareLaunchArgument("controlled_level_test", default_value="false"),
        DeclareLaunchArgument(
            "obstacle_filter_config",
            default_value=PathJoinSubstitution([share, "config", "terrain_obstacle_filter.yaml"]),
        ),
        DeclareLaunchArgument(
            "map_bundle_directory",
            default_value=str(workspace_path("maps", "track_v1")),
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_mapper_node",
            name="terrain_mapper",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[LaunchConfiguration("terrain_config"), {
                "map_bundle_directory": LaunchConfiguration("map_bundle_directory"),
            }],
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_obstacle_filter_node",
            name="terrain_obstacle_filter",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[LaunchConfiguration("obstacle_filter_config"), {
                "unknown_is_obstacle": ParameterValue(
                    LaunchConfiguration("controlled_level_test"), value_type=bool
                ),
                "unknown_level_ground_z_m": 0.0,
            }],
            condition=IfCondition(LaunchConfiguration("use_obstacle_filter")),
        ),
    ])
