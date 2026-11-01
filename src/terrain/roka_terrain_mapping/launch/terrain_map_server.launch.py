from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("bag_path"),
        DeclareLaunchArgument("bag_topic", default_value="/terrain/static_map"),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_map_server_node",
            name="terrain_map_server",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[{
                "bag_path": LaunchConfiguration("bag_path"),
                "bag_topic": LaunchConfiguration("bag_topic"),
                "output_topic": "/terrain/static_map",
            }],
        ),
    ])
