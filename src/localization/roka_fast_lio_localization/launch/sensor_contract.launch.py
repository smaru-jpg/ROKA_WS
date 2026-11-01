from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("duration_sec", default_value="8.0"),
        DeclareLaunchArgument("output", default_value=""),
        Node(
            package="roka_fast_lio_localization",
            executable="live_sensor_contract",
            name="roka_sensor_contract",
            output="screen",
            parameters=[{
                "duration_sec": ParameterValue(
                    LaunchConfiguration("duration_sec"), value_type=float
                ),
                "output": LaunchConfiguration("output"),
            }],
        ),
    ])
