from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_share = FindPackageShare("roka_terrain_mapping")
    supervisor_share = FindPackageShare("roka_terrain_supervisor")
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("terrain_type", default_value="uphill"),
        DeclareLaunchArgument("slope_deg", default_value="15.0"),
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
                PathJoinSubstitution([mapping_share, "launch", "terrain_mapping.launch.py"])
            ),
            launch_arguments={"log_level": LaunchConfiguration("log_level")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([supervisor_share, "launch", "safety_pipeline.launch.py"])
            ),
            launch_arguments={
                "use_collision_monitor": "false",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
    ])
