from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("folder_path"),
        DeclareLaunchArgument("pcd_filename"),
        DeclareLaunchArgument("output_grid_map", default_value="terrain_global.bag"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        Node(
            package="grid_map_pcl",
            executable="grid_map_pcl_loader_node",
            name="global_pcd_to_grid_map",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                {
                    "folder_path": LaunchConfiguration("folder_path"),
                    "pcd_filename": LaunchConfiguration("pcd_filename"),
                    "map_rosbag_topic": "/terrain/static_map",
                    "output_grid_map": LaunchConfiguration("output_grid_map"),
                    "map_frame": LaunchConfiguration("map_frame"),
                    "map_layer_name": "elevation",
                    "prefix": "",
                },
            ],
        ),
    ])
