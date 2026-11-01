"""Build a 2D occupancy map and aligned route from one raw acquisition bag."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _actions(context):
    source = Path(LaunchConfiguration("source_directory").perform(context)).resolve()
    output = Path(LaunchConfiguration("output_directory").perform(context)).resolve()
    bag = source / "sensors"
    if not (bag / "metadata.yaml").is_file():
        return [EmitEvent(event=Shutdown(reason=f"raw sensor bag is missing: {bag}"))]
    output.mkdir(parents=True, exist_ok=True)

    nav_share = Path(get_package_share_directory("roka_nav2_bringup"))
    terrain_share = Path(get_package_share_directory("roka_terrain_mapping"))
    velodyne_share = Path(get_package_share_directory("velodyne_pointcloud"))
    params_file = velodyne_share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    transform_params = yaml.safe_load(params_file.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    transform_params["calibration"] = str(velodyne_share / "params" / "VLP16db.yaml")
    transform_params["use_sim_time"] = True

    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag),
            "--rate", LaunchConfiguration("playback_rate"),
            "--clock", "100", "--topics",
            "/velodyne_packets", "/tf", "/tf_static", "/odom", "/wheel/odom",
            "--remap", "/tf:=/replay/raw_tf",
        ],
        output="screen",
    )
    finalizer = ExecuteProcess(
        cmd=[
            "ros2", "run", "roka_nav2_bringup", "offline_2d_finalize",
            str(output),
        ],
        output="screen",
    )

    return [
        Node(
            package="roka_nav2_bringup",
            executable="tf_replay_filter",
            name="tf_replay_filter",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="velodyne_pointcloud",
            executable="velodyne_transform_node",
            name="velodyne_transform_node",
            parameters=[transform_params],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="stabilized_pointcloud_to_laserscan",
            parameters=[
                str(terrain_share / "config" / "pointcloud_to_scan.yaml"),
                {"use_sim_time": True},
            ],
            remappings=[("cloud_in", "/velodyne_points"), ("scan", "/scan_stabilized")],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[
                str(nav_share / "config" / "lidar_md_slam.yaml"),
                {"use_sim_time": True},
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_mapper_node",
            name="terrain_mapper",
            parameters=[
                str(terrain_share / "config" / "terrain_mapping.yaml"),
                {"use_sim_time": True, "map_bundle_directory": str(output)},
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="track_path_recorder",
            name="track_path_recorder",
            parameters=[{
                "use_sim_time": True,
                "output_file": str(output / "track_profile.yaml"),
                "map_frame": "map",
                "sample_frame": "map",
                "base_frame": "base_footprint",
                "auto_start": True,
                "environment": "flat",
                "track_width_m": 1.20,
            }],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        TimerAction(period=2.0, actions=[playback]),
        RegisterEventHandler(OnProcessExit(
            target_action=playback,
            on_exit=[TimerAction(period=2.0, actions=[finalizer])],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=finalizer,
            on_exit=[EmitEvent(event=Shutdown(reason="offline 2D processing complete"))],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source_directory"),
        DeclareLaunchArgument("output_directory"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        OpaqueFunction(function=_actions),
    ])
