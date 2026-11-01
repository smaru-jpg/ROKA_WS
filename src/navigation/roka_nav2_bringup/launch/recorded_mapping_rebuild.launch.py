"""Rebuild a growing 2D/2.5D map from a recorded mapping sensor bag."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _actions(context):
    bundle = Path(
        LaunchConfiguration("map_directory").perform(context)
    ).expanduser().resolve()
    required = [
        bundle / "track_profile.yaml",
        bundle / "sensors" / "metadata.yaml",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        return [EmitEvent(event=Shutdown(
            reason="recorded mapping rebuild is missing: " + ", ".join(missing)
        ))]

    velodyne_share = Path(get_package_share_directory("velodyne_pointcloud"))
    parameter_file = velodyne_share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    transform_parameters = yaml.safe_load(parameter_file.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    transform_parameters["calibration"] = str(velodyne_share / "params" / "VLP16db.yaml")
    transform_parameters["use_sim_time"] = True

    nav_share = Path(get_package_share_directory("roka_nav2_bringup"))
    terrain_share = Path(get_package_share_directory("roka_terrain_mapping"))
    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bundle / "sensors"),
            "--rate", LaunchConfiguration("playback_rate"),
            "--clock", "100", "--topics",
            "/velodyne_packets", "/tf", "/tf_static", "/odom",
            "/track/recorded_path", "--remap",
            "/tf:=/replay/raw_tf",
            "/track/recorded_path:=/replay/growing_path",
        ],
        output="screen",
    )

    return [
        Node(
            package="roka_nav2_bringup",
            executable="tf_replay_filter",
            name="tf_replay_filter",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="velodyne_pointcloud",
            executable="velodyne_transform_node",
            name="velodyne_transform_node",
            output="screen",
            parameters=[transform_parameters],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="stabilized_pointcloud_to_laserscan",
            output="screen",
            parameters=[
                str(terrain_share / "config" / "pointcloud_to_scan.yaml"),
                {"use_sim_time": True},
            ],
            remappings=[
                ("cloud_in", "/velodyne_points"),
                ("scan", "/scan_stabilized"),
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                str(nav_share / "config" / "lidar_md_slam.yaml"),
                {"use_sim_time": True},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_mapper_node",
            name="terrain_mapper",
            output="screen",
            parameters=[
                str(terrain_share / "config" / "terrain_mapping.yaml"),
                {
                    "use_sim_time": True,
                    "map_bundle_directory": str(bundle),
                },
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_map_localization",
            executable="pointcloud_sector_filter",
            name="recorded_mapping_front_sector",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/velodyne_points",
                "output_topic": "/mapping/front_120_points",
                "horizontal_fov_deg": 120.0,
                "minimum_range_m": 0.35,
                "maximum_range_m": 30.0,
                "reliable_output": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="recorded_route_visualizer",
            name="recorded_route_visualizer",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "profile_file": str(bundle / "track_profile.yaml"),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="replay_cloud_accumulator",
            name="rebuild_raw_cloud_accumulator",
            output="screen",
            parameters=[{
                "input_topic": "/velodyne_points",
                "output_topic": "/replay/raw_accumulated_cloud",
                "target_frame": "map",
                "voxel_size_m": 0.08,
                "scan_stride": 2,
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_review_visualizer_node",
            name="rebuild_terrain_review_visualizer",
            output="screen",
            parameters=[{
                "input_topic": "/terrain/local_map",
                "output_topic": "/terrain/visual/classes",
                "enable_click_review": False,
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="recorded_mapping_rebuild_rviz",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-d", str(nav_share / "rviz" / "recorded_mapping_rebuild.rviz"),
                "--ros-args", "--log-level", "fatal",
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        TimerAction(period=2.0, actions=[playback]),
        RegisterEventHandler(OnProcessExit(
            target_action=playback,
            on_exit=[LogInfo(msg=(
                "[REPLAY] Sensor bag finished. The rebuilt map remains open; "
                "inspect RViz and press Ctrl+C when done."
            ))],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_directory"),
        DeclareLaunchArgument("playback_rate", default_value="0.5"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_actions),
    ])
