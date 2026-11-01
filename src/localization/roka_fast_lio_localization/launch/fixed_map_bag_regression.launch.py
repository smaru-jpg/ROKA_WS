"""Replay an ROKA bag against a fixed SC-PGO PCD without hardware."""

from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from roka_robot_description.workspace import workspace_path


def _cleanup(_context, *, replay_directory):
    path = Path(replay_directory)
    if path.is_dir() and path.name.startswith("roka_localization_replay_"):
        shutil.rmtree(path)
    return []


def _actions(context):
    bag = Path(LaunchConfiguration("bag").perform(context)).expanduser().resolve()
    map_pcd = Path(LaunchConfiguration("map_pcd").perform(context)).expanduser().resolve()
    initial_pose = LaunchConfiguration("initial_pose_file").perform(context).strip()
    output = Path(LaunchConfiguration("output_file").perform(context)).expanduser().resolve()
    if not (bag / "metadata.yaml").is_file() or not map_pcd.is_file():
        raise RuntimeError("bag metadata and map_pcd are required")
    replay_directory = Path(tempfile.mkdtemp(prefix="roka_localization_replay_"))
    for item in bag.iterdir():
        if item.is_file() and (item.name == "metadata.yaml" or item.name.endswith(".zstd")):
            shutil.copy2(item, replay_directory / item.name)

    pointcloud_share = Path(get_package_share_directory("velodyne_pointcloud"))
    config = pointcloud_share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    params = yaml.safe_load(config.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    params["calibration"] = str(pointcloud_share / "params" / "VLP16db.yaml")
    monitor = Node(
        package="roka_fast_lio_localization",
        executable="map_localization_regression_monitor",
        output="screen",
        parameters=[{"output_file": str(output), "timeout_sec": 90.0}],
    )
    replay = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(replay_directory), "--topics",
            "/velodyne_packets", "/imu/data",
        ],
        output="screen",
    )
    return [
        Node(
            package="velodyne_pointcloud",
            executable="velodyne_transform_node",
            name="offline_velodyne_transform",
            output="screen",
            parameters=[params],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("roka_fast_lio_localization")) /
                "launch" / "fastlio_sc_pgo_mapping.launch.py"
            )),
            launch_arguments={
                "output_dir": "/tmp/roka_lio_localization_frontend",
                "use_sc_pgo": "false",
                "publish_lio_tf": "true",
                "world_frame": "odom",
                "body_frame": "base_link",
                "log_level": "warn",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("roka_map_localization")) /
                "launch" / "fixed_map_localization.launch.py"
            )),
            launch_arguments={
                "map_pcd": str(map_pcd),
                "initial_pose_file": initial_pose,
                "log_level": "warn",
            }.items(),
        ),
        monitor,
        TimerAction(period=2.0, actions=[replay]),
        RegisterEventHandler(OnProcessExit(
            target_action=monitor,
            on_exit=[
                OpaqueFunction(
                    function=_cleanup,
                    kwargs={"replay_directory": str(replay_directory)},
                ),
                EmitEvent(event=Shutdown(reason="fixed-map regression completed")),
            ],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag", default_value=str(workspace_path("maps", "flat", "section_1", "sensors"))
        ),
        DeclareLaunchArgument("map_pcd", default_value=""),
        DeclareLaunchArgument("initial_pose_file", default_value=""),
        DeclareLaunchArgument(
            "output_file", default_value="/tmp/fixed_map_regression.yaml"
        ),
        OpaqueFunction(function=_actions),
    ])
