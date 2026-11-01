"""Reprocess a recorded VLP-16/EBIMU bag through LI-Init without hardware."""

from pathlib import Path
import shutil
import tempfile

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


def _cleanup(_context, *, replay_directory):
    path = Path(replay_directory)
    if path.is_dir() and path.name.startswith("roka_li_init_replay_"):
        shutil.rmtree(path)
    return []


def _actions(context):
    bag = Path(LaunchConfiguration("bag").perform(context)).expanduser().resolve()
    output = Path(LaunchConfiguration("output_dir").perform(context)).expanduser().resolve()
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"calibration rosbag metadata is missing: {bag}")
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"offline calibration output is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    # Work on a private copy so rosbag decompression/index recovery can never
    # modify the only field recording.
    replay_directory = Path(tempfile.mkdtemp(prefix="roka_li_init_replay_"))
    for item in bag.iterdir():
        if item.is_file() and (
            item.name == "metadata.yaml"
            or item.name.endswith(".db3")
            or item.name.endswith(".db3.zstd")
        ):
            shutil.copy2(item, replay_directory / item.name)

    pointcloud_share = Path(get_package_share_directory("velodyne_pointcloud"))
    transform_config = (
        pointcloud_share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    )
    transform_params = yaml.safe_load(transform_config.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    transform_params["calibration"] = str(pointcloud_share / "params" / "VLP16db.yaml")

    li_init_config = str(
        Path(get_package_share_directory("lidar_imu_init"))
        / "config"
        / "roka_vlp16.yaml"
    )
    li_init = Node(
        package="lidar_imu_init",
        executable="li_init",
        name="roka_lidar_imu_offline_calibration",
        output="screen",
        emulate_tty=True,
        parameters=[
            li_init_config,
            {
                "initialization.result_file": str(output / "Initialization_result.txt"),
                "initialization.shutdown_on_complete": True,
                "initialization.required_excitation_fraction": float(
                    LaunchConfiguration("required_excitation_fraction").perform(context)
                ),
                "initialization.minimum_accumulation_duration_sec": float(
                    LaunchConfiguration("minimum_accumulation_duration_sec").perform(context)
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    replay = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(replay_directory),
            "--topics", "/velodyne_packets", "/imu/data",
            "--rate", LaunchConfiguration("replay_rate"),
        ],
        output="screen",
    )
    cleanup = OpaqueFunction(
        function=_cleanup,
        kwargs={"replay_directory": str(replay_directory)},
    )
    return [
        Node(
            package="velodyne_pointcloud",
            executable="velodyne_transform_node",
            name="offline_velodyne_transform",
            output="screen",
            parameters=[transform_params],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        li_init,
        TimerAction(period=2.0, actions=[replay]),
        RegisterEventHandler(OnProcessExit(
            target_action=li_init,
            on_exit=[
                cleanup,
                EmitEvent(event=Shutdown(reason="offline LI-Init finished")),
            ],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=replay,
            on_exit=[TimerAction(period=5.0, actions=[
                cleanup,
                EmitEvent(event=Shutdown(reason="calibration bag exhausted")),
            ])],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("replay_rate", default_value="1.0"),
        DeclareLaunchArgument("required_excitation_fraction", default_value="0.06"),
        DeclareLaunchArgument("minimum_accumulation_duration_sec", default_value="60.0"),
        OpaqueFunction(function=_actions),
    ])
