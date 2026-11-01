"""Replay an existing ROKA sensor bag through FAST-LIO2 and SC-PGO."""

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


def _cleanup_replay(_context, *, replay_directory):
    path = Path(replay_directory)
    if path.is_dir() and path.name.startswith("roka_lio_replay_"):
        shutil.rmtree(path)
    return []


def _actions(context):
    bag = Path(LaunchConfiguration("bag").perform(context)).expanduser().resolve()
    output = Path(LaunchConfiguration("output_dir").perform(context)).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"rosbag metadata is missing: {bag}")
    environment = "unspecified"
    bundle_metadata = bag.parent / "metadata.yaml"
    if bundle_metadata.is_file():
        environment = str(
            (yaml.safe_load(bundle_metadata.read_text(encoding="utf-8")) or {}).get(
                "environment", "unspecified"
            )
        ).strip().lower()
    # Unified capture deliberately does not label the future output as flat or
    # track. Infer only the regression envelope from the recorded excitation;
    # never give an ordinary flat-like bag the old 5 m track Z allowance.
    validation_context = environment
    maximum_z_span_m = 2.0
    sensor_contract = bag.parent / "derived" / "sensor_contract.yaml"
    if sensor_contract.is_file():
        contract = yaml.safe_load(sensor_contract.read_text(encoding="utf-8")) or {}
        excitation = contract.get("calibration_excitation") or {}
        if excitation.get("sufficient_roll_pitch_excitation") is False:
            validation_context = "flat_like_capture"
            maximum_z_span_m = 0.35
        else:
            validation_context = "terrain_capture"
    elif environment == "flat":
        validation_context = "declared_flat"
        maximum_z_span_m = 0.35
    replay_directory = Path(tempfile.mkdtemp(prefix="roka_lio_replay_"))
    shutil.copy2(bag / "metadata.yaml", replay_directory / "metadata.yaml")
    for item in bag.iterdir():
        if not item.is_file():
            continue
        if item.name.endswith(".db3.zstd"):
            shutil.copy2(item, replay_directory / item.name)
        elif item.name.endswith(".db3") and not (bag / f"{item.name}.zstd").exists():
            shutil.copy2(item, replay_directory / item.name)

    pointcloud_share = Path(get_package_share_directory("velodyne_pointcloud"))
    params_path = pointcloud_share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    params = yaml.safe_load(params_path.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    params["calibration"] = str(pointcloud_share / "params" / "VLP16db.yaml")

    replay = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(replay_directory),
            "--rate", LaunchConfiguration("playback_rate"),
            "--topics", "/velodyne_packets", "/imu/data", "/wheel/odom",
        ],
        output="screen",
    )
    monitor = Node(
        package="roka_fast_lio_localization",
        executable="lio_regression_monitor",
        name="lio_regression_monitor",
        output="screen",
        parameters=[{
            "output_file": str(output / "regression.yaml"),
            "idle_timeout_sec": 2.0,
            "startup_timeout_sec": 45.0,
            "maximum_step_m": 0.10,
            "maximum_yaw_step_deg": 8.0,
            "maximum_z_span_m": maximum_z_span_m,
            "validation_context": validation_context,
        }],
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
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("roka_fast_lio_localization")) /
                    "launch" / "fastlio_sc_pgo_mapping.launch.py")
            ),
            launch_arguments={
                "output_dir": str(output),
                "use_sc_pgo": "true",
                "publish_lio_tf": "false",
                "apply_dynamic_calibration": LaunchConfiguration(
                    "apply_dynamic_calibration"
                ),
                "time_offset_override_sec": LaunchConfiguration(
                    "time_offset_override_sec"
                ),
                "point_filter_num_override": LaunchConfiguration(
                    "point_filter_num_override"
                ),
                "filter_size_surf_override_m": LaunchConfiguration(
                    "filter_size_surf_override_m"
                ),
                "filter_size_map_override_m": LaunchConfiguration(
                    "filter_size_map_override_m"
                ),
                "blind_override_m": LaunchConfiguration("blind_override_m"),
                "use_sim_time": "false",
                "log_level": "warn",
            }.items(),
        ),
        monitor,
        TimerAction(period=2.0, actions=[replay]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=monitor,
                on_exit=[
                    OpaqueFunction(
                        function=_cleanup_replay,
                        kwargs={"replay_directory": str(replay_directory)},
                    ),
                    EmitEvent(event=Shutdown(
                        reason="offline LIO regression completed"
                    )),
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag", default_value=str(workspace_path("maps", "flat", "section_1", "sensors"))
        ),
        DeclareLaunchArgument(
            "output_dir", default_value="/tmp/roka_fastlio_regression"
        ),
        DeclareLaunchArgument("apply_dynamic_calibration", default_value="true"),
        DeclareLaunchArgument("time_offset_override_sec", default_value=""),
        DeclareLaunchArgument("point_filter_num_override", default_value=""),
        DeclareLaunchArgument("filter_size_surf_override_m", default_value=""),
        DeclareLaunchArgument("filter_size_map_override_m", default_value=""),
        DeclareLaunchArgument("blind_override_m", default_value=""),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        OpaqueFunction(function=_actions),
    ])
