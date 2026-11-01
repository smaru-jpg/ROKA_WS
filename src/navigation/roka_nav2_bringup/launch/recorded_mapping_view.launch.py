"""Replay one saved mapping bundle as map, route, terrain and dense 3D LiDAR."""

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _validate_bundle(context):
    bundle = Path(
        LaunchConfiguration("map_directory").perform(context)
    ).expanduser().resolve()
    configured_sensor = LaunchConfiguration("sensor_directory").perform(context).strip()
    if configured_sensor.lower() == "auto":
        local_sensor = bundle / "sensors"
        if (local_sensor / "metadata.yaml").is_file():
            sensor_directory = local_sensor
        else:
            metadata = yaml.safe_load(
                (bundle / "metadata.yaml").read_text(encoding="utf-8")
            ) or {}
            source = Path(str(metadata.get("source_acquisition", ""))).expanduser()
            sensor_directory = source / "sensors"
        context.launch_configurations["sensor_directory"] = str(sensor_directory)
    else:
        sensor_directory = Path(configured_sensor).expanduser().resolve()
    required = [
        bundle / "occupancy.yaml",
        bundle / "track_profile.yaml",
        sensor_directory / "metadata.yaml",
        bundle / "terrain_map.bag" / "metadata.yaml",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if not missing:
        return []
    return [EmitEvent(event=Shutdown(
        reason="recorded mapping view is missing: " + ", ".join(missing)
    ))]


def _map_server(context):
    bundle = Path(
        LaunchConfiguration("map_directory").perform(context)
    ).expanduser().resolve()
    processed = bundle / "derived" / "fastlio_sc_pgo"
    if not (processed / "map.pcd").is_file():
        processed = bundle
    terrain_bag = processed / "terrain_global.bag"
    if not terrain_bag.is_dir():
        terrain_bag = bundle / "terrain_map.bag"
    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="recorded_map_server",
            output="screen",
            parameters=[{
                "yaml_filename": str(bundle / "occupancy.yaml"),
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_terrain_mapping",
            executable="terrain_map_server_node",
            name="recorded_terrain_map_server",
            output="screen",
            parameters=[{
                "bag_path": str(terrain_bag),
                "bag_topic": "/terrain/static_map",
                "output_topic": "/terrain/static_map",
                "zero_timestamp": True,
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="recorded_route_visualizer",
            name="recorded_route_visualizer",
            output="screen",
            parameters=[{
                "profile_file": str(bundle / "track_profile.yaml"),
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="recorded_map_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["recorded_map_server"],
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ]
    if all((processed / name).is_file() for name in (
        "map.pcd", "track_profile.yaml", "occupancy.yaml"
    )):
        actions.append(Node(
            package="roka_nav2_bringup",
            executable="track_mapping_visualizer",
            name="recorded_processed_map_visualizer",
            output="screen",
            parameters=[{
                "bundle_directory": str(processed),
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ))
    actions.append(Node(
        package="roka_terrain_mapping",
        executable="terrain_review_visualizer_node",
        name="recorded_terrain_review_visualizer",
        output="screen",
        parameters=[{
            "input_topic": "/terrain/static_map",
            "output_topic": "/terrain/visual/classes",
            "audit_file": str(bundle / "terrain_review_annotations.yaml"),
            "enable_click_review": True,
            "use_sim_time": True,
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    ))
    return actions


def _velodyne_transform(_context):
    share = Path(get_package_share_directory("velodyne_pointcloud"))
    parameter_file = share / "config" / "VLP16-velodyne_transform_node-params.yaml"
    parameters = yaml.safe_load(parameter_file.read_text(encoding="utf-8"))[
        "velodyne_transform_node"
    ]["ros__parameters"]
    parameters["calibration"] = str(share / "params" / "VLP16db.yaml")
    parameters["use_sim_time"] = True
    return [Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        output="screen",
        parameters=[parameters],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )]


def generate_launch_description():
    bag = LaunchConfiguration("sensor_directory")
    replay = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag, "--clock", "100", "--rate",
            LaunchConfiguration("playback_rate"), "--topics",
            "/velodyne_packets", "/tf", "/tf_static", "/odom",
            "/track/recorded_path", "--remap",
            "/track/recorded_path:=/replay/growing_path",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_directory"),
        DeclareLaunchArgument(
            "sensor_directory",
            default_value="auto",
            description="Raw acquisition bag; auto follows processed-map lineage.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("forward_display_fov_deg", default_value="120.0"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_validate_bundle),
        OpaqueFunction(function=_map_server),
        OpaqueFunction(function=_velodyne_transform),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="recorded_map_to_odom",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
        ),
        Node(
            package="roka_map_localization",
            executable="pointcloud_sector_filter",
            name="recorded_mapping_front_sector",
            output="screen",
            parameters=[{
                "input_topic": "/velodyne_points",
                "output_topic": "/mapping/front_120_points",
                "horizontal_fov_deg": ParameterValue(
                    LaunchConfiguration("forward_display_fov_deg"), value_type=float
                ),
                "minimum_range_m": 0.35,
                "maximum_range_m": 30.0,
                "reliable_output": True,
                "use_sim_time": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="replay_cloud_accumulator",
            name="recorded_raw_cloud_accumulator",
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
            package="rviz2",
            executable="rviz2",
            name="recorded_mapping_rviz",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("roka_nav2_bringup"),
                    "rviz",
                    "recorded_mapping_replay.rviz",
                ]),
                "--ros-args",
                "--log-level",
                "fatal",
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        TimerAction(period=2.0, actions=[replay]),
        RegisterEventHandler(OnProcessExit(
            target_action=replay,
            on_exit=[LogInfo(msg=(
                "Recorded bag reached the end. Saved maps and the accumulated "
                "cloud remain visible; press Ctrl+C after review."
            ))],
        )),
    ])
