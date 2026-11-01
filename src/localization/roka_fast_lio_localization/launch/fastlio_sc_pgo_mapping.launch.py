"""ROKA VLP-16/EBIMU FAST-LIO2 frontend with guarded SC-PGO backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roka_robot_description.hardware_config import (
    default_hardware_config_path,
    load_hardware_config,
)


def _nodes(context):
    output_directory = Path(
        LaunchConfiguration("output_dir").perform(context)
    ).expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    fast_lio_config = LaunchConfiguration("fast_lio_config").perform(context)
    hardware = load_hardware_config(
        LaunchConfiguration("hardware_config").perform(context)
    )
    apply_dynamic_calibration = (
        LaunchConfiguration("apply_dynamic_calibration")
        .perform(context)
        .strip()
        .lower()
        in {"1", "true", "yes", "on"}
    )
    rotation = (
        list(hardware.lidar_in_imu_rotation_matrix)
        if apply_dynamic_calibration
        else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    time_offset = (
        hardware.time_offset_lidar_to_imu_sec if apply_dynamic_calibration else 0.0
    )
    time_offset_override = (
        LaunchConfiguration("time_offset_override_sec").perform(context).strip()
    )
    if time_offset_override:
        time_offset = float(time_offset_override)
    fast_lio_overrides = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "map_file_path": str(output_directory / "lio_frontend_map.pcd"),
        "common.publish_tf": LaunchConfiguration("publish_lio_tf"),
        "common.world_frame": LaunchConfiguration("world_frame"),
        "common.body_frame": LaunchConfiguration("body_frame"),
        "common.time_offset_lidar_to_imu": time_offset,
        "preprocess.scan_rate": int(round(hardware.lidar_rate_hz)),
        "mapping.extrinsic_T": list(hardware.lidar_in_imu_translation),
        "mapping.extrinsic_R": rotation,
        "mapping.base_to_imu_T": list(hardware.base_to_imu.xyz),
        "mapping.base_to_imu_R": list(hardware.base_to_imu_rotation_matrix),
    }
    optional_numeric_overrides = {
        "point_filter_num": ("point_filter_num_override", int),
        "filter_size_surf": ("filter_size_surf_override_m", float),
        "filter_size_map": ("filter_size_map_override_m", float),
        "preprocess.blind": ("blind_override_m", float),
    }
    for parameter_name, (argument_name, converter) in optional_numeric_overrides.items():
        raw_value = LaunchConfiguration(argument_name).perform(context).strip()
        if raw_value:
            fast_lio_overrides[parameter_name] = converter(raw_value)
    return [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="roka_fast_lio2",
            output="screen",
            parameters=[
                fast_lio_config,
                fast_lio_overrides,
            ],
            remappings=[
                ("/Odometry", "/lio/odom"),
                ("/path", "/lio/path"),
                ("/cloud_registered", "/lio/cloud_registered"),
                ("/cloud_registered_body", "/lio/cloud_body"),
                ("/cloud_effected", "/lio/cloud_effected"),
                ("/Laser_map", "/lio/map"),
                ("map_save", "/lio/save_map"),
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="sc_pgo",
            executable="alaserPGO",
            name="roka_sc_pgo",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_sc_pgo")),
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "save_directory": str(output_directory),
                "map_frame": "map",
                "scan_line": 16,
                "minimum_range": 0.9,
                "mapping_line_resolution": 0.20,
                "mapping_plane_resolution": 0.20,
                "mapviz_filter_size": 0.15,
                "keyframe_meter_gap": 0.30,
                "keyframe_deg_gap": 8.0,
                "sc_dist_thres": 0.15,
                "sc_max_radius": 30.0,
                "publish_tf": False,
                "use_current_stamp_for_aft_pgo_odom": False,
                "persist_keyframe_pcd": False,
                "use_gps": False,
                "maximum_sync_error_sec": 0.03,
                "minimum_loop_arc_length": 8.0,
                "stable_loop_candidate_count": 2,
                "stable_loop_candidate_tolerance": 2,
                "icp_max_correspondence_distance": 1.5,
                "icp_fitness_threshold": 0.20,
                "icp_minimum_overlap": 0.45,
                "maximum_loop_translation_correction": 2.0,
                "maximum_loop_z_correction": 0.75,
                "maximum_loop_rotation_correction": 0.78539816339,
                "loop_history_keyframes": 10,
                "maximum_adjacent_translation_deformation": 0.25,
                "maximum_adjacent_rotation_deformation": 0.15,
            }],
            remappings=[
                ("/aft_mapped_to_init", "/lio/odom"),
                ("/velodyne_cloud_registered_local", "/lio/cloud_body"),
                ("/aft_pgo_odom", "/pgo/odom"),
                ("/aft_pgo_path", "/pgo/path"),
                ("/aft_pgo_map", "/pgo/map"),
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("fast_lio")) / "config" / "roka_vlp16.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/roka_lio_mapping"),
        DeclareLaunchArgument("fast_lio_config", default_value=default_config),
        DeclareLaunchArgument(
            "hardware_config", default_value=str(default_hardware_config_path())
        ),
        DeclareLaunchArgument("use_sc_pgo", default_value="true"),
        DeclareLaunchArgument("publish_lio_tf", default_value="false"),
        # Diagnostic escape hatch used by the deterministic bag regression.
        # Production mapping keeps the dynamically validated values enabled.
        DeclareLaunchArgument("apply_dynamic_calibration", default_value="true"),
        DeclareLaunchArgument("time_offset_override_sec", default_value=""),
        DeclareLaunchArgument("point_filter_num_override", default_value=""),
        DeclareLaunchArgument("filter_size_surf_override_m", default_value=""),
        DeclareLaunchArgument("filter_size_map_override_m", default_value=""),
        DeclareLaunchArgument("blind_override_m", default_value=""),
        DeclareLaunchArgument("world_frame", default_value="lio_odom"),
        DeclareLaunchArgument("body_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_nodes),
    ])
