import logging
import math
from pathlib import Path

import launch.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

from roka_nav2_bringup.profile_validation import is_profile_allowed
from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_path


def _configure_launch_logging(_context):
    launch.logging.launch_config.level = logging.INFO
    return []


def _validated_autonomy(context):
    map_file = Path(LaunchConfiguration("map").perform(context)).expanduser()
    backend = LaunchConfiguration("localization_backend").perform(context).strip().lower()
    map_pcd_value = LaunchConfiguration("map_pcd").perform(context).strip()
    initial_pose_value = LaunchConfiguration("initial_pose_file").perform(context).strip()
    map_pcd = Path(map_pcd_value).expanduser() if map_pcd_value else map_file.parent / "map.pcd"
    initial_pose_file = (
        Path(initial_pose_value).expanduser()
        if initial_pose_value
        else map_file.parent / "optimized_path.csv"
    )
    profile = Path(LaunchConfiguration("track_profile").perform(context)).expanduser()
    if not map_file.is_file():
        return [
            LogInfo(msg=f"[ERROR] PK-3A map does not exist: {map_file}"),
            EmitEvent(event=Shutdown(reason="PK-3A map is missing")),
        ]
    if backend == "fastlio_map" and not map_pcd.is_file():
        return [
            LogInfo(msg=f"[ERROR] Fixed PCD does not exist: {map_pcd}"),
            EmitEvent(event=Shutdown(reason="fixed localization PCD is missing")),
        ]
    try:
        document = yaml.safe_load(profile.read_text(encoding="utf-8")) or {}
        points = document.get("points", [])
        point_count = len(points)
    except (OSError, yaml.YAMLError, TypeError) as error:
        return [
            LogInfo(msg=f"[ERROR] PK-3A track profile cannot be loaded: {profile}: {error}"),
            EmitEvent(event=Shutdown(reason="PK-3A track profile is invalid")),
        ]
    closed_loop = bool(document.get("closed_loop", False))
    if point_count < 10:
        return [
            LogInfo(msg=(
                "[ERROR] PK-3A profile must contain at least 10 points: "
                f"{profile}"
            )),
            EmitEvent(event=Shutdown(reason="PK-3A track profile is incomplete")),
        ]
    allow_test_profile = LaunchConfiguration("allow_test_profile").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    competition_track_mode = LaunchConfiguration("competition_track_mode").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    if not is_profile_allowed(document, allow_test_profile=allow_test_profile):
        return [
            LogInfo(
                msg=(
                    f"[ERROR] PK-3A profile is not explicitly autonomy_approved=true: "
                    f"{profile}"
                )
            ),
            EmitEvent(event=Shutdown(reason="PK-3A risk-aware profile is not approved")),
        ]
    try:
        for point in points:
            values = [float(point["x"]), float(point["y"]), float(point.get("yaw", 0.0))]
            if "speed_mps" in point:
                values.append(float(point["speed_mps"]))
                if values[-1] <= 0.0:
                    raise ValueError("speed_mps must be positive")
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite point value")
    except (KeyError, TypeError, ValueError) as error:
        return [
            LogInfo(msg=f"[ERROR] PK-3A track point is invalid: {profile}: {error}"),
            EmitEvent(event=Shutdown(reason="PK-3A track point is invalid")),
        ]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("roka_nav2_bringup"), "launch", "lidar_md_navigation.launch.py"
        ])),
        launch_arguments={
            "map": str(map_file),
            "localization_backend": backend,
            "map_pcd": str(map_pcd),
            "initial_pose_file": str(initial_pose_file) if initial_pose_file.is_file() else "",
            "track_profile": str(profile),
            "use_lidar_driver": LaunchConfiguration("use_lidar_driver"),
            "use_imu_driver": LaunchConfiguration("use_imu_driver"),
            # FAST-LIO is the sole odom->base_link owner in fixed-PCD mode.
            "use_imu_ekf": (
                "false"
                if backend == "fastlio_map"
                else LaunchConfiguration("use_imu_ekf")
            ),
            "imu_serial_port": LaunchConfiguration("imu_serial_port"),
            "left_serial_port": LaunchConfiguration("left_serial_port"),
            "right_serial_port": LaunchConfiguration("right_serial_port"),
            "wheel_separation": LaunchConfiguration("wheel_separation"),
            "md_simulation_mode": LaunchConfiguration("md_simulation_mode"),
            "md_allow_motion": LaunchConfiguration("md_allow_motion"),
            "md_max_linear_speed": LaunchConfiguration("md_max_linear_speed"),
            "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
            "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
                "md_feedback_overspeed_limit_rpm"
            ),
            "md_cmd_vel_topic": LaunchConfiguration("md_cmd_vel_topic"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "track_auto_start": LaunchConfiguration("track_auto_start"),
            "require_field_preflight": LaunchConfiguration(
                "require_field_preflight"
            ),
            "initialize_from_profile_start": LaunchConfiguration(
                "initialize_from_profile_start"
            ),
            "initializer_position_stddev_m": (
                "1.25"
                if LaunchConfiguration("direct_recorded_route").perform(context).lower()
                in {"1", "true", "yes", "on"}
                else "0.50"
            ),
            "initializer_yaw_stddev_deg": (
                "90.0"
                if LaunchConfiguration("direct_recorded_route").perform(context).lower()
                in {"1", "true", "yes", "on"}
                else "60.0"
            ),
            "maximum_start_distance_m": (
                "2.00"
                if LaunchConfiguration("direct_recorded_route").perform(
                    context
                ).lower() in {"1", "true", "yes", "on"}
                else LaunchConfiguration("maximum_start_distance_m")
            ),
            "maximum_start_heading_error_deg": (
                "135.0"
                if LaunchConfiguration("direct_recorded_route").perform(
                    context
                ).lower() in {"1", "true", "yes", "on"}
                else LaunchConfiguration("maximum_start_heading_error_deg")
            ),
            "allow_test_profile": LaunchConfiguration("allow_test_profile"),
            "direct_recorded_route": LaunchConfiguration(
                "direct_recorded_route"
            ),
            "use_terrain_mapping": (
                "false"
                if LaunchConfiguration("direct_recorded_route").perform(context).lower()
                in {"1", "true", "yes", "on"}
                else "true"
            ),
            "use_collision_monitor": (
                "false"
                if LaunchConfiguration("direct_recorded_route").perform(context).lower()
                in {"1", "true", "yes", "on"}
                else "true"
            ),
            "operation_profile": (
                "inline_track"
                if competition_track_mode or backend == "fastlio_map"
                else "flat_commissioning"
                if allow_test_profile or (
                    LaunchConfiguration("direct_recorded_route").perform(context).lower()
                    in {"1", "true", "yes", "on"}
                )
                else "inline_track"
            ),
            "supervisor_require_terrain": (
                "false"
                if allow_test_profile or (
                    LaunchConfiguration("direct_recorded_route").perform(context).lower()
                    in {"1", "true", "yes", "on"}
                )
                else "true"
            ),
            "supervisor_enable_terrain_decisions": (
                "false"
                if LaunchConfiguration("direct_recorded_route").perform(context).lower()
                in {"1", "true", "yes", "on"}
                else "true"
            ),
            "supervisor_allow_level_fallback": (
                "true" if (allow_test_profile or competition_track_mode) else "false"
            ),
            "supervisor_require_imu": LaunchConfiguration(
                "supervisor_require_imu"
            ),
            "supervisor_require_odom": LaunchConfiguration(
                "supervisor_require_odom"
            ),
            # Section-4 slip control is intentionally excluded.  Do not let
            # competition_track_mode silently re-enable it downstream.
            "supervisor_enable_slip_detection": "false",
            "supervisor_use_contact_estimator": "true",
            # FAST-LIO publishes a continuous pose but leaves Odometry.twist
            # empty. Derive body speed from pose only for the 3D backend; the
            # flat wheel/IMU EKF already publishes a valid twist.
            "supervisor_derive_body_twist_from_pose": (
                "true" if backend == "fastlio_map" else "false"
            ),
            "supervisor_body_odom_topic": (
                "/lio/odom" if backend == "fastlio_map" else "/odom"
            ),
            "supervisor_wheel_odom_topic": "/wheel/odom",
            # The VLP-16 conversion keeps only the newest unorganized scan.
            # Real-packet replay keeps terrain/obstacle p95 age below 0.10 s,
            # so both flat commissioning and track operation use the same
            # fail-closed 0.40 s freshness limit.
            "collision_source_timeout": "0.40",
            "use_dry_run_map_tf": LaunchConfiguration("use_dry_run_map_tf"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_track_follower": "true",
            # A full-track map repeats; a section map stops at its final point.
            "track_loop": "true" if closed_loop else "false",
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )]


def generate_launch_description():
    hardware = load_hardware_config()
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_configure_launch_logging),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("localization_backend", default_value="fastlio_map"),
        DeclareLaunchArgument("map_pcd", default_value=""),
        DeclareLaunchArgument("initial_pose_file", default_value=""),
        DeclareLaunchArgument(
            "track_profile",
            default_value=str(workspace_path("maps", "track_v1", "track_profile.yaml")),
        ),
        DeclareLaunchArgument("use_lidar_driver", default_value="true"),
        DeclareLaunchArgument("use_imu_driver", default_value="true"),
        DeclareLaunchArgument("use_imu_ekf", default_value="false"),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="auto",
        ),
        DeclareLaunchArgument(
            "left_serial_port",
            default_value=(
                "/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_"
                "A76BHBER-if00-port0"
            ),
        ),
        DeclareLaunchArgument(
            "right_serial_port",
            default_value=(
                "/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_"
                "A751CYHM-if00-port0"
            ),
        ),
        DeclareLaunchArgument(
            "wheel_separation",
            default_value=str(hardware.effective_drive_separation_m),
            description="Effective skid-steer separation; not physical wheel track.",
        ),
        DeclareLaunchArgument("md_simulation_mode", default_value="true"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("md_max_linear_speed", default_value="0.60"),
        DeclareLaunchArgument("md_max_wheel_rpm", default_value="56.0"),
        DeclareLaunchArgument(
            "md_feedback_overspeed_limit_rpm", default_value="90.0"
        ),
        DeclareLaunchArgument("md_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("track_auto_start", default_value="false"),
        DeclareLaunchArgument(
            "competition_track_mode",
            default_value="false",
            description=(
                "Keep banked track geometry out of obstacle decisions while "
                "retaining read-only wheel-contact diagnostics. Automatic slip "
                "derating remains disabled until repeated field data validates it."
            ),
        ),
        DeclareLaunchArgument("require_field_preflight", default_value="true"),
        DeclareLaunchArgument("supervisor_require_imu", default_value="true"),
        DeclareLaunchArgument("supervisor_require_odom", default_value="true"),
        DeclareLaunchArgument(
            "supervisor_enable_slip_detection", default_value="false"
        ),
        DeclareLaunchArgument(
            "initialize_from_profile_start", default_value="true"
        ),
        DeclareLaunchArgument("maximum_start_distance_m", default_value="0.75"),
        DeclareLaunchArgument(
            "maximum_start_heading_error_deg", default_value="90.0"
        ),
        DeclareLaunchArgument(
            "allow_test_profile",
            default_value="false",
            description="Explicit second gate for controlled test-only profiles.",
        ),
        DeclareLaunchArgument(
            "direct_recorded_route",
            default_value="false",
            description="Replay the recorded route without terrain/obstacle decisions.",
        ),
        DeclareLaunchArgument(
            "use_dry_run_map_tf",
            default_value="false",
            description="No-hardware launch validation only; keep false with real AMCL.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_validated_autonomy),
    ])
