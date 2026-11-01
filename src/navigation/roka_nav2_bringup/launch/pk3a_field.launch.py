"""One-command real PK-3A mapping or approved-path automatic autonomy."""

import logging
from pathlib import Path
import shutil

import launch.logging
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from roka_md_driver.xbox_identity import resolve_assigned_sdl_device_id
from roka_robot_description.hardware_config import (
    default_hardware_config_path,
    load_hardware_config,
)
from roka_robot_description.workspace import workspace_path
from roka_nav2_bringup.map_bundle_selection import (
    latest_valid_map_bundle,
    map_bundle_for_id,
    next_mapping_bundle,
    occupancy_map_is_valid,
)
from roka_nav2_bringup.map_bundle_quality import (
    bundle_is_complete,
    requires_embedded_sensor_bag,
    validate_bundle,
)
from roka_nav2_bringup.map_review import (
    active_reviewed_map,
    active_reviewed_occupancy,
)
from roka_nav2_bringup.field_session_guard import other_field_launches
from roka_nav2_bringup.profile_validation import (
    is_profile_allowed,
    is_test_only_profile,
)


def _configure_launch_logging(_context):
    # Child nodes still receive log_level:=warn.  Keep launch's forwarding at
    # INFO so operator-facing stdout (Xbox/mapping state) reaches the terminal.
    launch.logging.launch_config.level = logging.INFO
    return []


def _reject_concurrent_field_stack(_context):
    running = other_field_launches()
    if not running:
        return []
    details = "; ".join(f"pid={pid} {command}" for pid, command in running)
    launch.logging.get_logger("pk3a_field").error(
        f"Another ROKA field launch is already running: {details}"
    )
    return [
        EmitEvent(event=Shutdown(reason="only one ROKA field stack may run")),
    ]


def _real_motion_arguments(context):
    return {
        "use_lidar_driver": "true",
        "use_md_driver": "true",
        "md_simulation_mode": "false",
        "md_allow_motion": LaunchConfiguration("md_allow_motion"),
        "md_max_linear_speed": LaunchConfiguration("md_max_linear_speed"),
        "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
        "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
            "md_feedback_overspeed_limit_rpm"
        ),
        "left_serial_port": LaunchConfiguration("left_serial_port"),
        "right_serial_port": LaunchConfiguration("right_serial_port"),
        "wheel_separation": LaunchConfiguration("wheel_separation"),
        "hardware_config": LaunchConfiguration("hardware_config"),
        "use_imu_ekf": LaunchConfiguration("use_imu_ekf"),
        "imu_serial_port": LaunchConfiguration("imu_serial_port"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "forward_display_fov_deg": LaunchConfiguration("forward_display_fov_deg"),
        "log_level": LaunchConfiguration("log_level"),
    }


def _mapping_mode(context):
    environment = LaunchConfiguration("environment").perform(context).strip().lower()
    hardware = load_hardware_config(LaunchConfiguration("hardware_config").perform(context))
    environment_profiles = {
        # Acquisition is intentionally backend-neutral.  It records the raw
        # VLP-16, IMU, wheel odometry, TF and driven route once; 2D/3D maps are
        # produced later from this same immutable sensor session.
        "capture": {
            "track_width_m": "1.20",
            "track_closure_distance_m": "0.80",
            "track_closure_heading_tolerance_deg": "75.0",
            "track_minimum_closed_loop_length_m": "3.0",
            "track_minimum_closed_loop_area_m2": "0.50",
        },
        # One natural Xbox traversal is enough. These values allow an
        # approximate return to the marked start without accepting an
        # out-and-back path with the opposite heading as a closed loop.
        "flat": {
            "track_width_m": "1.20",
            "track_closure_distance_m": "0.80",
            "track_closure_heading_tolerance_deg": "75.0",
            "track_minimum_closed_loop_length_m": "3.0",
            "track_minimum_closed_loop_area_m2": "0.50",
        },
        # ``inline`` is the single 3D profile for both indoor slopes and the
        # real PK-3A course.  Keeping one profile prevents calibration/map
        # products from silently diverging between two names.
        "inline": {
            "track_width_m": "1.20",
            "track_closure_distance_m": "0.60",
            "track_closure_heading_tolerance_deg": "65.0",
            "track_minimum_closed_loop_length_m": "5.0",
            "track_minimum_closed_loop_area_m2": "1.00",
        },
    }
    if environment not in environment_profiles:
        launch.logging.get_logger("pk3a_field").error(
            f"Unsupported mapping environment: {environment}; use capture, flat or inline"
        )
        return [EmitEvent(event=Shutdown(reason="invalid mapping environment"))]
    if environment == "inline" and not (
        hardware.rotation_validated_dynamically
        and hardware.temporal_offset_validated_dynamically
    ):
        launch.logging.get_logger("pk3a_field").error(
            "Track 3D mapping is locked until LiDAR-IMU rotation and time offset "
            f"are dynamically validated in {hardware.path}"
        )
        return [EmitEvent(event=Shutdown(reason="LiDAR-IMU calibration is incomplete"))]
    map_id = LaunchConfiguration("map_id").perform(context).strip()
    configured = LaunchConfiguration("map_bundle_directory").perform(context).strip()
    overwrite_existing = LaunchConfiguration("overwrite_existing").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    if map_id.lower() != "auto":
        try:
            bundle = map_bundle_for_id(
                LaunchConfiguration("map_root").perform(context), map_id
            )
        except ValueError as error:
            launch.logging.get_logger("pk3a_field").error(str(error))
            return [EmitEvent(event=Shutdown(reason="invalid map_id"))]
    elif configured.lower() == "auto":
        bundle = next_mapping_bundle(LaunchConfiguration("map_root").perform(context))
    else:
        bundle = Path(configured).expanduser().resolve()
    bundle_has_content = bundle.exists() and (
        not bundle.is_dir() or any(bundle.iterdir())
    )
    if bundle_has_content:
        if not overwrite_existing:
            launch.logging.get_logger("pk3a_field").error(
                f"Mapping bundle is not empty: {bundle}; pass "
                "overwrite_existing:=true to replace this named section"
            )
            return [
                EmitEvent(event=Shutdown(reason="refusing to overwrite an existing map run")),
            ]
        # Automatic replacement is intentionally limited to a validated,
        # explicitly named map_id.  Never trash an arbitrary configured path
        # or whatever directory happens to be selected by map_id:=auto.
        replaceable_sections = {f"section_{number}" for number in range(1, 6)}
        if (
            map_id.lower() not in replaceable_sections
            or configured.lower() != "auto"
        ):
            launch.logging.get_logger("pk3a_field").error(
                "Automatic overwrite is limited to map_id:=section_1..section_5 "
                "with map_bundle_directory:=auto"
            )
            return [
                EmitEvent(event=Shutdown(reason="unsafe automatic map overwrite request")),
            ]
        launch.logging.get_logger("pk3a_field").warning(
            f"Existing map remains active until the replacement passes validation: {bundle}"
        )

    # Every producer writes into a fixed, private staging directory.  Removing
    # a stale staging run is safe because it was never promoted and can never
    # be selected for autonomy.  The current final bundle is untouched here.
    staging_root = bundle.parent / ".staging"
    staging = staging_root / bundle.name
    try:
        staging.relative_to(staging_root)
        if staging.exists():
            if staging.is_dir():
                shutil.rmtree(staging)
            else:
                staging.unlink()
        staging.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError) as error:
        launch.logging.get_logger("pk3a_field").error(
            f"Could not prepare mapping staging directory {staging}: {error}"
        )
        return [EmitEvent(event=Shutdown(reason="mapping staging preparation failed"))]
    arguments = _real_motion_arguments(context)
    arguments.update({
        "environment": environment,
        **environment_profiles[environment],
        "map_bundle_directory": str(staging),
        "final_map_bundle_directory": str(bundle),
        "overwrite_existing": "true" if overwrite_existing else "false",
        "use_imu_driver": "true",
        "use_2d_slam": "true" if environment == "flat" else "false",
        "use_3d_slam": "true" if environment == "inline" else "false",
        "mapping_backend": (
            "raw_sensor_capture" if environment == "capture"
            else "slam_toolbox_2d" if environment == "flat"
            else "fastlio_sc_pgo"
        ),
        "track_map_frame": "map" if environment == "flat" else "odom",
        "track_sample_frame": "map" if environment == "flat" else "odom",
        "track_base_frame": "base_footprint" if environment == "flat" else "base_link",
        "acquisition_only": "true" if environment == "capture" else "false",
        "use_terrain_mapping": "false" if environment == "capture" else "true",
        "use_terrain_monitor": "false" if environment == "capture" else "true",
        "record_bag": LaunchConfiguration("record_bag"),
        "record_track_path": "true",
        "use_xbox_mapping_controls": "true",
        "joy_device_id": LaunchConfiguration("joy_device_id"),
        # Mapping is human-driven with LB dead-man and the tested 0.30 m/s
        # Xbox limit.  Terrain data is recorded, but autonomous safety gating
        # must not deadlock the first map before terrain exists.
        "md_cmd_vel_topic": "/cmd_vel_joy",
        "use_safety_pipeline": "false",
    })
    return [
        LogInfo(msg=(
            f"PK-3A manual mapping staging: {staging}; "
            f"validated destination: {bundle}"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"),
                "launch",
                "mapping_manual.launch.py",
            ])),
            launch_arguments=arguments.items(),
        ),
    ]


def _autonomy_latest_mode(context):
    test_mode = LaunchConfiguration("test_mode").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    recorded_route_mode = LaunchConfiguration("recorded_route_mode").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    competition_track_mode = LaunchConfiguration("competition_track_mode").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    if test_mode and recorded_route_mode:
        launch.logging.get_logger("pk3a_field").error(
            "test_mode and recorded_route_mode are mutually exclusive"
        )
        return [EmitEvent(event=Shutdown(reason="conflicting autonomy route modes"))]
    auto_start = LaunchConfiguration("autonomy_auto_start").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    requested_localization_backend = LaunchConfiguration("localization_backend").perform(
        context
    ).strip().lower()
    map_id = LaunchConfiguration("map_id").perform(context).strip()
    configured = LaunchConfiguration("map_bundle_directory").perform(context).strip()
    if map_id.lower() != "auto":
        try:
            bundle = map_bundle_for_id(
                LaunchConfiguration("map_root").perform(context), map_id
            )
        except ValueError as error:
            launch.logging.get_logger("pk3a_field").error(str(error))
            return [EmitEvent(event=Shutdown(reason="invalid map_id"))]
    elif configured.lower() == "auto":
        bundle = latest_valid_map_bundle(
            LaunchConfiguration("map_root").perform(context)
        )
    else:
        bundle = Path(configured).expanduser().resolve()
    if bundle is None:
        launch.logging.get_logger("pk3a_field").error(
            "No valid maps/*/occupancy.yaml map bundle exists yet"
        )
        return [
            EmitEvent(event=Shutdown(reason="no saved PK-3A map is available")),
        ]
    source_map_yaml = bundle / "occupancy.yaml"
    if not bundle_is_complete(bundle) or not occupancy_map_is_valid(source_map_yaml):
        launch.logging.get_logger("pk3a_field").error(
            f"Invalid occupancy map or image: {source_map_yaml}"
        )
        return [
            EmitEvent(event=Shutdown(reason="selected PK-3A map is incomplete")),
        ]
    try:
        validate_bundle(
            bundle,
            require_sensor_bag=requires_embedded_sensor_bag(bundle),
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        # A legacy manifest only says that saving once completed; it cannot
        # prove that the PCD, 3D graph, 2D map and driven path still belong to
        # the same run.  Re-run the full coherence checks before real motion.
        launch.logging.get_logger("pk3a_field").error(
            f"Selected map bundle failed integrity/trajectory checks: {bundle}: {error}"
        )
        return [
            EmitEvent(event=Shutdown(reason="selected PK-3A map failed validation")),
        ]
    try:
        bundle_metadata = yaml.safe_load(
            (bundle / "metadata.yaml").read_text(encoding="utf-8")
        ) or {}
    except (OSError, TypeError, yaml.YAMLError) as error:
        launch.logging.get_logger("pk3a_field").error(
            f"Selected map metadata is unreadable: {bundle}: {error}"
        )
        return [EmitEvent(event=Shutdown(reason="selected map metadata is invalid"))]
    map_backend = str(
        bundle_metadata.get("mapping_backend", "slam_toolbox_2d")
    ).strip().lower()
    if requested_localization_backend == "auto":
        localization_backend = (
            "fastlio_map" if map_backend == "fastlio_sc_pgo" else "legacy"
        )
    else:
        localization_backend = requested_localization_backend
    if localization_backend not in {"fastlio_map", "legacy"}:
        launch.logging.get_logger("pk3a_field").error(
            f"Unsupported localization_backend={localization_backend}"
        )
        return [EmitEvent(event=Shutdown(reason="invalid localization backend"))]
    if localization_backend == "fastlio_map" and map_backend != "fastlio_sc_pgo":
        launch.logging.get_logger("pk3a_field").error(
            f"Map {bundle.name} was built with {map_backend}, not fastlio_sc_pgo. "
            "Remap this section before fixed-PCD autonomy, or use "
            "localization_backend:=auto (recommended) / :=legacy for this 2D map."
        )
        return [EmitEvent(event=Shutdown(reason="map/localization backend mismatch"))]
    if localization_backend == "legacy" and map_backend == "fastlio_sc_pgo":
        launch.logging.get_logger("pk3a_field").error(
            "A 3D SC-PGO map must use fixed-PCD localization, not legacy AMCL."
        )
        return [EmitEvent(event=Shutdown(reason="3D map requires fixed-PCD localization"))]
    try:
        map_yaml = active_reviewed_occupancy(bundle, require_approved=True)
        active_pcd = (
            active_reviewed_map(bundle, require_approved=True)
            if map_backend == "fastlio_sc_pgo"
            else bundle / "map.pcd"
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        detail = str(error)
        if map_backend == "slam_toolbox_2d":
            detail += (
                "; a newly mapped/overwritten section requires a new occupancy "
                "review, then pk3a_map_check --prepare-test"
            )
        launch.logging.get_logger("pk3a_field").error(detail)
        return [EmitEvent(event=Shutdown(reason="map review is incomplete"))]

    # The manual Xbox trace is evidence, not an autonomous command path.
    # Normal autonomy may consume only the separately generated, vehicle-aware
    # safe profile. Controlled flat commissioning keeps its own explicit file.
    profile = bundle / (
        "track_profile_recorded.yaml"
        if recorded_route_mode
        else ("track_profile_test.yaml" if test_mode else "track_profile_safe.yaml")
    )
    try:
        document = yaml.safe_load(profile.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError, TypeError) as error:
        launch.logging.get_logger("pk3a_field").error(
            f"Latest map has no usable track profile: {profile}: {error}"
        )
        return [
            EmitEvent(event=Shutdown(reason="latest map has no autonomous path")),
        ]
    points = document.get("points", [])
    if len(points) < 10:
        launch.logging.get_logger("pk3a_field").error(
            f"Selected track profile has fewer than 10 points: {profile}"
        )
        return [
            EmitEvent(event=Shutdown(reason="latest autonomous path is incomplete")),
        ]
    if recorded_route_mode and not (
        document.get("profile_role") == "recorded_route_replay"
        and document.get("recorded_route_replay") is True
        and document.get("bounded_route_smoothing") is True
        and isinstance(document.get("centerline_geometry"), dict)
    ):
        launch.logging.get_logger("pk3a_field").error(
            f"Selected recorded profile is not an exact-route replay: {profile}"
        )
        return [
            EmitEvent(event=Shutdown(reason="recorded route replay profile is invalid")),
        ]
    if test_mode and not is_test_only_profile(document):
        launch.logging.get_logger("pk3a_field").error(
            f"Selected test profile is not explicitly test-only: {profile}"
        )
        return [
            EmitEvent(event=Shutdown(reason="selected path is not a controlled test profile")),
        ]
    if (
        not test_mode and not recorded_route_mode
        and document.get("profile_role") != "safe_autonomy"
    ):
        launch.logging.get_logger("pk3a_field").error(
            f"Selected normal profile is not vehicle-aware safe output: {profile}"
        )
        return [
            EmitEvent(event=Shutdown(reason="normal path is not vehicle-aware safe")),
        ]
    if not is_profile_allowed(document, allow_test_profile=test_mode):
        reason = document.get("approval_reason", "terrain/risk approval is missing")
        launch.logging.get_logger("pk3a_field").error(
            f"Latest track profile is not autonomy-approved: {profile}; reason={reason}"
        )
        return [
            EmitEvent(event=Shutdown(reason="latest path is not approved for autonomy")),
        ]

    arguments = {
        "map": str(map_yaml),
        "localization_backend": localization_backend,
        "map_pcd": str(active_pcd),
        "initial_pose_file": (
            str(bundle / "optimized_path.csv")
            if (bundle / "optimized_path.csv").is_file()
            else ""
        ),
        "track_profile": str(profile),
        "use_lidar_driver": "true",
        "use_imu_driver": "true",
        "use_imu_ekf": (
            "false" if localization_backend == "fastlio_map"
            else LaunchConfiguration("use_imu_ekf")
        ),
        "imu_serial_port": LaunchConfiguration("imu_serial_port"),
        "left_serial_port": LaunchConfiguration("left_serial_port"),
        "right_serial_port": LaunchConfiguration("right_serial_port"),
        "wheel_separation": LaunchConfiguration("wheel_separation"),
        "md_simulation_mode": "false",
        "md_allow_motion": LaunchConfiguration("md_allow_motion"),
        "md_max_linear_speed": LaunchConfiguration("md_max_linear_speed"),
        "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
        "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
            "md_feedback_overspeed_limit_rpm"
        ),
        # The final Nav2/collision-monitor command remains visible on /cmd_vel,
        # but MD200T consumes only the arm-interlocked output below.
        "md_cmd_vel_topic": "/cmd_vel_arm_locked",
        "track_auto_start": "true" if auto_start else "false",
        "require_field_preflight": "true",
        "supervisor_require_imu": "true",
        "supervisor_require_odom": "true",
        # Section-4 slip control is intentionally excluded from this build.
        # MD communication, current and overspeed protection stay active.
        "supervisor_enable_slip_detection": "false",
        "initialize_from_profile_start": LaunchConfiguration(
            "initialize_from_profile_start"
        ),
        "maximum_start_distance_m": (
            "2.00"
            if recorded_route_mode
            else LaunchConfiguration("maximum_start_distance_m")
        ),
        "maximum_start_heading_error_deg": (
            "90.0"
            if recorded_route_mode
            else LaunchConfiguration("maximum_start_heading_error_deg")
        ),
        "allow_test_profile": "true" if test_mode else "false",
        "direct_recorded_route": "true" if recorded_route_mode else "false",
        "competition_track_mode": LaunchConfiguration("competition_track_mode"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "log_level": LaunchConfiguration("log_level"),
    }
    autonomy_label = (
        "recorded-route"
        if recorded_route_mode
        else ("controlled TEST" if test_mode else "latest-map")
    )
    actions = [
        LogInfo(msg=(
            f"PK-3A {autonomy_label} autonomy: "
            f"{bundle}; profile={profile.name}; "
            f"start={'automatic (no Xbox)' if auto_start else 'Xbox Y/A'}"
        )),
        Node(
            package="roka_nav2_bringup",
            executable="pk3a_field_preflight",
            name="pk3a_autonomy_preflight",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "duration_sec": 8.0,
                "failed_check_retry_sec": 10.0,
                "retry_until_ready": True,
                "require_xbox": False,
                "require_arm_hold": True,
                "left_serial_port": LaunchConfiguration("left_serial_port"),
                "right_serial_port": LaunchConfiguration("right_serial_port"),
            }],
            arguments=[
                "--ros-args", "--log-level", LaunchConfiguration("log_level")
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"),
                "launch",
                "pk3a_autonomy.launch.py",
            ])),
            launch_arguments=arguments.items(),
        ),
    ]
    external_start_control = LaunchConfiguration(
        "autonomy_external_start_control"
    ).perform(context).lower() in {"1", "true", "yes", "on"}
    if external_start_control:
        actions.append(LogInfo(msg=(
            "[AUTONOMY] route start is reserved for the Section mission controller; "
            "Xbox Y/A controls are disabled; B stop and LB manual takeover remain enabled"
        )))
    optional_xbox = LaunchConfiguration("autonomy_optional_xbox").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    if auto_start and not optional_xbox:
        return actions
    try:
        device_id = resolve_assigned_sdl_device_id(
            LaunchConfiguration("joy_device_id").perform(context)
        )
    except RuntimeError as error:
        if auto_start:
            # Auto start never depends on a controller. If the optional
            # controller is present, B cancellation and LB manual takeover are
            # added; otherwise autonomous commissioning continues unchanged.
            actions.append(LogInfo(msg=f"Optional Xbox controls unavailable: {error}"))
            return actions
        raise
    xbox_config = PathJoinSubstitution([
        FindPackageShare("roka_md_driver"),
        "config",
        "xbox_teleop.yaml",
    ])
    actions.extend([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                # The resolver already returns an int. Wrapping ``"0"`` in a
                # typed ParameterValue makes launch validate the raw string
                # before conversion and raises: value='0' is not an int.
                "device_id": device_id,
                "deadzone": 0.12,
                "autorepeat_rate": 20.0,
                "coalesce_interval_ms": 1,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[xbox_config],
            remappings=[("cmd_vel", "/cmd_vel_joy")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        # Print the Y/A-ready message only after every lifecycle component has
        # had time to configure and activate. Manual LB teleoperation remains
        # available through teleop_twist_joy while this guard starts.
        TimerAction(
            period=2.0 if auto_start else 10.0,
            actions=[Node(
                package="roka_nav2_bringup",
                executable="xbox_autonomy_controls",
                name="xbox_autonomy_controls",
                output="screen",
                parameters=[{
                    "manual_command_topic": "/cmd_vel_joy",
                    "allow_autonomy_start_controls": not external_start_control,
                    "track_stop_service_enabled": True,
                    "mission5_stop_service_enabled": False,
                }],
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
            )],
        ),
    ])
    return actions


def _select_mode(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode == "mapping":
        return _mapping_mode(context)
    if mode in {"autonomy", "autonomy_latest"}:
        return _autonomy_latest_mode(context)
    launch.logging.get_logger("pk3a_field").error(
        f"Unsupported PK-3A field mode: {mode}"
    )
    return [
        EmitEvent(event=Shutdown(reason="mode must be mapping or autonomy")),
    ]


def generate_launch_description():
    hardware = load_hardware_config()
    return LaunchDescription([
        DeclareLaunchArgument("ros_domain_id", default_value="31"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_configure_launch_logging),
        OpaqueFunction(function=_reject_concurrent_field_stack),
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument(
            "environment",
            default_value="flat",
            description=(
                "Mapping mode: capture (raw sensors only), flat (legacy live 2D) "
                "or inline (legacy live 3D)."
            ),
        ),
        DeclareLaunchArgument(
            "map_root", default_value=str(workspace_path("maps"))
        ),
        DeclareLaunchArgument(
            "map_id",
            default_value="auto",
            description=(
                "Named map bundle such as section_1..section_5; auto keeps the "
                "legacy next/latest selection."
            ),
        ),
        DeclareLaunchArgument("map_bundle_directory", default_value="auto"),
        DeclareLaunchArgument(
            "overwrite_existing",
            default_value="true",
            description=(
                "For map_id section_1..section_5 only, atomically replace the "
                "existing bundle after the new staging map passes validation."
            ),
        ),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("md_max_linear_speed", default_value="0.60"),
        DeclareLaunchArgument("md_max_wheel_rpm", default_value="56.0"),
        DeclareLaunchArgument(
            "md_feedback_overspeed_limit_rpm", default_value="90.0"
        ),
        DeclareLaunchArgument(
            "hold_arm",
            default_value="true",
            description="Recover to and torque-hold the section pose during base motion.",
        ),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "forward_display_fov_deg",
            default_value="120.0",
            description=(
                "Visualization-only forward sector in degrees; FAST-LIO/SC-PGO "
                "always retain the full VLP-16 cloud."
            ),
        ),
        DeclareLaunchArgument(
            "localization_backend",
            default_value="auto",
            description="Autonomy localization: auto, fastlio_map or legacy",
        ),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument(
            "use_imu_ekf",
            default_value="true",
            description=(
                "Legacy backend only: fuse wheel odometry and EBIMU. "
                "fastlio_map uses FAST-LIO2 as the sole odom->base_link owner."
            ),
        ),
        DeclareLaunchArgument("maximum_start_distance_m", default_value="0.75"),
        DeclareLaunchArgument(
            "initialize_from_profile_start",
            default_value="true",
            description=(
                "After returning the robot to its marked start, seed the active "
                "localizer from the recorded first point with broad covariance."
            ),
        ),
        DeclareLaunchArgument(
            "maximum_start_heading_error_deg", default_value="90.0"
        ),
        DeclareLaunchArgument(
            "test_mode",
            default_value="false",
            description=(
                "Use the separately generated track_profile_test.yaml; this never "
                "authorizes the normal track profile."
            ),
        ),
        DeclareLaunchArgument(
            "recorded_route_mode",
            default_value="false",
            description=(
                "Follow track_profile_recorded.yaml without terrain or obstacle "
                "cost decisions; localization and route-deviation checks remain."
            ),
        ),
        DeclareLaunchArgument(
            "competition_track_mode",
            default_value="false",
            description=(
                "Competition pump-track mode: recorded line is primary, track "
                "slope is not an obstacle; wheel-contact diagnostics remain visible "
                "but automatic slip derating stays disabled."
            ),
        ),
        DeclareLaunchArgument(
            "autonomy_auto_start",
            default_value="true",
            description=(
                "Start an approved path automatically after localization is ready; "
                "false restores optional Xbox Y/A start control."
            ),
        ),
        DeclareLaunchArgument(
            "autonomy_optional_xbox",
            default_value="true",
            description=(
                "Auto start never requires Xbox; when the ROKA controller is "
                "present, add B cancel and LB manual takeover."
            ),
        ),
        DeclareLaunchArgument(
            "autonomy_external_start_control",
            default_value="false",
            description=(
                "Keep the route stopped and suppress Xbox Y/A controls until an "
                "external mission controller calls /track_path_follower/start."
            ),
        ),
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
            "hardware_config", default_value=str(default_hardware_config_path())
        ),
        DeclareLaunchArgument(
            "wheel_separation", default_value=str(hardware.wheel_separation_m)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_arm"), "launch", "arm_hold.launch.py",
            ])),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level"),
                "start_pose_name": LaunchConfiguration("map_id"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("hold_arm")),
        ),
        Node(
            package="roka_arm",
            executable="roka_arm_hold_gate",
            name="autonomy_arm_hold_cmd_gate",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_topic": "/cmd_vel",
                "output_topic": "/cmd_vel_arm_locked",
                "require_field_preflight": True,
            }],
            arguments=[
                "--ros-args", "--log-level", LaunchConfiguration("log_level")
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("mode"), "' == 'autonomy'"
            ])),
        ),
        OpaqueFunction(function=_select_mode),
    ])
