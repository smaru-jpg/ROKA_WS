from pathlib import Path

import launch.logging
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import yaml

from roka_nav2_bringup.field_session_guard import other_field_launches
from roka_nav2_bringup.profile_validation import is_profile_allowed
from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_path


def _reject_concurrent_field_stack(_context):
    running = other_field_launches()
    if not running:
        return []
    details = "; ".join(f"pid={pid} {command}" for pid, command in running)
    launch.logging.get_logger("lidar_md_navigation").error(
        f"Another ROKA field launch is already running: {details}"
    )
    return [
        EmitEvent(event=Shutdown(reason="only one ROKA field stack may run")),
    ]


def _include_nav2(context):
    map_yaml = Path(LaunchConfiguration("map").perform(context)).expanduser()
    if not map_yaml.is_file():
        return [
            LogInfo(msg=f"[ERROR] Nav2 map YAML does not exist: {map_yaml}"),
            EmitEvent(event=Shutdown(reason="Nav2 map YAML is missing")),
        ]
    try:
        package_share = Path(get_package_share_directory("nav2_bringup"))
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "[ERROR] Nav2 is not installed. Install ros-humble-navigation2 and "
                    "ros-humble-nav2-bringup, then launch this file again."
                )
            ),
            EmitEvent(event=Shutdown(reason="Nav2 is missing")),
        ]
    track_mode = LaunchConfiguration("use_track_follower").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    direct_recorded_route = LaunchConfiguration("direct_recorded_route").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    dry_run = LaunchConfiguration("use_dry_run_map_tf").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    localization_backend = LaunchConfiguration("localization_backend").perform(
        context
    ).strip().lower()
    if localization_backend not in {"fastlio_map", "legacy"}:
        return [
            LogInfo(msg=f"[ERROR] Unsupported localization_backend={localization_backend}"),
            EmitEvent(event=Shutdown(reason="invalid localization backend")),
        ]
    if dry_run or localization_backend == "fastlio_map":
        # FAST-LIO fixed-map mode owns map->odom in roka_map_localization.
        # Dry-run syntax validation deliberately starts no localization owner.
        localization = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file"), {
                    "yaml_filename": str(map_yaml),
                }],
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "node_names": ["map_server"],
                }],
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
            ),
        ]
    else:
        localization = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_share / "launch" / "localization_launch.py")
            ),
            launch_arguments={
                "map": str(map_yaml),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": LaunchConfiguration("autostart"),
                "use_composition": "False",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        )]
    if not track_mode:
        return localization
    controller_parameters = LaunchConfiguration("params_file")
    if direct_recorded_route:
        try:
            profile_document = yaml.safe_load(
                Path(LaunchConfiguration("track_profile").perform(context)).expanduser().read_text(
                    encoding="utf-8"
                )
            ) or {}
            replay_speed = float(profile_document["default_speed_mps"])
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            return [
                LogInfo(msg=f"[ERROR] Recorded replay speed is invalid: {error}"),
                EmitEvent(event=Shutdown(reason="recorded replay speed is invalid")),
            ]
        if not 0.03 <= replay_speed <= 0.60:
            return [
                LogInfo(msg=f"[ERROR] Recorded replay speed is out of range: {replay_speed}"),
                EmitEvent(event=Shutdown(reason="recorded replay speed is out of range")),
            ]
        # AMCL continues to consume the live VLP-16 scan for localization.
        # Only the local obstacle marking and RPP collision/cost decisions are
        # disabled; the controller follows the recorded geometry at one speed.
        controller_parameters = RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            param_rewrites={
                "enabled": "false",
                "use_collision_detection": "false",
                "use_cost_regulated_linear_velocity_scaling": "false",
                "desired_linear_vel": str(replay_speed),
                "regulated_linear_scaling_min_speed": str(replay_speed),
                "xy_goal_tolerance": "0.08",
                "yaw_goal_tolerance": "0.174533",
            },
            convert_types=True,
        )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[controller_parameters],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        remappings=[("cmd_vel", "/cmd_vel_nav_smoothed")],
    )
    controller_manager = Node(
        package="roka_terrain_supervisor",
        executable="roka_lifecycle_activator",
        name="controller_server_lifecycle_activator",
        output="screen",
        parameters=[{
            "target_node": "/controller_server",
            "request_timeout_sec": 8.0,
            "retry_period_sec": 1.0,
        }],
        condition=IfCondition(LaunchConfiguration("autostart")),
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    # Controller plugin/costmap construction can exceed LifecycleManager's
    # short transition deadline on the field PC. The retrying activator waits
    # for the real state and never treats one late response as a permanent
    # inactive controller.
    delayed_controller_manager = TimerAction(
        period=4.0,
        actions=[controller_manager],
    )
    return [*localization, controller, delayed_controller_manager]


def _include_base(context):
    """Select the TF/odometry owner without creating competing transforms."""
    use_imu = LaunchConfiguration("use_imu_ekf").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_lio = LaunchConfiguration("localization_backend").perform(context).strip().lower() == (
        "fastlio_map"
    )
    three_dimensional_owner = use_imu or use_lio
    arguments = {
        "use_lidar_driver": LaunchConfiguration("use_lidar_driver"),
        "lidar_package": LaunchConfiguration("lidar_package"),
        "lidar_launch_file": LaunchConfiguration("lidar_launch_file"),
        "use_md_driver": LaunchConfiguration("use_md_driver"),
        "md_config": LaunchConfiguration("md_config"),
        "md_simulation_mode": LaunchConfiguration("md_simulation_mode"),
        "md_allow_motion": LaunchConfiguration("md_allow_motion"),
        "md_max_linear_speed": LaunchConfiguration("md_max_linear_speed"),
        "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
        "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
            "md_feedback_overspeed_limit_rpm"
        ),
        "md_cmd_vel_topic": LaunchConfiguration("md_cmd_vel_topic"),
        "left_serial_port": LaunchConfiguration("left_serial_port"),
        "right_serial_port": LaunchConfiguration("right_serial_port"),
        "wheel_separation": LaunchConfiguration("wheel_separation"),
        "md_odom_topic": (
            "/wheel/odom"
            if three_dimensional_owner
            else LaunchConfiguration("md_odom_topic")
        ),
        "md_base_frame": (
            "base_link"
            if three_dimensional_owner
            else LaunchConfiguration("md_base_frame")
        ),
        "md_publish_tf": (
            "false"
            if three_dimensional_owner
            else LaunchConfiguration("md_publish_tf")
        ),
        "base_frame": (
            "base_link"
            if three_dimensional_owner
            else LaunchConfiguration("base_frame")
        ),
        "lidar_x": LaunchConfiguration("lidar_x"),
        "lidar_y": LaunchConfiguration("lidar_y"),
        "lidar_z": (
            LaunchConfiguration("lidar_z_base_link")
            if three_dimensional_owner
            else LaunchConfiguration("lidar_z")
        ),
        "lidar_roll": LaunchConfiguration("lidar_roll"),
        "lidar_pitch": LaunchConfiguration("lidar_pitch"),
        "lidar_yaw": LaunchConfiguration("lidar_yaw"),
        "use_lidar_static_tf": LaunchConfiguration("use_lidar_static_tf"),
        "use_stabilized_scan": LaunchConfiguration("use_stabilized_scan"),
        "log_level": LaunchConfiguration("log_level"),
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("roka_nav2_bringup"), "launch", "lidar_md_base.launch.py"]
            )
        ),
        launch_arguments=arguments.items(),
    )]


def _include_raw_imu(context):
    hardware = load_hardware_config()
    use_driver = LaunchConfiguration("use_imu_driver").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_ekf = LaunchConfiguration("use_imu_ekf").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_lio = LaunchConfiguration("localization_backend").perform(context).strip().lower() == (
        "fastlio_map"
    )
    simulation = LaunchConfiguration("md_simulation_mode").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if not use_driver or use_ekf or simulation:
        return []
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_imu_driver"), "launch", "ebimu.launch.py"
            ])),
            launch_arguments={
                "serial_port": LaunchConfiguration("imu_serial_port"),
                "allow_sensor_configuration": LaunchConfiguration(
                    "imu_allow_sensor_configuration"
                ),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_tf",
            output="screen",
            arguments=[
                "--x", str(hardware.base_to_imu.xyz[0]),
                "--y", str(hardware.base_to_imu.xyz[1]),
                "--z", str(
                    hardware.base_to_imu.xyz[2]
                    if use_lio
                    else hardware.footprint_to_base.xyz[2] + hardware.base_to_imu.xyz[2]
                ),
                "--roll", str(hardware.base_to_imu.rpy[0]),
                "--pitch", str(hardware.base_to_imu.rpy[1]),
                "--yaw", str(hardware.base_to_imu.rpy[2]),
                "--frame-id", hardware.body_3d_frame if use_lio else hardware.body_2d_frame,
                "--child-frame-id", hardware.imu_frame,
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
        ),
    ]


def _include_lio_localization(context):
    backend = LaunchConfiguration("localization_backend").perform(context).strip().lower()
    if backend != "fastlio_map":
        return []
    map_pcd = Path(LaunchConfiguration("map_pcd").perform(context)).expanduser().resolve()
    initial_pose_value = LaunchConfiguration("initial_pose_file").perform(context).strip()
    initial_pose_file = (
        Path(initial_pose_value).expanduser().resolve() if initial_pose_value else None
    )
    if not map_pcd.is_file():
        return [
            LogInfo(msg=f"[ERROR] Fixed localization PCD does not exist: {map_pcd}"),
            EmitEvent(event=Shutdown(reason="fixed localization PCD is missing")),
        ]
    if initial_pose_file is not None and not initial_pose_file.is_file():
        return [
            LogInfo(msg=f"[ERROR] Optimized path CSV does not exist: {initial_pose_file}"),
            EmitEvent(event=Shutdown(reason="fixed localization initial pose is missing")),
        ]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_fast_lio_localization"), "launch",
                "fastlio_sc_pgo_mapping.launch.py",
            ])),
            launch_arguments={
                "output_dir": "/tmp/roka_lio_localization",
                "use_sc_pgo": "false",
                "publish_lio_tf": "true",
                "world_frame": "odom",
                "body_frame": "base_link",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_map_localization"), "launch",
                "fixed_map_localization.launch.py",
            ])),
            launch_arguments={
                "map_pcd": str(map_pcd),
                "initial_pose_file": str(initial_pose_file) if initial_pose_file else "",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_tf": "true",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("roka_nav_utils"), "launch", "tf_flatten_2d.launch.py",
            ])),
            launch_arguments={"log_level": LaunchConfiguration("log_level")}.items(),
        ),
    ]


def _include_track_follower(context):
    enabled = LaunchConfiguration("use_track_follower").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return []
    profile = Path(LaunchConfiguration("track_profile").perform(context)).expanduser()
    try:
        document = yaml.safe_load(profile.read_text(encoding="utf-8")) or {}
        point_count = len(document.get("points", []))
    except (OSError, yaml.YAMLError, TypeError) as error:
        return [
            LogInfo(msg=f"[ERROR] Track profile cannot be loaded: {profile}: {error}"),
            EmitEvent(event=Shutdown(reason="track profile is invalid")),
        ]
    if point_count < 2:
        return [
            LogInfo(msg=f"[ERROR] Track profile has fewer than two points: {profile}"),
            EmitEvent(event=Shutdown(reason="track profile is empty")),
        ]
    allow_test_profile = LaunchConfiguration("allow_test_profile").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    direct_recorded_route = LaunchConfiguration("direct_recorded_route").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    preserve_recorded_geometry = bool(
        document.get("preserve_recorded_geometry", False)
    )
    if not is_profile_allowed(document, allow_test_profile=allow_test_profile):
        return [
            LogInfo(msg=f"[ERROR] Track profile approval scope is not allowed: {profile}"),
            EmitEvent(event=Shutdown(reason="track profile approval scope is rejected")),
        ]
    return [
        Node(
            package="roka_nav2_bringup",
            executable="track_path_follower",
            name="track_path_follower",
            output="screen",
            parameters=[{
                "profile_file": str(profile),
                "auto_start": LaunchConfiguration("track_auto_start"),
                "loop": LaunchConfiguration("track_loop"),
                "allow_test_profile": LaunchConfiguration("allow_test_profile"),
                "operation_profile": LaunchConfiguration("operation_profile"),
                "maximum_start_distance_m": ParameterValue(
                    LaunchConfiguration("maximum_start_distance_m"), value_type=float
                ),
                "maximum_start_heading_error_deg": ParameterValue(
                    LaunchConfiguration("maximum_start_heading_error_deg"), value_type=float
                ),
                "require_field_preflight": ParameterValue(
                    LaunchConfiguration("require_field_preflight"), value_type=bool
                ),
                "preserve_recorded_geometry": (
                    direct_recorded_route and preserve_recorded_geometry
                ),
                "require_join_occupancy_clear": not direct_recorded_route,
                "enforce_join_corridor": not direct_recorded_route,
                "join_maximum_lookahead_m": (
                    2.50 if direct_recorded_route else 1.50
                ),
                "corridor_reference_mode": (
                    "vehicle_half_width"
                    if direct_recorded_route
                    else "track_remaining_width"
                ),
                "corridor_safety_margin_m": 0.0 if direct_recorded_route else 0.01,
                "corridor_warning_ratio": 0.70 if direct_recorded_route else 0.65,
                "corridor_recovery_ratio": 0.85 if direct_recorded_route else 0.80,
                "corridor_hard_ratio": 0.90 if direct_recorded_route else 1.00,
                "open_endpoint_tolerance_m": 0.08 if direct_recorded_route else 0.30,
                "open_completion_heading_deg": (
                    10.0 if direct_recorded_route else 180.0
                ),
                "required_active_lifecycle_nodes": (
                    ["/controller_server", "/terrain_velocity_smoother"]
                    if direct_recorded_route
                    else [
                        "/controller_server",
                        "/terrain_velocity_smoother",
                        "/collision_monitor",
                    ]
                ),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="roka_nav2_bringup",
            executable="autonomy_command_pipeline",
            name="autonomy_command_pipeline",
            output="screen",
            parameters=[{
                "obstacle_max_age_sec": ParameterValue(
                    LaunchConfiguration("collision_source_timeout"), value_type=float
                ),
                "direct_recorded_route": direct_recorded_route,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ]


def generate_launch_description():
    hardware = load_hardware_config()
    footprint_lidar_z = hardware.footprint_to_base.xyz[2] + hardware.base_to_lidar.xyz[2]
    return LaunchDescription(
        [
            OpaqueFunction(function=_reject_concurrent_field_stack),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Absolute path to a map YAML created by map_saver_cli.",
            ),
            DeclareLaunchArgument(
                "localization_backend",
                default_value="legacy",
                description="fastlio_map or legacy (AMCL)",
            ),
            DeclareLaunchArgument("map_pcd", default_value=""),
            DeclareLaunchArgument("initial_pose_file", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roka_nav2_bringup"), "config", "lidar_md_nav2.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_terrain_mapping", default_value="true"),
            DeclareLaunchArgument("use_safety_pipeline", default_value="true"),
            DeclareLaunchArgument("use_collision_monitor", default_value="true"),
            DeclareLaunchArgument(
                "direct_recorded_route",
                default_value="false",
                description=(
                    "Keep localization and route tracking, but bypass terrain and "
                    "obstacle decisions for an operator-approved clear track."
                ),
            ),
            DeclareLaunchArgument("collision_source_timeout", default_value="0.40"),
            DeclareLaunchArgument("supervisor_require_terrain", default_value="true"),
            DeclareLaunchArgument(
                "supervisor_enable_terrain_decisions", default_value="true"
            ),
            DeclareLaunchArgument(
                "supervisor_allow_level_fallback", default_value="false"
            ),
            DeclareLaunchArgument("supervisor_require_imu", default_value="false"),
            DeclareLaunchArgument("supervisor_require_odom", default_value="false"),
            DeclareLaunchArgument(
                "supervisor_enable_slip_detection", default_value="false"
            ),
            DeclareLaunchArgument(
                "supervisor_use_contact_estimator", default_value="true"
            ),
            DeclareLaunchArgument(
                "supervisor_derive_body_twist_from_pose", default_value="false"
            ),
            DeclareLaunchArgument(
                "supervisor_body_odom_topic", default_value="/odom"
            ),
            DeclareLaunchArgument(
                "supervisor_wheel_odom_topic", default_value="/wheel/odom"
            ),
            DeclareLaunchArgument("use_static_terrain_map", default_value="false"),
            DeclareLaunchArgument("terrain_bag_path", default_value=""),
            DeclareLaunchArgument("use_track_follower", default_value="false"),
            DeclareLaunchArgument(
                "track_profile",
                default_value=str(workspace_path("maps", "track_v1", "track_profile.yaml")),
            ),
            DeclareLaunchArgument("track_auto_start", default_value="false"),
            DeclareLaunchArgument("require_field_preflight", default_value="false"),
            DeclareLaunchArgument(
                "initialize_from_profile_start",
                default_value="false",
                description=(
                    "Seed AMCL from the first recorded path point after the robot "
                    "is physically returned near its marked start."
                ),
            ),
            DeclareLaunchArgument(
                "initializer_position_stddev_m", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "initializer_yaw_stddev_deg", default_value="60.0"
            ),
            DeclareLaunchArgument("track_loop", default_value="true"),
            DeclareLaunchArgument("maximum_start_distance_m", default_value="0.75"),
            DeclareLaunchArgument(
                "maximum_start_heading_error_deg", default_value="90.0"
            ),
            DeclareLaunchArgument(
                "allow_test_profile",
                default_value="false",
                description="Explicitly permit a controlled test-only path.",
            ),
            DeclareLaunchArgument(
                "operation_profile",
                default_value="inline_track",
                description="flat_commissioning or inline_track safety/route profile.",
            ),
            DeclareLaunchArgument(
                "use_dry_run_map_tf",
                default_value="false",
                description=(
                    "Publish identity map->odom only for no-LiDAR launch validation. "
                    "Never enable with AMCL receiving real scans."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("roka_nav2_bringup"),
                        "rviz",
                        "lidar_md_navigation.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument("use_lidar_driver", default_value="true"),
            DeclareLaunchArgument("lidar_package", default_value="velodyne"),
            DeclareLaunchArgument(
                "lidar_launch_file", default_value="velodyne-all-nodes-VLP16-launch.py"
            ),
            DeclareLaunchArgument("use_md_driver", default_value="true"),
            DeclareLaunchArgument(
                "use_imu_driver",
                default_value="true",
                description="Run EBIMU on real hardware even when EKF fusion is disabled.",
            ),
            DeclareLaunchArgument(
                "use_imu_ekf",
                default_value="false",
                description=(
                    "Fuse EBIMU-9DOFV5 with wheel odometry and publish the planar "
                    "base_footprint required by Nav2."
                ),
            ),
            DeclareLaunchArgument(
                "imu_serial_port",
                default_value="auto",
            ),
            DeclareLaunchArgument("imu_allow_sensor_configuration", default_value="false"),
            DeclareLaunchArgument(
                "md_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("roka_md_driver"),
                        "config",
                        "mdh250_md200t_four_wheel.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("md_simulation_mode", default_value="true"),
            DeclareLaunchArgument("md_allow_motion", default_value="false"),
            DeclareLaunchArgument("md_max_linear_speed", default_value="0.60"),
            DeclareLaunchArgument("md_max_wheel_rpm", default_value="56.0"),
            DeclareLaunchArgument(
                "md_feedback_overspeed_limit_rpm", default_value="90.0"
            ),
            DeclareLaunchArgument("md_cmd_vel_topic", default_value="/cmd_vel"),
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
                "wheel_separation", default_value=str(hardware.wheel_separation_m)
            ),
            DeclareLaunchArgument("md_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("md_base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("md_publish_tf", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("lidar_x", default_value=str(hardware.base_to_lidar.xyz[0])),
            DeclareLaunchArgument("lidar_y", default_value=str(hardware.base_to_lidar.xyz[1])),
            DeclareLaunchArgument("lidar_z", default_value=str(footprint_lidar_z)),
            DeclareLaunchArgument(
                "lidar_z_base_link", default_value=str(hardware.base_to_lidar.xyz[2])
            ),
            DeclareLaunchArgument("lidar_roll", default_value=str(hardware.base_to_lidar.rpy[0])),
            DeclareLaunchArgument("lidar_pitch", default_value=str(hardware.base_to_lidar.rpy[1])),
            DeclareLaunchArgument("lidar_yaw", default_value=str(hardware.base_to_lidar.rpy[2])),
            DeclareLaunchArgument("use_lidar_static_tf", default_value="true"),
            DeclareLaunchArgument("use_stabilized_scan", default_value="true"),
            OpaqueFunction(function=_include_base),
            OpaqueFunction(function=_include_raw_imu),
            OpaqueFunction(function=_include_lio_localization),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("roka_imu_driver"),
                        "launch",
                        "ebimu_wheel_ekf.launch.py",
                    ])
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("imu_serial_port"),
                    "allow_sensor_configuration": LaunchConfiguration(
                        "imu_allow_sensor_configuration"
                    ),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_imu_ekf")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("roka_nav_utils"),
                        "launch",
                        "tf_flatten_2d.launch.py",
                    ])
                ),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                    "controlled_level_test": LaunchConfiguration(
                        "allow_test_profile"
                    ),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_imu_ekf")),
            ),
            OpaqueFunction(function=_include_nav2),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("roka_terrain_mapping"),
                            "launch",
                            "terrain_mapping.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level")
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_terrain_mapping")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("roka_terrain_supervisor"),
                            "launch",
                            "safety_pipeline.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_collision_monitor": LaunchConfiguration("use_collision_monitor"),
                    "direct_recorded_route": LaunchConfiguration(
                        "direct_recorded_route"
                    ),
                    "collision_source_timeout": LaunchConfiguration(
                        "collision_source_timeout"
                    ),
                    "require_terrain": LaunchConfiguration(
                        "supervisor_require_terrain"
                    ),
                    "enable_terrain_decisions": LaunchConfiguration(
                        "supervisor_enable_terrain_decisions"
                    ),
                    "allow_level_fallback": LaunchConfiguration(
                        "supervisor_allow_level_fallback"
                    ),
                    "require_imu": LaunchConfiguration("supervisor_require_imu"),
                    "require_odom": LaunchConfiguration("supervisor_require_odom"),
                    "enable_slip_detection": LaunchConfiguration(
                        "supervisor_enable_slip_detection"
                    ),
                    "use_contact_estimator": LaunchConfiguration(
                        "supervisor_use_contact_estimator"
                    ),
                    "derive_body_twist_from_pose": LaunchConfiguration(
                        "supervisor_derive_body_twist_from_pose"
                    ),
                    "body_odom_topic": LaunchConfiguration(
                        "supervisor_body_odom_topic"
                    ),
                    "wheel_odom_topic": LaunchConfiguration(
                        "supervisor_wheel_odom_topic"
                    ),
                    "physical_wheel_track": str(hardware.physical_wheel_track_m),
                    "wheel_radius": str(hardware.wheel_radius_m),
                    "footprint_length": str(hardware.navigation_length_m),
                    "footprint_width": str(hardware.navigation_width_m),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_safety_pipeline")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("roka_terrain_mapping"),
                            "launch",
                            "terrain_map_server.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "bag_path": LaunchConfiguration("terrain_bag_path"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_static_terrain_map")),
            ),
            OpaqueFunction(function=_include_track_follower),
            Node(
                package="roka_nav2_bringup",
                executable="profile_start_initializer",
                name="profile_start_initializer",
                output="screen",
                parameters=[{
                    "profile_file": LaunchConfiguration("track_profile"),
                    # The operator is not expected to reproduce an exact line,
                    # heading, or 90-degree turn. AMCL receives a broad seed and
                    # must converge against the live scan before A can start.
                    "position_stddev_m": ParameterValue(
                        LaunchConfiguration("initializer_position_stddev_m"),
                        value_type=float,
                    ),
                    # Start above the follower's 45-degree uncertainty limit;
                    # the A request remains blocked until live AMCL scans
                    # actually reduce the uncertainty.
                    "yaw_stddev_deg": ParameterValue(
                        LaunchConfiguration("initializer_yaw_stddev_deg"),
                        value_type=float,
                    ),
                    "require_scan": True,
                }],
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                condition=IfCondition(
                    LaunchConfiguration("initialize_from_profile_start")
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="dry_run_map_to_odom_tf",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "map", "--child-frame-id", "odom",
                    "--ros-args", "--log-level", LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(LaunchConfiguration("use_dry_run_map_tf")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="lidar_md_navigation_rviz",
                output="screen",
                arguments=[
                    "-d", LaunchConfiguration("rviz_config"),
                    "--ros-args", "--log-level", LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
