import logging

import launch.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from roka_md_driver.xbox_identity import (
    resolve_assigned_sdl_device_id,
)
from roka_robot_description.hardware_config import (
    default_hardware_config_path,
    load_hardware_config,
)
from roka_robot_description.workspace import workspace_path
from roka_nav2_bringup.field_session_guard import other_field_launches


def _configure_launch_logging(_context):
    launch.logging.launch_config.level = logging.INFO
    return []


def _reject_concurrent_field_stack(_context):
    running = other_field_launches()
    if not running:
        return []
    details = "; ".join(f"pid={pid} {command}" for pid, command in running)
    launch.logging.get_logger("mapping_manual").error(
        f"Another ROKA field launch is already running: {details}"
    )
    return [
        EmitEvent(event=Shutdown(reason="only one ROKA field stack may run")),
    ]


def _select_roka_xbox(context):
    enabled = LaunchConfiguration("use_xbox_mapping_controls").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        context.launch_configurations["joy_available"] = "false"
        return []
    try:
        device_id = resolve_assigned_sdl_device_id(
            LaunchConfiguration("joy_device_id").perform(context),
        )
    except RuntimeError as error:
        # Keep the remaining hardware alive so the one-shot field check can
        # classify every device.  No joy/teleop node means no manual command.
        launch.logging.get_logger("mapping_manual").error(
            f"[FIELD-CHECK][MISSING] Xbox: {error}"
        )
        context.launch_configurations["joy_device_id"] = "0"
        context.launch_configurations["joy_available"] = "false"
        return []
    context.launch_configurations["joy_device_id"] = str(device_id)
    context.launch_configurations["joy_available"] = "true"
    return []


def _select_mapping_rviz(context):
    configured = LaunchConfiguration("rviz_config").perform(context).strip()
    if configured and configured.lower() != "auto":
        return []
    use_3d = LaunchConfiguration("use_3d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_2d = LaunchConfiguration("use_2d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    filename = (
        "track_mapping_3d.rviz" if use_3d
        else "mapping_growth_2d.rviz" if use_2d
        else "live_sensor_check.rviz"
    )
    selected = PathJoinSubstitution([
        FindPackageShare("roka_nav2_bringup"), "rviz", filename
    ]).perform(context)
    context.launch_configurations["rviz_config"] = selected
    launch.logging.get_logger("mapping_manual").info(
        f"Mapping visualization selected: {filename}"
    )
    return []


def _raw_capture_rviz(context):
    enabled = LaunchConfiguration("use_rviz").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_2d = LaunchConfiguration("use_2d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_3d = LaunchConfiguration("use_3d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if not enabled or use_2d or use_3d:
        return []
    return [Node(
        package="rviz2",
        executable="rviz2",
        name="raw_mapping_capture_rviz",
        output="screen",
        arguments=[
            "-d", LaunchConfiguration("rviz_config"),
            "--ros-args", "--log-level", "fatal",
        ],
    )]


def _two_dimensional_mapping(context):
    hardware = load_hardware_config(LaunchConfiguration("hardware_config").perform(context))
    use_2d_slam = LaunchConfiguration("use_2d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_imu = LaunchConfiguration("use_imu_ekf").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    arguments = {
        "use_lidar_driver": LaunchConfiguration("use_lidar_driver"),
        "use_md_driver": LaunchConfiguration("use_md_driver"),
        "md_simulation_mode": LaunchConfiguration("md_simulation_mode"),
        "md_allow_motion": LaunchConfiguration("md_allow_motion"),
        "md_max_linear_speed": LaunchConfiguration("md_max_linear_speed"),
        "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
        "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
            "md_feedback_overspeed_limit_rpm"
        ),
        "left_serial_port": LaunchConfiguration("left_serial_port"),
        "right_serial_port": LaunchConfiguration("right_serial_port"),
        "wheel_separation": str(hardware.wheel_separation_m),
        "md_cmd_vel_topic": LaunchConfiguration("md_cmd_vel_topic"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "md_odom_topic": "/wheel/odom" if use_imu else "/odom",
        "md_base_frame": "base_link" if use_imu else "base_footprint",
        "md_publish_tf": "false" if use_imu else "true",
        "base_frame": hardware.body_3d_frame if use_imu else hardware.body_2d_frame,
        "rviz_config": LaunchConfiguration("rviz_config"),
        "lidar_x": str(hardware.base_to_lidar.xyz[0]),
        "lidar_y": str(hardware.base_to_lidar.xyz[1]),
        "lidar_z": str(
            hardware.base_to_lidar.xyz[2]
            if use_imu
            else hardware.footprint_to_base.xyz[2] + hardware.base_to_lidar.xyz[2]
        ),
        "lidar_roll": str(hardware.base_to_lidar.rpy[0]),
        "lidar_pitch": str(hardware.base_to_lidar.rpy[1]),
        "lidar_yaw": str(hardware.base_to_lidar.rpy[2]),
        "log_level": LaunchConfiguration("log_level"),
    }
    launch_file = "lidar_md_slam.launch.py" if use_2d_slam else "lidar_md_base.launch.py"
    if not use_2d_slam:
        arguments.pop("rviz_config", None)
        arguments.pop("use_rviz", None)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("roka_nav2_bringup"),
                "launch",
                launch_file,
            ])
        ),
        launch_arguments=arguments.items(),
    )]


def _three_dimensional_mapping(context):
    enabled = LaunchConfiguration("use_3d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if not enabled:
        return []
    backend = LaunchConfiguration("mapping_backend").perform(context).strip().lower()
    if backend != "fastlio_sc_pgo":
        launch.logging.get_logger("mapping_manual").error(
            f"Unsupported 3D mapping_backend={backend}; use fastlio_sc_pgo"
        )
        return [EmitEvent(event=Shutdown(reason="invalid 3D mapping backend"))]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("roka_fast_lio_localization"),
                "launch",
                "fastlio_sc_pgo_mapping.launch.py",
            ])
        ),
        launch_arguments={
            "output_dir": LaunchConfiguration("map_bundle_directory"),
            "hardware_config": LaunchConfiguration("hardware_config"),
            "publish_lio_tf": "false",
            # Mapping starts in the same origin as the recorded 3D route.
            "world_frame": "map",
            "use_sim_time": "false",
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )]


def _inertial_sensing(context):
    hardware = load_hardware_config(LaunchConfiguration("hardware_config").perform(context))
    use_driver = LaunchConfiguration("use_imu_driver").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    use_ekf = LaunchConfiguration("use_imu_ekf").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if not use_driver and not use_ekf:
        return []
    common_arguments = {
        "serial_port": LaunchConfiguration("imu_serial_port"),
        "allow_sensor_configuration": LaunchConfiguration(
            "imu_allow_sensor_configuration"
        ),
        "log_level": LaunchConfiguration("log_level"),
    }
    if use_ekf:
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_imu_driver"),
                    "launch",
                    "ebimu_wheel_ekf.launch.py",
                ])
            ),
            launch_arguments=common_arguments.items(),
        )]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_imu_driver"),
                    "launch",
                    "ebimu.launch.py",
                ])
            ),
            launch_arguments=common_arguments.items(),
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
                    hardware.footprint_to_base.xyz[2] + hardware.base_to_imu.xyz[2]
                ),
                "--roll", str(hardware.base_to_imu.rpy[0]),
                "--pitch", str(hardware.base_to_imu.rpy[1]),
                "--yaw", str(hardware.base_to_imu.rpy[2]),
                "--frame-id", hardware.body_2d_frame,
                "--child-frame-id", hardware.imu_frame,
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
        ),
    ]


def _mapping_contact_diagnostics(context):
    """Record wheel/contact evidence without changing the manual drive command."""
    enabled = LaunchConfiguration("use_contact_diagnostics").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return []
    safety_pipeline = LaunchConfiguration("use_safety_pipeline").perform(context).lower()
    if safety_pipeline in {"1", "true", "yes", "on"}:
        # safety_pipeline.launch.py already owns this publisher.
        return []
    hardware = load_hardware_config(LaunchConfiguration("hardware_config").perform(context))
    return [Node(
        package="roka_terrain_supervisor",
        executable="wheel_contact_estimator_node",
        name="mapping_wheel_contact_estimator",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("roka_terrain_supervisor"),
                "config",
                "terrain_supervisor.yaml",
            ]),
            PathJoinSubstitution([
                FindPackageShare("roka_terrain_supervisor"),
                "config",
                "provisional_vehicle_limits.yaml",
            ]),
            {
                # Acquisition has wheel+IMU EKF odometry but no live FAST-LIO.
                # This makes the common-mode slip ratio non-independent; the
                # useful evidence here is IMU plus four RPM/current channels.
                "body_odom_topic": "/odom",
                "wheel_odom_topic": "/wheel/odom",
                "derive_body_twist_from_pose": False,
                "contact_state_topic": "/terrain/wheel_contact",
                "wheel_track": hardware.physical_wheel_track_m,
                "wheel_radius": hardware.wheel_radius_m,
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )]


def _field_preflight(context):
    enabled = LaunchConfiguration("require_preflight").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return []
    use_3d = LaunchConfiguration("use_3d_slam").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    backend = LaunchConfiguration("mapping_backend").perform(context).strip().lower()
    require_lio_stability = use_3d and backend == "fastlio_sc_pgo"
    require_xbox = LaunchConfiguration("use_xbox_mapping_controls").perform(
        context
    ).lower() in {"1", "true", "yes", "on"}
    return [Node(
        package="roka_nav2_bringup",
        executable="pk3a_field_preflight",
        name="pk3a_field_preflight",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "duration_sec": ParameterValue(
                LaunchConfiguration("startup_check_duration_sec"),
                value_type=float,
            ),
            "left_serial_port": LaunchConfiguration("left_serial_port"),
            "right_serial_port": LaunchConfiguration("right_serial_port"),
            "require_xbox": require_xbox,
            "require_arm_hold": True,
            "require_lio_stability": require_lio_stability,
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )]


def generate_launch_description():
    hardware = load_hardware_config()
    bundle = LaunchConfiguration("map_bundle_directory")
    mapping_recorder = Node(
        package="roka_nav2_bringup",
        executable="mapping_session_recorder",
        name="mapping_session_recorder",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "directory": bundle,
            "environment": LaunchConfiguration("environment"),
            "maximum_total_bytes": ParameterValue(
                LaunchConfiguration("maximum_bag_bytes"), value_type=int
            ),
            "maximum_duration_sec": ParameterValue(
                LaunchConfiguration("maximum_bag_duration_sec"), value_type=float
            ),
            "minimum_start_free_bytes": ParameterValue(
                LaunchConfiguration("minimum_start_free_bytes"), value_type=int
            ),
            "minimum_runtime_free_bytes": ParameterValue(
                LaunchConfiguration("minimum_runtime_free_bytes"), value_type=int
            ),
            "abort_on_md_disconnect": ParameterValue(
                LaunchConfiguration("abort_recording_on_md_disconnect"),
                value_type=bool,
            ),
            "odometry_topic": LaunchConfiguration("recording_odometry_topic"),
            "mapping_backend": LaunchConfiguration("mapping_backend"),
            "hardware_config": LaunchConfiguration("hardware_config"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    mapping_controls = Node(
        package="roka_nav2_bringup",
        executable="xbox_mapping_controls",
        name="xbox_mapping_controls",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "joy_command_topic": "/cmd_vel_joy_raw",
            "motor_command_topic": LaunchConfiguration("md_cmd_vel_topic"),
            "record_sensor_bag": ParameterValue(
                LaunchConfiguration("record_bag"), value_type=bool
            ),
            "start_3d_session": ParameterValue(
                LaunchConfiguration("use_3d_slam"), value_type=bool
            ),
            "require_preflight": ParameterValue(
                LaunchConfiguration("require_preflight"), value_type=bool
            ),
            # Keep MD/Xbox and the transport-pose arm hold alive after the map
            # bundle has been committed.  The operator may continue manual
            # driving with LB and presses B a second time to stop and exit.
            "exit_after_completion": False,
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("use_xbox_mapping_controls")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_configure_launch_logging),
        OpaqueFunction(function=_reject_concurrent_field_stack),
        DeclareLaunchArgument(
            "map_bundle_directory",
            default_value=str(workspace_path("maps", "track_v1")),
        ),
        DeclareLaunchArgument("final_map_bundle_directory", default_value=""),
        DeclareLaunchArgument("overwrite_existing", default_value="false"),
        DeclareLaunchArgument("use_lidar_driver", default_value="true"),
        DeclareLaunchArgument("use_md_driver", default_value="true"),
        DeclareLaunchArgument("md_simulation_mode", default_value="true"),
        DeclareLaunchArgument("md_allow_motion", default_value="false"),
        DeclareLaunchArgument("md_max_linear_speed", default_value="0.60"),
        DeclareLaunchArgument("md_max_wheel_rpm", default_value="56.0"),
        DeclareLaunchArgument(
            "md_feedback_overspeed_limit_rpm", default_value="90.0"
        ),
        DeclareLaunchArgument("use_imu_driver", default_value="true"),
        DeclareLaunchArgument("use_imu_ekf", default_value="false"),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="auto",
        ),
        DeclareLaunchArgument(
            "imu_allow_sensor_configuration", default_value="false"
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
        # Manual mapping uses the same proven dead-man path as the standalone
        # MD Xbox launch.  Autonomous driving continues to use /cmd_vel through
        # the terrain/collision safety pipeline.
        DeclareLaunchArgument("md_cmd_vel_topic", default_value="/cmd_vel_joy"),
        DeclareLaunchArgument("use_safety_pipeline", default_value="false"),
        DeclareLaunchArgument(
            "use_contact_diagnostics",
            default_value="true",
            description=(
                "Record read-only IMU/RPM/current contact evidence during mapping; "
                "never gates or modifies the Xbox command."
            ),
        ),
        DeclareLaunchArgument("use_terrain_monitor", default_value="true"),
        DeclareLaunchArgument("use_terrain_mapping", default_value="true"),
        DeclareLaunchArgument("use_2d_slam", default_value="true"),
        DeclareLaunchArgument("use_3d_slam", default_value="false"),
        DeclareLaunchArgument("mapping_backend", default_value="slam_toolbox_2d"),
        DeclareLaunchArgument("acquisition_only", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="auto",
        ),
        DeclareLaunchArgument("forward_display_fov_deg", default_value="120.0"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument(
            "abort_recording_on_md_disconnect", default_value="false"
        ),
        DeclareLaunchArgument(
            "recording_odometry_topic",
            default_value="/wheel/odom",
            description="Authoritative motion evidence for acquisition metrics.",
        ),
        DeclareLaunchArgument(
            "maximum_bag_bytes", default_value=str(5 * 1024 ** 3)
        ),
        DeclareLaunchArgument("maximum_bag_duration_sec", default_value="900.0"),
        DeclareLaunchArgument(
            "minimum_start_free_bytes", default_value=str(20 * 1024 ** 3)
        ),
        DeclareLaunchArgument(
            "minimum_runtime_free_bytes", default_value=str(10 * 1024 ** 3)
        ),
        DeclareLaunchArgument("require_preflight", default_value="true"),
        DeclareLaunchArgument("record_track_path", default_value="true"),
        DeclareLaunchArgument("environment", default_value="flat"),
        DeclareLaunchArgument("track_map_frame", default_value="map"),
        DeclareLaunchArgument("track_sample_frame", default_value="map"),
        DeclareLaunchArgument("track_base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("track_width_m", default_value="1.20"),
        DeclareLaunchArgument("track_closure_distance_m", default_value="0.80"),
        DeclareLaunchArgument(
            "track_closure_heading_tolerance_deg", default_value="75.0"
        ),
        DeclareLaunchArgument(
            "track_minimum_closed_loop_length_m", default_value="3.0"
        ),
        DeclareLaunchArgument(
            "track_minimum_closed_loop_area_m2", default_value="0.50"
        ),
        DeclareLaunchArgument("use_xbox_mapping_controls", default_value="true"),
        DeclareLaunchArgument(
            "track_profile",
            default_value=[bundle, "/track_profile.yaml"],
        ),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("joy_available", default_value="true"),
        DeclareLaunchArgument("startup_check_duration_sec", default_value="8.0"),
        OpaqueFunction(function=_select_roka_xbox),
        OpaqueFunction(function=_select_mapping_rviz),
        OpaqueFunction(function=_two_dimensional_mapping),
        OpaqueFunction(function=_three_dimensional_mapping),
        OpaqueFunction(function=_inertial_sensing),
        OpaqueFunction(function=_mapping_contact_diagnostics),
        OpaqueFunction(function=_raw_capture_rviz),
        # A visualization-only forward wedge.  FAST-LIO and SC-PGO still
        # consume the complete 360-degree VLP-16 cloud for observability.
        Node(
            package="roka_map_localization",
            executable="pointcloud_sector_filter",
            name="mapping_front_sector_display",
            output="screen",
            parameters=[{
                "input_topic": "/velodyne_points",
                "output_topic": "/mapping/front_120_points",
                "horizontal_fov_deg": ParameterValue(
                    LaunchConfiguration("forward_display_fov_deg"),
                    value_type=float,
                ),
                # VLP-16 is usable from roughly 0.3 m.  The old 0.9 m display
                # cutoff hid most nearby indoor walls and made the cloud look
                # much smaller than the actual scan.  This filter is visual
                # only; SLAM continues to consume the complete 360 deg cloud.
                "minimum_range_m": 0.35,
                "maximum_range_m": 30.0,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_terrain_mapping"),
                    "launch",
                    "terrain_mapping.launch.py",
                ])
            ),
            launch_arguments={
                "map_bundle_directory": bundle,
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_terrain_mapping")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_terrain_supervisor"),
                    "launch",
                    "safety_pipeline.launch.py",
                ])
            ),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level")
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_safety_pipeline")),
        ),
        # Read-only slope/tilt comparison for IMU-LiDAR commissioning.  The
        # output topic is deliberately unused, so this node can never gate or
        # command the manual mapping drive path.
        Node(
            package="roka_terrain_supervisor",
            executable="terrain_supervisor_node",
            name="mapping_terrain_monitor",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("roka_terrain_supervisor"),
                    "config",
                    "terrain_supervisor.yaml",
                ]),
                PathJoinSubstitution([
                    FindPackageShare("roka_terrain_supervisor"),
                    "config",
                    "provisional_vehicle_limits.yaml",
                ]),
                {
                    "input_cmd_topic": "/cmd_vel_joy",
                    "output_cmd_topic": "/mapping/terrain_cmd_unused",
                    "use_track_speed_limit": False,
                    # This node records slope/attitude evidence only. Missing
                    # autonomous inputs during Xbox mapping are observations,
                    # not safety faults, and diagnostics are rate-limited.
                    "diagnostics_observe_only": True,
                    "diagnostic_rate_hz": 1.0,
                },
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("use_terrain_monitor")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_nav_utils"),
                    "launch",
                    "tf_flatten_2d.launch.py",
                ])
            ),
            condition=IfCondition(LaunchConfiguration("use_imu_ekf")),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level")
            }.items(),
        ),
        Node(
            package="roka_nav2_bringup",
            executable="track_path_recorder",
            name="track_path_recorder",
            output="screen",
            parameters=[{
                "output_file": LaunchConfiguration("track_profile"),
                "map_frame": LaunchConfiguration("track_map_frame"),
                # Record the SLAM-corrected pose directly. The bundle validator
                # rejects large map->odom jumps, while this keeps the saved
                # route spatially aligned with the occupancy map.
                "sample_frame": LaunchConfiguration("track_sample_frame"),
                "base_frame": LaunchConfiguration("track_base_frame"),
                "path_topic": "/track/recorded_path",
                "auto_start": False,
                "environment": LaunchConfiguration("environment"),
                "track_width_m": ParameterValue(
                    LaunchConfiguration("track_width_m"), value_type=float
                ),
                "closure_distance_m": ParameterValue(
                    LaunchConfiguration("track_closure_distance_m"), value_type=float
                ),
                "closure_heading_tolerance_deg": ParameterValue(
                    LaunchConfiguration("track_closure_heading_tolerance_deg"),
                    value_type=float,
                ),
                "minimum_closed_loop_length_m": ParameterValue(
                    LaunchConfiguration("track_minimum_closed_loop_length_m"),
                    value_type=float,
                ),
                "minimum_closed_loop_area_m2": ParameterValue(
                    LaunchConfiguration("track_minimum_closed_loop_area_m2"),
                    value_type=float,
                ),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("record_track_path")),
        ),
        Node(
            package="roka_nav2_bringup",
            executable="track_mapping_visualizer",
            name="track_mapping_visualizer",
            output="screen",
            parameters=[{"map_frame": "map"}],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("use_3d_slam")),
        ),
        Node(
            package="roka_nav2_bringup",
            executable="map_bundle_server",
            name="map_bundle_server",
            output="screen",
            parameters=[{
                "directory": bundle,
                "final_directory": LaunchConfiguration(
                    "final_map_bundle_directory"
                ),
                "overwrite_existing": ParameterValue(
                    LaunchConfiguration("overwrite_existing"), value_type=bool
                ),
                "environment": LaunchConfiguration("environment"),
                "mapping_backend": LaunchConfiguration("mapping_backend"),
                "require_sensor_bag": ParameterValue(
                    LaunchConfiguration("record_bag"), value_type=bool
                ),
                "acquisition_only": ParameterValue(
                    LaunchConfiguration("acquisition_only"), value_type=bool
                ),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("use_xbox_mapping_controls")),
        ),
        mapping_recorder,
        mapping_controls,
        RegisterEventHandler(
            OnProcessExit(
                target_action=mapping_controls,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="mapping controls completed or stopped"
                ))],
            )
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_id": ParameterValue(
                    LaunchConfiguration("joy_device_id"), value_type=int
                ),
                "deadzone": 0.12,
                "autorepeat_rate": 20.0,
                "coalesce_interval_ms": 1,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("joy_available")),
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("roka_md_driver"), "config", "xbox_teleop.yaml"
            ])],
            # The mapping control node forwards this raw command only while
            # recording or after a complete save, and only while hardware
            # preflight (including arm hold) and live MD feedback still pass.
            remappings=[("cmd_vel", "/cmd_vel_joy_raw")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            condition=IfCondition(LaunchConfiguration("joy_available")),
        ),
        # Observe the same nodes that remain active for mapping.  FAST-LIO
        # backends additionally have to remain stationary through the startup
        # window before Xbox A can begin a recording.
        OpaqueFunction(function=_field_preflight),
    ])
