"""Targetless dynamic VLP-16/EBIMU calibration with manual Xbox motion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from roka_robot_description.hardware_config import (
    default_hardware_config_path,
    load_hardware_config,
)


def _calibration_stack(context):
    output = Path(LaunchConfiguration("output_dir").perform(context)).expanduser().resolve()
    result = output / "Initialization_result.txt"
    bag = output / "sensors"
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"calibration output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config = str(
        Path(get_package_share_directory("lidar_imu_init"))
        / "config"
        / "roka_vlp16.yaml"
    )
    bag_qos = str(
        Path(get_package_share_directory("roka_fast_lio_localization"))
        / "config"
        / "calibration_bag_qos.yaml"
    )
    hardware = load_hardware_config(
        LaunchConfiguration("hardware_config").perform(context)
    )
    common_height = hardware.footprint_to_base.xyz[2]

    li_init = Node(
        package="lidar_imu_init",
        executable="li_init",
        name="roka_lidar_imu_dynamic_calibration",
        output="screen",
        emulate_tty=True,
        parameters=[
            config,
            {
                "initialization.result_file": str(result),
                "initialization.shutdown_on_complete": True,
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    bag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--output", str(bag),
            "--storage", "sqlite3",
            "--max-bag-size", str(512 * 1024 * 1024),
            "--compression-mode", "file",
            "--compression-format", "zstd",
            "--compression-threads", "2",
            "--qos-profile-overrides-path", bag_qos,
            "--log-level", "warn",
            "/velodyne_packets", "/imu/data", "/wheel/odom",
            "/joy", "/cmd_vel", "/md/base_connected",
            "/md/left/raw_state", "/md/right/raw_state",
        ],
        output="screen",
    )
    static_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="calibration_base_to_velodyne_tf",
            arguments=[
                "--x", str(hardware.base_to_lidar.xyz[0]),
                "--y", str(hardware.base_to_lidar.xyz[1]),
                "--z", str(common_height + hardware.base_to_lidar.xyz[2]),
                "--roll", str(hardware.base_to_lidar.rpy[0]),
                "--pitch", str(hardware.base_to_lidar.rpy[1]),
                "--yaw", str(hardware.base_to_lidar.rpy[2]),
                "--frame-id", hardware.body_2d_frame,
                "--child-frame-id", hardware.lidar_frame,
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="calibration_base_to_imu_tf",
            arguments=[
                "--x", str(hardware.base_to_imu.xyz[0]),
                "--y", str(hardware.base_to_imu.xyz[1]),
                "--z", str(common_height + hardware.base_to_imu.xyz[2]),
                "--roll", str(hardware.base_to_imu.rpy[0]),
                "--pitch", str(hardware.base_to_imu.rpy[1]),
                "--yaw", str(hardware.base_to_imu.rpy[2]),
                "--frame-id", hardware.body_2d_frame,
                "--child-frame-id", hardware.imu_frame,
            ],
        ),
    ]
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_arm"), "launch",
                    "arm_locked_xbox_drive.launch.py",
                ])
            ),
            launch_arguments={
                "allow_motion": LaunchConfiguration("allow_motion"),
                "joy_device_id": LaunchConfiguration("joy_device_id"),
                "cmd_vel_topic": "/cmd_vel",
                "require_field_preflight": "true",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("velodyne"), "launch",
                    "velodyne-all-nodes-VLP16-launch.py",
                ])
            ),
            launch_arguments={"log_level": LaunchConfiguration("log_level")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("roka_imu_driver"), "launch", "ebimu.launch.py",
                ])
            ),
            launch_arguments={
                "imu_config": PathJoinSubstitution([
                    FindPackageShare("roka_imu_driver"), "config", "ebimu_v5.yaml",
                ]),
                "serial_port": LaunchConfiguration("imu_serial_port"),
                "allow_sensor_configuration": "false",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        *static_nodes,
        Node(
            package="roka_nav2_bringup",
            executable="pk3a_field_preflight",
            name="calibration_field_preflight",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "duration_sec": 8.0,
                "require_xbox": True,
                "require_2d_scan": False,
                "require_arm_hold": True,
                "require_lio_stability": False,
                "retry_until_ready": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        bag_record,
        li_init,
        Node(
            package="rviz2",
            executable="rviz2",
            name="lidar_imu_calibration_rviz",
            output="screen",
            arguments=[
                "-d",
                str(Path(get_package_share_directory("lidar_imu_init")) / "rviz_cfg" / "airy.rviz"),
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
            parameters=[{"use_sim_time": False}],
            condition=None,
        ) if LaunchConfiguration("use_rviz").perform(context).lower() in {
            "1", "true", "yes", "on"
        } else None,
        RegisterEventHandler(
            OnProcessExit(
                target_action=li_init,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="LI-Init dynamic calibration finished"
                ))],
            )
        ),
    ]
    return [action for action in actions if action is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("allow_motion", default_value="true"),
        DeclareLaunchArgument("joy_device_id", default_value="auto"),
        DeclareLaunchArgument("imu_serial_port", default_value="auto"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument(
            "hardware_config", default_value=str(default_hardware_config_path())
        ),
        OpaqueFunction(function=_calibration_stack),
    ])
