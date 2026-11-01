import logging
from pathlib import Path

import launch.logging
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from roka_robot_description.hardware_config import load_hardware_config


def _configure_launch_logging(_context):
    launch.logging.launch_config.level = logging.INFO
    return []


def _include_lidar(context):
    enabled = LaunchConfiguration("use_lidar_driver").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return [LogInfo(msg="[lidar_md_base] Velodyne driver disabled.")]

    package = LaunchConfiguration("lidar_package").perform(context)
    launch_file = LaunchConfiguration("lidar_launch_file").perform(context)
    try:
        launch_path = Path(get_package_share_directory(package)) / "launch" / launch_file
    except PackageNotFoundError:
        return [LogInfo(msg=f"[lidar_md_base] LiDAR package '{package}' is missing.")]
    if not launch_path.exists():
        return [LogInfo(msg=f"[lidar_md_base] LiDAR launch is missing: {launch_path}")]
    return [
        LogInfo(msg=f"[lidar_md_base] Starting {package}/{launch_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_path)),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level")
            }.items(),
        ),
    ]


def generate_launch_description():
    hardware = load_hardware_config()
    footprint_lidar_z = hardware.footprint_to_base.xyz[2] + hardware.base_to_lidar.xyz[2]
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="warn"),
            OpaqueFunction(function=_configure_launch_logging),
            DeclareLaunchArgument("use_lidar_driver", default_value="true"),
            DeclareLaunchArgument("lidar_package", default_value="velodyne"),
            DeclareLaunchArgument(
                "lidar_launch_file", default_value="velodyne-all-nodes-VLP16-launch.py"
            ),
            DeclareLaunchArgument("use_md_driver", default_value="true"),
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
            DeclareLaunchArgument(
                "md_simulation_mode",
                default_value="true",
                description="Safe default: run the two-controller base without serial hardware.",
            ),
            DeclareLaunchArgument("md_allow_motion", default_value="false"),
            DeclareLaunchArgument("md_max_linear_speed", default_value="0.60"),
            DeclareLaunchArgument("md_max_wheel_rpm", default_value="56.0"),
            DeclareLaunchArgument(
                "md_feedback_overspeed_limit_rpm", default_value="90.0"
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
                "wheel_separation", default_value=str(hardware.wheel_separation_m)
            ),
            DeclareLaunchArgument("md_cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("md_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("md_base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("md_publish_tf", default_value="true"),
            DeclareLaunchArgument("use_lidar_static_tf", default_value="true"),
            # MD-only mode uses base_footprint. Set base_frame:=base_link after
            # the 3D EBIMU EKF owns odom->base_link.
            DeclareLaunchArgument("base_frame", default_value=hardware.body_2d_frame),
            DeclareLaunchArgument("lidar_frame", default_value=hardware.lidar_frame),
            DeclareLaunchArgument("lidar_x", default_value=str(hardware.base_to_lidar.xyz[0])),
            DeclareLaunchArgument("lidar_y", default_value=str(hardware.base_to_lidar.xyz[1])),
            DeclareLaunchArgument("lidar_z", default_value=str(footprint_lidar_z)),
            DeclareLaunchArgument("lidar_roll", default_value=str(hardware.base_to_lidar.rpy[0])),
            DeclareLaunchArgument("lidar_pitch", default_value=str(hardware.base_to_lidar.rpy[1])),
            DeclareLaunchArgument("lidar_yaw", default_value=str(hardware.base_to_lidar.rpy[2])),
            DeclareLaunchArgument("use_stabilized_scan", default_value="true"),
            DeclareLaunchArgument(
                "scan_cloud_topic",
                default_value="/velodyne_points",
                description=(
                    "Dedicated raw cloud slice for SLAM/AMCL. Terrain obstacle "
                    "clouds are derived, sparse, and must not own localization."
                ),
            ),
            DeclareLaunchArgument(
                "pointcloud_to_scan_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("roka_terrain_mapping"),
                        "config",
                        "pointcloud_to_scan.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_include_lidar),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("roka_md_driver"),
                            "launch",
                            "mdh250_md200t_four_wheel.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config": LaunchConfiguration("md_config"),
                    "simulation_mode": LaunchConfiguration("md_simulation_mode"),
                    "allow_motion": LaunchConfiguration("md_allow_motion"),
                    "max_linear_speed": LaunchConfiguration(
                        "md_max_linear_speed"
                    ),
                    "max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
                    "feedback_overspeed_limit_rpm": LaunchConfiguration(
                        "md_feedback_overspeed_limit_rpm"
                    ),
                    "left_serial_port": LaunchConfiguration("left_serial_port"),
                    "right_serial_port": LaunchConfiguration("right_serial_port"),
                    "wheel_separation": LaunchConfiguration("wheel_separation"),
                    "cmd_vel_topic": LaunchConfiguration("md_cmd_vel_topic"),
                    "odom_topic": LaunchConfiguration("md_odom_topic"),
                    "base_frame_id": LaunchConfiguration("md_base_frame"),
                    "publish_tf": LaunchConfiguration("md_publish_tf"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_md_driver")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_velodyne_tf",
                output="screen",
                arguments=[
                    "--x",
                    LaunchConfiguration("lidar_x"),
                    "--y",
                    LaunchConfiguration("lidar_y"),
                    "--z",
                    LaunchConfiguration("lidar_z"),
                    "--roll",
                    LaunchConfiguration("lidar_roll"),
                    "--pitch",
                    LaunchConfiguration("lidar_pitch"),
                    "--yaw",
                    LaunchConfiguration("lidar_yaw"),
                    "--frame-id",
                    LaunchConfiguration("base_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("lidar_frame"),
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(LaunchConfiguration("use_lidar_static_tf")),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="stabilized_pointcloud_to_laserscan",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                parameters=[LaunchConfiguration("pointcloud_to_scan_config")],
                remappings=[
                    ("cloud_in", LaunchConfiguration("scan_cloud_topic")),
                    ("scan", "/scan_stabilized"),
                ],
                condition=IfCondition(LaunchConfiguration("use_stabilized_scan")),
            ),
        ]
    )
