from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from roka_robot_description.hardware_config import load_hardware_config


def _include_slam_toolbox(context):
    try:
        package_share = Path(get_package_share_directory("slam_toolbox"))
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "[ERROR] slam_toolbox is not installed. Install ros-humble-slam-toolbox, "
                    "then launch this file again."
                )
            ),
            EmitEvent(event=Shutdown(reason="slam_toolbox is missing")),
        ]
    return [Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "--ros-args", "--log-level", LaunchConfiguration("log_level")
        ],
    )]


def generate_launch_description():
    hardware = load_hardware_config()
    footprint_lidar_z = hardware.footprint_to_base.xyz[2] + hardware.base_to_lidar.xyz[2]
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roka_nav2_bringup"), "config", "lidar_md_slam.yaml"]
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
                "md_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("roka_md_driver"), "config", "mdh250_md200t_four_wheel.yaml"]
                ),
            ),
            DeclareLaunchArgument("md_simulation_mode", default_value="true"),
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
            DeclareLaunchArgument("base_frame", default_value=hardware.body_2d_frame),
            DeclareLaunchArgument("lidar_x", default_value=str(hardware.base_to_lidar.xyz[0])),
            DeclareLaunchArgument("lidar_y", default_value=str(hardware.base_to_lidar.xyz[1])),
            DeclareLaunchArgument("lidar_z", default_value=str(footprint_lidar_z)),
            DeclareLaunchArgument("lidar_roll", default_value=str(hardware.base_to_lidar.rpy[0])),
            DeclareLaunchArgument("lidar_pitch", default_value=str(hardware.base_to_lidar.rpy[1])),
            DeclareLaunchArgument("lidar_yaw", default_value=str(hardware.base_to_lidar.rpy[2])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("roka_nav2_bringup"), "launch", "lidar_md_base.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_lidar_driver": LaunchConfiguration("use_lidar_driver"),
                    "lidar_package": LaunchConfiguration("lidar_package"),
                    "lidar_launch_file": LaunchConfiguration("lidar_launch_file"),
                    "use_md_driver": LaunchConfiguration("use_md_driver"),
                    "md_config": LaunchConfiguration("md_config"),
                    "md_simulation_mode": LaunchConfiguration("md_simulation_mode"),
                    "md_allow_motion": LaunchConfiguration("md_allow_motion"),
                    "md_max_linear_speed": LaunchConfiguration(
                        "md_max_linear_speed"
                    ),
                    "md_max_wheel_rpm": LaunchConfiguration("md_max_wheel_rpm"),
                    "md_feedback_overspeed_limit_rpm": LaunchConfiguration(
                        "md_feedback_overspeed_limit_rpm"
                    ),
                    "left_serial_port": LaunchConfiguration("left_serial_port"),
                    "right_serial_port": LaunchConfiguration("right_serial_port"),
                    "wheel_separation": LaunchConfiguration("wheel_separation"),
                    "md_cmd_vel_topic": LaunchConfiguration("md_cmd_vel_topic"),
                    "md_odom_topic": LaunchConfiguration("md_odom_topic"),
                    "md_base_frame": LaunchConfiguration("md_base_frame"),
                    "md_publish_tf": LaunchConfiguration("md_publish_tf"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "lidar_x": LaunchConfiguration("lidar_x"),
                    "lidar_y": LaunchConfiguration("lidar_y"),
                    "lidar_z": LaunchConfiguration("lidar_z"),
                    "lidar_roll": LaunchConfiguration("lidar_roll"),
                    "lidar_pitch": LaunchConfiguration("lidar_pitch"),
                    "lidar_yaw": LaunchConfiguration("lidar_yaw"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            OpaqueFunction(function=_include_slam_toolbox),
            Node(
                package="rviz2",
                executable="rviz2",
                name="lidar_md_slam_rviz",
                output="screen",
                arguments=[
                    "-d", LaunchConfiguration("rviz_config"),
                    "--ros-args", "--log-level", LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
