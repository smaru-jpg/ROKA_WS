import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roka_robot_description.hardware_config import load_hardware_config


def generate_launch_description():
    hardware = load_hardware_config()
    package_share = get_package_share_directory("roka_imu_driver")
    default_imu_config = os.path.join(package_share, "config", "ebimu_v5.yaml")
    default_ekf_config = os.path.join(package_share, "config", "imu_wheel_ekf.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_config", default_value=default_imu_config),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("ekf_config", default_value=default_ekf_config),
            DeclareLaunchArgument(
                "serial_port",
                default_value="auto",
            ),
            DeclareLaunchArgument("allow_sensor_configuration", default_value="false"),
            DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odom"),
            DeclareLaunchArgument("filtered_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("use_imu_static_tf", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value=hardware.body_3d_frame),
            DeclareLaunchArgument("imu_x", default_value=str(hardware.base_to_imu.xyz[0])),
            DeclareLaunchArgument("imu_y", default_value=str(hardware.base_to_imu.xyz[1])),
            DeclareLaunchArgument("imu_z", default_value=str(hardware.base_to_imu.xyz[2])),
            DeclareLaunchArgument("imu_roll", default_value=str(hardware.base_to_imu.rpy[0])),
            DeclareLaunchArgument("imu_pitch", default_value=str(hardware.base_to_imu.rpy[1])),
            DeclareLaunchArgument("imu_yaw", default_value=str(hardware.base_to_imu.rpy[2])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "ebimu.launch.py")
                ),
                launch_arguments={
                    "imu_config": LaunchConfiguration("imu_config"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "allow_sensor_configuration": LaunchConfiguration(
                        "allow_sensor_configuration"
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
                    "--x", LaunchConfiguration("imu_x"),
                    "--y", LaunchConfiguration("imu_y"),
                    "--z", LaunchConfiguration("imu_z"),
                    "--roll", LaunchConfiguration("imu_roll"),
                    "--pitch", LaunchConfiguration("imu_pitch"),
                    "--yaw", LaunchConfiguration("imu_yaw"),
                    "--frame-id", LaunchConfiguration("base_frame"),
                    "--child-frame-id", hardware.imu_frame,
                    "--ros-args", "--log-level", LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(LaunchConfiguration("use_imu_static_tf")),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[LaunchConfiguration("ekf_config")],
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                remappings=[
                    ("/wheel/odom", LaunchConfiguration("wheel_odom_topic")),
                    ("odometry/filtered", LaunchConfiguration("filtered_odom_topic")),
                ],
            ),
        ]
    )
