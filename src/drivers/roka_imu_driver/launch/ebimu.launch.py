import logging
import os

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _configure_launch_logging(_context):
    launch.logging.launch_config.level = logging.INFO
    return []


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("roka_imu_driver"), "config", "ebimu_v5.yaml"
    )
    return LaunchDescription(
        [
            # Use a sensor-specific launch key so an unscoped include cannot
            # replace the validated V5 configuration.
            DeclareLaunchArgument("imu_config", default_value=default_config),
            DeclareLaunchArgument("log_level", default_value="warn"),
            OpaqueFunction(function=_configure_launch_logging),
            DeclareLaunchArgument(
                "serial_port",
                default_value="auto",
            ),
            DeclareLaunchArgument("allow_sensor_configuration", default_value="false"),
            Node(
                package="roka_imu_driver",
                executable="ebimu_v5_driver_node",
                name="ebimu_driver",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                parameters=[
                    LaunchConfiguration("imu_config"),
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {
                        "allow_sensor_configuration": ParameterValue(
                            LaunchConfiguration("allow_sensor_configuration"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
