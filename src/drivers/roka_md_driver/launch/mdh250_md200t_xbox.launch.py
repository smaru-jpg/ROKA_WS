from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from roka_md_driver.xbox_identity import (
    resolve_assigned_sdl_device_id,
)


def _select_roka_xbox(context):
    device_id = resolve_assigned_sdl_device_id(
        LaunchConfiguration('joy_device_id').perform(context),
    )
    context.launch_configurations['joy_device_id'] = str(device_id)
    return []


def generate_launch_description():
    package_share = FindPackageShare('roka_md_driver')
    driver_launch = PathJoinSubstitution([
        package_share, 'launch', 'mdh250_md200t_four_wheel.launch.py'])
    xbox_config = PathJoinSubstitution([
        package_share, 'config', 'xbox_base_teleop.yaml'])

    simulation_mode = LaunchConfiguration('simulation_mode')
    allow_motion = LaunchConfiguration('allow_motion')
    joy_device_id = LaunchConfiguration('joy_device_id')
    ros_domain_id = LaunchConfiguration('ros_domain_id')

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='31'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='auto'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('teleop_cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.00'),
        DeclareLaunchArgument('max_wheel_rpm', default_value='112.0'),
        DeclareLaunchArgument(
            'feedback_overspeed_limit_rpm', default_value='130.0'),
        OpaqueFunction(function=_select_roka_xbox),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'simulation_mode': simulation_mode,
                'allow_motion': allow_motion,
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_wheel_rpm': LaunchConfiguration('max_wheel_rpm'),
                'feedback_overspeed_limit_rpm': LaunchConfiguration(
                    'feedback_overspeed_limit_rpm'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='screen',
            parameters=[{
                'device_id': ParameterValue(joy_device_id, value_type=int),
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1,
            }],
            arguments=[
                '--ros-args', '--log-level', LaunchConfiguration('log_level')]),
        Node(
            package='roka_md_driver', executable='roka_xbox_teleop',
            name='roka_xbox_teleop', output='screen',
            parameters=[xbox_config, {
                'cmd_vel_topic': LaunchConfiguration('teleop_cmd_vel_topic'),
            }],
            arguments=[
                '--ros-args', '--log-level', LaunchConfiguration('log_level')]),
    ])
