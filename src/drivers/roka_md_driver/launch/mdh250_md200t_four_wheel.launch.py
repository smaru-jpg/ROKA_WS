from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from roka_robot_description.hardware_config import load_hardware_config


def generate_launch_description():
    hardware = load_hardware_config()
    config = PathJoinSubstitution([
        FindPackageShare('roka_md_driver'), 'config',
        'mdh250_md200t_four_wheel.yaml'])
    args = [
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.60'),
        DeclareLaunchArgument('max_wheel_rpm', default_value='56.0'),
        DeclareLaunchArgument(
            'feedback_overspeed_limit_rpm', default_value='90.0'),
        DeclareLaunchArgument('hardware_layout', default_value='dual'),
        DeclareLaunchArgument('single_side', default_value='right'),
        DeclareLaunchArgument(
            'left_serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_USB_to_RS-422_485_Adapter_A76BHBER-if00-port0')),
        DeclareLaunchArgument(
            'right_serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_USB_to_RS-422_485_Adapter_A751CYHM-if00-port0')),
        DeclareLaunchArgument(
            'wheel_separation',
            default_value=str(hardware.effective_drive_separation_m),
            description='Effective skid-steer separation; not physical wheel track.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
    ]
    node = Node(
        package='roka_md_driver', executable='roka_md_driver_node',
        name='roka_md_driver', output='screen',
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[LaunchConfiguration('config'), {
            'simulation_mode': LaunchConfiguration('simulation_mode'),
            'allow_motion': LaunchConfiguration('allow_motion'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_wheel_rpm': LaunchConfiguration('max_wheel_rpm'),
            'feedback_overspeed_limit_rpm': LaunchConfiguration(
                'feedback_overspeed_limit_rpm'),
            'hardware_layout': LaunchConfiguration('hardware_layout'),
            'single_side': LaunchConfiguration('single_side'),
            'left_serial_port': LaunchConfiguration('left_serial_port'),
            'right_serial_port': LaunchConfiguration('right_serial_port'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }])
    return LaunchDescription(args + [node])
