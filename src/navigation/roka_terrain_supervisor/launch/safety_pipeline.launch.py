from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("roka_terrain_supervisor")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    direct_recorded_route = LaunchConfiguration("direct_recorded_route")
    # Wheel/body odometry comparison remains useful on a recorded route.  It
    # does not reinterpret the banked track as an obstacle; it only detects
    # relative wheel spin.  The caller still decides whether slip is enabled.
    normal_contact_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration("use_contact_estimator"),
        "'.lower() in ['true', '1', 'yes', 'on']",
    ]))
    normal_collision_condition = IfCondition(PythonExpression([
        "'", use_collision_monitor, "'.lower() in ['true', '1', 'yes', 'on'] and '",
        direct_recorded_route, "'.lower() not in ['true', '1', 'yes', 'on']",
    ]))
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("use_collision_monitor", default_value="true"),
        DeclareLaunchArgument("direct_recorded_route", default_value="false"),
        DeclareLaunchArgument("enable_terrain_decisions", default_value="true"),
        DeclareLaunchArgument("collision_source_timeout", default_value="0.40"),
        DeclareLaunchArgument("require_terrain", default_value="true"),
        DeclareLaunchArgument("allow_level_fallback", default_value="false"),
        DeclareLaunchArgument("require_imu", default_value="false"),
        DeclareLaunchArgument("require_odom", default_value="false"),
        DeclareLaunchArgument("enable_slip_detection", default_value="false"),
        DeclareLaunchArgument("use_contact_estimator", default_value="true"),
        DeclareLaunchArgument("derive_body_twist_from_pose", default_value="false"),
        DeclareLaunchArgument("body_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odom"),
        DeclareLaunchArgument("contact_state_topic", default_value="/terrain/wheel_contact"),
        DeclareLaunchArgument("physical_wheel_track", default_value="0.427149994"),
        DeclareLaunchArgument("wheel_radius", default_value="0.103"),
        DeclareLaunchArgument("footprint_length", default_value="0.88"),
        DeclareLaunchArgument("footprint_width", default_value="0.624"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "supervisor_config",
            default_value=PathJoinSubstitution([share, "config", "terrain_supervisor.yaml"]),
        ),
        DeclareLaunchArgument(
            "dynamic_limits_config",
            default_value=PathJoinSubstitution(
                [share, "config", "provisional_vehicle_limits.yaml"]
            ),
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                PathJoinSubstitution([share, "config", "twist_mux.yaml"]),
                {"use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )},
            ],
            remappings=[("cmd_vel_out", "/cmd_vel_mux_raw")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="terrain_velocity_smoother",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                PathJoinSubstitution([share, "config", "common_velocity_smoother.yaml"]),
                {"use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )},
            ],
            remappings=[
                ("cmd_vel", "/cmd_vel_mux_raw"),
                ("cmd_vel_smoothed", "/cmd_vel_mux"),
            ],
        ),
        # Lifecycle clients can reach a freshly spawned component before its
        # services are ready.  On the real field stack this left the velocity
        # smoother INACTIVE, so Nav2 had a valid goal but no command could
        # reach the motor driver.  Stagger lifecycle activation after process
        # construction and avoid racing the other Nav2 lifecycle managers.
        TimerAction(
            # Run after localization/controller construction instead of during
            # peak startup load; the transition response otherwise sometimes
            # misses the lifecycle client's deadline on the field PC.
            period=6.0,
            actions=[Node(
                package="roka_terrain_supervisor",
                executable="roka_lifecycle_activator",
                name="terrain_smoother_lifecycle_activator",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                parameters=[{
                    "target_node": "/terrain_velocity_smoother",
                    "request_timeout_sec": 5.0,
                    "retry_period_sec": 1.0,
                }],
            )],
        ),
        Node(
            package="roka_terrain_supervisor",
            executable="terrain_supervisor_node",
            name="terrain_supervisor",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("supervisor_config"),
                LaunchConfiguration("dynamic_limits_config"),
                {
                    "require_terrain": ParameterValue(
                        LaunchConfiguration("require_terrain"), value_type=bool
                    ),
                    "enable_terrain_decisions": ParameterValue(
                        LaunchConfiguration("enable_terrain_decisions"),
                        value_type=bool,
                    ),
                    "allow_level_fallback": ParameterValue(
                        LaunchConfiguration("allow_level_fallback"), value_type=bool
                    ),
                    "require_imu": ParameterValue(
                        LaunchConfiguration("require_imu"), value_type=bool
                    ),
                    "require_odom": ParameterValue(
                        LaunchConfiguration("require_odom"), value_type=bool
                    ),
                    "enable_slip_detection": ParameterValue(
                        LaunchConfiguration("enable_slip_detection"), value_type=bool
                    ),
                    "body_odom_topic": LaunchConfiguration("body_odom_topic"),
                    "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                    "contact_state_topic": LaunchConfiguration("contact_state_topic"),
                    "wheel_track": ParameterValue(
                        LaunchConfiguration("physical_wheel_track"), value_type=float
                    ),
                    "footprint_length": ParameterValue(
                        LaunchConfiguration("footprint_length"), value_type=float
                    ),
                    "footprint_width": ParameterValue(
                        LaunchConfiguration("footprint_width"), value_type=float
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "output_cmd_topic": PythonExpression([
                        "'/cmd_vel' if '", direct_recorded_route,
                        "'.lower() in ['true', '1', 'yes', 'on'] "
                        "else '/cmd_vel_terrain'",
                    ]),
                }],
        ),
        Node(
            package="roka_terrain_supervisor",
            executable="wheel_contact_estimator_node",
            name="wheel_contact_estimator",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("supervisor_config"),
                LaunchConfiguration("dynamic_limits_config"),
                {
                    "body_odom_topic": LaunchConfiguration("body_odom_topic"),
                    "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                    "derive_body_twist_from_pose": ParameterValue(
                        LaunchConfiguration("derive_body_twist_from_pose"),
                        value_type=bool,
                    ),
                    "contact_state_topic": LaunchConfiguration("contact_state_topic"),
                    "wheel_track": ParameterValue(
                        LaunchConfiguration("physical_wheel_track"), value_type=float
                    ),
                    "wheel_radius": ParameterValue(
                        LaunchConfiguration("wheel_radius"), value_type=float
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
            condition=normal_contact_condition,
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                PathJoinSubstitution([share, "config", "collision_monitor.yaml"]),
                {
                    "source_timeout": ParameterValue(
                        LaunchConfiguration("collision_source_timeout"),
                        value_type=float,
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
            condition=normal_collision_condition,
        ),
        TimerAction(
            # CollisionMonitor plugin setup is the slowest lifecycle
            # transition in this stack (about 0.45 s on the field PC). Start
            # it after localization, smoother and controller startup load has
            # settled so the lifecycle client's response deadline is not lost.
            period=10.0,
            actions=[Node(
                package="roka_terrain_supervisor",
                executable="roka_lifecycle_activator",
                name="collision_monitor_lifecycle_activator",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level", LaunchConfiguration("log_level")
                ],
                parameters=[{
                    "target_node": "/collision_monitor",
                    "request_timeout_sec": 5.0,
                    "retry_period_sec": 1.0,
                }],
                condition=normal_collision_condition,
            )],
        ),
    ])
