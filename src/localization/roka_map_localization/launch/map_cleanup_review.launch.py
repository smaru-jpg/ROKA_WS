"""Review a saved PCD in RViz and create a non-destructive cleaned copy."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from roka_robot_description.workspace import workspace_path


def _actions(context):
    directory = Path(
        LaunchConfiguration("map_directory").perform(context)
    ).expanduser().resolve()
    configured_input = LaunchConfiguration("input_pcd").perform(context).strip()
    input_pcd = (
        Path(configured_input).expanduser().resolve()
        if configured_input
        else directory / "map.pcd"
    )
    configured_output = LaunchConfiguration("output_pcd").perform(context).strip()
    output_pcd = (
        Path(configured_output).expanduser().resolve()
        if configured_output
        else directory / "map_cleaned.pcd"
    )
    configured_audit = LaunchConfiguration("audit_file").perform(context).strip()
    audit_file = (
        Path(configured_audit).expanduser().resolve()
        if configured_audit
        else directory / "map_cleanup.yaml"
    )
    if not input_pcd.is_file():
        raise RuntimeError(f"cleanup input PCD does not exist: {input_pcd}")

    share = Path(get_package_share_directory("roka_map_localization"))
    return [
        Node(
            package="roka_map_localization",
            executable="map_cleanup_editor",
            name="map_cleanup_editor",
            output="screen",
            parameters=[{
                "input_pcd": str(input_pcd),
                "output_pcd": str(output_pcd),
                "audit_file": str(audit_file),
                "map_frame": "map",
                "removal_radius_m": ParameterValue(
                    LaunchConfiguration("removal_radius_m"), value_type=float
                ),
                "minimum_remaining_fraction": ParameterValue(
                    LaunchConfiguration("minimum_remaining_fraction"), value_type=float
                ),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="map_cleanup_rviz",
            output="screen",
            arguments=[
                "-d", str(share / "rviz" / "map_cleanup.rviz"),
                "--ros-args", "--log-level", LaunchConfiguration("log_level"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_directory",
            default_value=str(workspace_path("maps", "flat", "section_1")),
        ),
        DeclareLaunchArgument("input_pcd", default_value=""),
        DeclareLaunchArgument("output_pcd", default_value=""),
        DeclareLaunchArgument("audit_file", default_value=""),
        DeclareLaunchArgument("removal_radius_m", default_value="0.30"),
        DeclareLaunchArgument("minimum_remaining_fraction", default_value="0.50"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        OpaqueFunction(function=_actions),
    ])
