from setuptools import find_packages, setup


package_name = "roka_nav2_bringup"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/lidar_md_base.launch.py",
                "launch/lidar_md_slam.launch.py",
                "launch/lidar_md_navigation.launch.py",
                "launch/mapping_manual.launch.py",
                "launch/pk3a_autonomy.launch.py",
                "launch/pk3a_field.launch.py",
                "launch/real_traversability_record.launch.py",
                "launch/occupancy_cleanup_review.launch.py",
                "launch/recorded_mapping_view.launch.py",
                "launch/recorded_mapping_rebuild.launch.py",
                "launch/offline_2d_processing.launch.py",
                "launch/track_map_view.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/lidar_md_slam.yaml",
                "config/lidar_md_nav2.yaml",
                "config/sensor_data_contract.yaml",
            ],
        ),
        (
            f"share/{package_name}/rviz",
            [
                "rviz/lidar_md_navigation.rviz",
                "rviz/live_sensor_check.rviz",
                "rviz/mapping_growth_2d.rviz",
                "rviz/mapping_fastlio.rviz",
                "rviz/track_mapping_3d.rviz",
                "rviz/occupancy_cleanup.rviz",
                "rviz/recorded_mapping_replay.rviz",
                "rviz/recorded_mapping_rebuild.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description="LiDAR and MD motor SLAM/Nav2 bringup for the ROKA robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_bundle_server = roka_nav2_bringup.map_bundle_server:main",
            "mapping_session_recorder = roka_nav2_bringup.mapping_session_recorder:main",
            "track_path_recorder = roka_nav2_bringup.track_path_recorder:main",
            "xbox_mapping_controls = roka_nav2_bringup.xbox_mapping_controls:main",
            "xbox_autonomy_controls = roka_nav2_bringup.xbox_autonomy_controls:main",
            "track_path_follower = roka_nav2_bringup.track_path_follower:main",
            "profile_start_initializer = roka_nav2_bringup.profile_start_initializer:main",
            "apply_traversability_envelope = roka_nav2_bringup.risk_aware_profile:main",
            "real_traversability_recorder = roka_nav2_bringup.real_traversability_recorder:main",
            "pk3a_field_preflight = roka_nav2_bringup.field_preflight:main",
            "pk3a_map_check = roka_nav2_bringup.field_map_check:main",
            "roka_operator_status = roka_nav2_bringup.operator_status:main",
            "autonomy_command_pipeline = roka_nav2_bringup.command_pipeline_diagnostics:main",
            "derive_sc_pgo_navigation = roka_nav2_bringup.optimized_map_products:main",
            "approve_reviewed_map = roka_nav2_bringup.map_review:main",
            "approve_reviewed_occupancy = roka_nav2_bringup.map_review:occupancy_main",
            "occupancy_cleanup_editor = roka_nav2_bringup.occupancy_cleanup_editor:main",
            "recorded_route_visualizer = roka_nav2_bringup.recorded_route_visualizer:main",
            "track_mapping_visualizer = roka_nav2_bringup.track_mapping_visualizer:main",
            "replay_cloud_accumulator = roka_nav2_bringup.replay_cloud_accumulator:main",
            "tf_replay_filter = roka_nav2_bringup.tf_replay_filter:main",
            "offline_session_processor = roka_nav2_bringup.offline_session_processor:main",
            "offline_2d_finalize = roka_nav2_bringup.offline_2d_finalize:main",
            "extract_drivable_surface = roka_nav2_bringup.drivable_surface:main",
        ],
    },
)
