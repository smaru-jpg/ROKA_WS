import os
from glob import glob

from setuptools import find_packages, setup


package_name = "roka_mujoco"


def install_tree(source_directory):
    """Return setuptools entries for a recursively installed data tree."""
    entries = []
    for directory, _, filenames in os.walk(source_directory):
        files = [os.path.join(directory, filename) for filename in filenames]
        if files:
            entries.append(
                (
                    os.path.join("share", package_name, directory),
                    files,
                )
            )
    return entries


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ]
    + install_tree("config")
    + install_tree("description"),
    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "Pillow",
        "mujoco",
        "trimesh",
        "PyYAML",
    ],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description=(
        "PK-3A track scene and native MuJoCo ros2_control launch package."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_ui_setup = roka_mujoco.mujoco_ui_setup:main",
            "generate_pk3a = roka_mujoco.generate_closed_loop:main",
            "validate_pk3a = roka_mujoco.validate_closed_loop:main",
            "generate_roka_robot = roka_mujoco.generate_robot:main",
            "validate_roka_robot = roka_mujoco.validate_robot:main",
            "export_pk3a_ground_truth = roka_mujoco.ground_truth_map:main",
            "generate_pk3a_safe_path = roka_mujoco.safe_path:main",
            "sim_safe_path_controller = roka_mujoco.safe_path_controller:main",
            "record_xbox_demonstration = roka_mujoco.demonstration_recorder:main",
            "train_xbox_policy = roka_mujoco.demonstration_learning:main",
            "build_traversability_dataset = roka_mujoco.traversability_dataset:main",
            "derive_dynamic_safety_envelope = roka_mujoco.dynamic_safety_envelope:main",
            "sim_learned_controller = roka_mujoco.learned_controller:main",
            "sim_learned_benchmark = roka_mujoco.benchmark_runner:main",
            "sim_mapped_profile_controller = roka_mujoco.mapped_profile_controller:main",
            "sim_sensor_adapter = roka_mujoco.sim_sensor_adapter:main",
            "sim_nav2_bridge = roka_mujoco.nav2_sim_bridge:main",
            "build_sim_assisted_map = roka_mujoco.sim_assisted_map:main",
            "sim_assisted_map_visualizer = roka_mujoco.sim_assisted_map_visualizer:main",
            "export_assumed_mapped_track = roka_mujoco.assumed_mapping:main",
        ],
    },
)
