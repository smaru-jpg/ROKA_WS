import os
from glob import glob

from setuptools import find_packages, setup


package_name = "roka_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description="ROKA section mission orchestration.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "box_pick_coordinator = roka_mission.box_pick_coordinator:main",
            "blind_segment_recorder = roka_mission.blind_segment_recorder:main",
            "blind_segment_controller = roka_mission.blind_segment_controller:main",
            "section5_lifecycle_gate = roka_mission.section5_lifecycle_gate:main",
            "section_manual_joy_router = roka_mission.section_manual_joy_router:main",
            "traffic_signal_route_gate = roka_mission.traffic_signal_route_gate:main",
        ],
    },
)
