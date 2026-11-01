
import os
from glob import glob

from setuptools import find_packages, setup


package_name = "mission_vision"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "models"),
            glob("models/*.pt") + glob("models/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share", package_name, "firmware", "roka_iff_neopixel"
            ),
            glob("firmware/roka_iff_neopixel/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="deco",
    maintainer_email="deco@todo.todo",
    description=(
        "Modular ROS2 mission vision package with "
        "swappable cameras, YOLO models, and dynamic filters."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_node = mission_vision.vision_node:main",
            "camera_view_node = mission_vision.camera_view_node:main",
            "neopixel_controller = mission_vision.neopixel_controller:main",
        ],
    },
)
