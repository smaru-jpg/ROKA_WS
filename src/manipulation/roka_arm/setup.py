import os
from glob import glob

from setuptools import find_packages, setup


package_name = "roka_arm"

setup(
    name=package_name,
    version="0.1.0",
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
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "models"),
            glob("models/*"),
        ),
        (
            os.path.join("share", package_name, "tools"),
            glob("tools/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description=(
        "Isolated Dynamixel arm and D435 box visual-servo runtime for ROKA."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "roka_arm_controller = roka_arm.arm_control_node:main",
            "roka_arm_hold = roka_arm.arm_hold_node:main",
            "roka_arm_hold_gate = roka_arm.arm_hold_gate:main",
            "roka_manual_arm_joy = roka_arm.manual_arm_joy:main",
            "roka_arm_preflight = roka_arm.preflight:main",
        ],
    },
)
