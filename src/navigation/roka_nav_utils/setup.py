from setuptools import find_packages, setup


package_name = "roka_nav_utils"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/tf_flatten_2d.launch.py"]),
        (f"share/{package_name}/config", ["config/tf_flatten_2d.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description="Navigation utility nodes for the ROS 2 migration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tf_flatten_2d = roka_nav_utils.tf_flatten_2d:main",
        ],
    },
)
