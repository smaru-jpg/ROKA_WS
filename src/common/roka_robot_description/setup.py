from setuptools import find_packages, setup


package_name = "roka_robot_description"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/pk3a_hardware.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description="Canonical ROKA robot geometry and sensor contract.",
    license="MIT",
)
