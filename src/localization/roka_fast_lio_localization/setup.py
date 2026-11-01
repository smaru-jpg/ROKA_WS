from setuptools import find_packages, setup


package_name = "roka_fast_lio_localization"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/sensor_contract.launch.py",
            "launch/fastlio_sc_pgo_mapping.launch.py",
            "launch/fastlio_bag_regression.launch.py",
            "launch/fixed_map_bag_regression.launch.py",
            "launch/lidar_imu_dynamic_calibration.launch.py",
            "launch/lidar_imu_bag_calibration.launch.py",
        ]),
        (f"share/{package_name}/config", [
            "config/calibration_bag_qos.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryoo",
    maintainer_email="ryoo@example.com",
    description="ROKA VLP-16/EBIMU FAST-LIO2, SC-PGO and regression integration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "roka_bag_sensor_audit = roka_fast_lio_localization.bag_sensor_audit:main",
            "live_sensor_contract = roka_fast_lio_localization.live_sensor_contract:main",
            "lio_regression_monitor = roka_fast_lio_localization.lio_regression_monitor:main",
            "map_localization_regression_monitor = roka_fast_lio_localization.map_localization_regression_monitor:main",
            "roka_lidar_imu_calibration = roka_fast_lio_localization.lidar_imu_calibration:main",
        ],
    },
)
