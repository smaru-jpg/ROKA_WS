from pathlib import Path
import math

import pytest
import yaml

from roka_robot_description.hardware_config import load_hardware_config


SOURCE_CONFIG = Path(__file__).parents[1] / "config" / "pk3a_hardware.yaml"


def test_pk3a_sensor_geometry_is_internally_consistent():
    config = load_hardware_config(SOURCE_CONFIG)
    assert config.coordinate_convention == "ros_flu"
    assert config.base_to_lidar.xyz == pytest.approx((0.1815, 0.0, 0.112325))
    assert config.base_to_imu.xyz == pytest.approx((0.1815, 0.0, 0.206625))
    assert math.sqrt(sum(value * value for value in config.lidar_in_imu_translation)) == pytest.approx(0.0943)
    assert config.lidar_rate_hz == pytest.approx(10.0)
    assert config.imu_rate_hz == pytest.approx(100.0)


def test_pk3a_physical_and_effective_drive_geometry_are_not_conflated():
    config = load_hardware_config(SOURCE_CONFIG)
    assert config.wheel_radius_m == pytest.approx(0.103)
    assert config.wheelbase_m == pytest.approx(0.340)
    assert config.physical_wheel_track_m == pytest.approx(0.427149994)
    assert config.tire_half_width_m == pytest.approx(0.0591)
    assert config.wheel_outer_width_m == pytest.approx(0.545349994)
    assert config.effective_drive_separation_m == pytest.approx(0.66)
    assert config.wheel_separation_m == config.effective_drive_separation_m
    assert config.effective_drive_separation_status == "section5_multibag_flat_calibrated"
    assert config.navigation_width_m == pytest.approx(0.624)
    assert config.navigation_length_m == pytest.approx(0.880)
    assert tuple(value for point in config.navigation_footprint for value in point) == pytest.approx((
        0.590, 0.312, 0.590, -0.312,
        -0.290, -0.312, -0.290, 0.312,
    ))


def test_dynamic_calibration_state_is_coherent():
    config = load_hardware_config(SOURCE_CONFIG)
    assert config.rotation_validated_dynamically == config.temporal_offset_validated_dynamically
    assert math.isfinite(config.time_offset_lidar_to_imu_sec)
    assert abs(config.time_offset_lidar_to_imu_sec) <= 0.20
    document = yaml.safe_load(SOURCE_CONFIG.read_text(encoding="utf-8"))
    calibration = document["calibration"]["lidar_to_imu"]
    if config.rotation_validated_dynamically:
        assert calibration["rotation_source"] == "li_init_dynamic_two_run_validation"
        assert calibration["temporal_source"] in {
            "li_init_dynamic_two_run_validation",
            "li_init_dynamic_two_run_vlp16_cut_frame_phase_residual",
            "li_init_dynamic_two_run_vlp16_common_cut_frame_branch",
        }
        assert calibration["validated_at_utc"]
        assert calibration["validation_repeatability"]
    else:
        assert calibration["rotation_source"] == "aligned_mount_assumption"
        assert calibration["temporal_source"] == "host_timestamp_observation"


def test_nonzero_mount_rotation_is_converted_to_fastlio_extrinsics(tmp_path):
    document = yaml.safe_load(SOURCE_CONFIG.read_text(encoding="utf-8"))
    document["geometry"]["base_link_to_lidar"]["rpy_rad"] = [0.0, 0.0, math.pi / 2.0]
    document["geometry"]["base_link_to_imu"]["rpy_rad"] = [0.0, 0.0, math.pi / 2.0]
    config_path = tmp_path / "hardware.yaml"
    config_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    config = load_hardware_config(config_path)
    assert config.lidar_in_imu_rotation_matrix == pytest.approx(
        (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), abs=1.0e-12
    )
    # A base -Z displacement is unchanged by a pure yaw rotation.
    assert config.lidar_in_imu_translation == pytest.approx((0.0, 0.0, -0.0943))
