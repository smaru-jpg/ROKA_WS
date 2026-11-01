from pathlib import Path

import numpy as np
import pytest
import yaml

from roka_fast_lio_localization.lidar_imu_calibration import (
    apply_calibration,
    compare_runs,
    parse_li_init_result,
)


def result_block(*, rotation=(0.0, 0.0, 0.0), translation=(0.0, 0.0, -0.0943), time=0.004):
    return f"""Rotation LiDAR to IMU (degree)     = {rotation[0]} {rotation[1]} {rotation[2]}
Translation LiDAR to IMU (meter)   = {translation[0]} {translation[1]} {translation[2]}
Time Lag IMU to LiDAR (second)     = {time}
Bias of Gyroscope  (rad/s)         = 0 0 0
Bias of Accelerometer (meters/s^2) = 0 0 0
Gravity in World Frame(meters/s^2) = 0 0 -9.81

Homogeneous Transformation Matrix from LiDAR to IMU:
1 0 0 {translation[0]}
0 1 0 {translation[1]}
0 0 1 {translation[2]}
0 0 0 1
"""


def result_text(*, rotation=(0.0, 0.0, 0.0), translation=(0.0, 0.0, -0.0943), time=0.004):
    block = result_block(rotation=rotation, translation=translation, time=time)
    return f"""Initialization result:
{block}
Refinement result:
{block}
"""


def test_parse_final_refinement_result(tmp_path):
    source = tmp_path / "Initialization_result.txt"
    source.write_text(result_text(), encoding="utf-8")
    parsed = parse_li_init_result(source)
    assert parsed["result"] == "PASS"
    assert parsed["selected_stage"] == "batch_initialization"
    assert parsed["refinement_diagnostics"]["stable"] is True
    assert parsed["time_offset_lidar_to_imu_sec"] == pytest.approx(0.004)
    assert parsed["translation_lidar_in_imu_m"][2] == pytest.approx(-0.0943)


def test_unstable_refinement_does_not_replace_batch_candidate(tmp_path):
    source = tmp_path / "Initialization_result.txt"
    initial = result_block(time=-0.099)
    refinement = result_block(translation=(0.5, 0.5, 0.5), time=-0.099)
    source.write_text(
        f"Initialization result:\n{initial}\nRefinement result:\n{refinement}",
        encoding="utf-8",
    )
    parsed = parse_li_init_result(source)
    assert parsed["raw_time_offset_lidar_to_imu_sec"] == pytest.approx(-0.099)
    assert parsed["time_offset_lidar_to_imu_sec"] == pytest.approx(-0.099)
    assert parsed["translation_lidar_in_imu_m"][2] == pytest.approx(-0.0943)
    assert parsed["refinement_diagnostics"]["stable"] is False


def test_vlp16_absolute_lag_is_kept_for_two_run_branch_alignment(tmp_path):
    source = tmp_path / "Initialization_result.txt"
    source.write_text(result_text(time=-0.128141), encoding="utf-8")
    parsed = parse_li_init_result(source)
    assert parsed["raw_time_offset_lidar_to_imu_sec"] == pytest.approx(-0.128141)
    assert parsed["time_offset_lidar_to_imu_sec"] == pytest.approx(-0.128141)
    assert parsed["time_offset_normalization"]["nearest_whole_cut_frames"] == -4


def test_two_runs_align_to_stable_second_cut_frame_branch(tmp_path):
    first_file = tmp_path / "first.txt"
    second_file = tmp_path / "second.txt"
    first_file.write_text(result_text(time=-0.099510), encoding="utf-8")
    second_file.write_text(result_text(time=-0.128141), encoding="utf-8")
    first = parse_li_init_result(first_file)
    second = parse_li_init_result(second_file)
    first["refinement_diagnostics"]["stable"] = False
    comparison = compare_runs(first, second)
    alignment = comparison["time_offset_alignment"]
    assert comparison["result"] == "PASS"
    assert alignment["run_1_whole_frame_shift"] == -1
    assert alignment["run_2_whole_frame_shift"] == 0
    assert alignment["mean_offset_sec"] == pytest.approx(-0.1304921667)


def test_two_runs_must_repeat_rotation_and_time(tmp_path):
    source = tmp_path / "Initialization_result.txt"
    source.write_text(result_text(), encoding="utf-8")
    first = parse_li_init_result(source)
    second = dict(first)
    second["time_offset_lidar_to_imu_sec"] = 0.011
    second["raw_time_offset_lidar_to_imu_sec"] = 0.011
    assert compare_runs(first, second)["result"] == "FAIL"


def test_apply_sets_both_dynamic_validation_flags(tmp_path):
    source = tmp_path / "Initialization_result.txt"
    source.write_text(result_text(rotation=(0.0, 0.0, 0.0)), encoding="utf-8")
    first = parse_li_init_result(source)
    second = dict(first)
    hardware = tmp_path / "pk3a_hardware.yaml"
    hardware.write_text(yaml.safe_dump({
        "geometry": {
            "base_link_to_lidar": {"rpy_rad": [0.0, 0.0, 0.0]},
            "base_link_to_imu": {"rpy_rad": [0.0, 0.0, 0.0]},
        },
        "calibration": {"lidar_to_imu": {
            "rotation_validated_dynamically": False,
            "temporal_offset_validated_dynamically": False,
            "time_offset_lidar_to_imu_sec": 0.0,
        }},
    }, sort_keys=False), encoding="utf-8")
    report = tmp_path / "validation.yaml"
    apply_calibration(first, second, hardware, report)
    updated = yaml.safe_load(hardware.read_text(encoding="utf-8"))
    calibration = updated["calibration"]["lidar_to_imu"]
    assert calibration["rotation_validated_dynamically"] is True
    assert calibration["temporal_offset_validated_dynamically"] is True
    assert calibration["time_offset_lidar_to_imu_sec"] == pytest.approx(0.004)
    assert np.allclose(
        updated["geometry"]["base_link_to_imu"]["rpy_rad"], [0.0, 0.0, 0.0]
    )
    assert report.is_file()
