"""Derive provisional attitude/contact guards from PK-3A MuJoCo runs.

The output is intentionally marked provisional.  It has the same ROS parameter
shape that will later be regenerated from successful and failed field runs.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from roka_robot_description.workspace import workspace_root

from .traversability_dataset import PATH_POINT_COUNT, _unwrap_progress


WORKSPACE = workspace_root()
PREDICTION_HORIZON_SEC = 0.20


def _number(row: dict[str, str], name: str, fallback: str | None = None) -> float:
    for key in (name, fallback):
        if not key:
            continue
        try:
            value = float(row.get(key, ""))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return math.nan


def _round_half_up(value: float) -> float:
    return math.ceil(value * 2.0 - 1.0e-9) / 2.0


def _load_session(path: Path) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as stream:
        source = [
            row for row in csv.DictReader(stream)
            if int(row.get("enabled", "0") or 0) != 0
        ]
    if not source:
        raise ValueError(f"{path.name} contains no enabled samples")
    indices = [int(row["path_index"]) for row in source]
    completed = _unwrap_progress(indices, PATH_POINT_COUNT) >= 0.90
    samples: list[dict[str, float | bool]] = []
    previous_time = math.nan
    previous_vertical_speed = math.nan
    for row in source:
        roll = _number(row, "roll_deg")
        pitch = _number(row, "pitch_deg")
        cross = _number(
            row, "actual_cross_slope_deg", "planned_cross_slope_deg"
        )
        longitudinal = _number(
            row,
            "actual_longitudinal_slope_deg",
            "planned_longitudinal_slope_deg",
        )
        if not all(math.isfinite(value) for value in (roll, pitch, cross, longitudinal)):
            continue
        roll_error = roll - cross
        pitch_error = pitch + longitudinal
        roll_rate = _number(row, "roll_rate_radps", None)
        pitch_rate = _number(row, "pitch_rate_radps", None)
        speed = abs(_number(row, "forward_speed_mps", None))
        yaw_rate = abs(_number(row, "yaw_rate_radps", None))
        outward_roll_rate = (
            abs(math.degrees(roll_rate))
            if math.isfinite(roll_rate) and roll_error * roll_rate > 0.0
            else 0.0
        )
        outward_pitch_rate = (
            abs(math.degrees(pitch_rate))
            if math.isfinite(pitch_rate) and pitch_error * pitch_rate > 0.0
            else 0.0
        )
        equivalent_bank = math.degrees(
            math.atan2(speed * yaw_rate, 9.80665)
        ) if math.isfinite(speed) and math.isfinite(yaw_rate) else 0.0
        vertical_speed = _number(row, "vertical_speed_mps", None)
        time_sec = _number(row, "sim_time_sec", None)
        acceleration_norm_proxy = math.nan
        if (
            math.isfinite(previous_time)
            and math.isfinite(previous_vertical_speed)
            and math.isfinite(vertical_speed)
            and 0.005 <= time_sec - previous_time <= 0.20
        ):
            acceleration_norm_proxy = abs(
                (vertical_speed - previous_vertical_speed)
                / (time_sec - previous_time)
                + 9.80665
            )
        previous_time = time_sec
        previous_vertical_speed = vertical_speed
        samples.append({
            "time_sec": time_sec,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "relative_roll_deg": abs(roll_error),
            "relative_pitch_deg": abs(pitch_error),
            "dynamic_roll_load_deg": (
                abs(roll_error)
                + PREDICTION_HORIZON_SEC * outward_roll_rate
                + equivalent_bank
            ),
            "dynamic_pitch_load_deg": (
                abs(pitch_error)
                + PREDICTION_HORIZON_SEC * outward_pitch_rate
            ),
            "roll_rate_radps": abs(roll_rate),
            "pitch_rate_radps": abs(pitch_rate),
            "speed_mps": speed,
            "yaw_rate_radps": yaw_rate,
            "acceleration_norm_proxy_mps2": acceleration_norm_proxy,
            "unsafe": bool(int(row.get("unsafe_label", "0") or 0)),
        })
    return {
        "name": path.stem,
        "source": str(path.resolve()),
        "completed": completed,
        "samples": samples,
    }


def _model_rollover_geometry(scene_file: Path) -> dict[str, float]:
    root = ET.parse(scene_file).getroot()
    base = root.find(".//body[@name='base_link']")
    if base is None:
        raise ValueError("MuJoCo scene has no base_link")
    base_position = [float(value) for value in base.attrib["pos"].split()]
    inertial = base.find("inertial")
    if inertial is None:
        raise ValueError("MuJoCo base_link has no inertial")
    base_mass = float(inertial.attrib["mass"])
    base_com = [float(value) for value in inertial.attrib["pos"].split()]
    total_mass = base_mass
    weighted_x = base_mass * base_com[0]
    weighted_z = base_mass * (base_position[2] + base_com[2])
    wheel_x: list[float] = []
    wheel_y: list[float] = []
    wheel_radius = math.nan
    for wheel in base.findall("body"):
        if "wheel_link" not in wheel.attrib.get("name", ""):
            continue
        position = [float(value) for value in wheel.attrib["pos"].split()]
        collision = next(
            geom for geom in wheel.findall("geom")
            if "wheel_collision" in geom.attrib.get("name", "")
        )
        mass = float(collision.attrib["mass"])
        size = [float(value) for value in collision.attrib["size"].split()]
        wheel_radius = size[2]
        total_mass += mass
        weighted_x += mass * position[0]
        weighted_z += mass * base_position[2]
        wheel_x.append(position[0])
        wheel_y.append(position[1])
    ground_z = base_position[2] - wheel_radius
    com_x = weighted_x / total_mass
    com_height = weighted_z / total_mass - ground_z
    front_support = max(wheel_x) - com_x
    rear_support = com_x - min(wheel_x)
    lateral_support = min(abs(min(wheel_y)), abs(max(wheel_y)))
    return {
        "total_mass_kg": total_mass,
        "center_of_mass_height_m": com_height,
        "center_of_mass_x_m": com_x,
        "lateral_support_half_span_m": lateral_support,
        "front_support_m": front_support,
        "rear_support_m": rear_support,
        "static_rollover_deg": math.degrees(math.atan2(lateral_support, com_height)),
        "static_forward_pitchover_deg": math.degrees(math.atan2(front_support, com_height)),
        "static_reverse_pitchover_deg": math.degrees(math.atan2(rear_support, com_height)),
    }


def _statistics(samples: list[dict[str, float | bool]], key: str) -> dict[str, float]:
    values = np.asarray([
        float(sample[key]) for sample in samples
        if math.isfinite(float(sample[key]))
    ])
    return {
        "p95": float(np.percentile(values, 95.0)),
        "p99": float(np.percentile(values, 99.0)),
        "maximum": float(np.max(values)),
    }


def derive_envelope(input_directory: Path, scene_file: Path) -> dict[str, object]:
    sessions = [_load_session(path) for path in sorted(input_directory.glob("*.csv"))]
    successful = [
        sample
        for session in sessions
        if bool(session["completed"])
        for sample in session["samples"]  # type: ignore[index]
    ]
    failed = [
        sample
        for session in sessions
        if not bool(session["completed"])
        for sample in session["samples"]  # type: ignore[index]
    ]
    if not successful or not failed:
        raise ValueError("derivation needs completed and failed MuJoCo sessions")
    geometry = _model_rollover_geometry(scene_file)
    roll_stats = _statistics(successful, "dynamic_roll_load_deg")
    pitch_stats = _statistics(successful, "dynamic_pitch_load_deg")
    failed_roll_onsets = [
        float(sample["dynamic_roll_load_deg"])
        for sample in failed
        if bool(sample["unsafe"])
        and float(sample["dynamic_roll_load_deg"]) >= roll_stats["maximum"]
    ]
    failed_roll_onset = min(failed_roll_onsets) if failed_roll_onsets else math.nan

    # The operator requested a permissive envelope at 90% of the model's
    # static support-polygon limit. Warning/critical stages are deliberately
    # late so normal pump/bank motion is not derated prematurely.
    roll_hard = math.floor(float(geometry["static_rollover_deg"]) * 0.90 * 10.0) / 10.0
    roll_warning = math.floor(roll_hard * 0.80 * 10.0) / 10.0
    roll_critical = math.floor(roll_hard * 0.94 * 10.0) / 10.0

    static_pitch = min(
        float(geometry["static_forward_pitchover_deg"]),
        float(geometry["static_reverse_pitchover_deg"]),
    )
    pitch_hard = math.floor(static_pitch * 0.90 * 10.0) / 10.0
    pitch_warning = math.floor(pitch_hard * 0.80 * 10.0) / 10.0
    pitch_critical = math.floor(pitch_hard * 0.94 * 10.0) / 10.0

    failed_absolute_roll = [
        abs(float(sample["roll_deg"]))
        for sample in failed if bool(sample["unsafe"])
    ]
    absolute_roll_hard = max(
        85.0,
        math.floor((min(failed_absolute_roll) - 3.0) * 2.0) / 2.0
        if failed_absolute_roll else 85.0,
    )
    accel_stats = _statistics(successful, "acceleration_norm_proxy_mps2")
    limits = {
        "roll_warning_deg": roll_warning,
        "roll_critical_deg": roll_critical,
        "roll_hard_deg": roll_hard,
        "pitch_warning_deg": pitch_warning,
        "pitch_critical_deg": pitch_critical,
        "pitch_hard_deg": pitch_hard,
        "absolute_roll_hard_deg": absolute_roll_hard,
        "absolute_pitch_hard_deg": 65.0,
        "attitude_prediction_horizon_sec": PREDICTION_HORIZON_SEC,
        "attitude_hard_hold_sec": 0.08,
    }
    first_failed_roll_guard_time = min(
        (
            float(sample["time_sec"])
            for sample in failed
            if float(sample["dynamic_roll_load_deg"]) >= roll_hard
        ),
        default=math.nan,
    )
    first_failed_unsafe_time = min(
        (
            float(sample["time_sec"])
            for sample in failed if bool(sample["unsafe"])
        ),
        default=math.nan,
    )
    guard_lead = (
        first_failed_unsafe_time - first_failed_roll_guard_time
        if math.isfinite(first_failed_roll_guard_time)
        and math.isfinite(first_failed_unsafe_time)
        else math.nan
    )
    successful_roll_hard_samples = sum(
        float(sample["dynamic_roll_load_deg"]) >= roll_hard
        for sample in successful
    )
    successful_pitch_hard_samples = sum(
        float(sample["dynamic_pitch_load_deg"]) >= pitch_hard
        for sample in successful
    )
    successful_false_airborne_samples = sum(
        math.isfinite(float(sample["acceleration_norm_proxy_mps2"]))
        and float(sample["acceleration_norm_proxy_mps2"]) <= 3.0
        for sample in successful
    )
    return {
        "schema_version": 1,
        "status": "provisional_mujoco_only_replace_with_field_envelope",
        "purpose": "surface-relative roll/pitch and wheel-contact guard calibration",
        "inputs": [
            {
                "name": session["name"],
                "source": session["source"],
                "completed": session["completed"],
                "sample_count": len(session["samples"]),  # type: ignore[arg-type]
            }
            for session in sessions
        ],
        "surface_relative_convention": {
            "roll_error_deg": "imu_roll_deg - terrain_lateral_slope_deg",
            "pitch_error_deg": "imu_pitch_deg + terrain_longitudinal_slope_deg",
            "dynamic_roll_load": "projected roll error + atan(|v*yaw_rate|/g)",
        },
        "model_geometry": geometry,
        "successful_dynamic_roll_load": roll_stats,
        "successful_dynamic_pitch_load": pitch_stats,
        "failed_rollover_onset_dynamic_load_deg": failed_roll_onset,
        "successful_acceleration_norm_proxy": accel_stats,
        "limits": limits,
        "replay_validation": {
            "successful_roll_hard_samples": successful_roll_hard_samples,
            "successful_pitch_hard_samples": successful_pitch_hard_samples,
            "successful_false_airborne_samples": successful_false_airborne_samples,
            "first_failed_roll_guard_time_sec": first_failed_roll_guard_time,
            "first_failed_unsafe_time_sec": first_failed_unsafe_time,
            "roll_guard_lead_time_sec": guard_lead,
            "pass": (
                successful_roll_hard_samples == 0
                and successful_pitch_hard_samples == 0
                and successful_false_airborne_samples == 0
                and math.isfinite(guard_lead)
                and guard_lead >= float(limits["attitude_hard_hold_sec"])
            ),
        },
        "contact_estimator": {
            "airborne_accel_norm_max_mps2": 3.0,
            "landing_accel_norm_min_mps2": 13.0,
            "airborne_hold_sec": 0.05,
            "partial_contact_hold_sec": 0.08,
            "landing_hold_sec": 0.30,
            "note": (
                "completed MuJoCo logs contain no confirmed free-flight sample and no "
                "motor current; acceleration thresholds are model-informed provisional "
                "guards and partial-contact current thresholds require field calibration"
            ),
        },
        "limitations": [
            "MuJoCo mass, center of mass, friction and tire compliance remain provisional",
            "absolute_pitch_hard_deg retains the MuJoCo gross inversion guard because no pitch-over failure was recorded",
            "do not claim real-world safety until a separate field calibration and validation run replace this file",
        ],
    }


def ros_parameters(envelope: dict[str, object]) -> dict[str, object]:
    limits = dict(envelope["limits"])  # type: ignore[arg-type]
    contact = dict(envelope["contact_estimator"])  # type: ignore[arg-type]
    contact.pop("note", None)
    return {
        # Separate dictionaries are intentional: rcl_yaml_param_parser does
        # not support YAML anchors/aliases emitted for shared objects.
        "terrain_supervisor": {"ros__parameters": dict(limits)},
        "mapping_terrain_monitor": {"ros__parameters": dict(limits)},
        "wheel_contact_estimator": {"ros__parameters": contact},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-directory",
        type=Path,
        default=WORKSPACE / "datasets" / "xbox_demonstrations",
    )
    parser.add_argument(
        "--scene-file",
        type=Path,
        default=(
            WORKSPACE
            / "src/simulation/roka_mujoco/description/pk3a/models"
            / "pk3a_mapped_flat_scene.xml"
        ),
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=WORKSPACE / "models/traversability/mujoco_dynamic_safety_v1.json",
    )
    parser.add_argument(
        "--output-ros-yaml",
        type=Path,
        default=(
            WORKSPACE
            / "src/navigation/roka_terrain_supervisor/config"
            / "provisional_vehicle_limits.yaml"
        ),
    )
    arguments = parser.parse_args()
    envelope = derive_envelope(arguments.input_directory, arguments.scene_file)
    arguments.output_json.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_json.write_text(
        json.dumps(envelope, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    arguments.output_ros_yaml.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_ros_yaml.write_text(
        yaml.safe_dump(ros_parameters(envelope), sort_keys=False),
        encoding="utf-8",
    )
    print(json.dumps(envelope, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
