#!/usr/bin/env python3
"""Parse, compare and safely apply two dynamic LI-Init calibration runs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import re
import tempfile

import numpy as np
import yaml

from roka_robot_description.workspace import workspace_root


ROTATION_LIMIT_DEG = 2.0
TIME_OFFSET_LIMIT_SEC = 0.005
TRANSLATION_LIMIT_M = 0.05
REFINEMENT_ROTATION_WARNING_DEG = 3.0
REFINEMENT_TRANSLATION_WARNING_M = 0.15
# VLP-16 scan headers refer to the first packet while a complete revolution
# arrives roughly 0.10 s later.  LI-Init correlates cut-frame odometry states,
# so a bounded value slightly beyond one scan is valid; anything beyond two
# scans remains a hard failure.
MAXIMUM_ABS_TIME_OFFSET_SEC = 0.20
VLP16_CUT_FRAME_RATE_HZ = 30.0


def _vlp16_cut_frame_diagnostic(raw_offset_sec: float) -> dict:
    """Report the 30 Hz phase without discarding the absolute LI-Init lag.

    The old implementation folded every result independently into the phase
    interval around zero.  That converted the repeatable -0.10/-0.13 second
    VLP-16 lag into a few milliseconds and produced large FAST-LIO Z drift.
    Point timing removes motion *within* one scan; it does not authorize
    deleting the absolute scan/IMU timestamp lag.
    """
    period = 1.0 / VLP16_CUT_FRAME_RATE_HZ
    whole_frames = int(round(raw_offset_sec / period))
    residual = raw_offset_sec - whole_frames * period
    return {
        "raw_offset_sec": raw_offset_sec,
        "cut_frame_period_sec": period,
        "nearest_whole_cut_frames": whole_frames,
        "phase_residual_sec": residual,
    }


def _align_two_run_time_offsets(first: dict, second: dict) -> dict:
    """Put both raw LI-Init lags on one 30 Hz cut-frame branch.

    Prefer the run whose online refinement remained stable as the absolute
    branch anchor.  Only the other run may move by an integer cut-frame period;
    this resolves the planar-motion alias while retaining the measured
    approximately 0.13 second VLP-16 lag.
    """
    period = 1.0 / VLP16_CUT_FRAME_RATE_HZ
    first_raw = float(first["raw_time_offset_lidar_to_imu_sec"])
    second_raw = float(second["raw_time_offset_lidar_to_imu_sec"])
    first_stable = (first.get("refinement_diagnostics") or {}).get("stable") is True
    second_stable = (second.get("refinement_diagnostics") or {}).get("stable") is True
    if first_stable and not second_stable:
        first_shift = 0
        second_shift = int(round((first_raw - second_raw) / period))
        anchor = "run_1_stable_refinement"
    else:
        second_shift = 0
        first_shift = int(round((second_raw - first_raw) / period))
        anchor = (
            "run_2_stable_refinement"
            if second_stable and not first_stable
            else "run_2_deterministic_anchor"
        )
    first_aligned = first_raw + first_shift * period
    second_aligned = second_raw + second_shift * period
    return {
        "cut_frame_period_sec": period,
        "anchor": anchor,
        "run_1_whole_frame_shift": first_shift,
        "run_2_whole_frame_shift": second_shift,
        "run_1_aligned_offset_sec": first_aligned,
        "run_2_aligned_offset_sec": second_aligned,
        "difference_sec": abs(first_aligned - second_aligned),
        "mean_offset_sec": 0.5 * (first_aligned + second_aligned),
    }


def _floats(line: str, expected: int) -> list[float]:
    values = [
        float(value)
        for value in re.findall(
            r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", line
        )
    ]
    if len(values) != expected:
        raise ValueError(f"expected {expected} numeric values, got {len(values)}: {line}")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("calibration contains a non-finite value")
    return values


def _parse_result_section(section: str, stage: str) -> dict:
    rotation_match = re.search(
        r"Rotation LiDAR to IMU \(degree\)\s*=\s*([^\n]+)", section
    )
    translation_match = re.search(
        r"Translation LiDAR to IMU \(meter\)\s*=\s*([^\n]+)", section
    )
    time_match = re.search(
        r"Time Lag IMU to LiDAR \(second\)\s*=\s*([^\n]+)", section
    )
    matrix_match = re.search(
        r"Homogeneous Transformation Matrix from LiDAR to IMU:\s*\n"
        r"([^\n]+)\n([^\n]+)\n([^\n]+)\n([^\n]+)",
        section,
    )
    if not all((rotation_match, translation_match, time_match, matrix_match)):
        raise ValueError(f"LI-Init {stage} result is incomplete or unsupported")

    rotation_deg = _floats(rotation_match.group(1), 3)
    translation_m = _floats(translation_match.group(1), 3)
    time_offset_sec = _floats(time_match.group(1), 1)[0]
    homogeneous = np.asarray(
        [_floats(matrix_match.group(index), 4) for index in range(1, 5)],
        dtype=np.float64,
    )
    rotation = homogeneous[:3, :3]
    if not np.allclose(homogeneous[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-5):
        raise ValueError("LI-Init homogeneous matrix has an invalid final row")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2.0e-3):
        raise ValueError("LI-Init rotation matrix is not orthonormal")
    determinant = float(np.linalg.det(rotation))
    if abs(determinant - 1.0) > 2.0e-3:
        raise ValueError(f"LI-Init rotation determinant is invalid: {determinant:.6f}")
    if not np.allclose(homogeneous[:3, 3], translation_m, atol=2.0e-4):
        raise ValueError("LI-Init translation text and matrix disagree")
    matrix_rpy_deg = [math.degrees(value) for value in _matrix_rpy(rotation)]
    if not np.allclose(matrix_rpy_deg, rotation_deg, atol=0.10):
        raise ValueError("LI-Init rotation text and matrix disagree")
    if abs(time_offset_sec) > MAXIMUM_ABS_TIME_OFFSET_SEC:
        raise ValueError(
            f"absolute LiDAR-IMU time offset is implausible: {time_offset_sec:.6f}s"
        )
    timing = _vlp16_cut_frame_diagnostic(time_offset_sec)
    return {
        "stage": stage,
        "rotation_lidar_in_imu_deg": rotation_deg,
        "rotation_lidar_in_imu_matrix": rotation.tolist(),
        "translation_lidar_in_imu_m": translation_m,
        "raw_time_offset_lidar_to_imu_sec": time_offset_sec,
        "time_offset_lidar_to_imu_sec": time_offset_sec,
        "time_offset_normalization": timing,
    }


def parse_li_init_result(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    initialization_marker = "Initialization result:"
    refinement_marker = "Refinement result:"
    if initialization_marker not in text:
        raise ValueError("LI-Init batch initialization did not finish")
    initialization_section = text.split(initialization_marker, 1)[1]
    if refinement_marker in initialization_section:
        initialization_section = initialization_section.split(refinement_marker, 1)[0]
    selected = _parse_result_section(initialization_section, "batch_initialization")

    # A wheeled robot cannot excite translation and all rotations like a
    # hand-carried sensor.  LI-Init's later unconstrained online refinement can
    # therefore absorb planar odometry error into the rigid sensor transform.
    # Keep the batch solution as the candidate and report refinement drift as
    # a diagnostic.  Nothing is applied until a second independent batch run
    # repeats both rotation and time offset.
    refinement = None
    refinement_diagnostics = {
        "available": False,
        "stable": None,
    }
    if refinement_marker in text:
        refinement = _parse_result_section(
            text.rsplit(refinement_marker, 1)[1], "online_refinement"
        )
        selected_rotation = np.asarray(
            selected["rotation_lidar_in_imu_matrix"], dtype=float
        )
        refinement_rotation = np.asarray(
            refinement["rotation_lidar_in_imu_matrix"], dtype=float
        )
        rotation_delta = _rotation_difference_deg(
            selected_rotation, refinement_rotation
        )
        translation_delta = float(np.linalg.norm(
            np.asarray(selected["translation_lidar_in_imu_m"], dtype=float)
            - np.asarray(refinement["translation_lidar_in_imu_m"], dtype=float)
        ))
        refinement_diagnostics = {
            "available": True,
            "stable": (
                rotation_delta <= REFINEMENT_ROTATION_WARNING_DEG
                and translation_delta <= REFINEMENT_TRANSLATION_WARNING_M
            ),
            "rotation_change_deg": rotation_delta,
            "translation_change_m": translation_delta,
            "limits": {
                "rotation_warning_deg": REFINEMENT_ROTATION_WARNING_DEG,
                "translation_warning_m": REFINEMENT_TRANSLATION_WARNING_M,
            },
            "reported_result": refinement,
        }

    return {
        "schema_version": 2,
        "algorithm": "LI-Init",
        "result": "PASS",
        "selected_stage": "batch_initialization",
        "rotation_lidar_in_imu_deg": selected["rotation_lidar_in_imu_deg"],
        "rotation_lidar_in_imu_matrix": selected["rotation_lidar_in_imu_matrix"],
        "translation_lidar_in_imu_m": selected["translation_lidar_in_imu_m"],
        # LI-Init and the local FAST-LIO implementation both subtract this
        # value from the IMU timestamp, so the sign is copied directly.
        "time_offset_lidar_to_imu_sec": selected["time_offset_lidar_to_imu_sec"],
        "raw_time_offset_lidar_to_imu_sec": selected[
            "raw_time_offset_lidar_to_imu_sec"
        ],
        "time_offset_policy": "preserve_raw_li_init_until_two_run_branch_alignment",
        "time_offset_normalization": selected["time_offset_normalization"],
        "translation_policy": "diagnostic_only_cad_translation_is_retained",
        "refinement_diagnostics": refinement_diagnostics,
        "source_result": str(path.resolve()),
    }


def _rotation_difference_deg(left: np.ndarray, right: np.ndarray) -> float:
    relative = left.T @ right
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def compare_runs(first: dict, second: dict) -> dict:
    first_rotation = np.asarray(first["rotation_lidar_in_imu_matrix"], dtype=float)
    second_rotation = np.asarray(second["rotation_lidar_in_imu_matrix"], dtype=float)
    rotation_delta = _rotation_difference_deg(first_rotation, second_rotation)
    time_alignment = _align_two_run_time_offsets(first, second)
    time_delta = float(time_alignment["difference_sec"])
    translation_delta = float(
        np.linalg.norm(
            np.asarray(first["translation_lidar_in_imu_m"], dtype=float)
            - np.asarray(second["translation_lidar_in_imu_m"], dtype=float)
        )
    )
    failures = []
    if rotation_delta > ROTATION_LIMIT_DEG:
        failures.append(
            f"rotation repeatability {rotation_delta:.3f}deg > {ROTATION_LIMIT_DEG:.3f}deg"
        )
    if time_delta > TIME_OFFSET_LIMIT_SEC:
        failures.append(
            f"time-offset repeatability {1000.0 * time_delta:.3f}ms > "
            f"{1000.0 * TIME_OFFSET_LIMIT_SEC:.3f}ms"
        )
    # The physical LiDAR/IMU translation comes from CAD and is never replaced
    # by the weakly observable wheeled-motion estimate.  Keep its repeatability
    # in the report, but do not use it to unlock mapping.
    return {
        "schema_version": 1,
        "result": "FAIL" if failures else "PASS",
        "limits": {
            "maximum_rotation_difference_deg": ROTATION_LIMIT_DEG,
            "maximum_time_offset_difference_sec": TIME_OFFSET_LIMIT_SEC,
            "maximum_translation_difference_m": TRANSLATION_LIMIT_M,
            "translation_is_gating": False,
        },
        "measured": {
            "rotation_difference_deg": rotation_delta,
            "time_offset_difference_sec": time_delta,
            "translation_difference_m": translation_delta,
        },
        "time_offset_alignment": time_alignment,
        "failures": failures,
    }


def _rpy_matrix(rpy) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _matrix_rpy(matrix: np.ndarray) -> list[float]:
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(float(-matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return [roll, pitch, yaw]


def _mean_rotation(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    u, _singular, vt = np.linalg.svd(first + second)
    mean = u @ vt
    if np.linalg.det(mean) < 0.0:
        u[:, -1] *= -1.0
        mean = u @ vt
    return mean


def apply_calibration(first: dict, second: dict, hardware_path: Path, report_path: Path):
    comparison = compare_runs(first, second)
    if comparison["result"] != "PASS":
        raise ValueError("two dynamic calibration runs are not repeatable")
    document = yaml.safe_load(hardware_path.read_text(encoding="utf-8")) or {}
    geometry = document["geometry"]
    lidar_rpy = geometry["base_link_to_lidar"]["rpy_rad"]
    lidar_in_imu = _mean_rotation(
        np.asarray(first["rotation_lidar_in_imu_matrix"], dtype=float),
        np.asarray(second["rotation_lidar_in_imu_matrix"], dtype=float),
    )
    # R_BI * R_IL = R_BL, therefore R_BI = R_BL * R_IL^T.
    base_to_imu = _rpy_matrix(lidar_rpy) @ lidar_in_imu.T
    imu_rpy = _matrix_rpy(base_to_imu)
    time_offset = float(comparison["time_offset_alignment"]["mean_offset_sec"])

    old_imu_rpy = list(geometry["base_link_to_imu"]["rpy_rad"])
    calibration = document.setdefault("calibration", {}).setdefault(
        "lidar_to_imu", {}
    )
    old_time_offset = float(calibration.get("time_offset_lidar_to_imu_sec", 0.0))
    geometry["base_link_to_imu"]["rpy_rad"] = [round(value, 10) for value in imu_rpy]
    calibration.update({
        "rotation_source": "li_init_dynamic_two_run_validation",
        "temporal_source": "li_init_dynamic_two_run_vlp16_common_cut_frame_branch",
        "rotation_validated_dynamically": True,
        "temporal_offset_validated_dynamically": True,
        "time_offset_lidar_to_imu_sec": round(time_offset, 9),
        "time_offset_policy": "preserve_raw_li_init_align_common_30hz_cut_frame_branch",
        "validated_at_utc": datetime.now(timezone.utc).isoformat(),
        "validation_report": "reports/lidar_imu_calibration/validation.yaml",
        "validation_repeatability": {
            "rotation_difference_deg": round(
                comparison["measured"]["rotation_difference_deg"], 6
            ),
            "time_offset_difference_sec": round(
                comparison["measured"]["time_offset_difference_sec"], 9
            ),
            "translation_difference_m": round(
                comparison["measured"]["translation_difference_m"], 6
            ),
        },
        "validated_rotation_lidar_in_imu_matrix": [
            [round(float(value), 10) for value in row] for row in lidar_in_imu
        ],
    })

    report = {
        **comparison,
        "algorithm": "LI-Init two-run dynamic validation",
        "run_1": first,
        "run_2": second,
        "applied": {
            "hardware_config": str(hardware_path.resolve()),
            "previous_base_to_imu_rpy_rad": old_imu_rpy,
            "base_to_imu_rpy_rad": imu_rpy,
            "previous_time_offset_sec": old_time_offset,
            "time_offset_lidar_to_imu_sec": time_offset,
        },
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")

    serialized = yaml.safe_dump(document, sort_keys=False)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=hardware_path.parent, delete=False
    ) as temporary:
        temporary.write(serialized)
        temporary_path = Path(temporary.name)
    os.replace(temporary_path, hardware_path)
    return report


def default_hardware_path() -> Path:
    return workspace_root() / "src/common/roka_robot_description/config/pk3a_hardware.yaml"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, help="LI-Init Initialization_result.txt")
    parser.add_argument("--write-run", type=Path, help="normalized run YAML")
    parser.add_argument("--run-1", type=Path)
    parser.add_argument("--run-2", type=Path)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--hardware-config", type=Path, default=default_hardware_path())
    parser.add_argument(
        "--report",
        type=Path,
        default=workspace_root() / "reports/lidar_imu_calibration/validation.yaml",
    )
    args = parser.parse_args()

    if args.result:
        run = parse_li_init_result(args.result.expanduser().resolve())
        rendered = yaml.safe_dump(run, sort_keys=False)
        if args.write_run:
            output = args.write_run.expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(rendered, encoding="utf-8")
        print(
            "[CALIBRATION][PASS] LI-Init batch candidate: "
            f"rotation={run['rotation_lidar_in_imu_deg']}deg, "
            f"time_offset={1000.0 * run['time_offset_lidar_to_imu_sec']:.3f}ms, "
            f"refinement_stable={run['refinement_diagnostics']['stable']}"
        )
        return

    if not args.run_1 or not args.run_2:
        parser.error("use --result, or provide both --run-1 and --run-2")
    first = yaml.safe_load(args.run_1.read_text(encoding="utf-8")) or {}
    second = yaml.safe_load(args.run_2.read_text(encoding="utf-8")) or {}
    comparison = compare_runs(first, second)
    if comparison["result"] != "PASS":
        print(yaml.safe_dump(comparison, sort_keys=False), end="")
        raise SystemExit(1)
    if args.apply:
        report = apply_calibration(
            first,
            second,
            args.hardware_config.expanduser().resolve(),
            args.report.expanduser().resolve(),
        )
        measured = report["measured"]
        print(
            "[CALIBRATION][APPLIED] two runs agree: "
            f"rotation_delta={measured['rotation_difference_deg']:.3f}deg, "
            f"time_delta={1000.0 * measured['time_offset_difference_sec']:.3f}ms; "
            "track 3D mapping calibration lock is released after rebuild"
        )
    else:
        print(yaml.safe_dump(comparison, sort_keys=False), end="")


if __name__ == "__main__":
    main()
