"""Build a unified demonstration dataset and a conservative planner envelope."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from roka_robot_description.workspace import workspace_root

from .safe_path import DEFAULT_EDGE_MARGIN_M, TIRE_HALF_WIDTH_M


PATH_POINT_COUNT = 858
PLANNER_LINEAR_LIMIT_MPS = 0.55
PLANNER_ANGULAR_LIMIT_RADPS = 0.80
RISK_HORIZON_SEC = 1.50
STALL_COMMAND_MIN_MPS = 0.15
STALL_SPEED_RATIO = 0.35
STALL_HOLD_SEC = 0.80

FEATURE_NAMES = (
    "candidate_speed_ratio",
    "candidate_speed_squared",
    "abs_cross_slope_ratio",
    "abs_longitudinal_slope_ratio",
    "curvature_ratio",
    "edge_deficit_ratio",
    "speed_cross_interaction",
    "speed_longitudinal_interaction",
    "speed_curvature_interaction",
)

DYNAMIC_NAMES = (
    "roll_deg",
    "pitch_deg",
    "forward_speed_mps",
    "lateral_speed_mps",
    "yaw_rate_radps",
    "lateral_error_m",
    "heading_error_deg",
    "tire_edge_clearance_m",
    "slip_ratio",
)


def _float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    if value in (None, ""):
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result


def _unwrap_progress(indices: list[int], count: int = PATH_POINT_COUNT) -> float:
    total = 0
    half = count // 2
    for before, after in zip(indices, indices[1:]):
        delta = after - before
        if delta > half:
            delta -= count
        elif delta < -half:
            delta += count
        total += delta
    return total / count if count else 0.0


def path_curvature(profile_file: Path) -> np.ndarray:
    document = yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {}
    points = list(document.get("points", []))
    if not points:
        raise ValueError("path profile contains no points")
    result = np.zeros(len(points), dtype=np.float64)
    closed = bool(document.get("closed_loop", False))
    for index, point in enumerate(points):
        if index + 1 == len(points) and not closed:
            following = point
        else:
            following = points[(index + 1) % len(points)]
        distance = max(
            math.hypot(
                float(following["x"]) - float(point["x"]),
                float(following["y"]) - float(point["y"]),
            ),
            1.0e-3,
        )
        delta = math.atan2(
            math.sin(float(following["yaw"]) - float(point["yaw"])),
            math.cos(float(following["yaw"]) - float(point["yaw"])),
        )
        result[index] = abs(delta) / distance
    return result


def static_feature_vector(
    speed_mps: float,
    cross_slope_deg: float,
    longitudinal_slope_deg: float,
    curvature_radpm: float,
    tire_edge_clearance_m: float,
) -> np.ndarray:
    """Return monotone non-negative features shared with the real planner."""
    speed_ratio = max(0.0, abs(speed_mps)) / PLANNER_LINEAR_LIMIT_MPS
    cross_ratio = max(0.0, abs(cross_slope_deg)) / 35.0
    longitudinal_ratio = max(0.0, abs(longitudinal_slope_deg)) / 35.0
    curvature_ratio = max(0.0, abs(curvature_radpm)) / 1.5
    if math.isfinite(tire_edge_clearance_m):
        edge_deficit = max(
            0.0,
            (DEFAULT_EDGE_MARGIN_M - tire_edge_clearance_m) / 0.15,
        )
    else:
        edge_deficit = 1.0
    return np.asarray([
        speed_ratio,
        speed_ratio * speed_ratio,
        cross_ratio,
        longitudinal_ratio,
        curvature_ratio,
        edge_deficit,
        speed_ratio * cross_ratio,
        speed_ratio * longitudinal_ratio,
        speed_ratio * curvature_ratio,
    ], dtype=np.float64)


def _mark_sustained_stalls(rows: list[dict[str, Any]]) -> np.ndarray:
    stalled = np.zeros(len(rows), dtype=bool)
    start: int | None = None
    for index, row in enumerate(rows + [None]):
        condition = False
        if row is not None:
            command = abs(float(row["command_linear_mps"]))
            speed = abs(float(row["forward_speed_mps"]))
            condition = (
                command >= STALL_COMMAND_MIN_MPS
                and speed < STALL_SPEED_RATIO * command
            )
            if index > 0 and float(row["time_sec"]) - float(
                rows[index - 1]["time_sec"]
            ) > 0.20:
                condition = False
        if condition and start is None:
            start = index
        elif not condition and start is not None:
            end = index - 1
            duration = float(rows[end]["time_sec"]) - float(
                rows[start]["time_sec"]
            )
            if duration >= STALL_HOLD_SEC:
                stalled[start:index] = True
            start = None
    return stalled


def _future_labels(
    times: np.ndarray,
    events: np.ndarray,
    horizon_sec: float = RISK_HORIZON_SEC,
) -> np.ndarray:
    result = np.zeros(len(events), dtype=bool)
    end = 0
    event_count = 0
    for start in range(len(events)):
        if end < start:
            end = start
            event_count = 0
        while end < len(events) and times[end] <= times[start] + horizon_sec:
            event_count += int(events[end])
            end += 1
        result[start] = event_count > 0
        event_count -= int(events[start])
    return result


def _normalize_session(
    path: Path,
    curvature: np.ndarray,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    enabled = [row for row in source if int(row.get("enabled", 0)) != 0]
    if not enabled:
        raise ValueError(f"{path.name} contains no enabled Xbox samples")
    indices = [int(row["path_index"]) for row in enabled]
    progress = _unwrap_progress(indices, len(curvature))
    completed = progress >= 0.90
    rows: list[dict[str, Any]] = []
    for row in enabled:
        index = min(max(int(row["path_index"]), 0), len(curvature) - 1)
        planned_cross = _float(row, "planned_cross_slope_deg", 0.0)
        planned_longitudinal = _float(
            row, "planned_longitudinal_slope_deg", 0.0
        )
        actual_cross = _float(row, "actual_cross_slope_deg", planned_cross)
        actual_longitudinal = _float(
            row, "actual_longitudinal_slope_deg", planned_longitudinal
        )
        if not math.isfinite(actual_cross):
            actual_cross = planned_cross
        if not math.isfinite(actual_longitudinal):
            actual_longitudinal = planned_longitudinal
        centre_edge = _float(row, "actual_wheel_center_edge_m")
        if not math.isfinite(centre_edge):
            centre_edge = _float(row, "actual_minimum_edge_m")
        tire_clearance = (
            centre_edge - TIRE_HALF_WIDTH_M
            if math.isfinite(centre_edge) else math.nan
        )
        roll = _float(row, "roll_deg", 0.0)
        pitch = _float(row, "pitch_deg", 0.0)
        lateral_error = _float(row, "lateral_error_m", 0.0)
        heading_error = _float(row, "heading_error_deg", 0.0)
        rideable = bool(int(row.get("actual_rideable", 0)))
        hard_boundary = (
            not rideable
            or (math.isfinite(tire_clearance) and tire_clearance < 0.0)
        )
        catastrophic = (
            not rideable
            or abs(roll) >= 70.0
            or abs(pitch) >= 55.0
            or abs(lateral_error) >= 0.50
        )
        warning = (
            hard_boundary
            or (
                math.isfinite(tire_clearance)
                and tire_clearance < DEFAULT_EDGE_MARGIN_M
            )
            or abs(lateral_error) >= 0.20
            or abs(heading_error) >= 20.0
        )
        command = _float(row, "cmd_linear_x_mps", 0.0)
        forward = _float(row, "forward_speed_mps", 0.0)
        slip_ratio = (
            abs(forward) / max(abs(command), 1.0e-3)
            if abs(command) >= STALL_COMMAND_MIN_MPS else 1.0
        )
        rows.append({
            "time_sec": _float(row, "sim_time_sec", 0.0),
            "session": path.stem,
            "path_index": index,
            "module_id": str(row.get("module_id", "")),
            "command_linear_mps": command,
            "command_angular_radps": _float(
                row, "cmd_angular_z_radps", 0.0
            ),
            "cross_slope_deg": actual_cross,
            "longitudinal_slope_deg": actual_longitudinal,
            "curvature_radpm": float(curvature[index]),
            "roll_deg": roll,
            "pitch_deg": pitch,
            "forward_speed_mps": forward,
            "lateral_speed_mps": _float(row, "lateral_speed_mps", 0.0),
            "yaw_rate_radps": _float(row, "yaw_rate_radps", 0.0),
            "lateral_error_m": lateral_error,
            "heading_error_deg": heading_error,
            "tire_edge_clearance_m": tire_clearance,
            "slip_ratio": slip_ratio,
            "hard_boundary": hard_boundary,
            "catastrophic": catastrophic,
            "warning": warning,
        })

    stalled = _mark_sustained_stalls(rows)
    catastrophic = np.asarray([row["catastrophic"] for row in rows])
    catastrophe_indices = np.flatnonzero(catastrophic)
    if not completed:
        failure_time = (
            float(rows[int(catastrophe_indices[0])]["time_sec"])
            if catastrophe_indices.size else float(rows[-1]["time_sec"])
        )
        failure_precursor = np.asarray([
            float(row["time_sec"]) >= failure_time - 3.0 for row in rows
        ])
    else:
        failure_precursor = np.zeros(len(rows), dtype=bool)
    events = np.asarray([
        bool(row["hard_boundary"]) for row in rows
    ]) | stalled | failure_precursor
    future_risk = _future_labels(
        np.asarray([float(row["time_sec"]) for row in rows]), events
    )
    for index, row in enumerate(rows):
        row["stalled"] = bool(stalled[index])
        row["failure_precursor"] = bool(failure_precursor[index])
        row["future_risk"] = bool(future_risk[index])
        row["planning_risk"] = bool(
            future_risk[index] or row["warning"]
        )
        row["static_features"] = static_feature_vector(
            row["command_linear_mps"],
            row["cross_slope_deg"],
            row["longitudinal_slope_deg"],
            row["curvature_radpm"],
            row["tire_edge_clearance_m"],
        )

    hard_fraction = float(np.mean([
        row["hard_boundary"] for row in rows
    ]))
    if not completed:
        role = "failed_negative"
    elif hard_fraction >= 0.05:
        role = "completed_boundary"
    else:
        role = "completed_reference"
    summary = {
        "session": path.stem,
        "source_file": str(path.resolve()),
        "rows": len(source),
        "enabled_rows": len(rows),
        "progress_laps": progress,
        "completed": completed,
        "role": role,
        "hard_boundary_rows": int(sum(row["hard_boundary"] for row in rows)),
        "warning_rows": int(sum(row["warning"] for row in rows)),
        "stalled_rows": int(np.count_nonzero(stalled)),
        "failure_precursor_rows": int(np.count_nonzero(failure_precursor)),
        "planning_risk_rows": int(sum(row["planning_risk"] for row in rows)),
        "maximum_command_linear_mps": max(
            abs(float(row["command_linear_mps"])) for row in rows
        ),
        "maximum_command_angular_radps": max(
            abs(float(row["command_angular_radps"])) for row in rows
        ),
        "maximum_abs_roll_deg": max(abs(float(row["roll_deg"])) for row in rows),
        "maximum_abs_pitch_deg": max(
            abs(float(row["pitch_deg"])) for row in rows
        ),
        "maximum_abs_lateral_error_m": max(
            abs(float(row["lateral_error_m"])) for row in rows
        ),
    }
    return rows, summary


def _weighted_auc(labels: np.ndarray, scores: np.ndarray, weights: np.ndarray) -> float:
    order = np.argsort(scores)
    labels = labels[order]
    weights = weights[order]
    negative_before = 0.0
    concordant = 0.0
    positive_total = float(np.sum(weights[labels == 1]))
    negative_total = float(np.sum(weights[labels == 0]))
    for label, weight in zip(labels, weights):
        if label:
            concordant += weight * negative_before
        else:
            negative_before += weight
    denominator = positive_total * negative_total
    return concordant / denominator if denominator > 0.0 else math.nan


def fit_monotone_risk_model(
    x: np.ndarray,
    labels: np.ndarray,
    session_weights: np.ndarray,
    iterations: int = 2500,
) -> tuple[dict[str, Any], dict[str, float]]:
    """Fit projected logistic regression whose difficulty weights stay positive."""
    mean = np.mean(x, axis=0)
    std = np.std(x, axis=0)
    std[std < 1.0e-6] = 1.0
    z = (x - mean) / std
    positive = labels > 0.5
    negative = ~positive
    weights = session_weights.copy()
    if np.any(positive) and np.any(negative):
        weights[positive] *= 0.5 / max(float(np.sum(weights[positive])), 1.0e-9)
        weights[negative] *= 0.5 / max(float(np.sum(weights[negative])), 1.0e-9)
    weights /= max(float(np.sum(weights)), 1.0e-9)
    coefficients = np.zeros(x.shape[1], dtype=np.float64)
    positive_rate = float(np.sum(weights * labels))
    bias = math.log(max(positive_rate, 1.0e-4) / max(1.0 - positive_rate, 1.0e-4))
    learning_rate = 0.08
    l2 = 0.015
    for _ in range(iterations):
        logits = np.clip(z @ coefficients + bias, -30.0, 30.0)
        prediction = 1.0 / (1.0 + np.exp(-logits))
        residual = (prediction - labels) * weights
        gradient = z.T @ residual + l2 * coefficients
        coefficients -= learning_rate * gradient
        coefficients = np.maximum(coefficients, 0.0)
        bias -= learning_rate * float(np.sum(residual))
    logits = np.clip(z @ coefficients + bias, -30.0, 30.0)
    prediction = 1.0 / (1.0 + np.exp(-logits))
    metrics = {
        "weighted_brier": float(np.sum(weights * (prediction - labels) ** 2)),
        "weighted_auc": _weighted_auc(labels.astype(int), prediction, weights),
        "positive_fraction_unweighted": float(np.mean(labels)),
    }
    model = {
        "feature_names": list(FEATURE_NAMES),
        "feature_mean": mean.tolist(),
        "feature_std": std.tolist(),
        "coefficients": coefficients.tolist(),
        "bias": bias,
        "monotone_non_decreasing": True,
    }
    return model, metrics


def _percentiles(rows: list[dict[str, Any]], key: str) -> dict[str, float]:
    values = np.asarray([
        abs(float(row[key])) for row in rows if math.isfinite(float(row[key]))
    ])
    if values.size == 0:
        return {"p95": math.nan, "p99": math.nan, "maximum": math.nan}
    return {
        "p95": float(np.percentile(values, 95.0)),
        "p99": float(np.percentile(values, 99.0)),
        "maximum": float(np.max(values)),
    }


def _weighted_percentile(
    values: np.ndarray, weights: np.ndarray, percentile: float
) -> float:
    order = np.argsort(values)
    values = values[order]
    weights = weights[order]
    cumulative = np.cumsum(weights)
    target = percentile / 100.0 * float(cumulative[-1])
    return float(values[min(int(np.searchsorted(cumulative, target)), len(values) - 1)])


def _bank_speed_floor_table(
    rows: list[dict[str, Any]], summaries: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Estimate a demonstrated lower speed floor for steep bank traversal."""
    completed = {
        str(item["session"]) for item in summaries if bool(item["completed"])
    }
    bins = ((0.0, 8.0), (8.0, 16.0), (16.0, 24.0), (24.0, 32.0), (32.0, 45.0))
    result = []
    for lower, upper in bins:
        selected = [
            row for row in rows
            if row["session"] in completed
            and not row["hard_boundary"]
            and abs(float(row["command_linear_mps"])) >= 0.05
            and lower <= abs(float(row["cross_slope_deg"])) < upper
        ]
        if selected:
            counts: dict[str, int] = {}
            for row in selected:
                counts[str(row["session"])] = counts.get(str(row["session"]), 0) + 1
            values = np.asarray([
                abs(float(row["command_linear_mps"])) for row in selected
            ])
            weights = np.asarray([
                1.0 / counts[str(row["session"])] for row in selected
            ])
            observed = _weighted_percentile(values, weights, 10.0)
        else:
            observed = 0.10
        # Mild cross-slope does not require momentum. On the steep bank, keep
        # the lower edge of speeds that were actually demonstrated without a
        # tyre-centre loss instead of reducing all the way to a crawl.
        floor = 0.10 if upper <= 16.0 else max(0.10, min(0.35, observed))
        result.append({
            "minimum_abs_cross_slope_deg": lower,
            "maximum_abs_cross_slope_deg": upper,
            "minimum_speed_mps": floor,
            "safe_samples": len(selected),
            "method": "session-balanced completed-lap p10",
        })
    return result


def build_dataset_and_envelope(
    input_files: list[Path],
    profile_file: Path,
) -> tuple[dict[str, np.ndarray], dict[str, Any], dict[str, Any]]:
    curvature = path_curvature(profile_file)
    all_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for path in input_files:
        rows, summary = _normalize_session(path, curvature)
        all_rows.extend(rows)
        summaries.append(summary)
    static = np.stack([row["static_features"] for row in all_rows])
    dynamic = np.asarray([
        [float(row[name]) for name in DYNAMIC_NAMES] for row in all_rows
    ], dtype=np.float64)
    labels = np.asarray([row["planning_risk"] for row in all_rows], dtype=np.uint8)
    sessions = np.asarray([str(row["session"]) for row in all_rows])
    unique, counts = np.unique(sessions, return_counts=True)
    count_by_session = dict(zip(unique.tolist(), counts.tolist()))
    session_weights = np.asarray([
        1.0 / count_by_session[str(session)] for session in sessions
    ], dtype=np.float64)
    model, metrics = fit_monotone_risk_model(
        static, labels.astype(np.float64), session_weights
    )
    completed_reference = [
        row for row in all_rows
        if next(
            item for item in summaries if item["session"] == row["session"]
        )["completed"] and not row["hard_boundary"]
    ]
    safe_static = static[np.asarray([
        next(item for item in summaries if item["session"] == row["session"])[
            "completed"
        ] and not row["planning_risk"] for row in all_rows
    ])]
    observed_max = np.percentile(safe_static, 99.0, axis=0)
    envelope = {
        "schema_version": 2,
        "purpose": "planner speed/cost calibration; not a standalone safety certificate",
        "planner_linear_limit_mps": PLANNER_LINEAR_LIMIT_MPS,
        "planner_angular_limit_radps": PLANNER_ANGULAR_LIMIT_RADPS,
        "tire_half_width_m": TIRE_HALF_WIDTH_M,
        "edge_safety_margin_m": DEFAULT_EDGE_MARGIN_M,
        "risk_horizon_sec": RISK_HORIZON_SEC,
        "risk_model": model,
        "risk_model_metrics": metrics,
        "planner": {
            "maximum_accepted_risk": 0.35,
            "minimum_speed_mps": 0.10,
            "speed_candidates_mps": [0.55, 0.50, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20, 0.15, 0.10],
            "maximum_lateral_acceleration_mps2": 0.25,
            "maximum_acceleration_mps2": 0.50,
            "maximum_deceleration_mps2": 0.80,
            "out_of_distribution_speed_mps": 0.10,
            "bank_speed_floor_table": _bank_speed_floor_table(
                all_rows, summaries
            ),
        },
        "observed_safe_feature_p99": dict(zip(FEATURE_NAMES, observed_max.tolist())),
        "successful_dynamic_envelope": {
            name: _percentiles(completed_reference, name) for name in DYNAMIC_NAMES
        },
        "sessions": summaries,
    }
    dataset = {
        "static_features": static.astype(np.float32),
        "dynamic_features": dynamic.astype(np.float32),
        "planning_risk": labels,
        "hard_boundary": np.asarray(
            [row["hard_boundary"] for row in all_rows], dtype=np.uint8
        ),
        "warning": np.asarray(
            [row["warning"] for row in all_rows], dtype=np.uint8
        ),
        "stalled": np.asarray(
            [row["stalled"] for row in all_rows], dtype=np.uint8
        ),
        "failure_precursor": np.asarray(
            [row["failure_precursor"] for row in all_rows], dtype=np.uint8
        ),
        "sessions": sessions,
        "path_indices": np.asarray(
            [row["path_index"] for row in all_rows], dtype=np.int32
        ),
        "module_ids": np.asarray([row["module_id"] for row in all_rows]),
        "feature_names": np.asarray(FEATURE_NAMES),
        "dynamic_names": np.asarray(DYNAMIC_NAMES),
    }
    report = {
        "schema_version": 2,
        "input_files": [str(path.resolve()) for path in input_files],
        "profile_file": str(profile_file.resolve()),
        "total_enabled_samples": len(all_rows),
        "planning_risk_samples": int(np.count_nonzero(labels)),
        "hard_boundary_samples": int(np.count_nonzero(dataset["hard_boundary"])),
        "stalled_samples": int(np.count_nonzero(dataset["stalled"])),
        "sessions": summaries,
        "limitations": [
            "four correlated MuJoCo sessions do not establish real-world limits",
            "risk labels depend on rasterized edge clearance and require field calibration",
            "the envelope must remain behind terrain and collision supervisors",
        ],
    }
    return dataset, envelope, report


def main() -> None:
    workspace = workspace_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-directory",
        type=Path,
        default=workspace / "datasets" / "xbox_demonstrations",
    )
    parser.add_argument(
        "--profile-file",
        type=Path,
        default=workspace / "maps" / "sim_pk3a_ground_truth" / "pk3a_safe_path.yaml",
    )
    parser.add_argument(
        "--dataset-file",
        type=Path,
        default=workspace / "datasets" / "processed" / "traversability_dataset_v2.npz",
    )
    parser.add_argument(
        "--envelope-file",
        type=Path,
        default=workspace / "models" / "traversability" / "traversability_envelope_v2.json",
    )
    arguments = parser.parse_args()
    inputs = sorted(arguments.input_directory.glob("*.csv"))
    if not inputs:
        raise FileNotFoundError(f"no demonstrations in {arguments.input_directory}")
    dataset, envelope, report = build_dataset_and_envelope(
        inputs, arguments.profile_file
    )
    arguments.dataset_file.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(arguments.dataset_file, **dataset)
    arguments.envelope_file.parent.mkdir(parents=True, exist_ok=True)
    arguments.envelope_file.write_text(
        json.dumps(envelope, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    report["dataset_file"] = str(arguments.dataset_file.resolve())
    report["envelope_file"] = str(arguments.envelope_file.resolve())
    report_file = arguments.dataset_file.with_name(
        f"{arguments.dataset_file.stem}_report.json"
    )
    report_file.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
