"""Prepare Xbox demonstrations and train a small NumPy behavior policy."""

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


LINEAR_SCALE = 0.30
ANGULAR_SCALE = 0.60
PATH_POINT_COUNT = 858
FEATURE_NAMES = (
    "lateral_error_m",
    "heading_error_sin",
    "heading_error_cos",
    "planned_cross_slope_deg",
    "planned_longitudinal_slope_deg",
    "actual_cross_slope_deg",
    "actual_longitudinal_slope_deg",
    "roll_deg",
    "pitch_deg",
    "forward_speed_mps",
    "lateral_speed_mps",
    "yaw_rate_radps",
    "tire_edge_clearance_m",
    "planned_lateral_offset_m",
    "path_curvature_radpm",
    "lookahead_cross_slope_deg",
)


def _finite(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def _path_features(profile_file: Path) -> dict[str, np.ndarray]:
    points = list((yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {})["points"])
    count = len(points)
    curvature = np.zeros(count, dtype=float)
    cross = np.asarray([float(point["cross_slope_deg"]) for point in points])
    for index, point in enumerate(points):
        following = points[(index + 1) % count]
        distance = max(
            math.hypot(
                float(following["x"]) - float(point["x"]),
                float(following["y"]) - float(point["y"]),
            ),
            1.0e-3,
        )
        yaw_delta = math.atan2(
            math.sin(float(following["yaw"]) - float(point["yaw"])),
            math.cos(float(following["yaw"]) - float(point["yaw"])),
        )
        curvature[index] = yaw_delta / distance
    lookahead_steps = max(1, int(round(0.35 / 0.08)))
    return {
        "curvature": curvature,
        "lookahead_cross": np.roll(cross, -lookahead_steps),
    }


def _unwrap_progress(indices: list[int], count: int = PATH_POINT_COUNT) -> float:
    if len(indices) < 2:
        return 0.0
    total = 0
    half = count // 2
    for before, after in zip(indices, indices[1:]):
        delta = after - before
        if delta > half:
            delta -= count
        elif delta < -half:
            delta += count
        total += delta
    return total / count


def _completed_lap_reference(
    lateral_samples: list[list[float]],
    heading_samples: list[list[float]],
) -> tuple[np.ndarray, np.ndarray]:
    """Build a smooth, periodic reference from the successful driven lap."""

    def fill(samples: list[list[float]]) -> np.ndarray:
        values = np.asarray([
            float(np.median(bucket)) if bucket else np.nan for bucket in samples
        ])
        valid = np.flatnonzero(np.isfinite(values))
        if valid.size == 0:
            raise RuntimeError("completed lap contains no usable path reference")
        count = len(values)
        extended_indices = np.concatenate((valid - count, valid, valid + count))
        extended_values = np.tile(values[valid], 3)
        filled = np.interp(np.arange(count), extended_indices, extended_values)
        # Nine path samples correspond to about 0.72 m. This removes joystick
        # jitter without flattening the deliberate downhill-side line.
        return np.mean(
            np.stack([np.roll(filled, offset) for offset in range(-4, 5)]),
            axis=0,
        )

    lateral = fill(lateral_samples)
    heading = np.unwrap(fill(heading_samples))
    heading = np.arctan2(np.sin(heading), np.cos(heading))
    return lateral, heading


def _feature_vector(
    row: dict[str, str],
    path_features: dict[str, np.ndarray],
    tire_clearance: float,
) -> np.ndarray:
    heading = math.radians(_finite(row, "heading_error_deg"))
    index = int(row["path_index"])
    return np.asarray([
        _finite(row, "lateral_error_m"),
        math.sin(heading),
        math.cos(heading),
        _finite(row, "planned_cross_slope_deg"),
        _finite(row, "planned_longitudinal_slope_deg"),
        _finite(row, "actual_cross_slope_deg"),
        _finite(row, "actual_longitudinal_slope_deg"),
        _finite(row, "roll_deg"),
        _finite(row, "pitch_deg"),
        _finite(row, "forward_speed_mps"),
        _finite(row, "lateral_speed_mps"),
        _finite(row, "yaw_rate_radps"),
        tire_clearance,
        _finite(row, "planned_lateral_offset_m"),
        float(path_features["curvature"][index]),
        float(path_features["lookahead_cross"][index]),
    ], dtype=np.float64)


def prepare_demonstrations(
    input_files: list[Path],
    profile_file: Path,
    downsample: int = 3,
) -> dict[str, Any]:
    path_features = _path_features(profile_file)
    imitation_x: list[np.ndarray] = []
    imitation_y: list[np.ndarray] = []
    weights: list[float] = []
    sessions: list[str] = []
    path_indices: list[int] = []
    risk_x: list[np.ndarray] = []
    risk_sessions: list[str] = []
    summaries: list[dict[str, Any]] = []
    reference_lateral_samples: list[list[float]] = [
        [] for _ in range(PATH_POINT_COUNT)
    ]
    reference_heading_samples: list[list[float]] = [
        [] for _ in range(PATH_POINT_COUNT)
    ]

    for path in input_files:
        rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
        enabled = [row for row in rows if int(row["enabled"]) != 0]
        indices = [int(row["path_index"]) for row in enabled]
        progress_laps = _unwrap_progress(indices)
        completed = progress_laps >= 0.90
        original_unsafe_times = [
            float(row["sim_time_sec"])
            for row in enabled
            if int(row["unsafe_label"]) != 0
        ]
        danger_start = (
            min(original_unsafe_times) - 5.0 if original_unsafe_times else math.inf
        )
        if completed:
            for row in enabled:
                try:
                    index = int(row["path_index"])
                    reference_lateral_samples[index].append(
                        _finite(row, "lateral_error_m")
                    )
                    reference_heading_samples[index].append(
                        math.radians(_finite(row, "heading_error_deg"))
                    )
                except (IndexError, KeyError, ValueError):
                    continue
        counts = {"imitation": 0, "risk": 0, "excluded": 0}
        imitation_sequence = 0
        for row in enabled:
            try:
                centre_edge = _finite(row, "actual_minimum_edge_m")
                tire_clearance = centre_edge - TIRE_HALF_WIDTH_M
                vector = _feature_vector(row, path_features, tire_clearance)
            except (KeyError, ValueError):
                counts["excluded"] += 1
                continue
            time = float(row["sim_time_sec"])
            hard_risk = (
                tire_clearance < 0.0
                or int(row["actual_rideable"]) == 0
                or (
                    not completed
                    and (
                        int(row["unsafe_label"]) != 0
                        or time >= danger_start
                    )
                )
            )
            if hard_risk:
                risk_x.append(vector)
                risk_sessions.append(path.stem)
                counts["risk"] += 1
            # A failed run is a negative example, never an imitation target.
            # The complete run remains usable even where the rasterized map
            # calls partial tyre overhang unsafe: MuJoCo proved it stayed up.
            if not completed:
                counts["excluded"] += 1
                continue
            if imitation_sequence % downsample == 0:
                command = np.asarray([
                    _finite(row, "cmd_linear_x_mps") / LINEAR_SCALE,
                    _finite(row, "cmd_angular_z_radps") / ANGULAR_SCALE,
                ])
                imitation_x.append(vector)
                imitation_y.append(np.clip(command, -1.0, 1.0))
                safety_clearance = tire_clearance - DEFAULT_EDGE_MARGIN_M
                weights.append(
                    1.0 if safety_clearance >= 0.0 and not hard_risk else 0.35
                )
                sessions.append(path.stem)
                path_indices.append(int(row["path_index"]))
                counts["imitation"] += 1
            imitation_sequence += 1
        summaries.append({
            "session": path.stem,
            "source_file": str(path.resolve()),
            "enabled_rows": len(enabled),
            "progress_laps": progress_laps,
            "completed": completed,
            "first_recorded_unsafe_time_sec": (
                min(original_unsafe_times) if original_unsafe_times else None
            ),
            **counts,
        })

    if len(imitation_x) < 100:
        raise RuntimeError("not enough safe Xbox demonstration samples")
    reference_lateral, reference_heading = _completed_lap_reference(
        reference_lateral_samples,
        reference_heading_samples,
    )
    return {
        "x": np.asarray(imitation_x, dtype=np.float64),
        "y": np.asarray(imitation_y, dtype=np.float64),
        "weights": np.asarray(weights, dtype=np.float64),
        "sessions": np.asarray(sessions),
        "path_indices": np.asarray(path_indices, dtype=np.int32),
        "risk_x": np.asarray(risk_x, dtype=np.float64),
        "risk_sessions": np.asarray(risk_sessions),
        "reference_lateral_offset_m": reference_lateral,
        "reference_heading_error_rad": reference_heading,
        "summaries": summaries,
    }


def _predict(parameters: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    hidden = np.tanh(x @ parameters["w1"] + parameters["b1"])
    return np.tanh(hidden @ parameters["w2"] + parameters["b2"])


def train_policy(
    x: np.ndarray,
    y: np.ndarray,
    weights: np.ndarray,
    path_indices: np.ndarray,
    epochs: int = 600,
    seed: int = 23,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    # Hold out a contiguous fifth of the lap. This is harder and more honest
    # than a random split of adjacent 30 Hz samples.
    validation = (path_indices >= 600) & (path_indices < 772)
    if np.count_nonzero(validation) < 100:
        validation = np.arange(x.shape[0]) % 5 == 0
    training = ~validation
    mean = np.mean(x[training], axis=0)
    std = np.std(x[training], axis=0)
    std[std < 1.0e-6] = 1.0
    normalized = (x - mean) / std
    x_train, y_train, w_train = normalized[training], y[training], weights[training]
    x_valid, y_valid = normalized[validation], y[validation]

    rng = np.random.default_rng(seed)
    hidden_size = 48
    parameters = {
        "w1": rng.normal(0.0, math.sqrt(2.0 / (x.shape[1] + hidden_size)), (x.shape[1], hidden_size)),
        "b1": np.zeros(hidden_size),
        "w2": rng.normal(0.0, math.sqrt(2.0 / (hidden_size + 2)), (hidden_size, 2)),
        "b2": np.zeros(2),
    }
    first = {name: np.zeros_like(value) for name, value in parameters.items()}
    second = {name: np.zeros_like(value) for name, value in parameters.items()}
    best = {name: value.copy() for name, value in parameters.items()}
    best_loss = math.inf
    best_epoch = 0
    patience = 80
    step = 0
    batch_size = min(256, len(x_train))
    learning_rate = 0.003
    l2 = 1.0e-5

    for epoch in range(epochs):
        order = rng.permutation(len(x_train))
        for start in range(0, len(order), batch_size):
            selection = order[start:start + batch_size]
            xb, yb, wb = x_train[selection], y_train[selection], w_train[selection]
            hidden = np.tanh(xb @ parameters["w1"] + parameters["b1"])
            prediction = np.tanh(hidden @ parameters["w2"] + parameters["b2"])
            denominator = max(float(np.sum(wb)) * y.shape[1], 1.0)
            output_gradient = (
                2.0 * (prediction - yb) * wb[:, None] / denominator
            )
            z2_gradient = output_gradient * (1.0 - prediction * prediction)
            gradients = {
                "w2": hidden.T @ z2_gradient + l2 * parameters["w2"],
                "b2": np.sum(z2_gradient, axis=0),
            }
            hidden_gradient = z2_gradient @ parameters["w2"].T
            z1_gradient = hidden_gradient * (1.0 - hidden * hidden)
            gradients["w1"] = xb.T @ z1_gradient + l2 * parameters["w1"]
            gradients["b1"] = np.sum(z1_gradient, axis=0)
            step += 1
            for name in parameters:
                first[name] = 0.9 * first[name] + 0.1 * gradients[name]
                second[name] = 0.999 * second[name] + 0.001 * gradients[name] ** 2
                first_hat = first[name] / (1.0 - 0.9**step)
                second_hat = second[name] / (1.0 - 0.999**step)
                parameters[name] -= learning_rate * first_hat / (
                    np.sqrt(second_hat) + 1.0e-8
                )
        valid_prediction = _predict(parameters, x_valid)
        valid_loss = float(np.mean((valid_prediction - y_valid) ** 2))
        if valid_loss < best_loss - 1.0e-6:
            best_loss = valid_loss
            best_epoch = epoch + 1
            best = {name: value.copy() for name, value in parameters.items()}
        elif epoch + 1 - best_epoch >= patience:
            break

    best["feature_mean"] = mean
    best["feature_std"] = std
    training_prediction = _predict(best, normalized[training])
    validation_prediction = _predict(best, normalized[validation])
    scales = np.asarray([LINEAR_SCALE, ANGULAR_SCALE])
    train_error = (training_prediction - y[training]) * scales
    valid_error = (validation_prediction - y[validation]) * scales
    baseline = np.average(y[training], axis=0, weights=weights[training])
    baseline_error = (baseline[None, :] - y[validation]) * scales

    def metrics(error: np.ndarray) -> dict[str, list[float]]:
        return {
            "mae": np.mean(np.abs(error), axis=0).tolist(),
            "rmse": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        }

    report = {
        "training_samples": int(np.count_nonzero(training)),
        "validation_samples": int(np.count_nonzero(validation)),
        "validation_path_index_range": [600, 771],
        "best_epoch": best_epoch,
        "normalized_validation_mse": best_loss,
        "output_units": ["m/s", "rad/s"],
        "training": metrics(train_error),
        "validation": metrics(valid_error),
        "constant_baseline_validation": metrics(baseline_error),
    }
    return best, report


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
        default=workspace / "datasets" / "processed" / "xbox_behavior_v1.npz",
    )
    parser.add_argument(
        "--model-file",
        type=Path,
        default=workspace / "models" / "learned" / "xbox_behavior_policy_v1.npz",
    )
    parser.add_argument("--epochs", type=int, default=600)
    arguments = parser.parse_args()
    inputs = sorted(arguments.input_directory.glob("*.csv"))
    if not inputs:
        raise FileNotFoundError(f"no Xbox CSV files in {arguments.input_directory}")
    prepared = prepare_demonstrations(inputs, arguments.profile_file)
    parameters, report = train_policy(
        prepared["x"],
        prepared["y"],
        prepared["weights"],
        prepared["path_indices"],
        epochs=arguments.epochs,
    )
    arguments.dataset_file.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        arguments.dataset_file,
        x=prepared["x"].astype(np.float32),
        y=prepared["y"].astype(np.float32),
        weights=prepared["weights"].astype(np.float32),
        sessions=prepared["sessions"],
        path_indices=prepared["path_indices"],
        risk_x=prepared["risk_x"].astype(np.float32),
        risk_sessions=prepared["risk_sessions"],
        reference_lateral_offset_m=prepared["reference_lateral_offset_m"].astype(
            np.float32
        ),
        reference_heading_error_rad=prepared["reference_heading_error_rad"].astype(
            np.float32
        ),
        feature_names=np.asarray(FEATURE_NAMES),
    )
    arguments.model_file.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        arguments.model_file,
        **parameters,
        feature_names=np.asarray(FEATURE_NAMES),
        output_scales=np.asarray([LINEAR_SCALE, ANGULAR_SCALE]),
        reference_lateral_offset_m=prepared["reference_lateral_offset_m"],
        reference_heading_error_rad=prepared["reference_heading_error_rad"],
    )
    result = {
        "schema_version": 1,
        "tire_half_width_m": TIRE_HALF_WIDTH_M,
        "edge_safety_margin_m": DEFAULT_EDGE_MARGIN_M,
        "input_files": [str(path.resolve()) for path in inputs],
        "sessions": prepared["summaries"],
        "imitation_samples": int(prepared["x"].shape[0]),
        "risk_samples": int(prepared["risk_x"].shape[0]),
        "reference_source": "completed Xbox lap median, circular 0.72 m smoothing",
        "feature_names": list(FEATURE_NAMES),
        "dataset_file": str(arguments.dataset_file.resolve()),
        "model_file": str(arguments.model_file.resolve()),
        "training_report": report,
    }
    report_path = arguments.model_file.with_name(
        f"{arguments.model_file.stem}_report.json"
    )
    report_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
