"""Apply a demonstration-calibrated traversability envelope to a track profile."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def static_feature_vector(
    speed_mps: float,
    cross_slope_deg: float,
    longitudinal_slope_deg: float,
    curvature_radpm: float,
    tire_edge_clearance_m: float,
    linear_limit_mps: float,
    edge_margin_m: float,
) -> np.ndarray:
    speed_ratio = max(0.0, abs(speed_mps)) / linear_limit_mps
    cross_ratio = max(0.0, abs(cross_slope_deg)) / 35.0
    longitudinal_ratio = max(0.0, abs(longitudinal_slope_deg)) / 35.0
    curvature_ratio = max(0.0, abs(curvature_radpm)) / 1.5
    if math.isfinite(tire_edge_clearance_m):
        edge_deficit = max(
            0.0, (edge_margin_m - tire_edge_clearance_m) / 0.15
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


def risk_probability(envelope: dict[str, Any], features: np.ndarray) -> float:
    model = envelope["risk_model"]
    mean = np.asarray(model["feature_mean"], dtype=np.float64)
    std = np.asarray(model["feature_std"], dtype=np.float64)
    coefficients = np.asarray(model["coefficients"], dtype=np.float64)
    if features.shape != mean.shape or mean.shape != coefficients.shape:
        raise ValueError("traversability envelope feature schema mismatch")
    logit = float(((features - mean) / std) @ coefficients + model["bias"])
    logit = max(-30.0, min(30.0, logit))
    return 1.0 / (1.0 + math.exp(-logit))


def profile_curvature(points: list[dict[str, Any]], closed: bool) -> np.ndarray:
    result = np.zeros(len(points), dtype=np.float64)
    for index, point in enumerate(points):
        before_index = (index - 1) % len(points) if closed else max(index - 1, 0)
        after_index = (index + 1) % len(points) if closed else min(index + 1, len(points) - 1)
        before = points[before_index]
        after = points[after_index]
        dx = float(after["x"]) - float(before["x"])
        dy = float(after["y"]) - float(before["y"])
        distance = max(math.hypot(dx, dy), 1.0e-3)
        before_yaw = float(before.get("yaw", point.get("yaw", 0.0)))
        after_yaw = float(after.get("yaw", point.get("yaw", 0.0)))
        result[index] = abs(wrap_angle(after_yaw - before_yaw)) / distance
    return result


def _terrain_value(point: dict[str, Any], names: tuple[str, ...]) -> float | None:
    for name in names:
        if name in point:
            value = float(point[name])
            if math.isfinite(value):
                return value
    return None


def _point_conditions(
    point: dict[str, Any], envelope: dict[str, Any]
) -> tuple[float | None, float | None, float | None]:
    cross = _terrain_value(point, ("cross_slope_deg", "lateral_slope_deg"))
    longitudinal = _terrain_value(point, ("longitudinal_slope_deg",))
    tire_clearance = _terrain_value(point, ("tire_edge_clearance_m",))
    if tire_clearance is None:
        centre_edge = _terrain_value(
            point,
            (
                "minimum_wheel_edge_m",
                "minimum_wheel_center_edge_m",
                "minimum_edge_m",
            ),
        )
        if centre_edge is not None:
            tire_clearance = centre_edge - float(envelope["tire_half_width_m"])
    return cross, longitudinal, tire_clearance


def _out_of_distribution(
    envelope: dict[str, Any], features: np.ndarray
) -> bool:
    names = list(envelope["risk_model"]["feature_names"])
    observed = envelope["observed_safe_feature_p99"]
    checked = {
        "abs_cross_slope_ratio",
        "abs_longitudinal_slope_ratio",
        "curvature_ratio",
        "edge_deficit_ratio",
    }
    return any(
        features[index] > 1.20 * max(float(observed[name]), 1.0e-6)
        for index, name in enumerate(names) if name in checked
    )


def _bank_speed_floor(planner: dict[str, Any], cross_slope_deg: float) -> float:
    absolute = abs(cross_slope_deg)
    for row in planner.get("bank_speed_floor_table", []):
        if (
            float(row["minimum_abs_cross_slope_deg"]) <= absolute
            < float(row["maximum_abs_cross_slope_deg"])
        ):
            return float(row["minimum_speed_mps"])
    return float(planner["minimum_speed_mps"])


def _smooth_speed_caps(
    points: list[dict[str, Any]],
    speeds: np.ndarray,
    closed: bool,
    acceleration: float,
    deceleration: float,
) -> np.ndarray:
    result = speeds.copy()
    count = len(points)
    passes = 2 if closed else 1
    for _ in range(passes):
        for index in range(1, count):
            before = points[index - 1]
            point = points[index]
            distance = max(
                math.hypot(
                    float(point["x"]) - float(before["x"]),
                    float(point["y"]) - float(before["y"]),
                ),
                1.0e-3,
            )
            reachable = math.sqrt(max(result[index - 1] ** 2 + 2.0 * acceleration * distance, 0.0))
            result[index] = min(result[index], reachable)
        for index in range(count - 2, -1, -1):
            point = points[index]
            after = points[index + 1]
            distance = max(
                math.hypot(
                    float(after["x"]) - float(point["x"]),
                    float(after["y"]) - float(point["y"]),
                ),
                1.0e-3,
            )
            reachable = math.sqrt(max(result[index + 1] ** 2 + 2.0 * deceleration * distance, 0.0))
            result[index] = min(result[index], reachable)
        if closed:
            seam = min(result[0], result[-1])
            result[0] = seam
            result[-1] = seam
    return result


def annotate_profile(
    document: dict[str, Any],
    envelope: dict[str, Any],
    allow_missing_terrain: bool = False,
    maximum_speed_mps: float | None = None,
) -> dict[str, Any]:
    points = [dict(point) for point in document.get("points", [])]
    if len(points) < 2:
        raise ValueError("track profile requires at least two points")
    closed = bool(document.get("closed_loop", False))
    curvature = profile_curvature(points, closed)
    planner = envelope["planner"]
    model_linear_limit = float(envelope["planner_linear_limit_mps"])
    profile_linear_limit = model_linear_limit
    if maximum_speed_mps is not None:
        if maximum_speed_mps <= 0.0:
            raise ValueError("maximum_speed_mps must be positive")
        profile_linear_limit = min(
            model_linear_limit, float(maximum_speed_mps)
        )
    angular_limit = float(envelope["planner_angular_limit_radps"])
    edge_margin = float(envelope["edge_safety_margin_m"])
    minimum_speed = float(planner["minimum_speed_mps"])
    risk_limit = float(planner["maximum_accepted_risk"])
    lateral_accel = float(planner["maximum_lateral_acceleration_mps2"])
    candidates = [float(value) for value in planner["speed_candidates_mps"]]
    raw_speeds = np.zeros(len(points), dtype=np.float64)
    metadata: list[dict[str, Any]] = []
    for index, point in enumerate(points):
        cross, longitudinal, tire_clearance = _point_conditions(point, envelope)
        missing = [
            name for name, value in (
                ("cross_slope", cross),
                ("longitudinal_slope", longitudinal),
                ("edge_clearance", tire_clearance),
            ) if value is None
        ]
        if missing and not allow_missing_terrain:
            raise ValueError(
                f"point {index} lacks {', '.join(missing)}; finish terrain mapping first"
            )
        cross = 0.0 if cross is None else cross
        longitudinal = 0.0 if longitudinal is None else longitudinal
        tire_clearance = math.nan if tire_clearance is None else tire_clearance
        curve = float(curvature[index])
        curve_cap = math.sqrt(lateral_accel / max(curve, 1.0e-6))
        yaw_cap = angular_limit / max(curve, 1.0e-6)
        bank_floor = max(minimum_speed, _bank_speed_floor(planner, cross))
        if bank_floor > profile_linear_limit + 1.0e-9:
            raise ValueError(
                f"point {index} requires minimum bank speed {bank_floor:.3f} m/s "
                f"above requested maximum {profile_linear_limit:.3f} m/s"
            )
        selected = bank_floor
        selected_risk = 1.0
        selected_features = static_feature_vector(
            selected, cross, longitudinal, curve, tire_clearance,
            model_linear_limit, edge_margin,
        )
        for candidate in candidates:
            if (
                candidate < bank_floor
                or candidate > profile_linear_limit
                or candidate > curve_cap
                or candidate > yaw_cap
            ):
                continue
            features = static_feature_vector(
                candidate, cross, longitudinal, curve, tire_clearance,
                model_linear_limit, edge_margin,
            )
            probability = risk_probability(envelope, features)
            if probability <= risk_limit:
                selected = candidate
                selected_risk = probability
                selected_features = features
                break
        out_of_distribution = _out_of_distribution(envelope, selected_features)
        if missing or out_of_distribution:
            selected = max(
                bank_floor,
                min(selected, float(planner["out_of_distribution_speed_mps"])),
            )
            selected_features = static_feature_vector(
                selected, cross, longitudinal, curve, tire_clearance,
                model_linear_limit, edge_margin,
            )
            selected_risk = risk_probability(envelope, selected_features)
        raw_speeds[index] = max(minimum_speed, selected)
        metadata.append({
            "cross_slope_deg": cross,
            "longitudinal_slope_deg": longitudinal,
            "tire_edge_clearance_m": tire_clearance,
            "curvature_radpm": curve,
            "risk": selected_risk,
            "missing": missing,
            "out_of_distribution": out_of_distribution,
            "bank_speed_floor_mps": bank_floor,
        })

    speeds = _smooth_speed_caps(
        points,
        raw_speeds,
        closed,
        float(planner["maximum_acceleration_mps2"]),
        float(planner["maximum_deceleration_mps2"]),
    )
    for index, point in enumerate(points):
        info = metadata[index]
        features = static_feature_vector(
            float(speeds[index]),
            info["cross_slope_deg"],
            info["longitudinal_slope_deg"],
            info["curvature_radpm"],
            info["tire_edge_clearance_m"],
            model_linear_limit,
            edge_margin,
        )
        risk = risk_probability(envelope, features)
        point["speed_mps"] = float(speeds[index])
        point["demonstration_risk"] = float(risk)
        point["traversability_cost"] = float(min(1.0, max(0.0, risk)))
        point["curvature_radpm"] = float(info["curvature_radpm"])
        point["minimum_bank_speed_mps"] = float(info["bank_speed_floor_mps"])
        point["terrain_data_complete"] = not bool(info["missing"])
        if info["missing"]:
            point["planner_reason"] = "missing_terrain_data"
        elif info["out_of_distribution"]:
            point["planner_reason"] = "outside_demonstrated_envelope"
        elif risk > risk_limit:
            point["planner_reason"] = "minimum_speed_still_high_risk"
        else:
            point["planner_reason"] = "demonstrated_envelope"

    result = dict(document)
    result["schema_version"] = max(2, int(document.get("schema_version", 1)))
    result["default_speed_mps"] = float(min(speeds))
    result["traversability_envelope"] = {
        "schema_version": int(envelope["schema_version"]),
        "purpose": envelope["purpose"],
        "maximum_accepted_risk": risk_limit,
        "profile_speed_limit_mps": profile_linear_limit,
    }
    result["autonomy_approved"] = all(
        point["terrain_data_complete"]
        and point["planner_reason"] == "demonstrated_envelope"
        for point in points
    )
    result["approval_reason"] = (
        "terrain_and_demonstration_envelope_passed"
        if result["autonomy_approved"]
        else "terrain_or_demonstration_envelope_incomplete"
    )
    result["points"] = points
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-profile", type=Path, required=True)
    parser.add_argument("--envelope-file", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--allow-missing-terrain", action="store_true")
    parser.add_argument("--maximum-speed-mps", type=float)
    arguments = parser.parse_args()
    document = yaml.safe_load(
        arguments.input_profile.read_text(encoding="utf-8")
    ) or {}
    envelope = json.loads(arguments.envelope_file.read_text(encoding="utf-8"))
    result = annotate_profile(
        document,
        envelope,
        arguments.allow_missing_terrain,
        arguments.maximum_speed_mps,
    )
    arguments.output_profile.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_profile.write_text(
        yaml.safe_dump(result, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    points = result["points"]
    summary = {
        "output_profile": str(arguments.output_profile.resolve()),
        "points": len(points),
        "minimum_speed_mps": min(float(point["speed_mps"]) for point in points),
        "maximum_speed_mps": max(float(point["speed_mps"]) for point in points),
        "maximum_risk": max(
            float(point["demonstration_risk"]) for point in points
        ),
        "incomplete_terrain_points": sum(
            not bool(point["terrain_data_complete"]) for point in points
        ),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
