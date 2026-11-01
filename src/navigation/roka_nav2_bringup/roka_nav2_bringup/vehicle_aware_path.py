"""Choose a smooth, full-vehicle-safe path from a mapped 3D corridor."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from roka_nav2_bringup.pcd_terrain_profile import SpatialCloud, surface_features
from roka_nav2_bringup.risk_aware_profile import annotate_profile
from roka_nav2_bringup.route_geometry import prepare_centerline
from roka_robot_description.hardware_config import load_hardware_config


def _finite_pose(point: dict[str, Any]) -> tuple[float, float, float]:
    values = (
        float(point["x"]),
        float(point["y"]),
        float(point.get("yaw", 0.0)),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("raw path contains a non-finite pose")
    return values


def _candidate_offsets(
    track_width: float,
    vehicle_width: float,
    edge_margin: float,
    count: int,
) -> np.ndarray:
    if count < 3 or count % 2 == 0:
        raise ValueError("candidate_count must be an odd integer of at least three")
    maximum = 0.5 * (track_width - vehicle_width) - edge_margin
    if maximum <= 0.0:
        raise ValueError(
            "track width does not contain the measured vehicle width and edge margin"
        )
    # A manual mapping line is only a reference. Keep the search bounded so a
    # nominally wide map cannot pull the robot into unobserved space.
    maximum = min(maximum, 0.25)
    return np.linspace(-maximum, maximum, count, dtype=np.float64)


def _candidate_cost(
    features: dict[str, float],
    offset: float,
    maximum_offset: float,
    edge_margin: float,
) -> float:
    clearance = float(features["tire_edge_clearance_m"])
    if clearance < edge_margin:
        return math.inf
    cross = abs(float(features["cross_slope_deg"]))
    longitudinal = abs(float(features["longitudinal_slope_deg"]))
    roughness = max(0.0, float(features.get("surface_roughness_m", 0.0)))
    if cross > 35.0 or longitudinal > 35.0 or roughness > 0.06:
        return math.inf
    clearance_reserve = max(0.0, edge_margin + 0.08 - clearance)
    normalized_offset = abs(offset) / max(maximum_offset, 1.0e-6)
    return (
        2.5 * (cross / 35.0) ** 2
        + 1.5 * (longitudinal / 35.0) ** 2
        + 2.0 * (roughness / 0.06) ** 2
        + 3.0 * (clearance_reserve / 0.08) ** 2
        + 0.08 * normalized_offset ** 2
    )


def _select_open_path(
    rows: list[list[dict[str, Any] | None]],
    offsets: np.ndarray,
    source_points: list[dict[str, Any]],
) -> list[int]:
    count = len(rows)
    states = len(offsets)
    costs = np.full((count, states), math.inf, dtype=np.float64)
    parents = np.full((count, states), -1, dtype=np.int64)
    for candidate, entry in enumerate(rows[0]):
        if entry is not None:
            costs[0, candidate] = float(entry["cost"])
    for index in range(1, count):
        x0, y0, _ = _finite_pose(source_points[index - 1])
        x1, y1, _ = _finite_pose(source_points[index])
        distance = max(math.hypot(x1 - x0, y1 - y0), 0.03)
        maximum_step = max(0.06, 1.25 * distance)
        for candidate, entry in enumerate(rows[index]):
            if entry is None:
                continue
            for before in range(states):
                if not math.isfinite(float(costs[index - 1, before])):
                    continue
                change = abs(float(offsets[candidate] - offsets[before]))
                if change > maximum_step:
                    continue
                transition = 1.5 * (change / distance) ** 2
                total = costs[index - 1, before] + float(entry["cost"]) + transition
                if total < costs[index, candidate]:
                    costs[index, candidate] = total
                    parents[index, candidate] = before
    end = int(np.argmin(costs[-1]))
    if not math.isfinite(float(costs[-1, end])):
        raise ValueError("no continuous full-vehicle-safe candidate path exists")
    selected = [end]
    for index in range(count - 1, 0, -1):
        end = int(parents[index, end])
        if end < 0:
            raise ValueError("vehicle-aware candidate path is discontinuous")
        selected.append(end)
    selected.reverse()
    return selected


def _select_path(
    rows: list[list[dict[str, Any] | None]],
    offsets: np.ndarray,
    source_points: list[dict[str, Any]],
    closed: bool,
) -> list[int]:
    if not closed:
        return _select_open_path(rows, offsets, source_points)
    best: tuple[float, list[int]] | None = None
    for start, entry in enumerate(rows[0]):
        if entry is None:
            continue
        constrained = deepcopy(rows)
        constrained[0] = [None] * len(offsets)
        constrained[0][start] = entry
        try:
            selected = _select_open_path(constrained, offsets, source_points)
        except ValueError:
            continue
        seam = abs(float(offsets[selected[-1]] - offsets[start]))
        if seam > 0.08:
            continue
        score = sum(float(rows[index][choice]["cost"]) for index, choice in enumerate(selected))
        score += 2.0 * (seam / 0.05) ** 2
        if best is None or score < best[0]:
            best = score, selected
    if best is None:
        raise ValueError("no seam-continuous closed full-vehicle-safe path exists")
    return best[1]


def _recompute_yaws(points: list[dict[str, Any]], closed: bool) -> None:
    for index, point in enumerate(points):
        before = points[(index - 1) % len(points)] if closed else points[max(0, index - 1)]
        after = points[(index + 1) % len(points)] if closed else points[min(len(points) - 1, index + 1)]
        dx = float(after["x"]) - float(before["x"])
        dy = float(after["y"]) - float(before["y"])
        if math.hypot(dx, dy) > 1.0e-6:
            point["yaw"] = math.atan2(dy, dx)


def plan_vehicle_aware_profile(
    document: dict[str, Any],
    cloud_xyz: np.ndarray,
    envelope: dict[str, Any],
    *,
    maximum_speed_mps: float | None = None,
    vehicle_width_m: float | None = None,
    wheel_track_m: float | None = None,
    tire_half_width_m: float | None = None,
    candidate_count: int = 11,
) -> dict[str, Any]:
    """Plan and risk-annotate a smooth path inside a mapped surface corridor.

    The Xbox line is retained as a reference. Every lateral candidate must
    contain both tire corridors in the PCD and retain the configured edge
    margin. Dynamic programming prevents point-by-point lateral jumps.
    """
    hardware = load_hardware_config()
    vehicle_width_m = (
        hardware.navigation_width_m if vehicle_width_m is None else float(vehicle_width_m)
    )
    wheel_track_m = (
        hardware.physical_wheel_track_m if wheel_track_m is None else float(wheel_track_m)
    )
    tire_half_width_m = (
        hardware.tire_half_width_m if tire_half_width_m is None else float(tire_half_width_m)
    )
    points = [dict(point) for point in document.get("points", [])]
    if len(points) < 10:
        raise ValueError("track profile needs at least 10 points")
    track_width = float(document.get("track_width_m", 0.9144))
    edge_margin = float(envelope["edge_safety_margin_m"])
    offsets = _candidate_offsets(
        track_width, vehicle_width_m, edge_margin, candidate_count
    )
    index = SpatialCloud(cloud_xyz)
    query_radius = math.hypot(0.35, track_width / 2.0 + 0.30)
    rows: list[list[dict[str, Any] | None]] = []
    failure_count = 0
    for point in points:
        x, y, yaw = _finite_pose(point)
        row: list[dict[str, Any] | None] = []
        for offset in offsets:
            candidate_x = x - math.sin(yaw) * float(offset)
            candidate_y = y + math.cos(yaw) * float(offset)
            local_cloud = index.query(candidate_x, candidate_y, query_radius)
            try:
                features = surface_features(
                    local_cloud,
                    candidate_x,
                    candidate_y,
                    yaw,
                    track_width,
                    wheel_track_m,
                    tire_half_width_m,
                )
                cost = _candidate_cost(
                    features, float(offset), float(max(abs(offsets))), edge_margin
                )
                if not math.isfinite(cost):
                    raise ValueError("candidate violates vehicle terrain limits")
                row.append({
                    "x": candidate_x,
                    "y": candidate_y,
                    "offset": float(offset),
                    "features": features,
                    "cost": cost,
                })
            except (ValueError, np.linalg.LinAlgError):
                row.append(None)
                failure_count += 1
        if not any(entry is not None for entry in row):
            raise ValueError(
                f"no mapped full-vehicle surface candidate at path point {len(rows)}"
            )
        rows.append(row)

    closed = bool(document.get("closed_loop", False))
    selected = _select_path(rows, offsets, points, closed)
    planned = []
    for source, row, choice in zip(points, rows, selected):
        entry = row[choice]
        if entry is None:
            raise RuntimeError("selected vehicle-aware candidate is invalid")
        point = dict(source)
        point.update(entry["features"])
        point.update({
            "x": float(entry["x"]),
            "y": float(entry["y"]),
            "lateral_offset_from_demonstration_m": float(entry["offset"]),
            "terrain_source": "mapped_pcd_vehicle_wheel_corridors",
        })
        planned.append(point)
    _recompute_yaws(planned, closed)

    # Smooth before final approval, then recompute terrain at the exact points
    # that Nav2 will receive. Smoothing an already-approved path at runtime
    # would make its PCD slope/edge evidence refer to different coordinates.
    planned, centerline_geometry = prepare_centerline(
        planned,
        closed=closed,
        spacing_m=0.05,
        maximum_deviation_m=0.03,
    )
    smoothing_failures = []
    for point_number, point in enumerate(planned):
        x, y, yaw = _finite_pose(point)
        local_cloud = index.query(x, y, query_radius)
        try:
            point.update(surface_features(
                local_cloud,
                x,
                y,
                yaw,
                track_width,
                wheel_track_m,
                tire_half_width_m,
            ))
            point["terrain_source"] = "mapped_pcd_smoothed_vehicle_wheel_corridors"
        except (ValueError, np.linalg.LinAlgError) as error:
            smoothing_failures.append(f"{point_number}:{error}")
    if smoothing_failures:
        raise ValueError(
            "smoothed full-vehicle path lost mapped terrain support: "
            + "; ".join(smoothing_failures[:5])
        )

    candidate = dict(document)
    candidate["points"] = planned
    candidate["centerline_geometry"] = centerline_geometry
    candidate["profile_role"] = "safe_autonomy"
    candidate["source_profile"] = "track_profile.yaml"
    candidate["vehicle_aware_planner"] = {
        "method": "lateral_candidates_dynamic_programming",
        "vehicle_width_m": float(vehicle_width_m),
        "wheel_track_m": float(wheel_track_m),
        "tire_half_width_m": float(tire_half_width_m),
        "edge_safety_margin_m": edge_margin,
        "candidate_count": int(candidate_count),
        "maximum_lateral_search_m": float(max(abs(offsets))),
        "rejected_candidate_count": int(failure_count),
        "selected_offset_min_m": float(min(point["lateral_offset_from_demonstration_m"] for point in planned)),
        "selected_offset_max_m": float(max(point["lateral_offset_from_demonstration_m"] for point in planned)),
        "fail_closed": True,
    }
    prepared = annotate_profile(
        candidate,
        envelope,
        allow_missing_terrain=False,
        maximum_speed_mps=maximum_speed_mps,
    )
    prepared["approval_scope"] = "vehicle_aware_mapped_traversability"
    if prepared.get("autonomy_approved") is not True:
        raise ValueError(
            "vehicle-aware path lies outside the demonstrated risk envelope"
        )
    prepared["approval_reason"] = (
        "vehicle_footprint_terrain_and_demonstration_envelope_passed"
    )
    return prepared


def profile_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_vehicle_aware_profile(
    source_path: Path,
    output_path: Path,
    pcd_path: Path,
    envelope_path: Path,
    maximum_speed_mps: float,
) -> dict[str, Any]:
    from roka_nav2_bringup.pcd_terrain_profile import load_ascii_xyz

    document = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
    envelope = json.loads(envelope_path.read_text(encoding="utf-8"))
    result = plan_vehicle_aware_profile(
        document,
        load_ascii_xyz(pcd_path),
        envelope,
        maximum_speed_mps=maximum_speed_mps,
    )
    result["source_profile_sha256"] = profile_sha256(source_path)
    temporary = output_path.with_suffix(output_path.suffix + ".preparing")
    temporary.write_text(
        yaml.safe_dump(result, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(output_path)
    return result
