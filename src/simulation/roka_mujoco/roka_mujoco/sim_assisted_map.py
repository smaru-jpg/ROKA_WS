"""Fuse a real 2D mapping bundle with a metric MuJoCo terrain prior.

This tool does not claim to reconstruct an unobserved real height field.  It
keeps the real occupancy and recorded XY route authoritative, then transfers
MuJoCo elevation, bank, grade, edge clearance and demonstrated risk along the
same physical arc length.  The result is explicitly marked as a simulation
prior and cannot replace a FAST-LIO/SC-PGO map for field approval.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
import yaml

from roka_nav2_bringup.pcd_terrain_profile import annotate_profile_from_pcd
from roka_nav2_bringup.risk_aware_profile import annotate_profile
from roka_robot_description.workspace import workspace_path


DEFAULT_START_MODULE = "S00_DB1"
DEFAULT_LATERAL_SPACING_M = 0.05
DEFAULT_OUTSIDE_MARGIN_M = 0.20
# The lower apron is an edge sentinel, not a claimed real sidewall height.
# 0.30 m stays above the 0.12 m edge threshold after local raster medians mix
# adjacent high-grade route samples.
DEFAULT_TRACK_DROP_M = 0.30
DEFAULT_WALL_HEIGHT_M = 1.20
DEFAULT_WALL_SPACING_M = 0.10


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_yaml(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(document, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return document


def cumulative_distance(points: list[dict[str, Any]], closed: bool = False) -> np.ndarray:
    """Return point arc lengths; the optional closing segment is not appended."""
    if len(points) < 2:
        raise ValueError("a route needs at least two points")
    xy = np.asarray([[float(point["x"]), float(point["y"])] for point in points])
    if not np.all(np.isfinite(xy)):
        raise ValueError("route XY contains a non-finite value")
    lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    if np.any(lengths <= 1.0e-6):
        # Repeated points carry no metric information and break interpolation.
        raise ValueError("route contains coincident consecutive points")
    result = np.concatenate((np.asarray([0.0]), np.cumsum(lengths)))
    if closed and np.linalg.norm(xy[0] - xy[-1]) <= 1.0e-6:
        raise ValueError("closed profiles must not duplicate their first point")
    return result


def _closed_reference(
    points: list[dict[str, Any]],
) -> tuple[np.ndarray, list[dict[str, Any]], float]:
    arc = cumulative_distance(points)
    first = np.asarray([float(points[0]["x"]), float(points[0]["y"])])
    last = np.asarray([float(points[-1]["x"]), float(points[-1]["y"])])
    closing = float(np.linalg.norm(first - last))
    if closing <= 1.0e-6:
        raise ValueError("simulation closed route has no closing segment")
    total = float(arc[-1] + closing)
    return np.concatenate((arc, [total])), points + [dict(points[0])], total


def _wrapped_query(
    start_distance: float, route_distance: np.ndarray, total: float
) -> np.ndarray:
    if float(route_distance[-1]) >= total - 1.0e-6:
        raise ValueError(
            "one transferred route may not span a full MuJoCo lap; use real 3D mapping"
        )
    return np.mod(start_distance + route_distance, total)


def _interpolate_scalar(
    reference_arc: np.ndarray,
    reference_points: list[dict[str, Any]],
    query: np.ndarray,
    name: str,
    default: float = math.nan,
) -> np.ndarray:
    values = np.asarray(
        [float(point.get(name, default)) for point in reference_points],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError(f"simulation profile lacks finite {name}")
    return np.interp(query, reference_arc, values)


def _module_at(
    reference_arc: np.ndarray,
    reference_points: list[dict[str, Any]],
    query: np.ndarray,
) -> list[str]:
    indices = np.searchsorted(reference_arc, query, side="right") - 1
    indices = np.clip(indices, 0, len(reference_points) - 1)
    return [str(reference_points[int(index)].get("module_id", "")) for index in indices]


def transfer_simulation_terrain(
    real_document: dict[str, Any],
    simulation_document: dict[str, Any],
    start_module: str = DEFAULT_START_MODULE,
    maximum_speed_mps: float | None = None,
    envelope: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a real-XY route with a metric, low-confidence MuJoCo Z prior."""
    real_points = [dict(point) for point in real_document.get("points", [])]
    simulation_points = [
        dict(point) for point in simulation_document.get("points", [])
    ]
    if not real_points or not simulation_points:
        raise ValueError("real and simulation profiles must both contain points")
    if not bool(simulation_document.get("closed_loop", False)):
        raise ValueError("MuJoCo reference profile must be a closed loop")
    real_arc = cumulative_distance(real_points)
    reference_arc, reference_points, total = _closed_reference(simulation_points)
    starts = [
        index
        for index, point in enumerate(simulation_points)
        if str(point.get("module_id", "")) == start_module
    ]
    if not starts:
        raise ValueError(f"MuJoCo profile has no start module {start_module}")
    start_index = starts[0]
    start_distance = float(reference_arc[start_index])
    query = _wrapped_query(start_distance, real_arc, total)

    fields = {
        name: _interpolate_scalar(
            reference_arc, reference_points, query, name
        )
        for name in (
            "elevation_m",
            "cross_slope_deg",
            "longitudinal_slope_deg",
            "minimum_wheel_edge_m",
            "maximum_wheel_slope_deg",
            "mean_wheel_slope_deg",
            "demonstration_risk",
            "traversability_cost",
        )
    }
    module_ids = _module_at(reference_arc, reference_points, query)
    start_elevation = float(fields["elevation_m"][0])
    transferred: list[dict[str, Any]] = []
    for index, source in enumerate(real_points):
        point = dict(source)
        cross = float(fields["cross_slope_deg"][index])
        longitudinal = float(fields["longitudinal_slope_deg"][index])
        point.update({
            "z": float(fields["elevation_m"][index] - start_elevation),
            "roll": math.radians(cross),
            # Existing ROKA convention: pitch + longitudinal terrain slope =
            # surface-relative pitch error.
            "pitch": -math.radians(longitudinal),
            "cross_slope_deg": cross,
            "longitudinal_slope_deg": longitudinal,
            "minimum_wheel_edge_m": float(
                fields["minimum_wheel_edge_m"][index]
            ),
            "maximum_wheel_slope_deg": float(
                fields["maximum_wheel_slope_deg"][index]
            ),
            "mean_wheel_slope_deg": float(
                fields["mean_wheel_slope_deg"][index]
            ),
            "simulation_demonstration_risk": float(
                fields["demonstration_risk"][index]
            ),
            "simulation_traversability_cost": float(
                fields["traversability_cost"][index]
            ),
            "simulation_module_id": module_ids[index],
            "terrain_source": "mujoco_metric_arclength_prior",
            "terrain_confidence": 0.25,
        })
        transferred.append(point)

    result = dict(real_document)
    result.update({
        "schema_version": max(3, int(real_document.get("schema_version", 1))),
        "environment": "sim_assisted_real_xy",
        "closed_loop": bool(real_document.get("closed_loop", False)),
        "track_width_m": float(simulation_document["track_width_m"]),
        "autonomy_approved": False,
        "approval_reason": "simulation_terrain_prior_requires_real_3d_confirmation",
        "simulation_prior": {
            "method": "metric_arclength_same_start_module",
            "start_module": start_module,
            "real_xy_authoritative": True,
            "real_occupancy_authoritative": True,
            "mujoco_z_slope_edge_prior": True,
            "confidence": 0.25,
            "field_navigation_authorized": False,
            "reference_lap_length_m": total,
            "transferred_length_m": float(real_arc[-1]),
        },
        "points": transferred,
    })
    if envelope is not None:
        planned = annotate_profile(
            result,
            envelope,
            allow_missing_terrain=False,
            maximum_speed_mps=maximum_speed_mps,
        )
        planner_would_approve = bool(planned.get("autonomy_approved", False))
        planned["autonomy_approved"] = False
        planned["approval_reason"] = (
            "simulation_terrain_prior_requires_real_3d_confirmation"
        )
        planned["simulation_prior"]["planner_envelope_would_approve"] = (
            planner_would_approve
        )
        result = planned
    return result


def project_to_route(
    xy: np.ndarray, route_points: list[dict[str, Any]]
) -> tuple[np.ndarray, np.ndarray]:
    """Project XY samples onto an open route, returning arc length and signed offset."""
    route = np.asarray(
        [[float(point["x"]), float(point["y"])] for point in route_points],
        dtype=np.float64,
    )
    arc = cumulative_distance(route_points)
    best_distance = np.full(len(xy), np.inf, dtype=np.float64)
    best_arc = np.zeros(len(xy), dtype=np.float64)
    best_lateral = np.zeros(len(xy), dtype=np.float64)
    for index, (begin, end) in enumerate(zip(route[:-1], route[1:])):
        vector = end - begin
        length_squared = float(vector @ vector)
        delta = xy - begin
        ratio = np.clip((delta @ vector) / length_squared, 0.0, 1.0)
        projected = begin + ratio[:, None] * vector
        error = xy - projected
        distance_squared = np.einsum("ij,ij->i", error, error)
        better = distance_squared < best_distance
        segment_length = math.sqrt(length_squared)
        cross = vector[0] * error[:, 1] - vector[1] * error[:, 0]
        best_distance[better] = distance_squared[better]
        best_arc[better] = arc[index] + ratio[better] * segment_length
        best_lateral[better] = cross[better] / segment_length
    return best_arc, best_lateral


def _profile_samples(
    profile: dict[str, Any], arc_query: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = list(profile["points"])
    arc = cumulative_distance(points)
    elevation = np.interp(arc_query, arc, [float(point["z"]) for point in points])
    cross = np.interp(
        arc_query, arc, [float(point["cross_slope_deg"]) for point in points]
    )
    risk = np.interp(
        arc_query,
        arc,
        [float(point.get("traversability_cost", 1.0)) for point in points],
    )
    return elevation, cross, risk


def terrain_ribbon_points(
    profile: dict[str, Any],
    lateral_spacing_m: float = DEFAULT_LATERAL_SPACING_M,
    outside_margin_m: float = DEFAULT_OUTSIDE_MARGIN_M,
    drop_m: float = DEFAULT_TRACK_DROP_M,
) -> np.ndarray:
    """Sample a route-aligned 2.5D ribbon and a lower outside apron."""
    points = list(profile["points"])
    half_width = 0.5 * float(profile["track_width_m"])
    lateral = np.arange(
        -half_width - outside_margin_m,
        half_width + outside_margin_m + 0.5 * lateral_spacing_m,
        lateral_spacing_m,
    )
    rows: list[tuple[float, float, float, float]] = []
    for point in points:
        yaw = float(point["yaw"])
        center_z = float(point["z"])
        cross = math.tan(math.radians(float(point["cross_slope_deg"])))
        risk = float(point.get("traversability_cost", 1.0))
        for offset in lateral:
            clipped = min(half_width, max(-half_width, float(offset)))
            z = center_z + cross * clipped
            on_track = abs(offset) <= half_width + 1.0e-9
            if not on_track:
                z -= drop_m
            x = float(point["x"]) - math.sin(yaw) * offset
            y = float(point["y"]) + math.cos(yaw) * offset
            intensity = 32.0 if not on_track else 64.0 + 128.0 * risk
            rows.append((x, y, z, intensity))
    return np.asarray(rows, dtype=np.float32)


def _occupancy_cells(
    occupancy_yaml: Path,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    metadata = _load_yaml(occupancy_yaml)
    image_path = (occupancy_yaml.parent / str(metadata["image"])).resolve()
    image = np.asarray(Image.open(image_path).convert("L"))
    resolution = float(metadata["resolution"])
    origin_x, origin_y = [float(value) for value in metadata["origin"][:2]]
    rows, columns = image.shape
    column_grid, row_grid = np.meshgrid(np.arange(columns), np.arange(rows))
    x = origin_x + (column_grid + 0.5) * resolution
    y = origin_y + (rows - row_grid - 0.5) * resolution
    occupancy_probability = (255.0 - image.astype(np.float64)) / 255.0
    known = image != 205
    occupied = occupancy_probability >= float(metadata.get("occupied_thresh", 0.65))
    return metadata, np.column_stack((x.ravel(), y.ravel())), known.ravel(), occupied.ravel()


def lifted_occupancy(
    occupancy_yaml: Path,
    profile: dict[str, Any],
    wall_height_m: float = DEFAULT_WALL_HEIGHT_M,
    wall_spacing_m: float = DEFAULT_WALL_SPACING_M,
    drop_m: float = DEFAULT_TRACK_DROP_M,
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, int]]:
    metadata, xy, known, occupied = _occupancy_cells(occupancy_yaml)
    route_points = list(profile["points"])
    arc, lateral = project_to_route(xy, route_points)
    elevation, cross, risk = _profile_samples(profile, arc)
    half_width = 0.5 * float(profile["track_width_m"])
    clipped = np.clip(lateral, -half_width, half_width)
    ground = elevation + np.tan(np.radians(cross)) * clipped
    ground[np.abs(lateral) > half_width] -= drop_m

    wall_xy = xy[occupied]
    wall_ground = ground[occupied]
    levels = np.arange(0.0, wall_height_m + 0.5 * wall_spacing_m, wall_spacing_m)
    if len(wall_xy):
        repeated_xy = np.repeat(wall_xy, len(levels), axis=0)
        z = np.repeat(wall_ground, len(levels)) + np.tile(levels, len(wall_xy))
        intensity = np.full(len(z), 255.0, dtype=np.float64)
        walls = np.column_stack((repeated_xy, z, intensity)).astype(np.float32)
    else:
        walls = np.empty((0, 4), dtype=np.float32)

    image_shape = (
        int(round((max(xy[:, 1]) - min(xy[:, 1])) / float(metadata["resolution"]))) + 1,
        int(round((max(xy[:, 0]) - min(xy[:, 0])) / float(metadata["resolution"]))) + 1,
    )
    grid = {
        "elevation": np.where(known, ground, np.nan).reshape(image_shape).astype(np.float32),
        "cross_slope_deg": np.where(known, cross, np.nan).reshape(image_shape).astype(np.float32),
        "risk": np.where(known, risk, np.nan).reshape(image_shape).astype(np.float32),
        "lateral_distance_m": np.where(known, lateral, np.nan).reshape(image_shape).astype(np.float32),
        "known": known.reshape(image_shape),
        "occupied": occupied.reshape(image_shape),
        "sim_prior_traversable": (
            known & ~occupied & (np.abs(lateral) <= half_width)
        ).reshape(image_shape),
    }
    counts = {
        "cell_count": int(len(xy)),
        "known_cell_count": int(np.count_nonzero(known)),
        "occupied_cell_count": int(np.count_nonzero(occupied)),
        "prior_corridor_cell_count": int(
            np.count_nonzero(grid["sim_prior_traversable"])
        ),
    }
    return walls, grid, counts


def write_pcd(path: Path, points: np.ndarray) -> None:
    if points.ndim != 2 or points.shape[1] != 4 or not np.all(np.isfinite(points)):
        raise ValueError("PCD points must be finite Nx4 XYZI")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = "\n".join((
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z intensity",
        "SIZE 4 4 4 4",
        "TYPE F F F F",
        "COUNT 1 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    )) + "\n"
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        np.savetxt(stream, points, fmt="%.6f %.6f %.6f %.3f")


def build_sim_assisted_map(
    map_directory: Path,
    simulation_profile: Path,
    envelope_file: Path,
    output_directory: Path,
    start_module: str = DEFAULT_START_MODULE,
    maximum_speed_mps: float | None = None,
    dynamic_safety_file: Path | None = None,
) -> dict[str, Any]:
    map_directory = map_directory.expanduser().resolve()
    simulation_profile = simulation_profile.expanduser().resolve()
    envelope_file = envelope_file.expanduser().resolve()
    dynamic_safety_file = (
        workspace_path(
            "models", "traversability", "mujoco_dynamic_safety_v1.json"
        )
        if dynamic_safety_file is None
        else dynamic_safety_file.expanduser().resolve()
    )
    output_directory = output_directory.expanduser().resolve()
    real_profile_file = map_directory / "track_profile.yaml"
    occupancy_yaml = map_directory / "occupancy.yaml"
    for path in (
        real_profile_file,
        occupancy_yaml,
        simulation_profile,
        envelope_file,
        dynamic_safety_file,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
    real = _load_yaml(real_profile_file)
    simulation = _load_yaml(simulation_profile)
    envelope = json.loads(envelope_file.read_text(encoding="utf-8"))
    dynamic_safety = json.loads(dynamic_safety_file.read_text(encoding="utf-8"))
    profile = transfer_simulation_terrain(
        real,
        simulation,
        start_module=start_module,
        maximum_speed_mps=maximum_speed_mps,
        envelope=envelope,
    )
    ribbon = terrain_ribbon_points(profile)
    walls, grid, counts = lifted_occupancy(occupancy_yaml, profile)
    combined = np.vstack((ribbon, walls))

    output_directory.mkdir(parents=True, exist_ok=True)
    profile_file = output_directory / "track_profile_sim_assisted.yaml"
    profile_file.write_text(
        yaml.safe_dump(profile, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    terrain_pcd = output_directory / "terrain_prior.pcd"
    occupancy_pcd = output_directory / "occupancy_3d.pcd"
    visual_pcd = output_directory / "hybrid_visual.pcd"
    write_pcd(terrain_pcd, ribbon)
    write_pcd(occupancy_pcd, walls)
    write_pcd(visual_pcd, combined)
    extracted = annotate_profile_from_pcd(profile, ribbon[:, :3].astype(np.float64))
    residuals = np.asarray([
        (
            abs(
                float(source["cross_slope_deg"])
                - float(recovered["cross_slope_deg"])
            ),
            abs(
                float(source["longitudinal_slope_deg"])
                - float(recovered["longitudinal_slope_deg"])
            ),
        )
        for source, recovered in zip(profile["points"], extracted["points"])
    ])
    npz_file = output_directory / "hybrid_map.npz"
    np.savez_compressed(
        npz_file,
        visual_points=combined,
        resolution_m=np.float32(_load_yaml(occupancy_yaml)["resolution"]),
        **grid,
    )

    profile_points = list(profile["points"])
    elevations = np.asarray([float(point["z"]) for point in profile_points])
    cross = np.asarray([float(point["cross_slope_deg"]) for point in profile_points])
    longitudinal = np.asarray(
        [float(point["longitudinal_slope_deg"]) for point in profile_points]
    )
    modules = list(dict.fromkeys(
        str(point["simulation_module_id"]) for point in profile_points
    ))
    sensor_contract_file = map_directory / "sensor_contract.yaml"
    session_metrics_file = map_directory / "session_metrics.yaml"
    sensor_contract = (
        _load_yaml(sensor_contract_file) if sensor_contract_file.is_file() else {}
    )
    session_metrics = (
        _load_yaml(session_metrics_file) if session_metrics_file.is_file() else {}
    )
    contract_topics = sensor_contract.get("topics", {})
    timing = sensor_contract.get("lidar_imu_timing", {})
    excitation = sensor_contract.get("calibration_excitation", {})
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "SIMULATION_PRIOR_ONLY",
        "field_navigation_authorized": False,
        "reason": (
            "real 2D occupancy has no observed height; MuJoCo Z/slope is a low-confidence prior"
        ),
        "fusion_policy": {
            "xy_route": "real_mapping",
            "occupancy": "real_2d_mapping",
            "z_bank_grade_edge": "mujoco_metric_arclength_prior",
            "speed_schedule": "mujoco_demonstration_envelope_provisional",
            "live_field_override": "VLP16+EBIMU+FAST-LIO/terrain mapper required",
        },
        "alignment": {
            "method": "metric_arclength_same_start_module",
            "start_module": start_module,
            "transferred_modules": modules,
            "real_path_length_m": float(cumulative_distance(profile_points)[-1]),
            "simulation_reference_lap_length_m": float(
                simulation["simulation_prior"]["reference_lap_length_m"]
            ) if "simulation_prior" in simulation else float(
                _closed_reference(list(simulation["points"]))[2]
            ),
            "scale": 1.0,
        },
        "transferred_terrain": {
            "minimum_relative_elevation_m": float(np.min(elevations)),
            "maximum_relative_elevation_m": float(np.max(elevations)),
            "maximum_absolute_cross_slope_deg": float(np.max(np.abs(cross))),
            "maximum_absolute_longitudinal_slope_deg": float(
                np.max(np.abs(longitudinal))
            ),
            "minimum_provisional_speed_mps": float(
                min(float(point["speed_mps"]) for point in profile_points)
            ),
            "maximum_provisional_speed_mps": float(
                max(float(point["speed_mps"]) for point in profile_points)
            ),
            "planner_would_approve": bool(
                profile.get("simulation_prior", {}).get(
                    "planner_envelope_would_approve", False
                )
            ),
            "outside_demonstrated_envelope_points": int(sum(
                point.get("planner_reason") == "outside_demonstrated_envelope"
                for point in profile_points
            )),
        },
        "real_2d_map": counts,
        "terrain_prior_round_trip": {
            "existing_pcd_terrain_extractor": "PASS",
            "cross_slope_residual_deg": {
                "median": float(np.percentile(residuals[:, 0], 50)),
                "p95": float(np.percentile(residuals[:, 0], 95)),
                "maximum": float(np.max(residuals[:, 0])),
            },
            "longitudinal_slope_residual_deg": {
                "median": float(np.percentile(residuals[:, 1], 50)),
                "p95": float(np.percentile(residuals[:, 1], 95)),
                "maximum": float(np.max(residuals[:, 1])),
            },
        },
        "real_recording_quality": {
            "sensor_contract_result": sensor_contract.get("result", "UNKNOWN"),
            "vlp16_rate_hz": contract_topics.get("/velodyne_packets", {}).get(
                "rate_hz"
            ),
            "imu_rate_hz": contract_topics.get("/imu/data", {}).get("rate_hz"),
            "wheel_odom_rate_hz": contract_topics.get("/wheel/odom", {}).get(
                "rate_hz"
            ),
            "lidar_imu_nearest_p95_ms": timing.get(
                "nearest_absolute_difference_p95_ms"
            ),
            "sufficient_yaw_excitation": excitation.get(
                "sufficient_yaw_excitation"
            ),
            "sufficient_roll_pitch_excitation": excitation.get(
                "sufficient_roll_pitch_excitation"
            ),
            "odom_cumulative_distance_m": session_metrics.get(
                "odom_cumulative_distance_m"
            ),
            "maximum_map_odom_translation_jump_m": session_metrics.get(
                "maximum_map_odom_translation_jump_m"
            ),
            "maximum_map_odom_yaw_jump_deg": session_metrics.get(
                "maximum_map_odom_yaw_jump_deg"
            ),
            "md_disconnect_count": session_metrics.get("md_disconnect_count"),
        },
        "simulation_evidence": {
            "reference_profile_points": len(simulation.get("points", [])),
            "reference_lap_length_m": float(
                _closed_reference(list(simulation["points"]))[2]
            ),
            "dynamic_safety_status": dynamic_safety.get("status", "UNKNOWN"),
            "dynamic_safety_inputs": dynamic_safety.get("inputs", []),
            "dynamic_safety_limits": dynamic_safety.get("limits", {}),
            "limitations": dynamic_safety.get("limitations", []),
        },
        "artifacts": {},
        "required_before_field_use": [
            "register the real track start/module anchors to the MuJoCo course",
            "replace prior Z/slope with dynamically validated FAST-LIO/SC-PGO PCD",
            "compare live terrain residuals and reject the prior when inconsistent",
            "approve the resulting real 3D route through the normal track map check",
        ],
    }
    inputs = {
        "real_profile": real_profile_file,
        "real_occupancy": occupancy_yaml,
        "simulation_profile": simulation_profile,
        "demonstration_envelope": envelope_file,
        "mujoco_dynamic_safety": dynamic_safety_file,
    }
    if sensor_contract_file.is_file():
        inputs["real_sensor_contract"] = sensor_contract_file
    if session_metrics_file.is_file():
        inputs["real_session_metrics"] = session_metrics_file
    manifest["inputs"] = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in inputs.items()
    }
    for path in (profile_file, terrain_pcd, occupancy_pcd, visual_pcd, npz_file):
        manifest["artifacts"][path.name] = {
            "path": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
    manifest_file = output_directory / "manifest.yaml"
    manifest_file.write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return {"output_directory": str(output_directory), **manifest}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-directory", type=Path, required=True)
    parser.add_argument(
        "--simulation-profile",
        type=Path,
        default=workspace_path(
            "maps", "sim_pk3a_ground_truth", "pk3a_risk_aware_profile_v2.yaml"
        ),
    )
    parser.add_argument(
        "--envelope-file",
        type=Path,
        default=workspace_path(
            "models", "traversability", "traversability_envelope_v2.json"
        ),
    )
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument(
        "--dynamic-safety-file",
        type=Path,
        default=workspace_path(
            "models", "traversability", "mujoco_dynamic_safety_v1.json"
        ),
    )
    parser.add_argument("--start-module", default=DEFAULT_START_MODULE)
    parser.add_argument("--maximum-speed-mps", type=float, default=0.55)
    arguments = parser.parse_args()
    result = build_sim_assisted_map(
        arguments.map_directory,
        arguments.simulation_profile,
        arguments.envelope_file,
        arguments.output_directory,
        start_module=arguments.start_module,
        maximum_speed_mps=arguments.maximum_speed_mps,
        dynamic_safety_file=arguments.dynamic_safety_file,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
