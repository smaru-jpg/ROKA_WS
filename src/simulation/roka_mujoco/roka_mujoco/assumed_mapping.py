"""Build an explicitly simulated PK-3A mapping bundle from a manual MuJoCo lap.

The collision height field supplies the environment that a VLP-16 would see.
The route, speed, attitude and commands come from an Xbox demonstration rather
than from the analytic safe-path generator.  The result deliberately lacks a
real-map bundle manifest, so the field launcher cannot mistake it for an
approved FAST-LIO/SC-PGO product.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from typing import Any

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt
import yaml

from roka_robot_description.workspace import workspace_root

from .sim_assisted_map import write_pcd


DEFAULT_SUCCESS = "lap01_2.csv"
DEFAULT_FAILURES = ("lap01.csv", "fast_test01.csv", "fast_test01_2.csv")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "sim_time_sec", "enabled", "x_m", "y_m", "z_m", "roll_deg",
        "pitch_deg", "yaw_deg", "cmd_linear_x_mps", "cmd_angular_z_radps",
        "path_index", "actual_rideable", "unsafe_label",
    }
    missing = sorted(required - set(rows[0] if rows else ()))
    if missing:
        raise ValueError(f"manual demonstration lacks columns: {missing}")
    return rows


def _manual_route(rows: list[dict[str, str]], spacing_m: float) -> list[dict[str, Any]]:
    usable = [
        row for row in rows
        if int(row["enabled"]) == 1 and int(row["actual_rideable"]) == 1
    ]
    if len(usable) < 2:
        raise ValueError("manual demonstration contains no enabled rideable route")
    selected = [usable[0]]
    accumulated = 0.0
    before = usable[0]
    for row in usable[1:]:
        step = math.hypot(
            float(row["x_m"]) - float(before["x_m"]),
            float(row["y_m"]) - float(before["y_m"]),
        )
        before = row
        if not math.isfinite(step) or step > 0.25:
            continue
        accumulated += step
        if accumulated >= spacing_m:
            selected.append(row)
            accumulated = 0.0
    if selected[-1] is not usable[-1]:
        selected.append(usable[-1])
    if len(selected) < 50:
        raise ValueError("manual route is too short after metric resampling")
    return [
        {
            "time_sec": float(row["sim_time_sec"]),
            "x": float(row["x_m"]),
            "y": float(row["y_m"]),
            "body_z": float(row["z_m"]),
            "roll_deg": float(row["roll_deg"]),
            "pitch_deg": float(row["pitch_deg"]),
            "yaw": math.radians(float(row["yaw_deg"])),
            "cmd_linear_x_mps": float(row["cmd_linear_x_mps"]),
            "cmd_angular_z_radps": float(row["cmd_angular_z_radps"]),
            "forward_speed_mps": float(row.get("forward_speed_mps", 0.0)),
            "path_index": float(row["path_index"]),
            "module_id": row.get("module_id", "unknown") or "unknown",
            "actual_cross_slope_deg": float(row.get("actual_cross_slope_deg", 0.0)),
            "actual_longitudinal_slope_deg": float(
                row.get("actual_longitudinal_slope_deg", 0.0)
            ),
            "actual_minimum_edge_m": float(row.get("actual_minimum_edge_m", "nan")),
        }
        for row in selected
    ]


def _failure_rates(paths: list[Path], count: int = 858) -> np.ndarray:
    visits = np.zeros(count, dtype=np.float64)
    unsafe = np.zeros(count, dtype=np.float64)
    for path in paths:
        for row in _load_rows(path):
            if int(row["enabled"]) != 1:
                continue
            index = max(0, min(count - 1, int(float(row["path_index"]))))
            visits[index] += 1.0
            unsafe[index] += float(int(row["unsafe_label"]) != 0)
    return np.divide(unsafe, visits, out=np.zeros_like(unsafe), where=visits > 0.0)


def _grid_xy(metadata: dict[str, Any], shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    origin_x, origin_y = [float(value) for value in metadata["origin_xy_m"]]
    resolution = float(metadata["resolution_m"])
    rows, columns = shape
    x = origin_x + (np.arange(columns, dtype=np.float64) + 0.5) * resolution
    y = origin_y + (np.arange(rows, dtype=np.float64) + 0.5) * resolution
    return x, y


def _sample_route_surface(
    route: list[dict[str, Any]],
    layers: dict[str, np.ndarray],
    metadata: dict[str, Any],
) -> None:
    valid = np.asarray(layers["rideable"], dtype=bool)
    elevation = np.asarray(layers["elevation"], dtype=np.float64)
    slope = np.asarray(layers["slope_deg"], dtype=np.float64)
    edge = np.asarray(layers["edge_distance_m"], dtype=np.float64)
    nearest = distance_transform_edt(~valid, return_distances=False, return_indices=True)
    origin_x, origin_y = [float(value) for value in metadata["origin_xy_m"]]
    resolution = float(metadata["resolution_m"])
    rows, columns = valid.shape
    for point in route:
        column = int(math.floor((point["x"] - origin_x) / resolution))
        row = int(math.floor((point["y"] - origin_y) / resolution))
        row = max(0, min(rows - 1, row))
        column = max(0, min(columns - 1, column))
        if not valid[row, column]:
            row, column = int(nearest[0, row, column]), int(nearest[1, row, column])
        point["surface_height_m"] = float(elevation[row, column])
        point["maximum_wheel_slope_deg"] = float(slope[row, column])
        point["minimum_wheel_edge_m"] = float(edge[row, column])


def _visible_surface(
    route: list[dict[str, Any]],
    layers: dict[str, np.ndarray],
    metadata: dict[str, Any],
    *,
    horizontal_fov_deg: float,
    random_seed: int,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    valid = np.asarray(layers["rideable"], dtype=bool)
    elevation = np.asarray(layers["elevation"], dtype=np.float64)
    x_axis, y_axis = _grid_xy(metadata, valid.shape)
    rows, columns = np.nonzero(valid)
    points = np.column_stack((x_axis[columns], y_axis[rows], elevation[rows, columns]))
    visible = np.zeros(len(points), dtype=bool)
    half_fov = math.radians(0.5 * horizontal_fov_deg)
    # Roughly one virtual scan pose per 0.4 m is enough to reproduce the
    # accumulated one-lap coverage without claiming ray-level occlusion.
    pose_stride = max(1, len(route) // 170)
    for pose in route[::pose_stride]:
        dx = points[:, 0] - pose["x"]
        dy = points[:, 1] - pose["y"]
        cosine, sine = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        forward = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        distance = np.hypot(dx, dy)
        bearing = np.arctan2(lateral, forward)
        sensor_z = pose["surface_height_m"] + 0.72
        vertical = np.arctan2(points[:, 2] - sensor_z, np.maximum(distance, 1.0e-6))
        visible |= (
            (forward > 0.0)
            & (distance >= 0.30)
            & (distance <= 25.0)
            & (np.abs(bearing) <= half_fov)
            & (vertical >= math.radians(-16.0))
            & (vertical <= math.radians(15.0))
        )
    rng = np.random.default_rng(random_seed)
    observed = points[visible]
    if len(observed) == 0:
        raise RuntimeError("virtual VLP-16 did not observe the MuJoCo track")
    keep = rng.random(len(observed)) >= 0.08
    observed = observed[keep]
    noisy = observed + rng.normal(
        loc=(0.0, 0.0, 0.0), scale=(0.004, 0.004, 0.006), size=observed.shape
    )
    intensity = np.clip(90.0 + 150.0 * (noisy[:, 2] - noisy[:, 2].min()) /
                        max(float(np.ptp(noisy[:, 2])), 1.0e-6), 0.0, 255.0)
    dense = np.column_stack((noisy, intensity)).astype(np.float32)
    exact_surface = np.column_stack(
        (observed, np.full(len(observed), 128.0, dtype=np.float64))
    ).astype(np.float32)
    quantized = np.floor(dense[:, :3] / 0.08).astype(np.int32)
    _, unique = np.unique(quantized, axis=0, return_index=True)
    light = dense[np.sort(unique)]
    return dense, light, {
        "surface": exact_surface,
        "total_rideable_cells": int(len(points)),
        "visible_cells_before_dropout": int(np.count_nonzero(visible)),
        "observed_cells": int(len(observed)),
        "coverage_fraction": float(np.count_nonzero(visible) / len(points)),
        "horizontal_fov_deg": horizontal_fov_deg,
        "vertical_fov_deg": [-16.0, 15.0],
        "range_m": [0.30, 25.0],
        "dropout_fraction": 0.08,
        "noise_stddev_m": [0.004, 0.004, 0.006],
    }


def _write_occupancy(
    directory: Path,
    layers: dict[str, np.ndarray],
    metadata: dict[str, Any],
) -> None:
    rideable = np.asarray(layers["rideable"], dtype=bool)
    image = np.full(rideable.shape, 205, dtype=np.uint8)
    image[rideable] = 254
    boundary = binary_dilation(rideable, iterations=2) & ~rideable
    image[boundary] = 0
    Image.fromarray(np.flipud(image), mode="L").save(directory / "occupancy.pgm")
    (directory / "occupancy.yaml").write_text(
        yaml.safe_dump({
            "image": "occupancy.pgm",
            "mode": "trinary",
            "resolution": float(metadata["resolution_m"]),
            "origin": [
                float(metadata["origin_xy_m"][0]),
                float(metadata["origin_xy_m"][1]),
                0.0,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }, sort_keys=False),
        encoding="utf-8",
    )


def _write_pose_graph(directory: Path, route: list[dict[str, Any]]) -> None:
    sampled = route[::20]
    information = " ".join(["1"] * 21)
    lines: list[str] = []
    for index, point in enumerate(sampled):
        half = 0.5 * point["yaw"]
        lines.append(
            f"VERTEX_SE3:QUAT {index} {point['x']:.6f} {point['y']:.6f} "
            f"{point['surface_height_m']:.6f} 0 0 {math.sin(half):.8f} "
            f"{math.cos(half):.8f}"
        )
        if index:
            lines.append(f"EDGE_SE3:QUAT {index - 1} {index} 0 0 0 0 0 0 1 {information}")
    if len(sampled) > 2:
        lines.append(
            f"EDGE_SE3:QUAT {len(sampled) - 1} 0 0 0 0 0 0 0 1 {information}"
        )
    (directory / "pose_graph.g2o").write_text("\n".join(lines) + "\n", encoding="ascii")


def _write_terrain_bag(directory: Path) -> None:
    terrain_share = Path(get_package_share_directory("roka_terrain_mapping"))
    output = directory / "terrain_global.bag"
    if output.exists():
        shutil.rmtree(output)
    subprocess.run([
        "ros2", "run", "roka_terrain_mapping", "terrain_standard_postprocessor_node",
        "--ros-args", "--params-file",
        str(terrain_share / "config" / "terrain_filter_chain.yaml"),
        "-p", f"input_pcd:={directory / 'surface.pcd'}",
        "-p", f"pcl_config:={terrain_share / 'config' / 'global_pcd.yaml'}",
        "-p", f"output_bag:={output}",
        "-p", "output_topic:=/terrain/static_map", "-p", "map_frame:=map",
    ], check=True)


def build_assumed_mapping(
    ground_truth_npz: Path,
    ground_truth_metadata: Path,
    success_csv: Path,
    failure_csvs: list[Path],
    risk_profile_file: Path,
    output_directory: Path,
    *,
    spacing_m: float = 0.08,
    horizontal_fov_deg: float = 120.0,
    random_seed: int = 3103,
    build_terrain: bool = True,
) -> dict[str, Any]:
    paths = [ground_truth_npz, ground_truth_metadata, success_csv, risk_profile_file]
    paths.extend(failure_csvs)
    for path in paths:
        if not path.expanduser().is_file():
            raise FileNotFoundError(path)
    output_directory = output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    metadata = json.loads(ground_truth_metadata.read_text(encoding="utf-8"))
    archive = np.load(ground_truth_npz)
    layers = {name: archive[name] for name in archive.files}
    route = _manual_route(_load_rows(success_csv), spacing_m)
    _sample_route_surface(route, layers, metadata)
    risk_profile = yaml.safe_load(risk_profile_file.read_text(encoding="utf-8")) or {}
    reference_risk = np.asarray([
        float(point.get("traversability_cost", 0.0))
        for point in risk_profile.get("points", [])
    ])
    failure_rate = _failure_rates(failure_csvs, max(858, len(reference_risk)))
    for point in route:
        index = max(0, min(len(failure_rate) - 1, int(round(point["path_index"]))))
        base_risk = float(reference_risk[index]) if index < len(reference_risk) else 0.0
        point["failure_observation_rate"] = float(failure_rate[index])
        point["traversability_cost"] = float(min(1.5, max(
            base_risk, 1.25 * failure_rate[index]
        )))
        point["risk"] = point["traversability_cost"]
        point["z"] = point["surface_height_m"]
        point["cross_slope_deg"] = point["actual_cross_slope_deg"]
        point["longitudinal_slope_deg"] = point["actual_longitudinal_slope_deg"]
        point["speed_mps"] = float(max(0.05, min(
            0.60, abs(point["cmd_linear_x_mps"])
        )))

    dense, light, observation = _visible_surface(
        route, layers, metadata,
        horizontal_fov_deg=horizontal_fov_deg,
        random_seed=random_seed,
    )
    surface = observation.pop("surface")
    write_pcd(output_directory / "map_dense.pcd", dense)
    write_pcd(output_directory / "map.pcd", light)
    write_pcd(output_directory / "surface.pcd", surface)
    _write_occupancy(output_directory, layers, metadata)
    _write_pose_graph(output_directory, route)

    closure = math.hypot(
        route[-1]["x"] - route[0]["x"], route[-1]["y"] - route[0]["y"]
    )
    length = sum(math.hypot(
        after["x"] - before["x"], after["y"] - before["y"]
    ) for before, after in zip(route, route[1:]))
    module_risk: dict[str, dict[str, float | int | str]] = {}
    for point in route:
        module = str(point["module_id"])
        summary = module_risk.setdefault(module, {
            "module_id": module,
            "sample_count": 0,
            "maximum_risk": 0.0,
            "mean_risk": 0.0,
            "high_risk_sample_count": 0,
        })
        summary["sample_count"] = int(summary["sample_count"]) + 1
        summary["mean_risk"] = float(summary["mean_risk"]) + float(point["risk"])
        summary["maximum_risk"] = max(
            float(summary["maximum_risk"]), float(point["risk"])
        )
        if float(point["risk"]) >= 0.50:
            summary["high_risk_sample_count"] = int(
                summary["high_risk_sample_count"]
            ) + 1
    for summary in module_risk.values():
        summary["mean_risk"] = float(summary["mean_risk"]) / max(
            1, int(summary["sample_count"])
        )
    risk_hotspots = sorted(
        (
            summary for summary in module_risk.values()
            if int(summary["high_risk_sample_count"]) > 0
        ),
        key=lambda item: (float(item["maximum_risk"]), float(item["mean_risk"])),
        reverse=True,
    )
    profile = {
        "schema_version": 3,
        "frame_id": "map",
        "environment": "mujoco_assumed_vlp16_mapping",
        "closed_loop": closure <= 0.30,
        "path_role": "manual_xbox_successful_lap",
        "path_source": success_csv.name,
        "field_navigation_authorized": False,
        "track_width_m": 1.20,
        "recorded_path_length_m": float(length),
        "closure_distance_m": float(closure),
        "points": route,
    }
    (output_directory / "track_profile.yaml").write_text(
        yaml.safe_dump(profile, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    surface_report = {
        "schema_version": 1,
        "result": "PASS",
        "method": "virtual_vlp16_120deg_one_manual_lap",
        "surface_pcd": "surface.pcd",
        "surface_cells": int(len(surface)),
        "route_station_coverage": 1.0,
        **observation,
        "limitations": [
            "ray-level occlusion and moving objects are not simulated",
            "this bundle is simulation evidence and cannot authorize field motion",
        ],
    }
    (output_directory / "surface_extraction.yaml").write_text(
        yaml.safe_dump(surface_report, sort_keys=False), encoding="utf-8"
    )
    if build_terrain:
        _write_terrain_bag(output_directory)
        (output_directory / "terrain_global_metadata.yaml").write_text(
            yaml.safe_dump({
                "schema_version": 1,
                "source_pcd": "surface.pcd",
                "topic": "/terrain/static_map",
                "pipeline": "ANYbotics grid_map_pcl + grid_map_filters",
                "role": "mujoco_assumed_mapped_surface",
                "field_navigation_authorized": False,
            }, sort_keys=False), encoding="utf-8"
        )

    result: dict[str, Any] = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "SIMULATED_MAPPING_READY_FOR_RVIZ",
        "mapping_backend_assumption": "VLP16_120deg+EBIMU+wheel_odom+FAST_LIO2+SC_PGO",
        "route_source": "successful_manual_xbox_demonstration",
        "success_csv": str(success_csv.resolve()),
        "failure_csvs_used_for_risk_only": [str(path.resolve()) for path in failure_csvs],
        "field_navigation_authorized": False,
        "reason": "MuJoCo map and demonstrations are not real field observations",
        "route": {
            "points": len(route), "length_m": float(length),
            "closure_distance_m": float(closure),
            "maximum_abs_roll_deg": max(abs(point["roll_deg"]) for point in route),
            "maximum_abs_pitch_deg": max(abs(point["pitch_deg"]) for point in route),
            "maximum_risk": max(point["risk"] for point in route),
            "module_risk_summary": risk_hotspots,
        },
        "virtual_lidar": observation,
        "products": {},
    }
    products = [
        output_directory / name for name in (
            "map.pcd", "map_dense.pcd", "surface.pcd", "occupancy.pgm",
            "occupancy.yaml", "pose_graph.g2o", "track_profile.yaml",
            "surface_extraction.yaml",
        )
    ]
    if build_terrain:
        products.append(output_directory / "terrain_global_metadata.yaml")
    result["products"] = {
        path.name: {"size_bytes": path.stat().st_size, "sha256": _sha256(path)}
        for path in products
    }
    manifest = output_directory / "simulation_mapping_manifest.yaml"
    manifest.write_text(
        yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return result


def main() -> None:
    workspace = workspace_root()
    ground = workspace / "maps" / "sim_pk3a_ground_truth"
    demonstrations = workspace / "datasets" / "xbox_demonstrations"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, default=ground / "pk3a_ground_truth.npz")
    parser.add_argument("--ground-truth-metadata", type=Path, default=ground / "pk3a_ground_truth.json")
    parser.add_argument("--success-csv", type=Path, default=demonstrations / DEFAULT_SUCCESS)
    parser.add_argument(
        "--failure-csv", type=Path, action="append",
        default=[demonstrations / name for name in DEFAULT_FAILURES],
    )
    parser.add_argument(
        "--risk-profile", type=Path,
        default=ground / "pk3a_risk_aware_profile_v2.yaml",
    )
    parser.add_argument(
        "--output-directory", type=Path,
        default=workspace / "maps" / "sim_pk3a_assumed_mapping",
    )
    parser.add_argument("--spacing", type=float, default=0.08)
    parser.add_argument("--horizontal-fov", type=float, default=120.0)
    parser.add_argument("--random-seed", type=int, default=3103)
    parser.add_argument("--skip-terrain", action="store_true")
    parser.add_argument("--json", action="store_true", help="print the full manifest")
    args = parser.parse_args()
    result = build_assumed_mapping(
        args.ground_truth, args.ground_truth_metadata, args.success_csv,
        list(args.failure_csv), args.risk_profile, args.output_directory,
        spacing_m=args.spacing, horizontal_fov_deg=args.horizontal_fov,
        random_seed=args.random_seed, build_terrain=not args.skip_terrain,
    )
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    route = result["route"]
    lidar = result["virtual_lidar"]
    print(
        "[SIM-MAP][PASS] "
        f"manual route={route['length_m']:.2f}m/{route['points']} points, "
        f"closure={route['closure_distance_m']:.3f}m, "
        f"virtual VLP coverage={100.0 * lidar['coverage_fraction']:.1f}%"
    )
    for hotspot in route["module_risk_summary"][:5]:
        print(
            "[SIM-MAP][RISK] "
            f"{hotspot['module_id']}: max={hotspot['maximum_risk']:.3f}, "
            f"high-risk samples={hotspot['high_risk_sample_count']}"
        )


if __name__ == "__main__":
    main()
