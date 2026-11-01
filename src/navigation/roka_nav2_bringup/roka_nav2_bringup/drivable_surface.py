"""Extract a path-connected drivable surface from a registered 3D PCD.

FAST-LIO maps contain every visible return: floor, track, walls, furniture and
ceilings.  A 2.5D elevation map must not receive that complete scene directly.
This module keeps the full cloud as SLAM evidence and creates a separate,
path-connected surface cloud for terrain analysis.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from roka_nav2_bringup.pcd_terrain_profile import load_ascii_xyz
from roka_robot_description.hardware_config import load_hardware_config


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _load_route(directory: Path) -> tuple[np.ndarray, float]:
    csv_path = directory / "optimized_path.csv"
    profile_path = directory / "track_profile.yaml"
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
    track_width = float(profile.get("track_width_m", 0.9144))
    rows: list[tuple[float, float, float, float, float, float]] = []
    if csv_path.is_file():
        with csv_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                rows.append(tuple(
                    float(row[name]) for name in ("x", "y", "z", "roll", "pitch", "yaw")
                ))
    else:
        for point in profile.get("points") or []:
            rows.append(tuple(
                float(point.get(name, 0.0))
                for name in ("x", "y", "z", "roll", "pitch", "yaw")
            ))
    route = np.asarray(rows, dtype=np.float64)
    if route.ndim != 2 or route.shape[0] < 2 or route.shape[1] != 6:
        raise ValueError("optimized/recorded route has fewer than two valid poses")
    if not np.all(np.isfinite(route)):
        raise ValueError("route contains a non-finite pose")
    return route, track_width


def _resample_route(route: np.ndarray, spacing_m: float) -> np.ndarray:
    samples: list[tuple[float, float, float, float, float, float, float]] = []
    station = 0.0
    for first, second in zip(route, route[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        distance = math.hypot(dx, dy)
        count = max(1, int(math.ceil(distance / spacing_m)))
        yaw = math.atan2(dy, dx) if distance > 1.0e-6 else float(first[5])
        for index in range(count):
            ratio = index / count
            samples.append((
                float(first[0] + ratio * dx),
                float(first[1] + ratio * dy),
                float(first[2] + ratio * (second[2] - first[2])),
                float(first[3] + ratio * _wrap(second[3] - first[3])),
                float(first[4] + ratio * _wrap(second[4] - first[4])),
                yaw,
                station,
            ))
            station += distance / count
    samples.append((
        float(route[-1, 0]), float(route[-1, 1]), float(route[-1, 2]),
        float(route[-1, 3]), float(route[-1, 4]), float(route[-1, 5]), station,
    ))
    return np.asarray(samples, dtype=np.float64)


def _nearest_route(cloud: np.ndarray, route: np.ndarray) -> np.ndarray:
    """Return nearest resampled route index without allocating a huge matrix."""
    result = np.empty(len(cloud), dtype=np.int64)
    chunk_size = max(250, min(5000, int(4_000_000 / max(1, len(route)))))
    for start in range(0, len(cloud), chunk_size):
        stop = min(len(cloud), start + chunk_size)
        delta = cloud[start:stop, None, :2] - route[None, :, :2]
        result[start:stop] = np.argmin(np.einsum("ijk,ijk->ij", delta, delta), axis=1)
    return result


def _clusters(values: np.ndarray, gap_m: float) -> list[np.ndarray]:
    order = np.argsort(values[:, 2], kind="stable")
    ordered = values[order]
    split = np.where(np.diff(ordered[:, 2]) > gap_m)[0] + 1
    return [group for group in np.split(ordered, split) if len(group)]


def _write_ascii_pcd(path: Path, cloud: np.ndarray) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(cloud)}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        np.savetxt(stream, cloud.astype(np.float32), fmt="%.6f %.6f %.6f")


def extract_drivable_surface(
    directory: Path,
    *,
    input_name: str = "map_dense.pcd",
    output_name: str = "surface.pcd",
    resolution_m: float = 0.10,
    maximum_slope_deg: float = 55.0,
) -> dict[str, Any]:
    directory = directory.expanduser().resolve()
    input_path = directory / input_name
    if not input_path.is_file():
        input_path = directory / "map.pcd"
    route_raw, track_width = _load_route(directory)
    cloud = load_ascii_xyz(input_path)
    route = _resample_route(route_raw, min(0.05, resolution_m / 2.0))
    nearest = _nearest_route(cloud, route)
    reference = route[nearest]
    dx, dy = cloud[:, 0] - reference[:, 0], cloud[:, 1] - reference[:, 1]
    distance = np.hypot(dx, dy)
    lateral = -dx * np.sin(reference[:, 5]) + dy * np.cos(reference[:, 5])

    hardware = load_hardware_config()
    base_height = float(hardware.footprint_to_base.xyz[2])
    corridor_radius = max(
        0.5 * track_width,
        float(hardware.navigation_half_width_m) + 0.15,
    )
    keep = (distance <= corridor_radius + 0.10) & (np.abs(lateral) <= corridor_radius)
    corridor = cloud[keep]
    corridor_reference = reference[keep]
    corridor_lateral = lateral[keep]
    if len(corridor) < 100:
        raise ValueError("fewer than 100 PCD returns lie in the driven corridor")

    station_bin = np.floor(corridor_reference[:, 6] / resolution_m).astype(np.int64)
    lateral_bin = np.floor((corridor_lateral + corridor_radius) / resolution_m).astype(np.int64)
    grouped: dict[tuple[int, int], list[np.ndarray]] = {}
    raw_groups: dict[tuple[int, int], list[list[float]]] = {}
    for point, station, lateral_index in zip(corridor, station_bin, lateral_bin):
        raw_groups.setdefault((int(station), int(lateral_index)), []).append(point.tolist())
    for key, values in raw_groups.items():
        station, lateral_index = key
        route_slice = route[
            (route[:, 6] >= station * resolution_m)
            & (route[:, 6] < (station + 1) * resolution_m)
        ]
        if len(route_slice) == 0:
            continue
        route_pose = np.median(route_slice, axis=0)
        lateral_center = (
            (lateral_index + 0.5) * resolution_m - corridor_radius
        )
        expected_height = (
            float(route_pose[2]) - base_height
            + lateral_center * math.tan(float(route_pose[3]))
        )
        # A candidate must agree with the IMU-observed body plane before it is
        # allowed into continuity growth. This rejects walls and furniture
        # instead of letting them masquerade as steep terrain.
        grouped[key] = [
            group for group in _clusters(np.asarray(values, dtype=np.float64), 0.09)
            if abs(float(np.median(group[:, 2])) - expected_height) <= 0.18
        ]

    selected: dict[tuple[int, int], np.ndarray] = {}
    maximum_seed_error = 0.22
    center_index = corridor_radius / resolution_m
    station_values = sorted(set(int(value) for value in station_bin))
    # The base trajectory is the strongest ground prior: seed only close to the
    # wheel centre, then grow laterally through height-continuous neighbours.
    for station in station_values:
        route_slice = route[
            (route[:, 6] >= station * resolution_m)
            & (route[:, 6] < (station + 1) * resolution_m)
        ]
        if len(route_slice) == 0:
            continue
        expected = float(np.median(route_slice[:, 2]) - base_height)
        candidates: list[tuple[float, tuple[int, int], np.ndarray]] = []
        for lateral_index in range(int(center_index) - 2, int(center_index) + 3):
            key = (station, lateral_index)
            for group in grouped.get(key, []):
                median = float(np.median(group[:, 2]))
                candidates.append((abs(median - expected), key, group))
        if candidates:
            error, key, group = min(candidates, key=lambda item: item[0])
            if error <= maximum_seed_error:
                selected[key] = group

    tangent = math.tan(math.radians(maximum_slope_deg))
    neighbours = (
        (-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1),
        (-2, 0), (2, 0), (0, -2), (0, 2),
    )
    for _ in range(32):
        additions: dict[tuple[int, int], np.ndarray] = {}
        for key, candidates in grouped.items():
            if key in selected:
                continue
            adjacent: list[tuple[float, float]] = []
            for ds, dl in neighbours:
                neighbour = selected.get((key[0] + ds, key[1] + dl))
                if neighbour is not None:
                    adjacent.append((
                        float(np.median(neighbour[:, 2])),
                        resolution_m * math.hypot(ds, dl),
                    ))
            if not adjacent:
                continue
            accepted: list[tuple[float, np.ndarray]] = []
            for group in candidates:
                height = float(np.median(group[:, 2]))
                deltas = [abs(height - other) for other, _ in adjacent]
                allowed = [tangent * spacing + 0.035 for _, spacing in adjacent]
                ratios = [delta / limit for delta, limit in zip(deltas, allowed)]
                if min(ratios) <= 1.0 and float(np.median(ratios)) <= 1.25:
                    accepted.append((float(np.median(deltas)), group))
            if accepted:
                additions[key] = min(accepted, key=lambda item: item[0])[1]
        if not additions:
            break
        selected.update(additions)

    minimum_station_coverage = 0.70
    covered_stations = len({key[0] for key in selected})
    station_coverage = covered_stations / max(1, len(station_values))
    if station_coverage < minimum_station_coverage or len(selected) < 100:
        raise ValueError(
            "path-connected surface is too sparse: "
            f"cells={len(selected)}, station_coverage={station_coverage:.1%}"
        )

    # One robust surface sample per Frenet cell is sufficient for the 10 cm
    # GridMap.  Merge revisited XY cells so a loop does not add double layers.
    xy_cells: dict[tuple[int, int], list[np.ndarray]] = {}
    for group in selected.values():
        representative = np.median(group, axis=0)
        xy_key = tuple(np.floor(representative[:2] / resolution_m).astype(np.int64))
        xy_cells.setdefault(xy_key, []).append(representative)
    surface = np.asarray([
        np.median(np.asarray(values), axis=0) for values in xy_cells.values()
    ], dtype=np.float64)
    if len(surface) < 100:
        raise ValueError(f"surface has only {len(surface)} unique XY cells")
    output_path = directory / output_name
    _write_ascii_pcd(output_path, surface)

    gradients: list[float] = []
    surface_lookup = {
        key: float(np.median(np.asarray(values)[:, 2]))
        for key, values in xy_cells.items()
    }
    for key, height in surface_lookup.items():
        for neighbour in ((key[0] + 1, key[1]), (key[0], key[1] + 1)):
            if neighbour in surface_lookup:
                gradients.append(math.degrees(math.atan2(
                    abs(surface_lookup[neighbour] - height), resolution_m
                )))
    gradient_values = np.asarray(gradients, dtype=np.float64)
    report: dict[str, Any] = {
        "schema_version": 1,
        "result": "PASS",
        "source_pcd": input_path.name,
        "surface_pcd": output_path.name,
        "method": "route_seeded_height_continuity",
        "resolution_m": resolution_m,
        "track_width_m": track_width,
        "corridor_half_width_m": corridor_radius,
        "maximum_surface_slope_deg": maximum_slope_deg,
        "input_points": int(len(cloud)),
        "corridor_points": int(len(corridor)),
        "surface_cells": int(len(surface)),
        "rejected_corridor_cells": int(max(0, len(grouped) - len(selected))),
        "route_station_coverage": float(station_coverage),
        "surface_z_min_m": float(np.min(surface[:, 2])),
        "surface_z_max_m": float(np.max(surface[:, 2])),
        "neighbour_slope_p95_deg": (
            float(np.quantile(gradient_values, 0.95)) if len(gradient_values) else None
        ),
        "limitations": [
            "Only the corridor actually observed beside the recorded route is classified.",
            "A flat capture validates processing but does not measure future track slope.",
        ],
    }
    (directory / "surface_extraction.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--input-name", default="map_dense.pcd")
    parser.add_argument("--output-name", default="surface.pcd")
    parser.add_argument("--resolution-m", type=float, default=0.10)
    parser.add_argument("--maximum-slope-deg", type=float, default=55.0)
    args = parser.parse_args(argv)
    try:
        report = extract_drivable_surface(
            args.directory,
            input_name=args.input_name,
            output_name=args.output_name,
            resolution_m=args.resolution_m,
            maximum_slope_deg=args.maximum_slope_deg,
        )
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"[SURFACE][FAIL] {error}")
        return 1
    print(yaml.safe_dump(report, sort_keys=False, allow_unicode=True).rstrip())
    print("[SURFACE][RESULT] PASS; full scene and drivable surface are separate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
