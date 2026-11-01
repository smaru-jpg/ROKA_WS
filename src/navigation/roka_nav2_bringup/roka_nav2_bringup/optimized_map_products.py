"""Derive one coherent centerline and 2D navigation map from SC-PGO output."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from roka_nav2_bringup.pcd_terrain_profile import load_ascii_xyz


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _optimized_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            values = {
                name: float(row[name])
                for name in ("stamp", "x", "y", "z", "roll", "pitch", "yaw")
            }
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("optimized_path.csv contains a non-finite value")
            rows.append(values)
    if len(rows) < 2:
        raise ValueError("optimized_path.csv has fewer than two poses")
    return rows


def _resample(rows: list[dict[str, float]], spacing_m: float = 0.10) -> list[dict[str, float]]:
    if not 0.02 <= spacing_m <= 0.50:
        raise ValueError("profile spacing must be in [0.02, 0.50] m")
    result: list[dict[str, float]] = []
    for first, second in zip(rows, rows[1:]):
        distance = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
        count = max(1, int(math.ceil(distance / spacing_m)))
        yaw_delta = _wrap(second["yaw"] - first["yaw"])
        for index in range(count):
            ratio = index / count
            result.append({
                "x": first["x"] + ratio * (second["x"] - first["x"]),
                "y": first["y"] + ratio * (second["y"] - first["y"]),
                "z": first["z"] + ratio * (second["z"] - first["z"]),
                "roll": first["roll"] + ratio * _wrap(second["roll"] - first["roll"]),
                "pitch": first["pitch"] + ratio * _wrap(second["pitch"] - first["pitch"]),
                "yaw": _wrap(first["yaw"] + ratio * yaw_delta),
            })
    result.append({name: rows[-1][name] for name in ("x", "y", "z", "roll", "pitch", "yaw")})
    return result


def _path_length(points: list[dict[str, float]], closed: bool) -> float:
    pairs = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        pairs.append((points[-1], points[0]))
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in pairs)


def derive_optimized_profile(
    raw_profile_path: Path,
    optimized_csv: Path,
    output_path: Path,
    *,
    spacing_m: float = 0.10,
) -> dict[str, Any]:
    raw = yaml.safe_load(raw_profile_path.read_text(encoding="utf-8")) or {}
    rows = _optimized_rows(optimized_csv)
    points = _resample(rows, spacing_m)
    closed = bool(raw.get("closed_loop", False))
    default_speed = float(raw.get("default_speed_mps", 0.30))
    minimum_speed = float(raw.get("minimum_curve_speed_mps", 0.10))
    lateral_acceleration = float(raw.get("maximum_lateral_accel_mps2", 0.25))

    for index, point in enumerate(points):
        before = points[(index - 1) % len(points)] if closed or index else points[index]
        after = points[(index + 1) % len(points)] if closed or index + 1 < len(points) else points[index]
        dx, dy = after["x"] - before["x"], after["y"] - before["y"]
        if math.hypot(dx, dy) > 1.0e-6:
            point["yaw"] = math.atan2(dy, dx)
        distance = max(math.hypot(point["x"] - before["x"], point["y"] - before["y"]), 1.0e-3)
        curvature = abs(_wrap(point["yaw"] - before["yaw"])) / distance
        curve_speed = math.sqrt(lateral_acceleration / max(curvature, 1.0e-6))
        point["speed_mps"] = float(min(default_speed, max(minimum_speed, curve_speed)))

    result = {
        "schema_version": 2,
        "frame_id": "map",
        "environment": str(raw.get("environment", "inline")),
        "profile_role": "optimized_mapping_evidence",
        "source_pose_graph": optimized_csv.name,
        "closed_loop": closed,
        "autonomy_approved": False,
        "approval_reason": "optimized_map_requires_review_and_traversability_approval",
        "track_width_m": float(raw.get("track_width_m", 0.9144)),
        "default_speed_mps": default_speed,
        "minimum_curve_speed_mps": minimum_speed,
        "maximum_lateral_accel_mps2": lateral_acceleration,
        "recorded_path_length_m": float(raw.get("recorded_path_length_m", _path_length(points, closed))),
        "optimized_path_length_m": _path_length(points, closed),
        "points": points,
    }
    output_path.write_text(
        yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return result


def derive_occupancy_from_optimized_map(
    pcd_path: Path,
    profile_path: Path,
    output_yaml: Path,
    *,
    resolution_m: float = 0.05,
    margin_m: float = 1.5,
) -> dict[str, int | float]:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
    points = profile.get("points") or []
    if len(points) < 10:
        raise ValueError("optimized profile needs at least ten points")
    if not 0.02 <= resolution_m <= 0.20:
        raise ValueError("occupancy resolution must be in [0.02, 0.20] m")
    track_width = float(profile.get("track_width_m", 0.9144))
    corridor_radius = max(0.25, 0.5 * track_width)
    path_xy = np.asarray([(float(point["x"]), float(point["y"])) for point in points])
    minimum = np.min(path_xy, axis=0) - margin_m
    maximum = np.max(path_xy, axis=0) + margin_m
    width, height = np.ceil((maximum - minimum) / resolution_m).astype(np.int64) + 1
    if width < 20 or height < 20 or width * height > 50_000_000:
        raise ValueError(f"derived occupancy dimensions are unsafe: {width}x{height}")

    # ROS map_server convention: 0 occupied, 254 free, 205 unknown.
    image = np.full((int(height), int(width)), 205, dtype=np.uint8)
    cloud = load_ascii_xyz(pcd_path)
    inside = (
        (cloud[:, 0] >= minimum[0]) & (cloud[:, 0] <= maximum[0])
        & (cloud[:, 1] >= minimum[1]) & (cloud[:, 1] <= maximum[1])
    )
    cloud = cloud[inside]
    columns = np.floor((cloud[:, 0] - minimum[0]) / resolution_m).astype(np.int64)
    rows_from_bottom = np.floor((cloud[:, 1] - minimum[1]) / resolution_m).astype(np.int64)
    rows = int(height) - 1 - rows_from_bottom
    valid = (rows >= 0) & (rows < height) & (columns >= 0) & (columns < width)
    image[rows[valid], columns[valid]] = 0

    # One VLP return per cell produces salt-and-pepper obstacle islands. Join
    # immediate neighbours before carving the driven corridor so the Nav2 map
    # represents continuous track boundaries rather than thousands of specks.
    occupied = image == 0
    for _ in range(2):
        padded = np.pad(occupied, 1, mode="constant", constant_values=False)
        occupied = np.logical_or.reduce([
            padded[dy:dy + int(height), dx:dx + int(width)]
            for dy in range(3) for dx in range(3)
        ])
    image[occupied] = 0

    radius_cells = int(math.ceil(corridor_radius / resolution_m))
    for x, y in path_xy:
        center_x = int(math.floor((x - minimum[0]) / resolution_m))
        center_y_bottom = int(math.floor((y - minimum[1]) / resolution_m))
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                column = center_x + dx
                row = int(height) - 1 - (center_y_bottom + dy)
                if 0 <= row < height and 0 <= column < width:
                    image[row, column] = 254

    output_pgm = output_yaml.with_suffix(".pgm")
    with output_pgm.open("wb") as stream:
        stream.write(f"P5\n{int(width)} {int(height)}\n255\n".encode("ascii"))
        stream.write(image.tobytes())
    output_yaml.write_text(
        yaml.safe_dump({
            "image": output_pgm.name,
            "mode": "trinary",
            "resolution": float(resolution_m),
            "origin": [float(minimum[0]), float(minimum[1]), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }, sort_keys=False),
        encoding="utf-8",
    )
    return {
        "width": int(width),
        "height": int(height),
        "occupied_cells": int(np.count_nonzero(image == 0)),
        "free_cells": int(np.count_nonzero(image == 254)),
        "resolution_m": float(resolution_m),
    }


def derive_sc_pgo_navigation_products(directory: Path) -> dict[str, Any]:
    directory = Path(directory).expanduser().resolve()
    profile = derive_optimized_profile(
        directory / "track_profile.yaml",
        directory / "optimized_path.csv",
        directory / "track_profile.yaml",
    )
    occupancy = derive_occupancy_from_optimized_map(
        directory / "map.pcd", directory / "track_profile.yaml", directory / "occupancy.yaml"
    )
    return {"profile_points": len(profile["points"]), "occupancy": occupancy}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    args = parser.parse_args(argv)
    result = derive_sc_pgo_navigation_products(Path(args.directory))
    print(yaml.safe_dump(result, sort_keys=False).strip())
