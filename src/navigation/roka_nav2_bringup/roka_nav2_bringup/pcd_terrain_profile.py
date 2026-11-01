"""Conservative path terrain and edge extraction from an ASCII/binary PCD map."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from roka_robot_description.hardware_config import load_hardware_config


def load_ascii_xyz(path: Path) -> np.ndarray:
    """Load finite XYZ from the PCD formats emitted by ROKA map producers.

    The historical function name is retained for API compatibility. New maps
    use uncompressed binary PCD; legacy maps use ASCII. binary_compressed is
    rejected deliberately so every accepted navigation product is readable by
    both the PCL localizer and the offline validator.
    """
    fields: list[str] = []
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    data_kind = ""
    header_lines = 0
    data_offset = 0
    point_count = 0
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                break
            header_lines += 1
            line = raw.decode("ascii", errors="strict")
            words = line.strip().split()
            if not words:
                continue
            key = words[0].upper()
            if key == "FIELDS":
                fields = words[1:]
            elif key == "SIZE":
                sizes = [int(value) for value in words[1:]]
            elif key == "TYPE":
                types = words[1:]
            elif key == "COUNT":
                counts = [int(value) for value in words[1:]]
            elif key == "POINTS":
                point_count = int(words[1])
            elif key == "DATA":
                data_kind = words[1].lower() if len(words) > 1 else ""
                data_offset = stream.tell()
                break
    if data_kind not in {"ascii", "binary"}:
        raise ValueError(
            f"map PCD must use DATA ascii or binary, got {data_kind or 'missing'}"
        )
    try:
        columns = [fields.index(name) for name in ("x", "y", "z")]
    except ValueError as error:
        raise ValueError("map PCD has no x/y/z fields") from error
    if data_kind == "ascii":
        cloud = np.loadtxt(path, skiprows=header_lines, usecols=columns, ndmin=2)
    else:
        if not counts:
            counts = [1] * len(fields)
        if not (
            len(fields) == len(sizes) == len(types) == len(counts)
            and point_count > 0
        ):
            raise ValueError("binary PCD header is incomplete")
        formats = {
            ("F", 4): "<f4", ("F", 8): "<f8",
            ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
            ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
        }
        dtype_fields = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            value_format = formats.get((kind.upper(), size))
            if value_format is None:
                raise ValueError(f"unsupported binary PCD field {name}: {kind}{size}")
            dtype_fields.append((name, value_format, (count,)) if count > 1 else (name, value_format))
        with path.open("rb") as stream:
            stream.seek(data_offset)
            records = np.fromfile(stream, dtype=np.dtype(dtype_fields), count=point_count)
        cloud = np.column_stack([records[name] for name in ("x", "y", "z")])
    cloud = np.asarray(cloud, dtype=np.float64)
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    if len(cloud) < 100:
        raise ValueError("map PCD has fewer than 100 finite XYZ points")
    return cloud


class SpatialCloud:
    def __init__(self, xyz: np.ndarray, cell_size: float = 1.0):
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("XYZ cloud must have shape (N, 3)")
        self.xyz = xyz
        self.cell_size = float(cell_size)
        self.origin = np.floor(np.min(xyz[:, :2], axis=0) / self.cell_size)
        cells = (
            np.floor(xyz[:, :2] / self.cell_size).astype(np.int64)
            - self.origin.astype(np.int64)
        )
        self.nx = int(np.max(cells[:, 0])) + 1
        keys = cells[:, 0] + cells[:, 1] * self.nx
        order = np.argsort(keys, kind="stable")
        unique, start = np.unique(keys[order], return_index=True)
        stop = np.r_[start[1:], len(order)]
        self.buckets = {
            int(key): order[int(first):int(last)]
            for key, first, last in zip(unique, start, stop)
        }

    def query(self, x: float, y: float, radius: float) -> np.ndarray:
        low = np.floor(
            (np.asarray([x, y]) - radius) / self.cell_size
        ).astype(np.int64)
        high = np.floor(
            (np.asarray([x, y]) + radius) / self.cell_size
        ).astype(np.int64)
        low -= self.origin.astype(np.int64)
        high -= self.origin.astype(np.int64)
        selected = []
        for iy in range(int(low[1]), int(high[1]) + 1):
            for ix in range(int(low[0]), int(high[0]) + 1):
                bucket = self.buckets.get(ix + iy * self.nx)
                if bucket is not None:
                    selected.append(bucket)
        if not selected:
            return np.empty((0, 3), dtype=np.float64)
        indices = np.concatenate(selected)
        cloud = self.xyz[indices]
        mask = (
            np.abs(cloud[:, 0] - x) <= radius
        ) & (np.abs(cloud[:, 1] - y) <= radius)
        return cloud[mask]


def _surface_cells(local: np.ndarray, resolution: float = 0.10) -> np.ndarray:
    uv = np.floor(local[:, :2] / resolution).astype(np.int64)
    minimum = np.min(uv, axis=0)
    shifted = uv - minimum
    nx = int(np.max(shifted[:, 0])) + 1
    keys = shifted[:, 0] + shifted[:, 1] * nx
    order = np.argsort(keys, kind="stable")
    unique, start = np.unique(keys[order], return_index=True)
    stop = np.r_[start[1:], len(order)]
    cells = []
    for first, last in zip(start, stop):
        values = local[order[int(first):int(last)]]
        cells.append((
            float(np.mean(values[:, 0])),
            float(np.mean(values[:, 1])),
            float(np.quantile(values[:, 2], 0.80)),
        ))
    return np.asarray(cells, dtype=np.float64)


def _plane_model(
    cells: np.ndarray, wheel_track: float
) -> tuple[float, float, float, float]:
    selected = cells[
        (np.abs(cells[:, 0]) <= 0.30)
        & (np.abs(cells[:, 1]) <= wheel_track / 2.0 + 0.10)
    ]
    if len(selected) < 6:
        raise ValueError("not enough mapped wheel-corridor surface cells")
    design = np.column_stack((
        selected[:, 0], selected[:, 1], np.ones(len(selected))
    ))
    coefficients, *_ = np.linalg.lstsq(design, selected[:, 2], rcond=None)
    residual = np.abs(design @ coefficients - selected[:, 2])
    inliers = residual <= 0.08
    if np.count_nonzero(inliers) < 5:
        raise ValueError("wheel-corridor surface is not planar enough")
    coefficients, *_ = np.linalg.lstsq(
        design[inliers], selected[inliers, 2], rcond=None
    )
    residual = design[inliers] @ coefficients - selected[inliers, 2]
    longitudinal = math.degrees(math.atan(float(coefficients[0])))
    cross = math.degrees(math.atan(float(coefficients[1])))
    height = float(coefficients[2])
    roughness = float(np.sqrt(np.mean(residual * residual)))
    return cross, longitudinal, height, roughness


def _lateral_profile(cells: np.ndarray, resolution: float = 0.10) -> np.ndarray:
    selected = cells[np.abs(cells[:, 0]) <= 0.25]
    if len(selected) < 5:
        raise ValueError("not enough lateral surface cells")
    bins = np.round(selected[:, 1] / resolution).astype(np.int64)
    rows = []
    for value in np.unique(bins):
        group = selected[bins == value]
        rows.append((float(np.mean(group[:, 1])), float(np.median(group[:, 2]))))
    profile = np.asarray(sorted(rows), dtype=np.float64)
    if len(profile) < 5:
        raise ValueError("lateral surface profile is too sparse")
    return profile


def _edge_distance(profile: np.ndarray, direction: int) -> float:
    center = int(np.argmin(np.abs(profile[:, 0])))
    if abs(float(profile[center, 0])) > 0.15:
        raise ValueError("mapped surface does not cover the driven path")
    index = center
    while True:
        following = index + direction
        if following < 0 or following >= len(profile):
            raise ValueError("track edge is outside mapped PCD coverage")
        lateral_gap = abs(float(profile[following, 0] - profile[index, 0]))
        height_step = abs(float(profile[following, 1] - profile[index, 1]))
        if lateral_gap > 0.22 or height_step > 0.12:
            return abs(0.5 * float(profile[following, 0] + profile[index, 0]))
        index = following


def surface_features(
    cloud_xyz: np.ndarray,
    x: float,
    y: float,
    yaw: float,
    track_width: float,
    wheel_track: float,
    tire_half_width: float,
) -> dict[str, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    delta_x = cloud_xyz[:, 0] - x
    delta_y = cloud_xyz[:, 1] - y
    local = np.column_stack((
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        cloud_xyz[:, 2],
    ))
    maximum_lateral = track_width / 2.0 + 0.30
    local = local[
        (np.abs(local[:, 0]) <= 0.35)
        & (np.abs(local[:, 1]) <= maximum_lateral)
    ]
    if len(local) < 40:
        raise ValueError("fewer than 40 local PCD points")
    cells = _surface_cells(local)
    cross, longitudinal, height, roughness = _plane_model(cells, wheel_track)
    lateral = _lateral_profile(cells)
    left_edge = _edge_distance(lateral, +1)
    right_edge = _edge_distance(lateral, -1)
    tire_clearance = (
        min(left_edge, right_edge) - wheel_track / 2.0 - tire_half_width
    )
    if tire_clearance <= 0.0:
        raise ValueError("mapped edge intersects the tire envelope")
    return {
        "cross_slope_deg": cross,
        "longitudinal_slope_deg": longitudinal,
        "surface_height_m": height,
        "surface_roughness_m": roughness,
        "tire_edge_clearance_m": tire_clearance,
        "minimum_wheel_edge_m": tire_clearance + tire_half_width,
    }


def annotate_profile_from_pcd(
    document: dict[str, Any],
    cloud_xyz: np.ndarray,
    wheel_track: float | None = None,
    tire_half_width: float | None = None,
) -> dict[str, Any]:
    hardware = load_hardware_config()
    wheel_track = (
        hardware.physical_wheel_track_m if wheel_track is None else float(wheel_track)
    )
    tire_half_width = (
        hardware.tire_half_width_m if tire_half_width is None else float(tire_half_width)
    )
    points = [dict(point) for point in document.get("points", [])]
    if len(points) < 10:
        raise ValueError("track profile needs at least 10 points")
    track_width = float(document.get("track_width_m", 0.9144))
    index = SpatialCloud(cloud_xyz)
    query_radius = math.hypot(0.35, track_width / 2.0 + 0.30)
    failed = []
    for number, point in enumerate(points):
        local_cloud = index.query(float(point["x"]), float(point["y"]), query_radius)
        try:
            point.update(surface_features(
                local_cloud,
                float(point["x"]),
                float(point["y"]),
                float(point.get("yaw", 0.0)),
                track_width,
                wheel_track,
                tire_half_width,
            ))
            point["terrain_source"] = "mapped_pcd_surface_edges"
        except (ValueError, np.linalg.LinAlgError) as error:
            failed.append(f"{number}:{error}")
    if failed:
        preview = "; ".join(failed[:5])
        raise ValueError(
            "PCD terrain/edge extraction failed at "
            f"{len(failed)}/{len(points)} points; {preview}"
        )
    result = dict(document)
    result["points"] = points
    result["terrain_annotation"] = {
        "source": "map.pcd",
        "method": "wheel_corridor_plane_and_bilateral_surface_break",
        "fail_closed": True,
    }
    return result
