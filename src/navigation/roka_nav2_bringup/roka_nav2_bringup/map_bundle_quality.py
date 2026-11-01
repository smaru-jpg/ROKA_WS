"""Fail-closed quality checks and atomic promotion for real map bundles."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import shutil
from typing import Any

import yaml

from roka_fast_lio_localization.archive_format import validate_archive_pair


MANIFEST_NAME = "bundle_manifest.yaml"
COMMON_REQUIRED_NONEMPTY_FILES = ("occupancy.yaml", "track_profile.yaml")
BACKEND_REQUIRED_NONEMPTY_FILES = {
    "slam_toolbox_2d": ("slam_posegraph.posegraph",),
    "fastlio_sc_pgo": ("map.pcd", "pose_graph.g2o"),
}
MANIFEST_HASH_FILES = (
    "occupancy.yaml",
    "occupancy.pgm",
    "slam_posegraph.posegraph",
    "map.pcd",
    "pose_graph.g2o",
    "track_profile.yaml",
    "session_metrics.yaml",
    "pgo_metrics.yaml",
    "optimized_path.csv",
    "keyframe_poses.bin",
    "scan_context.bin",
    "sensor_contract.yaml",
)


def recursive_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(
        item.stat().st_size
        for item in path.rglob("*")
        if item.is_file() and not item.is_symlink()
    )


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _pgm_tokens(path: Path) -> tuple[int, int, int, list[int]]:
    data = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(data):
            if data[index:index + 1] == b"#":
                newline = data.find(b"\n", index)
                index = len(data) if newline < 0 else newline + 1
            elif data[index:index + 1].isspace():
                index += 1
            else:
                break
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        return data[start:index]

    magic = token()
    if magic not in {b"P2", b"P5"}:
        raise ValueError(f"unsupported occupancy image format: {magic!r}")
    width, height, maximum = int(token()), int(token()), int(token())
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
        raise ValueError("invalid occupancy PGM header")
    while index < len(data) and data[index:index + 1].isspace():
        index += 1
    if magic == b"P2":
        pixels = [int(value) for value in data[index:].split()]
    elif maximum < 256:
        pixels = list(data[index:index + width * height])
    else:
        raw = data[index:index + 2 * width * height]
        pixels = [int.from_bytes(raw[pos:pos + 2], "big") for pos in range(0, len(raw), 2)]
    if len(pixels) != width * height:
        raise ValueError(
            f"occupancy PGM payload has {len(pixels)} pixels; expected {width * height}"
        )
    return width, height, maximum, pixels


def _component_sizes(mask: list[bool], width: int, height: int) -> list[int]:
    return [
        component["size"]
        for component in _component_geometry(mask, width, height)
    ]


def _component_geometry(
    mask: list[bool], width: int, height: int
) -> list[dict[str, int]]:
    visited = bytearray(len(mask))
    components = []
    for start, enabled in enumerate(mask):
        if not enabled or visited[start]:
            continue
        visited[start] = 1
        queue = deque([start])
        size = 0
        start_row, start_column = divmod(start, width)
        minimum_row = maximum_row = start_row
        minimum_column = maximum_column = start_column
        while queue:
            current = queue.popleft()
            size += 1
            row, column = divmod(current, width)
            minimum_row = min(minimum_row, row)
            maximum_row = max(maximum_row, row)
            minimum_column = min(minimum_column, column)
            maximum_column = max(maximum_column, column)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = row + dr, column + dc
                    if not (0 <= nr < height and 0 <= nc < width):
                        continue
                    neighbor = nr * width + nc
                    if mask[neighbor] and not visited[neighbor]:
                        visited[neighbor] = 1
                        queue.append(neighbor)
        row_span = maximum_row - minimum_row + 1
        column_span = maximum_column - minimum_column + 1
        components.append({
            "size": size,
            "row_span_cells": row_span,
            "column_span_cells": column_span,
            "maximum_span_cells": max(row_span, column_span),
        })
    return sorted(
        components,
        key=lambda component: component["size"],
        reverse=True,
    )


def occupancy_metrics(map_yaml: Path) -> dict[str, Any]:
    document = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ValueError("occupancy.yaml has no image")
    image = Path(image_value).expanduser()
    if not image.is_absolute():
        image = map_yaml.parent / image
    width, height, maximum, pixels = _pgm_tokens(image.resolve())
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    free_threshold = float(document.get("free_thresh", 0.196))
    resolution = float(document.get("resolution", 0.0))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("occupancy map resolution must be positive")
    negate = bool(int(document.get("negate", 0)))
    probabilities = [
        (value / maximum if negate else (maximum - value) / maximum)
        for value in pixels
    ]
    occupied = [value > occupied_threshold for value in probabilities]
    free = [value < free_threshold for value in probabilities]
    known = [
        is_occupied or is_free
        for is_occupied, is_free in zip(occupied, free)
    ]
    occupied_component_geometry = _component_geometry(occupied, width, height)
    occupied_components = [
        component["size"] for component in occupied_component_geometry
    ]
    known_components = _component_sizes(known, width, height)
    # Small isolated returns are common around chair legs, mesh edges and
    # VLP-16 grazing hits.  Keep them occupied in the map and in route checks;
    # only exclude components no larger than 15 cm from the map-fragmentation
    # quality count.  The physical threshold is converted using each map's own
    # resolution, so it is not tied to the current 5 cm occupancy grid.
    fragmentation_ignore_maximum_extent_m = 0.15
    significant_occupied_components = [
        component
        for component in occupied_component_geometry
        if component["maximum_span_cells"] * resolution
        > fragmentation_ignore_maximum_extent_m + 1e-9
    ]
    occupied_count = sum(occupied)
    known_count = sum(known)
    return {
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "pixel_count": width * height,
        "known_count": known_count,
        "occupied_count": occupied_count,
        "known_fraction": known_count / max(1, width * height),
        "known_component_count": len(known_components),
        "known_largest_fraction": (
            known_components[0] / known_count if known_components and known_count else 0.0
        ),
        "occupied_component_count": len(occupied_components),
        "occupied_fragment_component_count": len(
            significant_occupied_components
        ),
        "occupied_ignored_small_component_count": (
            len(occupied_components) - len(significant_occupied_components)
        ),
        "fragmentation_ignore_maximum_extent_m": (
            fragmentation_ignore_maximum_extent_m
        ),
        "occupied_largest_fraction": (
            occupied_components[0] / occupied_count
            if occupied_components and occupied_count
            else 0.0
        ),
    }


def path_occupancy_metrics(
    map_yaml: Path,
    track_profile: Path,
    *,
    wheel_lane_offset_m: float = 0.30,
) -> dict[str, Any]:
    """Check the recorded centre and wheel lanes against the saved map.

    A route can be internally well-formed while being displaced from the
    occupancy map by a bad map/odom transform.  Sample the centre line and two
    approximate outer wheel lanes in map coordinates so that such a bundle is
    rejected before it is promoted for autonomy.
    """
    document = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ValueError("occupancy.yaml has no image")
    resolution = float(document.get("resolution", 0.0))
    origin = document.get("origin")
    if (
        resolution <= 0.0
        or not isinstance(origin, list)
        or len(origin) < 3
    ):
        raise ValueError("occupancy.yaml needs a positive resolution and 3D origin")
    origin_x, origin_y, origin_yaw = map(float, origin[:3])
    if not all(math.isfinite(value) for value in (
        resolution, origin_x, origin_y, origin_yaw
    )):
        raise ValueError("occupancy map geometry contains a non-finite value")

    image = Path(image_value).expanduser()
    if not image.is_absolute():
        image = map_yaml.parent / image
    width, height, maximum, pixels = _pgm_tokens(image.resolve())
    free_threshold = float(document.get("free_thresh", 0.196))
    negate = bool(int(document.get("negate", 0)))
    probabilities = [
        (value / maximum if negate else (maximum - value) / maximum)
        for value in pixels
    ]
    free = [value < free_threshold for value in probabilities]

    profile = yaml.safe_load(track_profile.read_text(encoding="utf-8")) or {}
    if str(profile.get("frame_id", "map")).lstrip("/") != "map":
        raise ValueError("track profile is not expressed in the map frame")
    points = profile.get("points")
    if not isinstance(points, list) or len(points) < 10:
        raise ValueError("track_profile needs at least 10 points")
    if wheel_lane_offset_m <= 0.0:
        raise ValueError("wheel lane offset must be positive")

    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)

    def sample(x: float, y: float) -> tuple[bool, bool]:
        dx, dy = x - origin_x, y - origin_y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = math.floor(local_x / resolution)
        grid_y = math.floor(local_y / resolution)
        row = height - 1 - grid_y
        in_bounds = 0 <= row < height and 0 <= column < width
        return in_bounds, bool(free[row * width + column]) if in_bounds else False

    centre_in_bounds = centre_free = 0
    lane_in_bounds = lane_free = 0
    for point in points:
        x, y = float(point["x"]), float(point["y"])
        yaw = float(point.get("yaw", 0.0))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("track_profile contains a non-finite pose")
        in_bounds, is_free = sample(x, y)
        centre_in_bounds += int(in_bounds)
        centre_free += int(is_free)
        for side in (-1.0, 1.0):
            lane_x = x - side * wheel_lane_offset_m * math.sin(yaw)
            lane_y = y + side * wheel_lane_offset_m * math.cos(yaw)
            in_bounds, is_free = sample(lane_x, lane_y)
            lane_in_bounds += int(in_bounds)
            lane_free += int(is_free)
    centre_count = len(points)
    lane_count = 2 * centre_count
    return {
        "point_count": centre_count,
        "wheel_lane_offset_m": wheel_lane_offset_m,
        "centre_in_bounds_fraction": centre_in_bounds / centre_count,
        "centre_free_fraction": centre_free / centre_count,
        "wheel_lane_in_bounds_fraction": lane_in_bounds / lane_count,
        "wheel_lane_free_fraction": lane_free / lane_count,
    }


def pointcloud_count(path: Path) -> int:
    with path.open("rb") as stream:
        for raw_line in stream:
            line = raw_line.decode("ascii", errors="ignore").strip()
            if line.upper().startswith("POINTS "):
                return int(line.split()[1])
            if line.upper().startswith("DATA "):
                break
    raise ValueError("map.pcd has no POINTS header")


def graph_counts(path: Path) -> tuple[int, int]:
    vertices = 0
    edges = 0
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        value = line.lstrip()
        if value.startswith("VERTEX"):
            vertices += 1
        elif value.startswith("EDGE"):
            edges += 1
    return vertices, edges


def path_metrics(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    points = document.get("points")
    if not isinstance(points, list) or len(points) < 10:
        raise ValueError("track_profile needs at least 10 points")
    xy = []
    for point in points:
        x, y = float(point["x"]), float(point["y"])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("track_profile contains a non-finite point")
        xy.append((x, y))
    length = sum(
        math.hypot(xy[index][0] - xy[index - 1][0], xy[index][1] - xy[index - 1][1])
        for index in range(1, len(xy))
    )
    closed = document.get("closed_loop") is True
    closure = math.hypot(xy[-1][0] - xy[0][0], xy[-1][1] - xy[0][1])
    if closed:
        length += closure
    maximum_displacement = max(
        math.hypot(x - xy[0][0], y - xy[0][1]) for x, y in xy
    )
    recorded = float(document.get("recorded_path_length_m", length))
    area = 0.5 * abs(sum(
        xy[index][0] * xy[(index + 1) % len(xy)][1]
        - xy[(index + 1) % len(xy)][0] * xy[index][1]
        for index in range(len(xy))
    ))
    return {
        "point_count": len(xy),
        "closed_loop": closed,
        "path_length_m": length,
        "recorded_path_length_m": recorded,
        "maximum_displacement_m": maximum_displacement,
        "closure_distance_m": closure,
        "enclosed_area_m2": area,
    }


def bag_topic_counts(metadata: Path) -> dict[str, int]:
    document = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    information = document.get("rosbag2_bagfile_information", {})
    result = {}
    for entry in information.get("topics_with_message_count", []):
        topic = entry.get("topic_metadata", {}).get("name")
        if topic:
            result[str(topic)] = int(entry.get("message_count", 0))
    return result


def graph_trajectory_metrics(path: Path) -> dict[str, float | int]:
    positions: dict[int, tuple[float, float, float]] = {}
    edge_count = 0
    for raw_line in path.read_text(encoding="ascii", errors="strict").splitlines():
        fields = raw_line.split()
        if not fields:
            continue
        if fields[0] == "VERTEX_SE3:QUAT" and len(fields) >= 9:
            positions[int(fields[1])] = tuple(float(value) for value in fields[2:5])
        elif fields[0] == "EDGE_SE3:QUAT":
            edge_count += 1
    ordered = [positions[index] for index in sorted(positions)]
    steps = [math.dist(first, second) for first, second in zip(ordered, ordered[1:])]
    return {
        "vertices": len(ordered),
        "edges": edge_count,
        "path_length_m": sum(steps),
        "maximum_step_m": max(steps, default=0.0),
        "displacement_m": math.dist(ordered[0], ordered[-1]) if ordered else 0.0,
        "z_span_m": (
            max(point[2] for point in ordered) - min(point[2] for point in ordered)
            if ordered else 0.0
        ),
    }


def pgo_quality_metrics(path: Path) -> dict[str, float | int]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    required = (
        "keyframes",
        "map_points",
        "accepted_loops",
        "rejected_loops",
        "raw_closure_translation_m",
        "optimized_closure_translation_m",
        "maximum_adjacent_translation_deformation_m",
        "maximum_adjacent_rotation_deformation_rad",
    )
    missing = [name for name in required if name not in document]
    if missing:
        raise ValueError(f"missing SC-PGO metrics: {', '.join(missing)}")
    result = {
        "keyframes": int(document["keyframes"]),
        "map_points": int(document["map_points"]),
        "accepted_loops": int(document["accepted_loops"]),
        "rejected_loops": int(document["rejected_loops"]),
        "raw_closure_translation_m": float(document["raw_closure_translation_m"]),
        "optimized_closure_translation_m": float(
            document["optimized_closure_translation_m"]
        ),
        "maximum_adjacent_translation_deformation_m": float(
            document["maximum_adjacent_translation_deformation_m"]
        ),
        "maximum_adjacent_rotation_deformation_rad": float(
            document["maximum_adjacent_rotation_deformation_rad"]
        ),
    }
    if not all(
        math.isfinite(value)
        for key, value in result.items()
        if key not in {"keyframes", "map_points", "accepted_loops", "rejected_loops"}
    ):
        raise ValueError("SC-PGO metrics contain a non-finite value")
    return result


def optimized_path_metrics(path: Path) -> dict[str, float | int]:
    rows: list[tuple[float, float, float]] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    for line in lines[1:]:
        fields = line.split(",")
        if len(fields) < 4:
            continue
        point = tuple(float(value) for value in fields[1:4])
        if not all(math.isfinite(value) for value in point):
            raise ValueError("optimized path contains a non-finite point")
        rows.append(point)
    if len(rows) < 2:
        raise ValueError("optimized path has fewer than two poses")
    steps = [math.dist(first, second) for first, second in zip(rows, rows[1:])]
    return {
        "pose_count": len(rows),
        "path_length_m": sum(steps),
        "maximum_step_m": max(steps, default=0.0),
        "closure_distance_m": math.dist(rows[0], rows[-1]),
        "z_span_m": max(point[2] for point in rows) - min(point[2] for point in rows),
        "start_x": rows[0][0],
        "start_y": rows[0][1],
    }


def validate_bundle(directory: Path, *, require_sensor_bag: bool = True) -> dict[str, Any]:
    directory = Path(directory).expanduser().resolve()
    problems = []
    if (directory / "SESSION_ABORTED.txt").exists():
        problems.append("mapping sensor session was aborted")
    for name in COMMON_REQUIRED_NONEMPTY_FILES:
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            problems.append(f"missing or empty product: {name}")
    mapping_backend = "slam_toolbox_2d"
    metadata_path = directory / "metadata.yaml"
    if metadata_path.is_file():
        try:
            metadata_document = yaml.safe_load(
                metadata_path.read_text(encoding="utf-8")
            ) or {}
            mapping_backend = str(
                metadata_document.get("mapping_backend", "slam_toolbox_2d")
            ).strip().lower()
        except (OSError, TypeError, yaml.YAMLError) as error:
            problems.append(f"metadata validation failed: {error}")
    metrics: dict[str, Any] = {"mapping_backend": mapping_backend}
    if mapping_backend not in BACKEND_REQUIRED_NONEMPTY_FILES:
        problems.append(f"unsupported mapping backend: {mapping_backend}")
    for name in BACKEND_REQUIRED_NONEMPTY_FILES.get(mapping_backend, ()):
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            problems.append(f"missing or empty {mapping_backend} product: {name}")
    if mapping_backend == "fastlio_sc_pgo":
        for name in (
            "optimized_path.csv",
            "pgo_metrics.yaml",
            "keyframe_poses.bin",
            "scan_context.bin",
        ):
            path = directory / name
            if not path.is_file() or path.stat().st_size == 0:
                problems.append(f"missing or empty FAST-LIO/SC-PGO product: {name}")
        try:
            archives = validate_archive_pair(directory)
            metrics["sc_pgo_archives"] = archives
            if any(bool(item["legacy_magic"]) for item in archives.values()):
                metrics["sc_pgo_archive_migration_required"] = True
        except (OSError, ValueError) as error:
            problems.append(f"SC-PGO archive validation failed: {error}")
    terrain = directory / "terrain_map.bag"
    if not terrain.is_dir() or recursive_size(terrain) < 4096:
        problems.append("missing or empty product: terrain_map.bag")

    try:
        occupancy = occupancy_metrics(directory / "occupancy.yaml")
        metrics["occupancy"] = occupancy
        if occupancy["width"] < 20 or occupancy["height"] < 20:
            problems.append("occupancy map dimensions are too small")
        if occupancy["known_count"] < 500 or occupancy["occupied_count"] < 30:
            problems.append("occupancy map has insufficient observed structure")
        if occupancy["known_largest_fraction"] < 0.70:
            problems.append("occupancy known space is fragmented")
        maximum_components = max(50, int(0.15 * occupancy["occupied_count"]))
        if occupancy["occupied_fragment_component_count"] > maximum_components:
            problems.append("occupancy obstacles are excessively fragmented")
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        problems.append(f"occupancy validation failed: {error}")

    has_3d_map = mapping_backend == "fastlio_sc_pgo"
    if has_3d_map:
        try:
            points = pointcloud_count(directory / "map.pcd")
            metrics["pointcloud_points"] = points
            if points < 1000:
                problems.append(f"map.pcd has too few points: {points}")
        except (OSError, ValueError) as error:
            problems.append(f"pointcloud validation failed: {error}")

    graph: dict[str, float | int] | None = None
    if has_3d_map:
        try:
            graph = graph_trajectory_metrics(directory / "pose_graph.g2o")
            metrics["pose_graph_3d"] = graph
            metrics["pose_graph_3d_vertices"] = graph["vertices"]
            metrics["pose_graph_3d_edges"] = graph["edges"]
            if graph["vertices"] < 2 or graph["edges"] < 1:
                problems.append(
                    "3D pose graph is incomplete: "
                    f"vertices={graph['vertices']}, edges={graph['edges']}"
                )
            if graph["maximum_step_m"] > 2.0:
                problems.append(
                    f"3D pose graph has a {graph['maximum_step_m']:.3f} m pose jump"
                )
        except (OSError, TypeError, ValueError) as error:
            problems.append(f"3D pose graph validation failed: {error}")

    pgo: dict[str, float | int] | None = None
    if mapping_backend == "fastlio_sc_pgo":
        try:
            pgo = pgo_quality_metrics(directory / "pgo_metrics.yaml")
            metrics["sc_pgo"] = pgo
            if pgo["keyframes"] < 2 or pgo["map_points"] < 1000:
                problems.append("SC-PGO output has insufficient keyframes or map points")
            if pgo["maximum_adjacent_translation_deformation_m"] > 0.25:
                problems.append("SC-PGO introduced an excessive adjacent translation deformation")
            if pgo["maximum_adjacent_rotation_deformation_rad"] > 0.15:
                problems.append("SC-PGO introduced an excessive adjacent rotation deformation")
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            problems.append(f"SC-PGO metric validation failed: {error}")
        try:
            optimized_path = optimized_path_metrics(directory / "optimized_path.csv")
            metrics["optimized_path"] = optimized_path
            if optimized_path["maximum_step_m"] > 2.0:
                problems.append("optimized path contains a pose jump greater than 2.0 m")
        except (OSError, TypeError, ValueError) as error:
            optimized_path = None
            problems.append(f"optimized path validation failed: {error}")
    else:
        optimized_path = None

    try:
        track = path_metrics(directory / "track_profile.yaml")
        metrics["track"] = track
        if track["path_length_m"] < 0.75:
            problems.append(
                f"saved path is too short: {track['path_length_m']:.3f} m < 0.750 m"
            )
        if track["maximum_displacement_m"] < 0.50:
            problems.append("saved path has less than 0.50 m real displacement")
        if track["recorded_path_length_m"] > max(
            track["path_length_m"] * 1.75,
            track["path_length_m"] + 0.50,
        ):
            problems.append(
                "SLAM path collapsed during smoothing: recorded and saved lengths disagree"
            )
        if track["closed_loop"] and track["path_length_m"] < 2.0:
            problems.append("closed-loop path is shorter than 2.0 m")
        if track["closed_loop"] and track["enclosed_area_m2"] < 0.25:
            problems.append("closed-loop path encloses less than 0.25 m^2")
        if mapping_backend == "fastlio_sc_pgo" and track["closed_loop"] and pgo is not None:
            if pgo["accepted_loops"] < 1:
                problems.append("closed-loop path has no accepted SC-PGO loop closure")
            if pgo["optimized_closure_translation_m"] > 0.30:
                problems.append(
                    "SC-PGO optimized closure exceeds 0.30 m: "
                    f"{pgo['optimized_closure_translation_m']:.3f} m"
                )
        if graph is not None and track["recorded_path_length_m"] > 0.0:
            graph_ratio = (
                float(graph["path_length_m"]) / track["recorded_path_length_m"]
            )
            metrics["pose_graph_to_recorded_path_ratio"] = graph_ratio
            if not 0.50 <= graph_ratio <= 1.75:
                problems.append(
                    "3D pose graph and recorded path disagree: "
                    f"graph={graph['path_length_m']:.2f}m, "
                    f"recorded={track['recorded_path_length_m']:.2f}m"
                )
        if optimized_path is not None and track["recorded_path_length_m"] > 0.0:
            optimized_ratio = (
                float(optimized_path["path_length_m"]) /
                track["recorded_path_length_m"]
            )
            metrics["optimized_path_to_recorded_path_ratio"] = optimized_ratio
            if not 0.50 <= optimized_ratio <= 1.75:
                problems.append(
                    "SC-PGO optimized path and recorded path disagree: "
                    f"optimized={optimized_path['path_length_m']:.2f}m, "
                    f"recorded={track['recorded_path_length_m']:.2f}m"
                )
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        problems.append(f"track validation failed: {error}")

    try:
        alignment = path_occupancy_metrics(
            directory / "occupancy.yaml", directory / "track_profile.yaml"
        )
        metrics["path_occupancy_alignment"] = alignment
        if alignment["centre_in_bounds_fraction"] < 0.95:
            problems.append("recorded path centre falls outside the occupancy map")
        if alignment["centre_free_fraction"] < 0.90:
            problems.append("recorded path centre is not aligned with mapped free space")
        if alignment["wheel_lane_in_bounds_fraction"] < 0.90:
            problems.append("recorded wheel lanes fall outside the occupancy map")
        if alignment["wheel_lane_free_fraction"] < 0.80:
            problems.append("recorded wheel lanes do not have enough mapped free space")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        problems.append(f"path/map alignment validation failed: {error}")

    if require_sensor_bag:
        sensor_metadata = directory / "sensors" / "metadata.yaml"
        if not sensor_metadata.is_file():
            problems.append("sensor bag did not finalize metadata.yaml")
        else:
            try:
                counts = bag_topic_counts(sensor_metadata)
                metrics["sensor_message_counts"] = counts
                for topic in ("/velodyne_packets", "/imu/data", "/track/recorded_path"):
                    if counts.get(topic, 0) <= 0:
                        problems.append(f"sensor bag has no messages on {topic}")
                if max(counts.get("/odom", 0), counts.get("/wheel/odom", 0)) <= 0:
                    problems.append("sensor bag has no wheel odometry messages")
            except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
                problems.append(f"sensor bag metadata validation failed: {error}")
            contract_path = directory / "sensor_contract.yaml"
            if not contract_path.is_file():
                problems.append("strict sensor_contract.yaml is missing")
            else:
                try:
                    sensor_contract = yaml.safe_load(
                        contract_path.read_text(encoding="utf-8")
                    ) or {}
                    metrics["sensor_contract"] = sensor_contract
                    if int(sensor_contract.get("schema_version", 0)) < 2:
                        problems.append("sensor contract schema is obsolete")
                    if sensor_contract.get("result") != "PASS":
                        contract_failures = sensor_contract.get("failures") or [
                            "unspecified sensor-contract failure"
                        ]
                        problems.append(
                            "sensor bag failed the runtime contract: "
                            + ", ".join(str(item) for item in contract_failures)
                        )
                    fingerprint = sensor_contract.get("bag_fingerprint") or {}
                    metadata_hash = file_sha256(sensor_metadata)
                    if fingerprint.get("metadata_sha256") != metadata_hash:
                        problems.append("sensor bag metadata changed after its audit")
                    expected_files = fingerprint.get("data_files") or []
                    actual_files = [
                        {"name": item.name, "size_bytes": item.stat().st_size}
                        for item in sorted((directory / "sensors").iterdir())
                        if item.is_file() and (
                            item.name.endswith(".db3")
                            or item.name.endswith(".db3.zstd")
                        )
                    ]
                    if expected_files != actual_files:
                        problems.append("sensor bag data files changed after their audit")
                except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
                    problems.append(f"sensor contract validation failed: {error}")
        session_metrics_path = directory / "session_metrics.yaml"
        if not session_metrics_path.is_file():
            problems.append("mapping session metrics are missing")
        else:
            try:
                session = yaml.safe_load(
                    session_metrics_path.read_text(encoding="utf-8")
                ) or {}
                metrics["session"] = session
                translation_jump = float(
                    session["maximum_map_odom_translation_jump_m"]
                )
                yaw_jump = float(session["maximum_map_odom_yaw_jump_deg"])
                odom_distance = float(session["odom_cumulative_distance_m"])
                if translation_jump > 0.20:
                    problems.append(
                        f"map->odom translation jumped {translation_jump:.3f} m"
                    )
                if yaw_jump > 5.0:
                    problems.append(f"map->odom yaw jumped {yaw_jump:.2f} deg")
                saved_distance = metrics.get("track", {}).get("recorded_path_length_m")
                if saved_distance is not None and odom_distance > 0.0:
                    ratio = float(saved_distance) / odom_distance
                    if not 0.75 <= ratio <= 1.25:
                        problems.append(
                            "recorded path and wheel odometry distance disagree: "
                            f"path={saved_distance:.2f}m, odom={odom_distance:.2f}m"
                        )
            except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
                problems.append(f"mapping session metric validation failed: {error}")

    if problems:
        raise ValueError("; ".join(problems))
    metrics["bundle_size_bytes"] = recursive_size(directory)
    return metrics


def validate_acquisition_bundle(directory: Path) -> dict[str, Any]:
    """Validate one environment-independent raw mapping acquisition.

    Acquisition deliberately has no occupancy, point-cloud map, pose graph or
    terrain product.  Those products are derived later by the selected 2D or
    3D processor.  Keeping this validator separate prevents a raw sensor run
    from being rejected merely because no mapping backend was selected yet.
    """
    directory = Path(directory).expanduser().resolve()
    problems: list[str] = []
    if (directory / "SESSION_ABORTED.txt").exists():
        problems.append("mapping sensor session was aborted")

    for name in (
        "track_profile.yaml",
        "session_metrics.yaml",
        "session_contract.yaml",
        "sensor_contract.yaml",
        "metadata.yaml",
    ):
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            problems.append(f"missing or empty acquisition product: {name}")

    metrics: dict[str, Any] = {"mapping_backend": "raw_sensor_capture"}
    sensor_metadata = directory / "sensors" / "metadata.yaml"
    if not sensor_metadata.is_file():
        problems.append("sensor bag did not finalize metadata.yaml")
    else:
        try:
            counts = bag_topic_counts(sensor_metadata)
            metrics["sensor_message_counts"] = counts
            for topic in ("/velodyne_packets", "/imu/data", "/track/recorded_path"):
                if counts.get(topic, 0) <= 0:
                    problems.append(f"sensor bag has no messages on {topic}")
            if max(counts.get("/odom", 0), counts.get("/wheel/odom", 0)) <= 0:
                problems.append("sensor bag has no wheel odometry messages")
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            problems.append(f"sensor bag metadata validation failed: {error}")

    contract_path = directory / "sensor_contract.yaml"
    if contract_path.is_file() and sensor_metadata.is_file():
        try:
            contract = yaml.safe_load(contract_path.read_text(encoding="utf-8")) or {}
            metrics["sensor_contract"] = contract
            if int(contract.get("schema_version", 0)) < 2:
                problems.append("sensor contract schema is obsolete")
            if contract.get("result") != "PASS":
                problems.append(
                    "sensor bag failed the runtime contract: "
                    + ", ".join(str(item) for item in contract.get("failures") or [])
                )
            fingerprint = contract.get("bag_fingerprint") or {}
            if fingerprint.get("metadata_sha256") != file_sha256(sensor_metadata):
                problems.append("sensor bag metadata changed after its audit")
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            problems.append(f"sensor contract validation failed: {error}")

    try:
        track = path_metrics(directory / "track_profile.yaml")
        metrics["track"] = track
        if track["path_length_m"] < 0.75:
            problems.append(
                f"saved path is too short: {track['path_length_m']:.3f} m < 0.750 m"
            )
        if track["maximum_displacement_m"] < 0.50:
            problems.append("saved path has less than 0.50 m real displacement")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        problems.append(f"acquisition path validation failed: {error}")

    if problems:
        raise ValueError("; ".join(problems))
    metrics["bundle_size_bytes"] = recursive_size(directory)
    return metrics


def write_manifest(directory: Path, metrics: dict[str, Any]) -> Path:
    path = directory / MANIFEST_NAME
    temporary = directory / f".{MANIFEST_NAME}.preparing"
    product_sha256 = {
        name: file_sha256(directory / name)
        for name in MANIFEST_HASH_FILES
        if (directory / name).is_file()
    }
    document = {
        "schema_version": 3,
        "status": "complete",
        "validated_utc": datetime.now(timezone.utc).isoformat(),
        "product_sha256": product_sha256,
        "quality": metrics,
    }
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def bundle_is_complete(directory: Path) -> bool:
    path = Path(directory) / MANIFEST_NAME
    if not path.is_file():
        return False
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return False
    if document.get("status") != "complete":
        return False
    if int(document.get("schema_version", 0)) < 3:
        return True
    hashes = document.get("product_sha256")
    if not isinstance(hashes, dict) or not hashes:
        return False
    try:
        for name, expected in hashes.items():
            product = Path(directory) / str(name)
            if not product.is_file() or file_sha256(product) != str(expected):
                return False
    except OSError:
        return False
    return True


def requires_embedded_sensor_bag(directory: Path) -> bool:
    """Return whether runtime integrity checks must find a local rosbag.

    Legacy/live map bundles contain their acquisition bag.  Offline processed
    maps intentionally keep the large immutable bag once under datasets and
    carry only its signed audit/metrics, so autonomy remains deployable without
    copying the raw bag into every 2D and 3D product.
    """
    metadata = Path(directory) / "metadata.yaml"
    try:
        document = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    except (OSError, TypeError, yaml.YAMLError):
        return True
    environment = str(document.get("environment", "")).strip().lower()
    return not (
        environment in {"processed_2d", "processed_3d"}
        and bool(str(document.get("source_acquisition", "")).strip())
    )


def promote_bundle(
    staging: Path,
    final: Path,
    *,
    overwrite_existing: bool,
) -> Path | None:
    staging = staging.expanduser().resolve()
    final = final.expanduser().resolve()
    if staging == final:
        return None
    if not staging.is_dir() or not bundle_is_complete(staging):
        raise ValueError("staging bundle is not marked complete")
    if final.exists() and not overwrite_existing:
        raise FileExistsError(f"final map already exists: {final}")
    final.parent.mkdir(parents=True, exist_ok=True)
    backup_root = final.parent / ".backup"
    backup = backup_root / final.name
    moved_old = False
    if final.exists():
        backup_root.mkdir(parents=True, exist_ok=True)
        if backup.exists():
            if backup.is_dir():
                shutil.rmtree(backup)
            else:
                backup.unlink()
        final.replace(backup)
        moved_old = True
    try:
        staging.replace(final)
    except Exception:
        if moved_old and backup.exists() and not final.exists():
            backup.replace(final)
        raise
    return backup if moved_old else None
