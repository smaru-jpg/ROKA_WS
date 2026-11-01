"""Export the exact MuJoCo PK-3A collision surface as a 2.5D map.

The exporter deliberately ray-casts only geoms in MuJoCo group 4.  In the
packaged scenes that group contains the rideable heightfields and excludes
the floor, visual meshes, start-line decoration, and the robot.  Sampling the
compiled MJCF keeps this artifact aligned with the surface used by physics.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.ndimage import binary_closing, distance_transform_edt

from .closed_loop_pipeline import _half_straight_spec, _turn_variant_spec
from .module_geometry import (
    footprint_contains,
    load_json,
    mirror_module_y,
    orient_course_across_width,
    package_root,
)
from .port_geometry import load_port_sources


TRACK_GEOM_GROUP = 4
DEFAULT_RESOLUTION_M = 0.025
DEFAULT_MARGIN_M = 0.10


def _track_geom_ids(model: mujoco.MjModel) -> np.ndarray:
    groups = np.asarray(model.geom_group)
    types = np.asarray(model.geom_type)
    return np.flatnonzero(
        (groups == TRACK_GEOM_GROUP)
        & (types == int(mujoco.mjtGeom.mjGEOM_HFIELD))
    )


def _track_bounds(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_ids: np.ndarray,
    margin_m: float,
) -> tuple[float, float, float, float]:
    points: list[np.ndarray] = []
    for geom_id in geom_ids:
        hfield_id = int(model.geom_dataid[geom_id])
        half_x, half_y = model.hfield_size[hfield_id, :2]
        rotation = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        centre = np.asarray(data.geom_xpos[geom_id])
        for x in (-half_x, half_x):
            for y in (-half_y, half_y):
                points.append(centre + rotation @ np.array([x, y, 0.0]))
    xy = np.asarray(points)[:, :2]
    return (
        float(xy[:, 0].min() - margin_m),
        float(xy[:, 0].max() + margin_m),
        float(xy[:, 1].min() - margin_m),
        float(xy[:, 1].max() + margin_m),
    )


def _aligned_axis(lower: float, upper: float, resolution_m: float) -> tuple[float, np.ndarray]:
    origin = math.floor(lower / resolution_m) * resolution_m
    end = math.ceil(upper / resolution_m) * resolution_m
    count = int(round((end - origin) / resolution_m))
    centres = origin + (np.arange(count, dtype=np.float64) + 0.5) * resolution_m
    return origin, centres


def _max_neighbor_step(
    elevation: np.ndarray,
    valid: np.ndarray,
    module_index: np.ndarray,
    module_count: int,
) -> np.ndarray:
    result = np.zeros(elevation.shape, dtype=np.float32)
    rows, columns = elevation.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            y0 = max(0, -dy)
            y1 = min(rows, rows - dy)
            x0 = max(0, -dx)
            x1 = min(columns, columns - dx)
            neighbour_y0 = y0 + dy
            neighbour_y1 = y1 + dy
            neighbour_x0 = x0 + dx
            neighbour_x1 = x1 + dx
            current_modules = module_index[y0:y1, x0:x1]
            neighbour_modules = module_index[
                neighbour_y0:neighbour_y1,
                neighbour_x0:neighbour_x1,
            ]
            module_delta = np.abs(current_modules - neighbour_modules)
            connected_modules = (
                (module_delta == 0)
                | (module_delta == 1)
                | (module_delta == module_count - 1)
            )
            pair_valid = (
                valid[y0:y1, x0:x1]
                & valid[neighbour_y0:neighbour_y1, neighbour_x0:neighbour_x1]
                & connected_modules
            )
            delta = np.zeros(pair_valid.shape, dtype=np.float32)
            delta[pair_valid] = np.abs(
                elevation[y0:y1, x0:x1][pair_valid]
                - elevation[
                    neighbour_y0:neighbour_y1,
                    neighbour_x0:neighbour_x1,
                ][pair_valid]
            )
            target = result[y0:y1, x0:x1]
            np.maximum(target, delta, out=target)
    result[~valid] = np.nan
    return result


def _module_catalog(config: dict[str, Any]) -> dict[str, Any]:
    registry_path = Path(config["port_registry"])
    if not registry_path.is_absolute():
        registry_path = package_root() / registry_path
    _, source_modules = load_port_sources(registry_path)
    modules = dict(source_modules)
    course_lengths = config.get("course_axis_lengths_m", {})
    course_blends = config.get("course_axis_blend_modes", {})
    for source_name in config.get("course_axis_modules", []):
        modules[f"{source_name}_C"] = orient_course_across_width(
            modules[source_name],
            f"{source_name}_C",
            course_lengths.get(source_name),
            course_blends.get(source_name, "linear"),
        )
    for source_name in config.get("mirror_y_modules", []):
        modules[f"{source_name}_M"] = mirror_module_y(
            modules[source_name], f"{source_name}_M"
        )
    for raw in config.get("turn_variants", []):
        modules[str(raw["name"])] = _turn_variant_spec(
            modules[str(raw["source"])],
            str(raw["name"]),
            float(raw["turn_deg"]),
        )
    if "half_straight" in config:
        modules["F_HALF"] = _half_straight_spec(config["half_straight"])
    for raw in config.get("custom_modules", []):
        custom = _half_straight_spec(raw)
        modules[custom.name] = custom
    return modules


def _rideable_mask(
    scene_path: Path,
    x_centres: np.ndarray,
    y_centres: np.ndarray,
    collision_valid: np.ndarray,
    module_index: np.ndarray,
    geom_ids: np.ndarray,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    track_root = scene_path.parent.parent
    metadata_path = track_root / "assets" / "pk3a_closed_loop_metadata.json"
    config_path = package_root() / "config" / "pk3a_closed_loop.json"
    if not metadata_path.is_file() or not config_path.is_file():
        raise FileNotFoundError(
            "rideable footprint requires pk3a_closed_loop metadata and config"
        )
    track_metadata = load_json(metadata_path)
    instances = track_metadata["instances"]
    if len(instances) != int(geom_ids.size):
        raise RuntimeError(
            "track metadata instance count does not match compiled collision geoms"
        )
    modules = _module_catalog(load_json(config_path))
    rideable = np.zeros(collision_valid.shape, dtype=bool)
    for index, instance in enumerate(instances):
        selected = collision_valid & (module_index == index)
        rows, columns = np.nonzero(selected)
        if rows.size == 0:
            continue
        world_x = x_centres[columns]
        world_y = y_centres[rows]
        pose_x, pose_y = instance["pose_xy_m"]
        yaw = math.radians(float(instance["yaw_deg"]))
        delta_x = world_x - float(pose_x)
        delta_y = world_y - float(pose_y)
        local_x = math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
        local_y = -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
        inside = footprint_contains(
            modules[instance["module"]],
            local_x,
            local_y,
            tolerance=1.0e-6,
        )
        rideable[rows[inside], columns[inside]] = True
    # At a module seam the upper of two overlapping rectangular hfields can
    # be the lowered apron of one module while the other module contains the
    # intended riding plate.  A one-cell closing removes only these raster
    # cracks (the analytic seam error is millimetric) and remains constrained
    # to actual group-4 collision support.
    rideable = binary_closing(
        rideable, structure=np.ones((3, 3), dtype=bool)
    ) & collision_valid
    return rideable, instances


def sample_ground_truth(
    scene_path: Path,
    resolution_m: float = DEFAULT_RESOLUTION_M,
    margin_m: float = DEFAULT_MARGIN_M,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Sample group-4 heightfields in *scene_path* and return map layers."""
    scene_path = scene_path.expanduser().resolve()
    if not scene_path.is_file():
        raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    if margin_m < resolution_m:
        raise ValueError("margin_m must be at least one grid cell")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom_ids = _track_geom_ids(model)
    if geom_ids.size == 0:
        raise RuntimeError("scene has no group-4 heightfield collision geoms")

    # mj_ray ignores fully transparent geometry.  This is an isolated offline
    # model, so making collision geoms visible in memory does not alter the
    # packaged scene or a running simulation.
    model.geom_rgba[geom_ids, 3] = 1.0

    min_x, max_x, min_y, max_y = _track_bounds(model, data, geom_ids, margin_m)
    origin_x, x_centres = _aligned_axis(min_x, max_x, resolution_m)
    origin_y, y_centres = _aligned_axis(min_y, max_y, resolution_m)
    rows = y_centres.size
    columns = x_centres.size

    elevation = np.full((rows, columns), np.nan, dtype=np.float32)
    normal_x = np.full_like(elevation, np.nan)
    normal_y = np.full_like(elevation, np.nan)
    normal_z = np.full_like(elevation, np.nan)
    geom_id_layer = np.full((rows, columns), -1, dtype=np.int16)
    module_index_layer = np.full((rows, columns), -1, dtype=np.int16)
    geom_id_set = {int(geom_id) for geom_id in geom_ids}
    geom_to_module = {
        int(geom_id): module_index
        for module_index, geom_id in enumerate(geom_ids)
    }

    hfield_sizes = np.asarray(model.hfield_size)[:, 2]
    ray_origin_z = float(np.max(data.geom_xpos[geom_ids, 2]) + np.max(hfield_sizes) + 2.0)
    direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geom_groups = np.zeros(mujoco.mjNGROUP, dtype=np.uint8)
    geom_groups[TRACK_GEOM_GROUP] = 1
    hit_geom_id = np.array([-1], dtype=np.int32)
    hit_normal = np.empty(3, dtype=np.float64)
    point = np.array([0.0, 0.0, ray_origin_z], dtype=np.float64)

    for row, y in enumerate(y_centres):
        point[1] = y
        for column, x in enumerate(x_centres):
            point[0] = x
            hit_geom_id[0] = -1
            distance = mujoco.mj_ray(
                model,
                data,
                point,
                direction,
                geom_groups,
                1,
                -1,
                hit_geom_id,
                hit_normal,
            )
            if distance < 0.0 or int(hit_geom_id[0]) not in geom_id_set:
                continue
            elevation[row, column] = ray_origin_z - float(distance)
            normal_x[row, column] = float(hit_normal[0])
            normal_y[row, column] = float(hit_normal[1])
            normal_z[row, column] = float(hit_normal[2])
            geom_id_layer[row, column] = int(hit_geom_id[0])
            module_index_layer[row, column] = geom_to_module[int(hit_geom_id[0])]

    collision_valid = geom_id_layer >= 0
    collision_hit_count = int(np.count_nonzero(collision_valid))
    if collision_hit_count == 0:
        raise RuntimeError("no track cells were hit by vertical rays")
    rideable, instances = _rideable_mask(
        scene_path,
        x_centres,
        y_centres,
        collision_valid,
        module_index_layer,
        geom_ids,
    )
    hit_count = int(np.count_nonzero(rideable))
    if hit_count == 0:
        raise RuntimeError("analytic module footprints rejected all track cells")

    slope_deg = np.full_like(elevation, np.nan)
    slope_deg[rideable] = np.degrees(
        np.arccos(np.clip(normal_z[rideable], -1.0, 1.0))
    ).astype(np.float32)
    edge_distance_m = distance_transform_edt(rideable).astype(np.float32) * resolution_m
    edge_distance_m[~rideable] = np.nan
    step_height_m = _max_neighbor_step(
        elevation, rideable, module_index_layer, int(geom_ids.size)
    )

    layers = {
        "elevation": elevation,
        "normal_x": normal_x,
        "normal_y": normal_y,
        "normal_z": normal_z,
        "slope_deg": slope_deg,
        "step_height_m": step_height_m,
        "edge_distance_m": edge_distance_m,
        "collision_valid": collision_valid,
        "rideable": rideable,
        "valid": rideable,
        "geom_id": geom_id_layer,
        "module_index": module_index_layer,
    }
    geom_names = {
        str(int(geom_id)): mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)
        )
        for geom_id in geom_ids
    }
    metadata: dict[str, Any] = {
        "schema_version": 1,
        "source_scene": str(scene_path),
        "source_scene_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        "frame_id": "map",
        "axis_convention": "+X forward at start, +Y left, +Z up",
        "collision_geom_group": TRACK_GEOM_GROUP,
        "track_geom_count": int(geom_ids.size),
        "track_geoms": geom_names,
        "module_instances": [
            {
                "index": int(instance["index"]),
                "id": instance["id"],
                "module": instance["module"],
            }
            for instance in instances
        ],
        "resolution_m": resolution_m,
        "origin_xy_m": [origin_x, origin_y],
        "size_cells": [int(columns), int(rows)],
        "length_xy_m": [columns * resolution_m, rows * resolution_m],
        "sampled_bounds_xy_m": [
            origin_x,
            origin_x + columns * resolution_m,
            origin_y,
            origin_y + rows * resolution_m,
        ],
        "valid_cell_count": hit_count,
        "collision_cell_count": collision_hit_count,
        "excluded_hfield_apron_cell_count": collision_hit_count - hit_count,
        "track_area_m2": hit_count * resolution_m * resolution_m,
        "minimum_elevation_m": float(np.nanmin(elevation[rideable])),
        "maximum_elevation_m": float(np.nanmax(elevation[rideable])),
        "maximum_slope_deg": float(np.nanmax(slope_deg)),
        "maximum_core_slope_deg": float(
            np.nanmax(slope_deg[rideable & (edge_distance_m >= 2.0 * resolution_m)])
        ),
        "maximum_neighbor_step_m": float(np.nanmax(step_height_m)),
        "layers": list(layers),
    }
    return layers, metadata


def export_ground_truth(
    scene_path: Path,
    output_directory: Path,
    resolution_m: float = DEFAULT_RESOLUTION_M,
    margin_m: float = DEFAULT_MARGIN_M,
) -> dict[str, Any]:
    layers, metadata = sample_ground_truth(scene_path, resolution_m, margin_m)
    output_directory = output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    npz_path = output_directory / "pk3a_ground_truth.npz"
    metadata_path = output_directory / "pk3a_ground_truth.json"
    np.savez_compressed(npz_path, **layers)
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {**metadata, "map_file": str(npz_path), "metadata_file": str(metadata_path)}


def _default_scene() -> Path:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("roka_mujoco"))
    return share / "description" / "pk3a" / "models" / "pk3a_closed_loop.xml"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export the exact PK-3A MuJoCo collision surface."
    )
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path.cwd() / "maps" / "sim_pk3a_ground_truth",
    )
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION_M)
    parser.add_argument("--margin", type=float, default=DEFAULT_MARGIN_M)
    arguments = parser.parse_args()
    result = export_ground_truth(
        arguments.scene or _default_scene(),
        arguments.output_directory,
        arguments.resolution,
        arguments.margin,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
