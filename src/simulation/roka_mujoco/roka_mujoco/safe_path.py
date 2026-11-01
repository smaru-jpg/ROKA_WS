"""Generate a low-slope, wheel-safe closed path for the MuJoCo PK-3A robot."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_root

from .ground_truth_map import _module_catalog
from .module_geometry import load_json, package_root, surface_point


_HARDWARE = load_hardware_config()
WHEELBASE_M = _HARDWARE.wheelbase_m
WHEEL_TRACK_M = _HARDWARE.physical_wheel_track_m
TIRE_HALF_WIDTH_M = _HARDWARE.tire_half_width_m
DEFAULT_EDGE_MARGIN_M = 0.050
DEFAULT_SAMPLE_SPACING_M = 0.08
DEFAULT_LATERAL_STEP_M = 0.025


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class GroundTruthGrid:
    def __init__(self, map_file: Path, metadata_file: Path):
        self.map_file = map_file.expanduser().resolve()
        self.metadata_file = metadata_file.expanduser().resolve()
        self.metadata = load_json(self.metadata_file)
        archive = np.load(self.map_file)
        self.elevation = archive["elevation"]
        self.slope = archive["slope_deg"]
        self.edge_distance = archive["edge_distance_m"]
        self.rideable = archive["rideable"]
        self.origin_x, self.origin_y = [
            float(value) for value in self.metadata["origin_xy_m"]
        ]
        self.resolution = float(self.metadata["resolution_m"])

    def indices(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        columns = np.floor((x - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((y - self.origin_y) / self.resolution).astype(int)
        inside = (
            (rows >= 0)
            & (columns >= 0)
            & (rows < self.rideable.shape[0])
            & (columns < self.rideable.shape[1])
        )
        return rows, columns, inside

    def sample(self, x: np.ndarray, y: np.ndarray) -> dict[str, np.ndarray]:
        x_values = np.asarray(x, dtype=float)
        y_values = np.asarray(y, dtype=float)
        rows, columns, inside = self.indices(x_values, y_values)
        valid = np.zeros(x_values.shape, dtype=bool)
        elevation = np.full(x_values.shape, np.nan, dtype=float)
        slope = np.full(x_values.shape, np.nan, dtype=float)
        edge = np.full(x_values.shape, np.nan, dtype=float)
        selected = np.nonzero(inside)[0]
        if selected.size:
            selected_rows = rows[selected]
            selected_columns = columns[selected]
            selected_valid = self.rideable[selected_rows, selected_columns]
            accepted = selected[selected_valid]
            accepted_rows = rows[accepted]
            accepted_columns = columns[accepted]
            valid[accepted] = True
            elevation[accepted] = self.elevation[accepted_rows, accepted_columns]
            slope[accepted] = self.slope[accepted_rows, accepted_columns]
            edge[accepted] = self.edge_distance[accepted_rows, accepted_columns]
        return {
            "valid": valid,
            "elevation": elevation,
            "slope_deg": slope,
            "edge_distance_m": edge,
        }


def _world_pose(instance: dict[str, Any], spec: Any, progress: float, lateral: float) -> tuple[float, float, float]:
    reverse = instance.get("input_port") == "outlet" or (
        instance.get("input_port") is None and instance.get("output_port") == "inlet"
    )
    local_lateral = -lateral if reverse else lateral
    local = np.asarray(surface_point(spec, progress, local_lateral), dtype=float)
    epsilon = min(0.01, 0.1 * spec.length)
    before = np.asarray(
        surface_point(spec, max(0.0, progress - epsilon), local_lateral), dtype=float
    )
    after = np.asarray(
        surface_point(spec, min(spec.length, progress + epsilon), local_lateral), dtype=float
    )
    local_yaw = math.atan2(after[1] - before[1], after[0] - before[0])
    if reverse:
        local_yaw = _wrap_angle(local_yaw + math.pi)
    yaw = math.radians(float(instance["yaw_deg"]))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    pose_x, pose_y = [float(value) for value in instance["pose_xy_m"]]
    return (
        float(pose_x + cos_yaw * local[0] - sin_yaw * local[1]),
        float(pose_y + sin_yaw * local[0] + cos_yaw * local[1]),
        float(_wrap_angle(yaw + local_yaw)),
    )


def _candidate_metrics(
    grid: GroundTruthGrid,
    x: float,
    y: float,
    yaw: float,
    required_wheel_edge_m: float,
) -> dict[str, float] | None:
    half_wheelbase = 0.5 * WHEELBASE_M
    half_track = 0.5 * WHEEL_TRACK_M
    wheel_local = np.array([
        [half_wheelbase, half_track],
        [half_wheelbase, -half_track],
        [-half_wheelbase, half_track],
        [-half_wheelbase, -half_track],
    ])
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw)],
        [math.sin(yaw), math.cos(yaw)],
    ])
    wheel_world = wheel_local @ rotation.T + np.array([x, y])
    samples = grid.sample(wheel_world[:, 0], wheel_world[:, 1])
    if not np.all(samples["valid"]):
        return None
    minimum_edge = float(np.min(samples["edge_distance_m"]))
    if minimum_edge < required_wheel_edge_m:
        return None
    slopes = samples["slope_deg"]
    elevations = samples["elevation"]
    if not np.isfinite(slopes).all():
        return None
    if not np.isfinite(elevations).all():
        return None
    # Wheel order is front-left, front-right, rear-left, rear-right.  The
    # height differences below describe the attitude the rigid four-wheel
    # chassis must assume more reliably than a single noisy height-map pixel.
    left_height = 0.5 * float(elevations[0] + elevations[2])
    right_height = 0.5 * float(elevations[1] + elevations[3])
    front_height = 0.5 * float(elevations[0] + elevations[1])
    rear_height = 0.5 * float(elevations[2] + elevations[3])
    cross_slope = math.degrees(
        math.atan2(left_height - right_height, WHEEL_TRACK_M)
    )
    longitudinal_slope = math.degrees(
        math.atan2(front_height - rear_height, WHEELBASE_M)
    )
    left_edge = float(np.min(samples["edge_distance_m"][[0, 2]]))
    right_edge = float(np.min(samples["edge_distance_m"][[1, 3]]))
    downhill_edge = right_edge if cross_slope >= 0.0 else left_edge
    centre = grid.sample(np.array([x]), np.array([y]))
    if not bool(centre["valid"][0]):
        return None
    return {
        "minimum_wheel_edge_m": minimum_edge,
        "maximum_wheel_slope_deg": float(np.max(slopes)),
        "mean_wheel_slope_deg": float(np.mean(slopes)),
        "cross_slope_deg": float(cross_slope),
        "longitudinal_slope_deg": float(longitudinal_slope),
        "left_wheel_edge_m": left_edge,
        "right_wheel_edge_m": right_edge,
        "downhill_wheel_edge_m": downhill_edge,
        "elevation_m": float(centre["elevation"][0]),
    }


def _candidate_cost(metrics: dict[str, float], preferred_edge_m: float) -> float:
    slope = metrics["maximum_wheel_slope_deg"]
    mean_slope = metrics["mean_wheel_slope_deg"]
    cross_slope = abs(metrics["cross_slope_deg"])
    edge = metrics["minimum_wheel_edge_m"]
    downhill_edge = metrics["downhill_wheel_edge_m"]
    edge_deficit = max(0.0, preferred_edge_m - edge) / preferred_edge_m
    downhill_preferred = preferred_edge_m + 0.04 * min(1.0, cross_slope / 30.0)
    downhill_deficit = max(0.0, downhill_preferred - downhill_edge) / downhill_preferred
    return (
        1.6 * (slope / 35.0) ** 2
        + 0.4 * (mean_slope / 35.0) ** 2
        # Cross-slope directly drives downhill tyre scrub/sliding on this
        # rigid, suspension-less four-wheel model, so weight it separately.
        + 2.4 * (cross_slope / 25.0) ** 2
        + 24.0 * edge_deficit**2
        # A banked rigid vehicle moves downhill while it settles into roll,
        # even when the tyre contacts retain static friction.  Reserve extra
        # corridor on that side instead of placing the path at the low edge.
        + 80.0 * downhill_deficit**2
    )


def _choose_closed_offsets(
    costs: np.ndarray,
    offsets: np.ndarray,
    maximum_lateral_change_m: float,
) -> np.ndarray:
    count, candidate_count = costs.shape
    transition_scale = max(float(np.diff(offsets).min()), 1.0e-6)
    best_total = math.inf
    best_indices: np.ndarray | None = None
    for start in range(candidate_count):
        if not math.isfinite(float(costs[0, start])):
            continue
        previous = np.full(candidate_count, math.inf, dtype=float)
        previous[start] = costs[0, start]
        back = np.full((count, candidate_count), -1, dtype=np.int16)
        for row in range(1, count):
            current = np.full(candidate_count, math.inf, dtype=float)
            for target in range(candidate_count):
                if not math.isfinite(float(costs[row, target])):
                    continue
                changes = np.abs(offsets[target] - offsets)
                transition = 0.20 * (changes / transition_scale) ** 2
                transition[changes > maximum_lateral_change_m] = math.inf
                source = int(np.argmin(previous + transition))
                value = previous[source] + transition[source] + costs[row, target]
                if math.isfinite(float(value)):
                    current[target] = value
                    back[row, target] = source
            previous = current
        closure_changes = np.abs(offsets - offsets[start])
        closure = 0.20 * (closure_changes / transition_scale) ** 2
        closure[closure_changes > maximum_lateral_change_m] = math.inf
        end = int(np.argmin(previous + closure))
        total = float(previous[end] + closure[end])
        if total >= best_total or not math.isfinite(total):
            continue
        indices = np.empty(count, dtype=np.int16)
        indices[-1] = end
        for row in range(count - 1, 0, -1):
            indices[row - 1] = back[row, indices[row]]
        if indices[0] != start or np.any(indices < 0):
            continue
        best_total = total
        best_indices = indices
    if best_indices is None:
        raise RuntimeError("no closed wheel-safe lateral-offset sequence exists")
    return best_indices


def generate_safe_path(
    map_file: Path,
    metadata_file: Path,
    track_metadata_file: Path,
    output_file: Path,
    sample_spacing_m: float = DEFAULT_SAMPLE_SPACING_M,
    lateral_step_m: float = DEFAULT_LATERAL_STEP_M,
    edge_margin_m: float = DEFAULT_EDGE_MARGIN_M,
    maximum_speed_mps: float = 0.12,
    minimum_speed_mps: float = 0.05,
) -> dict[str, Any]:
    if sample_spacing_m <= 0.0 or lateral_step_m <= 0.0:
        raise ValueError("path sample spacing and lateral step must be positive")
    if not 0.0 < minimum_speed_mps <= maximum_speed_mps:
        raise ValueError("speed range is invalid")
    grid = GroundTruthGrid(map_file, metadata_file)
    track_metadata = load_json(track_metadata_file)
    config = load_json(package_root() / "config" / "pk3a_closed_loop.json")
    modules = _module_catalog(config)
    instances = track_metadata["instances"]
    required_edge = TIRE_HALF_WIDTH_M + edge_margin_m

    minimum_width = min(float(modules[item["module"]].width) for item in instances)
    maximum_offset = (
        0.5 * minimum_width
        - 0.5 * WHEEL_TRACK_M
        - TIRE_HALF_WIDTH_M
        - edge_margin_m
    )
    if maximum_offset <= 0.0:
        raise RuntimeError("vehicle wheels and safety margin do not fit the track")
    lateral_count = int(math.floor(maximum_offset / lateral_step_m))
    offsets = np.arange(-lateral_count, lateral_count + 1, dtype=float) * lateral_step_m

    samples: list[tuple[dict[str, Any], Any, float]] = []
    for instance in instances:
        spec = modules[instance["module"]]
        count = max(2, int(math.ceil(spec.length / sample_spacing_m)))
        reverse = instance.get("input_port") == "outlet" or (
            instance.get("input_port") is None and instance.get("output_port") == "inlet"
        )
        for route_progress in np.linspace(0.0, spec.length, count, endpoint=False):
            local_progress = spec.length - route_progress if reverse else route_progress
            samples.append((instance, spec, float(local_progress)))

    costs = np.full((len(samples), offsets.size), math.inf, dtype=float)
    candidates: list[list[dict[str, Any] | None]] = []
    # Keep substantial tracking reserve beyond the physical tire+margin
    # requirement. The low-slope side is preferred only while this reserve is
    # preserved, so a few centimetres of controller error cannot drop a tire.
    preferred_edge = required_edge + 0.11
    for row, (instance, spec, progress) in enumerate(samples):
        row_candidates: list[dict[str, Any] | None] = []
        for column, lateral in enumerate(offsets):
            x, y, yaw = _world_pose(instance, spec, progress, float(lateral))
            metrics = _candidate_metrics(grid, x, y, yaw, required_edge)
            if metrics is None:
                row_candidates.append(None)
                continue
            candidate = {
                "x": x,
                "y": y,
                "yaw": yaw,
                "lateral_offset_m": float(lateral),
                **metrics,
            }
            costs[row, column] = _candidate_cost(metrics, preferred_edge)
            row_candidates.append(candidate)
        if not np.isfinite(costs[row]).any():
            raise RuntimeError(
                f"no wheel-safe pose at module {instance['id']} progress {progress:.3f} m"
            )
        candidates.append(row_candidates)

    maximum_change = max(lateral_step_m, 0.5 * sample_spacing_m)
    selected_indices = _choose_closed_offsets(costs, offsets, maximum_change)
    raw_offsets = offsets[selected_indices]
    smoothing_window = 11 if len(raw_offsets) >= 11 else max(3, len(raw_offsets) // 2 * 2 + 1)
    radius = smoothing_window // 2
    wrapped_offsets = np.concatenate((raw_offsets[-radius:], raw_offsets, raw_offsets[:radius]))
    smoothed_offsets = np.convolve(
        wrapped_offsets,
        np.ones(smoothing_window, dtype=float) / smoothing_window,
        mode="valid",
    )
    points = []
    for row, ((instance, spec, progress), smoothed, raw) in enumerate(
        zip(samples, smoothed_offsets, raw_offsets)
    ):
        selected = None
        # Smoothing normally increases clearance. If a rare seam raster cell
        # rejects it, blend back toward the DP-validated offset locally.
        for blend in (1.0, 0.75, 0.50, 0.25, 0.0):
            lateral = float(blend * smoothed + (1.0 - blend) * raw)
            x, y, yaw = _world_pose(instance, spec, progress, lateral)
            metrics = _candidate_metrics(grid, x, y, yaw, required_edge)
            if metrics is not None:
                selected = {
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "lateral_offset_m": lateral,
                    **metrics,
                }
                break
        if selected is None:
            raise RuntimeError(f"smoothed wheel-safe path failed at sample {row}")
        points.append(selected)

    count = len(points)
    for index, point in enumerate(points):
        before = points[(index - 1) % count]
        after = points[(index + 1) % count]
        point["yaw"] = math.atan2(after["y"] - before["y"], after["x"] - before["x"])

    # Validate against the actual path tangent after lateral smoothing, not
    # only the individual module's analytic tangent.
    for point in points:
        metrics = _candidate_metrics(
            grid, point["x"], point["y"], point["yaw"], required_edge
        )
        if metrics is None:
            raise RuntimeError("final smoothed tangent violates wheel-edge clearance")
        point.update(metrics)

    maximum_curvature = 0.0
    for index, point in enumerate(points):
        before = points[(index - 1) % count]
        distance = max(math.hypot(point["x"] - before["x"], point["y"] - before["y"]), 1.0e-3)
        curvature = abs(_wrap_angle(point["yaw"] - before["yaw"])) / distance
        maximum_curvature = max(maximum_curvature, curvature)
        curve_speed = math.sqrt(0.08 / max(curvature, 1.0e-6))
        slope_ratio = min(1.0, point["maximum_wheel_slope_deg"] / 65.0)
        slope_speed = maximum_speed_mps - 0.07 * slope_ratio
        cross_ratio = min(1.0, abs(point["cross_slope_deg"]) / 30.0)
        cross_speed = maximum_speed_mps - 0.08 * cross_ratio
        edge_ratio = min(1.0, max(0.0, (point["minimum_wheel_edge_m"] - required_edge) / 0.10))
        edge_speed = minimum_speed_mps + (maximum_speed_mps - minimum_speed_mps) * edge_ratio
        point["speed_mps"] = float(max(
            minimum_speed_mps,
            min(maximum_speed_mps, curve_speed, slope_speed, cross_speed, edge_speed),
        ))
        point["module_index"] = int(samples[index][0]["index"])
        point["module_id"] = samples[index][0]["id"]

    document = {
        "schema_version": 2,
        "frame_id": "map",
        "closed_loop": True,
        "path_role": "mujoco_low_slope_wheel_safe",
        "track_width_m": minimum_width,
        "vehicle": {
            "wheelbase_m": WHEELBASE_M,
            "wheel_track_m": WHEEL_TRACK_M,
            "tire_half_width_m": TIRE_HALF_WIDTH_M,
            "edge_margin_m": edge_margin_m,
            "required_wheel_edge_m": required_edge,
        },
        "default_speed_mps": maximum_speed_mps,
        "points": points,
    }
    output_file = output_file.expanduser().resolve()
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    offsets_selected = np.asarray([point["lateral_offset_m"] for point in points])
    result = {
        "output_file": str(output_file),
        "point_count": count,
        "candidate_offset_range_m": [float(offsets[0]), float(offsets[-1])],
        "selected_offset_range_m": [float(offsets_selected.min()), float(offsets_selected.max())],
        "noncentral_point_count": int(np.count_nonzero(np.abs(offsets_selected) > 1.0e-6)),
        "minimum_wheel_edge_m": float(min(point["minimum_wheel_edge_m"] for point in points)),
        "maximum_wheel_slope_deg": float(max(point["maximum_wheel_slope_deg"] for point in points)),
        "maximum_curvature_rad_m": maximum_curvature,
        "speed_range_mps": [
            float(min(point["speed_mps"] for point in points)),
            float(max(point["speed_mps"] for point in points)),
        ],
    }
    return result


def main() -> None:
    workspace = workspace_root()
    map_root = workspace / "maps" / "sim_pk3a_ground_truth"
    track_root = package_root() / "description" / "pk3a"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-file", type=Path, default=map_root / "pk3a_ground_truth.npz")
    parser.add_argument("--metadata-file", type=Path, default=map_root / "pk3a_ground_truth.json")
    parser.add_argument(
        "--track-metadata-file",
        type=Path,
        default=track_root / "assets" / "pk3a_closed_loop_metadata.json",
    )
    parser.add_argument("--output-file", type=Path, default=map_root / "pk3a_safe_path.yaml")
    parser.add_argument("--sample-spacing", type=float, default=DEFAULT_SAMPLE_SPACING_M)
    parser.add_argument("--lateral-step", type=float, default=DEFAULT_LATERAL_STEP_M)
    parser.add_argument("--edge-margin", type=float, default=DEFAULT_EDGE_MARGIN_M)
    arguments = parser.parse_args()
    result = generate_safe_path(
        arguments.map_file,
        arguments.metadata_file,
        arguments.track_metadata_file,
        arguments.output_file,
        sample_spacing_m=arguments.sample_spacing,
        lateral_step_m=arguments.lateral_step,
        edge_margin_m=arguments.edge_margin,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
