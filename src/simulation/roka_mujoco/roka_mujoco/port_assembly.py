"""Rigidly place module surface patches by matching sampled boundary ports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from .module_geometry import ModuleSpec
from .port_geometry import sample_port


@dataclass(frozen=True)
class PlanPose:
    xy: np.ndarray
    yaw: float


def rotation_row(yaw: float) -> np.ndarray:
    """Matrix for transforming row-vector XY points by a conventional yaw."""
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    return np.array([[cosine, sine], [-sine, cosine]])


def transform_xy(points: np.ndarray, pose: PlanPose) -> np.ndarray:
    return np.asarray(points, dtype=float) @ rotation_row(pose.yaw) + pose.xy


def transform_direction(direction: np.ndarray, pose: PlanPose) -> np.ndarray:
    return np.asarray(direction, dtype=float) @ rotation_row(pose.yaw)


def transform_xyz(points: np.ndarray, pose: PlanPose) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return np.column_stack((transform_xy(points[:, :2], pose), points[:, 2]))


def _rigid_row_transform(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    source_centre = source.mean(axis=0)
    target_centre = target.mean(axis=0)
    source_zero = source - source_centre
    target_zero = target - target_centre
    u, _, vt = np.linalg.svd(source_zero.T @ target_zero)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    translation = target_centre - source_centre @ rotation
    return rotation, translation


def _yaw_from_row_rotation(rotation: np.ndarray) -> float:
    return float(np.arctan2(rotation[0, 1], rotation[0, 0]))


def place_at_port(
    target_port: Dict[str, Any],
    target_pose: PlanPose,
    source_port: Dict[str, Any],
    maximum_height_error: float,
) -> Tuple[PlanPose, Dict[str, Any]]:
    """Place a source module outside a target boundary without changing Z."""
    target_local = np.asarray(target_port["points_xyz_m"], dtype=float)
    source_local = np.asarray(source_port["points_xyz_m"], dtype=float)
    target_world_xy = transform_xy(target_local[:, :2], target_pose)
    target_height = target_local[:, 2]
    target_inward = transform_direction(target_port["inward_direction_xy"], target_pose)
    candidates = []
    for reverse in (False, True):
        ordered = source_local[::-1] if reverse else source_local
        rotation, translation = _rigid_row_transform(ordered[:, :2], target_world_xy)
        fitted_xy = ordered[:, :2] @ rotation + translation
        plan_error = np.linalg.norm(fitted_xy - target_world_xy, axis=1)
        height_error = np.abs(ordered[:, 2] - target_height)
        source_inward = np.asarray(source_port["inward_direction_xy"], dtype=float) @ rotation
        inward_dot = float(np.dot(target_inward, source_inward))
        score = (
            float(height_error.max()) / maximum_height_error
            + float(plan_error.max()) / 0.005
            + 0.5 * (inward_dot + 1.0)
        )
        candidates.append({
            "reverse": reverse,
            "rotation": rotation,
            "translation": translation,
            "height_error": height_error,
            "plan_error": plan_error,
            "inward_dot": inward_dot,
            "source_inward": source_inward,
            "score": score,
        })
    outward = [item for item in candidates if item["inward_dot"] < -0.95]
    if not outward:
        raise ValueError(
            f'No outward placement for {target_port["id"]} and {source_port["id"]}'
        )
    best = min(outward, key=lambda item: item["score"])
    maximum_error = float(best["height_error"].max())
    if maximum_error >= maximum_height_error:
        raise ValueError(
            f'Height mismatch {maximum_error:.6f} m for '
            f'{target_port["id"]} and {source_port["id"]}'
        )
    pose = PlanPose(
        xy=np.asarray(best["translation"], dtype=float),
        yaw=_yaw_from_row_rotation(best["rotation"]),
    )
    return pose, {
        "target_port": target_port["id"],
        "source_port": source_port["id"],
        "source_order": "reversed" if best["reverse"] else "same_order",
        "maximum_height_error_m": maximum_error,
        "rms_height_error_m": float(np.sqrt(np.mean(best["height_error"] ** 2))),
        "maximum_plan_error_m": float(best["plan_error"].max()),
        "rms_plan_error_m": float(np.sqrt(np.mean(best["plan_error"] ** 2))),
        "inward_direction_dot": best["inward_dot"],
        "target_inward_direction_xy": target_inward.tolist(),
        "source_inward_direction_xy": best["source_inward"].tolist(),
    }


def expand_route(route: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    expanded = []
    for item in route:
        copies = int(item.get("copies", 1))
        for copy_index in range(copies):
            record = dict(item)
            record.pop("copies", None)
            if copies > 1:
                prefix = str(item["id"]).rstrip("0123456789")
                record["id"] = f"{prefix}{copy_index + 1:02d}"
            expanded.append(record)
    return expanded


def assemble_route(
    route: Iterable[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
    aliases: Dict[str, Dict[str, str]],
    sample_count: int,
    maximum_height_error: float,
    start_pose: PlanPose,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    expanded = expand_route(route)
    port_cache = {
        name: {
            port_name: sample_port(
                spec,
                port_name,
                sample_count,
                aliases.get(name, {}).get(port_name),
            )
            for port_name in ("inlet", "outlet", "right_edge", "left_edge")
        }
        for name, spec in modules.items()
    }
    instances = []
    seams = []
    for index, item in enumerate(expanded):
        if index == 0:
            pose = start_pose
        else:
            previous = instances[-1]
            target_port_name = previous["output_port"]
            source_port_name = item["input_port"]
            pose, seam = place_at_port(
                port_cache[previous["module"]][target_port_name],
                previous["pose"],
                port_cache[item["module"]][source_port_name],
                maximum_height_error,
            )
            seam.update({
                "index": index - 1,
                "from_instance": previous["id"],
                "to_instance": item["id"],
            })
            target_points = transform_xyz(
                np.asarray(
                    port_cache[previous["module"]][target_port_name]["points_xyz_m"]
                ),
                previous["pose"],
            )
            seam["points_xyz_m"] = target_points.tolist()
            seams.append(seam)
        instances.append({
            "index": index,
            "id": item["id"],
            "module": item["module"],
            "input_port": item.get("input_port"),
            "output_port": item["output_port"],
            "pose": pose,
        })
    return instances, seams
