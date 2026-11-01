"""Sample and compare the physical boundary ports of PK-3A modules."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from .module_geometry import (
    ModuleSpec,
    load_json,
    load_modules,
    package_root,
    surface_height,
    surface_point,
)


PORT_NAMES = ("inlet", "outlet", "right_edge", "left_edge")


def default_port_sources_path() -> Path:
    return package_root() / "config" / "pk3a_port_sources.json"


def load_port_sources(
    path: Path | str | None = None,
) -> Tuple[Dict[str, Any], Dict[str, ModuleSpec]]:
    """Load one approved source config per module."""
    registry_path = Path(path or default_port_sources_path()).resolve()
    document = load_json(registry_path)
    root = package_root()
    modules: Dict[str, ModuleSpec] = {}
    for name, source in document["modules"].items():
        config_path = Path(source["config"])
        if not config_path.is_absolute():
            config_path = root / config_path
        available = load_modules(config_path)
        if name not in available:
            raise KeyError(f"{name} is missing from {config_path}")
        modules[name] = available[name]
    return document, modules


def _port_coordinates(
    spec: ModuleSpec,
    port_name: str,
    sample_count: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if port_name == "inlet":
        progress = np.zeros(sample_count)
        lateral = np.linspace(-0.5 * spec.width, 0.5 * spec.width, sample_count)
    elif port_name == "outlet":
        progress = np.full(sample_count, spec.length)
        lateral = np.linspace(-0.5 * spec.width, 0.5 * spec.width, sample_count)
    elif port_name == "right_edge":
        progress = np.linspace(0.0, spec.length, sample_count)
        lateral = np.full(sample_count, -0.5 * spec.width)
    elif port_name == "left_edge":
        progress = np.linspace(0.0, spec.length, sample_count)
        lateral = np.full(sample_count, 0.5 * spec.width)
    else:
        raise ValueError(f"Unsupported port: {port_name}")
    return progress, lateral


def _polyline_distances(points: np.ndarray) -> Tuple[np.ndarray, float]:
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(steps)))
    total = float(distance[-1])
    if total <= 0.0:
        raise ValueError("Port polyline has zero length")
    return distance / total, total


def _inward_direction(
    spec: ModuleSpec,
    port_name: str,
    sample_count: int,
) -> np.ndarray:
    """Return the local XY direction from a boundary into the riding surface."""
    middle = sample_count // 2
    epsilon = min(spec.length, spec.width) * 1.0e-4
    progress, lateral = _port_coordinates(spec, port_name, sample_count)
    p = float(progress[middle])
    y = float(lateral[middle])
    boundary = surface_point(spec, p, y)
    if port_name == "inlet":
        interior = surface_point(spec, min(spec.length, p + epsilon), y)
    elif port_name == "outlet":
        interior = surface_point(spec, max(0.0, p - epsilon), y)
    elif port_name == "right_edge":
        interior = surface_point(spec, p, min(0.5 * spec.width, y + epsilon))
    else:
        interior = surface_point(spec, p, max(-0.5 * spec.width, y - epsilon))
    direction = np.asarray(interior) - np.asarray(boundary)
    return direction / np.linalg.norm(direction)


def sample_port(
    spec: ModuleSpec,
    port_name: str,
    sample_count: int,
    alias: str | None = None,
) -> Dict[str, Any]:
    """Create a metric port record in the module's local coordinates."""
    progress, lateral = _port_coordinates(spec, port_name, sample_count)
    xy = np.asarray([
        surface_point(spec, float(p), float(y))
        for p, y in zip(progress, lateral)
    ])
    height = surface_height(spec, progress, lateral)
    points = np.column_stack((xy, height))
    normalized_distance, arc_length = _polyline_distances(xy)
    chord = float(np.linalg.norm(xy[-1] - xy[0]))
    straight_line = (
        xy[0]
        + normalized_distance[:, None] * (xy[-1] - xy[0])
    )
    planarity_error = np.linalg.norm(xy - straight_line, axis=1)
    return {
        "id": f"{spec.name}:{port_name}",
        "module": spec.name,
        "name": port_name,
        "alias": alias or port_name,
        "sample_order": (
            "right_to_left"
            if port_name in ("inlet", "outlet")
            else "inlet_to_outlet"
        ),
        "sample_count": sample_count,
        "normalized_distance": np.round(normalized_distance, 9).tolist(),
        "points_xyz_m": np.round(points, 9).tolist(),
        "height_profile_m": np.round(height, 9).tolist(),
        "arc_length_m": round(arc_length, 9),
        "chord_length_m": round(chord, 9),
        "maximum_planarity_error_m": round(float(planarity_error.max()), 9),
        "inward_direction_xy": np.round(
            _inward_direction(spec, port_name, sample_count), 9
        ).tolist(),
        "height_min_m": round(float(height.min()), 9),
        "height_max_m": round(float(height.max()), 9),
        "height_range_m": round(float(np.ptp(height)), 9),
        "constant_height": bool(np.ptp(height) <= 1.0e-8),
    }


def sample_all_ports(
    registry: Dict[str, Any],
    modules: Dict[str, ModuleSpec],
) -> Dict[str, List[Dict[str, Any]]]:
    sample_count = int(registry["sample_count"])
    result: Dict[str, List[Dict[str, Any]]] = {}
    for name, spec in modules.items():
        aliases = registry["modules"][name].get("aliases", {})
        result[name] = [
            sample_port(spec, port_name, sample_count, aliases.get(port_name))
            for port_name in PORT_NAMES
        ]
    return result


def _resample_port(port: Dict[str, Any], sample_count: int) -> Tuple[np.ndarray, np.ndarray]:
    source_distance = np.asarray(port["normalized_distance"], dtype=float)
    source_points = np.asarray(port["points_xyz_m"], dtype=float)
    target_distance = np.linspace(0.0, 1.0, sample_count)
    xy = np.column_stack([
        np.interp(target_distance, source_distance, source_points[:, axis])
        for axis in (0, 1)
    ])
    height = np.interp(target_distance, source_distance, source_points[:, 2])
    return xy, height


def _rigid_plan_error(reference: np.ndarray, candidate: np.ndarray) -> Tuple[float, float]:
    reference_centre = reference.mean(axis=0)
    candidate_centre = candidate.mean(axis=0)
    a = reference - reference_centre
    b = candidate - candidate_centre
    u, _, vt = np.linalg.svd(b.T @ a)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    fitted = b @ rotation + reference_centre
    error = np.linalg.norm(reference - fitted, axis=1)
    return float(np.sqrt(np.mean(error**2))), float(error.max())


def compare_ports(
    port_a: Dict[str, Any],
    port_b: Dict[str, Any],
    tolerances: Dict[str, float],
    sample_count: int = 101,
) -> Dict[str, Any]:
    """Compare both point orders and retain the physically best seam."""
    xy_a, height_a = _resample_port(port_a, sample_count)
    xy_b, height_b = _resample_port(port_b, sample_count)
    span_error = abs(float(port_a["arc_length_m"]) - float(port_b["arc_length_m"]))
    choices = []
    for reversed_order in (False, True):
        candidate_xy = xy_b[::-1] if reversed_order else xy_b
        candidate_height = height_b[::-1] if reversed_order else height_b
        plan_rms, plan_max = _rigid_plan_error(xy_a, candidate_xy)
        height_error = np.abs(height_a - candidate_height)
        height_max = float(height_error.max())
        height_rms = float(np.sqrt(np.mean(height_error**2)))
        score = max(
            height_max / float(tolerances["maximum_height_error_m"]),
            plan_max / float(tolerances["maximum_plan_error_m"]),
            span_error / float(tolerances["maximum_span_error_m"]),
        )
        choices.append({
            "reversed": reversed_order,
            "height_max_m": height_max,
            "height_rms_m": height_rms,
            "plan_max_m": plan_max,
            "plan_rms_m": plan_rms,
            "score": score,
        })
    best = min(choices, key=lambda item: item["score"])
    allowed = (
        best["height_max_m"] < float(tolerances["maximum_height_error_m"])
        and best["plan_max_m"] <= float(tolerances["maximum_plan_error_m"])
        and span_error <= float(tolerances["maximum_span_error_m"])
    )
    return {
        "port_a": port_a["id"],
        "port_b": port_b["id"],
        "port_a_alias": port_a["alias"],
        "port_b_alias": port_b["alias"],
        "allowed": bool(allowed),
        "orientation": "reversed" if best["reversed"] else "same_order",
        "maximum_height_error_m": round(best["height_max_m"], 9),
        "rms_height_error_m": round(best["height_rms_m"], 9),
        "maximum_plan_error_m": round(best["plan_max_m"], 9),
        "rms_plan_error_m": round(best["plan_rms_m"], 9),
        "span_error_m": round(span_error, 9),
        "normalized_score": round(float(best["score"]), 6),
    }


def compatibility_catalog(
    registry: Dict[str, Any],
    ports: Dict[str, List[Dict[str, Any]]],
) -> Dict[str, Any]:
    tolerances = registry["tolerances"]
    module_names = list(ports)
    module_pairs = []
    detailed_pairs = []
    for module_a_index, module_a in enumerate(module_names):
        for module_b in module_names[module_a_index:]:
            comparisons = [
                compare_ports(port_a, port_b, tolerances)
                for port_a in ports[module_a]
                for port_b in ports[module_b]
            ]
            comparisons.sort(key=lambda item: item["normalized_score"])
            detailed_pairs.extend(comparisons)
            best = comparisons[0]
            module_pairs.append({
                "module_a": module_a,
                "module_b": module_b,
                "allowed_pair_count": sum(item["allowed"] for item in comparisons),
                "best": best,
            })
    required = []
    for module_a, module_b in registry["required_module_pairs"]:
        record = next(
            item for item in module_pairs
            if {item["module_a"], item["module_b"]} == {module_a, module_b}
        )
        required.append({
            "module_a": module_a,
            "module_b": module_b,
            "passed": record["allowed_pair_count"] > 0,
            "allowed_pair_count": record["allowed_pair_count"],
            "best": record["best"],
        })
    return {
        "schema_version": 1,
        "tolerances": tolerances,
        "module_pairs": module_pairs,
        "required_pair_audits": required,
        "all_required_pairs_passed": all(item["passed"] for item in required),
        "detailed_pairs": detailed_pairs,
    }


def source_hashes(registry: Dict[str, Any]) -> Dict[str, str]:
    result = {}
    root = package_root()
    for name, source in registry["modules"].items():
        path = Path(source["config"])
        if not path.is_absolute():
            path = root / path
        result[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def iter_ports(ports: Dict[str, List[Dict[str, Any]]]) -> Iterable[Dict[str, Any]]:
    for module_ports in ports.values():
        yield from module_ports
