"""Analytic slope and tangent-continuity checks for PK-3A surfaces."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List

import numpy as np

from .module_geometry import ModuleSpec, surface_height


def surface_slope_metrics(
    spec: ModuleSpec,
    sample_count: int = 401,
) -> Dict[str, float]:
    """Return longitudinal, transverse and resultant surface angles."""
    if sample_count < 5:
        raise ValueError("sample_count must be at least five")
    progress = np.linspace(0.0, spec.length, sample_count)
    lateral = np.linspace(-0.5 * spec.width, 0.5 * spec.width, sample_count)
    xx, yy = np.meshgrid(progress, lateral)
    height = surface_height(spec, xx, yy)
    transverse_gradient, longitudinal_gradient = np.gradient(
        height,
        lateral,
        progress,
        edge_order=2,
    )
    longitudinal = np.rad2deg(np.arctan(np.abs(longitudinal_gradient)))
    transverse = np.rad2deg(np.arctan(np.abs(transverse_gradient)))
    resultant = np.rad2deg(np.arctan(np.hypot(
        longitudinal_gradient,
        transverse_gradient,
    )))
    interior = np.s_[1:-1, 1:-1]
    return {
        "maximum_longitudinal_slope_deg": float(longitudinal[interior].max()),
        "p95_longitudinal_slope_deg": float(np.percentile(
            longitudinal[interior], 95.0
        )),
        "maximum_transverse_slope_deg": float(transverse[interior].max()),
        "p95_transverse_slope_deg": float(np.percentile(
            transverse[interior], 95.0
        )),
        "maximum_resultant_slope_deg": float(resultant[interior].max()),
        "p95_resultant_slope_deg": float(np.percentile(
            resultant[interior], 95.0
        )),
    }


def _port_course_angles(
    spec: ModuleSpec,
    port_name: str,
    sample_count: int,
) -> np.ndarray:
    """Signed dz/dcourse angles in the module's positive-X direction."""
    lateral = np.linspace(-0.5 * spec.width, 0.5 * spec.width, sample_count)
    step = max(1.0e-7, 1.0e-5 * spec.length)
    if port_name == "inlet":
        boundary = surface_height(spec, np.zeros(sample_count), lateral)
        interior = surface_height(
            spec,
            np.full(sample_count, step),
            lateral,
        )
        gradient = (interior - boundary) / step
    elif port_name == "outlet":
        boundary = surface_height(
            spec,
            np.full(sample_count, spec.length),
            lateral,
        )
        interior = surface_height(
            spec,
            np.full(sample_count, spec.length - step),
            lateral,
        )
        gradient = (boundary - interior) / step
    else:
        raise ValueError(
            f"Course tangent validation requires inlet/outlet ports, got {port_name}"
        )
    return np.rad2deg(np.arctan(gradient))


def annotate_seam_tangent_metrics(
    instances: Iterable[Dict[str, Any]],
    seams: List[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
) -> float:
    """Add C1 course-angle errors to assembled seam records."""
    instance_by_id = {item["id"]: item for item in instances}
    maximum = 0.0
    for seam in seams:
        previous = instance_by_id[seam["from_instance"]]
        following = instance_by_id[seam["to_instance"]]
        previous_port = str(seam["target_port"]).rsplit(":", 1)[-1]
        following_port = str(seam["source_port"]).rsplit(":", 1)[-1]
        sample_count = len(seam["points_xyz_m"])

        previous_angles = _port_course_angles(
            modules[previous["module"]], previous_port, sample_count
        )
        following_angles = _port_course_angles(
            modules[following["module"]], following_port, sample_count
        )
        if previous_port == "inlet":
            previous_angles = -previous_angles
        if following_port == "outlet":
            following_angles = -following_angles
        if seam["source_order"] == "reversed":
            following_angles = following_angles[::-1]

        error = np.abs(previous_angles - following_angles)
        seam["maximum_course_tangent_error_deg"] = float(error.max())
        seam["rms_course_tangent_error_deg"] = float(np.sqrt(np.mean(error**2)))
        seam["p95_course_tangent_error_deg"] = float(np.percentile(error, 95.0))
        maximum = max(maximum, seam["maximum_course_tangent_error_deg"])
    return maximum
