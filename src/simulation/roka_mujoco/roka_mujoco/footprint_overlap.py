"""Plan-view footprint overlap checks for rigid PK-3A module assemblies."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List

import numpy as np

from .module_geometry import ModuleSpec, surface_point
from .port_assembly import transform_xy


def _signed_area(polygon: np.ndarray) -> float:
    return 0.5 * float(
        np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1))
        - np.dot(polygon[:, 1], np.roll(polygon[:, 0], -1))
    )


def polygon_area(polygon: np.ndarray) -> float:
    if len(polygon) < 3:
        return 0.0
    return abs(_signed_area(np.asarray(polygon, dtype=float)))


def _cross_2d(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _line_intersection(
    segment_start: np.ndarray,
    segment_end: np.ndarray,
    clip_start: np.ndarray,
    clip_end: np.ndarray,
) -> np.ndarray:
    segment_direction = segment_end - segment_start
    clip_direction = clip_end - clip_start
    denominator = _cross_2d(segment_direction, clip_direction)
    if abs(denominator) <= 1.0e-14:
        return 0.5 * (segment_start + segment_end)
    parameter = _cross_2d(
        clip_start - segment_start, clip_direction
    ) / denominator
    return segment_start + parameter * segment_direction


def intersect_convex_polygons(
    subject_polygon: np.ndarray,
    clip_polygon: np.ndarray,
) -> np.ndarray:
    """Sutherland-Hodgman intersection for the small convex strip cells."""
    output = np.asarray(subject_polygon, dtype=float)
    clip = np.asarray(clip_polygon, dtype=float)
    if _signed_area(clip) < 0.0:
        clip = clip[::-1]
    tolerance = 1.0e-12
    for edge_index in range(len(clip)):
        clip_start = clip[edge_index]
        clip_end = clip[(edge_index + 1) % len(clip)]
        input_polygon = output
        if len(input_polygon) == 0:
            break
        clipped: List[np.ndarray] = []
        previous = input_polygon[-1]
        previous_inside = (
            _cross_2d(clip_end - clip_start, previous - clip_start)
            >= -tolerance
        )
        for current in input_polygon:
            current_inside = (
                _cross_2d(clip_end - clip_start, current - clip_start)
                >= -tolerance
            )
            if current_inside:
                if not previous_inside:
                    clipped.append(
                        _line_intersection(
                            previous, current, clip_start, clip_end
                        )
                    )
                clipped.append(current)
            elif previous_inside:
                clipped.append(
                    _line_intersection(
                        previous, current, clip_start, clip_end
                    )
                )
            previous = current
            previous_inside = current_inside
        output = np.asarray(clipped, dtype=float)
    return output


def footprint_strips(
    instance: Dict[str, Any],
    spec: ModuleSpec,
    segment_count: int = 32,
) -> List[np.ndarray]:
    """Approximate a straight or curved footprint as non-overlapping quads."""
    progress = np.linspace(0.0, spec.length, segment_count + 1)
    result = []
    for start, end in zip(progress[:-1], progress[1:]):
        local = np.asarray([
            surface_point(spec, float(start), -0.5 * spec.width),
            surface_point(spec, float(end), -0.5 * spec.width),
            surface_point(spec, float(end), 0.5 * spec.width),
            surface_point(spec, float(start), 0.5 * spec.width),
        ])
        result.append(transform_xy(local, instance["pose"]))
    return result


def _bounds(polygon: np.ndarray) -> np.ndarray:
    return np.array([
        polygon[:, 0].min(),
        polygon[:, 0].max(),
        polygon[:, 1].min(),
        polygon[:, 1].max(),
    ])


def _bounds_have_positive_overlap(
    first: np.ndarray,
    second: np.ndarray,
) -> bool:
    tolerance = 1.0e-10
    return not (
        first[1] <= second[0] + tolerance
        or second[1] <= first[0] + tolerance
        or first[3] <= second[2] + tolerance
        or second[3] <= first[2] + tolerance
    )


def pair_overlap_area(
    first_strips: Iterable[np.ndarray],
    second_strips: Iterable[np.ndarray],
) -> float:
    second_records = [(_bounds(strip), strip) for strip in second_strips]
    area = 0.0
    for first in first_strips:
        first_bounds = _bounds(first)
        for second_bounds, second in second_records:
            if not _bounds_have_positive_overlap(first_bounds, second_bounds):
                continue
            intersection = intersect_convex_polygons(first, second)
            area += polygon_area(intersection)
    return area


def find_nonadjacent_overlaps(
    instances: List[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
    segment_count: int = 32,
    minimum_report_area_m2: float = 1.0e-8,
) -> List[Dict[str, Any]]:
    """Return positive-area intersections, excluding consecutive seam pairs."""
    strips = [
        footprint_strips(item, modules[item["module"]], segment_count)
        for item in instances
    ]
    records = []
    for first_index in range(len(instances)):
        for second_index in range(first_index + 2, len(instances)):
            area = pair_overlap_area(strips[first_index], strips[second_index])
            if area <= minimum_report_area_m2:
                continue
            records.append({
                "first_index": first_index,
                "second_index": second_index,
                "first_instance": instances[first_index]["id"],
                "second_instance": instances[second_index]["id"],
                "overlap_area_m2": float(area),
            })
    records.sort(key=lambda item: item["overlap_area_m2"], reverse=True)
    return records
