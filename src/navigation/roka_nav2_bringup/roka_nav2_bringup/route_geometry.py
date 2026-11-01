"""Deterministic geometry for continuous route/corridor autonomy.

The module deliberately has no ROS dependencies.  Mapping tools, the runtime
follower and unit tests therefore use exactly the same smoothing, projection,
join and completion rules.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence


EPSILON = 1.0e-9


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _xy(point: dict[str, Any]) -> tuple[float, float]:
    x, y = float(point["x"]), float(point["y"])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("route contains a non-finite position")
    return x, y


def _distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return math.hypot(bx - ax, by - ay)


def remove_duplicate_points(
    points: Sequence[dict[str, Any]], *, closed: bool, minimum_spacing_m: float = 0.005
) -> list[dict[str, Any]]:
    if minimum_spacing_m <= 0.0:
        raise ValueError("minimum spacing must be positive")
    cleaned: list[dict[str, Any]] = []
    for source in points:
        point = dict(source)
        _xy(point)
        if not cleaned or _distance(cleaned[-1], point) >= minimum_spacing_m:
            cleaned.append(point)
    if closed and len(cleaned) > 2 and _distance(cleaned[0], cleaned[-1]) < minimum_spacing_m:
        cleaned.pop()
    if len(cleaned) < (3 if closed else 2):
        raise ValueError("route has too few distinct points")
    return cleaned


def cumulative_lengths(
    points: Sequence[dict[str, Any]], *, closed: bool = False
) -> tuple[list[float], float]:
    if len(points) < 2:
        raise ValueError("route length needs at least two points")
    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(cumulative[-1] + _distance(points[index - 1], points[index]))
    total = cumulative[-1]
    if closed:
        total += _distance(points[-1], points[0])
    if total <= EPSILON:
        raise ValueError("route has zero length")
    return cumulative, total


def _interpolate_metadata(
    before: dict[str, Any], after: dict[str, Any], ratio: float
) -> dict[str, Any]:
    # Preserve safety/speed metadata from the nearest demonstrated sample.
    result = dict(before if ratio < 0.5 else after)
    result["x"] = (1.0 - ratio) * float(before["x"]) + ratio * float(after["x"])
    result["y"] = (1.0 - ratio) * float(before["y"]) + ratio * float(after["y"])
    if "z" in before or "z" in after:
        result["z"] = (
            (1.0 - ratio) * float(before.get("z", after.get("z", 0.0)))
            + ratio * float(after.get("z", before.get("z", 0.0)))
        )
    return result


def resample_by_arclength(
    points: Sequence[dict[str, Any]], *, spacing_m: float = 0.05, closed: bool = False
) -> list[dict[str, Any]]:
    if spacing_m <= 0.0:
        raise ValueError("resampling spacing must be positive")
    source = remove_duplicate_points(points, closed=closed)
    cumulative, total = cumulative_lengths(source, closed=closed)
    if closed:
        count = max(3, int(math.ceil(total / spacing_m)))
        targets = [total * index / count for index in range(count)]
    else:
        count = max(2, int(math.ceil(total / spacing_m)) + 1)
        targets = [total * index / (count - 1) for index in range(count)]
    result: list[dict[str, Any]] = []
    segment = 0
    for target in targets:
        while segment < len(source) - 1 and cumulative[segment + 1] < target:
            segment += 1
        if segment == len(source) - 1:
            before = source[-1]
            after = source[0]
            low = cumulative[-1]
            high = total
        else:
            before = source[segment]
            after = source[segment + 1]
            low = cumulative[segment]
            high = cumulative[segment + 1]
        ratio = min(1.0, max(0.0, (target - low) / max(high - low, EPSILON)))
        result.append(_interpolate_metadata(before, after, ratio))
    return result


def _smooth_vertices(
    points: Sequence[dict[str, Any]], *, closed: bool, iterations: int, strength: float
) -> list[dict[str, Any]]:
    result = [dict(point) for point in points]
    for _ in range(iterations):
        updated = [dict(point) for point in result]
        first = 0 if closed else 1
        last = len(result) if closed else len(result) - 1
        for index in range(first, last):
            before = result[(index - 1) % len(result)]
            current = result[index]
            after = result[(index + 1) % len(result)]
            target_x = 0.25 * float(before["x"]) + 0.5 * float(current["x"]) + 0.25 * float(after["x"])
            target_y = 0.25 * float(before["y"]) + 0.5 * float(current["y"]) + 0.25 * float(after["y"])
            updated[index]["x"] = (1.0 - strength) * float(current["x"]) + strength * target_x
            updated[index]["y"] = (1.0 - strength) * float(current["y"]) + strength * target_y
        result = updated
    return result


def distance_to_polyline(
    x: float, y: float, points: Sequence[dict[str, Any]], *, closed: bool = False
) -> float:
    best = math.inf
    segment_count = len(points) if closed else len(points) - 1
    for index in range(segment_count):
        before = points[index]
        after = points[(index + 1) % len(points)]
        x0, y0 = _xy(before)
        x1, y1 = _xy(after)
        dx, dy = x1 - x0, y1 - y0
        scale = dx * dx + dy * dy
        ratio = 0.0 if scale <= EPSILON else ((x - x0) * dx + (y - y0) * dy) / scale
        ratio = min(1.0, max(0.0, ratio))
        best = min(best, math.hypot(x - (x0 + ratio * dx), y - (y0 + ratio * dy)))
    return best


def annotate_geometry(
    points: Sequence[dict[str, Any]], *, closed: bool
) -> list[dict[str, Any]]:
    result = [dict(point) for point in points]
    cumulative, total = cumulative_lengths(result, closed=closed)
    for index, point in enumerate(result):
        before_index = (index - 1) % len(result) if closed else max(0, index - 1)
        after_index = (index + 1) % len(result) if closed else min(len(result) - 1, index + 1)
        before = result[before_index]
        after = result[after_index]
        dx = float(after["x"]) - float(before["x"])
        dy = float(after["y"]) - float(before["y"])
        if math.hypot(dx, dy) > EPSILON:
            point["yaw"] = math.atan2(dy, dx)
        point["route_s_m"] = cumulative[index]
    for index, point in enumerate(result):
        before_index = (index - 1) % len(result) if closed else max(0, index - 1)
        after_index = (index + 1) % len(result) if closed else min(len(result) - 1, index + 1)
        span = _distance(result[before_index], point) + _distance(point, result[after_index])
        yaw_delta = wrap_angle(
            float(result[after_index].get("yaw", 0.0))
            - float(result[before_index].get("yaw", 0.0))
        )
        point["curvature_radpm"] = 0.0 if span <= EPSILON else yaw_delta / span
    return result


def prepare_centerline(
    points: Sequence[dict[str, Any]],
    *,
    closed: bool,
    spacing_m: float = 0.05,
    maximum_deviation_m: float = 0.10,
    smoothing_iterations: int = 3,
) -> tuple[list[dict[str, Any]], dict[str, float | int | str]]:
    """Smooth a manual trace while bounding deviation from its polyline."""
    raw = remove_duplicate_points(points, closed=closed)
    if maximum_deviation_m <= 0.0:
        raise ValueError("maximum centerline deviation must be positive")
    accepted: list[dict[str, Any]] | None = None
    accepted_deviation = math.inf
    strength = 0.70
    method = "bounded_laplacian_arclength"
    for _ in range(8):
        vertices = _smooth_vertices(
            raw, closed=closed, iterations=smoothing_iterations, strength=strength
        )
        candidate = resample_by_arclength(vertices, spacing_m=spacing_m, closed=closed)
        deviation = max(
            distance_to_polyline(float(point["x"]), float(point["y"]), raw, closed=closed)
            for point in candidate
        )
        if deviation <= maximum_deviation_m + 1.0e-6:
            accepted, accepted_deviation = candidate, deviation
            break
        strength *= 0.5
    if accepted is None:
        accepted = resample_by_arclength(raw, spacing_m=spacing_m, closed=closed)
        accepted_deviation = 0.0
        method = "arclength_only_deviation_fallback"
    accepted = annotate_geometry(accepted, closed=closed)
    _, total = cumulative_lengths(accepted, closed=closed)
    metadata: dict[str, float | int | str] = {
        "method": method,
        "spacing_m": float(spacing_m),
        "source_point_count": len(raw),
        "point_count": len(accepted),
        "length_m": total,
        "maximum_source_deviation_m": accepted_deviation,
        "maximum_abs_curvature_radpm": max(
            abs(float(point["curvature_radpm"])) for point in accepted
        ),
    }
    return accepted, metadata


@dataclass(frozen=True)
class RouteProjection:
    index: int
    unwrapped_index: int
    x: float
    y: float
    distance_m: float
    signed_lateral_m: float
    heading_error_rad: float
    segment_ratio: float


def project_pose(
    points: Sequence[dict[str, Any]],
    x: float,
    y: float,
    yaw: float,
    *,
    closed: bool,
    previous_unwrapped_index: int | None = None,
    backward_window: int = 5,
    forward_window: int = 80,
) -> RouteProjection:
    """Project onto a topologically bounded forward route window.

    The unwrapped index prevents a nearby parallel or repeated track segment
    from becoming the next target merely because it is Euclidean-nearest.
    """
    if len(points) < 2:
        raise ValueError("projection needs at least two route points")
    segment_count = len(points) if closed else len(points) - 1
    if previous_unwrapped_index is None:
        candidates: Iterable[int] = range(segment_count)
    else:
        low = previous_unwrapped_index - max(0, int(backward_window))
        high = previous_unwrapped_index + max(1, int(forward_window))
        candidates = range(low, high + 1)
    best: tuple[float, RouteProjection] | None = None
    for unwrapped in candidates:
        if closed:
            segment = unwrapped % segment_count
        else:
            if not 0 <= unwrapped < segment_count:
                continue
            segment = unwrapped
        before = points[segment]
        after = points[(segment + 1) % len(points)]
        x0, y0 = _xy(before)
        x1, y1 = _xy(after)
        dx, dy = x1 - x0, y1 - y0
        length2 = dx * dx + dy * dy
        if length2 <= EPSILON:
            continue
        ratio = min(1.0, max(0.0, ((x - x0) * dx + (y - y0) * dy) / length2))
        projected_x, projected_y = x0 + ratio * dx, y0 + ratio * dy
        distance = math.hypot(x - projected_x, y - projected_y)
        tangent_yaw = math.atan2(dy, dx)
        heading_error = abs(wrap_angle(tangent_yaw - yaw))
        segment_length = math.sqrt(length2)
        signed_lateral = (dx * (y - projected_y) - dy * (x - projected_x)) / segment_length
        backward_penalty = 0.0
        if previous_unwrapped_index is not None and unwrapped < previous_unwrapped_index:
            backward_penalty = 0.02 * (previous_unwrapped_index - unwrapped)
        # Distance dominates; heading breaks ambiguity at crossings/parallel lanes.
        score = distance + 0.20 * heading_error + backward_penalty
        projection = RouteProjection(
            index=segment,
            unwrapped_index=unwrapped,
            x=projected_x,
            y=projected_y,
            distance_m=distance,
            signed_lateral_m=signed_lateral,
            heading_error_rad=heading_error,
            segment_ratio=ratio,
        )
        if best is None or score < best[0]:
            best = score, projection
    if best is None:
        raise ValueError("no route segment is available in the progress window")
    return best[1]


def _bezier_point(
    controls: Sequence[tuple[float, float]], ratio: float
) -> tuple[float, float]:
    inverse = 1.0 - ratio
    weights = (inverse ** 3, 3.0 * inverse * inverse * ratio, 3.0 * inverse * ratio * ratio, ratio ** 3)
    return (
        sum(weight * point[0] for weight, point in zip(weights, controls)),
        sum(weight * point[1] for weight, point in zip(weights, controls)),
    )


def _sample_bezier(
    controls: Sequence[tuple[float, float]], spacing_m: float
) -> list[dict[str, float]]:
    control_length = sum(
        math.hypot(controls[index + 1][0] - controls[index][0], controls[index + 1][1] - controls[index][1])
        for index in range(3)
    )
    dense_count = max(20, int(math.ceil(control_length / max(spacing_m * 0.25, 0.005))))
    dense = [
        {"x": xy[0], "y": xy[1]}
        for xy in (_bezier_point(controls, index / dense_count) for index in range(dense_count + 1))
    ]
    return annotate_geometry(
        resample_by_arclength(dense, spacing_m=spacing_m, closed=False), closed=False
    )


def _target_indices_by_distance(
    points: Sequence[dict[str, Any]], start_index: int, closed: bool,
    minimum_m: float, maximum_m: float
) -> list[int]:
    indices: list[int] = []
    travelled = 0.0
    current = start_index
    maximum_steps = len(points) - 1 if closed else len(points) - start_index - 1
    for _ in range(maximum_steps):
        following = (current + 1) % len(points)
        travelled += _distance(points[current], points[following])
        current = following
        if travelled >= minimum_m:
            indices.append(current)
        if travelled >= maximum_m:
            break
    return indices


@dataclass(frozen=True)
class JoinResult:
    points: list[dict[str, Any]]
    target_index: int
    length_m: float
    maximum_abs_curvature_radpm: float


def generate_join_path(
    route: Sequence[dict[str, Any]],
    current_x: float,
    current_y: float,
    current_yaw: float,
    projection: RouteProjection,
    *,
    closed: bool,
    spacing_m: float = 0.05,
    minimum_turn_radius_m: float = 0.50,
    minimum_lookahead_m: float = 0.50,
    maximum_lookahead_m: float = 1.50,
    corridor_half_width_m: float | None = None,
) -> JoinResult:
    if minimum_turn_radius_m <= 0.0:
        raise ValueError("minimum turn radius must be positive")
    target_indices = _target_indices_by_distance(
        route, projection.index, closed, minimum_lookahead_m, maximum_lookahead_m
    )
    if not target_indices:
        target_indices = [len(route) - 1] if not closed else [projection.index]
    maximum_curvature = 1.0 / minimum_turn_radius_m
    rejected_reasons: list[str] = []
    for target_index in target_indices:
        target = route[target_index]
        target_x, target_y = _xy(target)
        target_yaw = float(target.get("yaw", 0.0))
        chord = math.hypot(target_x - current_x, target_y - current_y)
        if chord < 0.10:
            continue
        for handle_scale in (0.45, 0.65, 0.85, 1.05):
            handle = max(0.20, handle_scale * chord)
            controls = (
                (current_x, current_y),
                (current_x + handle * math.cos(current_yaw), current_y + handle * math.sin(current_yaw)),
                (target_x - handle * math.cos(target_yaw), target_y - handle * math.sin(target_yaw)),
                (target_x, target_y),
            )
            points = _sample_bezier(controls, spacing_m)
            curvature = max(abs(float(point["curvature_radpm"])) for point in points)
            if curvature > 1.05 * maximum_curvature:
                rejected_reasons.append("curvature")
                continue
            reverse = False
            for before, after in zip(points, points[1:]):
                dx = float(after["x"]) - float(before["x"])
                dy = float(after["y"]) - float(before["y"])
                if dx * math.cos(float(before["yaw"])) + dy * math.sin(float(before["yaw"])) < -1.0e-4:
                    reverse = True
                    break
            if reverse:
                rejected_reasons.append("reverse")
                continue
            if corridor_half_width_m is not None:
                outside = any(
                    distance_to_polyline(
                        float(point["x"]), float(point["y"]), route, closed=closed
                    ) > corridor_half_width_m + 1.0e-6
                    for point in points
                )
                if outside:
                    rejected_reasons.append("corridor")
                    continue
            speed = float(target.get("speed_mps", 0.10))
            for point in points:
                point["speed_mps"] = speed
                point["route_phase"] = "join"
            _, length = cumulative_lengths(points, closed=False)
            return JoinResult(points, target_index, length, curvature)
    reasons = ", ".join(sorted(set(rejected_reasons))) or "no forward target"
    raise ValueError(f"no safe curvature-limited route join: {reasons}")


def corridor_boundaries(
    route: Sequence[dict[str, Any]], half_width_m: float
) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    if half_width_m <= 0.0:
        raise ValueError("corridor half width must be positive")
    left, right = [], []
    for point in route:
        yaw = float(point.get("yaw", 0.0))
        normal_x, normal_y = -math.sin(yaw), math.cos(yaw)
        left.append({"x": float(point["x"]) + half_width_m * normal_x, "y": float(point["y"]) + half_width_m * normal_y, "yaw": yaw})
        right.append({"x": float(point["x"]) - half_width_m * normal_x, "y": float(point["y"]) - half_width_m * normal_y, "yaw": yaw})
    return left, right


def route_completion(
    *,
    closed: bool,
    unwrapped_index: int,
    route_point_count: int,
    distance_to_endpoint_m: float,
    heading_error_rad: float,
    endpoint_tolerance_m: float = 0.30,
    open_heading_tolerance_rad: float = math.pi,
    closed_progress_ratio: float = 0.95,
    closed_heading_tolerance_rad: float = math.radians(60.0),
) -> bool:
    if route_point_count < 2:
        return False
    if not closed:
        return (
            unwrapped_index >= route_point_count - 2
            and distance_to_endpoint_m <= endpoint_tolerance_m
            and heading_error_rad <= open_heading_tolerance_rad
        )
    progress = max(0.0, unwrapped_index) / route_point_count
    return (
        progress >= closed_progress_ratio
        and distance_to_endpoint_m <= endpoint_tolerance_m
        and heading_error_rad <= closed_heading_tolerance_rad
    )
