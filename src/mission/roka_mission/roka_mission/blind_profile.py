"""Validated command profiles for a short straight LiDAR-degraded segment."""

from bisect import bisect_right
import math


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def validate_profile(document):
    if not isinstance(document, dict) or document.get("format") != "roka_blind_v2":
        raise ValueError(
            "not a start-origin ROKA blind profile; re-record it with blind-record"
        )
    entry = float(document.get("entry_progress_ratio", -1.0))
    exit_ratio = float(document.get("exit_progress_ratio", -1.0))
    if not math.isfinite(entry) or abs(entry) > 1.0e-9:
        raise ValueError("entry_progress_ratio must be 0.0 for Section 1")
    if not math.isfinite(exit_ratio) or not any(
        math.isclose(exit_ratio, choice, abs_tol=1.0e-9)
        for choice in (0.20, 0.30, 0.40)
    ):
        raise ValueError("exit_progress_ratio must be 0.20, 0.30 or 0.40")
    commands = document.get("commands")
    if not isinstance(commands, list) or len(commands) < 2:
        raise ValueError("blind profile requires at least two command samples")
    previous = -1.0
    normalized = []
    for sample in commands:
        elapsed = float(sample["elapsed_sec"])
        linear = float(sample["linear_mps"])
        angular = float(sample["angular_radps"])
        yaw_delta = float(sample.get("yaw_delta_rad", 0.0))
        if not all(math.isfinite(value) for value in (elapsed, linear, angular, yaw_delta)):
            raise ValueError("blind profile contains a non-finite value")
        if elapsed < 0.0 or elapsed <= previous:
            raise ValueError("blind profile timestamps must increase")
        if previous >= 0.0 and elapsed - previous > 0.25:
            raise ValueError("blind profile has a command gap larger than 0.25 s")
        if abs(linear) > 0.60 or abs(angular) > 2.5:
            raise ValueError("blind profile command exceeds the real base envelope")
        normalized.append({
            "elapsed_sec": elapsed,
            "linear_mps": linear,
            "angular_radps": angular,
            "yaw_delta_rad": yaw_delta,
        })
        previous = elapsed
    duration = float(document.get("duration_sec", normalized[-1]["elapsed_sec"]))
    if (
        not math.isfinite(duration)
        or duration <= 0.0
        or duration < normalized[-1]["elapsed_sec"]
        or duration - normalized[-1]["elapsed_sec"] > 0.25
        or duration > 120.0
    ):
        raise ValueError("blind profile duration is invalid")
    distance = float(document.get("distance_m", 0.0))
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("blind profile distance is invalid")
    return {
        **document,
        "entry_progress_ratio": entry,
        "exit_progress_ratio": exit_ratio,
        "duration_sec": duration,
        "distance_m": distance,
        "commands": normalized,
    }


def sample_at(profile, elapsed_sec):
    commands = profile["commands"]
    times = [sample["elapsed_sec"] for sample in commands]
    index = max(0, min(len(commands) - 1, bisect_right(times, elapsed_sec) - 1))
    return commands[index]


def route_length(points):
    """Return finite 2D polyline length for a recorded route."""
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("route requires at least two points")
    total = 0.0
    previous = None
    for point in points:
        x, y = float(point["x"]), float(point["y"])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("route contains a non-finite point")
        if previous is not None:
            total += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)
    if total <= 1.0e-6:
        raise ValueError("route length is zero")
    return total


def project_route_progress(points, x, y):
    """Project a position onto an open route and return ratio and distance."""
    x, y = float(x), float(y)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("projection position is not finite")
    total = route_length(points)
    travelled = 0.0
    best_distance = math.inf
    best_progress = 0.0
    for before, after in zip(points, points[1:]):
        x0, y0 = float(before["x"]), float(before["y"])
        x1, y1 = float(after["x"]), float(after["y"])
        dx, dy = x1 - x0, y1 - y0
        length_squared = dx * dx + dy * dy
        length = math.sqrt(length_squared)
        if length_squared <= 1.0e-12:
            continue
        ratio = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / length_squared))
        projected_x, projected_y = x0 + ratio * dx, y0 + ratio * dy
        distance = math.hypot(x - projected_x, y - projected_y)
        if distance < best_distance:
            best_distance = distance
            best_progress = (travelled + ratio * length) / total
        travelled += length
    return best_progress, best_distance
