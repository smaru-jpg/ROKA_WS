import math

import pytest

from roka_nav2_bringup.track_path_follower import bounded_start_distance
from roka_nav2_bringup.route_geometry import (
    corridor_boundaries,
    generate_join_path,
    prepare_centerline,
    project_pose,
    route_completion,
)


def test_short_route_tightens_start_join_without_changing_long_route_limit():
    assert bounded_start_distance(2.0, 1.13) == pytest.approx(0.565)
    assert bounded_start_distance(0.75, 10.0) == pytest.approx(0.75)
    assert bounded_start_distance(2.0, 0.20) == pytest.approx(0.30)
    with pytest.raises(ValueError):
        bounded_start_distance(2.0, 0.0)


def test_manual_trace_becomes_bounded_uniform_centerline():
    raw = [
        {"x": 0.10 * index, "y": 0.025 * math.sin(1.7 * index), "speed_mps": 0.10}
        for index in range(31)
    ]
    route, metadata = prepare_centerline(
        raw, closed=False, spacing_m=0.05, maximum_deviation_m=0.04
    )
    assert metadata["source_point_count"] == 31
    assert metadata["maximum_source_deviation_m"] <= 0.04 + 1.0e-6
    assert len(route) > len(raw)
    spacings = [
        math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        for a, b in zip(route, route[1:])
    ]
    assert max(spacings) <= 0.051
    assert all(math.isfinite(point["curvature_radpm"]) for point in route)


def test_projection_window_prevents_jump_to_nearby_later_lane():
    # The final return lane is only 0.10 m away from the first lane.
    raw = [
        {"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0},
        {"x": 2.0, "y": 0.0}, {"x": 2.0, "y": 1.0},
        {"x": 1.0, "y": 1.0}, {"x": 0.0, "y": 1.0},
        {"x": 0.0, "y": 0.10}, {"x": 1.0, "y": 0.10},
        {"x": 2.0, "y": 0.10},
    ]
    route, _ = prepare_centerline(
        raw, closed=False, spacing_m=0.10, maximum_deviation_m=0.02,
        smoothing_iterations=0,
    )
    initial = project_pose(route, 0.30, 0.01, 0.0, closed=False)
    bounded = project_pose(
        route, 0.60, 0.08, 0.0, closed=False,
        previous_unwrapped_index=initial.unwrapped_index,
        backward_window=2, forward_window=8,
    )
    assert bounded.unwrapped_index < 15
    assert bounded.y < 0.06


def test_curvature_limited_join_enters_forward_route_without_reverse():
    raw = [{"x": 0.10 * index, "y": 0.0, "speed_mps": 0.10} for index in range(31)]
    route, _ = prepare_centerline(raw, closed=False, spacing_m=0.05)
    projection = project_pose(route, -0.15, 0.12, 0.0, closed=False)
    join = generate_join_path(
        route, -0.15, 0.12, 0.0, projection,
        closed=False, minimum_turn_radius_m=0.50,
        corridor_half_width_m=0.25,
    )
    assert join.length_m > 0.50
    assert join.maximum_abs_curvature_radpm <= 2.12
    assert join.points[0]["x"] == pytest.approx(-0.15)
    assert join.points[-1]["y"] == pytest.approx(0.0, abs=1.0e-6)
    assert all(point["route_phase"] == "join" for point in join.points)


def test_corridor_and_open_closed_completion_are_distinct():
    route, _ = prepare_centerline(
        [{"x": 0.1 * index, "y": 0.0} for index in range(11)],
        closed=False,
    )
    left, right = corridor_boundaries(route, 0.20)
    assert left[5]["y"] == pytest.approx(0.20)
    assert right[5]["y"] == pytest.approx(-0.20)
    assert route_completion(
        closed=False, unwrapped_index=len(route) - 2,
        route_point_count=len(route), distance_to_endpoint_m=0.25,
        heading_error_rad=math.pi,
    )
    assert not route_completion(
        closed=False,
        unwrapped_index=len(route) - 2,
        route_point_count=len(route),
        distance_to_endpoint_m=0.07,
        heading_error_rad=math.radians(11.0),
        endpoint_tolerance_m=0.08,
        open_heading_tolerance_rad=math.radians(10.0),
    )
    assert route_completion(
        closed=False,
        unwrapped_index=len(route) - 2,
        route_point_count=len(route),
        distance_to_endpoint_m=0.07,
        heading_error_rad=math.radians(9.0),
        endpoint_tolerance_m=0.08,
        open_heading_tolerance_rad=math.radians(10.0),
    )
    assert not route_completion(
        closed=True, unwrapped_index=int(0.80 * len(route)),
        route_point_count=len(route), distance_to_endpoint_m=0.05,
        heading_error_rad=0.0,
    )
    assert route_completion(
        closed=True, unwrapped_index=len(route),
        route_point_count=len(route), distance_to_endpoint_m=0.05,
        heading_error_rad=math.radians(10.0),
    )
