import math

import numpy as np

from roka_mujoco.sim_assisted_map import (
    project_to_route,
    terrain_ribbon_points,
    transfer_simulation_terrain,
)


def _simulation_profile():
    points = []
    for index, (x, y, elevation, longitudinal) in enumerate((
        (0.0, 0.0, 0.4, -20.0),
        (1.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.2, 20.0),
        (0.0, 1.0, 0.4, 0.0),
    )):
        points.append({
            "x": x,
            "y": y,
            "yaw": 0.0,
            "elevation_m": elevation,
            "cross_slope_deg": 5.0,
            "longitudinal_slope_deg": longitudinal,
            "minimum_wheel_edge_m": 0.25,
            "maximum_wheel_slope_deg": abs(longitudinal),
            "mean_wheel_slope_deg": 0.5 * abs(longitudinal),
            "demonstration_risk": 0.1,
            "traversability_cost": 0.1,
            "module_id": "START" if index == 0 else f"M{index}",
        })
    return {
        "closed_loop": True,
        "track_width_m": 0.9,
        "points": points,
    }


def test_metric_transfer_keeps_real_xy_and_does_not_normalize_length():
    real = {
        "frame_id": "map",
        "closed_loop": False,
        "points": [
            {"x": 10.0, "y": 2.0, "yaw": 0.0},
            {"x": 10.5, "y": 2.0, "yaw": 0.0},
        ],
    }
    result = transfer_simulation_terrain(real, _simulation_profile(), "START")
    points = result["points"]
    assert [(point["x"], point["y"]) for point in points] == [
        (10.0, 2.0),
        (10.5, 2.0),
    ]
    assert math.isclose(points[0]["z"], 0.0, abs_tol=1.0e-12)
    assert math.isclose(points[1]["z"], -0.2, abs_tol=1.0e-12)
    assert math.isclose(points[0]["pitch"], math.radians(20.0))
    assert result["autonomy_approved"] is False
    assert result["simulation_prior"]["field_navigation_authorized"] is False


def test_projection_returns_metric_progress_and_signed_lateral_offset():
    route = [
        {"x": 0.0, "y": 0.0},
        {"x": 1.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
    ]
    arc, lateral = project_to_route(
        np.asarray([[0.25, 0.10], [1.75, -0.20]]), route
    )
    assert np.allclose(arc, [0.25, 1.75])
    assert np.allclose(lateral, [0.10, -0.20])


def test_ribbon_contains_track_and_lower_outside_apron():
    real = {
        "frame_id": "map",
        "closed_loop": False,
        "points": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"x": 0.5, "y": 0.0, "yaw": 0.0},
        ],
    }
    result = transfer_simulation_terrain(real, _simulation_profile(), "START")
    cloud = terrain_ribbon_points(
        result, lateral_spacing_m=0.05, outside_margin_m=0.20, drop_m=0.18
    )
    assert cloud.shape[1] == 4
    assert np.all(np.isfinite(cloud))
    # Intensity 32 identifies the deliberately lower, non-rideable apron.
    assert np.count_nonzero(np.isclose(cloud[:, 3], 32.0)) > 0
    assert np.max(cloud[:, 3]) > 32.0
