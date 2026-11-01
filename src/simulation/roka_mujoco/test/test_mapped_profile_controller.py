import math

import numpy as np

from roka_mujoco.mapped_profile_controller import (
    mapping_profile_in_start_frame,
    open_path_nearest_index,
)


def test_mapping_profile_is_aligned_to_origin_and_zero_heading():
    points = [
        {"x": 10.0, "y": 5.0, "yaw": math.pi / 2.0},
        {"x": 10.0, "y": 6.0, "yaw": math.pi / 2.0},
    ]
    transformed = mapping_profile_in_start_frame(points)
    assert np.allclose([transformed[0]["x"], transformed[0]["y"]], [0.0, 0.0])
    assert math.isclose(float(transformed[0]["yaw"]), 0.0, abs_tol=1.0e-12)
    assert np.allclose([transformed[1]["x"], transformed[1]["y"]], [1.0, 0.0])


def test_open_path_search_does_not_wrap_end_to_start():
    path = np.asarray([[float(index), 0.0] for index in range(30)])
    index, distance = open_path_nearest_index(path, 0.0, 0.0, current_index=28)
    assert index >= 24
    assert distance >= 24.0
