import math

import numpy as np

from roka_mujoco.demonstration_learning import (
    PATH_POINT_COUNT,
    _completed_lap_reference,
    _unwrap_progress,
)
from roka_mujoco.learned_controller import _pure_pursuit_angular


def test_completed_lap_reference_is_periodic_and_finite():
    lateral = [[] for _ in range(PATH_POINT_COUNT)]
    heading = [[] for _ in range(PATH_POINT_COUNT)]
    for index, offset, yaw in (
        (0, -0.10, math.radians(2.0)),
        (PATH_POINT_COUNT // 2, 0.05, math.radians(-3.0)),
        (PATH_POINT_COUNT - 1, -0.09, math.radians(1.5)),
    ):
        lateral[index].append(offset)
        heading[index].append(yaw)

    reference_lateral, reference_heading = _completed_lap_reference(
        lateral, heading
    )

    assert reference_lateral.shape == (PATH_POINT_COUNT,)
    assert reference_heading.shape == (PATH_POINT_COUNT,)
    assert np.all(np.isfinite(reference_lateral))
    assert np.all(np.isfinite(reference_heading))
    assert abs(reference_lateral[0] - reference_lateral[-1]) < 0.01


def test_unwrap_progress_distinguishes_partial_and_complete_laps():
    partial = list(range(0, 300))
    complete = list(range(100, PATH_POINT_COUNT)) + list(range(0, 101))

    assert _unwrap_progress(partial) < 0.90
    assert _unwrap_progress(complete) >= 0.99


def test_pure_pursuit_turns_toward_mapped_lookahead_point():
    left = _pure_pursuit_angular(0.0, 0.0, 0.0, 1.0, 0.5, 0.2)
    right = _pure_pursuit_angular(0.0, 0.0, 0.0, 1.0, -0.5, 0.2)
    straight = _pure_pursuit_angular(0.0, 0.0, 0.0, 1.0, 0.0, 0.2)

    assert left > 0.0
    assert right < 0.0
    assert straight == 0.0
