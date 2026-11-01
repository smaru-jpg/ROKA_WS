import math
from types import SimpleNamespace

import pytest

from roka_nav2_bringup.track_path_recorder import (
    point_distance,
    quaternion_from_rpy,
    rpy_from_quaternion,
)


def test_rpy_round_trip_preserves_ramp_attitude():
    expected = (math.radians(7.0), math.radians(-18.0), math.radians(42.0))
    x, y, z, w = quaternion_from_rpy(*expected)
    quaternion = SimpleNamespace(x=x, y=y, z=z, w=w)
    actual = rpy_from_quaternion(quaternion)
    assert actual == pytest.approx(expected, abs=1.0e-10)


def test_ramp_distance_includes_height_only_for_3d_profile():
    first = {"x": 0.0, "y": 0.0, "z": 0.0}
    second = {"x": 3.0, "y": 0.0, "z": 4.0}
    assert point_distance(first, second, three_dimensional=False) == 3.0
    assert point_distance(first, second, three_dimensional=True) == 5.0
