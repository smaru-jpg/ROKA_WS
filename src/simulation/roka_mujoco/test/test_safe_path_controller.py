import math

from roka_mujoco.demonstration_recorder import signed_path_errors
from roka_mujoco.safe_path_controller import (
    lateral_motion_speed_limit,
    pure_pursuit_angular_velocity,
    terrain_relative_roll_error_deg,
    wrap_angle,
)


def test_pure_pursuit_turns_toward_target_and_limits_yaw_rate():
    left = pure_pursuit_angular_velocity(0.0, 0.0, 0.0, 1.0, 0.4, 0.2, 0.45)
    right = pure_pursuit_angular_velocity(0.0, 0.0, 0.0, 1.0, -0.4, 0.2, 0.45)
    assert 0.0 < left <= 0.45
    assert -0.45 <= right < 0.0
    assert math.isclose(left, -right)


def test_wrap_angle_uses_short_rotation():
    assert math.isclose(wrap_angle(2.0 * math.pi + 0.2), 0.2)


def test_lateral_motion_governor_slows_before_hard_limit():
    assert math.isclose(lateral_motion_speed_limit(0.14, 0.02, 0.04, 0.10, 0.04), 0.14)
    reduced = lateral_motion_speed_limit(0.14, 0.07, 0.04, 0.10, 0.04)
    assert 0.04 < reduced < 0.14
    assert math.isclose(
        lateral_motion_speed_limit(0.14, 0.10, 0.04, 0.10, 0.04),
        0.04,
    )


def test_roll_error_is_measured_relative_to_bank_angle():
    assert math.isclose(terrain_relative_roll_error_deg(math.radians(45.0), 40.0), 5.0)
    assert math.isclose(terrain_relative_roll_error_deg(math.radians(-35.0), -40.0), 5.0)


def test_demonstration_path_errors_use_left_positive_frame():
    lateral, heading = signed_path_errors(1.0, 0.2, 0.1, 1.0, 0.0, 0.0)
    assert math.isclose(lateral, 0.2)
    assert math.isclose(heading, 0.1)
