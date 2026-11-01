import pytest

from roka_md_driver.xbox_teleop_node import shaped_axis, xbox_twist_values


def test_lb_normal_mode_scales_stick_to_point_three_mps():
    axes = [0.0, 1.0, 1.0]
    buttons = [0, 0, 0, 0, 1]
    linear, angular, enabled, turbo = xbox_twist_values(axes, buttons)
    assert enabled and not turbo
    assert linear == pytest.approx(0.30)
    assert angular == pytest.approx(0.0)


def test_lb_lt_turbo_is_binary_but_stick_remains_proportional():
    axes = [0.0, 0.5, -1.0]
    buttons = [0, 0, 0, 0, 1]
    linear, _angular, enabled, turbo = xbox_twist_values(
        axes, buttons, deadzone=0.0
    )
    assert enabled and turbo
    assert linear == pytest.approx(0.50)
    axes[1] = 1.0
    linear, _angular, _enabled, _turbo = xbox_twist_values(
        axes, buttons, deadzone=0.0
    )
    assert linear == pytest.approx(1.00)


def test_partial_lt_does_not_scale_speed_until_turbo_threshold():
    axes = [0.0, 1.0, -0.49]
    buttons = [0, 0, 0, 0, 1]
    linear, _angular, _enabled, turbo = xbox_twist_values(
        axes, buttons, deadzone=0.0
    )
    assert not turbo
    assert linear == pytest.approx(0.30)


def test_lb_release_and_bad_layout_fail_closed():
    assert xbox_twist_values([1.0, 1.0, -1.0], [0, 0, 0, 0, 0])[:2] == (0.0, 0.0)
    assert xbox_twist_values([], [0, 0, 0, 0, 1])[:2] == (0.0, 0.0)
    assert shaped_axis(0.04, 0.05) == 0.0

