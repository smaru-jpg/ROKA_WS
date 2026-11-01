"""Differential-drive conversions shared by the node and tests."""

import math


def twist_to_wheel_rpm(linear_x, angular_z, separation, radius, max_rpm):
    left_mps = linear_x - angular_z * separation * 0.5
    right_mps = linear_x + angular_z * separation * 0.5
    factor = 60.0 / (2.0 * math.pi * radius)
    left_rpm, right_rpm = left_mps * factor, right_mps * factor
    peak = max(abs(left_rpm), abs(right_rpm))
    if peak > max_rpm:
        scale = max_rpm / peak
        left_rpm *= scale
        right_rpm *= scale
    return left_rpm, right_rpm


def rpm_to_mps(rpm, radius):
    return rpm * 2.0 * math.pi * radius / 60.0


def diagonal_covariance(diagonal):
    """Expand a six-element ROS pose/twist diagonal into row-major 6x6."""
    if len(diagonal) != 6:
        raise ValueError('covariance diagonal must contain six values')
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal):
        numeric = float(value)
        if not math.isfinite(numeric) or numeric < 0.0:
            raise ValueError('covariance values must be finite and non-negative')
        covariance[index * 6 + index] = numeric
    return covariance
