import math

from geometry_msgs.msg import Quaternion
import pytest

from roka_mujoco.nav2_sim_bridge import (
    Nav2SimBridge,
    roll_pitch_from_quaternion,
    yaw_from_quaternion,
)


def test_bridge_quaternion_helpers_preserve_rep103_angles():
    yaw = math.radians(25.0)
    quaternion = Quaternion(z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))
    assert math.degrees(yaw_from_quaternion(quaternion)) == pytest.approx(25.0)
    roll, pitch = roll_pitch_from_quaternion(quaternion)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_flat_cloud_is_dense_odom_surface():
    from builtin_interfaces.msg import Time

    cloud = Nav2SimBridge._flat_cloud(Time(sec=1), 2.0, -1.0)
    assert cloud.header.frame_id == "odom"
    assert cloud.width == 41 * 41
    assert cloud.point_step == 12
    assert len(cloud.data) == cloud.width * cloud.point_step
    assert cloud.is_dense
