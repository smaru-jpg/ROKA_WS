"""Publish isolated, EBIMU-shaped MuJoCo IMU and wheel-odometry topics."""

from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class SimSensorAdapter(Node):
    """Expose simulator sensors without sharing the real-hardware topic names."""

    def __init__(self) -> None:
        super().__init__("sim_sensor_adapter")
        self.declare_parameter("input_imu_topic", "/sim_imu_broadcaster/imu")
        self.declare_parameter("output_imu_topic", "/sim/imu/data")
        self.declare_parameter(
            "input_wheel_odom_topic", "/sim_diff_drive_controller/odom"
        )
        self.declare_parameter("output_wheel_odom_topic", "/sim/wheel/odom")
        self.declare_parameter("connected_topic", "/sim/imu/connected")
        self.declare_parameter("add_measured_noise", True)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("random_seed", 3103)
        self.declare_parameter("gyro_noise_stddev_radps", 0.010344)
        self.declare_parameter("accel_noise_stddev_mps2", 0.020)
        self.declare_parameter("orientation_stddev_rpy", [0.035, 0.035, 0.175])
        self.declare_parameter("data_timeout_sec", 0.25)

        self.add_noise = bool(self.get_parameter("add_measured_noise").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.publish_period_ns = int(1.0e9 / self.publish_rate)
        self.gyro_noise = float(
            self.get_parameter("gyro_noise_stddev_radps").value
        )
        self.accel_noise = float(
            self.get_parameter("accel_noise_stddev_mps2").value
        )
        self.orientation_stddev = [
            float(value)
            for value in self.get_parameter("orientation_stddev_rpy").value
        ]
        if len(self.orientation_stddev) != 3:
            raise ValueError("orientation_stddev_rpy must contain three values")
        self.timeout = float(self.get_parameter("data_timeout_sec").value)
        if self.timeout <= 0.0:
            raise ValueError("data_timeout_sec must be positive")
        self.rng = np.random.default_rng(
            int(self.get_parameter("random_seed").value)
        )

        self.imu_pub = self.create_publisher(
            Imu,
            str(self.get_parameter("output_imu_topic").value),
            qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_wheel_odom_topic").value),
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.connected_pub = self.create_publisher(
            Bool, str(self.get_parameter("connected_topic").value), status_qos
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("input_imu_topic").value),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_wheel_odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.last_imu_time = None
        self.last_imu_stamp_ns = None
        self.last_connected = None
        self.create_timer(0.10, self._publish_connection_state)

    def _imu_callback(self, source: Imu) -> None:
        stamp_ns = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec
        if (
            self.last_imu_stamp_ns is not None
            and stamp_ns - self.last_imu_stamp_ns < self.publish_period_ns
        ):
            return
        self.last_imu_stamp_ns = stamp_ns
        message = deepcopy(source)
        message.header.frame_id = "imu_link"
        quaternion_norm = math.sqrt(
            message.orientation.x ** 2
            + message.orientation.y ** 2
            + message.orientation.z ** 2
            + message.orientation.w ** 2
        )
        if quaternion_norm > 1.0e-9:
            message.orientation.x /= quaternion_norm
            message.orientation.y /= quaternion_norm
            message.orientation.z /= quaternion_norm
            message.orientation.w /= quaternion_norm
        if self.add_noise:
            gyro_noise = self.rng.normal(0.0, self.gyro_noise, 3)
            accel_noise = self.rng.normal(0.0, self.accel_noise, 3)
            message.angular_velocity.x += float(gyro_noise[0])
            message.angular_velocity.y += float(gyro_noise[1])
            message.angular_velocity.z += float(gyro_noise[2])
            message.linear_acceleration.x += float(accel_noise[0])
            message.linear_acceleration.y += float(accel_noise[1])
            message.linear_acceleration.z += float(accel_noise[2])
        for index, stddev in enumerate(self.orientation_stddev):
            message.orientation_covariance[index * 3 + index] = stddev ** 2
            message.angular_velocity_covariance[index * 3 + index] = (
                self.gyro_noise ** 2
            )
            message.linear_acceleration_covariance[index * 3 + index] = (
                self.accel_noise ** 2
            )
        self.imu_pub.publish(message)
        self.last_imu_time = self.get_clock().now()

    def _odom_callback(self, source: Odometry) -> None:
        message = deepcopy(source)
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        self.odom_pub.publish(message)

    def _publish_connection_state(self) -> None:
        connected = False
        if self.last_imu_time is not None:
            age = (self.get_clock().now() - self.last_imu_time).nanoseconds * 1.0e-9
            connected = 0.0 <= age <= self.timeout
        if connected != self.last_connected:
            self.connected_pub.publish(Bool(data=connected))
            self.last_connected = connected


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSensorAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, ExternalShutdownException):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass


if __name__ == "__main__":
    main()
