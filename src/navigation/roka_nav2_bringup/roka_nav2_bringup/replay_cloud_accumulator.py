"""Accumulate recorded VLP-16 scans in a fixed frame for RViz review.

This is deliberately a display product.  It never writes a navigation map and
never participates in localization.  The accumulated cloud lets an operator
compare the raw rosbag evidence with the separately optimized FAST-LIO/SC-PGO
PCD in the same RViz window.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class ReplayCloudAccumulator(Node):
    def __init__(self) -> None:
        super().__init__("replay_cloud_accumulator")
        self.declare_parameter("input_topic", "/velodyne_points")
        self.declare_parameter("output_topic", "/replay/raw_accumulated_cloud")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("voxel_size_m", 0.08)
        self.declare_parameter("minimum_range_m", 0.35)
        self.declare_parameter("maximum_range_m", 30.0)
        self.declare_parameter("minimum_z_m", -3.0)
        self.declare_parameter("maximum_z_m", 4.0)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("maximum_points", 250_000)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value).lstrip("/")
        self.voxel_size = float(self.get_parameter("voxel_size_m").value)
        self.minimum_range = float(self.get_parameter("minimum_range_m").value)
        self.maximum_range = float(self.get_parameter("maximum_range_m").value)
        self.minimum_z = float(self.get_parameter("minimum_z_m").value)
        self.maximum_z = float(self.get_parameter("maximum_z_m").value)
        self.scan_stride = int(self.get_parameter("scan_stride").value)
        self.maximum_points = int(self.get_parameter("maximum_points").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if not (
            0.02 <= self.voxel_size <= 0.50
            and 0.0 <= self.minimum_range < self.maximum_range
            and self.minimum_z < self.maximum_z
            and self.scan_stride >= 1
            and self.maximum_points >= 10_000
            and publish_rate > 0.0
        ):
            raise ValueError("invalid replay cloud accumulation parameters")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud, qos_profile_sensor_data
        )
        self.create_service(Trigger, "~/reset", self._reset_service)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.voxels: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
        self.scan_count = 0
        self.accepted_scans = 0
        self.dropped_tf = 0
        self.last_clock_ns: int | None = None
        self.dirty = False
        self.tf_warning_active = False
        self.get_logger().info(
            "raw replay accumulation ready: %s -> %s, frame=%s, voxel=%.2fm"
            % (self.input_topic, self.output_topic, self.target_frame, self.voxel_size)
        )

    @staticmethod
    def _rotation(transform) -> np.ndarray:
        q = transform.rotation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.asarray([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    def _reset(self, reason: str) -> None:
        previous = len(self.voxels)
        self.voxels.clear()
        self.accepted_scans = 0
        self.dirty = True
        self.get_logger().info(
            f"raw accumulated display reset ({reason}); removed {previous} voxels"
        )

    def _reset_service(self, _request, response):
        self._reset("operator request")
        response.success = True
        response.message = "raw accumulated display cleared"
        return response

    def _cloud(self, message: PointCloud2) -> None:
        self.scan_count += 1
        if (self.scan_count - 1) % self.scan_stride:
            return
        clock_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and clock_ns + 100_000_000 < self.last_clock_ns:
            self._reset("rosbag loop/time rewind")
        self.last_clock_ns = clock_ns

        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame:
            return
        exact_error = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.08),
            )
            self.tf_warning_active = False
        except TransformException as error:
            exact_error = error
            # rosbag messages recorded by independent nodes can arrive a few
            # milliseconds before the matching TF callback.  For this
            # display-only accumulator, the latest available transform is a
            # better fallback than dropping the complete VLP-16 revolution.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
                self.tf_warning_active = False
            except TransformException as latest_error:
                self.dropped_tf += 1
                if not self.tf_warning_active:
                    self.get_logger().warning(
                        "raw scan waiting for %s<-%s TF: %s; latest: %s"
                        % (self.target_frame, source_frame, exact_error, latest_error)
                    )
                    self.tf_warning_active = True
                return

        field_names = {field.name for field in message.fields}
        requested = ["x", "y", "z"]
        has_intensity = "intensity" in field_names
        if has_intensity:
            requested.append("intensity")
        # VLP-16 clouds also contain integer ring fields, therefore Humble's
        # read_points_numpy() rejects the complete mixed-datatype schema even
        # when only float fields are requested. read_points() returns a
        # zero-copy structured NumPy view and supports that schema correctly.
        values = point_cloud2.read_points(
            message, field_names=requested, skip_nans=True
        )
        values = np.asarray(values)
        if values.size == 0:
            return
        if values.dtype.names:
            xyz = np.column_stack([values[name] for name in ("x", "y", "z")])
            intensity = (
                np.asarray(values["intensity"], dtype=np.float64)
                if has_intensity else np.zeros(len(xyz), dtype=np.float64)
            )
        else:
            values = values.reshape(-1, len(requested)).astype(np.float64, copy=False)
            xyz = values[:, :3]
            intensity = values[:, 3] if has_intensity else np.zeros(len(xyz))
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz), axis=1) & np.isfinite(intensity)
        radius = np.linalg.norm(xyz[:, :2], axis=1)
        selected = (
            finite
            & (radius >= self.minimum_range)
            & (radius <= self.maximum_range)
            & (xyz[:, 2] >= self.minimum_z)
            & (xyz[:, 2] <= self.maximum_z)
        )
        if not np.any(selected):
            return
        xyz = xyz[selected]
        intensity = np.asarray(intensity[selected], dtype=np.float64)

        rotation = self._rotation(transform.transform)
        translation = np.asarray([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        fixed = xyz @ rotation.T + translation
        indices = np.floor(fixed / self.voxel_size).astype(np.int64)
        # One representative per voxel per scan keeps the Python replay node
        # deterministic and bounded without changing any saved evidence.
        _, unique_indices = np.unique(indices, axis=0, return_index=True)
        for index in unique_indices:
            key = tuple(int(value) for value in indices[index])
            if key not in self.voxels and len(self.voxels) >= self.maximum_points:
                continue
            point = fixed[index]
            self.voxels[key] = (
                float(point[0]), float(point[1]), float(point[2]), float(intensity[index])
            )
        self.accepted_scans += 1
        self.dirty = True

    def _publish(self) -> None:
        if not self.dirty or not self.voxels:
            return
        header = message_header = PointCloud2().header
        header.frame_id = self.target_frame
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        points = np.asarray(list(self.voxels.values()), dtype=np.float32)
        self.publisher.publish(point_cloud2.create_cloud(message_header, fields, points))
        self.dirty = False


def main() -> None:
    rclpy.init()
    node = ReplayCloudAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
