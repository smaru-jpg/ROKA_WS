"""Publish one simulation-assisted 3D prior for inspection in RViz."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
import yaml


class SimAssistedMapVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("sim_assisted_map_visualizer")
        self.declare_parameter("output_directory", "")
        directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser().resolve()
        archive_file = directory / "hybrid_map.npz"
        profile_file = directory / "track_profile_sim_assisted.yaml"
        manifest_file = directory / "manifest.yaml"
        for path in (archive_file, profile_file, manifest_file):
            if not path.is_file():
                raise FileNotFoundError(path)
        manifest = yaml.safe_load(manifest_file.read_text(encoding="utf-8")) or {}
        if manifest.get("status") != "SIMULATION_PRIOR_ONLY":
            raise ValueError("artifact is not an explicit simulation prior")
        if manifest.get("field_navigation_authorized") is not False:
            raise ValueError("simulation prior manifest must fail closed for field use")
        archive = np.load(archive_file)
        self.cloud = np.asarray(archive["visual_points"], dtype=np.float32)
        if self.cloud.ndim != 2 or self.cloud.shape[1] != 4:
            raise ValueError("hybrid_map visual_points must be finite Nx4 XYZI")
        if not np.all(np.isfinite(self.cloud)):
            raise ValueError("hybrid_map contains non-finite visualization points")
        document = yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {}
        self.frame_id = str(document.get("frame_id", "map")).lstrip("/")
        self.track_width = float(document["track_width_m"])
        self.points = [dict(point) for point in document.get("points", [])]
        if len(self.points) < 2:
            raise ValueError("simulation-assisted route needs at least two points")

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/sim_assisted/hybrid_cloud", latched
        )
        self.path_publisher = self.create_publisher(
            PathMessage, "/sim_assisted/path_3d", latched
        )
        self.left_publisher = self.create_publisher(
            PathMessage, "/sim_assisted/track_left", latched
        )
        self.right_publisher = self.create_publisher(
            PathMessage, "/sim_assisted/track_right", latched
        )
        self.risk_publisher = self.create_publisher(
            Marker, "/sim_assisted/risk", latched
        )
        self.timer = self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().warn(
            "simulation-assisted 3D prior ready: "
            f"{len(self.points)} route points, {len(self.cloud)} cloud points; "
            "not authorized as a real 3D localization map"
        )

    def _header(self) -> Header:
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _cloud_message(self) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        return point_cloud2.create_cloud(self._header(), fields, self.cloud)

    def _path(self, side: int = 0) -> PathMessage:
        message = PathMessage()
        message.header = self._header()
        lateral = side * 0.5 * self.track_width
        for point in self.points:
            yaw = float(point["yaw"])
            cross = math.tan(math.radians(float(point["cross_slope_deg"])))
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"]) - math.sin(yaw) * lateral
            pose.pose.position.y = float(point["y"]) + math.cos(yaw) * lateral
            pose.pose.position.z = float(point["z"]) + cross * lateral + 0.03
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    @staticmethod
    def _risk_color(risk: float) -> ColorRGBA:
        risk = min(1.0, max(0.0, risk))
        color = ColorRGBA()
        color.a = 1.0
        if risk <= 0.5:
            color.r = float(2.0 * risk)
            color.g = 1.0
        else:
            color.r = 1.0
            color.g = float(2.0 * (1.0 - risk))
        color.b = 0.05
        return color

    def _risk_marker(self) -> Marker:
        marker = Marker()
        marker.header = self._header()
        marker.ns = "sim_assisted_provisional_risk"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        for point in self.points:
            sample = Point()
            sample.x = float(point["x"])
            sample.y = float(point["y"])
            sample.z = float(point["z"]) + 0.08
            marker.points.append(sample)
            marker.colors.append(self._risk_color(
                float(point.get("traversability_cost", 1.0))
            ))
        return marker

    def _publish(self) -> None:
        self.cloud_publisher.publish(self._cloud_message())
        self.path_publisher.publish(self._path(0))
        self.left_publisher.publish(self._path(+1))
        self.right_publisher.publish(self._path(-1))
        self.risk_publisher.publish(self._risk_marker())


def main() -> None:
    rclpy.init()
    node = SimAssistedMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
