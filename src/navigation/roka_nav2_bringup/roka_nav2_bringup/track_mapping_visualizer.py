"""Unified live/saved 3D track-mapping visualization for RViz.

The node deliberately publishes display products only.  The projected 2D grid
is useful for reviewing map growth, but is never written into a navigation map
bundle or used to authorize autonomous motion.
"""

from __future__ import annotations

import math
import csv
from pathlib import Path

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from roka_interfaces.msg import TerrainState
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from roka_nav2_bringup.pcd_terrain_profile import load_ascii_xyz
from roka_robot_description.hardware_config import load_hardware_config


def _yaw(pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class TrackMappingVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("track_mapping_visualizer")
        self.declare_parameter("bundle_directory", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("projection_resolution_m", 0.05)
        self.declare_parameter("projection_margin_m", 1.5)
        self.declare_parameter("maximum_projection_cells", 2_000_000)
        self.frame_id = str(self.get_parameter("map_frame").value).lstrip("/")
        self.resolution = float(
            self.get_parameter("projection_resolution_m").value
        )
        self.margin = float(self.get_parameter("projection_margin_m").value)
        self.maximum_cells = int(
            self.get_parameter("maximum_projection_cells").value
        )
        if not 0.02 <= self.resolution <= 0.20:
            raise ValueError("projection_resolution_m must be in [0.02, 0.20]")
        if self.margin <= 0.0 or self.maximum_cells < 10_000:
            raise ValueError("projection margin/cell limit is invalid")

        hardware = load_hardware_config()
        self.half_width = float(hardware.navigation_half_width_m)
        self.light_cloud_xyz: np.ndarray | None = None
        self.dense_cloud_xyz: np.ndarray | None = None
        self.surface_cloud_xyz: np.ndarray | None = None
        self.graph_vertices: dict[int, tuple[float, float, float]] = {}
        self.graph_edges: list[tuple[int, int]] = []
        self.route: list[dict[str, float]] = []
        self.route_risk: list[float] = []
        self.current_risk = math.nan

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        live_cloud = QoSProfile(
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/track_visual/map_3d", latched
        )
        self.dense_cloud_pub = self.create_publisher(
            PointCloud2, "/track_visual/map_dense", latched
        )
        self.surface_cloud_pub = self.create_publisher(
            PointCloud2, "/track_visual/surface_3d", latched
        )
        self.graph_pub = self.create_publisher(
            MarkerArray, "/track_visual/pose_graph", latched
        )
        self.path_pub = self.create_publisher(
            PathMessage, "/track_visual/path_3d", latched
        )
        self.left_pub = self.create_publisher(
            PathMessage, "/track_visual/vehicle_left", latched
        )
        self.right_pub = self.create_publisher(
            PathMessage, "/track_visual/vehicle_right", latched
        )
        self.risk_pub = self.create_publisher(
            Marker, "/track_visual/risk", latched
        )
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, "/track_visual/occupancy", latched
        )
        self.create_subscription(
            PointCloud2, "/pgo/map", self._live_cloud, live_cloud
        )
        self.create_subscription(
            PathMessage, "/track/recorded_path", self._live_path, latched
        )
        self.create_subscription(
            TerrainState, "/terrain/state", self._terrain, 10
        )

        configured = str(self.get_parameter("bundle_directory").value).strip()
        if configured:
            self._load_bundle(Path(configured).expanduser().resolve())
        self.timer = self.create_timer(1.0, self._publish_all)
        self._publish_all()
        self.get_logger().info(
            "track mapping visualization ready: live 3D + driven route + "
            "vehicle envelope + display-only 2D projection"
        )

    def _header(self) -> Header:
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _load_bundle(self, directory: Path) -> None:
        pcd = directory / "map.pcd"
        dense_pcd = directory / "map_dense.pcd"
        surface_pcd = directory / "surface.pcd"
        profile = directory / "track_profile.yaml"
        if not pcd.is_file() or not profile.is_file():
            self.get_logger().warning(
                f"saved 3D bundle is incomplete; continuing in live mode: {directory}"
            )
            return
        self.light_cloud_xyz = load_ascii_xyz(pcd).astype(np.float32)
        if dense_pcd.is_file():
            self.dense_cloud_xyz = load_ascii_xyz(dense_pcd).astype(np.float32)
        else:
            self.dense_cloud_xyz = self.light_cloud_xyz
        if surface_pcd.is_file():
            self.surface_cloud_xyz = load_ascii_xyz(surface_pcd).astype(np.float32)
        document = yaml.safe_load(profile.read_text(encoding="utf-8")) or {}
        self.frame_id = str(document.get("frame_id", self.frame_id)).lstrip("/")
        self.route = []
        self.route_risk = []
        for entry in document.get("points") or []:
            point = {
                "x": float(entry["x"]),
                "y": float(entry["y"]),
                "z": float(entry.get("surface_height_m", entry.get("z", 0.0))),
                "yaw": float(entry.get("yaw", 0.0)),
                "cross_slope_deg": float(entry.get("cross_slope_deg", 0.0)),
            }
            self.route.append(point)
            risk = entry.get("risk", entry.get("traversability_cost"))
            self.route_risk.append(float(risk) if risk is not None else math.nan)
        optimized_path = directory / "optimized_path.csv"
        if optimized_path.is_file():
            optimized_route: list[dict[str, float]] = []
            with optimized_path.open(newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    optimized_route.append({
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "z": float(row["z"]),
                        "yaw": float(row["yaw"]),
                        "cross_slope_deg": 0.0,
                    })
            if optimized_route:
                self.route = optimized_route
                self.route_risk = [math.nan] * len(optimized_route)
        self._load_pose_graph(directory / "pose_graph.g2o")
        self.get_logger().info(
            f"saved track view loaded from {directory}: "
            f"{len(self.dense_cloud_xyz)} dense / {len(self.light_cloud_xyz)} "
            f"light / {len(self.surface_cloud_xyz) if self.surface_cloud_xyz is not None else 0} "
            f"surface points, {len(self.route)} route points, "
            f"{len(self.graph_vertices)} graph vertices; "
            "visualization does not alter or grant navigation approval"
        )

    def _load_pose_graph(self, path: Path) -> None:
        self.graph_vertices = {}
        self.graph_edges = []
        if not path.is_file():
            return
        for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            values = raw_line.split()
            if len(values) >= 5 and values[0] == "VERTEX_SE3:QUAT":
                self.graph_vertices[int(values[1])] = (
                    float(values[2]), float(values[3]), float(values[4])
                )
            elif len(values) >= 3 and values[0] == "EDGE_SE3:QUAT":
                self.graph_edges.append((int(values[1]), int(values[2])))

    def _live_cloud(self, message: PointCloud2) -> None:
        try:
            values = point_cloud2.read_points_numpy(
                message, field_names=["x", "y", "z"]
            )
            cloud = np.asarray(values, dtype=np.float32).reshape(-1, 3)
        except (AttributeError, ValueError):
            cloud = np.asarray(
                list(point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )),
                dtype=np.float32,
            ).reshape(-1, 3)
        cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
        if len(cloud) == 0:
            return
        self.frame_id = message.header.frame_id.lstrip("/") or self.frame_id
        self.light_cloud_xyz = cloud
        self.dense_cloud_xyz = cloud
        # A live registered cloud is the full scene, not a classified surface.
        self.surface_cloud_xyz = None
        self._publish_all()

    def _live_path(self, message: PathMessage) -> None:
        route = []
        for item in message.poses:
            pose = item.pose
            route.append({
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw": _yaw(pose),
                "cross_slope_deg": 0.0,
            })
        if len(route) < 1:
            return
        self.frame_id = message.header.frame_id.lstrip("/") or self.frame_id
        previous = self.route_risk
        self.route = route
        self.route_risk = previous[:len(route)]
        while len(self.route_risk) < len(route):
            self.route_risk.append(self.current_risk)
        self._publish_all()

    def _terrain(self, message: TerrainState) -> None:
        risk = float(message.risk)
        self.current_risk = min(1.0, max(0.0, risk)) if math.isfinite(risk) else math.nan

    def _cloud_message(self, cloud_xyz: np.ndarray | None) -> PointCloud2 | None:
        if cloud_xyz is None or len(cloud_xyz) == 0:
            return None
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(
            self._header(), fields, np.asarray(cloud_xyz, dtype=np.float32)
        )

    def _pose_graph_markers(self) -> MarkerArray:
        result = MarkerArray()
        vertices = Marker()
        vertices.header = self._header()
        vertices.ns = "keyframes"
        vertices.id = 0
        vertices.type = Marker.SPHERE_LIST
        vertices.action = Marker.ADD
        vertices.pose.orientation.w = 1.0
        vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.12
        vertices.color.r, vertices.color.g = 1.0, 0.55
        vertices.color.b, vertices.color.a = 0.05, 0.95
        for _, xyz in sorted(self.graph_vertices.items()):
            point = Point()
            point.x, point.y, point.z = xyz
            vertices.points.append(point)
        factors = Marker()
        factors.header = self._header()
        factors.ns = "factors"
        factors.id = 1
        factors.type = Marker.LINE_LIST
        factors.action = Marker.ADD
        factors.pose.orientation.w = 1.0
        factors.scale.x = 0.025
        factors.color.r, factors.color.g = 0.25, 0.65
        factors.color.b, factors.color.a = 1.0, 0.70
        for source, target in self.graph_edges:
            if source not in self.graph_vertices or target not in self.graph_vertices:
                continue
            for xyz in (self.graph_vertices[source], self.graph_vertices[target]):
                point = Point()
                point.x, point.y, point.z = xyz
                factors.points.append(point)
        result.markers = [vertices, factors]
        return result

    def _path_message(self, side: int = 0) -> PathMessage:
        message = PathMessage()
        message.header = self._header()
        lateral = side * self.half_width
        for point in self.route:
            yaw = point["yaw"]
            cross = math.tan(math.radians(point.get("cross_slope_deg", 0.0)))
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point["x"] - math.sin(yaw) * lateral
            pose.pose.position.y = point["y"] + math.cos(yaw) * lateral
            pose.pose.position.z = point["z"] + cross * lateral + 0.04
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    @staticmethod
    def _risk_color(risk: float) -> ColorRGBA:
        color = ColorRGBA()
        color.a = 1.0
        if not math.isfinite(risk):
            color.r, color.g, color.b = 0.65, 0.65, 0.70
        elif risk <= 0.5:
            color.r, color.g, color.b = 2.0 * risk, 1.0, 0.05
        else:
            color.r, color.g, color.b = 1.0, 2.0 * (1.0 - risk), 0.05
        return color

    def _risk_marker(self) -> Marker:
        marker = Marker()
        marker.header = self._header()
        marker.ns = "real_track_traversability"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.09
        for index, point in enumerate(self.route):
            sample = Point()
            sample.x, sample.y = point["x"], point["y"]
            sample.z = point["z"] + 0.10
            marker.points.append(sample)
            risk = self.route_risk[index] if index < len(self.route_risk) else math.nan
            marker.colors.append(self._risk_color(risk))
        return marker

    def _project_occupancy(self) -> OccupancyGrid | None:
        if self.light_cloud_xyz is None or len(self.light_cloud_xyz) == 0 or not self.route:
            return None
        path_xy = np.asarray([(p["x"], p["y"]) for p in self.route], dtype=np.float64)
        cloud_xy = np.asarray(self.light_cloud_xyz[:, :2], dtype=np.float64)
        combined = np.vstack((path_xy, cloud_xy))
        minimum = np.min(combined, axis=0) - self.margin
        maximum = np.max(combined, axis=0) + self.margin
        width, height = np.ceil((maximum - minimum) / self.resolution).astype(int) + 1
        if width < 2 or height < 2 or int(width) * int(height) > self.maximum_cells:
            return None
        grid = np.full((int(height), int(width)), -1, dtype=np.int8)
        columns = np.floor((cloud_xy[:, 0] - minimum[0]) / self.resolution).astype(int)
        rows = np.floor((cloud_xy[:, 1] - minimum[1]) / self.resolution).astype(int)
        valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
        grid[rows[valid], columns[valid]] = 100
        radius = int(math.ceil(self.half_width / self.resolution))
        for x, y in path_xy:
            cx = int(math.floor((x - minimum[0]) / self.resolution))
            cy = int(math.floor((y - minimum[1]) / self.resolution))
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    ix, iy = cx + dx, cy + dy
                    if 0 <= ix < width and 0 <= iy < height:
                        grid[iy, ix] = 0
        message = OccupancyGrid()
        message.header = self._header()
        message.info.resolution = self.resolution
        message.info.width = int(width)
        message.info.height = int(height)
        message.info.origin.position.x = float(minimum[0])
        message.info.origin.position.y = float(minimum[1])
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).astype(int).tolist()
        return message

    def _publish_all(self) -> None:
        cloud = self._cloud_message(self.light_cloud_xyz)
        dense_cloud = self._cloud_message(self.dense_cloud_xyz)
        surface_cloud = self._cloud_message(self.surface_cloud_xyz)
        if cloud is not None:
            self.cloud_pub.publish(cloud)
        if dense_cloud is not None:
            self.dense_cloud_pub.publish(dense_cloud)
        if surface_cloud is not None:
            self.surface_cloud_pub.publish(surface_cloud)
        if self.graph_vertices:
            self.graph_pub.publish(self._pose_graph_markers())
        if self.route:
            self.path_pub.publish(self._path_message(0))
            self.left_pub.publish(self._path_message(+1))
            self.right_pub.publish(self._path_message(-1))
            self.risk_pub.publish(self._risk_marker())
        occupancy = self._project_occupancy()
        if occupancy is not None:
            self.occupancy_pub.publish(occupancy)


def main() -> None:
    rclpy.init()
    node = TrackMappingVisualizer()
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
