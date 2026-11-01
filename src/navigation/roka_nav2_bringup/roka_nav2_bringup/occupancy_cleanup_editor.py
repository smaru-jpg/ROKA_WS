"""RViz click-to-clear editor for a saved occupancy map."""

from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from std_srvs.srv import Trigger
import yaml

from roka_robot_description.hardware_config import load_hardware_config
from roka_nav2_bringup.map_bundle_quality import _pgm_tokens
from roka_nav2_bringup.route_geometry import corridor_boundaries


class OccupancyCleanupEditor(Node):
    def __init__(self):
        super().__init__("occupancy_cleanup_editor")
        self.declare_parameter("input_yaml", "")
        self.declare_parameter("output_yaml", "")
        self.declare_parameter("audit_file", "")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("clear_radius_m", 0.20)
        self.input_yaml = Path(
            str(self.get_parameter("input_yaml").value)
        ).expanduser().resolve()
        self.output_yaml = Path(
            str(self.get_parameter("output_yaml").value)
        ).expanduser().resolve()
        self.audit_file = Path(
            str(self.get_parameter("audit_file").value)
        ).expanduser().resolve()
        profile_value = str(self.get_parameter("profile_file").value).strip()
        if not profile_value:
            raise ValueError("profile_file is required for map/path review")
        self.profile_file = Path(profile_value).expanduser().resolve()
        self.radius = float(self.get_parameter("clear_radius_m").value)
        if not self.input_yaml.is_file():
            raise ValueError(f"input occupancy YAML is missing: {self.input_yaml}")
        if self.output_yaml == self.input_yaml:
            raise ValueError("occupancy cleanup never overwrites its source")
        if not 0.02 <= self.radius <= 2.0:
            raise ValueError("clear_radius_m must be in [0.02, 2.0]")
        self.document = yaml.safe_load(self.input_yaml.read_text(encoding="utf-8")) or {}
        image_value = Path(str(self.document.get("image", ""))).expanduser()
        self.input_image = (
            image_value if image_value.is_absolute() else self.input_yaml.parent / image_value
        ).resolve()
        self.width, self.height, maximum, pixels = _pgm_tokens(self.input_image)
        if maximum != 255:
            raise ValueError("occupancy editor requires an 8-bit PGM")
        self.original = bytearray(pixels)
        self.working = bytearray(pixels)
        self.history: list[list[tuple[int, int]]] = []
        self.resolution = float(self.document["resolution"])
        self.origin = [float(value) for value in self.document["origin"][:3]]
        if self.resolution <= 0.0:
            raise ValueError("occupancy resolution must be positive")
        self.route_frame, self.route_points = self._load_route_profile()
        hardware = load_hardware_config()
        self.vehicle_half_width = hardware.navigation_half_width_m
        self.vehicle_left, self.vehicle_right = corridor_boundaries(
            self.route_points, self.vehicle_half_width
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.original_pub = self.create_publisher(
            OccupancyGrid, "/occupancy_cleanup/original", qos
        )
        self.working_pub = self.create_publisher(
            OccupancyGrid, "/occupancy_cleanup/working", qos
        )
        self.route_pub = self.create_publisher(
            PathMessage, "/occupancy_cleanup/recorded_path", qos
        )
        self.vehicle_left_pub = self.create_publisher(
            PathMessage, "/occupancy_cleanup/vehicle_left", qos
        )
        self.vehicle_right_pub = self.create_publisher(
            PathMessage, "/occupancy_cleanup/vehicle_right", qos
        )
        self.create_subscription(PointStamped, "/clicked_point", self._click, 10)
        self.create_service(Trigger, "/occupancy_cleanup/undo", self._undo)
        self.create_service(Trigger, "/occupancy_cleanup/reset", self._reset)
        self.create_service(Trigger, "/occupancy_cleanup/save", self._save)
        self._publish()
        self.get_logger().info(
            "2D map/path review ready: %dx%d, route=%d points, "
            "vehicle width=%.3fm, clear radius=%.2fm"
            % (
                self.width,
                self.height,
                len(self.route_points),
                2.0 * self.vehicle_half_width,
                self.radius,
            )
        )

    def _load_route_profile(self) -> tuple[str, list[dict[str, float]]]:
        if not self.profile_file.is_file():
            raise ValueError(f"recorded route is missing: {self.profile_file}")
        document = yaml.safe_load(self.profile_file.read_text(encoding="utf-8")) or {}
        frame_id = str(document.get("frame_id", "map")).lstrip("/")
        if frame_id != "map":
            raise ValueError(
                f"recorded route must use map frame for 2D review, got: {frame_id}"
            )
        source = document.get("points")
        if not isinstance(source, list) or len(source) < 2:
            raise ValueError("recorded route needs at least two points")
        points: list[dict[str, float]] = []
        for index, entry in enumerate(source):
            if not isinstance(entry, dict):
                raise ValueError(f"recorded route point {index} is not a mapping")
            x, y = float(entry["x"]), float(entry["y"])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(f"recorded route point {index} is non-finite")
            yaw_value = entry.get("yaw")
            yaw = float(yaw_value) if yaw_value is not None else math.nan
            points.append({"x": x, "y": y, "yaw": yaw})
        for index, point in enumerate(points):
            if math.isfinite(point["yaw"]):
                continue
            before = points[max(0, index - 1)]
            after = points[min(len(points) - 1, index + 1)]
            point["yaw"] = math.atan2(after["y"] - before["y"], after["x"] - before["x"])
        return frame_id, points

    def _path_message(self, points: list[dict[str, float]]) -> PathMessage:
        message = PathMessage()
        message.header.frame_id = self.route_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            # Keep the route visible above the occupancy texture in RViz.
            pose.pose.position.z = 0.04
            yaw = float(point.get("yaw", 0.0))
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    def _message(self, pixels: bytearray) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin[0]
        message.info.origin.position.y = self.origin[1]
        message.info.origin.orientation.z = math.sin(0.5 * self.origin[2])
        message.info.origin.orientation.w = math.cos(0.5 * self.origin[2])
        # PGM rows are top-down; OccupancyGrid rows are bottom-up.
        values = []
        for row in range(self.height - 1, -1, -1):
            for column in range(self.width):
                pixel = pixels[row * self.width + column]
                values.append(0 if pixel >= 250 else 100 if pixel <= 10 else -1)
        message.data = values
        return message

    def _publish(self):
        self.original_pub.publish(self._message(self.original))
        self.working_pub.publish(self._message(self.working))
        self.route_pub.publish(self._path_message(self.route_points))
        self.vehicle_left_pub.publish(self._path_message(self.vehicle_left))
        self.vehicle_right_pub.publish(self._path_message(self.vehicle_right))

    def _click(self, message: PointStamped):
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().error("RViz Fixed Frame must be map")
            return
        dx, dy = message.point.x - self.origin[0], message.point.y - self.origin[1]
        cosine, sine = math.cos(self.origin[2]), math.sin(self.origin[2])
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        center_column = math.floor(local_x / self.resolution)
        center_bottom = math.floor(local_y / self.resolution)
        radius_cells = math.ceil(self.radius / self.resolution)
        operation = []
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if offset_x * offset_x + offset_y * offset_y > radius_cells * radius_cells:
                    continue
                column = center_column + offset_x
                row = self.height - 1 - (center_bottom + offset_y)
                if not (0 <= row < self.height and 0 <= column < self.width):
                    continue
                index = row * self.width + column
                if self.working[index] != 254:
                    operation.append((index, self.working[index]))
                    self.working[index] = 254
        if not operation:
            self.get_logger().warning("clicked area already contains only free cells")
            return
        self.history.append(operation)
        self.working_pub.publish(self._message(self.working))
        self.get_logger().info(f"preview cleared {len(operation)} occupancy cells")

    def _undo(self, _request, response):
        if not self.history:
            response.success = False
            response.message = "nothing to undo"
            return response
        for index, value in self.history.pop():
            self.working[index] = value
        self.working_pub.publish(self._message(self.working))
        response.success = True
        response.message = "last 2D cleanup operation undone"
        return response

    def _reset(self, _request, response):
        self.working = bytearray(self.original)
        self.history.clear()
        self.working_pub.publish(self._message(self.working))
        response.success = True
        response.message = "2D cleanup reset to source"
        return response

    def _save(self, _request, response):
        try:
            self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
            output_image = self.output_yaml.with_suffix(".pgm")
            image_temp = output_image.with_suffix(".pgm.tmp")
            yaml_temp = self.output_yaml.with_suffix(".yaml.tmp")
            audit_temp = self.audit_file.with_suffix(".yaml.tmp")
            with image_temp.open("wb") as stream:
                stream.write(f"P5\n{self.width} {self.height}\n255\n".encode("ascii"))
                stream.write(self.working)
            document = dict(self.document)
            document["image"] = output_image.name
            yaml_temp.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
            changed = sum(a != b for a, b in zip(self.original, self.working))
            audit_temp.write_text(yaml.safe_dump({
                "schema_version": 1,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "source_yaml": str(self.input_yaml),
                "cleaned_yaml": str(self.output_yaml),
                "source_preserved": True,
                "manual_operations": len(self.history),
                "changed_cells": changed,
            }, sort_keys=False), encoding="utf-8")
            image_temp.replace(output_image)
            yaml_temp.replace(self.output_yaml)
            audit_temp.replace(self.audit_file)
            response.success = True
            response.message = f"saved non-destructive 2D map: {self.output_yaml}"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response


def main():
    rclpy.init()
    node = OccupancyCleanupEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
