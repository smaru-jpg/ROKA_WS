"""Record a PK-3A centerline from map->base_footprint TF and save a YAML profile."""

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from roka_robot_description.workspace import workspace_path


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def point_distance(first, second, three_dimensional=False):
    dx = float(first["x"]) - float(second["x"])
    dy = float(first["y"]) - float(second["y"])
    dz = (
        float(first.get("z", 0.0)) - float(second.get("z", 0.0))
        if three_dimensional else 0.0
    )
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rpy_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch, yaw_from_quaternion(q)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class TrackPathRecorder(Node):
    def __init__(self):
        super().__init__("track_path_recorder")
        self.declare_parameter(
            "output_file",
            str(workspace_path("maps", "track_v1", "track_profile.yaml")),
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("sample_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("minimum_distance_m", 0.05)
        self.declare_parameter("minimum_yaw_change_rad", 0.05)
        self.declare_parameter("closure_distance_m", 0.30)
        self.declare_parameter("closure_heading_tolerance_deg", 60.0)
        self.declare_parameter("minimum_closed_loop_length_m", 2.0)
        self.declare_parameter("minimum_closed_loop_area_m2", 0.25)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("track_width_m", 0.9144)
        self.declare_parameter("default_speed_mps", 0.20)
        self.declare_parameter("minimum_curve_speed_mps", 0.10)
        self.declare_parameter("maximum_lateral_accel_mps2", 0.25)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("path_topic", "/track/recorded_path")
        self.declare_parameter("environment", "flat")

        self.output_file = Path(self.get_parameter("output_file").value).expanduser()
        self.map_frame = self.get_parameter("map_frame").value
        self.sample_frame = self.get_parameter("sample_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.minimum_distance = float(self.get_parameter("minimum_distance_m").value)
        self.minimum_yaw = float(self.get_parameter("minimum_yaw_change_rad").value)
        self.closure_distance = float(self.get_parameter("closure_distance_m").value)
        self.closure_heading_tolerance = math.radians(float(
            self.get_parameter("closure_heading_tolerance_deg").value
        ))
        self.minimum_closed_loop_length = float(
            self.get_parameter("minimum_closed_loop_length_m").value
        )
        self.minimum_closed_loop_area = float(
            self.get_parameter("minimum_closed_loop_area_m2").value
        )
        self.smoothing_window = max(1, int(self.get_parameter("smoothing_window").value))
        self.track_width = float(self.get_parameter("track_width_m").value)
        self.default_speed = float(self.get_parameter("default_speed_mps").value)
        self.minimum_speed = float(self.get_parameter("minimum_curve_speed_mps").value)
        self.max_lateral_accel = float(self.get_parameter("maximum_lateral_accel_mps2").value)
        self.environment = str(self.get_parameter("environment").value).strip().lower()
        rate = float(self.get_parameter("sample_rate_hz").value)
        if (
            rate <= 0.0
            or self.minimum_distance <= 0.0
            or self.minimum_closed_loop_length <= 0.0
            or self.closure_heading_tolerance <= 0.0
        ):
            raise ValueError("sample rate, distance, and loop length must be positive")
        if self.environment not in {"capture", "flat", "inline"}:
            raise ValueError("environment must be capture, flat or inline")

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.points = []
        self.recording = bool(self.get_parameter("auto_start").value)
        self.tf_available = None
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(
            PathMessage, str(self.get_parameter("path_topic").value), path_qos
        )
        self.create_service(Trigger, "~/start", self.start_callback)
        self.create_service(Trigger, "~/stop_and_save", self.stop_callback)
        self.create_service(Trigger, "~/reset", self.reset_callback)
        self.create_timer(1.0 / rate, self.sample)
        self.publish_path()
        self.get_logger().info(
            f"centerline recorder ready: {self.sample_frame}->{self.base_frame}, "
            f"output={self.output_file}"
        )

    def start_callback(self, _request, response):
        self.recording = True
        response.success = True
        response.message = f"recording started with {len(self.points)} existing points"
        return response

    def reset_callback(self, _request, response):
        self.recording = False
        self.points.clear()
        self.publish_path()
        response.success = True
        response.message = "recording cleared"
        return response

    def stop_callback(self, _request, response):
        self.recording = False
        try:
            closed, saved_count = self.save_profile()
            response.success = True
            response.message = (
                f"saved {saved_count} points to {self.output_file}; closed_loop={closed}"
            )
        except Exception as error:  # service must report file/geometry failures
            response.success = False
            response.message = str(error)
        return response

    def sample(self):
        if not self.recording:
            return
        try:
            transform = self.buffer.lookup_transform(
                self.sample_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException as error:
            if self.tf_available is not False:
                self.get_logger().warning(f"centerline TF unavailable: {error}")
            self.tf_available = False
            return
        if self.tf_available is False:
            self.get_logger().info("centerline TF recovered")
        self.tf_available = True
        t = transform.transform.translation
        roll, pitch, yaw = rpy_from_quaternion(transform.transform.rotation)
        point = {
            "x": float(t.x), "y": float(t.y), "z": float(t.z),
            "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw),
        }
        if self.points:
            previous = self.points[-1]
            distance = point_distance(
                point, previous, three_dimensional=self.environment != "flat"
            )
            yaw_delta = abs(wrap_angle(point["yaw"] - previous["yaw"]))
            if distance < self.minimum_distance and yaw_delta < self.minimum_yaw:
                return
        self.points.append(point)
        self.publish_path()

    def publish_path(self, points=None, frame_id=None):
        selected = self.points if points is None else points
        message = PathMessage()
        message.header.frame_id = frame_id or (
            self.sample_frame if points is None else self.map_frame
        )
        message.header.stamp = self.get_clock().now().to_msg()
        for point in selected:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            pose.pose.position.z = float(point.get("z", 0.0))
            quaternion = quaternion_from_rpy(
                float(point.get("roll", 0.0)),
                float(point.get("pitch", 0.0)),
                float(point.get("yaw", 0.0)),
            )
            pose.pose.orientation.x, pose.pose.orientation.y = quaternion[:2]
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion[2:]
            message.poses.append(pose)
        self.path_pub.publish(message)

    def smooth_positions(self, points, closed):
        if self.smoothing_window <= 1 or len(points) < self.smoothing_window:
            return [dict(point) for point in points]
        radius = self.smoothing_window // 2
        result = []
        for index, point in enumerate(points):
            if not closed and (index < radius or index >= len(points) - radius):
                result.append(dict(point))
                continue
            neighbors = [
                points[(index + offset) % len(points)]
                for offset in range(-radius, radius + 1)
            ]
            result.append({
                "x": sum(item["x"] for item in neighbors) / len(neighbors),
                "y": sum(item["y"] for item in neighbors) / len(neighbors),
                "z": sum(item.get("z", 0.0) for item in neighbors) / len(neighbors),
                "roll": point.get("roll", 0.0),
                "pitch": point.get("pitch", 0.0),
                "yaw": point["yaw"],
            })
        return result

    def add_heading_and_speed(self, points, closed):
        count = len(points)
        for index, point in enumerate(points):
            next_index = (index + 1) % count if closed else min(index + 1, count - 1)
            previous_index = (index - 1) % count if closed else max(index - 1, 0)
            before = points[previous_index]
            after = points[next_index]
            dx = after["x"] - before["x"]
            dy = after["y"] - before["y"]
            point["yaw"] = math.atan2(dy, dx) if math.hypot(dx, dy) > 1.0e-6 else point["yaw"]
        for index, point in enumerate(points):
            previous_index = (index - 1) % count if closed else max(index - 1, 0)
            previous = points[previous_index]
            distance = max(
                math.hypot(
                    point["x"] - previous["x"],
                    point["y"] - previous["y"],
                ),
                1.0e-3,
            )
            curvature = abs(wrap_angle(point["yaw"] - previous["yaw"])) / distance
            curve_speed = math.sqrt(self.max_lateral_accel / max(curvature, 1.0e-6))
            point["speed_mps"] = float(
                min(self.default_speed, max(self.minimum_speed, curve_speed))
            )

    def points_in_map(self):
        if self.sample_frame == self.map_frame:
            return [dict(point) for point in self.points]
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.sample_frame, rclpy.time.Time()
            )
        except TransformException as error:
            raise RuntimeError(
                f"final {self.map_frame}<-{self.sample_frame} transform unavailable: {error}"
            ) from error
        translation = transform.transform.translation
        rotation_yaw = yaw_from_quaternion(transform.transform.rotation)
        cosine, sine = math.cos(rotation_yaw), math.sin(rotation_yaw)
        return [
            {
                "x": float(translation.x) + cosine * point["x"] - sine * point["y"],
                "y": float(translation.y) + sine * point["x"] + cosine * point["y"],
                "z": float(translation.z) + point.get("z", 0.0),
                "roll": point.get("roll", 0.0),
                "pitch": point.get("pitch", 0.0),
                "yaw": wrap_angle(point["yaw"] + rotation_yaw),
            }
            for point in self.points
        ]

    def save_profile(self):
        if len(self.points) < 10:
            raise RuntimeError("at least 10 centerline samples are required")
        closure = math.hypot(
            self.points[-1]["x"] - self.points[0]["x"],
            self.points[-1]["y"] - self.points[0]["y"],
        )
        closure_heading_error = abs(wrap_angle(
            self.points[-1]["yaw"] - self.points[0]["yaw"]
        ))
        recorded_length = sum(
            point_distance(
                self.points[index], self.points[index - 1],
                three_dimensional=self.environment != "flat",
            )
            for index in range(1, len(self.points))
        )
        recorded_area = 0.5 * abs(sum(
            self.points[index]["x"] * self.points[(index + 1) % len(self.points)]["y"]
            - self.points[(index + 1) % len(self.points)]["x"] * self.points[index]["y"]
            for index in range(len(self.points))
        ))
        # A short out-and-back test often ends near its start but is not a
        # repeatable closed course. Avoid turning that into a zero-radius loop.
        closed = (
            recorded_length >= self.minimum_closed_loop_length
            and closure <= self.closure_distance
            and closure_heading_error <= self.closure_heading_tolerance
            and recorded_area >= self.minimum_closed_loop_area
        )
        map_points = self.points_in_map()
        points = self.smooth_positions(map_points, closed)
        self.add_heading_and_speed(points, closed)
        document = {
            "schema_version": 2,
            "frame_id": self.map_frame,
            "environment": self.environment,
            "closed_loop": closed,
            "autonomy_approved": False,
            "approval_reason": "raw_mapping_path_requires_traversability_annotation",
            "track_width_m": self.track_width,
            "default_speed_mps": self.default_speed,
            "recorded_path_length_m": recorded_length,
            "recorded_path_area_m2": recorded_area,
            "closure_distance_m": closure,
            "closure_heading_error_deg": math.degrees(closure_heading_error),
            "recording_frame": self.sample_frame,
            "map_alignment_method": (
                "direct_map_pose_samples"
                if self.sample_frame == self.map_frame
                else "single_final_map_from_odom_transform"
            ),
            "points": points,
        }
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.output_file.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        self.publish_path(points, self.map_frame)
        return closed, len(points)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPathRecorder()
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
