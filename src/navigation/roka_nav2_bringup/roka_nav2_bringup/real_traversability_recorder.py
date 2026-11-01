"""Record real post-mapping Xbox traversals without commanding the robot."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any

from roka_interfaces.msg import TerrainState, WheelContactState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from roka_robot_description.hardware_config import load_hardware_config
from roka_robot_description.workspace import workspace_path


TIRE_HALF_WIDTH_M = 0.0591
DEFAULT_EDGE_MARGIN_M = 0.05


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_rpy(q: Any) -> tuple[float, float, float]:
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


def signed_path_errors(
    x: float,
    y: float,
    yaw: float,
    point: dict[str, Any],
) -> tuple[float, float]:
    path_yaw = float(point.get("yaw", 0.0))
    dx = x - float(point["x"])
    dy = y - float(point["y"])
    lateral = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
    return lateral, wrap_angle(yaw - path_yaw)


def nominal_tire_clearance(
    track_width_m: float,
    wheel_track_m: float,
    lateral_error_m: float,
) -> float:
    """Conservative nominal clearance until mapped edge extraction is available."""
    return (
        0.5 * track_width_m
        - abs(lateral_error_m)
        - 0.5 * wheel_track_m
        - TIRE_HALF_WIDTH_M
    )


def profile_tire_clearance(
    point: dict[str, Any],
    lateral_error_m: float,
    track_width_m: float,
    wheel_track_m: float,
) -> tuple[float, str]:
    """Prefer a mapped path-to-edge measurement over nominal track width."""
    direct = point.get("tire_edge_clearance_m")
    if direct is not None and math.isfinite(float(direct)):
        return (
            float(direct) - abs(lateral_error_m),
            "mapped_profile_tire_edge",
        )
    for name in (
        "minimum_wheel_edge_m",
        "minimum_wheel_center_edge_m",
        "minimum_edge_m",
    ):
        centre_edge = point.get(name)
        if centre_edge is not None and math.isfinite(float(centre_edge)):
            return (
                float(centre_edge)
                - TIRE_HALF_WIDTH_M
                - abs(lateral_error_m),
                "mapped_profile_wheel_center_edge",
            )
    return (
        nominal_tire_clearance(
            track_width_m, wheel_track_m, lateral_error_m
        ),
        "nominal_track_width",
    )


class RealTraversabilityRecorder(Node):
    def __init__(self) -> None:
        super().__init__("real_traversability_recorder")
        self.declare_parameter("profile_file", "")
        self.declare_parameter(
            "output_directory",
            str(workspace_path("datasets", "real_traversability")),
        )
        self.declare_parameter("session_name", "")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("command_topic", "/cmd_vel_joy")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("terrain_topic", "/terrain/state")
        self.declare_parameter("contact_state_topic", "/terrain/wheel_contact")
        self.declare_parameter("enable_button_index", 4)
        self.declare_parameter("sensor_timeout_sec", 0.40)
        hardware = load_hardware_config()
        self.declare_parameter("wheel_track_m", hardware.physical_wheel_track_m)
        self.declare_parameter("stall_command_min_mps", 0.15)
        self.declare_parameter("stall_speed_ratio", 0.35)
        self.declare_parameter("stall_hold_sec", 0.80)
        self.declare_parameter("auto_record", True)

        profile_file = Path(
            str(self.get_parameter("profile_file").value)
        ).expanduser()
        if not profile_file.is_file():
            raise ValueError("profile_file must be the mapped real track profile")
        profile = yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {}
        self.environment = str(profile.get("environment", "unspecified"))
        self.points = list(profile.get("points", []))
        if len(self.points) < 10:
            raise ValueError("real track profile needs at least 10 points")
        self.track_width = float(profile.get("track_width_m", 0.9144))
        self.wheel_track = float(self.get_parameter("wheel_track_m").value)
        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self.session_name = str(self.get_parameter("session_name").value).strip()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.enable_button = int(self.get_parameter("enable_button_index").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_sec").value)
        self.stall_command_min = float(
            self.get_parameter("stall_command_min_mps").value
        )
        self.stall_speed_ratio = float(
            self.get_parameter("stall_speed_ratio").value
        )
        self.stall_hold = float(self.get_parameter("stall_hold_sec").value)
        rate = float(self.get_parameter("sample_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("sample_rate_hz must be positive")

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joy: Joy | None = None
        self.command: Twist | None = None
        self.odom: Odometry | None = None
        self.wheel_odom: Odometry | None = None
        self.imu: Imu | None = None
        self.terrain: TerrainState | None = None
        self.contact: WheelContactState | None = None
        self.motor_raw: dict[str, Float64MultiArray | None] = {
            "left": None,
            "right": None,
        }
        self.received: dict[str, rclpy.time.Time] = {}
        self.stall_since: rclpy.time.Time | None = None
        self.file = None
        self.writer = None
        self.csv_path: Path | None = None
        self.rows = 0
        self.enabled_rows = 0
        self.warning_rows = 0
        self.unsafe_rows = 0
        self.stalled_rows = 0
        self.edge_clearance_sources: set[str] = set()
        self.outcome = "unrated"
        self.tf_available: bool | None = None

        mapped_edge_points = sum(
            profile_tire_clearance(
                point, 0.0, self.track_width, self.wheel_track
            )[1] != "nominal_track_width"
            for point in self.points
        )
        if mapped_edge_points != len(self.points):
            self.get_logger().warning(
                "profile lacks mapped edge clearance at "
                f"{len(self.points) - mapped_edge_points}/{len(self.points)} "
                "points; those rows are collection-only and must not calibrate "
                "the real planner"
            )

        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._joy, 20
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("command_topic").value),
            self._command,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            30,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._wheel_odom,
            30,
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self._imu, 50
        )
        self.create_subscription(
            TerrainState,
            str(self.get_parameter("terrain_topic").value),
            self._terrain,
            20,
        )
        self.create_subscription(
            WheelContactState,
            str(self.get_parameter("contact_state_topic").value),
            self._contact,
            30,
        )
        self.create_subscription(
            Float64MultiArray,
            "/md/left/raw_state",
            lambda message: self._motor("left", message),
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            "/md/right/raw_state",
            lambda message: self._motor("right", message),
            20,
        )
        self.create_service(Trigger, "~/start", self._start)
        self.create_service(Trigger, "~/finish_success", self._finish_success)
        self.create_service(Trigger, "~/finish_failure", self._finish_failure)
        self.create_service(Trigger, "~/finish_incomplete", self._finish_incomplete)
        self.create_timer(1.0 / rate, self._sample)
        if bool(self.get_parameter("auto_record").value):
            self._open()

    @staticmethod
    def fields() -> list[str]:
        return [
            "wall_time_iso", "ros_time_sec", "environment", "enabled",
            "joy_axis_0", "joy_axis_1",
            "cmd_linear_x_mps", "cmd_angular_z_radps",
            "x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg",
            "relative_roll_deg", "relative_pitch_deg",
            "roll_rate_radps", "pitch_rate_radps",
            "forward_speed_mps", "lateral_speed_mps", "yaw_rate_radps",
            "path_index", "lateral_error_m", "heading_error_deg",
            "planned_cross_slope_deg", "planned_longitudinal_slope_deg",
            "actual_cross_slope_deg", "actual_longitudinal_slope_deg",
            "roughness_m", "step_height_m", "terrain_risk", "terrain_speed_scale",
            "tire_edge_clearance_m", "edge_clearance_source",
            "slip_ratio", "stalled_label", "safety_warning_label", "unsafe_label",
            "wheel_odom_speed_mps", "contact_state", "estimated_contact_count",
            "contact_confidence", "acceleration_norm_mps2",
            "left_mot1_rpm_raw", "left_mot2_rpm_raw",
            "left_mot1_current_a", "left_mot2_current_a",
            "right_mot1_rpm_raw", "right_mot2_rpm_raw",
            "right_mot1_current_a", "right_mot2_current_a",
            "body_odom_fresh", "wheel_odom_fresh", "imu_fresh",
            "terrain_fresh", "contact_fresh", "motor_raw_fresh",
        ]

    def _stamp(self, name: str) -> None:
        self.received[name] = self.get_clock().now()

    def _joy(self, message: Joy) -> None:
        self.joy = message
        self._stamp("joy")

    def _command(self, message: Twist) -> None:
        self.command = message
        self._stamp("command")

    def _odom(self, message: Odometry) -> None:
        self.odom = message
        self._stamp("odom")

    def _wheel_odom(self, message: Odometry) -> None:
        self.wheel_odom = message
        self._stamp("wheel_odom")

    def _imu(self, message: Imu) -> None:
        self.imu = message
        self._stamp("imu")

    def _terrain(self, message: TerrainState) -> None:
        self.terrain = message
        self._stamp("terrain")

    def _contact(self, message: WheelContactState) -> None:
        self.contact = message
        self._stamp("contact")

    def _motor(self, side: str, message: Float64MultiArray) -> None:
        self.motor_raw[side] = message
        self._stamp(f"motor_{side}")

    def _fresh(self, name: str) -> bool:
        stamp = self.received.get(name)
        return stamp is not None and (
            self.get_clock().now() - stamp
        ).nanoseconds * 1.0e-9 <= self.sensor_timeout

    def _open(self) -> None:
        if self.file is not None:
            return
        self.output_directory.mkdir(parents=True, exist_ok=True)
        suffix = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        name = self.session_name or f"real_traverse_{suffix}"
        candidate = self.output_directory / f"{name}.csv"
        counter = 2
        while candidate.exists():
            candidate = self.output_directory / f"{name}_{counter}.csv"
            counter += 1
        self.csv_path = candidate
        self.file = candidate.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields())
        self.writer.writeheader()
        self.file.flush()
        self.get_logger().info(f"recording real traversability: {candidate}")

    def _close(self, outcome: str) -> str:
        if self.file is None or self.csv_path is None:
            raise RuntimeError("not recording")
        self.outcome = outcome
        self.file.flush()
        self.file.close()
        limitations = [
            "operator outcome and sensor freshness must be reviewed before training",
        ]
        if "nominal_track_width" in self.edge_clearance_sources:
            limitations.insert(
                0,
                "nominal edge clearance must be replaced by mapped edge extraction",
            )
        summary = {
            "schema_version": 1,
            "source": "real_post_mapping_xbox_traversal",
            "environment": self.environment,
            "outcome": outcome,
            "csv_file": str(self.csv_path),
            "rows": self.rows,
            "enabled_rows": self.enabled_rows,
            "warning_rows": self.warning_rows,
            "unsafe_rows": self.unsafe_rows,
            "stalled_rows": self.stalled_rows,
            "edge_clearance_sources": sorted(self.edge_clearance_sources),
            "planner_calibration_ready": (
                bool(self.edge_clearance_sources)
                and "nominal_track_width" not in self.edge_clearance_sources
            ),
            "limitations": limitations,
        }
        summary_path = self.csv_path.with_name(f"{self.csv_path.stem}_summary.json")
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        path = str(self.csv_path)
        self.file = None
        self.writer = None
        return path

    def _finish(self, outcome: str, response: Any) -> Any:
        try:
            path = self._close(outcome)
            response.success = True
            response.message = path
        except RuntimeError as error:
            response.success = False
            response.message = str(error)
        return response

    def _start(self, _request: Any, response: Any) -> Any:
        if self.file is not None:
            response.success = False
            response.message = "already recording"
            return response
        self._open()
        response.success = True
        response.message = str(self.csv_path)
        return response

    def _finish_success(self, _request: Any, response: Any) -> Any:
        return self._finish("success", response)

    def _finish_failure(self, _request: Any, response: Any) -> Any:
        return self._finish("failure", response)

    def _finish_incomplete(self, _request: Any, response: Any) -> Any:
        return self._finish("incomplete", response)

    def _sample(self) -> None:
        if self.writer is None or self.file is None or self.odom is None:
            return
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException as error:
            if self.tf_available is not False:
                self.get_logger().warning(
                    f"real traversal TF unavailable: {error}"
                )
            self.tf_available = False
            return
        if self.tf_available is False:
            self.get_logger().info("real traversal TF recovered")
        self.tf_available = True
        translation = transform.transform.translation
        _, _, yaw = quaternion_to_rpy(transform.transform.rotation)
        x, y = float(translation.x), float(translation.y)
        index = min(
            range(len(self.points)),
            key=lambda value: (
                float(self.points[value]["x"]) - x
            ) ** 2 + (float(self.points[value]["y"]) - y) ** 2,
        )
        point = self.points[index]
        lateral_error, heading_error = signed_path_errors(x, y, yaw, point)
        imu_roll = imu_pitch = math.nan
        if self.imu is not None and self._fresh("imu"):
            imu_roll, imu_pitch, _ = quaternion_to_rpy(self.imu.orientation)
        command_linear = self.command.linear.x if self.command is not None else 0.0
        command_angular = self.command.angular.z if self.command is not None else 0.0
        forward = float(self.odom.twist.twist.linear.x)
        lateral = float(self.odom.twist.twist.linear.y)
        yaw_rate = float(self.odom.twist.twist.angular.z)
        wheel_speed = (
            float(self.wheel_odom.twist.twist.linear.x)
            if self.wheel_odom is not None and self._fresh("wheel_odom")
            else math.nan
        )
        slip_reference = (
            abs(wheel_speed)
            if math.isfinite(wheel_speed) and abs(wheel_speed) >= self.stall_command_min
            else abs(command_linear)
        )
        slip_ratio = (
            abs(forward) / max(slip_reference, 1.0e-3)
            if slip_reference >= self.stall_command_min else 1.0
        )
        stall_condition = (
            abs(command_linear) >= self.stall_command_min
            and slip_ratio < self.stall_speed_ratio
        )
        now = self.get_clock().now()
        if stall_condition:
            if self.stall_since is None:
                self.stall_since = now
        else:
            self.stall_since = None
        stalled = self.stall_since is not None and (
            now - self.stall_since
        ).nanoseconds * 1.0e-9 >= self.stall_hold
        terrain_fresh = self.terrain is not None and self._fresh("terrain")
        terrain_risk = float(self.terrain.risk) if terrain_fresh else math.nan
        cross = (
            float(self.terrain.lateral_slope_deg) if terrain_fresh else math.nan
        )
        longitudinal = (
            float(self.terrain.longitudinal_slope_deg)
            if terrain_fresh else math.nan
        )
        clearance, clearance_source = profile_tire_clearance(
            point,
            lateral_error,
            self.track_width,
            self.wheel_track,
        )
        self.edge_clearance_sources.add(clearance_source)
        axes = list(self.joy.axes) if self.joy is not None else []
        buttons = list(self.joy.buttons) if self.joy is not None else []
        enabled = (
            0 <= self.enable_button < len(buttons)
            and buttons[self.enable_button] != 0
        ) or abs(command_linear) > 1.0e-4 or abs(command_angular) > 1.0e-4
        roll_deg = math.degrees(imu_roll) if math.isfinite(imu_roll) else math.nan
        pitch_deg = math.degrees(imu_pitch) if math.isfinite(imu_pitch) else math.nan
        relative_roll_deg = (
            roll_deg - cross
            if math.isfinite(roll_deg) and math.isfinite(cross) else math.nan
        )
        relative_pitch_deg = (
            pitch_deg + longitudinal
            if math.isfinite(pitch_deg) and math.isfinite(longitudinal) else math.nan
        )
        contact_fresh = self.contact is not None and self._fresh("contact")
        motors_fresh = self._fresh("motor_left") and self._fresh("motor_right")
        left_raw = list(self.motor_raw["left"].data) if self.motor_raw["left"] else []
        right_raw = list(self.motor_raw["right"].data) if self.motor_raw["right"] else []

        def raw_value(values: list[float], index: int) -> float:
            return float(values[index]) if index < len(values) else math.nan
        unsafe = (
            clearance < 0.0
            or (math.isfinite(terrain_risk) and terrain_risk >= 1.0)
            or (math.isfinite(relative_roll_deg) and abs(relative_roll_deg) >= 41.0)
            or (math.isfinite(relative_pitch_deg) and abs(relative_pitch_deg) >= 35.5)
            or (math.isfinite(roll_deg) and abs(roll_deg) >= 85.0)
            or (math.isfinite(pitch_deg) and abs(pitch_deg) >= 65.0)
        )
        warning = (
            unsafe
            or clearance < DEFAULT_EDGE_MARGIN_M
            or (math.isfinite(terrain_risk) and terrain_risk >= 0.80)
            or stalled
        )
        self.writer.writerow({
            "wall_time_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "ros_time_sec": now.nanoseconds * 1.0e-9,
            "environment": self.environment,
            "enabled": int(enabled),
            "joy_axis_0": float(axes[0]) if len(axes) > 0 else 0.0,
            "joy_axis_1": float(axes[1]) if len(axes) > 1 else 0.0,
            "cmd_linear_x_mps": command_linear,
            "cmd_angular_z_radps": command_angular,
            "x_m": x, "y_m": y, "z_m": float(translation.z),
            "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": math.degrees(yaw),
            "relative_roll_deg": relative_roll_deg,
            "relative_pitch_deg": relative_pitch_deg,
            "roll_rate_radps": (
                float(self.imu.angular_velocity.x) if self.imu is not None else math.nan
            ),
            "pitch_rate_radps": (
                float(self.imu.angular_velocity.y) if self.imu is not None else math.nan
            ),
            "forward_speed_mps": forward, "lateral_speed_mps": lateral,
            "yaw_rate_radps": yaw_rate,
            "path_index": index,
            "lateral_error_m": lateral_error,
            "heading_error_deg": math.degrees(heading_error),
            "planned_cross_slope_deg": point.get("cross_slope_deg", math.nan),
            "planned_longitudinal_slope_deg": point.get("longitudinal_slope_deg", math.nan),
            "actual_cross_slope_deg": cross,
            "actual_longitudinal_slope_deg": longitudinal,
            "roughness_m": float(self.terrain.roughness_m) if terrain_fresh else math.nan,
            "step_height_m": float(self.terrain.step_height_m) if terrain_fresh else math.nan,
            "terrain_risk": terrain_risk,
            "terrain_speed_scale": float(self.terrain.speed_scale) if terrain_fresh else math.nan,
            "tire_edge_clearance_m": clearance,
            "edge_clearance_source": clearance_source,
            "slip_ratio": slip_ratio,
            "stalled_label": int(stalled),
            "safety_warning_label": int(warning),
            "unsafe_label": int(unsafe),
            "wheel_odom_speed_mps": wheel_speed,
            "contact_state": self.contact.state_name if contact_fresh else "UNKNOWN",
            "estimated_contact_count": (
                int(self.contact.estimated_contact_count) if contact_fresh else -1
            ),
            "contact_confidence": (
                float(self.contact.confidence) if contact_fresh else 0.0
            ),
            "acceleration_norm_mps2": (
                float(self.contact.acceleration_norm_mps2) if contact_fresh else math.nan
            ),
            "left_mot1_rpm_raw": raw_value(left_raw, 0),
            "left_mot2_rpm_raw": raw_value(left_raw, 1),
            "left_mot1_current_a": raw_value(left_raw, 2),
            "left_mot2_current_a": raw_value(left_raw, 3),
            "right_mot1_rpm_raw": raw_value(right_raw, 0),
            "right_mot2_rpm_raw": raw_value(right_raw, 1),
            "right_mot1_current_a": raw_value(right_raw, 2),
            "right_mot2_current_a": raw_value(right_raw, 3),
            "body_odom_fresh": int(self._fresh("odom")),
            "wheel_odom_fresh": int(self._fresh("wheel_odom")),
            "imu_fresh": int(self._fresh("imu")),
            "terrain_fresh": int(terrain_fresh),
            "contact_fresh": int(contact_fresh),
            "motor_raw_fresh": int(motors_fresh),
        })
        self.rows += 1
        self.enabled_rows += int(enabled)
        self.warning_rows += int(warning)
        self.unsafe_rows += int(unsafe)
        self.stalled_rows += int(stalled)
        if self.rows % 20 == 0:
            self.file.flush()

    def destroy_node(self) -> bool:
        if self.file is not None:
            self._close(self.outcome)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RealTraversabilityRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
