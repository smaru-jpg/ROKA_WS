"""Record Xbox MuJoCo demonstrations with terrain-relative state features."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger
import yaml

from roka_robot_description.workspace import workspace_path

from .safe_path import (
    DEFAULT_EDGE_MARGIN_M,
    TIRE_HALF_WIDTH_M,
    GroundTruthGrid,
    _candidate_metrics,
)
from .safe_path_controller import quaternion_to_rpy, wrap_angle


WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def signed_path_errors(
    x: float,
    y: float,
    yaw: float,
    path_x: float,
    path_y: float,
    path_yaw: float,
) -> tuple[float, float]:
    """Return left-positive lateral and heading error in the path frame."""
    dx = x - path_x
    dy = y - path_y
    lateral = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
    return lateral, wrap_angle(yaw - path_yaw)


class DemonstrationRecorder(Node):
    def __init__(self) -> None:
        super().__init__("sim_demonstration_recorder")
        self.declare_parameter(
            "output_directory",
            str(workspace_path("datasets", "xbox_demonstrations")),
        )
        self.declare_parameter("session_name", "")
        self.declare_parameter("sample_rate_hz", 30.0)
        self.declare_parameter("auto_record", True)
        self.declare_parameter("enable_button_index", 4)
        self.declare_parameter("joy_topic", "/sim/joy")
        self.declare_parameter("command_topic", "/sim/cmd_vel_joy")
        self.declare_parameter("odom_topic", "/simulator/floating_base_state")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "profile_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_safe_path.yaml")),
        )
        self.declare_parameter(
            "ground_truth_map_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.npz")),
        )
        self.declare_parameter(
            "ground_truth_metadata_file",
            str(workspace_path("maps", "sim_pk3a_ground_truth", "pk3a_ground_truth.json")),
        )

        rate = float(self.get_parameter("sample_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser().resolve()
        self.session_name = str(self.get_parameter("session_name").value).strip()
        self.enable_button = int(self.get_parameter("enable_button_index").value)
        profile_path = Path(str(self.get_parameter("profile_file").value)).expanduser()
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        self.points = list(document.get("points", []))
        if not document.get("closed_loop") or not self.points:
            raise ValueError("demonstration profile must contain a closed path")
        self.path_xy = np.asarray(
            [[float(point["x"]), float(point["y"])] for point in self.points],
            dtype=float,
        )
        self.grid = GroundTruthGrid(
            Path(str(self.get_parameter("ground_truth_map_file").value)),
            Path(str(self.get_parameter("ground_truth_metadata_file").value)),
        )

        self.joy: Joy | None = None
        self.command: Twist | None = None
        self.odom: Odometry | None = None
        self.joints: JointState | None = None
        self.file = None
        self.writer = None
        self.csv_path: Path | None = None
        self.row_count = 0
        self.enabled_count = 0
        self.command_count = 0
        self.unsafe_count = 0
        self.maximum_abs_roll_deg = 0.0
        self.maximum_abs_pitch_deg = 0.0
        self.minimum_edge_m = math.inf
        self.minimum_tire_clearance_m = math.inf

        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._joy_callback, 20
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("command_topic").value),
            self._command_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            50,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_callback,
            20,
        )
        self.create_service(Trigger, "~/start", self._start_service)
        self.create_service(Trigger, "~/stop", self._stop_service)
        self.timer = self.create_timer(1.0 / rate, self._sample)
        if bool(self.get_parameter("auto_record").value):
            self._open_session()

    @staticmethod
    def _fields() -> list[str]:
        fields = [
            "wall_time_iso",
            "sim_time_sec",
            "enabled",
            "joy_axis_0",
            "joy_axis_1",
            "cmd_linear_x_mps",
            "cmd_angular_z_radps",
            "x_m",
            "y_m",
            "z_m",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "forward_speed_mps",
            "lateral_speed_mps",
            "vertical_speed_mps",
            "roll_rate_radps",
            "pitch_rate_radps",
            "yaw_rate_radps",
            "path_index",
            "module_id",
            "lateral_error_m",
            "heading_error_deg",
            "planned_lateral_offset_m",
            "planned_cross_slope_deg",
            "planned_longitudinal_slope_deg",
            "planned_minimum_edge_m",
            "actual_rideable",
            "actual_cross_slope_deg",
            "actual_longitudinal_slope_deg",
            "actual_minimum_edge_m",
            "actual_wheel_center_edge_m",
            "tire_edge_clearance_m",
            "safety_margin_clearance_m",
            "safety_warning_label",
            "unsafe_label",
        ]
        for joint in WHEEL_JOINTS:
            fields.extend((f"{joint}_position_rad", f"{joint}_velocity_radps"))
        return fields

    def _open_session(self) -> None:
        if self.file is not None:
            return
        self.output_directory.mkdir(parents=True, exist_ok=True)
        suffix = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        name = self.session_name or f"xbox_demo_{suffix}"
        candidate = self.output_directory / f"{name}.csv"
        counter = 2
        while candidate.exists():
            candidate = self.output_directory / f"{name}_{counter}.csv"
            counter += 1
        self.csv_path = candidate
        self.file = candidate.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self._fields())
        self.writer.writeheader()
        self.file.flush()
        self.get_logger().info(f"recording Xbox demonstration: {candidate}")

    def _finish_session(self) -> None:
        if self.file is None or self.csv_path is None:
            return
        self.file.flush()
        self.file.close()
        summary = {
            "schema_version": 2,
            "csv_file": str(self.csv_path),
            "rows": self.row_count,
            "enabled_rows": self.enabled_count,
            "nonzero_command_rows": self.command_count,
            "unsafe_rows": self.unsafe_count,
            "maximum_abs_roll_deg": self.maximum_abs_roll_deg,
            "maximum_abs_pitch_deg": self.maximum_abs_pitch_deg,
            "minimum_actual_edge_m": (
                None if not math.isfinite(self.minimum_edge_m) else self.minimum_edge_m
            ),
            "minimum_tire_edge_clearance_m": (
                None
                if not math.isfinite(self.minimum_tire_clearance_m)
                else self.minimum_tire_clearance_m
            ),
            "tire_half_width_m": TIRE_HALF_WIDTH_M,
            "edge_safety_margin_m": DEFAULT_EDGE_MARGIN_M,
        }
        summary_path = self.csv_path.with_name(f"{self.csv_path.stem}_summary.json")
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(
            f"demonstration saved: {self.row_count} rows, "
            f"{self.enabled_count} enabled, {self.unsafe_count} unsafe"
        )
        self.file = None
        self.writer = None

    def _start_service(self, _request: object, response: object) -> object:
        if self.file is not None:
            response.success = False
            response.message = "already recording"
            return response
        self._open_session()
        response.success = True
        response.message = str(self.csv_path)
        return response

    def _stop_service(self, _request: object, response: object) -> object:
        if self.file is None:
            response.success = False
            response.message = "not recording"
            return response
        path = str(self.csv_path)
        self._finish_session()
        response.success = True
        response.message = path
        return response

    def _joy_callback(self, message: Joy) -> None:
        self.joy = message

    def _command_callback(self, message: Twist) -> None:
        self.command = message

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def _joint_callback(self, message: JointState) -> None:
        self.joints = message

    def _joint_values(self) -> dict[str, float]:
        values: dict[str, float] = {}
        if self.joints is None:
            for name in WHEEL_JOINTS:
                values[f"{name}_position_rad"] = math.nan
                values[f"{name}_velocity_radps"] = math.nan
            return values
        indices = {name: index for index, name in enumerate(self.joints.name)}
        for name in WHEEL_JOINTS:
            index = indices.get(name)
            values[f"{name}_position_rad"] = (
                float(self.joints.position[index])
                if index is not None and index < len(self.joints.position)
                else math.nan
            )
            values[f"{name}_velocity_radps"] = (
                float(self.joints.velocity[index])
                if index is not None and index < len(self.joints.velocity)
                else math.nan
            )
        return values

    def _sample(self) -> None:
        if self.writer is None or self.file is None or self.odom is None:
            return
        position = self.odom.pose.pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.odom.pose.pose.orientation)
        x = float(position.x)
        y = float(position.y)
        differences = self.path_xy - np.array([x, y])
        distances = np.einsum("ij,ij->i", differences, differences)
        path_index = int(np.argmin(distances))
        point = self.points[path_index]
        lateral_error, heading_error = signed_path_errors(
            x,
            y,
            yaw,
            float(point["x"]),
            float(point["y"]),
            float(point["yaw"]),
        )
        terrain = _candidate_metrics(self.grid, x, y, yaw, 0.0)
        actual_rideable = terrain is not None
        actual_edge = (
            float(terrain["minimum_wheel_edge_m"]) if terrain is not None else math.nan
        )
        if math.isfinite(actual_edge):
            self.minimum_edge_m = min(self.minimum_edge_m, actual_edge)
        tire_clearance = actual_edge - TIRE_HALF_WIDTH_M
        safety_clearance = tire_clearance - DEFAULT_EDGE_MARGIN_M
        if math.isfinite(tire_clearance):
            self.minimum_tire_clearance_m = min(
                self.minimum_tire_clearance_m, tire_clearance
            )
        velocity = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        forward_speed = math.cos(yaw) * float(velocity.x) + math.sin(yaw) * float(velocity.y)
        lateral_speed = -math.sin(yaw) * float(velocity.x) + math.cos(yaw) * float(velocity.y)
        axes = list(self.joy.axes) if self.joy is not None else []
        buttons = list(self.joy.buttons) if self.joy is not None else []
        enabled = int(
            0 <= self.enable_button < len(buttons) and buttons[self.enable_button] != 0
        )
        command_linear = float(self.command.linear.x) if self.command is not None else 0.0
        command_angular = float(self.command.angular.z) if self.command is not None else 0.0
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        unsafe = int(
            not actual_rideable
            or (math.isfinite(tire_clearance) and tire_clearance < 0.0)
            or abs(roll_deg) >= 85.0
            or abs(pitch_deg) >= 65.0
        )
        safety_warning = int(
            not actual_rideable
            or (math.isfinite(safety_clearance) and safety_clearance < 0.0)
        )
        row: dict[str, Any] = {
            "wall_time_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "sim_time_sec": self.get_clock().now().nanoseconds * 1.0e-9,
            "enabled": enabled,
            "joy_axis_0": float(axes[0]) if len(axes) > 0 else 0.0,
            "joy_axis_1": float(axes[1]) if len(axes) > 1 else 0.0,
            "cmd_linear_x_mps": command_linear,
            "cmd_angular_z_radps": command_angular,
            "x_m": x,
            "y_m": y,
            "z_m": float(position.z),
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": math.degrees(yaw),
            "forward_speed_mps": forward_speed,
            "lateral_speed_mps": lateral_speed,
            "vertical_speed_mps": float(velocity.z),
            "roll_rate_radps": float(angular.x),
            "pitch_rate_radps": float(angular.y),
            "yaw_rate_radps": float(angular.z),
            "path_index": path_index,
            "module_id": str(point.get("module_id", "")),
            "lateral_error_m": lateral_error,
            "heading_error_deg": math.degrees(heading_error),
            "planned_lateral_offset_m": float(point.get("lateral_offset_m", 0.0)),
            "planned_cross_slope_deg": float(point.get("cross_slope_deg", math.nan)),
            "planned_longitudinal_slope_deg": float(
                point.get("longitudinal_slope_deg", math.nan)
            ),
            "planned_minimum_edge_m": float(
                point.get("minimum_wheel_edge_m", math.nan)
            ),
            "actual_rideable": int(actual_rideable),
            "actual_cross_slope_deg": (
                float(terrain["cross_slope_deg"]) if terrain is not None else math.nan
            ),
            "actual_longitudinal_slope_deg": (
                float(terrain["longitudinal_slope_deg"])
                if terrain is not None
                else math.nan
            ),
            "actual_minimum_edge_m": actual_edge,
            "actual_wheel_center_edge_m": actual_edge,
            "tire_edge_clearance_m": tire_clearance,
            "safety_margin_clearance_m": safety_clearance,
            "safety_warning_label": safety_warning,
            "unsafe_label": unsafe,
            **self._joint_values(),
        }
        self.writer.writerow(row)
        self.row_count += 1
        self.enabled_count += enabled
        self.command_count += int(abs(command_linear) > 1.0e-4 or abs(command_angular) > 1.0e-4)
        self.unsafe_count += unsafe
        self.maximum_abs_roll_deg = max(self.maximum_abs_roll_deg, abs(roll_deg))
        self.maximum_abs_pitch_deg = max(self.maximum_abs_pitch_deg, abs(pitch_deg))
        if self.row_count % 30 == 0:
            self.file.flush()

    def destroy_node(self) -> bool:
        self._finish_session()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DemonstrationRecorder()
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
