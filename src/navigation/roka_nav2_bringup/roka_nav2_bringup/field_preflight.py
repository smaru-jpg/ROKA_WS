"""Print one concise startup health table for the running field stack."""

from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, Joy, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray
from tf2_ros import Buffer, TransformListener
from velodyne_msgs.msg import VelodyneScan


def measured_rate(count: int, duration_sec: float) -> float:
    return float(count) / max(float(duration_sec), 1.0e-6)


def sampled_rate(count: int, duration_sec: float) -> float:
    """Return interval rate without treating the first sample as a period."""
    if count < 2 or duration_sec <= 0.0:
        return 0.0
    return measured_rate(count - 1, duration_sec)


def finite_vector(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def stamp_age_seconds(now_nanoseconds: int, stamp) -> float:
    stamp_nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if stamp_nanoseconds <= 0:
        return math.inf
    return (int(now_nanoseconds) - stamp_nanoseconds) / 1.0e9


def point_time_bounds_valid(minimum: float, maximum: float) -> bool:
    """Validate VLP-16 offsets expressed in seconds from scan start."""
    return (
        math.isfinite(minimum)
        and math.isfinite(maximum)
        and minimum >= -0.002
        and maximum > 0.0
        and maximum <= 0.2
        and maximum >= minimum
    )


def point_time_bounds(message: PointCloud2):
    """Return finite PointCloud2 `time` bounds without copying XYZ fields."""
    try:
        values = point_cloud2.read_points(
            message, field_names=["time"], skip_nans=False
        )["time"]
    except (AssertionError, KeyError, ValueError):
        return None
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return None
    return float(np.min(finite)), float(np.max(finite))


def reset_stamp_error_window(errors: dict[str, int]) -> None:
    """Start a fresh timestamp-health window while retaining topic history.

    A driver/converter can deliver a few queued samples out of order while the
    ROS graph is still forming.  The first preflight attempt must reject those
    samples, but a clean retry window must be allowed to recover.  Keeping the
    last stamp still detects a clock reset at the boundary of the new window.
    """
    for name in errors:
        errors[name] = 0


class FieldPreflight(Node):
    """Observe the mapping nodes already using the hardware."""

    def __init__(self):
        super().__init__("pk3a_field_preflight")
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("minimum_imu_rate_hz", 80.0)
        self.declare_parameter("minimum_packet_rate_hz", 8.0)
        # After restoring the ROS 1 unorganized-cloud behavior and a newest-
        # only ROS 2 input queue, bag replay measures about 10 Hz. A lower rate
        # is treated as a conversion backlog before autonomous motion starts.
        self.declare_parameter("minimum_cloud_rate_hz", 5.0)
        self.declare_parameter("minimum_scan_rate_hz", 5.0)
        self.declare_parameter("maximum_lidar_message_age_sec", 0.40)
        self.declare_parameter("minimum_joy_rate_hz", 5.0)
        self.declare_parameter("minimum_md_rate_hz", 2.0)
        self.declare_parameter("minimum_rate_observation_sec", 2.0)
        self.declare_parameter("failed_check_retry_sec", 5.0)
        self.declare_parameter("retry_until_ready", False)
        self.declare_parameter("require_xbox", True)
        self.declare_parameter("require_2d_scan", True)
        self.declare_parameter("require_arm_hold", False)
        # Keep this synchronized with roka_md_driver abort_current_a.
        # MD200T is rated 10 A/channel; 9.5 A is the requested field guard.
        self.declare_parameter("maximum_motor_current_a", 9.5)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("lidar_frame", "velodyne")
        self.declare_parameter("left_serial_port", "")
        self.declare_parameter("right_serial_port", "")
        self.declare_parameter("require_lio_stability", False)
        self.declare_parameter("lio_stability_warmup_sec", 2.0)
        self.declare_parameter("minimum_lio_rate_hz", 5.0)
        self.declare_parameter("maximum_lio_stationary_drift_m", 0.10)
        self.declare_parameter("lio_stability_window_sec", 3.0)

        self.duration = float(self.get_parameter("duration_sec").value)
        if self.duration < 2.0:
            raise ValueError("duration_sec must be at least 2 seconds")
        self.minimum_rates = {
            "IMU": float(self.get_parameter("minimum_imu_rate_hz").value),
            "VLP packets": float(self.get_parameter("minimum_packet_rate_hz").value),
            "VLP cloud": float(self.get_parameter("minimum_cloud_rate_hz").value),
            "2D scan": float(self.get_parameter("minimum_scan_rate_hz").value),
            "Xbox": float(self.get_parameter("minimum_joy_rate_hz").value),
            "MD left": float(self.get_parameter("minimum_md_rate_hz").value),
            "MD right": float(self.get_parameter("minimum_md_rate_hz").value),
        }
        self.maximum_motor_current = float(
            self.get_parameter("maximum_motor_current_a").value
        )
        self.maximum_lidar_age = float(
            self.get_parameter("maximum_lidar_message_age_sec").value
        )
        self.minimum_rate_observation = float(
            self.get_parameter("minimum_rate_observation_sec").value
        )
        self.failed_check_retry = float(
            self.get_parameter("failed_check_retry_sec").value
        )
        self.retry_until_ready = bool(
            self.get_parameter("retry_until_ready").value
        )
        self.require_xbox = bool(self.get_parameter("require_xbox").value)
        self.require_2d_scan = bool(self.get_parameter("require_2d_scan").value)
        self.require_arm_hold = bool(
            self.get_parameter("require_arm_hold").value
        )
        if self.minimum_rate_observation < 1.0:
            raise ValueError("minimum_rate_observation_sec must be at least 1 second")
        if self.failed_check_retry < 1.0:
            raise ValueError("failed_check_retry_sec must be at least 1 second")
        if self.maximum_lidar_age <= 0.0:
            raise ValueError("maximum_lidar_message_age_sec must be positive")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.motor_ports = {
            "left": str(self.get_parameter("left_serial_port").value),
            "right": str(self.get_parameter("right_serial_port").value),
        }
        self.require_lio_stability = bool(
            self.get_parameter("require_lio_stability").value
        )
        self.lio_stability_warmup = float(
            self.get_parameter("lio_stability_warmup_sec").value
        )
        self.minimum_lio_rate = float(
            self.get_parameter("minimum_lio_rate_hz").value
        )
        self.maximum_lio_drift = float(
            self.get_parameter("maximum_lio_stationary_drift_m").value
        )
        self.lio_stability_window = float(
            self.get_parameter("lio_stability_window_sec").value
        )
        if self.lio_stability_warmup < 0.0:
            raise ValueError("lio_stability_warmup_sec cannot be negative")
        if self.minimum_lio_rate <= 0.0:
            raise ValueError("minimum_lio_rate_hz must be positive")
        if self.maximum_lio_drift <= 0.0:
            raise ValueError("maximum_lio_stationary_drift_m must be positive")
        if self.lio_stability_window < self.minimum_rate_observation:
            raise ValueError(
                "lio_stability_window_sec must cover minimum_rate_observation_sec"
            )
        self.started = time.monotonic()
        self.counts = {name: 0 for name in self.minimum_rates}
        self.first_seen = {name: None for name in self.minimum_rates}
        self.next_check_elapsed = self.duration
        self.check_attempts = 0
        self.imu_connected = False
        self.arm_hold_ready = False
        self.arm_hold_interlock_tripped = False
        self.imu_valid = False
        self.imu_acceleration_norm = math.nan
        self.imu_quaternion_norm = math.nan
        self.imu_covariance_valid = False
        self.cloud_valid = False
        self.cloud_fields_valid = False
        self.cloud_point_time_valid = False
        self.cloud_point_time_bounds = None
        self.scan_valid = False
        self.cloud_stamp = None
        self.scan_stamp = None
        self.last_sensor_stamp = {}
        self.backwards_sensor_stamps = {
            "IMU": 0,
            "VLP packets": 0,
            "VLP cloud": 0,
            "2D scan": 0,
        }
        self.joy_valid = False
        self.md_connected_seen = False
        self.md_connected = False
        self.md_raw = {"left": None, "right": None}
        self.lio_samples = []
        self.completed = False
        self.ready = False
        self.attempt_lines = []
        self.last_attempt_signature = None
        self.last_unchanged_notice = 0.0

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.ready_pub = self.create_publisher(
            Bool, "/field/preflight_ready", status_qos
        )
        # Do not publish a provisional failure.  Consumers keep readiness as
        # unknown (and therefore motion-start locked) until the timed check has
        # produced its one authoritative PASS/FAIL result.

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Bool, "/imu/connected", self._imu_connection, 10)
        self.create_subscription(
            Bool, "/roka/arm/hold_ready", self._arm_hold, status_qos
        )
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(
            VelodyneScan, "/velodyne_packets", self._packets, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/velodyne_points", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan_stabilized", self._scan, qos_profile_sensor_data
        )
        self.create_subscription(Joy, "/joy", self._joy, qos_profile_sensor_data)
        self.create_subscription(Bool, "/md/base_connected", self._md_connection, 10)
        self.create_subscription(
            Float64MultiArray,
            "/md/left/raw_state",
            lambda message: self._md_state("left", message),
            10,
        )
        self.create_subscription(
            Odometry, "/lio/odom", self._lio_odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            Float64MultiArray,
            "/md/right/raw_state",
            lambda message: self._md_state("right", message),
            10,
        )
        self.create_timer(0.10, self._tick)
        print(
            f"[FIELD-CHECK] {self.duration:.0f}s startup check: "
            f"{'Xbox / ' if self.require_xbox else ''}"
            "EBIMU / VLP-16 / MD-left / MD-right"
            f"{' / Arm-hold' if self.require_arm_hold else ''}",
            flush=True,
        )

    def _imu_connection(self, message: Bool) -> None:
        self.imu_connected = bool(message.data)

    def _arm_hold(self, message: Bool) -> None:
        was_ready = self.arm_hold_ready
        self.arm_hold_ready = bool(message.data)
        # Arm motion during transport-pose acquisition can rock the chassis and
        # poison an IMU/LIO startup sample.  Begin the stationary evidence
        # window only after that motion has completed.
        if self.arm_hold_ready and not was_ready:
            self.lio_samples.clear()
        if not self.require_arm_hold or not self.completed:
            return
        if not self.arm_hold_ready and self.ready:
            self.arm_hold_interlock_tripped = True
            self.ready = False
            self.publish_ready(False)
            print(
                "[FIELD-CHECK][STOP] Arm hold lost; base motion permission revoked",
                flush=True,
            )
        elif self.arm_hold_ready and self.arm_hold_interlock_tripped:
            self.arm_hold_interlock_tripped = False
            self.ready = True
            self.publish_ready(True)
            print(
                "[FIELD-CHECK][RECOVER] Arm hold restored; base motion permission restored",
                flush=True,
            )

    def _record(self, name: str) -> None:
        if self.first_seen[name] is None:
            self.first_seen[name] = time.monotonic()
        self.counts[name] += 1

    def _record_stamp(self, name: str, stamp) -> None:
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        previous = self.last_sensor_stamp.get(name)
        if value <= 0 or (previous is not None and value < previous):
            self.backwards_sensor_stamps[name] += 1
        self.last_sensor_stamp[name] = value

    def _imu(self, message: Imu) -> None:
        self._record("IMU")
        self._record_stamp("IMU", message.header.stamp)
        accel = message.linear_acceleration
        gyro = message.angular_velocity
        values = (accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z)
        acceleration_norm = math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2)
        orientation = message.orientation
        quaternion_values = (
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion_values))
        covariance_valid = (
            message.orientation_covariance[0] >= 0.0
            and any(abs(value) > 0.0 for value in message.orientation_covariance)
            and any(abs(value) > 0.0 for value in message.angular_velocity_covariance)
            and any(abs(value) > 0.0 for value in message.linear_acceleration_covariance)
        )
        self.imu_acceleration_norm = acceleration_norm
        self.imu_quaternion_norm = quaternion_norm
        self.imu_covariance_valid = covariance_valid
        # This check runs before motion. A stationary 1 g norm, normalized
        # quaternion and usable covariance are stronger evidence than merely
        # receiving six finite numbers from the serial parser.
        self.imu_valid = (
            finite_vector(values + quaternion_values)
            and 5.0 < acceleration_norm < 15.0
            and 0.95 < quaternion_norm < 1.05
            and covariance_valid
        )

    def _packets(self, message: VelodyneScan) -> None:
        self._record("VLP packets")
        self._record_stamp("VLP packets", message.header.stamp)

    def _cloud(self, message: PointCloud2) -> None:
        self._record("VLP cloud")
        self._record_stamp("VLP cloud", message.header.stamp)
        self.cloud_stamp = message.header.stamp
        self.cloud_valid = (
            message.width * message.height > 0
            and message.point_step > 0
            and len(message.data) >= message.width * message.height * message.point_step
        )
        fields = {field.name: field.datatype for field in message.fields}
        required = {
            "x": PointField.FLOAT32,
            "y": PointField.FLOAT32,
            "z": PointField.FLOAT32,
            "intensity": PointField.FLOAT32,
            "ring": PointField.UINT16,
            "time": PointField.FLOAT32,
        }
        self.cloud_fields_valid = all(
            fields.get(name) == datatype for name, datatype in required.items()
        )
        self.cloud_point_time_bounds = (
            point_time_bounds(message) if self.cloud_fields_valid else None
        )
        self.cloud_point_time_valid = (
            self.cloud_point_time_bounds is not None
            and point_time_bounds_valid(*self.cloud_point_time_bounds)
        )

    def _scan(self, message: LaserScan) -> None:
        self._record("2D scan")
        self._record_stamp("2D scan", message.header.stamp)
        self.scan_stamp = message.header.stamp
        self.scan_valid = bool(message.ranges) and any(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )

    def _joy(self, message: Joy) -> None:
        self._record("Xbox")
        self.joy_valid = len(message.axes) >= 2 and len(message.buttons) >= 5

    def _md_connection(self, message: Bool) -> None:
        self.md_connected_seen = True
        self.md_connected = bool(message.data)

    def _md_state(self, side: str, message: Float64MultiArray) -> None:
        key = f"MD {side}"
        self._record(key)
        self.md_raw[side] = tuple(float(value) for value in message.data)

    def _lio_odometry(self, message: Odometry) -> None:
        received = time.monotonic()
        if received - self.started < self.lio_stability_warmup:
            return
        position = message.pose.pose.position
        values = (float(position.x), float(position.y), float(position.z))
        if finite_vector(values):
            self.lio_samples.append((received, values))
            oldest = received - self.lio_stability_window
            self.lio_samples = [
                sample for sample in self.lio_samples if sample[0] >= oldest
            ]

    def _tf_ready(self, child: str) -> bool:
        return self.tf_buffer.can_transform(self.base_frame, child, rclpy.time.Time())

    def _line(self, state: str, label: str, detail: str) -> None:
        self.attempt_lines.append(
            f"[FIELD-CHECK][{state}] {label}: {detail}"
        )

    def _flush_attempt(self) -> None:
        signature = tuple(
            line for line in self.attempt_lines if "[RETRY]" not in line
        )
        now = time.monotonic()
        if signature != self.last_attempt_signature:
            for line in self.attempt_lines:
                print(line, flush=True)
            self.last_attempt_signature = signature
            self.last_unchanged_notice = now
        elif now - self.last_unchanged_notice >= 60.0:
            print(
                "[FIELD-CHECK][WAIT] hardware state unchanged; continuing quiet rechecks",
                flush=True,
            )
            self.last_unchanged_notice = now

    def publish_ready(self, ready: bool) -> None:
        message = Bool()
        message.data = bool(ready)
        self.ready_pub.publish(message)

    def _rate(self, name: str, _elapsed: float) -> float:
        first_seen = self.first_seen[name]
        if first_seen is None:
            return 0.0
        observation = time.monotonic() - first_seen
        if observation < self.minimum_rate_observation:
            return 0.0
        return sampled_rate(self.counts[name], observation)

    def _motor_result(self, side: str, elapsed: float) -> bool:
        name = f"MD {side}"
        rate = self._rate(name, elapsed)
        raw = self.md_raw[side]
        port = self.motor_ports[side]
        if raw is None:
            adapter = bool(port) and Path(port).exists()
            state = "INVALID" if adapter else "MISSING"
            detail = (
                f"adapter exists but no MD200T feedback ({port})"
                if adapter
                else f"serial adapter not found ({port or 'unset'})"
            )
            self._line(state, name, detail)
            return False
        if len(raw) < 6 or not finite_vector(raw[:6]):
            self._line("INVALID", name, "malformed motor feedback")
            return False
        states = (int(raw[4]), int(raw[5]))
        currents = (raw[2], raw[3])
        passed = (
            states == (0, 0)
            and max(currents) <= self.maximum_motor_current
            and rate >= self.minimum_rates[name]
            and self.md_connected_seen
            and self.md_connected
        )
        self._line(
            "PASS" if passed else "INVALID",
            name,
            f"{rate:.1f} Hz, rpm=({raw[0]:.0f},{raw[1]:.0f}), "
            f"current=({currents[0]:.1f},{currents[1]:.1f}) A, state={states}",
        )
        return passed

    def _tick(self) -> None:
        if self.completed:
            return
        elapsed = time.monotonic() - self.started
        if elapsed < self.next_check_elapsed:
            return
        self.check_attempts += 1
        self.attempt_lines = []
        results = []

        joy_rate = self._rate("Xbox", elapsed)
        if not self.require_xbox:
            self._line("SKIP", "Xbox", "not required for automatic autonomy")
        elif self.counts["Xbox"] == 0:
            self._line("MISSING", "Xbox", "no /joy data")
            results.append(False)
        else:
            passed = joy_rate >= self.minimum_rates["Xbox"] and self.joy_valid
            self._line(
                "PASS" if passed else "INVALID",
                "Xbox",
                f"{joy_rate:.1f} Hz, layout={'valid' if self.joy_valid else 'invalid'}",
            )
            results.append(passed)

        imu_rate = self._rate("IMU", elapsed)
        if self.counts["IMU"] == 0:
            self._line("MISSING", "EBIMU", "no /imu/data")
            results.append(False)
        else:
            passed = (
                imu_rate >= self.minimum_rates["IMU"]
                and self.imu_connected
                and self.imu_valid
                and self.backwards_sensor_stamps["IMU"] == 0
            )
            self._line(
                "PASS" if passed else "INVALID",
                "EBIMU",
                f"{imu_rate:.1f} Hz, connected={self.imu_connected}, "
                f"accel_norm={self.imu_acceleration_norm:.2f}m/s^2, "
                f"q_norm={self.imu_quaternion_norm:.4f}, "
                f"covariance={'valid' if self.imu_covariance_valid else 'invalid'}, "
                f"stamp_errors={self.backwards_sensor_stamps['IMU']}",
            )
            results.append(passed)

        lidar_rates = {
            name: self._rate(name, elapsed)
            for name in ("VLP packets", "VLP cloud", "2D scan")
        }
        required_lidar_names = ["VLP packets", "VLP cloud"]
        if self.require_2d_scan:
            required_lidar_names.append("2D scan")
        now_nanoseconds = self.get_clock().now().nanoseconds
        cloud_age = (
            stamp_age_seconds(now_nanoseconds, self.cloud_stamp)
            if self.cloud_stamp is not None else math.inf
        )
        scan_age = (
            stamp_age_seconds(now_nanoseconds, self.scan_stamp)
            if self.scan_stamp is not None else math.inf
        )
        ages_valid = (
            -0.05 <= cloud_age <= self.maximum_lidar_age
            and (
                not self.require_2d_scan
                or -0.05 <= scan_age <= self.maximum_lidar_age
            )
        )
        lidar_detail = (
            "packets/cloud/scan="
            f"{lidar_rates['VLP packets']:.1f}/"
            f"{lidar_rates['VLP cloud']:.1f}/"
            + (
                f"{lidar_rates['2D scan']:.1f} Hz, "
                if self.require_2d_scan else "SKIP, "
            )
            + f"cloud/scan age={cloud_age:.3f}/"
            + (f"{scan_age:.3f}s, " if self.require_2d_scan else "SKIP, ")
            +
            f"fields={'valid' if self.cloud_fields_valid else 'invalid'}, "
            "point_time="
            + (
                f"{self.cloud_point_time_bounds[0]:+.4f}.."
                f"{self.cloud_point_time_bounds[1]:+.4f}s"
                if self.cloud_point_time_bounds is not None
                else "missing"
            )
            + f" ({'valid' if self.cloud_point_time_valid else 'invalid'}), "
            "stamp_errors="
            f"{sum(self.backwards_sensor_stamps[name] for name in lidar_rates)}"
            " (packets="
            f"{self.backwards_sensor_stamps['VLP packets']},cloud="
            f"{self.backwards_sensor_stamps['VLP cloud']},scan="
            f"{self.backwards_sensor_stamps['2D scan']})"
        )
        if any(self.counts[name] == 0 for name in required_lidar_names):
            self._line("MISSING", "VLP-16", lidar_detail)
            results.append(False)
        else:
            passed = (
                all(
                    lidar_rates[name] >= self.minimum_rates[name]
                    for name in required_lidar_names
                )
                and self.cloud_valid
                and self.cloud_fields_valid
                and self.cloud_point_time_valid
                and (not self.require_2d_scan or self.scan_valid)
                and ages_valid
                and all(
                    self.backwards_sensor_stamps[name] == 0
                    for name in required_lidar_names
                )
            )
            self._line("PASS" if passed else "INVALID", "VLP-16", lidar_detail)
            results.append(passed)

        results.append(self._motor_result("left", elapsed))
        results.append(self._motor_result("right", elapsed))

        if self.require_arm_hold:
            self._line(
                "PASS" if self.arm_hold_ready else "MISSING",
                "Arm hold",
                "section start pose torque locked"
                if self.arm_hold_ready else "no confirmed arm torque hold",
            )
            results.append(self.arm_hold_ready)

        if not self.require_lio_stability:
            self._line("SKIP", "FAST-LIO stationary", "not required by this backend")
        else:
            # Prune against check time as well as callback time.  If FAST-LIO
            # stops publishing after a degenerate scan, an old window must not
            # be reported as a misleading 6-10 second stationary drift sample.
            now = time.monotonic()
            oldest = now - self.lio_stability_window
            self.lio_samples = [
                sample for sample in self.lio_samples if sample[0] >= oldest
            ]
        if self.require_lio_stability and len(self.lio_samples) < 2:
            self._line("MISSING", "FAST-LIO stationary", "no stable /lio/odom window")
            results.append(False)
        elif self.require_lio_stability:
            duration = self.lio_samples[-1][0] - self.lio_samples[0][0]
            rate = sampled_rate(len(self.lio_samples), duration)
            origin = self.lio_samples[0][1]
            maximum_drift = max(
                math.dist(origin, sample[1]) for sample in self.lio_samples
            )
            passed = (
                duration >= self.minimum_rate_observation
                and rate >= self.minimum_lio_rate
                and maximum_drift <= self.maximum_lio_drift
            )
            self._line(
                "PASS" if passed else "INVALID",
                "FAST-LIO stationary",
                f"{rate:.1f} Hz, drift={maximum_drift:.3f}m/"
                f"{duration:.1f}s (limit {self.maximum_lio_drift:.3f}m); "
                "keep robot still during startup",
            )
            results.append(passed)

        tf_ready = self._tf_ready(self.imu_frame) and self._tf_ready(self.lidar_frame)
        self._line(
            "PASS" if tf_ready else "INVALID",
            "sensor TF",
            f"{self.base_frame}->{self.imu_frame}, {self.base_frame}->{self.lidar_frame}",
        )
        results.append(tf_ready)

        self.ready = all(results)
        self.publish_ready(self.ready)
        self._line("RESULT", "field hardware", "READY" if self.ready else "NOT READY")
        if self.ready or (
            not self.retry_until_ready and self.check_attempts >= 2
        ):
            self.completed = True
        else:
            self.next_check_elapsed = elapsed + self.failed_check_retry
            self._line(
                "RETRY",
                "field hardware",
                f"automatic recheck in {self.failed_check_retry:.0f}s",
            )
        self._flush_attempt()
        if not self.completed:
            # Judge timestamp ordering over the next retry interval, not over
            # the entire process lifetime. Persistent/repeating disorder still
            # fails every interval; a startup-only transient can recover.
            reset_stamp_error_window(self.backwards_sensor_stamps)


def main(args=None):
    rclpy.init(args=args)
    node = FieldPreflight()
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
    # Keep the transient-local readiness publisher alive for the mapping
    # controls.  It remains an observer and never commands hardware.
    return 0
