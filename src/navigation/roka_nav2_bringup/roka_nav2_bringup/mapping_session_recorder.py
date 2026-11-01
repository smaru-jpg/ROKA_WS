"""Bounded rosbag2 recording for one operator-controlled mapping session."""

from __future__ import annotations

import os
import hashlib
import math
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from velodyne_msgs.msg import VelodyneScan
import yaml

from .data_contract import acquisition_topics, default_contract_path, load_data_contract


# Keep compact raw packets and low-bandwidth context.  Derived PointCloud2 and
# GridMap topics are intentionally rebuilt offline from this single contract.
DEFAULT_TOPICS = tuple(acquisition_topics(load_data_contract()))


def is_completed_recording_shutdown_error(
    error: RuntimeError,
    *,
    recording_active: bool,
    metadata_exists: bool,
) -> bool:
    """Recognize the rclpy teardown race only after rosbag was finalized."""
    return (
        not recording_active
        and metadata_exists
        and "Unable to convert call argument to Python object" in str(error)
    )


def directory_size(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for item in path.rglob("*"):
        try:
            if item.is_file() and not item.is_symlink():
                total += item.stat().st_size
        except OSError:
            continue
    return total


def critical_mapping_inputs(
    last_seen: dict[str, float | None],
    now: float,
    timeout: float,
    md_connected: bool | None,
    abort_on_md_disconnect: bool,
) -> list[str]:
    """Return inputs whose loss makes the current recording unusable.

    A temporary MD communication outage stops the chassis in the driver.  In
    the normal mapping policy it is therefore a recorded pause, not a reason
    to discard otherwise valid LiDAR/IMU data.  Wheel odometry is expected to
    pause with the MD link and is ignored only for that outage window.
    """
    stale = [
        name
        for name, stamp in last_seen.items()
        if stamp is None or now - float(stamp) > timeout
    ]
    if md_connected is False:
        if abort_on_md_disconnect:
            stale.append("MD base")
        else:
            stale = [name for name in stale if name != "odometry"]
    return stale


def build_record_command(
    output: Path,
    topics: list[str],
    max_bagfile_bytes: int,
    compression: bool,
) -> list[str]:
    command = [
        "ros2",
        "bag",
        "record",
        "--output",
        str(output),
        "--storage",
        "sqlite3",
        "--max-bag-size",
        str(max_bagfile_bytes),
        "--max-cache-size",
        str(32 * 1024 * 1024),
        "--log-level",
        "warn",
    ]
    if compression:
        command.extend([
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
            "--compression-queue-size",
            "2",
            "--compression-threads",
            "2",
        ])
    command.extend(topics)
    return command


class MappingSessionRecorder(Node):
    """Start rosbag on A, stop/finalize it on B, and enforce hard limits."""

    def __init__(self):
        super().__init__("mapping_session_recorder")
        self.declare_parameter("directory", "")
        self.declare_parameter("topics", list(DEFAULT_TOPICS))
        self.declare_parameter("maximum_total_bytes", 5 * 1024 ** 3)
        self.declare_parameter("maximum_duration_sec", 15.0 * 60.0)
        self.declare_parameter("maximum_bagfile_bytes", 512 * 1024 ** 2)
        self.declare_parameter("minimum_start_free_bytes", 20 * 1024 ** 3)
        self.declare_parameter("minimum_runtime_free_bytes", 10 * 1024 ** 3)
        self.declare_parameter("sensor_timeout_sec", 3.0)
        self.declare_parameter("abort_on_md_disconnect", False)
        self.declare_parameter("compression", True)
        self.declare_parameter("environment", "flat")
        self.declare_parameter("data_contract", str(default_contract_path()))
        self.declare_parameter("odometry_topic", "/wheel/odom")
        self.declare_parameter("mapping_backend", "slam_toolbox_2d")
        self.declare_parameter("hardware_config", "")

        directory = str(self.get_parameter("directory").value).strip()
        if not directory:
            raise ValueError("directory must be the mapping staging directory")
        self.directory = Path(directory).expanduser().resolve()
        self.output = self.directory / "sensors"
        contract_path = str(self.get_parameter("data_contract").value).strip()
        self.contract_path = Path(contract_path).expanduser().resolve()
        self.data_contract = load_data_contract(self.contract_path)
        configured_topics = [str(item) for item in self.get_parameter("topics").value]
        self.topics = configured_topics or acquisition_topics(self.data_contract)
        self.odometry_topic = str(
            self.get_parameter("odometry_topic").value
        ).strip()
        if not self.odometry_topic.startswith("/"):
            raise ValueError("odometry_topic must be an absolute ROS topic")
        self.mapping_backend = str(
            self.get_parameter("mapping_backend").value
        ).strip()
        hardware_config = str(self.get_parameter("hardware_config").value).strip()
        self.hardware_config = (
            Path(hardware_config).expanduser().resolve() if hardware_config else None
        )
        self.maximum_total = int(self.get_parameter("maximum_total_bytes").value)
        self.maximum_duration = float(self.get_parameter("maximum_duration_sec").value)
        self.maximum_bagfile = int(self.get_parameter("maximum_bagfile_bytes").value)
        self.minimum_start_free = int(
            self.get_parameter("minimum_start_free_bytes").value
        )
        self.minimum_runtime_free = int(
            self.get_parameter("minimum_runtime_free_bytes").value
        )
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_sec").value)
        self.abort_on_md_disconnect = bool(
            self.get_parameter("abort_on_md_disconnect").value
        )
        self.compression = bool(self.get_parameter("compression").value)
        self.environment = str(
            self.get_parameter("environment").value
        ).strip().lower()
        if min(
            self.maximum_total,
            self.maximum_bagfile,
            self.minimum_runtime_free,
        ) <= 0 or self.maximum_duration <= 0.0:
            raise ValueError("recording size, free-space, and duration limits must be positive")
        if self.environment not in {"capture", "flat", "inline"}:
            raise ValueError("environment must be capture, flat or inline")

        self.process: subprocess.Popen | None = None
        self.started_at: float | None = None
        self.aborted_reason = ""
        self.last_seen = {"VLP-16": None, "EBIMU": None, "odometry": None}
        self.md_connected = None
        self.md_disconnect_started = None
        self.md_disconnect_count = 0
        self.md_disconnect_total_sec = 0.0
        self.md_disconnect_max_sec = 0.0
        self.previous_odom_xy = None
        self.odom_distance_m = 0.0
        self.previous_map_odom = None
        self.maximum_map_odom_jump_m = 0.0
        self.maximum_map_odom_jump_deg = 0.0

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String, "/mapping/recording_status", status_qos
        )
        self.create_service(Trigger, "~/start", self.start_callback)
        self.create_service(Trigger, "~/stop", self.stop_callback)
        self.create_subscription(
            VelodyneScan,
            "/velodyne_packets",
            lambda _message: self.mark_seen("VLP-16"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/imu/data",
            lambda _message: self.mark_seen("EBIMU"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(TFMessage, "/tf", self.tf_callback, 50)
        self.create_subscription(Bool, "/md/base_connected", self.md_callback, 10)
        self.create_timer(0.50, self.monitor)
        self.publish_status("ready; recording is stopped until Xbox A")

    def publish_status(self, message: str, *, error: bool = False) -> None:
        status = String()
        status.data = message
        self.status_pub.publish(status)
        print(
            f"[MAPPING-BAG][{'ERROR' if error else 'STATUS'}] {message}",
            flush=True,
        )

    def mark_seen(self, name: str) -> None:
        self.last_seen[name] = time.monotonic()

    def md_callback(self, message: Bool) -> None:
        connected = bool(message.data)
        previous = self.md_connected
        self.md_connected = connected
        if self.process is None or connected == previous:
            return
        now = time.monotonic()
        if not connected:
            self.md_disconnect_count += 1
            self.md_disconnect_started = now
            self.publish_status(
                "MD feedback temporarily unavailable; base output is stopped "
                "and mapping recording continues"
            )
        elif self.md_disconnect_started is not None:
            duration = self._finish_md_outage(now)
            self.publish_status(
                f"MD feedback recovered after {duration:.2f}s; release and "
                "press LB to resume manual drive"
            )

    def _finish_md_outage(self, now: float) -> float:
        if self.md_disconnect_started is None:
            return 0.0
        duration = max(0.0, now - self.md_disconnect_started)
        self.md_disconnect_total_sec += duration
        self.md_disconnect_max_sec = max(
            self.md_disconnect_max_sec, duration
        )
        self.md_disconnect_started = None
        return duration

    @staticmethod
    def yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )

    @staticmethod
    def wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def odom_callback(self, message: Odometry) -> None:
        self.mark_seen("odometry")
        if self.process is None:
            return
        position = message.pose.pose.position
        xy = (float(position.x), float(position.y))
        if self.previous_odom_xy is not None:
            self.odom_distance_m += math.hypot(
                xy[0] - self.previous_odom_xy[0],
                xy[1] - self.previous_odom_xy[1],
            )
        self.previous_odom_xy = xy

    def tf_callback(self, message: TFMessage) -> None:
        if self.process is None:
            return
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent != "map" or child != "odom":
                continue
            translation = transform.transform.translation
            current = (
                float(translation.x),
                float(translation.y),
                self.yaw_from_quaternion(transform.transform.rotation),
            )
            if self.previous_map_odom is not None:
                jump = math.hypot(
                    current[0] - self.previous_map_odom[0],
                    current[1] - self.previous_map_odom[1],
                )
                yaw_jump = abs(self.wrap_angle(current[2] - self.previous_map_odom[2]))
                self.maximum_map_odom_jump_m = max(
                    self.maximum_map_odom_jump_m, jump
                )
                self.maximum_map_odom_jump_deg = max(
                    self.maximum_map_odom_jump_deg, math.degrees(yaw_jump)
                )
            self.previous_map_odom = current

    def free_bytes(self) -> int:
        self.directory.mkdir(parents=True, exist_ok=True)
        return int(shutil.disk_usage(self.directory).free)

    def start_callback(self, _request, response):
        if self.process is not None:
            response.success = False
            response.message = "sensor bag is already recording"
            return response
        if self.aborted_reason:
            response.success = False
            response.message = f"session is aborted: {self.aborted_reason}"
            return response
        if self.output.exists():
            response.success = False
            response.message = f"refusing to overwrite staging bag: {self.output}"
            return response
        free = self.free_bytes()
        if free < self.minimum_start_free:
            response.success = False
            response.message = (
                f"insufficient free space: {free / 1024 ** 3:.1f} GiB; "
                f"need {self.minimum_start_free / 1024 ** 3:.1f} GiB"
            )
            return response
        command = build_record_command(
            self.output,
            self.topics,
            self.maximum_bagfile,
            self.compression,
        )
        try:
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.25)
            if self.process.poll() is not None:
                code = self.process.returncode
                self.process = None
                raise RuntimeError(f"rosbag2 exited during startup with code {code}")
        except (OSError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
            return response
        self.started_at = time.monotonic()
        self.previous_odom_xy = None
        self.odom_distance_m = 0.0
        self.previous_map_odom = None
        self.maximum_map_odom_jump_m = 0.0
        self.maximum_map_odom_jump_deg = 0.0
        self.md_disconnect_started = None
        self.md_disconnect_count = 0
        self.md_disconnect_total_sec = 0.0
        self.md_disconnect_max_sec = 0.0
        if self.md_connected is False:
            self.md_disconnect_count = 1
            self.md_disconnect_started = self.started_at
        self.publish_status(
            f"recording started: {self.output}; limit="
            f"{self.maximum_total / 1024 ** 3:.1f} GiB/"
            f"{self.maximum_duration / 60.0:.0f} min"
        )
        response.success = True
        response.message = "bounded sensor recording started"
        return response

    def terminate_process(self) -> tuple[bool, str]:
        process = self.process
        if process is None:
            return False, "sensor bag is not recording"
        self._finish_md_outage(time.monotonic())
        try:
            os.killpg(process.pid, signal.SIGINT)
            return_code = process.wait(timeout=120.0)
        except ProcessLookupError:
            return_code = process.poll()
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                return_code = process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                return_code = process.wait(timeout=5.0)
        finally:
            self.process = None
            self.started_at = None
        metadata = self.output / "metadata.yaml"
        if return_code not in (0, 130, -signal.SIGINT):
            return False, f"rosbag2 stopped with code {return_code}"
        if not metadata.is_file() or metadata.stat().st_size == 0:
            return False, "rosbag2 did not finalize metadata.yaml"
        metrics = {
            "schema_version": 2,
            "environment": self.environment,
            "mapping_backend": self.mapping_backend,
            "acquisition_odometry_topic": self.odometry_topic,
            "recorded_size_bytes": directory_size(self.output),
            "odom_cumulative_distance_m": self.odom_distance_m,
            "maximum_map_odom_translation_jump_m": self.maximum_map_odom_jump_m,
            "maximum_map_odom_yaw_jump_deg": self.maximum_map_odom_jump_deg,
            "md_disconnect_count": self.md_disconnect_count,
            "md_disconnect_total_sec": self.md_disconnect_total_sec,
            "md_disconnect_max_sec": self.md_disconnect_max_sec,
        }
        (self.directory / "session_metrics.yaml").write_text(
            yaml.safe_dump(metrics, sort_keys=False), encoding="utf-8"
        )
        contract_snapshot = {
            "schema_version": 1,
            "data_contract": self.data_contract,
            "data_contract_sha256": hashlib.sha256(
                self.contract_path.read_bytes()
            ).hexdigest(),
            "hardware_config": str(self.hardware_config or ""),
            "hardware_config_sha256": (
                hashlib.sha256(self.hardware_config.read_bytes()).hexdigest()
                if self.hardware_config and self.hardware_config.is_file()
                else ""
            ),
            "mapping_backend": self.mapping_backend,
            "environment": self.environment,
            "recorded_topics": self.topics,
            "acquisition_odometry_topic": self.odometry_topic,
        }
        (self.directory / "session_contract.yaml").write_text(
            yaml.safe_dump(contract_snapshot, sort_keys=False), encoding="utf-8"
        )
        return True, f"sensor bag finalized ({directory_size(self.output) / 1024 ** 2:.1f} MiB)"

    def stop_callback(self, _request, response):
        success, message = self.terminate_process()
        if self.aborted_reason:
            success = False
            message = f"recording aborted: {self.aborted_reason}; {message}"
        response.success = success
        response.message = message
        self.publish_status(message, error=not success)
        return response

    def abort(self, reason: str) -> None:
        if self.aborted_reason:
            return
        self.aborted_reason = reason
        success, stop_message = self.terminate_process()
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / "SESSION_ABORTED.txt").write_text(
            reason + "\n" + stop_message + "\n", encoding="utf-8"
        )
        self.publish_status(
            f"recording stopped and bundle locked: {reason}; {stop_message}",
            error=True,
        )

    def monitor(self) -> None:
        if self.process is None:
            return
        return_code = self.process.poll()
        if return_code is not None:
            self.process = None
            self.started_at = None
            self.aborted_reason = f"rosbag2 exited unexpectedly with code {return_code}"
            self.publish_status(self.aborted_reason, error=True)
            return
        now = time.monotonic()
        elapsed = now - float(self.started_at)
        if elapsed > self.maximum_duration:
            self.abort(f"maximum recording time exceeded ({self.maximum_duration:.0f}s)")
            return
        size = directory_size(self.output)
        if size > self.maximum_total:
            self.abort(f"maximum recording size exceeded ({size / 1024 ** 3:.2f} GiB)")
            return
        free = self.free_bytes()
        if free < self.minimum_runtime_free:
            self.abort(
                "disk free space fell below "
                f"{self.minimum_runtime_free / 1024 ** 3:.1f} GiB"
            )
            return
        if elapsed < self.sensor_timeout:
            return
        stale = critical_mapping_inputs(
            self.last_seen,
            now,
            self.sensor_timeout,
            self.md_connected,
            self.abort_on_md_disconnect,
        )
        if stale:
            self.abort("critical input lost: " + ", ".join(stale))

    def destroy_node(self):
        if self.process is not None:
            self.abort("mapping launch stopped before Xbox B")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingSessionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if not is_completed_recording_shutdown_error(
            error,
            recording_active=node.process is not None,
            metadata_exists=(node.output / "metadata.yaml").is_file(),
        ):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
