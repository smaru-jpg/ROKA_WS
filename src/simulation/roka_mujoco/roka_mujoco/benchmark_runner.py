"""Reset and evaluate repeated learned-policy laps, writing per-run CSV results."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from roka_robot_description.workspace import workspace_path

from .runtime_environment import LOCAL_INSTALL, configure_local_dependencies


# The capstone-compatible MuJoCo runtime is user-local and has no setup.bash.
# Expose its generated Python service binding for direct ros2 run as well as launch.
_python_site = (
    LOCAL_INSTALL
    / "local"
    / "lib"
    / f"python{sys.version_info.major}.{sys.version_info.minor}"
    / "dist-packages"
)
if _python_site.is_dir() and str(_python_site) not in sys.path:
    sys.path.insert(0, str(_python_site))
configure_local_dependencies()
from mujoco_ros2_control_msgs.srv import ResetWorld  # noqa: E402


CSV_FIELDS = (
    "run",
    "reset_keyframe",
    "success",
    "outcome",
    "sim_duration_sec",
    "wall_duration_sec",
    "max_progress_pct",
    "max_abs_lateral_error_m",
    "rms_lateral_error_m",
    "max_abs_heading_error_deg",
    "max_abs_roll_deg",
    "max_abs_pitch_deg",
    "min_reference_height_delta_m",
    "min_tire_clearance_m",
    "recovery_count",
    "recovery_time_sec",
    "corridor_unavailable_samples",
    "max_linear_command_mps",
    "max_angular_command_radps",
    "status_samples",
)


class EpisodeMetrics:
    """Online accumulator for one reset-to-reset benchmark episode."""

    def __init__(self, run_number: int, reset_keyframe: str = "") -> None:
        self.run_number = run_number
        self.reset_keyframe = reset_keyframe
        self.wall_start = time.monotonic()
        self.samples = 0
        self.max_progress = 0.0
        self.max_lateral = 0.0
        self.lateral_square_sum = 0.0
        self.max_heading = 0.0
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.min_height = math.inf
        self.min_tire = math.inf
        self.recovery_count = 0
        self.recovery_time = 0.0
        self.corridor_unavailable = 0
        self.max_linear = 0.0
        self.max_angular = 0.0
        self.last_sim_time: float | None = None
        self.sim_duration = 0.0

    def update(self, status: dict[str, Any]) -> None:
        sim_time = float(status.get("episode_elapsed_sim_sec", 0.0))
        if self.last_sim_time is not None:
            delta = max(0.0, min(0.20, sim_time - self.last_sim_time))
            if bool(status.get("recovery_active", False)):
                self.recovery_time += delta
        self.last_sim_time = sim_time
        self.sim_duration = max(self.sim_duration, sim_time)
        lateral = abs(float(status.get("reference_lateral_error_m", 0.0)))
        heading = abs(float(status.get("reference_heading_error_deg", 0.0)))
        roll = abs(float(status.get("roll_deg", 0.0)))
        pitch = abs(float(status.get("pitch_deg", 0.0)))
        self.samples += 1
        self.max_progress = max(
            self.max_progress, float(status.get("path_progress_pct", 0.0))
        )
        self.max_lateral = max(self.max_lateral, lateral)
        self.lateral_square_sum += lateral * lateral
        self.max_heading = max(self.max_heading, heading)
        self.max_roll = max(self.max_roll, roll)
        self.max_pitch = max(self.max_pitch, pitch)
        self.min_height = min(
            self.min_height,
            float(status.get("reference_height_delta_m", math.inf)),
        )
        tire = status.get("tire_clearance_m")
        if tire is not None:
            self.min_tire = min(self.min_tire, float(tire))
        self.recovery_count = max(
            self.recovery_count, int(status.get("recovery_count", 0))
        )
        if not bool(status.get("corridor_available", True)):
            self.corridor_unavailable += 1
        self.max_linear = max(
            self.max_linear, abs(float(status.get("command_linear_mps", 0.0)))
        )
        self.max_angular = max(
            self.max_angular, abs(float(status.get("command_angular_radps", 0.0)))
        )

    def finish(self, success: bool, outcome: str) -> dict[str, Any]:
        rms = math.sqrt(self.lateral_square_sum / max(self.samples, 1))
        return {
            "run": self.run_number,
            "reset_keyframe": self.reset_keyframe,
            "success": int(success),
            "outcome": outcome,
            "sim_duration_sec": round(self.sim_duration, 3),
            "wall_duration_sec": round(time.monotonic() - self.wall_start, 3),
            "max_progress_pct": round(100.0 if success else self.max_progress, 3),
            "max_abs_lateral_error_m": round(self.max_lateral, 4),
            "rms_lateral_error_m": round(rms, 4),
            "max_abs_heading_error_deg": round(self.max_heading, 3),
            "max_abs_roll_deg": round(self.max_roll, 3),
            "max_abs_pitch_deg": round(self.max_pitch, 3),
            "min_reference_height_delta_m": (
                round(self.min_height, 4) if math.isfinite(self.min_height) else ""
            ),
            "min_tire_clearance_m": (
                round(self.min_tire, 4) if math.isfinite(self.min_tire) else ""
            ),
            "recovery_count": self.recovery_count,
            "recovery_time_sec": round(self.recovery_time, 3),
            "corridor_unavailable_samples": self.corridor_unavailable,
            "max_linear_command_mps": round(self.max_linear, 4),
            "max_angular_command_radps": round(self.max_angular, 4),
            "status_samples": self.samples,
        }


def benchmark_decision(rows: list[dict[str, Any]]) -> dict[str, Any]:
    successes = sum(int(row["success"]) for row in rows)
    rate = successes / max(len(rows), 1)
    max_lateral = max((float(row["max_abs_lateral_error_m"]) for row in rows), default=math.inf)
    max_roll = max((float(row["max_abs_roll_deg"]) for row in rows), default=math.inf)
    max_pitch = max((float(row["max_abs_pitch_deg"]) for row in rows), default=math.inf)
    if rate == 1.0 and max_lateral <= 0.25 and max_roll < 70.0 and max_pitch < 60.0:
        decision = "PASS"
        recommendation = "manual_speed_step_test_allowed"
    elif rate >= 0.8 and max_roll < 75.0 and max_pitch < 65.0:
        decision = "CAUTION"
        recommendation = "repeat_or_tune_before_speed_increase"
    else:
        decision = "FAIL"
        recommendation = "do_not_increase_speed"
    return {
        "runs": len(rows),
        "successes": successes,
        "completion_rate": rate,
        "worst_lateral_error_m": max_lateral,
        "maximum_abs_roll_deg": max_roll,
        "maximum_abs_pitch_deg": max_pitch,
        "decision": decision,
        "recommendation": recommendation,
    }


class BenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__("sim_learned_benchmark")
        self.declare_parameter("runs", 10)
        self.declare_parameter("output_file", "")
        self.declare_parameter("status_topic", "/sim/learned_controller/status")
        self.declare_parameter("controller_start_service", "/sim_learned_controller/start")
        self.declare_parameter("controller_stop_service", "/sim_learned_controller/stop")
        self.declare_parameter("reset_world_service", "/mujoco_ros2_control_node/reset_world")
        self.declare_parameter("settle_wall_sec", 0.75)
        self.declare_parameter("startup_retry_wall_sec", 30.0)
        self.declare_parameter("episode_timeout_sim_sec", 600.0)
        self.declare_parameter("reset_keyframes", [""])

        self.total_runs = int(self.get_parameter("runs").value)
        if self.total_runs <= 0:
            raise ValueError("benchmark runs must be positive")
        output = str(self.get_parameter("output_file").value).strip()
        if output:
            self.output_file = Path(output).expanduser().resolve()
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_file = Path(
                str(workspace_path("benchmarks", f"sim_autonomy_{stamp}.csv"))
            )
        self.summary_file = self.output_file.with_name(
            f"{self.output_file.stem}_summary.json"
        )
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.settle_wall_sec = float(self.get_parameter("settle_wall_sec").value)
        self.startup_retry_wall_sec = float(
            self.get_parameter("startup_retry_wall_sec").value
        )
        self.timeout_sim_sec = float(
            self.get_parameter("episode_timeout_sim_sec").value
        )
        self.reset_keyframes = [
            str(value) for value in self.get_parameter("reset_keyframes").value
        ]
        if not self.reset_keyframes:
            raise ValueError("reset_keyframes must contain at least one value")
        self.current_reset_keyframe = ""
        self.start_client = self.create_client(
            Trigger, str(self.get_parameter("controller_start_service").value)
        )
        self.stop_client = self.create_client(
            Trigger, str(self.get_parameter("controller_stop_service").value)
        )
        self.reset_client = self.create_client(
            ResetWorld, str(self.get_parameter("reset_world_service").value)
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self._status_callback,
            30,
        )
        self.timer = self.create_timer(0.10, self._tick)
        self.state = "WAIT_SERVICES"
        self.future = None
        self.settle_deadline = 0.0
        self.latest_status: dict[str, Any] | None = None
        self.metrics: EpisodeMetrics | None = None
        self.rows: list[dict[str, Any]] = []
        self.finished = False
        self.start_retry_deadline = 0.0
        self.last_start_failure = ""
        self.get_logger().info(
            f"benchmark ready: runs={self.total_runs}, output={self.output_file}"
        )

    def _status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.latest_status = status
        if self.state == "RUNNING" and self.metrics is not None:
            self.metrics.update(status)

    def _services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (self.start_client, self.stop_client, self.reset_client)
        )

    def _request_reset(self) -> None:
        request = ResetWorld.Request()
        self.current_reset_keyframe = self.reset_keyframes[
            len(self.rows) % len(self.reset_keyframes)
        ]
        request.keyframe = self.current_reset_keyframe
        self.future = self.reset_client.call_async(request)
        self.state = "RESETTING"

    def _request_start(self) -> None:
        if self.start_retry_deadline <= 0.0:
            self.start_retry_deadline = time.monotonic() + self.startup_retry_wall_sec
        self.future = self.start_client.call_async(Trigger.Request())
        self.state = "STARTING"

    def _record_episode(self, success: bool, outcome: str) -> None:
        if self.metrics is None:
            return
        row = self.metrics.finish(success, outcome)
        self.rows.append(row)
        with self.output_file.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(
            f"run {row['run']}/{self.total_runs}: "
            f"{'PASS' if success else 'FAIL'} {outcome}, "
            f"sim={row['sim_duration_sec']}s, "
            f"max_lateral={row['max_abs_lateral_error_m']}m, "
            f"recoveries={row['recovery_count']}"
        )
        self.metrics = None
        if len(self.rows) >= self.total_runs:
            self._finish_benchmark()
        else:
            self._request_reset()

    def _finish_benchmark(self) -> None:
        decision = benchmark_decision(self.rows)
        document = {
            "schema_version": 1,
            "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "csv_file": str(self.output_file),
            **decision,
        }
        self.summary_file.write_text(
            json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(
            f"benchmark {decision['decision']}: "
            f"{decision['successes']}/{decision['runs']} complete, "
            f"CSV={self.output_file}, summary={self.summary_file}"
        )
        self.state = "FINISHED"
        self.finished = True

    def _tick(self) -> None:
        if self.state == "WAIT_SERVICES":
            if self._services_ready():
                self._request_reset()
            return
        if self.state == "RESETTING":
            if self.future is None or not self.future.done():
                return
            response = self.future.result()
            if response is None or not response.success:
                message = "reset_world service failed"
                if response is not None:
                    message = response.message
                self.get_logger().error(message)
                self._finish_benchmark()
                return
            self.settle_deadline = time.monotonic() + self.settle_wall_sec
            self.state = "SETTLING"
            return
        if self.state == "SETTLING":
            if time.monotonic() >= self.settle_deadline:
                self._request_start()
            return
        if self.state == "STARTING":
            if self.future is None or not self.future.done():
                return
            response = self.future.result()
            if response is None or not response.success:
                reason = (
                    "controller start service returned no response"
                    if response is None else str(response.message)
                )
                if time.monotonic() < self.start_retry_deadline:
                    if reason != self.last_start_failure:
                        self.last_start_failure = reason
                        self.get_logger().info(
                            f"waiting for complete control chain: {reason}"
                        )
                    self.settle_deadline = time.monotonic() + 0.50
                    self.state = "SETTLING"
                    return
                self.get_logger().error(
                    f"controller start did not become ready: {reason}"
                )
                self._finish_benchmark()
                return
            self.start_retry_deadline = 0.0
            self.last_start_failure = ""
            run_number = len(self.rows) + 1
            self.metrics = EpisodeMetrics(run_number, self.current_reset_keyframe)
            self.latest_status = None
            self.state = "RUNNING"
            self.get_logger().info(f"run {run_number}/{self.total_runs} started")
            return
        if self.state != "RUNNING" or self.latest_status is None:
            return
        status = self.latest_status
        if int(status.get("completed_laps", 0)) >= 1 and not bool(status.get("active", True)):
            self._record_episode(True, "completed_lap")
            return
        fault = str(status.get("fault_reason", ""))
        if fault and not bool(status.get("active", True)):
            self._record_episode(False, fault)
            return
        if float(status.get("episode_elapsed_sim_sec", 0.0)) > self.timeout_sim_sec:
            self.stop_client.call_async(Trigger.Request())
            self._record_episode(False, "episode_timeout")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BenchmarkRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
