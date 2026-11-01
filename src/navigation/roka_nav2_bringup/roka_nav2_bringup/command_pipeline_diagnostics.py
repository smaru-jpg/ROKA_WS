"""Report the first blocked stage in the real autonomy velocity pipeline."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from roka_interfaces.msg import TerrainState


def command_magnitude(message: Twist | None) -> float:
    if message is None:
        return 0.0
    return max(abs(float(message.linear.x)), abs(float(message.angular.z)))


def blocked_stage(
    nav: float,
    mux: float,
    terrain: float,
    output: float,
    *,
    threshold: float,
    terrain_reason: str,
    obstacle_age_sec: float | None,
    obstacle_max_age_sec: float,
) -> tuple[str, str]:
    """Return the earliest stage that removed a nonzero Nav2 command."""
    if nav <= threshold:
        return "idle", "Nav2 has not requested motion"
    if mux <= threshold:
        return "mux_or_smoother", "Nav2 command did not reach /cmd_vel_mux"
    if terrain <= threshold:
        return "terrain", terrain_reason or "Terrain Supervisor output is zero"
    if output <= threshold:
        if obstacle_age_sec is None:
            detail = "collision source has not published"
        elif obstacle_age_sec > obstacle_max_age_sec:
            detail = (
                f"collision source is stale ({obstacle_age_sec:.3f}s > "
                f"{obstacle_max_age_sec:.3f}s)"
            )
        else:
            detail = "Collision Monitor stopped the command"
        return "collision", detail
    return "output", "motor command is flowing"


class CommandPipelineDiagnostics(Node):
    def __init__(self):
        super().__init__("autonomy_command_pipeline")
        self.declare_parameter("command_threshold", 0.01)
        self.declare_parameter("blocked_hold_sec", 0.75)
        self.declare_parameter("motion_hold_sec", 2.0)
        self.declare_parameter("obstacle_max_age_sec", 1.20)
        self.declare_parameter("message_timeout_sec", 0.60)
        self.declare_parameter("direct_recorded_route", False)
        self.threshold = float(self.get_parameter("command_threshold").value)
        self.blocked_hold = float(self.get_parameter("blocked_hold_sec").value)
        self.motion_hold = float(self.get_parameter("motion_hold_sec").value)
        self.obstacle_max_age = float(
            self.get_parameter("obstacle_max_age_sec").value
        )
        self.message_timeout = float(self.get_parameter("message_timeout_sec").value)
        self.direct_recorded_route = bool(
            self.get_parameter("direct_recorded_route").value
        )
        if min(
            self.threshold,
            self.blocked_hold,
            self.motion_hold,
            self.obstacle_max_age,
            self.message_timeout,
        ) <= 0.0:
            raise ValueError("pipeline diagnostic thresholds must be positive")

        self.commands: dict[str, tuple[Twist, Time] | None] = {
            "nav": None,
            "mux": None,
            "terrain": None,
            "output": None,
        }
        self.terrain_state: TerrainState | None = None
        self.obstacle_stamp: Time | None = None
        self.odom_speed = 0.0
        self.odom_received_at: Time | None = None
        self.candidate = "idle"
        self.candidate_since = self.get_clock().now()
        self.reported = ""
        self.output_started_at: Time | None = None

        for topic, key in (
            ("/cmd_vel_nav_smoothed", "nav"),
            ("/cmd_vel_mux", "mux"),
            ("/cmd_vel_terrain", "terrain"),
            ("/cmd_vel", "output"),
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, stage=key: self.command_callback(stage, message),
                10,
            )
        self.create_subscription(
            TerrainState, "/terrain/state", self.terrain_callback, 10
        )
        self.create_subscription(
            PointCloud2,
            "/terrain/obstacle_points",
            self.obstacle_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/wheel/odom", self.odom_callback, 10)
        self.status_pub = self.create_publisher(String, "/autonomy/pipeline_status", 10)
        self.create_timer(0.20, self.update)

    def command_callback(self, stage: str, message: Twist) -> None:
        self.commands[stage] = (message, self.get_clock().now())

    def terrain_callback(self, message: TerrainState) -> None:
        self.terrain_state = message

    def obstacle_callback(self, message: PointCloud2) -> None:
        self.obstacle_stamp = Time.from_msg(message.header.stamp)

    def odom_callback(self, message: Odometry) -> None:
        self.odom_speed = abs(float(message.twist.twist.linear.x))
        self.odom_received_at = self.get_clock().now()

    def fresh_magnitude(self, stage: str, now: Time) -> float:
        item = self.commands[stage]
        if item is None:
            return 0.0
        message, received = item
        if (now - received).nanoseconds / 1.0e9 > self.message_timeout:
            return 0.0
        return command_magnitude(message)

    def obstacle_age(self, now: Time) -> float | None:
        if self.obstacle_stamp is None or self.obstacle_stamp.nanoseconds <= 0:
            return None
        age = (now - self.obstacle_stamp).nanoseconds / 1.0e9
        return age if math.isfinite(age) else None

    def update(self) -> None:
        now = self.get_clock().now()
        values = {
            stage: self.fresh_magnitude(stage, now) for stage in self.commands
        }
        terrain_reason = (
            str(self.terrain_state.reason) if self.terrain_state is not None else
            "no /terrain/state received"
        )
        age = self.obstacle_age(now)
        if self.direct_recorded_route:
            if values["nav"] <= self.threshold:
                stage, detail = "idle", "Nav2 has not requested motion"
            elif values["output"] <= self.threshold:
                stage, detail = (
                    "smoother_or_output",
                    "recorded-route command did not reach /cmd_vel",
                )
            else:
                stage, detail = "output", "recorded-route motor command is flowing"
        else:
            stage, detail = blocked_stage(
                values["nav"],
                values["mux"],
                values["terrain"],
                values["output"],
                threshold=self.threshold,
                terrain_reason=terrain_reason,
                obstacle_age_sec=age,
                obstacle_max_age_sec=self.obstacle_max_age,
            )

        if stage == "output":
            if self.output_started_at is None:
                self.output_started_at = now
            odom_fresh = (
                self.odom_received_at is not None and
                (now - self.odom_received_at).nanoseconds / 1.0e9 <= self.message_timeout
            )
            if (
                odom_fresh and self.odom_speed < self.threshold and
                (now - self.output_started_at).nanoseconds / 1.0e9 >= self.motion_hold
            ):
                stage = "base_motion"
                detail = "nonzero /cmd_vel reached MD driver but wheel odometry stayed zero"
        else:
            self.output_started_at = None

        if stage != self.candidate:
            self.candidate = stage
            self.candidate_since = now
            return
        if (now - self.candidate_since).nanoseconds / 1.0e9 < self.blocked_hold:
            return
        if stage == self.reported or stage == "idle":
            return
        age_text = (
            "bypassed" if self.direct_recorded_route
            else ("none" if age is None else f"{age:.3f}s")
        )
        message = (
            f"[AUTONOMY-PIPELINE][{'PASS' if stage == 'output' else 'BLOCKED'}] "
            f"stage={stage}; {detail}; nav/mux/terrain/output="
            f"{values['nav']:.3f}/{values['mux']:.3f}/"
            f"{values['terrain']:.3f}/{values['output']:.3f}; "
            f"obstacle_age={age_text}; wheel_speed={self.odom_speed:.3f}m/s"
        )
        status = String()
        status.data = message
        self.status_pub.publish(status)
        if stage == "output":
            print(message, flush=True)
        else:
            self.get_logger().warning(message)
        self.reported = stage


def main(args=None):
    rclpy.init(args=args)
    node = CommandPipelineDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
