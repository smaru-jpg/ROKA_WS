#!/usr/bin/env python3
"""Fail-closed command gate for the two Section-5 Nav2 lifecycle nodes."""

from __future__ import annotations

import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def state_request_expired(sent_at, now, timeout):
    return now - sent_at >= timeout


class Section5LifecycleGate(Node):
    def __init__(self):
        super().__init__("section5_lifecycle_gate")
        self.input_topic = str(self.declare_parameter(
            "input_topic", "/cmd_vel_mission5_safe"
        ).value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/cmd_vel_mission5_lifecycle_safe"
        ).value)
        self.target_nodes = [str(value) for value in self.declare_parameter(
            "target_nodes",
            ["mission5_velocity_smoother", "mission5_collision_monitor"],
        ).value]
        self.input_timeout = float(self.declare_parameter(
            "input_timeout_sec", 0.30
        ).value)
        self.check_period = float(self.declare_parameter(
            "check_period_sec", 0.25
        ).value)
        self.state_request_timeout = float(self.declare_parameter(
            "state_request_timeout_sec", 1.0
        ).value)
        if (
            not self.target_nodes
            or self.input_timeout <= 0.0
            or self.check_period <= 0.0
            or self.state_request_timeout <= 0.0
        ):
            raise ValueError("lifecycle gate requires nodes and positive timeouts")

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._command, 10)
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.target_nodes
        }
        self.states = {name: False for name in self.target_nodes}
        # name -> (future, monotonic send time). A request issued during a
        # lifecycle transition can otherwise remain pending forever and keep
        # every Section-5 command locked at zero.
        self.pending = {}
        self.last_input = 0.0
        self.forwarding = False
        self.create_timer(self.check_period, self._check_states)
        self.create_timer(0.10, self._watchdog)

    def _zero(self):
        if not rclpy.ok(context=self.context):
            return
        try:
            self.publisher.publish(Twist())
        except RCLError:
            if rclpy.ok(context=self.context):
                raise

    def _ready(self):
        return all(self.states.values())

    def _command(self, message):
        self.last_input = time.monotonic()
        if self._ready():
            self.publisher.publish(message)
        else:
            self._zero()

    def _state_result(self, name, future):
        pending = self.pending.get(name)
        if pending is None or pending[0] is not future:
            return
        self.pending.pop(name, None)
        try:
            state = future.result().current_state.id
        except Exception:
            state = State.PRIMARY_STATE_UNKNOWN
        self.states[name] = state == State.PRIMARY_STATE_ACTIVE
        ready = self._ready()
        if ready != self.forwarding:
            self.forwarding = ready
            if ready:
                self.get_logger().info(
                    "[MISSION5-GATE] safety lifecycle ACTIVE; commands enabled"
                )
            else:
                self._zero()
                self.get_logger().warning(
                    "[MISSION5-GATE] safety lifecycle inactive; commands stopped"
                )

    def _check_states(self):
        now = time.monotonic()
        for name, client in self.state_clients.items():
            pending = self.pending.get(name)
            if pending is not None:
                future, sent_at = pending
                if not state_request_expired(
                    sent_at, now, self.state_request_timeout
                ):
                    continue
                future.cancel()
                self.pending.pop(name, None)
                self.states[name] = False
                self._zero()
                self.get_logger().warning(
                    f"[MISSION5-GATE] {name} lifecycle state request timed out; retrying"
                )
            if not client.service_is_ready():
                self.states[name] = False
                continue
            future = client.call_async(GetState.Request())
            self.pending[name] = (future, now)
            future.add_done_callback(
                lambda result, node_name=name: self._state_result(node_name, result)
            )
        if not self._ready():
            self._zero()

    def _watchdog(self):
        if time.monotonic() - self.last_input > self.input_timeout:
            self._zero()

    def close(self):
        self._zero()


def main(args=None):
    rclpy.init(args=args)
    node = Section5LifecycleGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
