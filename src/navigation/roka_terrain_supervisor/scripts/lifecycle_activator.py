#!/usr/bin/env python3
"""Retry a Nav2 lifecycle transition until the safety node is ACTIVE."""

import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


class LifecycleActivator(Node):
    def __init__(self):
        super().__init__("roka_lifecycle_activator")
        self.declare_parameter("target_node", "/collision_monitor")
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("retry_period_sec", 1.0)
        self.target = str(self.get_parameter("target_node").value).rstrip("/")
        self.timeout = float(self.get_parameter("request_timeout_sec").value)
        self.retry_period = float(self.get_parameter("retry_period_sec").value)
        if self.timeout <= 0.0 or self.retry_period <= 0.0:
            raise ValueError("lifecycle activator timing must be positive")
        self.get_client = self.create_client(GetState, f"{self.target}/get_state")
        self.change_client = self.create_client(
            ChangeState, f"{self.target}/change_state"
        )
        self.future = None
        self.operation = ""
        self.deadline = 0.0
        self.next_attempt = 0.0
        self.last_report = ""
        self.create_timer(0.20, self._tick)

    def _report(self, value, *, error=False):
        if value == self.last_report:
            return
        self.last_report = value
        if error:
            self.get_logger().error(value)
        else:
            self.get_logger().info(value)

    def _request_state(self):
        self.future = self.get_client.call_async(GetState.Request())
        self.operation = "get"
        self.deadline = time.monotonic() + self.timeout

    def _request_transition(self, transition_id, label):
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        self.future = self.change_client.call_async(request)
        self.operation = label
        self.deadline = time.monotonic() + self.timeout
        self._report(f"{self.target}: requesting {label}")

    def _retry(self, reason):
        self._report(f"{self.target}: {reason}; retrying", error=True)
        self.future = None
        self.operation = ""
        self.next_attempt = time.monotonic() + self.retry_period

    def _handle_state(self, state_id):
        if state_id == State.PRIMARY_STATE_ACTIVE:
            self._report(f"{self.target}: ACTIVE")
            self.future = None
            self.operation = ""
            self.next_attempt = time.monotonic() + self.retry_period
        elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
            self._request_transition(Transition.TRANSITION_CONFIGURE, "configure")
        elif state_id == State.PRIMARY_STATE_INACTIVE:
            self._request_transition(Transition.TRANSITION_ACTIVATE, "activate")
        else:
            self._retry(f"unsupported lifecycle state {state_id}")

    def _tick(self):
        now = time.monotonic()
        if self.future is None:
            if now < self.next_attempt:
                return
            if not self.get_client.service_is_ready() or not self.change_client.service_is_ready():
                self._report(f"{self.target}: waiting for lifecycle services")
                self.next_attempt = now + self.retry_period
                return
            self._request_state()
            return
        if not self.future.done():
            if now > self.deadline:
                self._retry(f"{self.operation} response timed out")
            return
        try:
            response = self.future.result()
        except Exception as error:
            self._retry(f"{self.operation} request failed: {error}")
            return
        operation = self.operation
        self.future = None
        self.operation = ""
        if operation == "get":
            self._handle_state(int(response.current_state.id))
            return
        if response is None or not response.success:
            self._retry(f"{operation} transition was rejected")
            return
        self.next_attempt = now + 0.20


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
