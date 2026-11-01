import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from roka_mission.traffic_signal_route_gate import TrafficSignalRouteGate


def spin_until(executor, predicate, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_red_zeroes_and_stops_then_confirmed_blue_restarts_route():
    rclpy.init()
    harness = Node("traffic_signal_route_gate_test_harness")
    calls = {"stop": 0, "start": 0, "zero": 0}

    def stop_route(_request, response):
        calls["stop"] += 1
        response.success = True
        response.message = "stopped"
        return response

    def start_route(_request, response):
        calls["start"] += 1
        response.success = True
        response.message = "started"
        return response

    harness.create_service(Trigger, "/track_path_follower/stop", stop_route)
    harness.create_service(Trigger, "/track_path_follower/start", start_route)
    harness.create_subscription(
        Twist,
        "/cmd_vel_mission",
        lambda message: calls.__setitem__(
            "zero",
            calls["zero"]
            + int(message.linear.x == 0.0 and message.angular.z == 0.0),
        ),
        10,
    )
    result_pub = harness.create_publisher(String, "/roka/vision/result", 10)
    latched_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    autonomy_status_pub = harness.create_publisher(
        String, "/autonomy/control_status", latched_qos
    )
    gate = TrafficSignalRouteGate()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(gate)

    try:
        assert spin_until(executor, lambda: result_pub.get_subscription_count() == 1)
        result_pub.publish(String(data="RED"))
        assert spin_until(
            executor,
            lambda: calls["stop"] == 1 and calls["zero"] > 0 and gate.motion_blocked,
        )

        # Loss of sight does not implicitly permit motion after RED.
        result_pub.publish(String(data="NONE"))
        for _ in range(10):
            executor.spin_once(timeout_sec=0.02)
        assert calls["start"] == 0
        assert gate.motion_blocked

        result_pub.publish(String(data="BLUE"))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert calls["start"] == 0
        result_pub.publish(String(data="BLUE"))
        assert spin_until(
            executor,
            lambda: calls["start"] == 1 and not gate.motion_blocked,
        )

        # Once B is pressed, later signal changes may not re-arm autonomy.
        result_pub.publish(String(data="RED"))
        assert spin_until(executor, lambda: calls["stop"] >= 2 and gate.motion_blocked)
        autonomy_status_pub.publish(String(data="B STOP: cancellation requested"))
        assert spin_until(executor, lambda: gate.manual_takeover)
        result_pub.publish(String(data="BLUE"))
        result_pub.publish(String(data="BLUE"))
        for _ in range(20):
            executor.spin_once(timeout_sec=0.02)
        assert calls["start"] == 1
        assert gate.motion_blocked
    finally:
        executor.remove_node(gate)
        executor.remove_node(harness)
        gate.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
