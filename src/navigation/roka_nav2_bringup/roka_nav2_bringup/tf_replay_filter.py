"""Republish recorded odom/body TF while letting a fresh SLAM own map->odom."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfReplayFilter(Node):
    def __init__(self) -> None:
        super().__init__("tf_replay_filter")
        self.declare_parameter("input_topic", "/replay/raw_tf")
        self.declare_parameter("output_topic", "/tf")
        self.declare_parameter("excluded_parent_frame", "map")
        self.declare_parameter("excluded_parent_frames", ["map", "lio_map"])
        legacy_excluded_parent = str(
            self.get_parameter("excluded_parent_frame").value
        ).lstrip("/")
        configured = {
            str(frame).lstrip("/")
            for frame in self.get_parameter("excluded_parent_frames").value
            if str(frame).strip()
        }
        configured.add(legacy_excluded_parent)
        self.excluded_parents = configured
        output_topic = str(self.get_parameter("output_topic").value)
        input_topic = str(self.get_parameter("input_topic").value)
        qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(TFMessage, output_topic, qos)
        self.subscription = self.create_subscription(
            TFMessage, input_topic, self._callback, qos
        )

    def _callback(self, message: TFMessage) -> None:
        filtered = [
            transform
            for transform in message.transforms
            if transform.header.frame_id.lstrip("/") not in self.excluded_parents
        ]
        if filtered:
            self.publisher.publish(TFMessage(transforms=filtered))


def main() -> None:
    rclpy.init()
    node = TfReplayFilter()
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
