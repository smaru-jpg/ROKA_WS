import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TfFlatten2D(Node):
    def __init__(self) -> None:
        super().__init__("tf_flatten_2d")

        self.declare_parameter("source_parent_frame", "map")
        self.declare_parameter("source_child_frame", "body")
        self.declare_parameter("flattened_child_frame", "body_2d")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("flatten_translation_z", True)
        self.declare_parameter("flatten_orientation", True)

        self.source_parent_frame = self.get_parameter("source_parent_frame").value
        self.source_child_frame = self.get_parameter("source_child_frame").value
        self.flattened_child_frame = self.get_parameter("flattened_child_frame").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.flatten_translation_z = bool(self.get_parameter("flatten_translation_z").value)
        self.flatten_orientation = bool(self.get_parameter("flatten_orientation").value)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.transform_available = None
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.timer_cb)

        self.get_logger().info(
            f"tf_flatten_2d: {self.source_parent_frame} -> {self.source_child_frame} "
            f"-> {self.source_parent_frame} -> {self.flattened_child_frame}"
        )

    def timer_cb(self) -> None:
        try:
            source_tf = self.buffer.lookup_transform(
                self.source_parent_frame,
                self.source_child_frame,
                Time(),
            )
        except TransformException as exc:
            if self.transform_available is not False:
                self.get_logger().warning(
                    f"waiting for transform {self.source_parent_frame} -> {self.source_child_frame}: {exc}"
                )
            self.transform_available = False
            return
        if self.transform_available is False:
            self.get_logger().info(
                f"transform recovered: {self.source_parent_frame} -> {self.source_child_frame}"
            )
        self.transform_available = True

        flattened = TransformStamped()
        # Re-broadcast as a derived helper transform at the current time.
        flattened.header.stamp = self.get_clock().now().to_msg()
        flattened.header.frame_id = self.source_parent_frame
        flattened.child_frame_id = self.flattened_child_frame
        flattened.transform.translation.x = source_tf.transform.translation.x
        flattened.transform.translation.y = source_tf.transform.translation.y
        flattened.transform.translation.z = 0.0 if self.flatten_translation_z else source_tf.transform.translation.z

        if self.flatten_orientation:
            q = source_tf.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            half_yaw = yaw * 0.5
            flattened.transform.rotation.x = 0.0
            flattened.transform.rotation.y = 0.0
            flattened.transform.rotation.z = math.sin(half_yaw)
            flattened.transform.rotation.w = math.cos(half_yaw)
        else:
            flattened.transform.rotation.x = source_tf.transform.rotation.x
            flattened.transform.rotation.y = source_tf.transform.rotation.y
            flattened.transform.rotation.z = source_tf.transform.rotation.z
            flattened.transform.rotation.w = source_tf.transform.rotation.w

        self.broadcaster.sendTransform(flattened)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfFlatten2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
