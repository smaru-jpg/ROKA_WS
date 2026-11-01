"""Latest-frame ROS Image camera backend.

The RealSense driver owns the D435.  This backend stores only the newest ROS
message and converts it to OpenCV at inference rate, avoiding a second camera
pipeline and avoiding conversion of frames that YOLO will not consume.
"""

import time

from cv_bridge import CvBridge
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class RosImageCamera:
    def __init__(self, config, node):
        self.bridge = CvBridge()
        self.logger = node.get_logger()
        self.latest = None
        self.last_stamp = None
        self.last_received_monotonic = None
        self.topic = str(
            config.get("topic", "/roka/arm_camera/color/image_raw")
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = node.create_subscription(
            Image, self.topic, self._receive, qos
        )

    def _receive(self, message):
        self.latest = message
        self.last_received_monotonic = time.monotonic()

    def age_seconds(self):
        if self.last_received_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self.last_received_monotonic)

    def read(self):
        message = self.latest
        if message is None:
            return None
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return None
        self.last_stamp = stamp
        try:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.logger.error(f"camera image conversion failed: {error}")
            return None

    def close(self):
        self.latest = None
