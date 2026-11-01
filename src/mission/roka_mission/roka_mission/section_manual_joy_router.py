#!/usr/bin/env python3
"""Route Xbox face-button approvals for section-specific manual driving."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


def normalize_section(value: str) -> str:
    normalized = str(value).strip().lower().replace("-", "_")
    if normalized.startswith("section_"):
        normalized = "section" + normalized[len("section_"):]
    if normalized not in {f"section{number}" for number in range(1, 6)}:
        raise ValueError("section must be section1..section5")
    return normalized


def rising_edge(previous, current, index: int) -> bool:
    before = bool(previous[index]) if 0 <= index < len(previous) else False
    now = bool(current[index]) if 0 <= index < len(current) else False
    return now and not before


class SectionManualJoyRouter(Node):
    def __init__(self):
        super().__init__("section_manual_joy_router")
        self.section = normalize_section(
            self.declare_parameter("section", "section1").value
        )
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.a_button = int(self.declare_parameter("a_button", 0).value)
        self.b_button = int(self.declare_parameter("b_button", 1).value)
        self.x_button = int(self.declare_parameter("x_button", 2).value)
        self.y_button = int(self.declare_parameter("y_button", 3).value)
        self.rb_button = int(self.declare_parameter("rb_button", 5).value)
        if min(
            self.a_button,
            self.b_button,
            self.x_button,
            self.y_button,
            self.rb_button,
        ) < 0:
            raise ValueError("Xbox button indexes cannot be negative")

        self.service_clients = {
            "A/SECTION2_START": self.create_client(
                Trigger, "/roka/arm/pose/section2_start"
            ),
            "X/SECTION1": self.create_client(
                Trigger, "/roka/arm/pose/section1"
            ),
            "Y/SECTION2_DRIVE": self.create_client(
                Trigger, "/roka/arm/pose/section2_drive"
            ),
        }
        if self.section == "section2":
            self.service_clients["B/BOX_PICK"] = self.create_client(
                Trigger, "/roka/mission/box_pick/trigger"
            )
        self.previous_buttons = []
        self.buttons_armed = False
        self.pending = set()
        self.create_subscription(
            Joy, self.joy_topic, self._joy, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"section manual buttons ready: section={self.section}; "
            "A=SECTION2_START, X=SECTION1, Y=SECTION2_DRIVE, "
            f"B={'BOX_PICK' if self.section == 'section2' else 'unused'}"
        )

    def _call(self, label):
        if label in self.pending:
            return
        client = self.service_clients.get(label)
        if client is None:
            return
        if not client.service_is_ready():
            self.get_logger().warning(f"{label} rejected: service is not ready")
            return
        self.pending.add(label)
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed, selected=label: self._completed(selected, completed)
        )

    def _completed(self, label, future):
        self.pending.discard(label)
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"{label} service failed: {error}")
            return
        if response.success:
            self.get_logger().info(f"{label}: {response.message}")
        else:
            self.get_logger().warning(f"{label}: {response.message}")

    def _joy(self, message: Joy):
        buttons = list(message.buttons)
        if not self.buttons_armed:
            self.buttons_armed = not any(buttons)
            self.previous_buttons = buttons
            return
        rb_pressed = (
            self.rb_button < len(buttons) and bool(buttons[self.rb_button])
        )
        # RB owns manual arm motion. Ignore face-button pose shortcuts while
        # it is held so a grip change cannot accidentally start a full pose.
        if not rb_pressed:
            if rising_edge(self.previous_buttons, buttons, self.a_button):
                self._call("A/SECTION2_START")
            if rising_edge(self.previous_buttons, buttons, self.x_button):
                self._call("X/SECTION1")
            if rising_edge(self.previous_buttons, buttons, self.y_button):
                self._call("Y/SECTION2_DRIVE")
            if (
                self.section == "section2"
                and rising_edge(self.previous_buttons, buttons, self.b_button)
            ):
                self._call("B/BOX_PICK")
        self.previous_buttons = buttons


def main(args=None):
    rclpy.init(args=args)
    node = SectionManualJoyRouter()
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
            rclpy.shutdown()


if __name__ == "__main__":
    main()
