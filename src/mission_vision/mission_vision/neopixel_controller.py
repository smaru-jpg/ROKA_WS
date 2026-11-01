from pathlib import Path
import time

import rclpy
import serial
import yaml

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


COMMAND_BYTES = {
    'ENEMY': b'E',
    'FRIEND': b'F',
    'BOTH': b'B',
    'NONE': b'N',
}


def normalize_led_state(raw_result):
    values = {
        value.strip().upper()
        for value in str(raw_result).split(',')
        if value.strip()
    }
    if values == {'ENEMY', 'FRIEND'}:
        return 'BOTH'
    if len(values) == 1:
        result = next(iter(values))
        if result in COMMAND_BYTES:
            return result
    return None


class NeoPixelController(Node):

    def __init__(self):
        super().__init__('neopixel_controller')

        self.declare_parameter(
            'config_file',
            'config/spring.yaml'
        )
        self.declare_parameter('enabled', False)
        self.declare_parameter('serial_port', '')

        package_share = Path(
            get_package_share_directory('mission_vision')
        )

        config_param = str(
            self.get_parameter('config_file').value
        )

        config_path = Path(config_param)

        if not config_path.is_absolute():
            config_path = package_share / config_path

        with config_path.open(
            'r',
            encoding='utf-8'
        ) as f:
            config = yaml.safe_load(f) or {}

        led_config = config.get('led', {})

        configured_enabled = bool(led_config.get('enabled', False))
        self.enabled = bool(self.get_parameter('enabled').value) or configured_enabled

        override_port = str(self.get_parameter('serial_port').value).strip()
        self.port_name = override_port or str(led_config.get('port', '')).strip()

        self.baud_rate = int(
            led_config.get('baud_rate', 115200)
        )
        self.startup_delay_sec = max(
            0.0,
            float(led_config.get('startup_delay_sec', 2.0)),
        )

        self.topic_name = str(
            led_config.get('topic', '/roka/vision/result')
        )

        self.serial_port = None
        self.last_state = None

        if not self.enabled:
            self.get_logger().info(
                'LED controller disabled by YAML'
            )
            return

        if not self.port_name.startswith('/dev/serial/by-id/'):
            raise RuntimeError(
                'LED enabled but serial_port is not a stable /dev/serial/by-id path'
            )

        try:
            self.serial_port = serial.Serial(
                self.port_name,
                self.baud_rate,
                timeout=1
            )
            # Opening a Nano USB serial port normally resets the ATmega328P.
            # Wait for its bootloader/setup before accepting the first IFF
            # result, otherwise a lost first byte would be hidden by the
            # duplicate-state suppression below.
            time.sleep(self.startup_delay_sec)
            self.serial_port.reset_input_buffer()
            self.serial_port.write(COMMAND_BYTES['NONE'])
            self.serial_port.flush()

            self.get_logger().info(
                f'Arduino connected: {self.port_name}'
            )

        except Exception as e:
            raise RuntimeError(
                f'LED serial connection failed ({self.port_name}): {e}'
            ) from e

        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.result_callback,
            10
        )

    def result_callback(self, msg):

        if not self.enabled:
            return

        raw_result = msg.data
        result = normalize_led_state(raw_result)

        if result is None:
            self.get_logger().warning(
                f'Unknown vision result: {raw_result}'
            )
            return

        if result == self.last_state:
            return

        self.last_state = result

        if self.serial_port is not None:
            self.serial_port.write(
                COMMAND_BYTES[result]
            )

            self.get_logger().info(
                f'LED state -> {result}'
            )

    def destroy_node(self):

        if self.serial_port is not None:
            try:
                self.serial_port.write(b'N')
                self.serial_port.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = NeoPixelController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'[MISSION-LED][ERROR] {error}', flush=True)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
