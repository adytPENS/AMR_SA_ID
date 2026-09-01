#!/usr/bin/env python3
"""Monitor START active-low dan STOP active-high tanpa aktuator."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class StartStopButtonMonitor(Node):
    def __init__(self) -> None:
        super().__init__('start_stop_button_monitor')
        self.last_state = {'START': None, 'STOP': None}
        self.press_count = {'START': 0, 'STOP': 0}
        self.create_subscription(
            Bool, '/start_button/state',
            lambda msg: self.on_button('START', msg), 10)
        self.create_subscription(
            Bool, '/stop_button/state',
            lambda msg: self.on_button('STOP', msg), 10)
        self.get_logger().info(
            'Monitoring START=DIO10 active-low dan STOP=DIO11 active-high; '
            'motor nonaktif')

    def on_button(self, name: str, msg: Bool) -> None:
        raw = bool(msg.data)
        if raw == self.last_state[name]:
            return
        pressed = (not raw) if name == 'START' else raw
        if pressed:
            self.press_count[name] += 1
        electrical = 'HIGH' if raw else 'LOW'
        state = 'PRESSED' if pressed else 'RELEASED'
        self.get_logger().info(
            f'{name}: raw={str(raw).lower()} ({electrical}) -> {state}; '
            f'press_count={self.press_count[name]}')
        self.last_state[name] = raw


def main() -> None:
    rclpy.init()
    node = StartStopButtonMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
