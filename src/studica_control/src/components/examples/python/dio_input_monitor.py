#!/usr/bin/env python3
"""Monitor aman digital input ROS tanpa mengendalikan aktuator."""

import argparse
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class DioInputMonitor(Node):
    def __init__(self, topic: str, active_low: bool) -> None:
        super().__init__('dio_input_monitor')
        self.topic = topic
        self.active_low = active_low
        self.last_raw = None
        self.last_message_time = 0.0
        self.press_count = 0
        self.warned_missing = False
        self.create_subscription(Bool, topic, self.callback, 10)
        self.create_timer(1.0, self.health_check)
        self.get_logger().info(
            f'Monitoring {topic}; active_low={active_low}; '
            'program ini tidak mengendalikan motor')

    def callback(self, msg: Bool) -> None:
        now = time.monotonic()
        raw = bool(msg.data)
        self.last_message_time = now
        self.warned_missing = False
        if raw == self.last_raw:
            return
        pressed = not raw if self.active_low else raw
        if pressed:
            self.press_count += 1
        electrical = 'HIGH' if raw else 'LOW'
        state = 'PRESSED' if pressed else 'RELEASED'
        self.get_logger().info(
            f'raw={str(raw).lower()} ({electrical}) -> {state}; '
            f'press_count={self.press_count}')
        self.last_raw = raw

    def health_check(self) -> None:
        now = time.monotonic()
        if (self.last_message_time == 0.0 or
                now - self.last_message_time > 1.0):
            if not self.warned_missing:
                self.get_logger().error(
                    f'Tidak ada data dari {self.topic}; periksa control_server, '
                    'konfigurasi DIO, dan wiring')
                self.warned_missing = True


def parse_args():
    parser = argparse.ArgumentParser(description='Monitor digital input ROS')
    parser.add_argument('--topic', default='/start_button/state')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--active-low', dest='active_low', action='store_true')
    mode.add_argument('--active-high', dest='active_low', action='store_false')
    parser.set_defaults(active_low=True)
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = DioInputMonitor(args.topic, args.active_low)
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
