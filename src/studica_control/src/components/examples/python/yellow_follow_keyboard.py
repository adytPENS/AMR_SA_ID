#!/usr/bin/env python3
"""Keyboard teleop + yellow object following using the color tracker result."""

import json
import math
import select
import sys
import termios
import time
import tty
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def compute_auto_cmd(pixel_x: float, image_width: float, distance_m: Optional[float]) -> Tuple[float, float]:
    """Return linear and angular command for the current detected object."""
    if (not math.isfinite(image_width) or image_width <= 0.0 or
            not math.isfinite(pixel_x) or not 0.0 <= pixel_x < image_width):
        return 0.0, 0.0
    x_error = (image_width / 2.0 - float(pixel_x)) / (image_width / 2.0)
    angular_z = clamp(1.4 * x_error, -0.8, 0.8)
    if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
        return 0.0, angular_z

    if distance_m > 0.40:
        linear_x = clamp((distance_m - 0.40) * 1.2, 0.0, 0.35)
    elif distance_m < 0.35:
        linear_x = -clamp((0.35 - distance_m) * 1.2, 0.0, 0.30)
    else:
        linear_x = 0.0
    return linear_x, angular_z


class YellowFollower:
    """Shared tracking state; never reuse detections after the camera stops."""

    def __init__(self):
        self.latest_result = None
        self.received_at = 0.0
        self.status = 'menunggu /color_tracker/result'

    def result_callback(self, msg):
        self.latest_result = None
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self.latest_result = payload
            self.received_at = time.monotonic()

    def command(self, now=None):
        now = time.monotonic() if now is None else now
        if self.latest_result is None or now - self.received_at > 0.5:
            self.status = 'STOP: data tracker belum tersedia/kedaluwarsa'
            return 0.0, 0.0
        objects = self.latest_result.get('objects', [])
        if not isinstance(objects, list):
            objects = []
        self.status = 'STOP: kuning tidak terdeteksi'
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            name = str(obj.get('color') or obj.get('label', '')).strip().casefold()
            if name not in ('yellow', 'kuning'):
                continue
            try:
                x = float(obj['pixel_x'])
                width = float(self.latest_result['image_width'])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if (not all(math.isfinite(v) for v in (x, width)) or
                    width <= 0 or not 0 <= x < width):
                continue
            try:
                distance = float(obj.get('distance_m'))
            except (TypeError, ValueError, OverflowError):
                distance = None
            if distance is not None and (not math.isfinite(distance) or distance <= 0):
                distance = None
            self.status = ('mengikuti kuning' if distance is not None else
                           'kuning: hanya putar kanan/kiri (depth tidak tersedia)')
            return compute_auto_cmd(x, width, distance)
        return 0.0, 0.0


class YellowFollowKeyboard(Node):
    def __init__(self) -> None:
        super().__init__('yellow_follow_keyboard')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.result_subscription = self.create_subscription(
            String, '/color_tracker/result', self.result_callback, 1)
        self.auto_mode = False
        self.follower = YellowFollower()
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        self.manual_until = 0.0
        self.should_exit = False

        self._fd = sys.stdin.fileno()
        self._attr = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self.get_logger().info('Keyboard mode ready. z = auto follow, w/s/a/d = manual, x = stop, q = quit.')

        self.create_timer(0.05, self.update_loop)

    def result_callback(self, msg: String) -> None:
        self.follower.result_callback(msg)

    def _handle_key_press(self, ch: str) -> None:
        now = time.monotonic()
        if ch == 'q':
            self.should_exit = True
            return
        if ch == 'z':
            self.auto_mode = not self.auto_mode
            self.manual_until = 0.0
            self.manual_linear = 0.0
            self.manual_angular = 0.0
            self.get_logger().info(f'Auto follow mode: {self.auto_mode}')
            return
        if ch in ('w', 's', 'a', 'd', 'x', 'e'):
            self.auto_mode = False
        if ch == 'e':
            ch = 'x'
        if not self.auto_mode:
            if ch in ('w', 's', 'a', 'd', 'x'):
                self.manual_until = now + 0.18
                if ch == 'w':
                    self.manual_linear = 0.35
                    self.manual_angular = 0.0
                elif ch == 's':
                    self.manual_linear = -0.30
                    self.manual_angular = 0.0
                elif ch == 'a':
                    self.manual_linear = 0.0
                    self.manual_angular = 0.70
                elif ch == 'd':
                    self.manual_linear = 0.0
                    self.manual_angular = -0.70
                elif ch == 'x':
                    self.manual_linear = 0.0
                    self.manual_angular = 0.0

    def _poll_key(self) -> None:
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return
        ch = sys.stdin.read(1)
        if not ch:
            return
        self._handle_key_press(ch.lower())

    def _auto_twist(self) -> Tuple[float, float]:
        return self.follower.command()

    def update_loop(self) -> None:
        self._poll_key()
        if self.should_exit:
            self.publish_stop()
            raise SystemExit

        now = time.monotonic()
        if self.auto_mode:
            linear_x, angular_z = self._auto_twist()
        else:
            if now > self.manual_until:
                self.manual_linear = 0.0
                self.manual_angular = 0.0
            linear_x = self.manual_linear
            angular_z = self.manual_angular

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_publisher.publish(msg)

    def publish_stop(self) -> None:
        msg = Twist()
        self.cmd_publisher.publish(msg)

    def cleanup(self) -> None:
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._attr)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YellowFollowKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
