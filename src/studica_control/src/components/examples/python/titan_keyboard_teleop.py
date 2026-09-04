#!/usr/bin/env python3
"""Keyboard AMR dan OMS: /cmd_vel base serta motor Titan kedua."""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class KeyboardCmdVel(Node):
    def __init__(self, sensor: str, oms_sensor: str) -> None:
        super().__init__('keyboard_cmd_vel')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.oms_lift_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_2/cmd', 10)
        self.oms_rotate_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_3/cmd', 10)
        self.encoder_values = [None] * 4
        self.encoder_subscriptions = [
            self.create_subscription(
                Float64, f'/{sensor}/m_{motor}/encoder',
                lambda msg, index=motor: self.encoder_callback(index, msg), 10)
            for motor in range(4)
        ]

    def encoder_callback(self, motor: int, msg: Float64) -> None:
        self.encoder_values[motor] = float(msg.data)

    def encoders_ready(self) -> bool:
        return all(value is not None for value in self.encoder_values)

    def publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_publisher.publish(msg)

    def publish_oms(self, lift: float, rotate: float) -> None:
        self.oms_lift_publisher.publish(Float64(data=float(lift)))
        self.oms_rotate_publisher.publish(Float64(data=float(rotate)))

    def stop(self) -> None:
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            self.publish_oms(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard /cmd_vel AMR')
    parser.add_argument('--sensor', default='titan0')
    parser.add_argument('--oms-sensor', default='titan1')
    parser.add_argument('--oms-speed', type=float, default=0.20)
    parser.add_argument('--lift-speed', type=float, default=None)
    parser.add_argument('--rotate-speed', type=float, default=None)
    parser.add_argument('--lift-polarity', type=float, choices=(-1.0, 1.0),
                        default=1.0)
    parser.add_argument('--rotate-polarity', type=float, choices=(-1.0, 1.0),
                        default=1.0)
    parser.add_argument('--linear-speed', type=float, default=0.35)
    parser.add_argument('--angular-speed', type=float, default=1.5)
    parser.add_argument('--release-timeout', type=float, default=0.65)
    parser.add_argument('--distance', type=float, default=1.0)
    parser.add_argument('--distance-timeout', type=float, default=20.0)
    args, _ = parser.parse_known_args()
    if not 0.02 <= args.linear_speed <= 0.75:
        parser.error('--linear-speed harus 0.02..0.75 m/s')
    if not 0.10 <= args.angular_speed <= 5.0:
        parser.error('--angular-speed harus 0.10..5.0 rad/s')
    if not 0.10 <= args.release_timeout <= 1.50:
        parser.error('--release-timeout harus 0.10..1.50 detik')
    if not 0.05 <= args.oms_speed <= 0.60:
        parser.error('--oms-speed harus 0.05..0.60 duty')
    args.lift_speed = (
        args.oms_speed if args.lift_speed is None else args.lift_speed)
    args.rotate_speed = (
        args.oms_speed if args.rotate_speed is None else args.rotate_speed)
    if not 0.05 <= args.lift_speed <= 0.60:
        parser.error('--lift-speed harus 0.05..0.60 duty')
    if not 0.05 <= args.rotate_speed <= 0.60:
        parser.error('--rotate-speed harus 0.05..0.60 duty')
    if not 0.05 <= args.distance <= 5.0:
        parser.error('--distance harus 0.05..5.0 meter')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = KeyboardCmdVel(args.sensor, args.oms_sensor)
    old_terminal = termios.tcgetattr(sys.stdin)
    key_commands = {
        'w': (args.linear_speed, 0.0),
        's': (-args.linear_speed, 0.0),
        'a': (0.0, args.angular_speed),
        'd': (0.0, -args.angular_speed),
    }
    oms_commands = {
        'i': (args.lift_speed * args.lift_polarity, 0.0),       # naik
        'k': (-args.lift_speed * args.lift_polarity, 0.0),      # turun
        'j': (0.0, args.rotate_speed * args.rotate_polarity),   # CCW
        'l': (0.0, -args.rotate_speed * args.rotate_polarity),  # CW
    }
    active_key = None
    last_key_time = 0.0
    last_label = None
    distance_active = False
    distance_start = None
    distance_started = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info(
            'BASE: W maju | S mundur | A kiri | D kanan | G target')
        node.get_logger().info(
            'OMS: I naik | K turun | J CCW | L CW | E stop semua | Q keluar')
        node.get_logger().info(
            f'/cmd_vel linear={args.linear_speed:.2f}m/s, '
            f'angular={args.angular_speed:.2f}rad/s')
        node.get_logger().info(
            f'OMS lift duty={args.lift_speed:.2f}, '
            f'rotate duty={args.rotate_speed:.2f}')
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            node.publish_cmd(0.0, 0.0)
            node.publish_oms(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

        while rclpy.ok():
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in key_commands and not distance_active:
                    active_key = key
                    last_key_time = now
                elif key in oms_commands and not distance_active:
                    active_key = key
                    last_key_time = now
                elif key == 'g' and not distance_active:
                    if not node.encoders_ready():
                        node.get_logger().error('G ditolak: encoder belum lengkap')
                    else:
                        active_key = None
                        distance_active = True
                        distance_start = list(node.encoder_values)
                        distance_started = now
                elif key == 'e':
                    active_key = None
                    distance_active = False
                    node.stop()
                    last_label = None
                elif key == 'q':
                    break

            if distance_active:
                distances = [abs(current - start) for current, start in
                             zip(node.encoder_values, distance_start)]
                progress = sum(distances) / 4.0
                if progress >= args.distance:
                    distance_active = False
                    node.get_logger().info(f'G selesai {progress:.3f}m')
                    command = (0.0, 0.0)
                elif now - distance_started >= args.distance_timeout:
                    distance_active = False
                    node.get_logger().error('G safety timeout')
                    command = (0.0, 0.0)
                else:
                    command = (args.linear_speed, 0.0)
            elif active_key and now - last_key_time <= args.release_timeout:
                command = key_commands.get(active_key, (0.0, 0.0))
            else:
                active_key = None
                command = (0.0, 0.0)

            node.publish_cmd(*command)
            oms_command = (
                oms_commands[active_key]
                if active_key in oms_commands and not distance_active
                else (0.0, 0.0))
            node.publish_oms(*oms_command)
            label = 'G' if distance_active else (active_key.upper() if active_key else 'STOP')
            if label != last_label:
                node.get_logger().info(label)
                last_label = label
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        node.get_logger().info('STOP — /cmd_vel nol')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
