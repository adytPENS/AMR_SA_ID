#!/usr/bin/env python3
"""Teleop keyboard aman untuk drivetrain 4 motor Titan.

Pemetaan fisik:
  M0 = depan kanan,   positif = CCW
  M1 = belakang kanan, positif = CCW
  M2 = depan kiri
  M3 = belakang kiri

Kontrol:
  W = maju, S = mundur, A = putar kiri, D = putar kanan
  E = stop langsung, Q = stop dan keluar

Terminal tidak menyediakan event key-release. Saat tombol ditahan, sistem
operasi mengirim key-repeat; bila repeat berhenti selama release_timeout,
program otomatis mengirim nol ke seluruh motor.
"""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TitanKeyboardTeleop(Node):
    def __init__(self, sensor: str) -> None:
        super().__init__('titan_keyboard_teleop')
        self.motor_publishers = [
            self.create_publisher(Float64, f'/{sensor}/m_{motor}/cmd', 1)
            for motor in range(4)
        ]

    def command(self, speeds) -> None:
        for publisher, speed in zip(self.motor_publishers, speeds):
            msg = Float64()
            msg.data = float(speed)
            publisher.publish(msg)

    def stop(self) -> None:
        # Ulangi agar STOP tetap diterima ketika DDS baru selesai discovery.
        for _ in range(5):
            self.command((0.0, 0.0, 0.0, 0.0))
            rclpy.spin_once(self, timeout_sec=0.02)


def parse_args():
    parser = argparse.ArgumentParser(description='Teleop WASD Titan 4 motor')
    parser.add_argument('--sensor', default='titan0', help='nama Titan di YAML')
    parser.add_argument('--duty', type=float, default=0.15,
                        help='besar duty 0.05..0.30 (default 0.15)')
    parser.add_argument('--release-timeout', type=float, default=0.65,
                        help='stop setelah tidak ada key-repeat (default 0.65 s)')
    args, _ = parser.parse_known_args()
    if not 0.05 <= args.duty <= 0.30:
        parser.error('--duty harus antara 0.05 dan 0.30')
    if not 0.10 <= args.release_timeout <= 1.50:
        parser.error('--release-timeout harus antara 0.10 dan 1.50 detik')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TitanKeyboardTeleop(args.sensor)
    old_terminal = termios.tcgetattr(sys.stdin)

    duty = args.duty
    # Hasil uji robot: motor kanan maju = negatif dan motor kiri maju = positif.
    # A/D memutar kedua sisi dengan arah linear berlawanan.
    key_commands = {
        'w': (-duty, -duty,  duty,  duty),
        's': ( duty,  duty, -duty, -duty),
        'a': (-duty, -duty, -duty, -duty),
        'd': ( duty,  duty,  duty,  duty),
    }

    active_key = None
    last_key_time = 0.0
    last_sent = None

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info(
            'W maju | S mundur | A kiri | D kanan | E stop | Q keluar')
        node.get_logger().info(
            f'duty={duty:.2f}; lepas tombol -> stop maksimal '
            f'{args.release_timeout:.2f} detik')

        # Tunggu discovery sebelum menerima perintah gerak.
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            node.command((0.0, 0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.02)

        while rclpy.ok():
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in key_commands:
                    active_key = key
                    last_key_time = now
                elif key == 'e':
                    active_key = None
                    last_key_time = 0.0
                    node.stop()
                    last_sent = None
                    node.get_logger().info('STOP')
                elif key == 'q':
                    break

            if active_key and now - last_key_time <= args.release_timeout:
                command = key_commands[active_key]
            else:
                active_key = None
                command = (0.0, 0.0, 0.0, 0.0)

            # Publish berulang pada sekitar 50 Hz untuk watchdog Titan.
            node.command(command)
            if command != last_sent:
                label = active_key.upper() if active_key else 'STOP (key released)'
                node.get_logger().info(label)
                last_sent = command
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        node.get_logger().warning('Dihentikan dengan Ctrl+C')
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        node.get_logger().info('STOP — semua motor nol')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
