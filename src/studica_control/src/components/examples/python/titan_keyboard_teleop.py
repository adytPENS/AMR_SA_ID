#!/usr/bin/env python3
"""Teleop keyboard aman untuk drivetrain 4 motor Titan.

Pemetaan fisik:
  M0 = depan kanan,   positif = CCW
  M1 = belakang kanan, positif = CCW
  M2 = depan kiri
  M3 = belakang kiri

Kontrol:
  W = maju, S = mundur, A = putar kiri, D = putar kanan
  G = maju otomatis sejauh target encoder (default 1 meter)
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
        self.encoder_values = [None] * 4
        self.encoder_subscriptions = [
            self.create_subscription(
                Float64,
                f'/{sensor}/m_{motor}/encoder',
                lambda msg, index=motor: self.encoder_callback(index, msg),
                10,
            )
            for motor in range(4)
        ]

    def encoder_callback(self, motor: int, msg: Float64) -> None:
        self.encoder_values[motor] = float(msg.data)

    def encoders_ready(self) -> bool:
        return all(value is not None for value in self.encoder_values)

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
    parser.add_argument('--turn-duty', type=float, default=None,
                        help='duty khusus A/D; default sama dengan --duty')
    parser.add_argument('--release-timeout', type=float, default=0.65,
                        help='stop setelah tidak ada key-repeat (default 0.65 s)')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='target tombol G dalam meter (default 1.0)')
    parser.add_argument('--distance-timeout', type=float, default=20.0,
                        help='safety timeout gerak G dalam detik (default 20)')
    args, _ = parser.parse_known_args()
    if not 0.05 <= args.duty <= 0.30:
        parser.error('--duty harus antara 0.05 dan 0.30')
    if args.turn_duty is None:
        args.turn_duty = args.duty
    if not 0.05 <= args.turn_duty <= 0.30:
        parser.error('--turn-duty harus antara 0.05 dan 0.30')
    if not 0.10 <= args.release_timeout <= 1.50:
        parser.error('--release-timeout harus antara 0.10 dan 1.50 detik')
    if not 0.05 <= args.distance <= 5.0:
        parser.error('--distance harus antara 0.05 dan 5.0 meter')
    if not 2.0 <= args.distance_timeout <= 60.0:
        parser.error('--distance-timeout harus antara 2 dan 60 detik')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TitanKeyboardTeleop(args.sensor)
    old_terminal = termios.tcgetattr(sys.stdin)

    duty = args.duty
    turn_duty = args.turn_duty
    # Hasil uji robot: motor kanan maju = negatif dan motor kiri maju = positif.
    # A/D memutar kedua sisi dengan arah linear berlawanan.
    key_commands = {
        'w': (-duty, -duty,  duty,  duty),
        's': ( duty,  duty, -duty, -duty),
        'a': (-turn_duty, -turn_duty, -turn_duty, -turn_duty),
        'd': ( turn_duty,  turn_duty,  turn_duty,  turn_duty),
    }

    active_key = None
    last_key_time = 0.0
    last_sent = None
    distance_active = False
    distance_start = None
    distance_start_time = 0.0
    last_progress = 0.0
    last_progress_time = 0.0
    next_progress_log = 0.10

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info(
            'W maju | S mundur | A kiri | D kanan | '
            'G maju target | E stop | Q keluar')
        node.get_logger().info(
            f'duty maju={duty:.2f}; duty putar={turn_duty:.2f}; '
            f'lepas tombol -> stop maksimal '
            f'{args.release_timeout:.2f} detik; target G={args.distance:.2f} m')

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
                if key in key_commands and not distance_active:
                    active_key = key
                    last_key_time = now
                elif key == 'g' and not distance_active:
                    if not node.encoders_ready():
                        node.get_logger().error(
                            'G ditolak: data empat encoder belum tersedia')
                    else:
                        active_key = None
                        distance_active = True
                        distance_start = list(node.encoder_values)
                        distance_start_time = now
                        last_progress = 0.0
                        last_progress_time = now
                        next_progress_log = 0.10
                        node.get_logger().info(
                            f'G: maju otomatis {args.distance:.2f} m')
                elif key == 'e':
                    active_key = None
                    distance_active = False
                    last_key_time = 0.0
                    node.stop()
                    last_sent = None
                    node.get_logger().info('STOP / target dibatalkan')
                elif key == 'q':
                    break

            if distance_active:
                distances = [
                    abs(current - start)
                    for current, start in zip(node.encoder_values, distance_start)
                ]
                progress = sum(distances) / 4.0

                if progress >= args.distance:
                    distance_active = False
                    node.stop()
                    last_sent = None
                    values = ', '.join(
                        f'M{i}={value:.3f}m'
                        for i, value in enumerate(distances)
                    )
                    node.get_logger().info(
                        f'TARGET TERCAPAI: rata-rata={progress:.3f}m; '
                        f'{values}; STOP')
                elif now - distance_start_time >= args.distance_timeout:
                    distance_active = False
                    node.stop()
                    last_sent = None
                    node.get_logger().error(
                        f'SAFETY TIMEOUT pada {progress:.3f}m; STOP')
                elif now - last_progress_time >= 1.5:
                    distance_active = False
                    node.stop()
                    last_sent = None
                    node.get_logger().error(
                        f'ENCODER STALL pada {progress:.3f}m; STOP')
                else:
                    if progress >= last_progress + 0.002:
                        last_progress = progress
                        last_progress_time = now
                    if progress >= next_progress_log:
                        node.get_logger().info(
                            f'G progress: {progress:.3f}/{args.distance:.3f} m')
                        next_progress_log += 0.10
                    command = key_commands['w']

            if distance_active:
                pass
            elif active_key and now - last_key_time <= args.release_timeout:
                command = key_commands[active_key]
            else:
                active_key = None
                command = (0.0, 0.0, 0.0, 0.0)

            # Publish berulang pada sekitar 50 Hz untuk watchdog Titan.
            node.command(command)
            if command != last_sent:
                if distance_active:
                    label = f'G ({args.distance:.2f} m)'
                else:
                    label = active_key.upper() if active_key else 'STOP'
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
