#!/usr/bin/env python3
"""Tes singkat motor fisik M1 dan quadrature encoder pada Titan.

Label fisik Titan sama dengan indeks driver: M1 menggunakan topic m_1.
Motor dijalankan open-loop dengan duty rendah, lalu selalu dihentikan.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from studica_control.srv import SetData


class TitanM1Test(Node):
    def __init__(self, sensor: str, motor: int) -> None:
        super().__init__('titan_m1_test')
        prefix = f'/{sensor}/m_{motor}'
        self.command_pub = self.create_publisher(Float64, f'{prefix}/cmd', 10)
        self.create_subscription(Float64, f'{prefix}/encoder', self._on_encoder, 10)
        self.create_subscription(Float64, f'{prefix}/rpm', self._on_rpm, 10)
        self.client = self.create_client(SetData, f'/{sensor}/titan_cmd')
        self.encoder = None
        self.rpm = None

    def _on_encoder(self, msg: Float64) -> None:
        self.encoder = msg.data

    def _on_rpm(self, msg: Float64) -> None:
        self.rpm = msg.data

    def publish_speed(self, duty: float) -> None:
        msg = Float64()
        msg.data = float(duty)
        self.command_pub.publish(msg)

    def reset_encoder(self, motor: int) -> bool:
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service titan_cmd tidak ditemukan. Apakah launch sudah berjalan?')
            return False

        request = SetData.Request()
        request.params = 'reset_encoder'
        request.initparams.n_encoder = motor
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            message = response.message if response else 'tidak ada respons'
            self.get_logger().error(f'Gagal reset encoder: {message}')
            return False
        return True

    def stop(self) -> None:
        # Kirim beberapa kali agar perintah nol diterima meski discovery baru selesai.
        for _ in range(5):
            self.publish_speed(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def parse_args():
    parser = argparse.ArgumentParser(description='Tes motor M1 dan encoder Titan')
    parser.add_argument('--sensor', default='titan0', help='nama Titan di YAML')
    parser.add_argument('--motor', type=int, default=1,
                        help='indeks/label motor 0..3; M1 = 1 (default)')
    parser.add_argument('--duty', type=float, default=0.15,
                        help='daya motor -1.0..1.0 (default: 0.15)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='durasi bergerak dalam detik (default: 3)')
    args, _ = parser.parse_known_args()
    if not 0 <= args.motor <= 3:
        parser.error('--motor harus 0 sampai 3')
    if not -0.3 <= args.duty <= 0.3:
        parser.error('--duty dibatasi antara -0.3 dan 0.3 untuk keselamatan tes')
    if not 0.1 <= args.duration <= 10.0:
        parser.error('--duration harus 0.1 sampai 10 detik')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TitanM1Test(args.sensor, args.motor)

    try:
        node.get_logger().info('Menunggu koneksi ke Titan component...')
        if not node.reset_encoder(args.motor):
            return

        # Beri waktu publisher/subscriber melakukan discovery dan menerima feedback.
        wait_until = time.monotonic() + 2.0
        while time.monotonic() < wait_until and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info(
            f'Menjalankan M{args.motor} (m_{args.motor}) duty={args.duty:.2f} '
            f'selama {args.duration:.1f} detik')
        started = time.monotonic()
        next_log = started
        while rclpy.ok() and time.monotonic() - started < args.duration:
            node.publish_speed(args.duty)
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_log:
                enc = 'belum ada data' if node.encoder is None else f'{node.encoder:.0f} tick'
                rpm = 'belum ada data' if node.rpm is None else f'{node.rpm:.1f} rpm'
                node.get_logger().info(f'encoder={enc}, rpm={rpm}')
                next_log += 0.5

    except KeyboardInterrupt:
        node.get_logger().warning('Tes dihentikan oleh pengguna')
    finally:
        node.stop()
        node.get_logger().info(
            f'STOP — hasil akhir: encoder={node.encoder}, rpm={node.rpm}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
