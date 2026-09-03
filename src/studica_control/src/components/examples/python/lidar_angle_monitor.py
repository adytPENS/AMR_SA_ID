#!/usr/bin/env python3
"""Monitor rentang sudut dan sektor LaserScan tanpa menggerakkan robot."""

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_degrees(angle: float) -> float:
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))


class LidarAngleMonitor(Node):
    def __init__(self) -> None:
        super().__init__('lidar_angle_monitor')
        self.last_log_time = 0.0
        self.metadata_logged = False
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            'Menunggu /scan; depan ROS=0 deg, kiri=+90 deg, kanan=-90 deg')

    @staticmethod
    def sector_min(
            samples: List[Tuple[float, float]],
            start_deg: float, end_deg: float) -> float:
        distances = [
            distance for angle, distance in samples
            if start_deg <= angle <= end_deg
        ]
        return min(distances) if distances else math.inf

    def scan_callback(self, msg: LaserScan) -> None:
        if not self.metadata_logged:
            count = len(msg.ranges)
            end_angle = msg.angle_min + max(0, count - 1) * msg.angle_increment
            self.get_logger().info(
                'SCAN CONFIG: '
                f'{normalize_degrees(msg.angle_min):.1f} deg sampai '
                f'{normalize_degrees(end_angle):.1f} deg; '
                f'{count} sampel; increment='
                f'{math.degrees(msg.angle_increment):.3f} deg; '
                f'range={msg.range_min:.2f}..{msg.range_max:.2f} m')
            self.metadata_logged = True

        now = time.monotonic()
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now

        samples: List[Tuple[float, float]] = []
        for index, raw_distance in enumerate(msg.ranges):
            if not math.isfinite(raw_distance):
                continue
            distance = float(raw_distance)
            if not msg.range_min <= distance <= msg.range_max:
                continue
            angle = normalize_degrees(
                msg.angle_min + index * msg.angle_increment)
            samples.append((angle, distance))

        if not samples:
            self.get_logger().warning('Tidak ada sampel jarak valid pada /scan')
            return

        closest_angle, closest_distance = min(
            samples, key=lambda sample: sample[1])
        valid_angles = [angle for angle, _distance in samples]
        usable_samples = [
            sample for sample in samples if -80.0 <= sample[0] <= 80.0
        ]
        usable_closest = min(
            usable_samples, key=lambda sample: sample[1]
        ) if usable_samples else None
        front = self.sector_min(samples, -30.0, 30.0)
        left = self.sector_min(samples, 30.0, 80.0)
        right = self.sector_min(samples, -80.0, -30.0)

        def distance_text(value: float) -> str:
            return f'{value:.2f}m' if math.isfinite(value) else 'tidak ada data'

        usable_text = (
            f'{distance_text(usable_closest[1])} @ '
            f'{usable_closest[0]:.1f}deg'
            if usable_closest else 'tidak ada data')
        self.get_logger().info(
            f'VALID={min(valid_angles):.1f}..{max(valid_angles):.1f}deg | '
            f'TERDEKAT={closest_distance:.2f}m @ {closest_angle:.1f}deg | '
            f'AREA(-80..80)={usable_text}')
        self.get_logger().info(
            f'SEKTOR KERJA: KANAN(-80..-30)={distance_text(right)} | '
            f'DEPAN(-30..30)={distance_text(front)} | '
            f'KIRI(30..80)={distance_text(left)}')


def main() -> None:
    rclpy.init()
    node = LidarAngleMonitor()
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
