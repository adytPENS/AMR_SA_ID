#!/usr/bin/env python3
"""Teruskan hanya sektor LiDAR depan yang aman ke /scan_front."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FrontScanFilter(Node):
    def __init__(self) -> None:
        super().__init__('front_scan_filter')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_front')
        self.declare_parameter('min_angle_deg', -80.0)
        self.declare_parameter('max_angle_deg', 80.0)
        self.min_angle = math.radians(
            float(self.get_parameter('min_angle_deg').value))
        self.max_angle = math.radians(
            float(self.get_parameter('max_angle_deg').value))
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}: '
            f'{math.degrees(self.min_angle):.1f}..'
            f'{math.degrees(self.max_angle):.1f} deg')

    def callback(self, msg: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        ranges = list(msg.ranges)
        intensities = list(msg.intensities)
        for index in range(len(ranges)):
            angle = math.atan2(
                math.sin(msg.angle_min + index * msg.angle_increment),
                math.cos(msg.angle_min + index * msg.angle_increment))
            if not self.min_angle <= angle <= self.max_angle:
                ranges[index] = math.inf
                if index < len(intensities):
                    intensities[index] = 0.0
        filtered.ranges = ranges
        filtered.intensities = intensities
        self.publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = FrontScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
