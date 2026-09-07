#!/usr/bin/env python3
"""Servo standard — read current angle and command a new angle.

Run:  python3 servo_example.py --sensor test_servo --angle 90
Requires: studica_launch.py running, a standard servo enabled in the params file

Topic:   subscribes to '/<sensor>/state' (std_msgs/Float64)
Service: '/<sensor>/set_servo' (SetData)
    Command: pass the target angle in 'initparams.speed' (e.g. 90)
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from studica_control.srv import SetData


class ServoExample(Node):
    def __init__(self, sensor, angle):
        super().__init__('servo_example')
        self.angle = angle
        self.sub = self.create_subscription(
            Float64, f'/{sensor}/state', self.on_angle, 10)
        self.client = self.create_client(SetData, f'/{sensor}/set_servo')
        self.get_logger().info(f'Listening on /{sensor}/state...')

    def on_angle(self, msg):
        self.get_logger().info(f'Current angle: {msg.data}')

    def set_angle(self, degrees):
        req = SetData.Request()
        req.initparams.speed = float(degrees)
        future = self.client.call_async(req)
        future.add_done_callback(self.on_set_angle)

    def on_set_angle(self, future):
        try:
            self.get_logger().info(f'set_angle: {future.result().message}')
        except Exception as error:
            self.get_logger().error(f'set_angle failed: {error}')


def main():
    parser = argparse.ArgumentParser(description='Test a standard servo')
    parser.add_argument('--sensor', default='servo')
    parser.add_argument('--angle', type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = ServoExample(args.sensor, args.angle)
    node.create_timer(2.0, lambda: node.set_angle(node.angle))
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
