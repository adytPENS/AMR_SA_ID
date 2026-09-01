#!/usr/bin/env python3
"""Universal /cmd_vel -> inverse kinematics -> PID M0-M3 -> Titan duty."""

import argparse
import time
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

from drive_kinematics import DriveKinematics
from motor_speed_pid import MotorSpeedPID


class DriveController(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('drive_controller')
        with Path(config_path).open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}

        drive_config = config.get('drive', {})
        motor_config = config.get('motors', {})
        pid_config = config.get('speed_pid', {})
        self.kinematics = DriveKinematics.from_mapping(drive_config)
        self.max_wheel_speed = float(
            drive_config.get('max_wheel_speed', 0.75))
        self.channels = [int(value) for value in
                         motor_config.get('titan_channels', [0, 1, 2, 3])]
        self.polarities = [float(value) for value in
                           motor_config.get(
                               'electrical_polarity', [-1, -1, 1, 1])]
        if sorted(self.channels) != [0, 1, 2, 3]:
            raise ValueError('titan_channels harus permutasi [0,1,2,3]')
        if len(self.polarities) != 4:
            raise ValueError('electrical_polarity harus berisi 4 nilai')

        sensor = str(config.get('sensor', 'titan0'))
        self.pid_enabled = bool(pid_config.get('enabled', True))
        self.cmd_timeout = float(config.get('safety', {}).get(
            'cmd_vel_timeout', 0.35))
        self.controllers = [
            MotorSpeedPID(
                polarity=self.polarities[index],
                speed_at_full_duty=self.max_wheel_speed,
                kp=float(pid_config.get('kp', 0.25)),
                ki=float(pid_config.get('ki', 0.10)),
                kd=float(pid_config.get('kd', 0.0)),
                duty_limit=float(pid_config.get('duty_limit', 0.70)),
                integral_limit=float(pid_config.get('integral_limit', 0.50)),
                feedback_timeout=float(
                    pid_config.get('feedback_timeout', 0.30)),
            ) for index in range(4)
        ]
        self.motor_publishers = [
            self.create_publisher(
                Float64, f'/{sensor}/m_{channel}/cmd', 1)
            for channel in self.channels
        ]
        self.encoder_subscriptions = [
            self.create_subscription(
                Float64, f'/{sensor}/m_{channel}/encoder',
                lambda msg, index=index: self.encoder_callback(index, msg), 10)
            for index, channel in enumerate(self.channels)
        ]
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.command: Optional[Twist] = None
        self.command_time = 0.0
        self.blocked_logged = False
        self.get_logger().info(
            f'Drive model={self.kinematics.model.value}, '
            f'wheel={self.kinematics.geometry.wheel_diameter:.3f}m, '
            f'track={self.kinematics.geometry.track_width:.3f}m, '
            f'wheelbase={self.kinematics.geometry.wheelbase:.3f}m, '
            f'PID={self.pid_enabled}')
        self.get_logger().info('Menunggu /cmd_vel; motor STOP')

    def encoder_callback(self, index: int, msg: Float64) -> None:
        self.controllers[index].update_encoder(
            float(msg.data), time.monotonic())

    def cmd_callback(self, msg: Twist) -> None:
        self.command = msg
        self.command_time = time.monotonic()

    def publish(self, duties: List[float]) -> None:
        for publisher, duty in zip(self.motor_publishers, duties):
            msg = Float64()
            msg.data = float(duty)
            publisher.publish(msg)

    def stop(self) -> None:
        for controller in self.controllers:
            controller.reset()
        for _ in range(3):
            self.publish([0.0] * 4)

    def control_loop(self) -> None:
        now = time.monotonic()
        if self.command is None or now - self.command_time > self.cmd_timeout:
            self.stop()
            return

        vx = float(self.command.linear.x)
        vy = float(self.command.linear.y)
        wz = float(self.command.angular.z)
        try:
            targets = self.kinematics.limit(
                self.kinematics.wheel_speeds(vx, vy, wz),
                self.max_wheel_speed)
        except ValueError as error:
            self.get_logger().error(str(error))
            self.stop()
            return

        if max(abs(value) for value in targets) < 1e-6:
            self.stop()
            return

        duties: List[float] = []
        try:
            for index, target_speed in enumerate(targets):
                physical_ff = target_speed / self.max_wheel_speed
                electrical_ff = physical_ff / self.polarities[index]
                duty = (
                    self.controllers[index].calculate(electrical_ff, now)
                    if self.pid_enabled else electrical_ff)
                duties.append(duty)
        except RuntimeError as error:
            self.stop()
            if not self.blocked_logged:
                self.get_logger().error(f'Gerak ditolak: {error}')
                self.blocked_logged = True
            return
        self.blocked_logged = False
        self.publish(duties)


def parse_args():
    parser = argparse.ArgumentParser(description='Universal AMR drive controller')
    parser.add_argument('--config', required=True, help='drive_controller.yaml')
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = DriveController(args.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
