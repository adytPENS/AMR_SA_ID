#!/usr/bin/env python3
"""Keyboard AMR dan OMS: /cmd_vel base serta motor Titan kedua."""

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class OmsRpmPid:
    """PI kecepatan OMS; PID tidak diizinkan membalik arah motor."""

    max_rpm: float
    kp: float
    ki: float
    duty_limit: float
    minimum_duty: float
    filter_alpha: float = 0.35
    feedback_timeout: float = 0.40
    rpm: Optional[float] = None
    feedback_time: float = 0.0
    integral: float = 0.0
    control_time: float = 0.0

    def update(self, rpm: float, now: float) -> None:
        magnitude = abs(float(rpm))
        self.rpm = (magnitude if self.rpm is None else
                    self.filter_alpha * magnitude +
                    (1.0 - self.filter_alpha) * self.rpm)
        self.feedback_time = now

    def reset(self) -> None:
        self.integral = 0.0
        self.control_time = 0.0

    def calculate(self, target_rpm: float, direction: float, now: float) -> float:
        if target_rpm <= 0.0 or direction == 0.0:
            self.reset()
            return 0.0
        if self.rpm is None or now - self.feedback_time > self.feedback_timeout:
            self.reset()
            raise RuntimeError('feedback RPM OMS stale/belum tersedia')
        error = target_rpm - self.rpm
        dt = now - self.control_time if self.control_time > 0.0 else 0.0
        candidate_integral = self.integral
        if 0.0 < dt < 0.25:
            candidate_integral = clamp(self.integral + error * dt,
                                       -100.0, 100.0)
        feedforward = target_rpm / self.max_rpm
        magnitude = clamp(feedforward + self.kp * error +
                          self.ki * candidate_integral,
                          self.minimum_duty, self.duty_limit)
        if magnitude < self.duty_limit - 1e-6:
            self.integral = candidate_integral
        self.control_time = now
        return (1.0 if direction > 0.0 else -1.0) * magnitude


class KeyboardCmdVel(Node):
    def __init__(self, sensor: str, oms_sensor: str, args) -> None:
        super().__init__('keyboard_cmd_vel')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.oms_lift_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_2/cmd', 10)
        self.oms_rotate_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_3/cmd', 10)
        self.slide_publisher = self.create_publisher(
            Float64, '/oms_slide/cmd', 10)
        self.wrist_publisher = self.create_publisher(
            Float64, '/oms_wrist/cmd', 10)
        self.gripper_publisher = self.create_publisher(
            Float64, '/oms_gripper/cmd', 10)
        self.encoder_values = [None] * 4
        self.encoder_subscriptions = [
            self.create_subscription(
                Float64, f'/{sensor}/m_{motor}/encoder',
                lambda msg, index=motor: self.encoder_callback(index, msg), 10)
            for motor in range(4)
        ]
        self.lift_pid = OmsRpmPid(
            args.lift_max_rpm, args.oms_pid_kp, args.oms_pid_ki,
            args.lift_duty_limit, args.lift_minimum_duty)
        self.rotate_pid = OmsRpmPid(
            args.rotate_max_rpm, args.oms_pid_kp, args.oms_pid_ki,
            args.rotate_duty_limit, args.rotate_minimum_duty)
        self.create_subscription(
            Float64, f'/{oms_sensor}/m_2/rpm', self.lift_rpm_callback, 10)
        self.create_subscription(
            Float64, f'/{oms_sensor}/m_3/rpm', self.rotate_rpm_callback, 10)

    def encoder_callback(self, motor: int, msg: Float64) -> None:
        self.encoder_values[motor] = float(msg.data)

    def lift_rpm_callback(self, msg: Float64) -> None:
        self.lift_pid.update(msg.data, time.monotonic())

    def rotate_rpm_callback(self, msg: Float64) -> None:
        self.rotate_pid.update(msg.data, time.monotonic())

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

    def publish_continuous_servos(
            self, slide: float, wrist: float, gripper: float = 0.0) -> None:
        self.slide_publisher.publish(Float64(data=float(slide)))
        self.wrist_publisher.publish(Float64(data=float(wrist)))
        self.gripper_publisher.publish(Float64(data=float(gripper)))

    def stop(self) -> None:
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            self.publish_oms(0.0, 0.0)
            self.publish_continuous_servos(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard /cmd_vel AMR')
    parser.add_argument('--sensor', default='titan0')
    parser.add_argument('--oms-sensor', default='titan1')
    parser.add_argument('--oms-speed', type=float, default=0.20)
    parser.add_argument('--lift-speed', type=float, default=None)
    parser.add_argument('--rotate-speed', type=float, default=None)
    parser.add_argument('--oms-pid', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--lift-rpm', type=float, default=None,
                        help='override target naik dan turun sekaligus')
    parser.add_argument('--lift-up-rpm', type=float, default=40.0)
    parser.add_argument('--lift-down-rpm', type=float, default=25.0)
    parser.add_argument('--rotate-rpm', type=float, default=35.0)
    parser.add_argument('--lift-max-rpm', type=float, default=100.0)
    parser.add_argument('--rotate-max-rpm', type=float, default=227.0)
    parser.add_argument('--oms-pid-kp', type=float, default=0.004)
    parser.add_argument('--oms-pid-ki', type=float, default=0.002)
    parser.add_argument('--lift-duty-limit', type=float, default=0.55)
    parser.add_argument('--rotate-duty-limit', type=float, default=0.65)
    parser.add_argument('--lift-minimum-duty', type=float, default=0.15)
    parser.add_argument('--rotate-minimum-duty', type=float, default=0.18)
    parser.add_argument('--lift-up-boost-duty', type=float, default=0.65)
    parser.add_argument('--lift-up-boost-time', type=float, default=0.20)
    parser.add_argument('--rotate-boost-duty', type=float, default=0.60)
    parser.add_argument('--rotate-boost-time', type=float, default=0.25)
    parser.add_argument('--slide-speed', type=float, default=40.0)
    parser.add_argument('--wrist-speed', type=float, default=25.0)
    parser.add_argument('--gripper-speed', type=float, default=15.0)
    parser.add_argument('--slide-polarity', type=float, choices=(-1.0, 1.0),
                        default=1.0)
    parser.add_argument('--wrist-polarity', type=float, choices=(-1.0, 1.0),
                        default=1.0)
    parser.add_argument('--gripper-polarity', type=float, choices=(-1.0, 1.0),
                        default=1.0)
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
    if args.lift_rpm is not None:
        if not 5.0 <= args.lift_rpm <= args.lift_max_rpm:
            parser.error('--lift-rpm harus 5 sampai --lift-max-rpm')
        args.lift_up_rpm = args.lift_rpm
        args.lift_down_rpm = args.lift_rpm
    if not 5.0 <= args.lift_up_rpm <= args.lift_max_rpm:
        parser.error('--lift-up-rpm harus 5 sampai --lift-max-rpm')
    if not 5.0 <= args.lift_down_rpm <= args.lift_max_rpm:
        parser.error('--lift-down-rpm harus 5 sampai --lift-max-rpm')
    if not 5.0 <= args.rotate_rpm <= args.rotate_max_rpm:
        parser.error('--rotate-rpm harus 5 sampai --rotate-max-rpm')
    if not 0.10 <= args.lift_duty_limit <= 0.90:
        parser.error('--lift-duty-limit harus 0.10..0.90')
    if not 0.10 <= args.rotate_duty_limit <= 0.90:
        parser.error('--rotate-duty-limit harus 0.10..0.90')
    for name in ('lift_up_boost_duty', 'rotate_boost_duty'):
        if not 0.10 <= getattr(args, name) <= 0.90:
            parser.error(f'--{name.replace("_", "-")} harus 0.10..0.90')
    for name in ('lift_up_boost_time', 'rotate_boost_time'):
        if not 0.0 <= getattr(args, name) <= 0.50:
            parser.error(f'--{name.replace("_", "-")} harus 0.0..0.50 detik')
    if not 1.0 <= args.slide_speed <= 100.0:
        parser.error('--slide-speed harus 1..100')
    if not 1.0 <= args.wrist_speed <= 100.0:
        parser.error('--wrist-speed harus 1..100')
    if not 1.0 <= args.gripper_speed <= 100.0:
        parser.error('--gripper-speed harus 1..100')
    if not 0.05 <= args.distance <= 5.0:
        parser.error('--distance harus 0.05..5.0 meter')
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = KeyboardCmdVel(args.sensor, args.oms_sensor, args)
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
    continuous_servo_commands = {
        'g': (args.slide_speed * args.slide_polarity, 0.0, 0.0),
        'h': (-args.slide_speed * args.slide_polarity, 0.0, 0.0),
        'r': (0.0, args.wrist_speed * args.wrist_polarity, 0.0),
        't': (0.0, -args.wrist_speed * args.wrist_polarity, 0.0),
        'y': (0.0, 0.0, args.gripper_speed * args.gripper_polarity),
        'u': (0.0, 0.0, -args.gripper_speed * args.gripper_polarity),
    }
    active_key = None
    active_key_started = 0.0
    last_key_time = 0.0
    last_label = None
    distance_active = False
    distance_start = None
    distance_started = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info(
            'BASE: W maju | S mundur | A kiri | D kanan')
        node.get_logger().info(
            'OMS: I naik | K turun | J CCW | L CW | E stop semua | Q keluar')
        node.get_logger().info(
            'SERVO: G/H pin18 | R/T pin19 | Y/U pin20')
        node.get_logger().info(
            f'/cmd_vel linear={args.linear_speed:.2f}m/s, '
            f'angular={args.angular_speed:.2f}rad/s')
        node.get_logger().info(
            f'OMS lift duty={args.lift_speed:.2f}, '
            f'rotate duty={args.rotate_speed:.2f}')
        if args.oms_pid:
            node.get_logger().info(
                f'OMS PID software: lift naik={args.lift_up_rpm:.0f}, '
                f'turun={args.lift_down_rpm:.0f}/'
                f'{args.lift_max_rpm:.0f} RPM, rotate={args.rotate_rpm:.0f}/'
                f'{args.rotate_max_rpm:.0f} RPM')
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            node.publish_cmd(0.0, 0.0)
            node.publish_oms(0.0, 0.0)
            node.publish_continuous_servos(0.0, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

        while rclpy.ok():
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in key_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
                elif key in oms_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
                elif key in continuous_servo_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
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
            oms_command = (0.0, 0.0)
            if active_key in oms_commands and not distance_active:
                requested_lift, requested_rotate = oms_commands[active_key]
                try:
                    if args.oms_pid and requested_lift:
                        lift_target = (args.lift_up_rpm if active_key == 'i'
                                       else args.lift_down_rpm)
                        if (active_key == 'i' and
                                now - active_key_started < args.lift_up_boost_time):
                            requested_lift = (
                                args.lift_up_boost_duty *
                                (1.0 if requested_lift > 0.0 else -1.0))
                            node.lift_pid.reset()
                        else:
                            requested_lift = node.lift_pid.calculate(
                                lift_target, requested_lift, now)
                    if args.oms_pid and requested_rotate:
                        if now - active_key_started < args.rotate_boost_time:
                            requested_rotate = (
                                args.rotate_boost_duty *
                                (1.0 if requested_rotate > 0.0 else -1.0))
                            node.rotate_pid.reset()
                        else:
                            requested_rotate = node.rotate_pid.calculate(
                                args.rotate_rpm, requested_rotate, now)
                    oms_command = (requested_lift, requested_rotate)
                except RuntimeError as error:
                    active_key = None
                    if last_label != 'OMS FEEDBACK ERROR':
                        node.get_logger().error(str(error))
                    last_label = 'OMS FEEDBACK ERROR'
            node.publish_oms(*oms_command)
            continuous_servo_command = (
                continuous_servo_commands[active_key]
                if active_key in continuous_servo_commands and not distance_active
                else (0.0, 0.0, 0.0))
            node.publish_continuous_servos(*continuous_servo_command)
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
