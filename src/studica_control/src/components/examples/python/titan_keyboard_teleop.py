#!/usr/bin/env python3
"""Keyboard AMR dan OMS: /cmd_vel base serta motor Titan kedua."""

import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path
import yaml
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from human_interaction import HumanInteraction
from studica_control.srv import SetData
from yellow_follow_keyboard import YellowFollower


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


@dataclass
class StandardServoJog:
    """Ramp target posisi, bukan feedback posisi fisik servo."""

    minimum: float = -150.0
    maximum: float = 150.0
    initial: float = 0.0
    target: Optional[float] = None
    last_sent: Optional[float] = None

    def observe(self, value: float) -> None:
        # Driver publishes min-1 (-151) until its first successful command.
        if (self.target is None and math.isfinite(value) and
                self.minimum <= value <= self.maximum):
            self.target = float(value)
            self.last_sent = float(round(value))

    def initialize(self) -> Optional[float]:
        if self.target is not None:
            return None
        self.target = self.initial
        self.last_sent = float(round(self.initial))
        return self.last_sent

    def jog(self, rate: float, dt: float) -> Optional[float]:
        if rate == 0.0:
            return None  # Idle/stop must never command the centre position.
        if self.target is None:
            raise RuntimeError('Tekan P untuk posisi awal servo standard dahulu')
        self.target = clamp(self.target + rate * clamp(dt, 0.0, 0.05),
                            self.minimum, self.maximum)
        value = float(round(self.target))
        if value == self.last_sent:
            return None
        self.last_sent = value
        return value


class ServoServiceCommand:
    """Send the latest target through the same service as servo_example.py.

    One request per servo in flight; intermediate jog targets are coalesced.
    Failed requests retain their target and retry without blocking keyboard STOP.
    """
    def __init__(self, client, logger, name):
        self.client = client
        self.logger = logger
        self.name = name
        self.target = None
        self.confirmed = None
        self.future = None
        self.sent = None
        self.next_attempt = 0.0

    def set_target(self, value):
        self.target = float(value)

    def pump(self, now):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                response = self.future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self.confirmed = self.sent
            except Exception as error:
                self.logger.error(f'{self.name}: set_servo gagal: {error}')
                self.next_attempt = now + 0.5
            self.future = None
        if (self.target is None or self.target == self.confirmed or
                now < self.next_attempt):
            return
        self.next_attempt = now + 0.05
        if not self.client.service_is_ready():
            self.logger.warning(f'{self.name}: menunggu service set_servo')
            self.next_attempt = now + 1.0
            return
        request = SetData.Request()
        request.initparams.speed = self.target
        try:
            self.future = self.client.call_async(request)
            self.sent = self.target
        except Exception as error:
            self.logger.error(f'{self.name}: set_servo gagal: {error}')
            self.next_attempt = now + 0.5


class KeyboardCmdVel(Node):
    def __init__(self, sensor: str, oms_sensor: str, args) -> None:
        super().__init__('keyboard_cmd_vel')
        human_config = {}
        if args.human_config:
            with Path(args.human_config).open(encoding='utf-8') as stream:
                human_config = yaml.safe_load(stream) or {}
        self.human = HumanInteraction(args.oms_default_file, time.monotonic,
                                      slide=human_config.get('slide'))
        self.human_status = self.create_publisher(String, '/human_interaction/status', 10)
        self.human_last_status = None
        for name, motor in (('lift', 2), ('rotate', 3)):
            self.create_subscription(
                Float64, f'/{oms_sensor}/m_{motor}/encoder',
                lambda msg, name=name: self.human.observe(name, msg.data), 10)
        for name in ('start', 'stop'):
            self.create_subscription(
                Bool, f'/{name}_button/state',
                lambda msg, name=name: self.human.button(name, not msg.data), 10)
        self.yellow_follower = YellowFollower()
        self.create_subscription(
            String, '/color_tracker/result', self.yellow_follower.result_callback, 1)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.oms_lift_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_2/cmd', 10)
        self.oms_rotate_publisher = self.create_publisher(
            Float64, f'/{oms_sensor}/m_3/cmd', 10)
        self.slide_publisher = self.create_publisher(
            Float64, '/oms_slide/cmd', 10)
        self.servo_commands = {
            name: ServoServiceCommand(
                self.create_client(SetData, f'/oms_{name}/set_servo'),
                self.get_logger(), name)
            for name in ('wrist', 'gripper')
        }
        self.standard_servos = {
            name: StandardServoJog(
                getattr(args, f'{name}_min_angle'),
                getattr(args, f'{name}_max_angle'),
                getattr(args, f'{name}_start_angle'))
            for name in ('wrist', 'gripper')
        }
        self.servo_positions = args.servo_positions
        self.servo_waiting = set()
        self.servo_subscriptions = [
            self.create_subscription(
                Float64, f'/oms_{name}/state',
                lambda msg, name=name: self.standard_servos[name].observe(msg.data),
                10)
            for name in self.standard_servos
        ]
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

    def command_servo_position(self, key):
        name, value = self.servo_positions[key]
        servo = self.standard_servos[name]
        servo.target = value
        servo.last_sent = value
        self.servo_commands[name].set_target(value)
        self.get_logger().info(f'{key.upper()}: {name} target={value:g} derajat')

    def initialize_standard_servos(self) -> None:
        for name, servo in self.standard_servos.items():
            value = servo.initialize()
            if value is not None:
                self.servo_commands[name].set_target(value)
                self.get_logger().info(f'{name}: posisi awal {value:.0f} derajat')

    def publish_servos(
            self, slide: float, wrist: float = 0.0, gripper: float = 0.0,
            dt: float = 0.0) -> None:
        self.slide_publisher.publish(Float64(data=float(slide)))
        for name, rate in (('wrist', wrist), ('gripper', gripper)):
            servo = self.standard_servos[name]
            if rate != 0.0 and servo.target is None:
                value = servo.initialize()
                self.get_logger().info(
                    f'{name}: inisialisasi target {value:.0f} derajat')
            else:
                value = servo.jog(rate, dt)
            if value is not None:
                self.servo_commands[name].set_target(value)
            self.servo_commands[name].pump(time.monotonic())

    def stop(self) -> None:
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            self.publish_oms(0.0, 0.0)
            self.publish_servos(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard /cmd_vel AMR')
    parser.add_argument('--servo-config', default=None,
                        help='YAML left_value/right_value wrist dan eof (derajat)')
    parser.add_argument('--human-config', default=None,
                        help='YAML konfigurasi mode human interaction')
    parser.add_argument('--oms-default-file', default='config/oms_default.json',
                        help='file posisi default OMS yang disimpan saat M')
    parser.add_argument('--sensor', default='titan0')
    parser.add_argument('--oms-sensor', default='titan1')
    parser.add_argument('--oms-speed', type=float, default=0.20)
    parser.add_argument('--lift-speed', type=float, default=None)
    parser.add_argument('--rotate-speed', '--rotate-duty', type=float, default=33.3,
                        help='duty putar OMS J/L langsung, 0..100 persen (tanpa PID)')
    parser.add_argument('--oms-pid', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--lift-rpm', type=float, default=None,
                        help='override target naik dan turun sekaligus')
    parser.add_argument('--lift-up-rpm', type=float, default=30.0)
    parser.add_argument('--lift-down-rpm', type=float, default=30.0)
    parser.add_argument('--rotate-rpm', type=float, default=15.0)
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
    parser.add_argument('--wrist-speed', type=float, default=25.0,
                        help='laju perubahan target posisi, derajat/detik')
    parser.add_argument('--gripper-speed', type=float, default=15.0,
                        help='laju perubahan target posisi, derajat/detik')
    for name in ('wrist', 'gripper'):
        parser.add_argument(f'--{name}-min-angle', type=int, default=-150)
        parser.add_argument(f'--{name}-max-angle', type=int, default=150)
        parser.add_argument(f'--{name}-start-angle', type=int, default=0,
                            help='posisi awal saat P ditekan; kalibrasi mekanisme')
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
    if not 0.05 <= args.lift_speed <= 0.60:
        parser.error('--lift-speed harus 0.05..0.60 duty')
    if not 0.0 <= args.rotate_speed <= 100.0:
        parser.error('--rotate-speed / --rotate-duty harus 0..100 persen')
    args.rotate_speed /= 100.0
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
    for name in ('wrist', 'gripper'):
        lower = getattr(args, f'{name}_min_angle')
        upper = getattr(args, f'{name}_max_angle')
        initial = getattr(args, f'{name}_start_angle')
        if not -150 <= lower < upper <= 150:
            parser.error(f'--{name}-min/max-angle harus -150..150 dan min < max')
        if not lower <= initial <= upper:
            parser.error(f'--{name}-start-angle harus di antara min/max-angle')
    if not 0.05 <= args.distance <= 5.0:
        parser.error('--distance harus 0.05..5.0 meter')
    # Standalone fallback matches the shipped YAML; launcher supplies its path.
    positions = {'wrist': {'left_value': 30.0, 'right_value': -30.0},
                 'eof': {'left_value': 30.0, 'right_value': -30.0}}
    if args.servo_config:
        try:
            with Path(args.servo_config).open(encoding='utf-8') as stream:
                positions = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            parser.error(f'--servo-config: {error}')
    args.servo_positions = {}
    for key, section, field, name in (
            ('r', 'wrist', 'left_value', 'wrist'),
            ('t', 'wrist', 'right_value', 'wrist'),
            ('y', 'eof', 'left_value', 'gripper'),
            ('u', 'eof', 'right_value', 'gripper')):
        try:
            value = float(positions[section][field])
        except (KeyError, TypeError, ValueError):
            parser.error(f'--servo-config perlu angka {section}.{field}')
        if not (getattr(args, f'{name}_min_angle') <= value <=
                getattr(args, f'{name}_max_angle')):
            parser.error(f'{section}.{field} di luar batas sudut {name}')
        args.servo_positions[key] = (name, value)
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
        'i': (-args.lift_speed * args.lift_polarity, 0.0),       # naik
        'k': (args.lift_speed * args.lift_polarity, 0.0),      # turun
        'j': (0.0, -args.rotate_speed * args.rotate_polarity),   # CCW
        'l': (0.0, args.rotate_speed * args.rotate_polarity),  # CW
    }
    servo_commands = {
        'g': (args.slide_speed * args.slide_polarity, 0.0, 0.0),
        'h': (-args.slide_speed * args.slide_polarity, 0.0, 0.0),
    }
    active_key = None
    active_key_started = 0.0
    last_key_time = 0.0
    last_label = None
    auto_follow = False
    distance_active = False
    distance_start = None
    distance_started = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info(
            'BASE: W maju | S mundur | A kiri | D kanan | Z follow kuning | E/X stop | M simpan OMS + READY')
        node.get_logger().info(
            'OMS: I naik | K turun | J CCW | L CW | E stop semua | Q keluar')
        node.get_logger().info(
            'SERVO: G/H slide | R/T wrist left/right YAML | Y/U EoF left/right YAML')
        node.get_logger().warning(
            f'P = inisialisasi servo tanpa target: wrist={args.wrist_start_angle}, '
            f'gripper={args.gripper_start_angle} derajat. '
            'R/T dan Y/U langsung menuju target sudut dari YAML. '
            'Dapat langsung bergerak ke posisi awal. '
            'E/tombol dilepas mempertahankan target, bukan melepas daya servo.')
        node.get_logger().info(
            f'/cmd_vel linear={args.linear_speed:.2f}m/s, '
            f'angular={args.angular_speed:.2f}rad/s')
        node.get_logger().info(
            f'OMS lift duty={args.lift_speed:.2f}, '
            f'rotate duty={args.rotate_speed * 100:.1f}% (langsung, tanpa PID)')
        if args.oms_pid:
            node.get_logger().info(
                f'OMS PID software: lift naik={args.lift_up_rpm:.0f}, '
                f'turun={args.lift_down_rpm:.0f}/'
                f'{args.lift_max_rpm:.0f} RPM')
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            node.publish_cmd(0.0, 0.0)
            node.publish_oms(0.0, 0.0)
            node.publish_servos(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

        previous_time = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            dt = now - previous_time
            previous_time = now
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1).lower()
                if node.human.stop_pressed and key != 'q':
                    key = 'e'
                if node.human.owns_control and key not in ('m', 'e', 'x', 'q'):
                    key = ''
                if (key in key_commands or key in oms_commands or
                        key in servo_commands or key in args.servo_positions or
                        key in ('e', 'x', 'p')):
                    auto_follow = False
                if key == 'm':
                    auto_follow = False
                    active_key = None
                    distance_active = False
                    node.lift_pid.reset()
                    node.stop()
                    node.human.capture()
                elif key == 'z':
                    auto_follow = not auto_follow
                    active_key = None
                    distance_active = False
                    node.stop()
                    node.get_logger().info(f'Follow kuning: {auto_follow}')
                elif key in key_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
                elif key in oms_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
                elif key in args.servo_positions and not distance_active:
                    active_key = None
                    node.command_servo_position(key)
                elif key in servo_commands and not distance_active:
                    if key != active_key:
                        active_key_started = now
                    active_key = key
                    last_key_time = now
                elif key in ('e', 'x'):
                    node.human.cancel()
                    active_key = None
                    distance_active = False
                    node.stop()
                    last_label = None
                elif key == 'q':
                    break
                elif key == 'p':
                    active_key = None
                    distance_active = False
                    node.stop()
                    node.initialize_standard_servos()

            node.human.tick({name: servo.target for name, servo in
                             node.standard_servos.items()})
            status = f'{node.human.state}: {node.human.message}'
            if status != node.human_last_status:
                node.get_logger().info(status)
                node.human_status.publish(String(data=status))
                node.human_last_status = status
            if node.human.owns_control or node.human.stop_pressed:
                auto_follow = False
                active_key = None
                distance_active = False
                node.lift_pid.reset()

            if auto_follow:
                vx, wz = node.yellow_follower.command()
                command = (clamp(vx, -args.linear_speed, args.linear_speed),
                           clamp(wz, -args.angular_speed, args.angular_speed))
            elif distance_active:
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
                        requested_lift = node.lift_pid.calculate(
                            lift_target, requested_lift, now)
                    oms_command = (requested_lift, requested_rotate)
                except RuntimeError as error:
                    active_key = None
                    if last_label != 'OMS FEEDBACK ERROR':
                        node.get_logger().error(str(error))
                    last_label = 'OMS FEEDBACK ERROR'
            node.publish_oms(*oms_command)
            servo_command = (
                servo_commands[active_key]
                if active_key in servo_commands and not distance_active
                else (0.0, 0.0, 0.0))
            node.publish_servos(*servo_command, dt=dt)
            label = (node.yellow_follower.status if auto_follow else
                     ('G' if distance_active else (active_key.upper() if active_key else 'STOP')))
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
