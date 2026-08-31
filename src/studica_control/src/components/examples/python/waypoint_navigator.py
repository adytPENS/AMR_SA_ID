#!/usr/bin/env python3
"""Navigasi waypoint lokal untuk robot skid-steer Studica.

Input:
  /odom                         nav_msgs/Odometry
  /scan                         sensor_msgs/LaserScan
  /waypoint_navigator/start     std_srvs/Trigger
  /waypoint_navigator/stop      std_srvs/Trigger

Output:
  /titan0/m_0..m_3/cmd          std_msgs/Float64

Koordinat waypoint dinyatakan dalam meter terhadap pose odom (0, 0, 0).
Obstacle avoidance bersifat reaktif dan tidak menggantikan global planner.
"""

import argparse
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty, Trigger


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointNavigator(Node):
    def __init__(self, config_path: str, sequence_override: Optional[str]) -> None:
        super().__init__('waypoint_navigator')
        config = self.load_config(config_path)

        raw_waypoints: Dict[str, Tuple[float, float]] = {
            name.upper(): (float(value['x']), float(value['y']))
            for name, value in config['waypoints'].items()
        }
        coordinate_mode = str(config.get('coordinate_mode', 'local')).lower()
        if coordinate_mode == 'map':
            start = config.get('start_pose', {})
            start_x = float(start.get('x', 0.0))
            start_y = float(start.get('y', 0.0))
            start_yaw = math.radians(float(start.get('yaw_deg', 0.0)))
            cosine = math.cos(start_yaw)
            sine = math.sin(start_yaw)
            self.waypoints = {}
            for name, (map_x, map_y) in raw_waypoints.items():
                dx = map_x - start_x
                dy = map_y - start_y
                self.waypoints[name] = (
                    cosine * dx + sine * dy,
                    -sine * dx + cosine * dy,
                )
        elif coordinate_mode == 'local':
            self.waypoints = raw_waypoints
        else:
            raise ValueError('coordinate_mode harus local atau map')
        sequence = (
            [item.strip().upper() for item in sequence_override.split(',')]
            if sequence_override else
            [str(item).upper() for item in config['sequence']]
        )
        missing = [name for name in sequence if name not in self.waypoints]
        if missing:
            raise ValueError(f'Waypoint tidak ditemukan: {missing}')
        self.sequence = sequence

        motion = config.get('motion', {})
        obstacle = config.get('obstacle_avoidance', {})
        self.forward_duty = float(motion.get('forward_duty', 0.10))
        self.minimum_duty = float(motion.get('minimum_duty', 0.07))
        self.turn_duty = float(motion.get('turn_duty', 0.30))
        self.heading_kp = float(motion.get('heading_kp', 0.18))
        self.distance_kp = float(motion.get('distance_kp', 0.20))
        self.position_tolerance = float(
            motion.get('position_tolerance', 0.12))
        self.turn_tolerance = math.radians(
            float(motion.get('turn_tolerance_deg', 6.0)))
        self.drive_heading_limit = math.radians(
            float(motion.get('drive_heading_limit_deg', 25.0)))

        self.avoidance_enabled = bool(obstacle.get('enabled', True))
        self.stop_distance = float(obstacle.get('stop_distance', 0.55))
        self.clear_distance = float(obstacle.get('clear_distance', 0.75))
        self.avoid_angle = math.radians(
            float(obstacle.get('avoid_angle_deg', 55.0)))
        self.avoid_step_distance = float(
            obstacle.get('avoid_step_distance', 0.60))
        self.avoid_timeout = float(obstacle.get('timeout', 12.0))
        button = config.get('start_button', {})
        self.button_enabled = bool(button.get('enabled', False))
        self.button_topic = str(
            button.get('topic', '/start_button/state'))
        self.button_active_high = bool(button.get('active_high', True))
        self.button_debounce = float(button.get('debounce_ms', 250)) / 1000.0

        sensor = str(config.get('sensor', 'titan0'))
        self.motor_publishers = [
            self.create_publisher(Float64, f'/{sensor}/m_{i}/cmd', 1)
            for i in range(4)
        ]
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.button_subscription = None
        if self.button_enabled:
            self.button_subscription = self.create_subscription(
                Bool, self.button_topic, self.button_callback, 10)
        self.create_service(
            Trigger, '/waypoint_navigator/start', self.start_callback)
        self.create_service(
            Trigger, '/waypoint_navigator/stop', self.stop_callback)
        self.timer = self.create_timer(0.04, self.control_loop)
        self.odom_reset_client = self.create_client(
            Empty, '/wheel_odometry/reset')

        self.pose: Optional[Tuple[float, float, float]] = None
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.front_clearance = math.inf
        self.left_clearance = math.inf
        self.right_clearance = math.inf
        self.active = False
        self.state = 'IDLE'
        self.index = 0
        self.state_started = time.monotonic()
        self.avoid_direction = 1.0
        self.avoid_target_yaw = 0.0
        self.avoid_start: Optional[Tuple[float, float]] = None
        self.next_log_time = 0.0
        self.last_button_active: Optional[bool] = None
        self.last_button_press = 0.0
        self.reset_future = None
        self.start_pending = False

        points = ', '.join(
            f'{name}=({self.waypoints[name][0]:.2f}, '
            f'{self.waypoints[name][1]:.2f})'
            for name in self.sequence)
        self.get_logger().info(f'Waypoint dimuat: {points}')
        self.get_logger().info(
            'Siap tetapi belum bergerak. Reset /wheel_odometry/reset, lalu '
            'panggil /waypoint_navigator/start.')
        if self.button_enabled:
            self.get_logger().info(
                f'Start button aktif: {self.button_topic}, '
                f'active_high={self.button_active_high}')

    @staticmethod
    def load_config(config_path: str) -> dict:
        path = Path(config_path)
        with path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError('Konfigurasi waypoint harus berupa YAML mapping')
        if 'waypoints' not in config or 'sequence' not in config:
            raise ValueError('YAML wajib memiliki waypoints dan sequence')
        return config

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
        )
        self.last_odom_time = time.monotonic()

    @staticmethod
    def sector_min(msg: LaserScan, start: float, end: float) -> float:
        values: List[float] = []
        for index, distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if start <= angle <= end and math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    values.append(float(distance))
        return min(values) if values else math.inf

    def scan_callback(self, msg: LaserScan) -> None:
        self.front_clearance = self.sector_min(
            msg, math.radians(-30), math.radians(30))
        self.left_clearance = self.sector_min(
            msg, math.radians(30), math.radians(100))
        self.right_clearance = self.sector_min(
            msg, math.radians(-100), math.radians(-30))
        self.last_scan_time = time.monotonic()

    def button_callback(self, msg: Bool) -> None:
        active = bool(msg.data) == self.button_active_high
        if self.last_button_active is None:
            self.last_button_active = active
            return
        rising_active = active and not self.last_button_active
        self.last_button_active = active
        now = time.monotonic()
        if rising_active and now - self.last_button_press >= self.button_debounce:
            self.last_button_press = now
            accepted, message = self.request_start()
            if accepted:
                self.get_logger().info('START BUTTON: ' + message)
            else:
                self.get_logger().error('START BUTTON DITOLAK: ' + message)

    def request_start(self) -> Tuple[bool, str]:
        if self.active or self.start_pending:
            return False, 'navigator sudah aktif atau sedang start'
        if self.pose is None:
            return False, '/odom belum tersedia'
        if self.avoidance_enabled and time.monotonic() - self.last_scan_time > 1.0:
            return False, '/scan belum tersedia atau stale'
        if not self.odom_reset_client.service_is_ready():
            return False, '/wheel_odometry/reset belum tersedia'
        self.stop_motors()
        self.reset_future = self.odom_reset_client.call_async(Empty.Request())
        self.start_pending = True
        self.state = 'RESETTING_ODOM'
        return True, f'reset odometri, lalu mulai {self.sequence}'

    def finish_pending_start(self) -> None:
        if not self.start_pending or self.reset_future is None:
            return
        if not self.reset_future.done():
            self.stop_motors()
            return
        try:
            self.reset_future.result()
        except Exception as error:
            self.get_logger().error(f'Reset odometri gagal: {error}; STOP')
            self.start_pending = False
            self.reset_future = None
            self.state = 'IDLE'
            self.stop_motors()
            return
        self.index = 0
        self.active = True
        self.start_pending = False
        self.reset_future = None
        self.set_state('TURN_TO_GOAL')
        self.get_logger().info(f'RUN: urutan {self.sequence}')

    def start_callback(self, _request, response):
        response.success, response.message = self.request_start()
        self.get_logger().info(response.message)
        return response

    def stop_callback(self, _request, response):
        self.active = False
        self.start_pending = False
        self.reset_future = None
        self.state = 'IDLE'
        self.stop_motors()
        response.success = True
        response.message = 'STOP waypoint navigator'
        self.get_logger().warning(response.message)
        return response

    def set_state(self, state: str) -> None:
        self.state = state
        self.state_started = time.monotonic()

    def publish_drive(self, forward: float, turn_left: float) -> None:
        # M0/M1 kanan memakai duty negatif untuk maju. M2/M3 kiri positif.
        right = clamp(-(forward + turn_left), -0.30, 0.30)
        left = clamp(forward - turn_left, -0.30, 0.30)
        commands = (right, right, left, left)
        for publisher, value in zip(self.motor_publishers, commands):
            msg = Float64()
            msg.data = float(value)
            publisher.publish(msg)

    def stop_motors(self) -> None:
        for _ in range(3):
            self.publish_drive(0.0, 0.0)

    def begin_avoidance(self) -> None:
        self.avoid_direction = (
            1.0 if self.left_clearance >= self.right_clearance else -1.0)
        assert self.pose is not None
        self.avoid_target_yaw = normalize_angle(
            self.pose[2] + self.avoid_direction * self.avoid_angle)
        self.set_state('AVOID_TURN')
        side = 'kiri' if self.avoid_direction > 0.0 else 'kanan'
        self.get_logger().warning(
            f'Obstacle {self.front_clearance:.2f} m; menghindar ke {side}')

    def control_loop(self) -> None:
        now = time.monotonic()
        if self.start_pending:
            self.finish_pending_start()
            return
        if not self.active:
            self.publish_drive(0.0, 0.0)
            return
        if self.pose is None or now - self.last_odom_time > 0.6:
            self.get_logger().error('Odometri stale; STOP')
            self.active = False
            self.stop_motors()
            return
        if self.avoidance_enabled and now - self.last_scan_time > 1.0:
            self.get_logger().error('LiDAR stale; STOP')
            self.active = False
            self.stop_motors()
            return
        if self.index >= len(self.sequence):
            self.active = False
            self.state = 'DONE'
            self.stop_motors()
            self.get_logger().info('SEMUA WAYPOINT SELESAI; STOP')
            return

        x, y, yaw = self.pose
        name = self.sequence[self.index]
        goal_x, goal_y = self.waypoints[name]
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)
        goal_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(goal_heading - yaw)

        if now >= self.next_log_time:
            self.get_logger().info(
                f'{self.state} -> {name}: pose=({x:.2f},{y:.2f},'
                f'{math.degrees(yaw):.1f}deg), jarak={distance:.2f}m, '
                f'front={self.front_clearance:.2f}m')
            self.next_log_time = now + 1.0

        if distance <= self.position_tolerance:
            self.stop_motors()
            self.get_logger().info(
                f'WAYPOINT {name} TERCAPAI pada ({x:.2f}, {y:.2f})')
            self.index += 1
            self.set_state('TURN_TO_GOAL')
            return

        if self.state in ('TURN_TO_GOAL', 'DRIVE_TO_GOAL'):
            if (self.avoidance_enabled and
                    self.front_clearance < self.stop_distance):
                self.stop_motors()
                self.begin_avoidance()
                return

            if abs(heading_error) > self.drive_heading_limit:
                self.set_state('TURN_TO_GOAL')
            elif self.state == 'TURN_TO_GOAL' and abs(heading_error) <= self.turn_tolerance:
                self.set_state('DRIVE_TO_GOAL')

            if self.state == 'TURN_TO_GOAL':
                turn = math.copysign(self.turn_duty, heading_error)
                self.publish_drive(0.0, turn)
            else:
                forward = clamp(
                    self.distance_kp * distance,
                    self.minimum_duty,
                    self.forward_duty)
                turn = clamp(
                    self.heading_kp * heading_error,
                    -0.08,
                    0.08)
                self.publish_drive(forward, turn)
            return

        if self.state == 'AVOID_TURN':
            if now - self.state_started > self.avoid_timeout:
                self.get_logger().error('Avoidance turn timeout; STOP')
                self.active = False
                self.stop_motors()
                return
            avoid_error = normalize_angle(self.avoid_target_yaw - yaw)
            if abs(avoid_error) <= self.turn_tolerance:
                self.avoid_start = (x, y)
                self.set_state('AVOID_FORWARD')
                self.stop_motors()
            else:
                self.publish_drive(
                    0.0, math.copysign(self.turn_duty, avoid_error))
            return

        if self.state == 'AVOID_FORWARD':
            assert self.avoid_start is not None
            traveled = math.hypot(
                x - self.avoid_start[0], y - self.avoid_start[1])
            if now - self.state_started > self.avoid_timeout:
                self.get_logger().error('Avoidance forward timeout; STOP')
                self.active = False
                self.stop_motors()
            elif self.front_clearance < self.stop_distance:
                self.stop_motors()
                self.begin_avoidance()
            elif traveled >= self.avoid_step_distance:
                self.stop_motors()
                self.set_state('TURN_TO_GOAL')
            else:
                avoid_error = normalize_angle(self.avoid_target_yaw - yaw)
                turn = clamp(self.heading_kp * avoid_error, -0.08, 0.08)
                self.publish_drive(self.minimum_duty, turn)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Navigasi waypoint odometri dengan obstacle avoidance')
    parser.add_argument('--config', required=True, help='file YAML waypoint')
    parser.add_argument(
        '--sequence', default=None,
        help='override urutan, contoh A,B,D,C')
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = WaypointNavigator(args.config, args.sequence)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning('Ctrl+C; STOP')
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
