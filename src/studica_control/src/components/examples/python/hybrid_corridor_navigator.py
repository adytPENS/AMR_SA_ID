#!/usr/bin/env python3
"""Waypoint corridor dengan kontrol per ruas: odometry/trace_left/trace_right."""

import argparse
import csv
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class HybridCorridorNavigator(Node):
    VALID_CONTROLS = {'odometry', 'trace_left', 'trace_right'}

    def __init__(self, config_path):
        super().__init__('waypoint_navigator')
        with Path(config_path).open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        debug = config.get('debug', {})
        self.debug_enabled = bool(debug.get('enabled', False))
        self.debug_rate_hz = clamp(
            float(debug.get('rate_hz', 10.0)), 1.0, 20.0)
        self.debug_file = None
        self.debug_writer = None
        self.debug_path = None
        raw_points = config.get('waypoints', {})
        self.sequence = [str(name).upper() for name in config.get('sequence', [])]
        if not self.sequence:
            raise ValueError('sequence tidak boleh kosong')
        missing = [name for name in self.sequence if name not in raw_points]
        if missing:
            raise ValueError(f'Waypoint tidak ditemukan: {missing}')
        self.points = {str(name).upper(): dict(value)
                       for name, value in raw_points.items()}
        for name in self.sequence:
            mode = str(self.points[name].get('control', 'odometry')).lower()
            if mode not in self.VALID_CONTROLS:
                raise ValueError(f'{name}: control tidak valid: {mode}')
            arrival = str(
                self.points[name].get('arrival', 'coordinate')).lower()
            if arrival not in {
                    'coordinate', 'front_wall',
                    'coordinate_or_front_wall'}:
                raise ValueError(f'{name}: arrival tidak valid: {arrival}')

        motion = config.get('motion', {})
        self.linear_speed = float(motion.get('linear_speed', 0.35))
        self.trace_speed = float(motion.get('trace_speed', 0.25))
        self.approach_speed = float(motion.get('approach_speed', 0.10))
        self.angular_speed = float(motion.get('angular_speed', 1.4))
        self.minimum_turn_speed = float(
            motion.get('minimum_turn_speed', 0.35))
        self.heading_kp = float(motion.get('heading_kp', 1.8))
        self.wall_kp = float(motion.get('wall_kp', 1.6))
        self.wall_angle_kp = float(motion.get('wall_angle_kp', 0.8))
        self.max_trace_turn = float(motion.get('max_trace_turn', 0.65))
        self.tolerance = float(motion.get('position_tolerance', 0.15))
        self.approach_radius = float(motion.get('approach_radius', 0.45))
        self.align_tolerance = math.radians(
            float(motion.get('align_tolerance_deg', 7.0)))
        self.front_stop = float(motion.get('front_stop_distance', 0.25))
        self.front_slow_distance = float(
            motion.get('front_slow_distance', 0.40))
        self.wall_visible = float(motion.get('wall_visible_distance', 1.20))
        self.scan_timeout = float(motion.get('scan_timeout', 0.70))
        self.odom_timeout = float(motion.get('odom_timeout', 0.60))
        self.pause_seconds = float(motion.get('waypoint_pause_seconds', 0.30))
        self.pause_waypoints = {
            str(name).upper()
            for name in motion.get('pause_waypoints', ['A', 'B', 'C', 'D'])
        }
        self.auto_corner_turn = bool(motion.get('auto_corner_turn', False))
        self.trace_preserve_heading = bool(
            motion.get('trace_preserve_heading', False))
        self.corner_front_distance = float(
            motion.get('corner_front_distance', 0.28))
        self.corner_right_distance = float(
            motion.get('corner_right_distance', 0.65))
        self.corner_clear_margin = float(
            motion.get('corner_clear_margin', 0.12))
        self.turn_tolerance = math.radians(
            float(motion.get('turn_tolerance_deg', 6.0)))
        self.turn_settle_time = float(motion.get('turn_settle_time', 0.20))
        self.turn_timeout = float(motion.get('turn_timeout', 12.0))
        self.trace_start_grace = float(motion.get('trace_start_grace', 0.0))
        self.trace_start_max_turn = float(
            motion.get('trace_start_max_turn', self.max_trace_turn))
        self.right_near_sector = tuple(
            float(v) for v in motion.get(
                'right_near_sector_deg', [-65.0, -55.0]))
        self.right_far_sector = tuple(
            float(v) for v in motion.get(
                'right_far_sector_deg', [-82.0, -75.0]))
        self.right_gap_u_turn = bool(motion.get('right_gap_u_turn', False))
        self.right_gap_debounce = float(
            motion.get('right_gap_debounce', 0.25))
        self.right_gap_open_ratio = float(
            motion.get('right_gap_open_ratio', 0.65))
        self.cross_wall_ratio = float(
            motion.get('cross_wall_ratio', 0.40))
        self.cross_wall_min_distance = float(
            motion.get('cross_wall_min_distance', 0.15))
        self.cross_wall_max_distance = float(
            motion.get('cross_wall_max_distance', 0.80))
        self.right_gap_front_clear = float(
            motion.get('right_gap_front_clear', 0.50))
        self.u_turn_forward_1 = float(
            motion.get('u_turn_forward_1', 0.40))
        self.u_turn_cross = float(motion.get('u_turn_cross', 0.50))
        self.u_turn_forward_2 = float(
            motion.get('u_turn_forward_2', 0.40))
        self.u_turn_speed = float(motion.get('u_turn_speed', 0.16))
        self.maneuver_front_stop = float(
            motion.get('maneuver_front_stop_distance', 0.22))

        start = config.get('start_button', {})
        stop = config.get('stop_button', {})
        self.start_active_high = bool(start.get('active_high', False))
        self.stop_active_high = bool(stop.get('active_high', False))
        self.debounce = float(start.get('debounce_ms', 250)) / 1000.0
        self.stop_debounce = float(stop.get('debounce_ms', 250)) / 1000.0

        self.pose = None
        self.last_odom = 0.0
        self.last_scan = 0.0
        self.front = math.inf
        self.left = math.inf
        self.right = math.inf
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.imu_yaw = None
        self.last_imu = 0.0
        self.last_command_linear = 0.0
        self.last_command_angular = 0.0
        self.active = False
        self.ready = False
        self.start_pending = False
        self.reset_future = None
        self.reset_requested = 0.0
        self.index = 0
        self.segment_index = -1
        self.segment_aligned = False
        self.segment_heading = None
        self.front_arrival_armed = False
        self.corner_armed = False
        self.corner_target_heading = None
        self.corner_settle_started = None
        self.right_wall_seen = False
        self.right_gap_started = None
        self.maneuver_start_pose = None
        self.maneuver_target_heading = None
        self.maneuver_settle_started = None
        self.gap_probe_samples = 0
        self.gap_probe_open_samples = 0
        self.cross_probe_samples = 0
        self.cross_wall_samples = 0
        self.state = 'BOOTING'
        self.state_started = time.monotonic()
        self.last_start_state = None
        self.last_stop_state = None
        self.last_start_press = 0.0
        self.last_stop_press = 0.0
        self.light_command = 'off'
        self.last_light_command = None

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu', self.imu_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        if bool(start.get('enabled', True)):
            self.create_subscription(Bool, str(start.get(
                'topic', '/start_button/state')), self.start_button_cb, 10)
        if bool(stop.get('enabled', True)):
            self.create_subscription(Bool, str(stop.get(
                'topic', '/stop_button/state')), self.stop_button_cb, 10)
        self.reset_client = self.create_client(Empty, '/wheel_odometry/reset')
        self.create_service(Trigger, '/waypoint_navigator/start', self.start_srv)
        self.create_service(Trigger, '/waypoint_navigator/stop', self.stop_srv)
        self.create_service(Trigger, '/waypoint_navigator/set_ready', self.ready_srv)
        self.lights = [
            self.create_publisher(Bool, '/light_control/cmd', 10),
            self.create_publisher(Bool, '/light_red/cmd', 10),
            self.create_publisher(Bool, '/light_green/cmd', 10),
            self.create_publisher(Bool, '/light_yellow/cmd', 10),
        ]
        self.create_timer(0.04, self.tick)
        self.create_timer(0.5, self.light_tick)
        if self.debug_enabled:
            log_dir = Path(debug.get(
                'directory', '/home/vmx/studica_ws/logs'))
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.debug_path = log_dir / f'right_wall_{stamp}.csv'
            self.debug_file = self.debug_path.open(
                'w', newline='', encoding='utf-8')
            self.debug_writer = csv.writer(self.debug_file)
            self.debug_writer.writerow([
                'wall_time', 'elapsed_s', 'active', 'state', 'waypoint',
                'x_m', 'y_m', 'odom_yaw_deg', 'imu_yaw_deg',
                'target_x_m', 'target_y_m', 'target_distance_m',
                'lidar_front_m', 'lidar_left_m', 'lidar_right_m',
                'right_wall_angle_deg', 'corner_armed', 'gap_open_ratio',
                'cross_wall_ratio',
                'cmd_linear_mps', 'cmd_angular_radps'])
            self.debug_started = time.monotonic()
            self.create_timer(1.0 / self.debug_rate_hz, self.write_debug_row)
            self.get_logger().info(f'DEBUG CSV: {self.debug_path}')
        self.get_logger().info(
            f'Hybrid corridor dimuat: {self.sequence}; BOOTING, lampu OFF')

    @staticmethod
    def sector(msg, low_deg, high_deg):
        values = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            degree = math.degrees(angle)
            if low_deg <= degree <= high_deg:
                values.append((distance * math.cos(angle),
                               distance * math.sin(angle), distance))
        return values

    def wall_measurement(self, msg, side):
        sign = 1.0 if side == 'left' else -1.0
        near = self.sector(msg, 55.0, 65.0) if side == 'left' else \
            self.sector(msg, *self.right_near_sector)
        far = self.sector(msg, 75.0, 82.0) if side == 'left' else \
            self.sector(msg, *self.right_far_sector)
        if len(near) < 3 or len(far) < 3:
            return math.inf, 0.0
        x1 = statistics.median(p[0] for p in near)
        y1 = sign * statistics.median(p[1] for p in near)
        x2 = statistics.median(p[0] for p in far)
        y2 = sign * statistics.median(p[1] for p in far)
        dx = x1 - x2
        dy = y1 - y2
        angle = math.atan2(dy, dx) if abs(dx) > 1e-4 else 0.0
        slope = dy / dx if abs(dx) > 1e-4 else 0.0
        return clamp(y2 - slope * x2, 0.05, 3.0), angle

    def scan_cb(self, msg):
        front_values = [p[2] for p in self.sector(msg, -18.0, 18.0)]
        if front_values:
            front_values.sort()
            count = max(3, len(front_values) // 8)
            self.front = statistics.median(front_values[:count])
        self.left, self.left_angle = self.wall_measurement(msg, 'left')
        self.right, self.right_angle = self.wall_measurement(msg, 'right')
        self.last_scan = time.monotonic()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (float(msg.pose.pose.position.x),
                     float(msg.pose.pose.position.y), yaw)
        self.last_odom = time.monotonic()

    def imu_cb(self, msg):
        q = msg.orientation
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_imu = time.monotonic()

    @staticmethod
    def finite_or_blank(value):
        return round(value, 5) if value is not None and math.isfinite(value) else ''

    def publish_command(self, command):
        self.last_command_linear = float(command.linear.x)
        self.last_command_angular = float(command.angular.z)
        self.cmd.publish(command)

    def write_debug_row(self):
        if self.debug_writer is None:
            return
        x = y = yaw = None
        if self.pose is not None:
            x, y, yaw = self.pose
        name = self.sequence[self.index] if self.index < len(self.sequence) else ''
        target_x = target_y = distance = None
        if name and x is not None:
            point = self.points[name]
            target_x = float(point.get('x', 0.0))
            target_y = float(point.get('y', 0.0))
            distance = math.hypot(target_x - x, target_y - y)
        self.debug_writer.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            round(time.monotonic() - self.debug_started, 3),
            int(self.active), self.state, name,
            self.finite_or_blank(x), self.finite_or_blank(y),
            self.finite_or_blank(math.degrees(yaw) if yaw is not None else None),
            self.finite_or_blank(math.degrees(self.imu_yaw)
                                 if self.imu_yaw is not None else None),
            self.finite_or_blank(target_x), self.finite_or_blank(target_y),
            self.finite_or_blank(distance), self.finite_or_blank(self.front),
            self.finite_or_blank(self.left), self.finite_or_blank(self.right),
            self.finite_or_blank(math.degrees(self.right_angle)),
            int(self.corner_armed),
            round(self.gap_probe_open_samples /
                  max(1, self.gap_probe_samples), 3),
            round(self.cross_wall_samples /
                  max(1, self.cross_probe_samples), 3),
            round(self.last_command_linear, 4),
            round(self.last_command_angular, 4)])
        self.debug_file.flush()

    def close_debug_log(self):
        if self.debug_file is not None and not self.debug_file.closed:
            self.debug_file.flush()
            self.debug_file.close()

    def start_button_cb(self, msg):
        active = bool(msg.data) == self.start_active_high
        pressed = active and self.last_start_state is False
        self.last_start_state = active
        now = time.monotonic()
        if pressed and now - self.last_start_press >= self.debounce:
            self.last_start_press = now
            ok, text = self.request_start()
            (self.get_logger().info if ok else self.get_logger().error)(text)

    def stop_button_cb(self, msg):
        active = bool(msg.data) == self.stop_active_high
        pressed = active and self.last_stop_state is False
        self.last_stop_state = active
        now = time.monotonic()
        if pressed and now - self.last_stop_press >= self.stop_debounce:
            self.last_stop_press = now
            self.deactivate('STOP BUTTON')

    def request_start(self):
        if not self.ready:
            return False, 'START ditolak: robot masih BOOTING'
        if self.active or self.start_pending:
            return False, 'START ditolak: navigator sudah aktif'
        if self.pose is None or time.monotonic() - self.last_odom > self.odom_timeout:
            return False, 'START ditolak: /odom belum siap'
        if time.monotonic() - self.last_scan > self.scan_timeout:
            return False, 'START ditolak: /scan belum siap'
        if not self.reset_client.service_is_ready():
            return False, 'START ditolak: service reset odometry belum siap'
        self.stop_motors()
        self.reset_requested = time.monotonic()
        self.reset_future = self.reset_client.call_async(Empty.Request())
        self.start_pending = True
        self.state = 'RESETTING'
        return True, 'Reset odometry; hybrid corridor akan dimulai'

    def start_srv(self, _request, response):
        response.success, response.message = self.request_start()
        return response

    def stop_srv(self, _request, response):
        self.deactivate('SERVICE STOP')
        response.success = True
        response.message = 'hybrid corridor berhenti'
        return response

    def ready_srv(self, _request, response):
        self.ready = True
        self.state = 'IDLE'
        self.light_command = 'red'
        self.stop_motors()
        response.success = True
        response.message = 'hybrid corridor READY; START diizinkan'
        return response

    def deactivate(self, reason):
        self.active = False
        self.start_pending = False
        self.reset_future = None
        self.state = 'IDLE'
        self.light_command = 'red'
        self.stop_motors()
        self.get_logger().warning(f'{reason}: motor STOP')

    def stop_motors(self):
        for _ in range(3):
            self.publish_command(Twist())

    def finish_reset(self):
        if self.reset_future is None or not self.reset_future.done():
            self.stop_motors()
            return
        try:
            self.reset_future.result()
        except Exception as error:
            self.deactivate(f'Reset odometry gagal: {error}')
            return
        if self.last_odom <= self.reset_requested:
            self.stop_motors()
            return
        x, y, yaw = self.pose
        if (abs(x) > 0.06 or abs(y) > 0.06 or
                abs(yaw) > math.radians(6.0)):
            if time.monotonic() - self.reset_requested > 2.0:
                self.deactivate('Odometry nol tidak terkonfirmasi')
            return
        self.index = 0
        self.segment_index = -1
        self.start_pending = False
        self.reset_future = None
        self.active = True
        self.state = 'MOVE'
        self.state_started = time.monotonic()
        self.light_command = 'green'
        self.get_logger().info('RUN hybrid corridor')

    def reach_waypoint(self, name):
        self.stop_motors()
        self.get_logger().info(f'WAYPOINT {name} tercapai')
        self.index += 1
        if self.index >= len(self.sequence):
            self.active = False
            self.state = 'DONE'
            self.light_command = 'red_blink'
            self.get_logger().info('HYBRID CORRIDOR SELESAI')
        elif name in self.pause_waypoints and self.pause_seconds > 0.0:
            self.state = 'PAUSE'
            self.state_started = time.monotonic()
            self.light_command = 'yellow'
            self.get_logger().info(
                f'TUNGGU {self.pause_seconds:.1f} detik di waypoint {name}')
        else:
            self.state = 'MOVE'
            self.light_command = 'green'
            self.get_logger().info(
                f'TITIK BANTU {name}: lanjut tanpa jeda')

    def begin_segment(self):
        self.segment_index = self.index
        self.segment_aligned = False
        self.segment_heading = None
        self.front_arrival_armed = False
        self.corner_armed = False
        self.corner_target_heading = None
        self.corner_settle_started = None
        self.right_wall_seen = False
        self.right_gap_started = None
        self.maneuver_start_pose = None
        self.maneuver_target_heading = None
        self.maneuver_settle_started = None
        self.gap_probe_samples = 0
        self.gap_probe_open_samples = 0
        self.cross_probe_samples = 0
        self.cross_wall_samples = 0
        self.state_started = time.monotonic()

    def begin_left_corner_turn(self, yaw):
        # Kunci heading ke arah court 0/+90/+180/-90 derajat, lalu tambah 90.
        cardinal = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        self.corner_target_heading = cardinal + math.pi / 2.0
        self.segment_heading = self.corner_target_heading
        self.corner_settle_started = None
        self.state = 'TURN_CORNER_LEFT'
        self.state_started = time.monotonic()
        self.stop_motors()
        self.get_logger().info(
            'SUDUT KANAN tertutup: belok kiri 90 derajat ke heading '
            f'{math.degrees(angle_error(self.corner_target_heading, 0.0)):.0f} deg')

    def update_corner_turn(self, yaw, now):
        command = Twist()
        if now - self.state_started > self.turn_timeout:
            self.deactivate('Belok kiri 90 derajat timeout')
            return
        error = angle_error(self.corner_target_heading, yaw)
        if abs(error) > self.turn_tolerance:
            self.corner_settle_started = None
            speed = clamp(self.heading_kp * abs(error),
                          self.minimum_turn_speed, self.angular_speed)
            command.angular.z = math.copysign(speed, error)
            self.publish_command(command)
            return
        if self.corner_settle_started is None:
            self.corner_settle_started = now
            self.publish_command(command)
            return
        if now - self.corner_settle_started < self.turn_settle_time:
            self.publish_command(command)
            return
        self.state = 'MOVE'
        self.segment_aligned = True
        self.corner_armed = False
        self.corner_settle_started = None
        self.get_logger().info('BELOK KIRI 90 selesai; lanjut trace_right')
        self.publish_command(command)

    def begin_right_gap_maneuver(self, x, y):
        self.state = 'GAP_FORWARD_1'
        self.state_started = time.monotonic()
        self.maneuver_start_pose = (x, y)
        self.maneuver_target_heading = None
        self.maneuver_settle_started = None
        self.right_gap_started = None
        self.gap_probe_samples = 0
        self.gap_probe_open_samples = 0
        self.cross_probe_samples = 0
        self.cross_wall_samples = 0
        self.stop_motors()
        self.get_logger().info(
            'CALON CELAH KANAN: maju uji 0.40 m sebelum memutuskan U-turn')

    def cancel_right_gap_probe(self, now):
        self.state = 'MOVE'
        self.state_started = now
        self.segment_aligned = True
        self.right_wall_seen = True
        self.right_gap_started = None
        self.maneuver_start_pose = None
        total = max(1, self.gap_probe_samples)
        ratio = self.gap_probe_open_samples / total
        self.get_logger().info(
            'KORIDOR LANJUT/NOISE: kanan kembali terlihat; '
            f'open_ratio={ratio:.2f}, lanjut trace')

    def maneuver_distance(self, x, y):
        if self.maneuver_start_pose is None:
            return 0.0
        return math.hypot(x - self.maneuver_start_pose[0],
                          y - self.maneuver_start_pose[1])

    def begin_maneuver_turn(self, yaw, next_state):
        cardinal = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        self.maneuver_target_heading = cardinal - math.pi / 2.0
        self.maneuver_settle_started = None
        self.state = next_state
        self.state_started = time.monotonic()
        self.stop_motors()

    def update_maneuver_turn(self, x, y, yaw, now, next_state):
        command = Twist()
        if now - self.state_started > self.turn_timeout:
            self.deactivate(f'{self.state}: putar kanan timeout')
            return
        error = angle_error(self.maneuver_target_heading, yaw)
        if abs(error) > self.turn_tolerance:
            self.maneuver_settle_started = None
            speed = clamp(self.heading_kp * abs(error),
                          self.minimum_turn_speed, self.angular_speed)
            command.angular.z = math.copysign(speed, error)
            self.publish_command(command)
            return
        if self.maneuver_settle_started is None:
            self.maneuver_settle_started = now
            self.publish_command(command)
            return
        if now - self.maneuver_settle_started < self.turn_settle_time:
            self.publish_command(command)
            return
        self.segment_heading = self.maneuver_target_heading
        self.state = next_state
        self.state_started = now
        self.maneuver_start_pose = (x, y)
        self.maneuver_settle_started = None
        if next_state == 'GAP_CROSS':
            self.cross_probe_samples = 0
            self.cross_wall_samples = 0
        self.get_logger().info(f'{self.state}: mulai maju')
        self.publish_command(command)

    def update_right_gap_maneuver(self, x, y, yaw, now):
        command = Twist()
        if self.state in ('GAP_FORWARD_1', 'GAP_CROSS', 'GAP_FORWARD_2'):
            if self.front <= self.maneuver_front_stop:
                self.deactivate(
                    f'{self.state}: obstacle depan {self.front:.2f} m')
                return
            # Ruas pertama adalah fase klasifikasi sepanjang 40 cm. Jangan
            # memutuskan dari satu scan: kumpulkan rasio kanan terbuka.
            if self.state == 'GAP_FORWARD_1':
                right_visible = (
                    math.isfinite(self.right) and
                    self.right < self.wall_visible)
                self.gap_probe_samples += 1
                if not right_visible:
                    self.gap_probe_open_samples += 1
            elif self.state == 'GAP_CROSS':
                right_visible = (
                    math.isfinite(self.right) and
                    self.cross_wall_min_distance <= self.right <=
                    self.cross_wall_max_distance)
                self.cross_probe_samples += 1
                if right_visible:
                    self.cross_wall_samples += 1
            targets = {
                'GAP_FORWARD_1': self.u_turn_forward_1,
                'GAP_CROSS': self.u_turn_cross,
                'GAP_FORWARD_2': self.u_turn_forward_2,
            }
            if self.maneuver_distance(x, y) < targets[self.state]:
                command.linear.x = self.u_turn_speed
                if self.segment_heading is not None:
                    heading_error = angle_error(self.segment_heading, yaw)
                    command.angular.z = clamp(
                        self.heading_kp * heading_error, -0.30, 0.30)
                self.publish_command(command)
                return
            if self.state == 'GAP_FORWARD_1':
                total = max(1, self.gap_probe_samples)
                open_ratio = self.gap_probe_open_samples / total
                right_visible = (
                    math.isfinite(self.right) and
                    self.cross_wall_min_distance <= self.right <=
                    self.cross_wall_max_distance)
                if right_visible or open_ratio < self.right_gap_open_ratio:
                    self.cancel_right_gap_probe(now)
                    self.publish_command(command)
                    return
                self.get_logger().info(
                    'BUKAAN KANAN terkonfirmasi setelah probe 0.40 m; '
                    f'open_ratio={open_ratio:.2f}; '
                    'kanan -90 lalu maju 0.40 m sambil klasifikasi')
                self.begin_maneuver_turn(yaw, 'GAP_TURN_1')
            elif self.state == 'GAP_CROSS':
                total = max(1, self.cross_probe_samples)
                wall_ratio = self.cross_wall_samples / total
                right_visible = (
                    math.isfinite(self.right) and
                    self.cross_wall_min_distance <= self.right <=
                    self.cross_wall_max_distance)
                if right_visible and wall_ratio >= self.cross_wall_ratio:
                    self.state = 'MOVE'
                    self.state_started = now
                    self.segment_aligned = True
                    self.right_wall_seen = True
                    self.right_gap_started = None
                    self.get_logger().info(
                        'KORIDOR LANJUT setelah belok kanan: '
                        f'wall_ratio={wall_ratio:.2f}; langsung trace_right')
                    self.publish_command(command)
                else:
                    self.get_logger().info(
                        'AREA TETAP TERPUTUS setelah maju 0.40 m: '
                        f'wall_ratio={wall_ratio:.2f}; putar kanan kedua')
                    self.begin_maneuver_turn(yaw, 'GAP_TURN_2')
            else:
                self.state = 'MOVE'
                self.state_started = now
                self.segment_aligned = True
                self.right_wall_seen = False
                self.right_gap_started = None
                self.get_logger().info(
                    'U-TURN selesai; mencari dan melanjutkan trace_right')
                self.publish_command(command)
            return
        if self.state == 'GAP_TURN_1':
            self.update_maneuver_turn(x, y, yaw, now, 'GAP_CROSS')
        elif self.state == 'GAP_TURN_2':
            self.update_maneuver_turn(x, y, yaw, now, 'GAP_FORWARD_2')

    def tick(self):
        command = Twist()
        now = time.monotonic()
        if self.start_pending:
            self.finish_reset()
            return
        if not self.active:
            self.publish_command(command)
            return
        if self.pose is None or now - self.last_odom > self.odom_timeout:
            self.deactivate('Odometry stale')
            return
        if now - self.last_scan > self.scan_timeout:
            self.deactivate('LiDAR stale')
            return
        if self.state == 'PAUSE':
            if now - self.state_started < self.pause_seconds:
                self.publish_command(command)
                return
            self.state = 'MOVE'
            self.light_command = 'green'

        if self.state == 'TURN_CORNER_LEFT':
            self.update_corner_turn(self.pose[2], now)
            return
        if self.state.startswith('GAP_'):
            self.update_right_gap_maneuver(*self.pose, now)
            return

        name = self.sequence[self.index]
        point = self.points[name]
        x, y, yaw = self.pose
        control = str(point.get('control', 'odometry')).lower()
        if self.segment_index != self.index:
            self.begin_segment()

        target_x, target_y = float(point['x']), float(point['y'])
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        arrival = str(point.get('arrival', 'coordinate')).lower()
        coordinate_reached = (
            distance <= float(point.get('position_tolerance', self.tolerance)))
        front_arrival_distance = float(
            point.get('front_arrival_distance', 0.40))
        # Jangan menerima tembok dari ruas sebelumnya. Setelah berbelok,
        # bagian depan harus pernah terbuka terlebih dahulu sebelum tembok
        # berikutnya boleh menandai waypoint tercapai.
        if self.front > front_arrival_distance + 0.10:
            self.front_arrival_armed = True
        front_reached = (
            self.front_arrival_armed and
            self.front <= front_arrival_distance)
        reached = (
            coordinate_reached if arrival == 'coordinate' else
            front_reached if arrival == 'front_wall' else
            coordinate_reached or front_reached)
        if reached:
            source = ('koordinat' if coordinate_reached else
                      f'tembok depan {self.front:.2f} m')
            self.get_logger().info(
                f'{name}: kondisi tiba terpenuhi oleh {source}')
            self.reach_waypoint(name)
            return
        # heading_deg mengunci arah lorong terhadap yaw nol saat START.
        # Tanpa parameter ini, heading diarahkan dinamis ke koordinat target.
        if self.segment_heading is None:
            if control in ('trace_left', 'trace_right') and self.trace_preserve_heading:
                self.segment_heading = (
                    round(yaw / (math.pi / 2.0)) * (math.pi / 2.0))
            else:
                self.segment_heading = (
                    math.radians(float(point['heading_deg']))
                    if 'heading_deg' in point else math.atan2(dy, dx))
        target_heading = self.segment_heading
        heading = angle_error(target_heading, yaw)

        # Ruas trace menghadap koordinat tujuan terlebih dahulu, sama seperti
        # waypoint odometry.
        if control in ('trace_left', 'trace_right') and not self.segment_aligned:
            if abs(heading) > self.align_tolerance:
                speed = clamp(self.heading_kp * abs(heading),
                              self.minimum_turn_speed,
                              self.angular_speed)
                command.angular.z = math.copysign(speed, heading)
                self.publish_command(command)
                return
            self.segment_aligned = True
            self.get_logger().info(
                f'{name}: heading siap, mulai {control}')
            self.publish_command(command)
            return
        if control == 'odometry':
            if abs(heading) > self.align_tolerance:
                speed = clamp(self.heading_kp * abs(heading), 0.35,
                              self.angular_speed)
                command.angular.z = math.copysign(speed, heading)
            else:
                if self.front < self.front_stop:
                    self.deactivate(f'Obstacle depan {self.front:.2f} m')
                    return
                speed = min(self.linear_speed,
                            max(self.approach_speed, distance))
                if distance < self.approach_radius:
                    speed = min(speed, self.approach_speed)
                if (arrival in ('front_wall', 'coordinate_or_front_wall') and
                        self.front < self.front_slow_distance):
                    speed = min(speed, self.approach_speed)
                command.linear.x = speed
                command.angular.z = clamp(self.heading_kp * heading,
                                          -0.45, 0.45)
        else:
            right_visible = (
                math.isfinite(self.right) and
                self.right < self.wall_visible)
            if control == 'trace_right' and right_visible:
                self.right_wall_seen = True
                self.right_gap_started = None
            elif (control == 'trace_right' and self.right_gap_u_turn and
                  self.right_wall_seen and
                  self.front > self.right_gap_front_clear):
                if self.right_gap_started is None:
                    self.right_gap_started = now
                elif now - self.right_gap_started >= self.right_gap_debounce:
                    self.begin_right_gap_maneuver(x, y)
                    return
            if self.front > self.corner_front_distance + self.corner_clear_margin:
                self.corner_armed = True
            if (control == 'trace_right' and self.auto_corner_turn and
                    self.corner_armed and
                    self.front <= self.corner_front_distance and
                    self.right <= self.corner_right_distance):
                self.begin_left_corner_turn(yaw)
                return
            if self.front < self.front_stop:
                self.deactivate(f'Obstacle depan {self.front:.2f} m')
                return
            side = 1.0 if control == 'trace_left' else -1.0
            wall_distance = self.left if side > 0.0 else self.right
            wall_angle = self.left_angle if side > 0.0 else self.right_angle
            target_wall = float(point.get('wall_distance', 0.40))
            command.linear.x = (self.approach_speed if
                                distance < self.approach_radius else
                                self.trace_speed)
            if (arrival in ('front_wall', 'coordinate_or_front_wall') and
                    self.front < self.front_slow_distance):
                command.linear.x = min(command.linear.x, self.approach_speed)
            if math.isfinite(wall_distance) and wall_distance < self.wall_visible:
                wall_error = wall_distance - target_wall
                wall_turn = side * (self.wall_kp * wall_error +
                                    self.wall_angle_kp * wall_angle)
                turn_limit = self.max_trace_turn
                if now - self.state_started < self.trace_start_grace:
                    turn_limit = min(turn_limit, self.trace_start_max_turn)
                command.angular.z = clamp(
                    wall_turn + 0.25 * self.heading_kp * heading,
                    -turn_limit, turn_limit)
            else:
                # Saat ujung dinding hilang, pertahankan arah menuju waypoint;
                # jangan membelok tajam untuk mengejar dinding yang sudah lewat.
                command.linear.x = min(command.linear.x, self.approach_speed)
                command.angular.z = clamp(
                    self.heading_kp * heading, -0.35, 0.35)
        self.publish_command(command)

    def light_tick(self):
        phase = int(time.monotonic() * 2.0) % 2 == 0
        outputs = {
            'off': (False, False, False, False),
            'red': (True, True, False, False),
            'green': (False, False, phase, False),
            'yellow': (False, False, False, phase),
            'red_blink': (False, phase, False, False),
        }
        values = outputs.get(self.light_command, outputs['off'])
        for publisher, value in zip(self.lights, values):
            publisher.publish(Bool(data=value))
        if self.light_command != self.last_light_command:
            self.get_logger().info(f'LIGHT: {self.light_command}')
            self.last_light_command = self.light_command


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = HybridCorridorNavigator(args.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_motors()
        node.close_debug_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
