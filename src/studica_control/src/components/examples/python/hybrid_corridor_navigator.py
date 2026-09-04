#!/usr/bin/env python3
"""Waypoint corridor dengan kontrol per ruas: odometry/trace_left/trace_right."""

import argparse
import math
import statistics
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
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
        self.active = False
        self.ready = False
        self.start_pending = False
        self.reset_future = None
        self.reset_requested = 0.0
        self.index = 0
        self.segment_index = -1
        self.segment_aligned = False
        self.front_arrival_armed = False
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

    @staticmethod
    def wall_measurement(msg, side):
        sign = 1.0 if side == 'left' else -1.0
        near = HybridCorridorNavigator.sector(
            msg, 55.0, 65.0) if side == 'left' else \
            HybridCorridorNavigator.sector(msg, -65.0, -55.0)
        far = HybridCorridorNavigator.sector(
            msg, 75.0, 82.0) if side == 'left' else \
            HybridCorridorNavigator.sector(msg, -82.0, -75.0)
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
            self.cmd.publish(Twist())

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
        self.front_arrival_armed = False
        self.state_started = time.monotonic()

    def tick(self):
        command = Twist()
        now = time.monotonic()
        if self.start_pending:
            self.finish_reset()
            return
        if not self.active:
            self.cmd.publish(command)
            return
        if self.pose is None or now - self.last_odom > self.odom_timeout:
            self.deactivate('Odometry stale')
            return
        if now - self.last_scan > self.scan_timeout:
            self.deactivate('LiDAR stale')
            return
        if self.state == 'PAUSE':
            if now - self.state_started < self.pause_seconds:
                self.cmd.publish(command)
                return
            self.state = 'MOVE'
            self.light_command = 'green'

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
        target_heading = (
            math.radians(float(point['heading_deg']))
            if 'heading_deg' in point else math.atan2(dy, dx))
        heading = angle_error(target_heading, yaw)

        # Ruas trace menghadap koordinat tujuan terlebih dahulu, sama seperti
        # waypoint odometry.
        if control in ('trace_left', 'trace_right') and not self.segment_aligned:
            if abs(heading) > self.align_tolerance:
                speed = clamp(self.heading_kp * abs(heading),
                              self.minimum_turn_speed,
                              self.angular_speed)
                command.angular.z = math.copysign(speed, heading)
                self.cmd.publish(command)
                return
            self.segment_aligned = True
            self.get_logger().info(
                f'{name}: heading siap, mulai {control}')
            self.cmd.publish(command)
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
                command.angular.z = clamp(
                    wall_turn + 0.25 * self.heading_kp * heading,
                    -self.max_trace_turn, self.max_trace_turn)
            else:
                # Saat ujung dinding hilang, pertahankan arah menuju waypoint;
                # jangan membelok tajam untuk mengejar dinding yang sudah lewat.
                command.linear.x = min(command.linear.x, self.approach_speed)
                command.angular.z = clamp(
                    self.heading_kp * heading, -0.35, 0.35)
        self.cmd.publish(command)

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
