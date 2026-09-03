#!/usr/bin/env python3
"""Uji mengikuti dinding kiri memakai LiDAR depan dan tombol VMX."""

import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def clamp(value, low, high):
    return max(low, min(high, value))


class LeftWallFollower(Node):
    def __init__(self):
        super().__init__('left_wall_follower')
        self.active = False
        self.last_start = None
        self.last_stop = None
        self.last_scan = 0.0
        self.front = math.inf
        self.left = math.inf
        self.wall_angle = 0.0
        self.pose = None
        self.state = 'IDLE'
        self.state_started = 0.0
        self.opening_start = None
        self.opening_missing_since = None
        self.turn_target = 0.0
        self.opening_cooldown = 0.0
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/start_button/state', self.start_cb, 10)
        self.create_subscription(Bool, '/stop_button/state', self.stop_cb, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lights = [
            self.create_publisher(Bool, '/light_control/cmd', 10),
            self.create_publisher(Bool, '/light_red/cmd', 10),
            self.create_publisher(Bool, '/light_green/cmd', 10),
            self.create_publisher(Bool, '/light_yellow/cmd', 10),
        ]
        self.create_service(Trigger, '/left_wall_follower/start', self.start_srv)
        self.create_service(Trigger, '/left_wall_follower/stop', self.stop_srv)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.light_tick)
        self.get_logger().info(
            'TRACE KIRI siap: target=0.40m, batas depan=0.25m; menunggu START DIO 10')

    @staticmethod
    def points(msg, lo_deg, hi_deg):
        result = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            degree = math.degrees(angle)
            if lo_deg <= degree <= hi_deg:
                result.append((distance * math.cos(angle),
                               distance * math.sin(angle), distance))
        return result

    def scan_cb(self, msg):
        front_points = self.points(msg, -18.0, 18.0)
        if front_points:
            values = sorted(p[2] for p in front_points)
            count = max(3, len(values) // 8)
            front = statistics.median(values[:count])
            self.front = front if not math.isfinite(self.front) else 0.55 * front + 0.45 * self.front

        # Gunakan dua kelompok sudut, bukan satu sinar: sekitar 60 dan 77.5
        # derajat. Median setiap kelompok meredam spike LiDAR. Garis melalui
        # kedua titik memberi sudut dinding dan jarak dinding pada sisi robot.
        band_60 = self.points(msg, 55.0, 65.0)
        band_80 = self.points(msg, 75.0, 80.0)
        if len(band_60) >= 3 and len(band_80) >= 3:
            x60 = statistics.median(p[0] for p in band_60)
            y60 = statistics.median(p[1] for p in band_60)
            x80 = statistics.median(p[0] for p in band_80)
            y80 = statistics.median(p[1] for p in band_80)
            dx = x60 - x80
            dy = y60 - y80
            angle = math.atan2(dy, dx) if abs(dx) > 1e-4 else 0.0
            slope = dy / dx if abs(dx) > 1e-4 else 0.0
            distance = y80 - slope * x80
            distance = clamp(distance, 0.05, 3.0)
            self.left = distance if not math.isfinite(self.left) else 0.35 * distance + 0.65 * self.left
            self.wall_angle = 0.35 * angle + 0.65 * self.wall_angle
        else:
            self.left = math.inf
        self.last_scan = time.monotonic()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    @staticmethod
    def angle_error(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def start_cb(self, msg):
        pressed = (not msg.data) and self.last_start is False
        self.last_start = not msg.data
        if pressed:
            self.activate()

    def stop_cb(self, msg):
        pressed = (not msg.data) and self.last_stop is False
        self.last_stop = not msg.data
        if pressed:
            self.deactivate('STOP DIO 11')

    def activate(self):
        if time.monotonic() - self.last_scan > 0.6:
            self.get_logger().error('START ditolak: data /scan tidak tersedia')
            return False
        if self.pose is None:
            self.get_logger().error('START ditolak: /odom belum tersedia')
            return False
        self.active = True
        self.state = 'FOLLOW'
        self.opening_missing_since = None
        self.opening_cooldown = time.monotonic() + 1.0
        self.get_logger().info('START: mulai mengikuti dinding kiri')
        return True

    def deactivate(self, source):
        self.active = False
        self.state = 'IDLE'
        self.cmd.publish(Twist())
        self.get_logger().warning(f'{source}: motor STOP')

    def start_srv(self, _req, response):
        response.success = self.activate()
        response.message = 'trace kiri aktif' if response.success else '/scan belum siap'
        return response

    def stop_srv(self, _req, response):
        self.deactivate('service STOP')
        response.success = True
        response.message = 'trace kiri berhenti'
        return response

    def tick(self):
        command = Twist()
        if not self.active:
            self.cmd.publish(command)
            return
        if time.monotonic() - self.last_scan > 0.6:
            self.deactivate('LiDAR timeout')
            return
        if self.pose is None:
            self.deactivate('Odometri belum tersedia')
            return

        now = time.monotonic()
        x, y, yaw = self.pose
        wall_visible = math.isfinite(self.left) and self.left < 1.20

        # Tembok depan selalu mendapat prioritas tertinggi.
        if self.front < 0.25 and self.state not in ('STOP_BEFORE_RIGHT', 'TURN_RIGHT'):
            self.state = 'STOP_BEFORE_RIGHT'
            self.state_started = now
            self.get_logger().warning(
                f'DEPAN TERHALANG {self.front:.2f}m: STOP lalu putar kanan 90 derajat')

        if self.state == 'STOP_BEFORE_RIGHT':
            if now - self.state_started >= 0.35:
                self.turn_target = self.angle_error(yaw - math.pi / 2.0, 0.0)
                self.state = 'TURN_RIGHT'
        elif self.state in ('TURN_RIGHT', 'TURN_LEFT'):
            error = self.angle_error(self.turn_target, yaw)
            if abs(error) <= math.radians(5.0):
                old_state = self.state
                self.state = 'FOLLOW'
                self.opening_cooldown = now + 1.2
                self.opening_missing_since = None
                self.get_logger().info(f'{old_state} selesai; lanjut trace kiri')
            else:
                speed = 1.10 if abs(error) > math.radians(15.0) else 0.60
                command.angular.z = math.copysign(speed, error)
        elif self.state == 'PASS_LEFT_OPENING':
            travelled = math.hypot(x - self.opening_start[0],
                                    y - self.opening_start[1])
            if travelled >= 0.45:
                self.turn_target = self.angle_error(yaw + math.pi / 2.0, 0.0)
                self.state = 'TURN_LEFT'
                self.get_logger().info(
                    'Bodi sudah melewati ujung tembok; putar kiri 90 derajat')
            else:
                command.linear.x = 0.16
        elif not wall_visible:
            if now < self.opening_cooldown:
                command.linear.x = 0.10
                command.angular.z = 0.12
            else:
                if self.opening_missing_since is None:
                    self.opening_missing_since = now
                if now - self.opening_missing_since >= 0.35:
                    self.state = 'PASS_LEFT_OPENING'
                    self.opening_start = (x, y)
                    self.get_logger().info(
                        'Bukaan kiri terdeteksi; maju 0.45m agar bodi tidak menyangkut')
                else:
                    command.linear.x = 0.14
        else:
            self.opening_missing_since = None
            error = self.left - 0.40
            command.angular.z = clamp(1.8 * error + 0.9 * self.wall_angle,
                                      -0.60, 0.60)
            command.linear.x = 0.10 if abs(command.angular.z) > 0.28 else 0.16
        self.cmd.publish(command)

    def light_tick(self):
        values = (False, False, True, False) if self.active else (True, True, False, False)
        for publisher, value in zip(self.lights, values):
            publisher.publish(Bool(data=value))


def main():
    rclpy.init()
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.deactivate('Ctrl+C')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
