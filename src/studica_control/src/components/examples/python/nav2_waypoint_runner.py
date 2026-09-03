#!/usr/bin/env python3
"""Runner kompetisi: tombol, initial pose AMCL, waypoint Nav2, dan lampu."""

import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener
import yaml


class Nav2WaypointRunner(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('nav2_waypoint_runner')
        with Path(config_path).open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        self.configured = bool(config.get('configured', False))
        self.home = config['home']
        self.waypoints = config['waypoints']
        self.sequence = list(config['sequence'])
        for name in self.sequence:
            if name not in self.waypoints:
                raise ValueError(f'Waypoint {name} tidak ditemukan')
        if self.configured:
            if not self.sequence or self.sequence[-1] != 'S':
                raise ValueError('Sequence aktif wajib berakhir dengan S/HOME')
            home_error = math.hypot(
                float(self.waypoints['S']['x']) - float(self.home['x']),
                float(self.waypoints['S']['y']) - float(self.home['y']))
            if home_error > 0.02:
                raise ValueError('Koordinat waypoint S harus sama dengan HOME')
        self.pause_seconds = max(
            5.0, float(config.get('waypoint_pause_seconds', 5.2)))
        self.final_light = str(config.get('final_light', 'red_blink'))
        buttons = config.get('buttons', {})
        self.start_active_high = bool(buttons.get('start_active_high', False))
        self.stop_active_high = bool(buttons.get('stop_active_high', False))
        self.last_start = None
        self.last_stop = None
        self.state = 'IDLE'
        self.index = 0
        self.deadline = 0.0
        self.localization_started = 0.0
        self.nav_ready_deadline = 0.0
        self.next_goal_attempt = 0.0
        self.bt_state_future = None
        self.reset_future = None
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None

        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.reset_client = self.create_client(Empty, '/wheel_odometry/reset')
        self.bt_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')
        self.create_service(Trigger, '/competition_navigation/start',
                            self.start_service)
        self.create_service(Trigger, '/competition_navigation/stop',
                            self.stop_service)
        self.create_subscription(Bool, '/start_button/state',
                                 self.start_button, 10)
        self.create_subscription(Bool, '/stop_button/state',
                                 self.stop_button, 10)
        initial_qos = QoSProfile(depth=1)
        initial_qos.reliability = ReliabilityPolicy.RELIABLE
        initial_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initial_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stop_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.light_publishers = {
            'control': self.create_publisher(Bool, '/light_control/cmd', 10),
            'red': self.create_publisher(Bool, '/light_red/cmd', 10),
            'green': self.create_publisher(Bool, '/light_green/cmd', 10),
            'yellow': self.create_publisher(Bool, '/light_yellow/cmd', 10),
        }
        self.timer = self.create_timer(0.1, self.tick)
        self.light_timer = self.create_timer(0.5, self.publish_light)
        self.get_logger().info(
            f'Siap: home={self.home}, sequence={self.sequence}; '
            'menunggu START DIO 10')

    @staticmethod
    def quaternion(yaw_deg: float):
        half = math.radians(yaw_deg) * 0.5
        return math.sin(half), math.cos(half)

    def start_button(self, msg: Bool) -> None:
        active = bool(msg.data) == self.start_active_high
        pressed = active and self.last_start is False
        self.last_start = active
        if pressed:
            ok, message = self.request_start()
            (self.get_logger().info if ok else self.get_logger().error)(
                'START: ' + message)

    def stop_button(self, msg: Bool) -> None:
        active = bool(msg.data) == self.stop_active_high
        pressed = active and self.last_stop is False
        self.last_stop = active
        if pressed:
            self.request_stop('STOP DIO 11')

    def start_service(self, _request, response):
        response.success, response.message = self.request_start()
        return response

    def stop_service(self, _request, response):
        self.request_stop('service STOP')
        response.success = True
        response.message = 'Navigasi dihentikan'
        return response

    def request_start(self):
        if not self.configured:
            return False, 'configured=false; isi HOME dan waypoint dahulu'
        if self.state not in ('IDLE', 'DONE', 'FAILED'):
            return False, f'masih aktif ({self.state})'
        if self.last_stop is True:
            return False, 'tombol STOP masih ditekan'
        if not self.reset_client.service_is_ready():
            return False, '/wheel_odometry/reset belum tersedia'
        if not self.action_client.server_is_ready():
            return False, '/navigate_to_pose belum tersedia'
        self.index = 0
        self.state = 'RESETTING'
        self.reset_future = self.reset_client.call_async(Empty.Request())
        return True, 'zero yaw/reset odom dimulai; motor tetap STOP'

    def request_stop(self, source: str) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        for _ in range(5):
            self.stop_publisher.publish(Twist())
        self.goal_handle = None
        self.state = 'IDLE'
        self.get_logger().warning(f'{source}: navigasi dan motor STOP')

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        # Stamp nol meminta TF terbaru dan mencegah extrapolation error karena
        # publikasi odometri sedikit tertinggal dari waktu saat ini.
        msg.header.stamp = Time().to_msg()
        msg.pose.pose.position.x = float(self.home['x'])
        msg.pose.pose.position.y = float(self.home['y'])
        qz, qw = self.quaternion(float(self.home.get('yaw_deg', 0.0)))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = math.radians(3.0) ** 2
        self.initial_pose_pub.publish(msg)

    def send_current_goal(self) -> None:
        name = self.sequence[self.index]
        point = self.waypoints[name]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        qz, qw = self.quaternion(float(point.get('yaw_deg', 0.0)))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.state = 'SENDING'
        self.goal_future = self.action_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_response)
        self.get_logger().info(
            f'GOAL {name}: ({point["x"]:.2f}, {point["y"]:.2f})')

    def goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            if time.monotonic() < self.nav_ready_deadline:
                self.state = 'WAITING_NAV2'
                self.next_goal_attempt = time.monotonic() + 1.0
                self.get_logger().warning(
                    'Goal belum diterima; menunggu lifecycle Nav2 aktif')
            else:
                self.fail('Goal ditolak Nav2 setelah timeout aktivasi')
            return
        self.goal_handle = handle
        self.state = 'NAVIGATING'
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result)

    def goal_result(self, future) -> None:
        result = future.result()
        self.goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(f'Navigasi gagal, status={result.status}')
            return
        name = self.sequence[self.index]
        self.state = 'WAITING'
        self.deadline = time.monotonic() + self.pause_seconds
        self.get_logger().info(
            f'WAYPOINT {name} tercapai; STOP {self.pause_seconds:.1f} detik')

    def fail(self, message: str) -> None:
        self.state = 'FAILED'
        self.goal_handle = None
        self.get_logger().error(message + '; STOP')

    def tick(self) -> None:
        if self.state == 'RESETTING' and self.reset_future.done():
            try:
                self.reset_future.result()
            except Exception as error:
                self.fail(f'Reset odometri gagal: {error}')
                return
            self.state = 'LOCALIZING'
            self.localization_started = time.monotonic()
            self.deadline = self.localization_started + 12.0
            self.get_logger().info(
                'Zero yaw selesai; menetapkan pose HOME dan menunggu TF map -> base_link')
        elif self.state == 'LOCALIZING':
            self.publish_initial_pose()
            localized = self.tf_buffer.can_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.0))
            # Beri AMCL sedikit waktu menerima beberapa scan setelah TF muncul.
            if localized and time.monotonic() - self.localization_started >= 1.0:
                self.get_logger().info(
                    'Lokalisasi AMCL siap; menunggu Nav2 menerima goal')
                self.state = 'WAITING_NAV2'
                self.nav_ready_deadline = time.monotonic() + 20.0
                self.next_goal_attempt = time.monotonic() + 1.0
            elif time.monotonic() >= self.deadline:
                self.fail(
                    'Lokalisasi AMCL timeout: TF map -> base_link belum tersedia')
        elif self.state == 'WAITING_NAV2':
            now = time.monotonic()
            if now >= self.nav_ready_deadline:
                self.fail('Nav2 tidak aktif/tidak menerima goal selama 20 detik')
            elif self.bt_state_future is not None and self.bt_state_future.done():
                try:
                    active = self.bt_state_future.result().current_state.id == 3
                except Exception as error:
                    self.get_logger().warning(
                        f'Gagal membaca lifecycle bt_navigator: {error}')
                    active = False
                self.bt_state_future = None
                if active:
                    self.get_logger().info(
                        'bt_navigator ACTIVE; mengirim waypoint')
                    self.send_current_goal()
                else:
                    self.next_goal_attempt = now + 0.5
            elif (self.bt_state_future is None and
                  now >= self.next_goal_attempt and
                  self.bt_state_client.service_is_ready()):
                self.bt_state_future = self.bt_state_client.call_async(
                    GetState.Request())
        elif self.state == 'WAITING' and time.monotonic() >= self.deadline:
            self.index += 1
            if self.index >= len(self.sequence):
                self.state = 'DONE'
                self.get_logger().info('HOME tercapai; seluruh navigasi selesai')
            else:
                self.send_current_goal()

    def publish_light(self) -> None:
        if self.state == 'NAVIGATING':
            values = (False, False, True, False)  # green blink
        elif self.state == 'WAITING':
            values = (False, False, False, True)  # yellow blink
        elif self.state == 'DONE' and self.final_light == 'red_blink':
            values = (False, True, False, False)
        else:
            values = (True, True, False, False)  # red solid
        for publisher, value in zip(self.light_publishers.values(), values):
            publisher.publish(Bool(data=value))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Nav2WaypointRunner(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.request_stop('Ctrl+C')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
