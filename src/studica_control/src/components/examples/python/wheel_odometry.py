#!/usr/bin/env python3
"""Odometri skid-steer dari empat encoder jarak Titan.

Pemetaan robot:
  M0 depan kanan, M1 belakang kanan
  M2 depan kiri,  M3 belakang kiri

Semua encoder harus sudah dinormalisasi sehingga gerak maju bernilai positif.
Node menerbitkan nav_msgs/Odometry pada /odom dan TF odom -> base_link.
"""

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry')

        self.declare_parameter('sensor', 'titan0')
        self.declare_parameter('track_width', 0.35)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_encoder_delta', 0.25)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('max_imu_delta', 0.50)
        self.declare_parameter('imu_yaw_sign', 1.0)

        sensor = self.get_parameter('sensor').value
        self.track_width = float(self.get_parameter('track_width').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_encoder_delta = float(
            self.get_parameter('max_encoder_delta').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        imu_topic = self.get_parameter('imu_topic').value
        self.max_imu_delta = float(self.get_parameter('max_imu_delta').value)
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)

        self.encoder_values: List[Optional[float]] = [None] * 4
        self.previous_values: Optional[List[float]] = None
        self.previous_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.imu_yaw: Optional[float] = None
        self.previous_imu_yaw: Optional[float] = None

        self.encoder_subscriptions = [
            self.create_subscription(
                Float64,
                f'/{sensor}/m_{motor}/encoder',
                lambda msg, index=motor: self.encoder_callback(index, msg),
                10,
            )
            for motor in range(4)
        ]
        self.imu_subscription = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.reset_service = self.create_service(
            Empty, '/wheel_odometry/reset', self.reset_callback)
        self.timer = self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            f'Odometri siap: track_width={self.track_width:.3f} m, '
            f'encoder=/{sensor}/m_0..m_3/encoder, '
            f'yaw={imu_topic if self.use_imu_yaw else "wheel differential"}')

    def encoder_callback(self, motor: int, msg: Float64) -> None:
        self.encoder_values[motor] = float(msg.data)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(sin_yaw, cos_yaw)

    def reset_callback(self, _request, response):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.previous_imu_yaw = self.imu_yaw
        if all(value is not None for value in self.encoder_values):
            self.previous_values = list(self.encoder_values)
        else:
            self.previous_values = None
        self.previous_time = self.get_clock().now()
        self.get_logger().info('Pose odometri di-reset ke (0, 0, 0)')
        return response

    @staticmethod
    def quaternion_from_yaw(yaw: float):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def update(self) -> None:
        if not all(value is not None for value in self.encoder_values):
            return

        now = self.get_clock().now()
        current = list(self.encoder_values)
        if self.previous_values is None or self.previous_time is None:
            self.previous_values = current
            self.previous_time = now
            self.previous_imu_yaw = self.imu_yaw
            return

        dt = (now - self.previous_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        deltas = [
            value - previous
            for value, previous in zip(current, self.previous_values)
        ]
        self.previous_values = current
        self.previous_time = now

        # Abaikan lompatan akibat reset encoder atau restart Titan.
        if any(abs(delta) > self.max_encoder_delta for delta in deltas):
            self.get_logger().warning(
                f'Lompatan encoder diabaikan: {deltas}',
                throttle_duration_sec=2.0)
            return

        right_distance = 0.5 * (deltas[0] + deltas[1])
        left_distance = 0.5 * (deltas[2] + deltas[3])
        linear_distance = 0.5 * (right_distance + left_distance)
        wheel_angular_distance = (
            right_distance - left_distance) / self.track_width
        angular_distance = wheel_angular_distance
        if self.use_imu_yaw and self.imu_yaw is not None:
            if self.previous_imu_yaw is None:
                self.previous_imu_yaw = self.imu_yaw
                angular_distance = 0.0
            else:
                imu_delta = self.normalize_angle(
                    self.imu_yaw - self.previous_imu_yaw)
                self.previous_imu_yaw = self.imu_yaw
                if abs(imu_delta) <= self.max_imu_delta:
                    angular_distance = self.imu_yaw_sign * imu_delta
                else:
                    self.get_logger().warning(
                        f'Lompatan yaw IMU diabaikan: {imu_delta:.3f} rad',
                        throttle_duration_sec=2.0)

        heading_midpoint = self.yaw + 0.5 * angular_distance
        self.x += linear_distance * math.cos(heading_midpoint)
        self.y += linear_distance * math.sin(heading_midpoint)
        self.yaw = self.normalize_angle(self.yaw + angular_distance)

        linear_velocity = linear_distance / dt
        angular_velocity = angular_distance / dt
        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
