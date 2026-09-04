#!/usr/bin/env python3
"""Lightweight, configurable RGB-D object detector for the Orbbec Gemini E."""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


class RgbdObjectDetector(Node):
    def __init__(self):
        super().__init__('rgbd_object_detector')
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.last_process_time = 0.0
        self.latest_color = None
        self.tracks = {}
        self.next_track_id = 1

        defaults = {
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'color_topic': '/camera/color/image_raw',
            'min_depth_mm': 200,
            'max_depth_mm': 2500,
            'background_depth_mm': 0,
            'foreground_gap_mm': 35,
            'min_area_px': 250,
            'max_area_px': 120000,
            'detection_rate_hz': 8.0,
            'debug_jpeg_quality': 70,
            'min_color_saturation': 45,
            'roi_x': 0,
            'roi_y': 0,
            'roi_width': 0,
            'roi_height': 0,
            'show_roi': True,
            'tracking_enabled': True,
            'tracking_match_distance_m': 0.30,
            'tracking_max_age_sec': 1.0,
            'tracking_smoothing': 0.65,
            'object_1_width_min': 50.0,
            'object_1_width_max': 75.0,
            'object_1_height_min': 100.0,
            'object_1_height_max': 160.0,
            'object_2_width_min': 80.0,
            'object_2_width_max': 110.0,
            'object_2_length_min': 120.0,
            'object_2_length_max': 180.0,
            'object_2_height_min': 40.0,
            'object_2_height_max': 60.0,
            'object_3_width_min': 120.0,
            'object_3_width_max': 160.0,
            'object_3_height_min': 50.0,
            'object_3_height_max': 70.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.result_pub = self.create_publisher(String, '/object_detection/results', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, '/object_detection/debug_image/compressed', 2)
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self.color_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info('RGB-D detector ready; waiting for depth and camera info')

    def info_callback(self, message):
        self.fx, self.fy = float(message.k[0]), float(message.k[4])
        self.cx, self.cy = float(message.k[2]), float(message.k[5])

    def p(self, name):
        return self.get_parameter(name).value

    def color_callback(self, message):
        channels = 3
        if message.encoding.lower() not in ('bgr8', 'rgb8'):
            return
        row_values = message.step
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, row_values)[:, :message.width * channels]
        image = image.reshape(message.height, message.width, channels)
        if message.encoding.lower() == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.latest_color = image.copy()

    def detect_color(self, x, y, width, height, depth_shape):
        if self.latest_color is None:
            return 'unknown', (0, 165, 255)
        color_image = self.latest_color
        depth_height, depth_width = depth_shape
        scale_x = color_image.shape[1] / float(depth_width)
        scale_y = color_image.shape[0] / float(depth_height)
        # Use the centre of the box to avoid sampling background at its edges.
        margin = 0.20
        x1 = int((x + width * margin) * scale_x)
        y1 = int((y + height * margin) * scale_y)
        x2 = int((x + width * (1.0 - margin)) * scale_x)
        y2 = int((y + height * (1.0 - margin)) * scale_y)
        roi = color_image[max(y1, 0):min(y2, color_image.shape[0]),
                          max(x1, 0):min(x2, color_image.shape[1])]
        if roi.size == 0:
            return 'unknown', (0, 165, 255)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        saturation_limit = int(self.p('min_color_saturation'))
        saturated = hsv[hsv[:, 1] >= saturation_limit]
        sample = saturated if saturated.size else hsv
        hue, saturation, value = np.median(sample, axis=0)
        if value < 45:
            return 'black', (40, 40, 40)
        if saturation < saturation_limit:
            if value > 190:
                return 'white', (240, 240, 240)
            return 'gray', (140, 140, 140)
        if hue < 10 or hue >= 170:
            return 'red', (0, 0, 255)
        if hue < 22:
            return 'orange', (0, 140, 255)
        if hue < 35:
            return 'yellow', (0, 255, 255)
        if hue < 85:
            return 'green', (0, 200, 0)
        if hue < 130:
            return 'blue', (255, 80, 0)
        if hue < 170:
            return 'purple', (200, 0, 200)
        return 'unknown', (0, 165, 255)

    @staticmethod
    def in_range(value, low, high, tolerance=0.20):
        margin = (high - low) * tolerance
        return low - margin <= value <= high + margin

    def classify(self, width_mm, height_mm, circularity):
        short_side, long_side = sorted((width_mm, height_mm))
        if (self.in_range(width_mm, self.p('object_1_width_min'), self.p('object_1_width_max'))
                and self.in_range(height_mm, self.p('object_1_height_min'), self.p('object_1_height_max'))):
            return 1, 'cylinder'
        if (self.in_range(short_side, self.p('object_2_width_min'), self.p('object_2_width_max'))
                and self.in_range(long_side, self.p('object_2_length_min'), self.p('object_2_length_max'))):
            return 2, 'rectangular'
        if (self.in_range(width_mm, self.p('object_3_width_min'), self.p('object_3_width_max'))
                and self.in_range(height_mm, self.p('object_3_height_min'), self.p('object_3_height_max'))
                and circularity > 0.45):
            return 3, 'semi-spherical'
        return 0, 'unknown'

    def update_track(self, detection, used_tracks, timestamp):
        if not bool(self.p('tracking_enabled')):
            detection['track_id'] = 0
            return
        position = detection['position']
        point = np.array([position['x_m'], position['y_m'], position['z_m']])
        best_id, best_distance = None, float(self.p('tracking_match_distance_m'))
        for track_id, track in self.tracks.items():
            if track_id in used_tracks:
                continue
            distance = float(np.linalg.norm(point - track['point']))
            if distance < best_distance:
                best_id, best_distance = track_id, distance
        if best_id is None:
            best_id = self.next_track_id
            self.next_track_id += 1
            smoothed = point
        else:
            alpha = float(np.clip(self.p('tracking_smoothing'), 0.0, 0.95))
            smoothed = alpha * self.tracks[best_id]['point'] + (1.0 - alpha) * point
        self.tracks[best_id] = {'point': smoothed, 'seen': timestamp}
        used_tracks.add(best_id)
        detection['track_id'] = best_id
        detection['position'] = {
            'x_m': round(float(smoothed[0]), 3),
            'y_m': round(float(smoothed[1]), 3),
            'z_m': round(float(smoothed[2]), 3),
        }

    def expire_tracks(self, timestamp):
        max_age = float(self.p('tracking_max_age_sec'))
        self.tracks = {
            track_id: track for track_id, track in self.tracks.items()
            if timestamp - track['seen'] <= max_age
        }

    @staticmethod
    def display_label(class_id, class_name):
        if class_id == 0:
            return 'Object #4 / Unknown'
        pretty_name = class_name.replace('-', ' ').title()
        return f'Object #{class_id} - {pretty_name}'

    def depth_callback(self, message):
        now = time.monotonic()
        period = 1.0 / max(float(self.p('detection_rate_hz')), 0.1)
        if now - self.last_process_time < period or self.fx <= 0.0 or self.fy <= 0.0:
            return
        self.last_process_time = now

        if message.encoding not in ('16UC1', 'mono16'):
            self.get_logger().error(
                f'Unsupported depth encoding: {message.encoding}; expected 16UC1',
                throttle_duration_sec=5.0,
            )
            return
        row_values = message.step // 2
        depth = np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, row_values)[:, :message.width]
        if bool(message.is_bigendian) != (np.dtype(np.uint16).byteorder == '>'):
            depth = depth.byteswap()
        depth = depth.astype(np.float32, copy=False)
        valid = (depth >= self.p('min_depth_mm')) & (depth <= self.p('max_depth_mm'))
        values = depth[valid]
        if values.size < 100:
            self.publish([], np.zeros((*depth.shape, 3), dtype=np.uint8), message)
            return

        configured_background = float(self.p('background_depth_mm'))
        background = configured_background if configured_background > 0 else float(np.percentile(values, 85))
        mask = valid & (depth < background - float(self.p('foreground_gap_mm')))
        mask = (mask.astype(np.uint8) * 255)

        roi_x = max(0, int(self.p('roi_x')))
        roi_y = max(0, int(self.p('roi_y')))
        roi_width = int(self.p('roi_width'))
        roi_height = int(self.p('roi_height'))
        if roi_width > 0 and roi_height > 0:
            roi_x2 = min(depth.shape[1] - 1, roi_x + roi_width)
            roi_y2 = min(depth.shape[0] - 1, roi_y + roi_height)
            roi_mask = np.zeros_like(mask)
            roi_mask[roi_y:roi_y2, roi_x:roi_x2] = 255
            mask = cv2.bitwise_and(mask, roi_mask)
        else:
            roi_x, roi_y = 0, 0
            roi_x2, roi_y2 = depth.shape[1] - 1, depth.shape[0] - 1
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Draw results over the live RGB camera image. Resize only the display
        # copy when needed; depth registration keeps both coordinate systems
        # aligned at the camera driver level.
        if self.latest_color is not None:
            debug = cv2.resize(
                self.latest_color,
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
        else:
            display_depth = np.clip(
                depth, self.p('min_depth_mm'), self.p('max_depth_mm'))
            display_depth = cv2.normalize(
                display_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            debug = cv2.applyColorMap(
                255 - display_depth, cv2.COLORMAP_TURBO)
            cv2.putText(
                debug, 'Waiting for RGB stream', (12, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if bool(self.p('show_roi')):
            cv2.rectangle(
                debug, (roi_x, roi_y), (roi_x2, roi_y2), (255, 255, 255), 1)
            cv2.putText(
                debug, 'Detection ROI', (roi_x + 5, max(roi_y + 18, 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        used_tracks = set()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.p('min_area_px') or area > self.p('max_area_px'):
                continue
            x, y, width_px, height_px = cv2.boundingRect(contour)
            object_depth = depth[y:y + height_px, x:x + width_px]
            object_mask = mask[y:y + height_px, x:x + width_px] > 0
            samples = object_depth[object_mask & (object_depth > 0)]
            if samples.size == 0:
                continue
            distance_mm = float(np.median(samples))
            width_mm = width_px * distance_mm / self.fx
            height_mm = height_px * distance_mm / self.fy
            perimeter = cv2.arcLength(contour, True)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
            class_id, class_name = self.classify(width_mm, height_mm, circularity)
            object_label = self.display_label(class_id, class_name)
            color_name, box_color = self.detect_color(
                x, y, width_px, height_px, depth.shape)
            center_u, center_v = x + width_px / 2.0, y + height_px / 2.0
            position = {
                'x_m': round((center_u - self.cx) * distance_mm / self.fx / 1000.0, 3),
                'y_m': round((center_v - self.cy) * distance_mm / self.fy / 1000.0, 3),
                'z_m': round(distance_mm / 1000.0, 3),
            }
            detection = {
                'class_id': class_id,
                'class_name': class_name,
                'label': object_label,
                'color': color_name,
                'position': position,
                'width_mm': round(width_mm, 1),
                'height_mm': round(height_mm, 1),
                'circularity': round(circularity, 2),
            }
            self.update_track(detection, used_tracks, now)
            position = detection['position']
            detections.append(detection)
            cv2.rectangle(
                debug, (x, y), (x + width_px, y + height_px), box_color, 3)
            track_prefix = (
                f"Track #{detection['track_id']} | "
                if detection['track_id'] else '')
            label = (
                f"{track_prefix}{object_label} - {color_name.title()} | "
                f"{distance_mm / 1000.0:.2f}m")
            coordinate_label = (
                f"X:{position['x_m']:+.2f}m Y:{position['y_m']:+.2f}m "
                f"D:{position['z_m']:.2f}m")
            size_label = f"Size: {width_mm:.0f} x {height_mm:.0f} mm"
            text_y = max(y - 43, 18)
            cv2.putText(
                debug, label, (x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, box_color, 2)
            cv2.putText(
                debug, coordinate_label, (x, text_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1)
            cv2.putText(
                debug, size_label, (x, text_y + 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1)

        self.expire_tracks(now)
        self.publish(detections, debug, message)

    def publish(self, detections, debug, source_message):
        result = String()
        result.data = json.dumps({'count': len(detections), 'objects': detections})
        self.result_pub.publish(result)
        quality = int(np.clip(self.p('debug_jpeg_quality'), 20, 95))
        encoded_ok, encoded = cv2.imencode(
            '.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not encoded_ok:
            return
        image = CompressedImage()
        image.header = source_message.header
        image.format = 'jpeg'
        image.data = encoded.tobytes()
        self.image_pub.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
