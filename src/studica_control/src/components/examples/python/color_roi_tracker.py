#!/usr/bin/env python3
"""Teach a color from an ROI, then track it continuously with HSV thresholding."""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
import yaml


class ColorRoiTracker(Node):
    def __init__(self):
        super().__init__('color_roi_tracker')
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'object_label': 'Object #1',
            'min_contour_area': 200,
            'morphology_kernel': 5,
            'hsv_margin_h': 8,
            'hsv_margin_s': 25,
            'hsv_margin_v': 25,
            'calibration_file': '/home/vmx/studica_ws/config/color_object_1_hsv.yaml',
            'load_saved_calibration': True,
            'jpeg_quality': 75,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.color_frame = None
        self.depth_frame = None
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.hsv_limits = None
        self.window = 'HSV ROI Teach and Track'
        self.result_pub = self.create_publisher(String, '/color_tracker/result', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, '/color_tracker/image/compressed', 2)
        self.mask_pub = self.create_publisher(
            CompressedImage, '/color_tracker/mask/compressed', 2)
        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self.color_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, qos_profile_sensor_data)
        self.create_timer(0.03, self.display_and_track)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 960, 540)
        if bool(self.get_parameter('load_saved_calibration').value):
            self.load_calibration()
        self.get_logger().info('Press R to select Object #1 ROI; Q/Esc exits')

    def p(self, name):
        return self.get_parameter(name).value

    def color_callback(self, message):
        if message.encoding.lower() not in ('bgr8', 'rgb8'):
            return
        row = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        frame = row[:, :message.width * 3].reshape(message.height, message.width, 3)
        if message.encoding.lower() == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.color_frame = frame.copy()

    def depth_callback(self, message):
        if message.encoding not in ('16UC1', 'mono16'):
            return
        row_values = message.step // 2
        depth = np.frombuffer(message.data, np.uint16).reshape(
            message.height, row_values)[:, :message.width]
        if bool(message.is_bigendian) != (np.dtype(np.uint16).byteorder == '>'):
            depth = depth.byteswap()
        self.depth_frame = depth.copy()

    def info_callback(self, message):
        self.fx, self.fy = float(message.k[0]), float(message.k[4])
        self.cx, self.cy = float(message.k[2]), float(message.k[5])

    def teach_roi(self):
        if self.color_frame is None:
            return
        roi = cv2.selectROI(
            'Select Object #1 - ENTER to accept', self.color_frame,
            showCrosshair=True, fromCenter=False)
        cv2.destroyWindow('Select Object #1 - ENTER to accept')
        x, y, width, height = (int(value) for value in roi)
        if width <= 0 or height <= 0:
            return
        sample = self.color_frame[y:y + height, x:x + width]
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        # Ignore very dark/desaturated background pixels inside the ROI.
        useful = hsv[(hsv[:, 1] > 25) & (hsv[:, 2] > 30)]
        if useful.shape[0] < 20:
            useful = hsv
        hues = useful[:, 0].astype(float)
        # Red straddles OpenCV hue 179/0. Store low_h > high_h to represent
        # a wrapped range and handle it with two masks during tracking.
        wrapped_hues = np.where(hues < 90, hues + 180, hues) if np.ptp(hues) > 90 else hues
        low_h = (np.percentile(wrapped_hues, 5) - self.p('hsv_margin_h')) % 180
        high_h = (np.percentile(wrapped_hues, 95) + self.p('hsv_margin_h')) % 180
        low_sv = np.percentile(useful[:, 1:3], 5, axis=0) - np.array(
            [self.p('hsv_margin_s'), self.p('hsv_margin_v')])
        high_sv = np.percentile(useful[:, 1:3], 95, axis=0) + np.array(
            [self.p('hsv_margin_s'), self.p('hsv_margin_v')])
        low_sv = np.clip(low_sv, 0, 255).astype(int)
        high_sv = np.clip(high_sv, 0, 255).astype(int)
        low = np.array([int(low_h), *low_sv], dtype=int)
        high = np.array([int(high_h), *high_sv], dtype=int)
        self.hsv_limits = {'low': low.tolist(), 'high': high.tolist()}
        self.save_calibration()
        self.get_logger().info(
            f"HSV learned: low={self.hsv_limits['low']} high={self.hsv_limits['high']}")

    def calibration_path(self):
        return Path(str(self.p('calibration_file'))).expanduser()

    def save_calibration(self):
        path = self.calibration_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            'object_label': str(self.p('object_label')),
            'hsv_low': self.hsv_limits['low'],
            'hsv_high': self.hsv_limits['high'],
        }
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')

    def load_calibration(self):
        path = self.calibration_path()
        if not path.is_file():
            return
        try:
            document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            low, high = document['hsv_low'], document['hsv_high']
            if len(low) == 3 and len(high) == 3:
                self.hsv_limits = {'low': list(map(int, low)), 'high': list(map(int, high))}
                self.get_logger().info(f'Loaded HSV calibration from {path}')
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().warning(f'Cannot load calibration: {error}')

    def position_for_bbox(self, x, y, width, height, color_shape):
        center_u, center_v = x + width // 2, y + height // 2
        result = {'pixel_x': center_u, 'pixel_y': center_v, 'distance_m': None,
                  'x_m': None, 'y_m': None}
        if self.depth_frame is None:
            return result
        depth_h, depth_w = self.depth_frame.shape
        color_h, color_w = color_shape[:2]
        depth_u = int(np.clip(center_u * depth_w / color_w, 0, depth_w - 1))
        depth_v = int(np.clip(center_v * depth_h / color_h, 0, depth_h - 1))
        radius = 4
        patch = self.depth_frame[max(0, depth_v-radius):depth_v+radius+1,
                                 max(0, depth_u-radius):depth_u+radius+1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return result
        distance_mm = float(np.median(valid))
        result['distance_m'] = round(distance_mm / 1000.0, 3)
        if self.fx > 0 and self.fy > 0:
            result['x_m'] = round((depth_u - self.cx) * distance_mm / self.fx / 1000.0, 3)
            result['y_m'] = round((depth_v - self.cy) * distance_mm / self.fy / 1000.0, 3)
        return result

    def display_and_track(self):
        if self.color_frame is None:
            return
        display = self.color_frame.copy()
        mask = np.zeros(display.shape[:2], np.uint8)
        result = {'tracked': False, 'label': str(self.p('object_label'))}
        if self.hsv_limits is None:
            cv2.putText(display, 'Press R, drag Object #1 ROI, press ENTER',
                        (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        else:
            hsv = cv2.cvtColor(display, cv2.COLOR_BGR2HSV)
            low = np.array(self.hsv_limits['low'], np.uint8)
            high = np.array(self.hsv_limits['high'], np.uint8)
            if low[0] <= high[0]:
                mask = cv2.inRange(hsv, low, high)
            else:
                lower_red = cv2.inRange(
                    hsv, np.array([0, low[1], low[2]], np.uint8), high)
                upper_red = cv2.inRange(
                    hsv, low, np.array([179, high[1], high[2]], np.uint8))
                mask = cv2.bitwise_or(lower_red, upper_red)
            size = max(3, int(self.p('morphology_kernel')) | 1)
            kernel = np.ones((size, size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) >= self.p('min_contour_area')]
            if contours:
                contour = max(contours, key=cv2.contourArea)
                x, y, width, height = cv2.boundingRect(contour)
                position = self.position_for_bbox(x, y, width, height, display.shape)
                result.update({'tracked': True, 'bbox': [x, y, width, height], **position,
                               'hsv_low': self.hsv_limits['low'],
                               'hsv_high': self.hsv_limits['high']})
                cv2.rectangle(display, (x, y), (x + width, y + height), (0, 255, 0), 3)
                cv2.circle(display, (position['pixel_x'], position['pixel_y']), 5, (0, 0, 255), -1)
                distance = ('N/A' if position['distance_m'] is None
                            else f"{position['distance_m']:.2f}m")
                cv2.putText(display, f"{self.p('object_label')} | Distance:{distance}",
                            (x, max(25, y - 28)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
                cv2.putText(display,
                            f"Pixel X:{position['pixel_x']} Y:{position['pixel_y']}",
                            (x, max(48, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        cv2.imshow(self.window, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            self.teach_roi()
        elif key in (ord('q'), 27):
            rclpy.shutdown()
        self.publish(result, display, mask)

    def publish(self, result, display, mask):
        message = String(data=json.dumps(result))
        self.result_pub.publish(message)
        quality = int(np.clip(self.p('jpeg_quality'), 20, 95))
        for publisher, image, fmt, params in (
                (self.image_pub, display, 'jpeg', [cv2.IMWRITE_JPEG_QUALITY, quality]),
                (self.mask_pub, mask, 'png', [cv2.IMWRITE_PNG_COMPRESSION, 3])):
            ok, encoded = cv2.imencode('.jpg' if fmt == 'jpeg' else '.png', image, params)
            if ok:
                publisher.publish(CompressedImage(format=fmt, data=encoded.tobytes()))

    def close(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ColorRoiTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
