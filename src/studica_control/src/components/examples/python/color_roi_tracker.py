#!/usr/bin/env python3
"""Teach a color from an ROI, then track it continuously with HSV thresholding."""

import json
import time
import os
import tempfile
from copy import copy
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
import yaml


def contour_shape(contour):
    """Classify the 2D silhouette; never infer a rectangle from its bounding box."""
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if area <= 0 or perimeter <= 0:
        return 'unknown'
    polygon = cv2.approxPolyDP(contour, .025 * perimeter, True)
    if len(polygon) == 3 and cv2.isContourConvex(polygon):
        return 'triangle'
    if len(polygon) == 4 and cv2.isContourConvex(polygon):
        points = polygon.reshape(-1, 2).astype(float)
        for i in range(4):
            first = points[(i-1) % 4] - points[i]
            second = points[(i+1) % 4] - points[i]
            denominator = np.linalg.norm(first) * np.linalg.norm(second)
            if denominator == 0 or abs(np.dot(first, second)/denominator) > .3:
                return 'unknown'
        return 'rectangle'
    if len(polygon) >= 6 and 4 * np.pi * area / perimeter**2 >= .8:
        return 'circle'
    return 'unknown'


class ColorProfiles:
    """Validated named HSV ranges and selection, persisted as one atomic file."""

    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.items = []

    @staticmethod
    def validate(items):
        if not isinstance(items, list):
            raise ValueError('profiles must be a list')
        names = set()
        for item in items:
            if not isinstance(item, dict):
                raise ValueError('Each profile must be a mapping')
            name = item.get('name')
            if (not isinstance(name, str) or not name.strip() or len(name) > 32
                    or not all(32 <= ord(c) < 127 for c in name)):
                raise ValueError('Use a non-empty name, up to 32 ASCII characters')
            if name.casefold() in names:
                raise ValueError('That name already exists; choose another name')
            names.add(name.casefold())
            if type(item.get('enabled')) is not bool:
                raise ValueError('enabled must be true or false')
            for key in ('low', 'high'):
                values = item.get(key)
                if (not isinstance(values, list) or len(values) != 3
                        or any(type(v) is not int or not 0 <= v <= cap
                               for v, cap in zip(values, (179, 255, 255)))):
                    raise ValueError('HSV must be [H:0..179, S:0..255, V:0..255]')
            if any(item['low'][i] > item['high'][i] for i in (1, 2)):
                raise ValueError('S/V lower bounds must not exceed upper bounds')

    def load(self):
        if not self.path.exists():
            return
        document = yaml.safe_load(self.path.read_text(encoding='utf-8'))
        if not isinstance(document, dict) or document.get('version') != 1:
            raise ValueError('Expected color profile file version 1')
        self.validate(document.get('profiles'))
        self.items = document['profiles']

    def commit(self, items):
        self.validate(items)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=self.path.parent, delete=False) as output:
                temporary = output.name
                yaml.safe_dump({'version': 1, 'profiles': items}, output, sort_keys=False)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.items = items

    def add(self, name, low, high):
        self.commit(self.items + [{'name': name.strip(), 'enabled': True,
                                   'low': list(low), 'high': list(high)}])

    def rename(self, index, name):
        items = [dict(item) for item in self.items]
        items[index]['name'] = name.strip()
        self.commit(items)

    def toggle(self, index):
        items = [dict(item) for item in self.items]
        items[index]['enabled'] = not items[index]['enabled']
        self.commit(items)


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
            'profiles_file': '',  # default: alongside the old single-ROI file
            'jpeg_quality': 75,
            'processing_width': 320,
            'processing_rate_hz': 30.0,
            'depth_processing_rate_hz': 10.0,
            'display_rate_hz': 10.0,
            'multiple_colors': False,
            'red_low': [170, 80, 50],
            'red_high': [10, 255, 255],
            'green_low': [35, 60, 40],
            'green_high': [85, 255, 255],
            'blue_low': [95, 80, 40],
            'blue_high': [130, 255, 255],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.color_frame = None
        self.depth_frame = None
        self.pending_depth = None
        self.last_display = 0.0
        self.color_received = 0.0
        self.depth_received = 0.0
        self.color_stamp = 0.0
        self.depth_stamp = 0.0
        self.color_sequence = 0
        self.processed_sequence = -1
        self.show_mask = False
        self.source_shape = None
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.hsv_limits = None
        self.panel = 'Color profiles - click checkboxes'
        self.profile_page = 0
        self.active_profile = None
        self.editing_name = False
        self.rename_index = None
        self.name_buffer = ''
        self.pending_limits = None
        self.panel_status = 'R: Add ROI | L: Rename selected | M: Mask | Q: Quit'
        self.window = 'HSV ROI Teach and Track'
        self.result_pub = self.create_publisher(String, '/color_tracker/result', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, '/color_tracker/image/compressed', 2)
        self.mask_pub = self.create_publisher(
            CompressedImage, '/color_tracker/mask/compressed', 2)
        latest_frame_qos = copy(qos_profile_sensor_data)
        latest_frame_qos.depth = 1
        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self.color_callback, latest_frame_qos)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, latest_frame_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / max(1.0, float(self.p('processing_rate_hz'))), self.display_and_track)
        self.create_timer(1.0 / max(1.0, float(self.p('depth_processing_rate_hz'))), self.sample_depth)
        cv2.setNumThreads(1)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 960, 540)
        if bool(self.get_parameter('load_saved_calibration').value):
            self.load_calibration()
        profiles_path = self.p('profiles_file') or str(
            self.calibration_path().with_name('color_profiles.yaml'))
        self.profiles = ColorProfiles(profiles_path)
        if not self.p('multiple_colors'):
            try:
                self.profiles.load()
                if not self.profiles.path.exists() and self.hsv_limits:
                    self.profiles.add(str(self.p('object_label')),
                                      self.hsv_limits['low'], self.hsv_limits['high'])
            except (OSError, ValueError, yaml.YAMLError) as error:
                # Do not overwrite an unreadable saved profile bank.
                raise RuntimeError(f'Cannot load {profiles_path}: {error}') from error
            cv2.namedWindow(self.panel, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.panel, self.profile_mouse)
            self.render_panel()
        self.get_logger().info(
            'Multi-color mode: M toggles mask; Q/Esc exits'
            if self.p('multiple_colors') else 'R: Add named ROI; tick profiles in the selection window')

    def render_panel(self):
        panel = np.full((620, 640, 3), 30, np.uint8)
        def label(text, x, y, color=(230, 230, 230), scale=0.5):
            cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)
        label('SAVED COLOR PROFILES', 16, 30, (0, 255, 255), 0.7)
        for left, right, caption in ((16, 185, 'Add ROI (R)'), (200, 420, 'Rename selected (L)')):
            cv2.rectangle(panel, (left, 45), (right, 80), (90, 90, 90), -1)
            label(caption, left + 8, 68)
        label('Tick to detect. Click a name to select it for renaming.', 16, 103)
        for row, item in enumerate(self.profiles.items[self.profile_page * 8:self.profile_page * 8 + 8]):
            index = self.profile_page * 8 + row
            top = 115 + row * 46
            if index == self.active_profile:
                cv2.rectangle(panel, (10, top), (630, top + 44), (65, 60, 40), -1)
            cv2.rectangle(panel, (18, top + 9), (40, top + 31), (220, 220, 220), 1)
            if item['enabled']:
                label('X', 22, top + 27, (0, 255, 0))
            label(item['name'], 52, top + 17)
            label(f"HSV {item['low']} -> {item['high']}", 52, top + 36,
                  (170, 170, 170), 0.4)
        for x1, x2, caption in ((16, 145, '< Previous'), (160, 285, 'Next >')):
            cv2.rectangle(panel, (x1, 490), (x2, 520), (70, 70, 70), -1)
            label(caption, x1 + 8, 511)
        label(f'Page {self.profile_page + 1}', 310, 511)
        if self.editing_name:
            label('Name: ' + self.name_buffer + '_', 16, 548, (0, 255, 255))
            cv2.rectangle(panel, (16, 560), (170, 595), (70, 110, 70), -1)
            label('Save (Enter)', 24, 584)
            label('Esc: cancel | Backspace: edit', 195, 584, scale=0.45)
        else:
            label('Selections saved automatically. M: mask. Q: quit.', 16, 548, scale=0.45)
        label(self.panel_status[:85], 16, 611, (180, 220, 255), 0.38)
        cv2.imshow(self.panel, panel)

    def begin_rename(self):
        if self.active_profile is None:
            self.panel_status = 'Click a profile name first.'
            return
        self.rename_index = self.active_profile
        self.name_buffer = self.profiles.items[self.active_profile]['name']
        self.editing_name = True
        self.panel_status = 'Edit name, then Enter to save; range is preserved.'

    def save_profile_name(self):
        try:
            if self.rename_index is not None:
                self.profiles.rename(self.rename_index, self.name_buffer)
            elif self.pending_limits:
                self.profiles.add(self.name_buffer, self.pending_limits['low'], self.pending_limits['high'])
                self.active_profile = len(self.profiles.items) - 1
                self.profile_page = self.active_profile // 8
            self.editing_name = False
            self.pending_limits = None
            self.processed_sequence = -1
            self.panel_status = 'Saved: ' + self.name_buffer
        except (ValueError, OSError) as error:
            self.panel_status = str(error)

    def profile_mouse(self, event, x, y, flags, userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.editing_name:
            if 16 <= x <= 170 and 560 <= y <= 595:
                self.save_profile_name()
        elif 45 <= y <= 80:
            if 16 <= x <= 185:
                self.teach_roi()
            elif 200 <= x <= 420:
                self.begin_rename()
        elif 115 <= y < 483:
            index = self.profile_page * 8 + (y - 115) // 46
            if index < len(self.profiles.items):
                self.active_profile = index
                if 18 <= x <= 40:
                    try:
                        self.profiles.toggle(index)
                        self.processed_sequence = -1
                        self.panel_status = 'Selection saved.'
                    except OSError as error:
                        self.panel_status = str(error)
        elif 490 <= y <= 520:
            last_page = max(0, (len(self.profiles.items) - 1) // 8)
            if 16 <= x <= 145:
                self.profile_page = max(0, self.profile_page - 1)
            elif 160 <= x <= 285:
                self.profile_page = min(last_page, self.profile_page + 1)
        self.render_panel()

    def profile_key(self, key):
        if self.editing_name:
            if key in (10, 13):
                self.save_profile_name()
            elif key == 27:
                self.editing_name = False
                self.pending_limits = None
                self.panel_status = 'Cancelled; saved profiles unchanged.'
            elif key in (8, 127):
                self.name_buffer = self.name_buffer[:-1]
            elif 32 <= key < 127 and len(self.name_buffer) < 32:
                self.name_buffer += chr(key)
        elif key in (ord('q'), 27):
            rclpy.shutdown()
        elif key == ord('r'):
            self.teach_roi()
        elif key == ord('l'):
            self.begin_rename()
        elif key == ord('m'):
            self.show_mask = not self.show_mask
            if not self.show_mask:
                cv2.destroyWindow('HSV mask')
        if key != 255:
            self.render_panel()

    @staticmethod
    def hsv_mask(hsv, low, high):
        """OpenCV H is circular: low H > high H spans 179/0 (red)."""
        low, high = np.asarray(low, np.uint8), np.asarray(high, np.uint8)
        if low[0] <= high[0]:
            return cv2.inRange(hsv, low, high)
        return cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, low[1], low[2]], np.uint8), high),
            cv2.inRange(hsv, low, np.array([179, high[1], high[2]], np.uint8)))

    def detect_multiple_colors(self, frame):
        """Return every sufficiently large color region, without class inference."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        combined = np.zeros(frame.shape[:2], np.uint8)
        objects = []
        size = max(3, int(self.p('morphology_kernel')) | 1)
        kernel = np.ones((size, size), np.uint8)
        if self.p('multiple_colors'):
            profiles = [{'name': name, 'low': self.p(name + '_low'),
                         'high': self.p(name + '_high')} for name in ('red', 'green', 'blue')]
        else:
            profiles = [item for item in self.profiles.items if item['enabled']]
        for item in profiles:
            name = item['name']
            mask = self.hsv_mask(hsv, item['low'], item['high'])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            combined = cv2.bitwise_or(combined, mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) >= self.minimum_area(frame.shape):
                    objects.append({'label': name, 'color': name,
                                    'shape': contour_shape(contour),
                                    'bbox': list(cv2.boundingRect(contour)),
                                    'hsv_low': item['low'], 'hsv_high': item['high']})
        return objects, combined

    def minimum_area(self, shape):
        # Preserve the source-image area threshold when processing a smaller frame.
        if self.source_shape is None:
            return self.p('min_contour_area')
        return self.p('min_contour_area') * (
            shape[0] * shape[1] / float(self.source_shape[0] * self.source_shape[1]))

    def display_multiple_colors(self):
        display = self.color_frame.copy()
        objects, mask = self.detect_multiple_colors(display)
        colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 100, 0)}
        draw = time.monotonic() - self.last_display >= 1.0 / max(1.0, float(self.p('display_rate_hz')))
        for obj in objects:
            x, y, width, height = obj['bbox']
            obj.update(self.position_for_bbox(x, y, width, height, display.shape))
            if not draw:
                continue
            color = colors.get(obj['color'], (0, 255, 255))
            cv2.rectangle(display, (x, y), (x + width, y + height), color, 2)
            cv2.circle(display, (obj['pixel_x'], obj['pixel_y']), 4, color, -1)
            distance = 'N/A' if obj['distance_m'] is None else f"{obj['distance_m']:.2f}m"
            label = f"{obj['label']} X:{obj['pixel_x']} Y:{obj['pixel_y']} D:{distance}"
            # Black outline keeps text readable on bright backgrounds.
            origin = (max(0, min(x, display.shape[1] - 300)), max(42, y - 8))
            for thickness, ink in ((3, (0, 0, 0)), (1, color)):
                cv2.putText(display, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, ink, thickness)
        mode = 'multiple_colors' if self.p('multiple_colors') else 'saved_profiles'
        self.publish({'mode': mode, 'tracked': bool(objects), 'count': len(objects),
                      'objects': objects}, display, mask)
        if not draw:
            return
        self.last_display = time.monotonic()
        hint = ('Multi-color | M: mask | Q: quit' if self.p('multiple_colors')
                else 'Saved profiles | R: add | M: mask | Q: quit')
        cv2.putText(display, hint,
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.imshow(self.window, display)
        self.last_mask = mask
        if self.show_mask:
            cv2.imshow('HSV mask', mask)

    def p(self, name):
        return self.get_parameter(name).value

    def color_callback(self, message):
        if message.encoding.lower() not in ('bgr8', 'rgb8'):
            return
        row = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        frame = row[:, :message.width * 3].reshape(message.height, message.width, 3)
        self.source_shape = frame.shape[:2]
        target_width = int(self.p('processing_width'))
        if 0 < target_width < frame.shape[1]:
            target_height = max(1, round(frame.shape[0] * target_width / frame.shape[1]))
            frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
        if message.encoding.lower() == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.color_frame = frame.copy()
        self.color_received = time.monotonic()
        self.color_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.color_sequence += 1

    def depth_callback(self, message):
        # Keep only the newest message; conversion/validation runs at 10 Hz.
        self.pending_depth = (message, time.monotonic())

    def sample_depth(self):
        if self.pending_depth is None:
            return
        message, received = self.pending_depth
        self.pending_depth = None
        if message.encoding not in ('16UC1', 'mono16'):
            return
        row_values = message.step // 2
        depth = np.frombuffer(message.data, np.uint16).reshape(
            message.height, row_values)[:, :message.width]
        if bool(message.is_bigendian) != (np.dtype(np.uint16).byteorder == '>'):
            depth = depth.byteswap()
        self.depth_frame = depth.copy()
        self.depth_received = received
        self.depth_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def info_callback(self, message):
        self.fx, self.fy = float(message.k[0]), float(message.k[4])
        self.cx, self.cy = float(message.k[2]), float(message.k[5])

    def teach_roi(self):
        if self.color_frame is None or time.monotonic() - self.color_received > 1.0:
            self.panel_status = 'No fresh camera image; wait for RGB before adding an ROI.'
            return
        frame = self.color_frame.copy()
        roi = cv2.selectROI(
            'Select color ROI - ENTER to accept', frame,
            showCrosshair=True, fromCenter=False)
        cv2.destroyWindow('Select color ROI - ENTER to accept')
        x, y, width, height = (int(value) for value in roi)
        if width <= 0 or height <= 0:
            return
        sample = frame[y:y + height, x:x + width]
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
        self.pending_limits = {'low': low.tolist(), 'high': high.tolist()}
        self.rename_index = None
        self.name_buffer = ''
        self.editing_name = True
        self.panel_status = f'HSV {low.tolist()} -> {high.tolist()}. Type name in the profile window.'
        self.render_panel()
        self.get_logger().info(
            f'HSV learned: low={low.tolist()} high={high.tolist()}; enter a profile name to save')

    def calibration_path(self):
        return Path(str(self.p('calibration_file'))).expanduser()

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
        if (self.depth_frame is None or time.monotonic() - self.depth_received > 0.5
                or abs(self.color_stamp - self.depth_stamp) > 0.25):
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
        if not self.p('multiple_colors'):
            self.profile_key(cv2.waitKey(1) & 0xFF)
            if not rclpy.ok():
                return
            if self.color_frame is None or time.monotonic() - self.color_received > 1.0:
                waiting = np.zeros((180, 320, 3), np.uint8)
                cv2.putText(waiting, 'Waiting for RGB | Q: quit', (8, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cv2.imshow(self.window, waiting)
                if self.show_mask:
                    cv2.imshow('HSV mask', waiting[:, :, 0])
                self.publish({'mode': 'saved_profiles', 'status': 'no_rgb',
                              'tracked': False, 'count': 0, 'objects': []},
                             waiting, waiting[:, :, 0])
            elif self.processed_sequence != self.color_sequence:
                self.processed_sequence = self.color_sequence
                self.display_multiple_colors()
            return
        if bool(self.p('multiple_colors')):
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                rclpy.shutdown()
                return
            if key == ord('m'):
                self.show_mask = not self.show_mask
                if not self.show_mask:
                    cv2.destroyWindow('HSV mask')
            if self.color_frame is None or time.monotonic() - self.color_received > 1.0:
                waiting = np.zeros((360, 640, 3), np.uint8)
                cv2.putText(waiting, 'Waiting for RGB frames | Q: quit', (15, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow(self.window, waiting)
                if self.show_mask:
                    cv2.imshow('HSV mask', waiting[:, :, 0])
                self.publish({'mode': 'multiple_colors', 'status': 'no_rgb',
                              'count': 0, 'objects': []}, waiting, waiting[:, :, 0])
            elif self.processed_sequence != self.color_sequence:
                self.processed_sequence = self.color_sequence
                self.display_multiple_colors()
            return
    def publish(self, result, display, mask):
        result['image_width'] = display.shape[1]
        result['image_height'] = display.shape[0]
        message = String(data=json.dumps(result))
        self.result_pub.publish(message)
        quality = int(np.clip(self.p('jpeg_quality'), 20, 95))
        for publisher, image, fmt, params in (
                (self.image_pub, display, 'jpeg', [cv2.IMWRITE_JPEG_QUALITY, quality]),
                (self.mask_pub, mask, 'png', [cv2.IMWRITE_PNG_COMPRESSION, 3])):
            if publisher.get_subscription_count() == 0:
                continue
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
