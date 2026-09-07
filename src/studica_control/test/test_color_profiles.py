"""Regression tests for saved ROI ranges and the checkbox detection workflow."""

import importlib.util
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / 'src/components/examples/python/color_roi_tracker.py'
spec = importlib.util.spec_from_file_location('color_roi_tracker', SOURCE)
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)


class Harness:
    hsv_mask = staticmethod(tracker.ColorRoiTracker.hsv_mask)
    minimum_area = tracker.ColorRoiTracker.minimum_area
    detect_multiple_colors = tracker.ColorRoiTracker.detect_multiple_colors
    profile_mouse = tracker.ColorRoiTracker.profile_mouse
    profile_key = tracker.ColorRoiTracker.profile_key
    save_profile_name = tracker.ColorRoiTracker.save_profile_name
    teach_roi = tracker.ColorRoiTracker.teach_roi
    begin_rename = tracker.ColorRoiTracker.begin_rename
    p = lambda self, key: self.params[key]
    render_panel = lambda self: None
    get_logger = lambda self: SimpleNamespace(info=lambda message: None)

    def __init__(self, store):
        self.profiles = store
        self.params = {'multiple_colors': False, 'morphology_kernel': 3,
                       'min_contour_area': 50, 'hsv_margin_h': 8,
                       'hsv_margin_s': 25, 'hsv_margin_v': 25}
        self.source_shape = None
        self.profile_page = 0
        self.active_profile = None
        self.editing_name = False
        self.pending_limits = None
        self.processed_sequence = 2


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'profiles.yaml'
        self.store = tracker.ColorProfiles(self.path)
        self.store.add('kuning', [9, 172, 60], [31, 255, 235])

    def test_roundtrip_rename_and_selection_preserve_hsv(self):
        self.store.rename(0, 'Object 1 - kuning')
        self.store.toggle(0)
        loaded = tracker.ColorProfiles(self.path)
        loaded.load()
        self.assertEqual(loaded.items, [{'name': 'Object 1 - kuning', 'enabled': False,
                                        'low': [9, 172, 60], 'high': [31, 255, 235]}])

    def test_invalid_or_duplicate_names_do_not_replace_good_file(self):
        before = self.path.read_bytes()
        for name in ('', '  ', 'KUNING'):
            with self.assertRaises(ValueError):
                self.store.add(name, [0, 0, 0], [179, 255, 255])
        with self.assertRaises(ValueError):
            self.store.add('bad', [180, 0, 0], [179, 255, 255])
        self.assertEqual(self.path.read_bytes(), before)

    def test_atomic_failure_preserves_selection_on_disk_and_in_memory(self):
        before = self.path.read_bytes()
        with patch.object(tracker.os, 'replace', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                self.store.toggle(0)
        self.assertTrue(self.store.items[0]['enabled'])
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(len(list(self.path.parent.iterdir())), 1)

    def test_checkbox_controls_detection_and_persists(self):
        self.store.add('merah', [170, 80, 50], [10, 255, 255])
        harness = Harness(self.store)
        frame = np.zeros((120, 220, 3), np.uint8)
        frame[20:90, 20:80] = (0, 220, 220)
        frame[20:90, 130:190] = (0, 0, 255)
        objects, _ = harness.detect_multiple_colors(frame)
        self.assertEqual({o['label'] for o in objects}, {'kuning', 'merah'})
        harness.profile_mouse(cv2.EVENT_LBUTTONDOWN, 25, 130, None, None)
        objects, _ = harness.detect_multiple_colors(frame)
        self.assertEqual([o['label'] for o in objects], ['merah'])
        reloaded = tracker.ColorProfiles(self.path)
        reloaded.load()
        self.assertFalse(reloaded.items[0]['enabled'])
        harness.profile_mouse(cv2.EVENT_LBUTTONDOWN, 25, 176, None, None)
        objects, mask = harness.detect_multiple_colors(frame)
        self.assertEqual(objects, [])
        self.assertFalse(mask.any())

    def test_teach_then_name_then_save_keeps_previous_profile(self):
        harness = Harness(self.store)
        harness.color_frame = np.full((100, 100, 3), (0, 0, 220), np.uint8)
        harness.color_received = time.monotonic()
        with patch.object(cv2, 'selectROI', return_value=(10, 10, 50, 50)), \
                patch.object(cv2, 'destroyWindow'):
            harness.teach_roi()
        self.assertTrue(harness.editing_name)
        self.assertEqual(len(self.store.items), 1)  # ROI alone must not overwrite.
        for key in 'merah':
            harness.profile_key(ord(key))
        harness.profile_key(13)
        self.assertFalse(harness.editing_name)
        self.assertEqual([p['name'] for p in self.store.items], ['kuning', 'merah'])
        self.assertEqual(self.store.items[0]['low'], [9, 172, 60])
        self.assertGreater(self.store.items[1]['low'][0], self.store.items[1]['high'][0])


if __name__ == '__main__':
    unittest.main()


class DepthSamplingTest(unittest.TestCase):
    def message(self, value):
        frame = np.full((12, 12), value, dtype=np.uint16)
        return SimpleNamespace(data=frame.tobytes(), encoding='16UC1',
                               height=12, width=12, step=24, is_bigendian=False,
                               header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))

    def test_depth_callback_only_queues_latest_and_sampling_preserves_age(self):
        node = SimpleNamespace(pending_depth=None, depth_frame=None)
        with patch.object(tracker.time, 'monotonic', return_value=20.0):
            tracker.ColorRoiTracker.depth_callback(node, self.message(400))
            tracker.ColorRoiTracker.depth_callback(node, self.message(600))
        self.assertIsNone(node.depth_frame)
        with patch.object(tracker.time, 'monotonic', return_value=20.2):
            tracker.ColorRoiTracker.sample_depth(node)
        self.assertTrue(np.all(node.depth_frame == 600))
        self.assertEqual(node.depth_received, 20.0)
        self.assertIsNone(node.pending_depth)
        tracker.ColorRoiTracker.sample_depth(node)
        self.assertEqual(node.depth_received, 20.0)

    def test_depth_sample_usable_between_ticks_but_stale_depth_rejected(self):
        node = SimpleNamespace(depth_frame=np.full((12, 12), 600, dtype=np.uint16),
                               depth_received=20.0, color_stamp=10.2, depth_stamp=10.0,
                               fx=0, fy=0)
        with patch.object(tracker.time, 'monotonic', return_value=20.2):
            result = tracker.ColorRoiTracker.position_for_bbox(node, 2, 2, 8, 8, (12, 12))
        self.assertEqual(result['distance_m'], 0.6)
        with patch.object(tracker.time, 'monotonic', return_value=20.6):
            result = tracker.ColorRoiTracker.position_for_bbox(node, 2, 2, 8, 8, (12, 12))
        self.assertIsNone(result['distance_m'])
