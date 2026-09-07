"""Synthetic regression checks; no camera or desktop required."""
import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('vision', Path(__file__).resolve().parents[1] / 'object_feature_detector.py')
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)


class ProfileTests(unittest.TestCase):
    def test_fifteen_profiles_across_entire_image(self):
        hsv = np.zeros((240, 600, 3), np.uint8)
        profiles = []
        for i in range(15):
            x, y = (i % 5)*120 + 20, (i // 5)*80 + 15
            hue = i*12
            hsv[y:y+45, x:x+55] = (hue, 220, 220)
            profiles.append(dict(name=f'Object {i+1}', low=[hue, 180, 180], high=[hue+2, 255, 255], enabled=True))
        vision.validate_profiles(profiles)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        found = vision.detect_profiles(frame, profiles, 500)
        self.assertEqual([name for name, _ in found], [p['name'] for p in profiles])
        profiles[7]['enabled'] = False
        self.assertEqual(len(vision.detect_profiles(frame, profiles, 500)), 14)

    def test_red_wrap_and_stationary_object(self):
        hsv = np.full((40, 40, 3), (179, 220, 220), np.uint8)
        hsv[:, 20:, 0] = 1
        low, high = vision.learn_hsv(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertGreater(low[0], high[0])
        profile = dict(name='Red block', low=low, high=high, enabled=True)
        frame = np.zeros((160, 240, 3), np.uint8)
        frame[90:130, 170:210] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        for _ in range(5):
            result = vision.detect_profiles(frame, [profile], 500)
            self.assertEqual(len(result), 1)
            self.assertEqual(cv2.boundingRect(result[0][1]), (170, 90, 40, 40))

    def test_overlap_priority(self):
        frame = np.zeros((100, 100, 3), np.uint8)
        frame[20:80, 20:80] = (0, 0, 255)
        profiles = [dict(name=n, low=[170, 100, 100], high=[10, 255, 255], enabled=True) for n in ('First', 'Second')]
        self.assertEqual([n for n, _ in vision.detect_profiles(frame, profiles, 500)], ['First'])

    def test_validation_and_label(self):
        profile = dict(name='Block', low=[170, 100, 100], high=[10, 255, 255], enabled=True)
        vision.validate_profiles([profile])
        with self.assertRaises(ValueError):
            vision.validate_profiles([profile, profile])
        with self.assertRaises(ValueError):
            vision.validate_profiles([dict(profile, low=[180, 100, 100])])
        self.assertEqual(vision.format_label('{roi} #{id:02d} X:{x}', roi='Block', id=3, x=55), 'Block #03 X:55')
        with self.assertRaises(ValueError):
            vision.format_label('{invalid}')


if __name__ == '__main__':
    unittest.main()
