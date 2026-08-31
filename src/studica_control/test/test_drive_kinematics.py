#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path

MODULE_DIR = (
    Path(__file__).resolve().parents[1] /
    'src/components/examples/python')
sys.path.insert(0, str(MODULE_DIR))

from drive_kinematics import DriveGeometry, DriveKinematics, DriveModel


class DriveKinematicsTest(unittest.TestCase):
    def setUp(self):
        self.geometry = DriveGeometry(0.12, 0.35, 0.29)

    def test_differential_forward_and_turn(self):
        drive = DriveKinematics(DriveModel.DIFFERENTIAL, self.geometry)
        self.assertEqual(drive.wheel_speeds(1.0, 0.0, 0.0), (1.0,) * 4)
        speeds = drive.wheel_speeds(0.0, 0.0, 1.0)
        self.assertGreater(speeds[0], 0.0)
        self.assertGreater(speeds[1], 0.0)
        self.assertLess(speeds[2], 0.0)
        self.assertLess(speeds[3], 0.0)
        with self.assertRaises(ValueError):
            drive.wheel_speeds(0.0, 0.1, 0.0)

    def test_all_terrain_matches_differential(self):
        normal = DriveKinematics(DriveModel.DIFFERENTIAL, self.geometry)
        terrain = DriveKinematics(
            DriveModel.DIFFERENTIAL_ALL_TERRAIN, self.geometry)
        self.assertEqual(
            normal.wheel_speeds(0.4, 0.0, -0.3),
            terrain.wheel_speeds(0.4, 0.0, -0.3))

    def test_mecanum_x_strafe_left(self):
        drive = DriveKinematics(DriveModel.MECANUM, self.geometry, 'x')
        self.assertEqual(
            drive.wheel_speeds(0.0, 1.0, 0.0),
            (1.0, -1.0, -1.0, 1.0))

    def test_x_drive_strafe_left(self):
        drive = DriveKinematics(DriveModel.X_DRIVE, self.geometry)
        value = 1.0 / math.sqrt(2.0)
        expected = (value, -value, -value, value)
        for actual, wanted in zip(
                drive.wheel_speeds(0.0, 1.0, 0.0), expected):
            self.assertAlmostEqual(actual, wanted)

    def test_limit_preserves_ratio(self):
        limited = DriveKinematics.limit((2.0, 1.0, -2.0, -1.0), 0.5)
        self.assertEqual(limited, (0.5, 0.25, -0.5, -0.25))


if __name__ == '__main__':
    unittest.main()
