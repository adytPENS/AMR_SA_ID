#!/usr/bin/env python3
"""Inverse kinematics untuk empat model drivetrain AMR.

Konvensi REP-103:
  vx  positif = maju
  vy  positif = ke kiri
  wz  positif = putar kiri / counter-clockwise

Urutan roda selalu:
  M0 front-right, M1 rear-right, M2 front-left, M3 rear-left.

Hasil `wheel_speeds()` adalah kecepatan linear permukaan roda (m/s), bukan
duty listrik. MotorSpeedPID bertugas mengubah target ini menjadi duty Titan.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Tuple


WheelSpeeds = Tuple[float, float, float, float]


class DriveModel(str, Enum):
    DIFFERENTIAL = 'differential'
    DIFFERENTIAL_ALL_TERRAIN = 'differential_all_terrain'
    MECANUM = 'mecanum'
    X_DRIVE = 'x_drive'


@dataclass(frozen=True)
class DriveGeometry:
    wheel_diameter: float = 0.12
    track_width: float = 0.35
    wheelbase: float = 0.29

    def __post_init__(self) -> None:
        if self.wheel_diameter <= 0.0:
            raise ValueError('wheel_diameter harus > 0')
        if self.track_width <= 0.0:
            raise ValueError('track_width harus > 0')
        if self.wheelbase <= 0.0:
            raise ValueError('wheelbase harus > 0')

    @property
    def wheel_radius(self) -> float:
        return self.wheel_diameter / 2.0

    @property
    def mecanum_lever(self) -> float:
        return (self.wheelbase + self.track_width) / 2.0

    @property
    def x_drive_lever(self) -> float:
        # Projection of the corner rotational velocity onto a 45-degree
        # X-drive wheel traction axis.
        return (self.wheelbase + self.track_width) / (2.0 * math.sqrt(2.0))


class DriveKinematics:
    def __init__(self, model: DriveModel, geometry: DriveGeometry,
                 mecanum_roller_pattern: str = 'x') -> None:
        self.model = DriveModel(model)
        self.geometry = geometry
        pattern = mecanum_roller_pattern.lower()
        if pattern not in ('x', 'o'):
            raise ValueError('mecanum_roller_pattern harus x atau o')
        self.mecanum_pattern = pattern

    @classmethod
    def from_mapping(cls, config: Mapping) -> 'DriveKinematics':
        """Bangun kinematika dari mapping YAML `drive`."""
        geometry = DriveGeometry(
            wheel_diameter=float(config.get('wheel_diameter', 0.12)),
            track_width=float(config.get('track_width', 0.35)),
            wheelbase=float(config.get('wheelbase', 0.29)),
        )
        return cls(
            DriveModel(str(config.get('model', 'differential')).lower()),
            geometry,
            str(config.get('mecanum_roller_pattern', 'x')),
        )

    def wheel_speeds(self, vx: float, vy: float, wz: float) -> WheelSpeeds:
        """Ubah body velocity menjadi linear speed setiap roda (m/s)."""
        vx, vy, wz = float(vx), float(vy), float(wz)
        if self.model in (
                DriveModel.DIFFERENTIAL,
                DriveModel.DIFFERENTIAL_ALL_TERRAIN):
            if abs(vy) > 1e-9:
                raise ValueError(
                    f'{self.model.value} tidak mendukung gerak lateral vy')
            half_track = self.geometry.track_width / 2.0
            right = vx + half_track * wz
            left = vx - half_track * wz
            return right, right, left, left

        if self.model == DriveModel.MECANUM:
            k = self.geometry.mecanum_lever
            lateral = vy if self.mecanum_pattern == 'x' else -vy
            # Standard X roller pattern.
            front_right = vx + lateral + k * wz
            rear_right = vx - lateral + k * wz
            front_left = vx - lateral - k * wz
            rear_left = vx + lateral - k * wz
            return front_right, rear_right, front_left, rear_left

        # X-drive omni wheels: wheel traction axes are 45 degrees to body X.
        # Translation projection uses 1/sqrt(2); rotation uses corner radius.
        root_two = math.sqrt(2.0)
        k = self.geometry.x_drive_lever
        front_right = (vx + vy) / root_two + k * wz
        rear_right = (vx - vy) / root_two + k * wz
        front_left = (vx - vy) / root_two - k * wz
        rear_left = (vx + vy) / root_two - k * wz
        return front_right, rear_right, front_left, rear_left

    @staticmethod
    def limit(speeds: WheelSpeeds, maximum: float) -> WheelSpeeds:
        """Skalakan seragam agar rasio arah tetap dan |speed| <= maximum."""
        if maximum <= 0.0:
            raise ValueError('maximum harus > 0')
        peak = max(abs(value) for value in speeds)
        if peak <= maximum:
            return speeds
        scale = maximum / peak
        return tuple(value * scale for value in speeds)  # type: ignore[return-value]

    def wheel_angular_speeds(self, vx: float, vy: float,
                             wz: float) -> WheelSpeeds:
        """Kecepatan sudut roda dalam rad/s."""
        radius = self.geometry.wheel_radius
        return tuple(
            value / radius for value in self.wheel_speeds(vx, vy, wz)
        )  # type: ignore[return-value]

    def wheel_rpm(self, vx: float, vy: float, wz: float) -> WheelSpeeds:
        """Target kecepatan poros keluaran roda dalam RPM."""
        factor = 60.0 / (math.pi * self.geometry.wheel_diameter)
        return tuple(
            value * factor for value in self.wheel_speeds(vx, vy, wz)
        )  # type: ignore[return-value]
