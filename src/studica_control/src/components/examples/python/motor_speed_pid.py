#!/usr/bin/env python3
"""PID kecepatan satu motor berbasis posisi encoder yang sudah dinormalisasi."""

from dataclasses import dataclass
from typing import Optional


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class MotorSpeedPID:
    """Kontrol speed satu motor, bebas dari ROS dan dapat dipakai ulang.

    `polarity` mengubah duty listrik menjadi arah fisik encoder:
    motor kanan=-1, motor kiri=+1 pada robot ini.
    """

    polarity: float
    speed_at_full_duty: float = 0.75
    kp: float = 0.25
    ki: float = 0.10
    kd: float = 0.0
    duty_limit: float = 0.70
    integral_limit: float = 0.50
    feedback_timeout: float = 0.30
    filter_alpha: float = 0.35

    position: Optional[float] = None
    feedback_time: float = 0.0
    speed: Optional[float] = None
    integral: float = 0.0
    previous_error: float = 0.0
    control_time: float = 0.0

    def update_encoder(self, position: float, now: float) -> None:
        if self.position is not None and self.feedback_time > 0.0:
            dt = now - self.feedback_time
            if 0.01 <= dt <= 0.25:
                raw_speed = (position - self.position) / dt
                self.speed = (
                    raw_speed if self.speed is None else
                    self.filter_alpha * raw_speed +
                    (1.0 - self.filter_alpha) * self.speed)
        self.position = float(position)
        self.feedback_time = now

    def ready(self, now: float) -> bool:
        return (
            self.speed is not None and
            now - self.feedback_time <= self.feedback_timeout)

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.control_time = 0.0

    def calculate(self, electrical_feedforward: float, now: float) -> float:
        if abs(electrical_feedforward) < 1e-6:
            self.reset()
            return 0.0
        if not self.ready(now):
            self.reset()
            raise RuntimeError('feedback encoder stale/belum tersedia')

        physical_feedforward = self.polarity * electrical_feedforward
        target_speed = physical_feedforward * self.speed_at_full_duty
        error = target_speed - float(self.speed)
        dt = now - self.control_time if self.control_time > 0.0 else 0.0
        derivative = 0.0
        candidate_integral = self.integral
        if 0.0 < dt < 0.25:
            candidate_integral = clamp(
                self.integral + error * dt,
                -self.integral_limit, self.integral_limit)
            derivative = (error - self.previous_error) / dt

        physical_output = (
            physical_feedforward + self.kp * error +
            self.ki * candidate_integral + self.kd * derivative)
        # PID menyeimbangkan besar output tanpa membalik arah perintah.
        if physical_feedforward > 0.0:
            physical_output = clamp(physical_output, 0.0, self.duty_limit)
        else:
            physical_output = clamp(physical_output, -self.duty_limit, 0.0)

        if abs(physical_output) < self.duty_limit - 1e-6:
            self.integral = candidate_integral
        self.previous_error = error
        self.control_time = now
        return self.polarity * physical_output
