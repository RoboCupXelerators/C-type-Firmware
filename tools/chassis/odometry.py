"""Wheel-feedback reconstruction and planar odometry."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import ChassisKinematics, ChassisTwist


@dataclass(frozen=True)
class FeedbackEstimate:
    twist: ChassisTwist
    residual_rad_s: tuple[float, float, float, float]
    residual_rms_rad_s: float


def estimate_feedback(kinematics: ChassisKinematics, motor_rad_s) -> FeedbackEstimate:
    values = tuple(float(value) for value in motor_rad_s)
    twist = kinematics.forward_kinematics(values)
    reconstructed = kinematics.inverse_kinematics(twist).raw_motor_rad_s
    residual = tuple(actual - predicted for actual, predicted in zip(values, reconstructed))
    return FeedbackEstimate(twist, residual, math.sqrt(sum(v * v for v in residual) / 4.0))


@dataclass
class PlanarOdometry:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def update(self, body_twist, dt_s):
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        cosine, sine = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        self.x_m += (cosine * body_twist.vx_mps - sine * body_twist.vy_mps) * dt_s
        self.y_m += (sine * body_twist.vx_mps + cosine * body_twist.vy_mps) * dt_s
        angle = self.yaw_rad + body_twist.wz_rad_s * dt_s
        self.yaw_rad = math.atan2(math.sin(angle), math.cos(angle))
