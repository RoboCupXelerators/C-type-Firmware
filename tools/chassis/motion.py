"""Deterministic acceleration and jerk limiting for reference simulations."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import ChassisTwist


@dataclass(frozen=True)
class AxisLimit:
    max_speed: float
    max_accel: float
    max_jerk: float

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.max_speed, self.max_accel, self.max_jerk)):
            raise ValueError("axis limits must be finite and positive")


@dataclass
class AxisState:
    value: float = 0.0
    acceleration: float = 0.0


def step_axis(target, state, limit, dt):
    if not math.isfinite(target) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("target and dt must be finite; dt must be positive")
    target = max(-limit.max_speed, min(limit.max_speed, target))
    desired = max(-limit.max_accel, min(limit.max_accel, (target - state.value) / dt))
    delta = limit.max_jerk * dt
    acceleration = max(state.acceleration - delta, min(state.acceleration + delta, desired))
    value = state.value + acceleration * dt
    if (target - state.value) * (target - value) <= 0:
        value, acceleration = target, 0.0
    return AxisState(value, acceleration)


class TwistLimiter:
    def __init__(self, vx_limit, vy_limit, wz_limit):
        self.limits = (vx_limit, vy_limit, wz_limit)
        self.states = [AxisState(), AxisState(), AxisState()]

    def update(self, target, dt):
        self.states = [
            step_axis(value, state, limit, dt)
            for value, state, limit in zip(target.values(), self.states, self.limits)
        ]
        return ChassisTwist(*(state.value for state in self.states))

    def reset(self):
        self.states = [AxisState(), AxisState(), AxisState()]


@dataclass(frozen=True)
class VelocityProfile:
    name: str
    normal_accel: float
    normal_decel: float
    reverse_brake: float
    normal_jerk: float
    reverse_jerk: float
    zero_speed_epsilon: float = 0.2
    reverse_detection_epsilon: float = 0.5
    velocity_deadband: float = 0.05

    def __post_init__(self):
        values = (
            self.normal_accel, self.normal_decel, self.reverse_brake,
            self.normal_jerk, self.reverse_jerk, self.zero_speed_epsilon,
            self.reverse_detection_epsilon, self.velocity_deadband,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("velocity profile values must be finite and positive")
        if not self.reverse_brake > self.normal_decel >= self.normal_accel:
            raise ValueError("reverse_brake > normal_decel >= normal_accel is required")
        if self.reverse_jerk <= self.normal_jerk:
            raise ValueError("reverse jerk must exceed normal jerk")


VELOCITY_PROFILES = {
    "safe": VelocityProfile("safe", 8.0, 12.0, 18.0, 40.0, 120.0),
    "normal": VelocityProfile("normal", 16.7, 25.0, 40.0, 66.7, 240.0),
    "competition_candidate": VelocityProfile(
        "competition_candidate", 40.0, 60.0, 100.0, 200.0, 600.0
    ),
}


@dataclass
class VelocityState:
    target: float = 0.0
    acceleration: float = 0.0
    reverse_state: str = "idle"
    limiter_state: str = "normal_accel"


def step_velocity_target(target, measured, state, profile, dt=0.01):
    """Reference model of the C-board motor-side reversal limiter."""
    values = (target, measured, state.target, state.acceleration, dt)
    if not all(math.isfinite(value) for value in values) or dt <= 0:
        raise ValueError("velocity step values must be finite and dt positive")
    if abs(target) < profile.velocity_deadband:
        target = 0.0
    reversing = (
        abs(target) > profile.reverse_detection_epsilon
        and abs(measured) > profile.reverse_detection_epsilon
        and target * measured < 0
    )
    decelerating = not reversing and abs(target) < abs(measured)
    accel_limit = profile.normal_accel
    jerk_limit = profile.normal_jerk
    limiter = "normal_accel"
    reverse_state = state.reverse_state
    if reversing:
        accel_limit = profile.reverse_brake
        jerk_limit = profile.reverse_jerk
        limiter = "reverse_brake"
        reverse_state = "braking"
    elif decelerating:
        accel_limit = profile.normal_decel
        limiter = "normal_decel"
    desired = max(-accel_limit, min(accel_limit, (target - state.target) / dt))
    acceleration = state.acceleration
    if reversing and acceleration * desired < 0:
        acceleration = 0.0
    delta = jerk_limit * dt
    acceleration = max(-accel_limit, min(accel_limit,
        max(acceleration - delta, min(acceleration + delta, desired))))
    next_target = state.target + acceleration * dt
    if (target - state.target) * (target - next_target) <= 0:
        next_target, acceleration = target, 0.0
    return VelocityState(next_target, acceleration, reverse_state, limiter)
