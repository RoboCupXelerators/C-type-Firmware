"""Replay four-wheel telemetry without substituting command values."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

from .kinematics import ChassisKinematics, ChassisTwist
from .odometry import FeedbackEstimate, estimate_feedback


WHEEL_COLUMNS = ("fl_speed_rad_s", "fr_speed_rad_s", "rl_speed_rad_s", "rr_speed_rad_s")
VALID_SOURCES = {"hardware", "simulation", "mock"}


@dataclass(frozen=True)
class FeedbackReplaySample:
    source: str
    elapsed_s: float
    phase: str
    motor_rad_s: tuple[float, float, float, float]
    estimate: FeedbackEstimate
    command: ChassisTwist | None


def replay_four_motor_csv(
    path: str | Path,
    kinematics: ChassisKinematics,
    *,
    phases: set[str] | None = None,
) -> tuple[FeedbackReplaySample, ...]:
    """Reconstruct chassis motion strictly from the four feedback columns."""
    samples = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            phase = row.get("phase", "")
            if phases is not None and phase not in phases:
                continue
            source = row.get("source", "")
            if source not in VALID_SOURCES:
                raise ValueError(f"unlabelled replay source: {source!r}")
            motors = tuple(float(row[column]) for column in WHEEL_COLUMNS)
            if not all(math.isfinite(value) for value in motors):
                raise ValueError("feedback replay contains a nonfinite wheel speed")
            command = None
            command_fields = ("command_vx", "command_vy", "command_wz")
            if all(row.get(field, "") != "" for field in command_fields):
                values = tuple(float(row[field]) for field in command_fields)
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("feedback replay contains a nonfinite command")
                command = ChassisTwist(*values)
            samples.append(
                FeedbackReplaySample(
                    source=source,
                    elapsed_s=float(row["elapsed_s"]),
                    phase=phase,
                    motor_rad_s=motors,
                    estimate=estimate_feedback(kinematics, motors),
                    command=command,
                )
            )
    if not samples:
        raise ValueError("feedback replay selected no samples")
    return tuple(samples)
