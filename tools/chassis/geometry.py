"""Geometry and motor mapping for the FL, FR, RL, RR wheel order."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Sequence


class WheelIndex(IntEnum):
    FL = 0
    FR = 1
    RL = 2
    RR = 3


WHEEL_NAMES = tuple(w.name for w in WheelIndex)


class GeometryStatus:
    TEMPORARY = "TEMPORARY"
    VERIFIED = "VERIFIED"


@dataclass(frozen=True)
class MotorMapping:
    motor_id: Optional[int]
    master_id: Optional[int]
    motor_sign: float = 1.0
    gear_ratio: float = 1.0

    @property
    def assigned(self) -> bool:
        return self.motor_id is not None and self.master_id is not None


@dataclass(frozen=True)
class WheelGeometry:
    name: str
    x_m: float
    y_m: float
    position_angle_rad: float
    axis_angle_rad: float
    drive_direction_angle_rad: float
    wheel_radius_m: float
    mapping: MotorMapping


@dataclass(frozen=True)
class ChassisGeometry:
    wheels: tuple[WheelGeometry, WheelGeometry, WheelGeometry, WheelGeometry]
    status: str
    source: str

    def __post_init__(self) -> None:
        if len(self.wheels) != 4:
            raise ValueError("exactly four wheels are required")
        names = tuple(w.name for w in self.wheels)
        if names != WHEEL_NAMES:
            raise ValueError(f"wheel order must be {WHEEL_NAMES}, got {names}")


def _finite_positive(value: float, field: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return value


def included_angle_positions(
    front_included_angle_deg: float,
    rear_included_angle_deg: float,
) -> tuple[float, float, float, float]:
    """Return candidate wheel-center angles in canonical wheel order.

    The included-angle interpretation is explicitly provisional: front centers
    are symmetric around +X and rear centers around -X.
    """
    front = _finite_positive(front_included_angle_deg, "front included angle")
    rear = _finite_positive(rear_included_angle_deg, "rear included angle")
    if front >= 180.0 or rear >= 180.0:
        raise ValueError("included angles must be less than 180 degrees")
    return (
        math.radians(front / 2.0),
        math.radians(-front / 2.0),
        math.radians(180.0 - rear / 2.0),
        math.radians(-180.0 + rear / 2.0),
    )


def candidate_80_100_geometry(
    wheel_radius_m: float = 0.030,
    wheel_center_radius_m: float = 0.0605,
) -> ChassisGeometry:
    """Build the temporary 80/100 candidate, never a verified production map.

    The candidate assumes radial wheel axes and therefore tangential drive
    directions. Motor IDs/signs for FR/RL/RR remain deliberately unassigned.
    """
    wheel_radius_m = _finite_positive(wheel_radius_m, "wheel radius")
    wheel_center_radius_m = _finite_positive(
        wheel_center_radius_m, "wheel center radius"
    )
    position_angles = included_angle_positions(80.0, 100.0)
    mappings = (
        MotorMapping(1, 0, 1.0, 1.0),
        MotorMapping(None, None, 1.0, 1.0),
        MotorMapping(None, None, 1.0, 1.0),
        MotorMapping(None, None, 1.0, 1.0),
    )
    wheels = []
    for name, angle, mapping in zip(WHEEL_NAMES, position_angles, mappings):
        wheels.append(
            WheelGeometry(
                name=name,
                x_m=wheel_center_radius_m * math.cos(angle),
                y_m=wheel_center_radius_m * math.sin(angle),
                position_angle_rad=angle,
                axis_angle_rad=angle,
                drive_direction_angle_rad=angle + math.pi / 2.0,
                wheel_radius_m=wheel_radius_m,
                mapping=mapping,
            )
        )
    return ChassisGeometry(
        wheels=tuple(wheels),
        status=GeometryStatus.TEMPORARY,
        source=(
            "INCLUDED_ANGLE_MODE candidate: front=80deg rear=100deg; "
            "radial-axis/tangential-drive assumption UNCONFIRMED"
        ),
    )


def explicit_geometry(
    wheels: Sequence[WheelGeometry],
    *,
    status: str = GeometryStatus.TEMPORARY,
    source: str = "EXPLICIT_WHEEL_MODE",
) -> ChassisGeometry:
    return ChassisGeometry(tuple(wheels), status=status, source=source)
