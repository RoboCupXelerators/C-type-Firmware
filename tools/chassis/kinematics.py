"""Dependency-free reference kinematics for an arbitrary four-wheel chassis."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from .geometry import ChassisGeometry


@dataclass(frozen=True)
class ChassisTwist:
    vx_mps: float
    vy_mps: float
    wz_rad_s: float

    def values(self) -> tuple[float, float, float]:
        return (self.vx_mps, self.vy_mps, self.wz_rad_s)


@dataclass(frozen=True)
class WheelTargets:
    raw_wheel_rad_s: tuple[float, float, float, float]
    raw_motor_rad_s: tuple[float, float, float, float]
    wheel_rad_s: tuple[float, float, float, float]
    motor_rad_s: tuple[float, float, float, float]
    scale: float


@dataclass(frozen=True)
class GeometryValidation:
    rank: int
    singular_values: tuple[float, float, float]
    condition_number: float


def _finite(values: Iterable[float], field: str) -> tuple[float, ...]:
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{field} must contain only finite values")
    return result


def limit_wheel_speeds(
    motor_rad_s: Sequence[float],
    max_motor_rad_s: float | Sequence[float],
) -> tuple[tuple[float, float, float, float], float]:
    """Apply one common scale so no wheel exceeds its positive limit."""
    values = _finite(motor_rad_s, "motor speeds")
    if len(values) != 4:
        raise ValueError("exactly four motor speeds are required")
    limits = (
        (float(max_motor_rad_s),) * 4
        if isinstance(max_motor_rad_s, (int, float))
        else _finite(max_motor_rad_s, "motor speed limits")
    )
    if len(limits) != 4 or any(not math.isfinite(limit) or limit <= 0.0 for limit in limits):
        raise ValueError("four positive motor speed limits are required")
    ratios = [limit / abs(value) for value, limit in zip(values, limits) if value]
    scale = min([1.0, *ratios])
    return tuple(value * scale for value in values), scale


def _transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [list(col) for col in zip(*matrix)]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    bt = _transpose(b)
    return [[sum(x * y for x, y in zip(row, col)) for col in bt] for row in a]


def _matvec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> tuple[float, ...]:
    return tuple(sum(x * y for x, y in zip(row, vector)) for row in matrix)


def _inverse3(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    cof00, cof01, cof02 = e * i - f * h, c * h - b * i, b * f - c * e
    cof10, cof11, cof12 = f * g - d * i, a * i - c * g, c * d - a * f
    cof20, cof21, cof22 = d * h - e * g, b * g - a * h, a * e - b * d
    det = a * cof00 + b * cof10 + c * cof20
    scale = max(abs(v) for row in matrix for v in row)
    if scale == 0.0 or abs(det) <= (scale ** 3) * 1e-12:
        raise ValueError("geometry matrix is singular")
    inv_det = 1.0 / det
    return [
        [cof00 * inv_det, cof01 * inv_det, cof02 * inv_det],
        [cof10 * inv_det, cof11 * inv_det, cof12 * inv_det],
        [cof20 * inv_det, cof21 * inv_det, cof22 * inv_det],
    ]


def _symmetric_eigenvalues3(matrix: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    """Jacobi eigenvalues for a real symmetric 3x3 matrix."""
    a = [list(row) for row in matrix]
    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) <= 1e-14 * max(1.0, *(abs(a[k][k]) for k in range(3))):
            break
        tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau)) if tau else 1.0
        cosine = 1.0 / math.sqrt(1.0 + t * t)
        sine = t * cosine
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = app - t * apq
        a[q][q] = aqq + t * apq
        a[p][q] = a[q][p] = 0.0
        for r in range(3):
            if r in (p, q):
                continue
            arp, arq = a[r][p], a[r][q]
            a[r][p] = a[p][r] = cosine * arp - sine * arq
            a[r][q] = a[q][r] = sine * arp + cosine * arq
    return tuple(sorted((a[0][0], a[1][1], a[2][2]), reverse=True))


class ChassisKinematics:
    def __init__(self, geometry: ChassisGeometry, max_condition_number: float = 1e4):
        self.geometry = geometry
        self._jacobian = tuple(self._row(w) for w in geometry.wheels)
        self._motor_matrix = tuple(
            tuple(
                value * w.mapping.motor_sign * w.mapping.gear_ratio / w.wheel_radius_m
                for value in row
            )
            for w, row in zip(geometry.wheels, self._jacobian)
        )
        at = _transpose(self._motor_matrix)
        ata = _matmul(at, self._motor_matrix)
        eigenvalues = _symmetric_eigenvalues3(ata)
        singular = tuple(math.sqrt(max(0.0, value)) for value in eigenvalues)
        tolerance = max(singular) * 1e-6 if singular else 0.0
        rank = sum(value > tolerance for value in singular)
        condition = singular[0] / singular[-1] if singular[-1] > 0.0 else math.inf
        self.validation = GeometryValidation(rank, singular, condition)
        if rank != 3:
            raise ValueError("wheel geometry Jacobian must have rank 3")
        if not math.isfinite(condition) or condition > max_condition_number:
            raise ValueError(f"wheel geometry is ill-conditioned: {condition:.6g}")
        self._pseudoinverse = tuple(tuple(v for v in row) for row in _matmul(_inverse3(ata), at))

    @staticmethod
    def _row(wheel) -> tuple[float, float, float]:
        if not math.isfinite(wheel.wheel_radius_m) or wheel.wheel_radius_m <= 0.0:
            raise ValueError("wheel radius must be finite and positive")
        if not math.isfinite(wheel.mapping.gear_ratio) or wheel.mapping.gear_ratio <= 0.0:
            raise ValueError("gear ratio must be finite and positive")
        if wheel.mapping.motor_sign not in (-1.0, 1.0):
            raise ValueError("motor sign must be +1 or -1")
        ux = math.cos(wheel.drive_direction_angle_rad)
        uy = math.sin(wheel.drive_direction_angle_rad)
        return (ux, uy, -wheel.y_m * ux + wheel.x_m * uy)

    def jacobian(self) -> tuple[tuple[float, float, float], ...]:
        return self._jacobian

    def motor_matrix(self) -> tuple[tuple[float, float, float], ...]:
        return self._motor_matrix

    def pseudoinverse(self) -> tuple[tuple[float, float, float, float], ...]:
        return self._pseudoinverse

    def inverse_kinematics(
        self,
        twist: ChassisTwist,
        max_motor_rad_s: float | Sequence[float] | None = None,
    ) -> WheelTargets:
        vector = _finite(twist.values(), "chassis twist")
        raw_wheel = _matvec(self._jacobian, vector)
        raw_wheel = tuple(v / w.wheel_radius_m for v, w in zip(raw_wheel, self.geometry.wheels))
        raw_motor = tuple(
            v * w.mapping.motor_sign * w.mapping.gear_ratio
            for v, w in zip(raw_wheel, self.geometry.wheels)
        )
        scale = 1.0
        if max_motor_rad_s is not None:
            _, scale = limit_wheel_speeds(raw_motor, max_motor_rad_s)
        return WheelTargets(
            raw_wheel_rad_s=tuple(raw_wheel),
            raw_motor_rad_s=tuple(raw_motor),
            wheel_rad_s=tuple(value * scale for value in raw_wheel),
            motor_rad_s=tuple(value * scale for value in raw_motor),
            scale=scale,
        )

    def forward_kinematics(self, motor_rad_s: Sequence[float]) -> ChassisTwist:
        motor = _finite(motor_rad_s, "motor speeds")
        if len(motor) != 4:
            raise ValueError("exactly four motor speeds are required")
        vx, vy, wz = _matvec(self._pseudoinverse, motor)
        return ChassisTwist(vx, vy, wz)

    @staticmethod
    def limit_twist(
        twist: ChassisTwist,
        *,
        max_vx_mps: float,
        max_vy_mps: float,
        max_wz_rad_s: float,
    ) -> ChassisTwist:
        values = _finite(twist.values(), "chassis twist")
        limits = _finite((max_vx_mps, max_vy_mps, max_wz_rad_s), "twist limits")
        if any(limit <= 0.0 for limit in limits):
            raise ValueError("twist limits must be positive")
        return ChassisTwist(*(max(-limit, min(limit, value)) for value, limit in zip(values, limits)))


def unwrap_position(previous_unwrapped: float | None, wrapped: float, period: float = 2.0 * math.pi) -> float:
    """Extend a wrapped position using the shortest physically continuous delta."""
    if not math.isfinite(wrapped) or not math.isfinite(period) or period <= 0.0:
        raise ValueError("wrapped position and period must be finite; period must be positive")
    if previous_unwrapped is None:
        return wrapped
    if not math.isfinite(previous_unwrapped):
        raise ValueError("previous position must be finite")
    previous_wrapped = math.remainder(previous_unwrapped, period)
    delta = math.remainder(wrapped - previous_wrapped, period)
    return previous_unwrapped + delta
