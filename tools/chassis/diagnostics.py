"""Conservative advisory diagnostics based only on real feedback fields."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class DiagnosticCode(str, Enum):
    NORMAL = "NORMAL"
    WHEEL_STALL_SUSPECTED = "WHEEL_STALL_SUSPECTED"
    WHEEL_CONTACT_LOSS_SUSPECTED = "WHEEL_CONTACT_LOSS_SUSPECTED"
    WHEEL_DRAG_SUSPECTED = "WHEEL_DRAG_SUSPECTED"
    SLIP_SUSPECTED = "SLIP_SUSPECTED"
    MAPPING_ERROR_SUSPECTED = "MAPPING_ERROR_SUSPECTED"
    CHASSIS_IMPACT_OR_BLOCKAGE = "CHASSIS_IMPACT_OR_BLOCKAGE"
    TEMPERATURE_WARNING = "TEMPERATURE_WARNING"
    TEMPERATURE_FAULT = "TEMPERATURE_FAULT"
    # Compatibility aliases for early diagnostic consumers.
    RESIDUAL_ELEVATED = "SLIP_SUSPECTED"
    OVERTEMPERATURE_WARNING = "TEMPERATURE_WARNING"


@dataclass(frozen=True)
class DiagnosticThresholds:
    moving_target_rad_s: float = 1.0
    low_speed_rad_s: float = 0.15
    high_torque_nm: float = 0.8
    low_torque_nm: float = 0.05
    residual_rms_rad_s: float = 0.5
    drag_speed_ratio: float = 0.6
    mapping_min_speed_rad_s: float = 0.25
    blockage_wheel_count: int = 3
    temperature_warning_c: float = 70.0
    temperature_fault_c: float = 85.0
    status: str = "TEMPORARY"


def diagnose_wheel(target, speed, torque, temperature, residual, thresholds=DiagnosticThresholds()):
    codes = []
    if abs(target) >= thresholds.moving_target_rad_s and abs(speed) <= thresholds.low_speed_rad_s and abs(torque) >= thresholds.high_torque_nm:
        codes.append(DiagnosticCode.WHEEL_STALL_SUSPECTED)
    if abs(target) >= thresholds.moving_target_rad_s and abs(speed) >= 0.8 * abs(target) and abs(torque) <= thresholds.low_torque_nm and abs(residual) >= thresholds.residual_rms_rad_s:
        codes.append(DiagnosticCode.WHEEL_CONTACT_LOSS_SUSPECTED)
    if (abs(target) >= thresholds.moving_target_rad_s
            and thresholds.low_speed_rad_s < abs(speed) < thresholds.drag_speed_ratio * abs(target)
            and abs(torque) >= thresholds.high_torque_nm):
        codes.append(DiagnosticCode.WHEEL_DRAG_SUSPECTED)
    if (abs(target) >= thresholds.moving_target_rad_s
            and abs(speed) >= thresholds.mapping_min_speed_rad_s
            and target * speed < 0.0):
        codes.append(DiagnosticCode.MAPPING_ERROR_SUSPECTED)
    if abs(residual) >= thresholds.residual_rms_rad_s:
        codes.append(DiagnosticCode.SLIP_SUSPECTED)
    if temperature >= thresholds.temperature_fault_c:
        codes.append(DiagnosticCode.TEMPERATURE_FAULT)
    elif temperature >= thresholds.temperature_warning_c:
        codes.append(DiagnosticCode.TEMPERATURE_WARNING)
    return tuple(codes) or (DiagnosticCode.NORMAL,)


def diagnose_chassis(
    targets: Sequence[float],
    speeds: Sequence[float],
    torques: Sequence[float],
    temperatures: Sequence[float],
    residuals: Sequence[float],
    thresholds: DiagnosticThresholds = DiagnosticThresholds(),
):
    """Return per-wheel diagnostics plus conservative whole-chassis findings."""
    vectors = tuple(tuple(float(value) for value in values) for values in (
        targets, speeds, torques, temperatures, residuals
    ))
    if any(len(values) != 4 for values in vectors):
        raise ValueError("diagnostics require four FL,FR,RL,RR values")
    per_wheel = tuple(
        diagnose_wheel(*items, thresholds)
        for items in zip(*vectors)
    )
    constrained = sum(
        DiagnosticCode.WHEEL_STALL_SUSPECTED in codes
        or DiagnosticCode.WHEEL_DRAG_SUSPECTED in codes
        for codes in per_wheel
    )
    chassis = []
    if constrained >= thresholds.blockage_wheel_count:
        chassis.append(DiagnosticCode.CHASSIS_IMPACT_OR_BLOCKAGE)
    if any(DiagnosticCode.MAPPING_ERROR_SUSPECTED in codes for codes in per_wheel):
        chassis.append(DiagnosticCode.MAPPING_ERROR_SUSPECTED)
    if any(DiagnosticCode.TEMPERATURE_FAULT in codes for codes in per_wheel):
        chassis.append(DiagnosticCode.TEMPERATURE_FAULT)
    return per_wheel, tuple(chassis) or (DiagnosticCode.NORMAL,)
