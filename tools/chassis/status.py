"""Typed four-motor status model in canonical FL, FR, RL, RR order."""

from __future__ import annotations

from dataclasses import dataclass

from .geometry import WheelIndex


@dataclass(frozen=True)
class WheelMotorStatus:
    wheel: str
    esc_id: int
    master_id: int
    online: bool
    enabled: bool
    fault: bool
    fault_code: int
    position_rad: float
    speed_rad_s: float
    torque_nm: float
    temperature_c: float
    feedback_sequence: int
    feedback_age_ms: float
    rotor_temperature_c: float | None = None
    position_unwrapped_rad: float = 0.0
    rx_count: int = 0
    invalid_count: int = 0
    stale_count: int = 0
    last_raw_can_id: int = 0
    source: str = "hardware"


@dataclass(frozen=True)
class FourMotorStatus:
    motors: tuple[WheelMotorStatus, WheelMotorStatus, WheelMotorStatus, WheelMotorStatus]
    snapshot_sequence: int
    received_monotonic: float
    online_mask: int
    enabled_mask: int
    fault_mask: int
    stale_mask: int
    can_esr: int
    runtime_state: int
    runtime_fault: int
    raw_payload: bytes
    source: str = "hardware"

    @property
    def fl(self): return self.motors[WheelIndex.FL]
    @property
    def fr(self): return self.motors[WheelIndex.FR]
    @property
    def rl(self): return self.motors[WheelIndex.RL]
    @property
    def rr(self): return self.motors[WheelIndex.RR]
    @property
    def all_online(self): return self.online_mask == 0x0F
    @property
    def all_disabled(self): return self.enabled_mask == 0 and all(m.fault_code == 0 for m in self.motors)
    @property
    def all_enabled(self): return self.enabled_mask == 0x0F
    @property
    def any_fault(self): return bool(self.fault_mask or self.runtime_fault)
    @property
    def any_stale(self): return bool(self.stale_mask)


def status_from_protocol(decoded: dict, raw_payload: bytes, received_monotonic: float) -> FourMotorStatus:
    board_tick = decoded["board_tick_ms"]
    motors = []
    stale_mask = 0
    for index, item in enumerate(decoded["wheels"]):
        age = float(item.get("feedback_age_ms", (board_tick - item["feedback_tick_ms"]) & 0xFFFFFFFF))
        if age > 100.0:
            stale_mask |= 1 << index
        motors.append(WheelMotorStatus(
            wheel=item["wheel"], esc_id=item["esc_id"], master_id=item["master_id"],
            online=item["online"] and age <= 100.0, enabled=item["enabled"],
            fault=item["fault"], fault_code=item["error_code"],
            position_rad=item["position_rad"], speed_rad_s=item["speed_rad_s"],
            torque_nm=item["torque_nm"], temperature_c=item["temperature_mos_c"],
            rotor_temperature_c=item["temperature_rotor_c"],
            feedback_sequence=item["feedback_sequence"], feedback_age_ms=age,
            position_unwrapped_rad=item.get("position_unwrapped_rad", item["position_rad"]),
            rx_count=item.get("rx_count", item["feedback_sequence"]),
            invalid_count=item.get("invalid_count", 0),
            stale_count=item.get("stale_count", 0),
            last_raw_can_id=item.get("last_raw_can_id", item["master_id"]),
        ))
    return FourMotorStatus(
        motors=tuple(motors), snapshot_sequence=decoded["control_sequence"],
        received_monotonic=received_monotonic, online_mask=decoded["online_mask"],
        enabled_mask=decoded["enabled_mask"], fault_mask=decoded["fault_mask"],
        stale_mask=stale_mask, can_esr=decoded["can_esr"],
        runtime_state=decoded["state"], runtime_fault=decoded["runtime_fault"],
        raw_payload=raw_payload,
    )
