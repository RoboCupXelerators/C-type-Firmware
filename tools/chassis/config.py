"""Load and validate the single canonical chassis geometry configuration."""

from __future__ import annotations

import json
import math
from pathlib import Path

from .geometry import ChassisGeometry, MotorMapping, WheelGeometry, explicit_geometry
from .kinematics import ChassisKinematics


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "chassis_geometry.json"


def _calibration(data: dict) -> dict[str, tuple[float, ...]]:
    """Validate canonical FL/FR/RL/RR calibration arrays without guessing values."""
    raw = data.get("calibration", {})
    defaults = {
        "wheel_radius_scale": (1.0,) * 4,
        "motor_speed_scale": (1.0,) * 4,
        "drive_angle_offset_deg": (0.0,) * 4,
        "x_offset_m": (0.0,) * 4,
        "y_offset_m": (0.0,) * 4,
        "motor_sign": (1.0,) * 4,
    }
    result = {}
    for name, default in defaults.items():
        values = tuple(float(value) for value in raw.get(name, default))
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"calibration {name} must contain four finite FL,FR,RL,RR values")
        result[name] = values
    if any(value <= 0.0 for value in result["wheel_radius_scale"] + result["motor_speed_scale"]):
        raise ValueError("calibration scales must be positive")
    if any(value not in (-1.0, 1.0) for value in result["motor_sign"]):
        raise ValueError("calibration motor_sign values must be +1 or -1")
    return result


def load_geometry(path: str | Path = DEFAULT_CONFIG_PATH) -> ChassisGeometry:
    """Load FL, FR, RL, RR geometry and reject ambiguous motor mappings."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if tuple(data.get("wheel_order", ())) != ("FL", "FR", "RL", "RR"):
        raise ValueError("wheel_order must be FL, FR, RL, RR")
    calibration = _calibration(data)

    wheels = []
    for index, item in enumerate(data.get("wheels", ())):
        wheels.append(
            WheelGeometry(
                name=item["name"],
                x_m=float(item["x_m"]) + calibration["x_offset_m"][index],
                y_m=float(item["y_m"]) + calibration["y_offset_m"][index],
                position_angle_rad=math.radians(float(item["position_angle_deg"])),
                axis_angle_rad=math.radians(float(item["axis_angle_deg"])),
                drive_direction_angle_rad=math.radians(
                    float(item["drive_angle_deg"]) + calibration["drive_angle_offset_deg"][index]
                ),
                wheel_radius_m=float(item["wheel_radius_m"]) * calibration["wheel_radius_scale"][index],
                mapping=MotorMapping(
                    int(item["motor_id"]),
                    int(item["master_id"]),
                    calibration["motor_sign"][index],
                    float(item["gear_ratio"]) * calibration["motor_speed_scale"][index],
                ),
            )
        )
    geometry = explicit_geometry(
        wheels,
        status=data.get("geometry_status", "TEMPORARY"),
        source=data.get("source", "unspecified"),
    )
    motor_ids = [wheel.mapping.motor_id for wheel in geometry.wheels]
    master_ids = [wheel.mapping.master_id for wheel in geometry.wheels]
    if len(set(motor_ids)) != 4 or len(set(master_ids)) != 4:
        raise ValueError("ESC IDs and Master IDs must each be unique")
    for wheel in geometry.wheels:
        if wheel.mapping.motor_id & 0x0F != wheel.mapping.motor_id:
            raise ValueError(f"{wheel.name} ESC ID must fit the feedback low nibble")
    # Constructing the solver proves rank and conditioning before the config is usable.
    ChassisKinematics(geometry)
    return geometry
