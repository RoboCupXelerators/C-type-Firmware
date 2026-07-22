"""Generate the firmware motor mapping from the unique JSON configuration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def render(config_path: Path) -> str:
    data = json.loads(config_path.read_text())
    wheels = data["wheels"]
    if [wheel["name"] for wheel in wheels] != ["FL", "FR", "RL", "RR"]:
        raise ValueError("wheel order must be FL, FR, RL, RR")
    esc = [wheel["motor_id"] for wheel in wheels]
    master = [wheel["master_id"] for wheel in wheels]
    if any(value is None for value in esc + master):
        raise ValueError("all motor IDs must be assigned")
    if len(set(esc)) != 4 or len(set(master)) != 4:
        raise ValueError("ESC IDs and Master IDs must each be unique")
    values = lambda items: ", ".join(f"0x{int(value):02X}U" for value in items)
    floats = lambda items: ", ".join(f"{float(value):.9f}f" for value in items)
    calibration = data.get("calibration", {})
    wheel_radius_scale = calibration.get("wheel_radius_scale", [1.0] * 4)
    motor_speed_scale = calibration.get("motor_speed_scale", [1.0] * 4)
    drive_angle_offset = calibration.get("drive_angle_offset_deg", [0.0] * 4)
    x_offset = calibration.get("x_offset_m", [0.0] * 4)
    y_offset = calibration.get("y_offset_m", [0.0] * 4)
    motor_sign = calibration.get("motor_sign", [w["motor_sign"] for w in wheels])
    arrays = (wheel_radius_scale, motor_speed_scale, drive_angle_offset, x_offset, y_offset, motor_sign)
    if any(len(items) != 4 for items in arrays):
        raise ValueError("calibration arrays must use FL,FR,RL,RR order")
    signs_verified = all(wheel.get("motor_sign_status") == "VERIFIED" for wheel in wheels)
    return f"""/* Generated from config/chassis_geometry.json; do not hand-edit. */
#ifndef CHASSIS_MOTOR_CONFIG_GENERATED_H
#define CHASSIS_MOTOR_CONFIG_GENERATED_H

#include <stdint.h>

#define CHASSIS_CONFIG_WHEEL_COUNT 4U
static const uint16_t CHASSIS_ESC_IDS[CHASSIS_CONFIG_WHEEL_COUNT] = {{{values(esc)}}};
static const uint16_t CHASSIS_MASTER_IDS[CHASSIS_CONFIG_WHEEL_COUNT] = {{{values(master)}}};
static const float CHASSIS_WHEEL_X_M[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats([float(w['x_m']) + float(x_offset[i]) for i, w in enumerate(wheels)])}}};
static const float CHASSIS_WHEEL_Y_M[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats([float(w['y_m']) + float(y_offset[i]) for i, w in enumerate(wheels)])}}};
static const float CHASSIS_DRIVE_ANGLE_RAD[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats([(float(w['drive_angle_deg']) + float(drive_angle_offset[i])) * 3.141592653589793 / 180.0 for i, w in enumerate(wheels)])}}};
static const float CHASSIS_WHEEL_RADIUS_M[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats([float(w['wheel_radius_m']) * float(wheel_radius_scale[i]) for i, w in enumerate(wheels)])}}};
static const float CHASSIS_GEAR_RATIO[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats([float(w['gear_ratio']) * float(motor_speed_scale[i]) for i, w in enumerate(wheels)])}}};
static const float CHASSIS_MOTOR_SIGN[CHASSIS_CONFIG_WHEEL_COUNT] = {{{floats(motor_sign)}}};
#define CHASSIS_NONZERO_HARDWARE_ALLOWED {1 if signs_verified else 0}

#endif
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config/chassis_geometry.json"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    content = render(args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
