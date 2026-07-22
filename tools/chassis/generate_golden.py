"""Generate deterministic non-orthogonal kinematics fixtures and simulation CSV."""

from __future__ import annotations

import csv
import json
import random
from pathlib import Path

from .geometry import candidate_80_100_geometry
from .kinematics import ChassisKinematics, ChassisTwist


ROOT = Path(__file__).parents[2]
GOLDEN_PATH = ROOT / "tests/data/nonorthogonal_kinematics_golden.json"
SIMULATION_PATH = ROOT / "docs/nonorthogonal_kinematics_simulation.csv"


def vectors() -> list[ChassisTwist]:
    fixed = [
        ChassisTwist(0, 0, 0),
        ChassisTwist(0.1, 0, 0), ChassisTwist(-0.1, 0, 0),
        ChassisTwist(0, 0.1, 0), ChassisTwist(0, -0.1, 0),
        ChassisTwist(0, 0, 1), ChassisTwist(0, 0, -1),
        ChassisTwist(0.1, 0.1, 0), ChassisTwist(0.1, 0, 1),
        ChassisTwist(0, 0.1, 1), ChassisTwist(0.1, -0.1, 1),
    ]
    source = random.Random(2325)
    fixed.extend(
        ChassisTwist(
            source.uniform(-0.3, 0.3),
            source.uniform(-0.3, 0.3),
            source.uniform(-2.0, 2.0),
        )
        for _ in range(1000)
    )
    return fixed


def generate() -> tuple[Path, Path]:
    geometry = candidate_80_100_geometry()
    kinematics = ChassisKinematics(geometry)
    records = []
    for index, twist in enumerate(vectors()):
        targets = kinematics.inverse_kinematics(twist)
        reconstructed = kinematics.forward_kinematics(targets.motor_rad_s)
        residual = tuple(a - b for a, b in zip(reconstructed.values(), twist.values()))
        records.append({
            "index": index,
            "twist": list(twist.values()),
            "wheel_rad_s": list(targets.wheel_rad_s),
            "motor_rad_s": list(targets.motor_rad_s),
            "reconstructed_twist": list(reconstructed.values()),
            "residual": list(residual),
        })
    document = {
        "source": "simulation",
        "seed": 2325,
        "geometry_status": geometry.status,
        "geometry_source": geometry.source,
        "wheel_order": [wheel.name for wheel in geometry.wheels],
        "jacobian": [list(row) for row in kinematics.jacobian()],
        "rank": kinematics.validation.rank,
        "singular_values": list(kinematics.validation.singular_values),
        "condition_number": kinematics.validation.condition_number,
        "vectors": records,
    }
    GOLDEN_PATH.parent.mkdir(parents=True, exist_ok=True)
    GOLDEN_PATH.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    SIMULATION_PATH.parent.mkdir(parents=True, exist_ok=True)
    with SIMULATION_PATH.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "source", "index", "vx_mps", "vy_mps", "wz_rad_s",
            "fl_motor_rad_s", "fr_motor_rad_s", "rl_motor_rad_s", "rr_motor_rad_s",
            "estimated_vx_mps", "estimated_vy_mps", "estimated_wz_rad_s",
            "residual_norm",
        ])
        for item in records:
            residual_norm = sum(value * value for value in item["residual"]) ** 0.5
            writer.writerow([
                "simulation", item["index"], *item["twist"], *item["motor_rad_s"],
                *item["reconstructed_twist"], residual_norm,
            ])
    return GOLDEN_PATH, SIMULATION_PATH


if __name__ == "__main__":
    for path in generate():
        print(path)
