"""Deterministic long-running Mock validation for the four-wheel chassis."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path

from .config import load_geometry
from .kinematics import ChassisKinematics, ChassisTwist
from .motion import AxisLimit, TwistLimiter
from .odometry import PlanarOdometry, estimate_feedback


@dataclass
class SimulationSummary:
    source: str
    duration_s: float
    ticks: int
    seed: int
    max_round_trip_error: float
    max_residual_rms_rad_s: float
    stop_events: int
    disable_events: int
    offline_events: int
    fault_events: int
    can_busy_events: int
    watchdog_events: int
    final_state: str
    final_enabled_mask: int


def run_simulation(duration_s=600.0, dt_s=0.01, seed=2325, csv_path=None):
    """Run faster than real time; no hardware transport is opened."""
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration and dt must be positive")
    rng = random.Random(seed)
    kinematics = ChassisKinematics(load_geometry())
    limiter = TwistLimiter(
        AxisLimit(0.03, 0.10, 1.0),
        AxisLimit(0.03, 0.10, 1.0),
        AxisLimit(0.20, 0.50, 5.0),
    )
    odometry = PlanarOdometry()
    rows = []
    counters = dict(stop=0, disable=0, offline=0, fault=0, can_busy=0, watchdog=0)
    target = ChassisTwist(0.0, 0.0, 0.0)
    enabled = True
    max_round_trip = 0.0
    max_residual = 0.0
    ticks = int(round(duration_s / dt_s))

    for tick in range(ticks):
        now = tick * dt_s
        event = ""
        if tick % 173 == 0:
            target = ChassisTwist(rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), rng.uniform(-0.2, 0.2))
        # Deterministic injections exercise every safety input without touching hardware.
        if tick and tick % 5000 == 0:
            target, event = ChassisTwist(0.0, 0.0, 0.0), "STOP"
            counters["stop"] += 1
        elif tick and tick % 7001 == 0:
            target, enabled, event = ChassisTwist(0.0, 0.0, 0.0), False, "DISABLE"
            counters["disable"] += 1
        elif tick and tick % 7001 == 1:
            enabled = True
        elif tick and tick % 9001 == 0:
            target, event = ChassisTwist(0.0, 0.0, 0.0), "WHEEL_OFFLINE"
            counters["offline"] += 1
        elif tick and tick % 11003 == 0:
            target, event = ChassisTwist(0.0, 0.0, 0.0), "WHEEL_FAULT"
            counters["fault"] += 1
        elif tick and tick % 13001 == 0:
            event = "CAN_BUSY"
            counters["can_busy"] += 1
        elif tick and tick % 17011 == 0:
            target, event = ChassisTwist(0.0, 0.0, 0.0), "CDC_WATCHDOG"
            counters["watchdog"] += 1
        if tick == ticks - 1:
            # Every simulation exits through the same safe state as production shutdown.
            target, enabled, event = ChassisTwist(0.0, 0.0, 0.0), False, "FINAL_DISABLE"

        limited = limiter.update(target if enabled else ChassisTwist(0.0, 0.0, 0.0), dt_s)
        command_wheels = kinematics.inverse_kinematics(limited, max_motor_rad_s=20.0)
        # Mock feedback is independently generated with bounded sensor noise.
        feedback = tuple(value + rng.uniform(-0.002, 0.002) for value in command_wheels.motor_rad_s)
        estimate = estimate_feedback(kinematics, feedback)
        odometry.update(estimate.twist, dt_s)
        error = max(abs(a - b) for a, b in zip(limited.values(), estimate.twist.values()))
        max_round_trip = max(max_round_trip, error)
        max_residual = max(max_residual, estimate.residual_rms_rad_s)
        if csv_path and (event or tick % 100 == 0 or tick == ticks - 1):
            rows.append({
                "source": "mock",
                "time_s": f"{now:.3f}",
                "event": event,
                "enabled_mask": 15 if enabled else 0,
                "command_vx_mps": target.vx_mps,
                "command_vy_mps": target.vy_mps,
                "command_wz_rad_s": target.wz_rad_s,
                "estimated_vx_mps": estimate.twist.vx_mps,
                "estimated_vy_mps": estimate.twist.vy_mps,
                "estimated_wz_rad_s": estimate.twist.wz_rad_s,
                "residual_rms_rad_s": estimate.residual_rms_rad_s,
                "odom_x_m": odometry.x_m,
                "odom_y_m": odometry.y_m,
                "odom_yaw_rad": odometry.yaw_rad,
            })

    if csv_path:
        path = Path(csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
    return SimulationSummary(
        source="mock",
        duration_s=duration_s,
        ticks=ticks,
        seed=seed,
        max_round_trip_error=max_round_trip,
        max_residual_rms_rad_s=max_residual,
        stop_events=counters["stop"],
        disable_events=counters["disable"],
        offline_events=counters["offline"],
        fault_events=counters["fault"],
        can_busy_events=counters["can_busy"],
        watchdog_events=counters["watchdog"],
        final_state="DISABLED",
        final_enabled_mask=0,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--seed", type=int, default=2325)
    parser.add_argument("--csv", default="docs/nonorthogonal_kinematics_mock_10min.csv")
    parser.add_argument("--json", default="docs/nonorthogonal_kinematics_mock_10min.json")
    args = parser.parse_args()
    summary = run_simulation(args.duration, args.dt, args.seed, args.csv)
    output = json.dumps(asdict(summary), indent=2, sort_keys=True)
    Path(args.json).write_text(output + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
