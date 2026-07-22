"""Offline DM2325 reversal benchmark using the firmware reference limiter."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .config import load_geometry
from .kinematics import ChassisKinematics, ChassisTwist
from .motion import VELOCITY_PROFILES, VelocityState, step_velocity_target


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV = ROOT / "docs/dm2325_reverse_step_software.csv"
DEFAULT_JSON = ROOT / "docs/dm2325_reverse_step_software.json"
STEPS = (
    ("zero_to_pos25", 0.25),
    ("pos25_to_neg25", -0.25),
    ("neg25_to_pos25", 0.25),
    ("pos50", 0.50),
    ("pos50_to_neg50", -0.50),
    ("pos100", 1.00),
    ("pos100_to_neg100", -1.00),
    ("pos100_to_zero", 0.0),
    ("zero_to_neg100", -1.00),
)


def run(profile_name="normal", dt=0.01, phase_s=4.0):
    profile = VELOCITY_PROFILES[profile_name]
    solver = ChassisKinematics(load_geometry())
    states = [VelocityState() for _ in range(4)]
    measured = [0.0] * 4
    rows = []
    summaries = []
    tick = 0
    for phase, fraction in STEPS:
        twist = ChassisTwist(2.29 * fraction, 0.0, 0.0)
        desired = solver.inverse_kinematics(twist, (58.64,) * 4).motor_rad_s
        before = tuple(state.target for state in states)
        first_change_tick = None
        exact_zero_cycles = [0] * 4
        for phase_tick in range(round(phase_s / dt)):
            for wheel in range(4):
                states[wheel] = step_velocity_target(
                    desired[wheel], measured[wheel], states[wheel], profile, dt
                )
                measured[wheel] = states[wheel].target
                if states[wheel].target == 0.0 and desired[wheel] != 0.0:
                    exact_zero_cycles[wheel] += 1
            if first_change_tick is None and any(
                (state.target - old) * target > 0
                for state, old, target in zip(states, before, desired)
                if old * target < 0
            ):
                first_change_tick = phase_tick
            rows.append({
                "time_s": round(tick * dt, 6), "phase": phase,
                "requested_vx_mps": twist.vx_mps,
                **{f"wheel_{name}_requested_rad_s": desired[index]
                   for index, name in enumerate(("FL", "FR", "RL", "RR"))},
                **{f"wheel_{name}_limited_rad_s": states[index].target
                   for index, name in enumerate(("FL", "FR", "RL", "RR"))},
                **{f"wheel_{name}_accel_rad_s2": states[index].acceleration
                   for index, name in enumerate(("FL", "FR", "RL", "RR"))},
                **{f"wheel_{name}_limiter": states[index].limiter_state
                   for index, name in enumerate(("FL", "FR", "RL", "RR"))},
            })
            tick += 1
        reversal = any(old * target < 0 for old, target in zip(before, desired))
        summaries.append({
            "phase": phase, "reversal": reversal,
            "command_to_output_change_cycles": first_change_tick,
            "artificial_zero_wait_cycles": max(exact_zero_cycles),
            "continuous_output": max(exact_zero_cycles) <= 1,
            "final_targets_rad_s": [state.target for state in states],
            "requested_targets_rad_s": list(desired),
        })
    return rows, {
        "source": "software_reference_model",
        "hardware_measurement": "not_measured",
        "profile": profile_name,
        "control_period_ms": dt * 1000.0,
        "limits": profile.__dict__,
        "phases": summaries,
        "all_reversals_change_next_cycle": all(
            item["command_to_output_change_cycles"] == 0
            for item in summaries if item["reversal"]
        ),
        "no_artificial_zero_wait": all(item["continuous_output"] for item in summaries),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=VELOCITY_PROFILES, default="normal")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    args = parser.parse_args()
    rows, summary = run(args.profile)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    args.json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
