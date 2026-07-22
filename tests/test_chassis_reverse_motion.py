import math

import pytest

from tools.chassis.motion import VELOCITY_PROFILES, VelocityState, step_velocity_target
from tools.chassis.reverse_step_benchmark import run


def test_profiles_preserve_ordered_reverse_authority():
    for profile in VELOCITY_PROFILES.values():
        assert profile.reverse_brake > profile.normal_decel >= profile.normal_accel
        assert profile.reverse_jerk > profile.normal_jerk


@pytest.mark.parametrize("old,new", [(25.0, -25.0), (-25.0, 25.0)])
def test_reversal_changes_output_on_next_control_cycle(old, new):
    state = VelocityState(target=old, acceleration=12.0 * math.copysign(1.0, old))
    result = step_velocity_target(new, old, state, VELOCITY_PROFILES["normal"])
    assert (result.target - old) * new > 0.0
    assert result.limiter_state == "reverse_brake"
    assert result.reverse_state == "braking"


def test_reversal_crosses_zero_without_holding():
    profile = VELOCITY_PROFILES["normal"]
    state = VelocityState(target=2.0)
    zero_cycles = 0
    for _ in range(200):
        state = step_velocity_target(-2.0, state.target, state, profile)
        zero_cycles += state.target == 0.0
        if state.target < 0.0:
            break
    assert state.target < 0.0
    assert zero_cycles <= 1


@pytest.mark.parametrize("target,measured,dt", [(math.nan, 0.0, 0.01), (0.0, math.inf, 0.01), (0.0, 0.0, 0.0)])
def test_velocity_limiter_rejects_non_finite_or_invalid_inputs(target, measured, dt):
    with pytest.raises(ValueError):
        step_velocity_target(target, measured, VelocityState(), VELOCITY_PROFILES["normal"], dt)


def test_required_software_benchmark_has_no_artificial_reverse_wait():
    _, summary = run(phase_s=4.0)
    names = {phase["phase"] for phase in summary["phases"]}
    assert {"pos25_to_neg25", "neg25_to_pos25", "pos50_to_neg50", "pos100_to_neg100"} <= names
    assert summary["all_reversals_change_next_cycle"] is True
    assert summary["no_artificial_zero_wait"] is True
    assert summary["hardware_measurement"] == "not_measured"
