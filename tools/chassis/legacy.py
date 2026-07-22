"""Explicit conversion at the old LF, LB, RF, RB API boundary."""

from __future__ import annotations


def legacy_order_to_canonical(values):
    """Convert old LF, LB, RF, RB to canonical FL, FR, RL, RR."""
    if len(values) != 4:
        raise ValueError("four wheel values are required")
    lf, lb, rf, rb = values
    return (lf, rf, lb, rb)


def canonical_to_legacy_order(values):
    """Convert canonical FL, FR, RL, RR to old LF, LB, RF, RB."""
    if len(values) != 4:
        raise ValueError("four wheel values are required")
    fl, fr, rl, rr = values
    return (fl, rl, fr, rr)
