"""Non-orthogonal four-wheel chassis reference implementation."""

from .geometry import (
    ChassisGeometry,
    GeometryStatus,
    MotorMapping,
    WheelGeometry,
    WheelIndex,
    candidate_80_100_geometry,
)
from .kinematics import ChassisKinematics, ChassisTwist, WheelTargets, limit_wheel_speeds, unwrap_position
from .config import DEFAULT_CONFIG_PATH, load_geometry
from .controller import ChassisController, ChassisState, ControlLimits, MotorFeedback
from .legacy import canonical_to_legacy_order, legacy_order_to_canonical
from .status import FourMotorStatus, WheelMotorStatus, status_from_protocol
from .motion import AxisLimit, TwistLimiter
from .odometry import FeedbackEstimate, PlanarOdometry, estimate_feedback
from .diagnostics import DiagnosticCode, DiagnosticThresholds, diagnose_chassis, diagnose_wheel
from .replay import FeedbackReplaySample, replay_four_motor_csv

__all__ = [
    "ChassisGeometry",
    "ChassisKinematics",
    "ChassisController",
    "ChassisState",
    "ChassisTwist",
    "GeometryStatus",
    "ControlLimits",
    "MotorMapping",
    "MotorFeedback",
    "WheelGeometry",
    "WheelIndex",
    "WheelTargets",
    "candidate_80_100_geometry",
    "canonical_to_legacy_order",
    "legacy_order_to_canonical",
    "FourMotorStatus",
    "WheelMotorStatus",
    "status_from_protocol",
    "AxisLimit",
    "TwistLimiter",
    "FeedbackEstimate",
    "PlanarOdometry",
    "estimate_feedback",
    "DiagnosticCode",
    "DiagnosticThresholds",
    "diagnose_wheel",
    "diagnose_chassis",
    "FeedbackReplaySample",
    "replay_four_motor_csv",
    "limit_wheel_speeds",
    "unwrap_position",
    "DEFAULT_CONFIG_PATH",
    "load_geometry",
]
