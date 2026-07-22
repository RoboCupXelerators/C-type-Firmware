"""Pure-software four-motor safety state machine and command limiter."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

from .kinematics import ChassisKinematics, ChassisTwist, WheelTargets


class ChassisState(str, Enum):
    BOOT = "BOOT"
    DISABLED = "DISABLED"
    ARMING = "ARMING"
    READY = "READY"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"


@dataclass(frozen=True)
class MotorFeedback:
    assigned: bool
    online: bool
    enabled: bool
    fault_code: int
    received_monotonic: float


@dataclass(frozen=True)
class ControlLimits:
    max_vx_mps: float = 0.3
    max_vy_mps: float = 0.3
    max_wz_rad_s: float = 2.0
    max_motor_rad_s: tuple[float, float, float, float] = (20.0,) * 4
    feedback_timeout_s: float = 0.1
    command_timeout_s: float = 0.1


@dataclass
class ChassisController:
    kinematics: ChassisKinematics
    limits: ControlLimits = field(default_factory=ControlLimits)
    state: ChassisState = ChassisState.BOOT
    fault_reason: str | None = None
    latest_command: ChassisTwist = field(default_factory=lambda: ChassisTwist(0, 0, 0))
    latest_command_monotonic: float | None = None
    last_sequence: int | None = None

    def initialize(self) -> None:
        if self.kinematics.validation.rank != 3:
            self._fault("INVALID_GEOMETRY")
            return
        self.state = ChassisState.DISABLED

    def arm(self, feedback: Sequence[MotorFeedback], now: float) -> bool:
        self._require_four(feedback)
        if self.state != ChassisState.DISABLED:
            return False
        if not all(item.assigned for item in feedback):
            return False
        if not self._healthy(feedback, now, require_enabled=False):
            self._fault("ARM_FEEDBACK_UNHEALTHY")
            return False
        self.state = ChassisState.ARMING
        return True

    def observe_feedback(self, feedback: Sequence[MotorFeedback], now: float) -> None:
        self._require_four(feedback)
        if self.state in (ChassisState.ARMING, ChassisState.READY, ChassisState.RUNNING):
            if not self._healthy(feedback, now, require_enabled=True):
                self._fault("MOTOR_FEEDBACK_UNHEALTHY")
            elif self.state == ChassisState.ARMING:
                self.state = ChassisState.READY

    def accept_twist(self, sequence: int, command: ChassisTwist, now: float) -> bool:
        if self.state not in (ChassisState.READY, ChassisState.RUNNING):
            return False
        sequence &= 0xFF
        if self.last_sequence is not None:
            distance = (sequence - self.last_sequence) & 0xFF
            if distance == 0 or distance > 127:
                return False
        if not all(math.isfinite(value) for value in command.values()):
            self._fault("NONFINITE_COMMAND")
            return False
        self.latest_command = self.kinematics.limit_twist(
            command,
            max_vx_mps=self.limits.max_vx_mps,
            max_vy_mps=self.limits.max_vy_mps,
            max_wz_rad_s=self.limits.max_wz_rad_s,
        )
        self.latest_command_monotonic = now
        self.last_sequence = sequence
        self.state = ChassisState.RUNNING if any(self.latest_command.values()) else ChassisState.READY
        return True

    def update(self, now: float) -> WheelTargets:
        if self.state == ChassisState.RUNNING:
            if self.latest_command_monotonic is None or now - self.latest_command_monotonic > self.limits.command_timeout_s:
                self.stop("HOST_COMMAND_TIMEOUT")
        command = self.latest_command if self.state in (ChassisState.READY, ChassisState.RUNNING) else ChassisTwist(0, 0, 0)
        return self.kinematics.inverse_kinematics(command, self.limits.max_motor_rad_s)

    def stop(self, reason: str = "STOP_REQUESTED") -> None:
        self.latest_command = ChassisTwist(0, 0, 0)
        self.latest_command_monotonic = None
        if self.state not in (ChassisState.DISABLED, ChassisState.FAULT):
            self.state = ChassisState.STOPPING
        if reason != "STOP_REQUESTED" and self.fault_reason is None:
            self.fault_reason = reason

    def disabled_confirmed(self, feedback: Sequence[MotorFeedback], now: float) -> bool:
        self._require_four(feedback)
        if self.state != ChassisState.STOPPING:
            return False
        if all(item.assigned and item.online and not item.enabled and item.fault_code == 0 and now - item.received_monotonic <= self.limits.feedback_timeout_s for item in feedback):
            self.state = ChassisState.DISABLED
            return True
        return False

    def _fault(self, reason: str) -> None:
        self.latest_command = ChassisTwist(0, 0, 0)
        self.latest_command_monotonic = None
        self.fault_reason = reason
        self.state = ChassisState.FAULT

    def _healthy(self, feedback: Sequence[MotorFeedback], now: float, require_enabled: bool) -> bool:
        return all(
            item.assigned
            and item.online
            and item.fault_code == 0
            and (not require_enabled or item.enabled)
            and 0.0 <= now - item.received_monotonic <= self.limits.feedback_timeout_s
            for item in feedback
        )

    @staticmethod
    def _require_four(feedback: Sequence[MotorFeedback]) -> None:
        if len(feedback) != 4:
            raise ValueError("exactly four motor feedback records are required")
