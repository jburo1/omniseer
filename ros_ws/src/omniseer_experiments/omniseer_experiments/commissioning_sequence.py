"""Declarative, one-shot motion sequence for real-hardware commissioning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

COMMAND_RATE_HZ = 20.0
MAX_ABS_VX_M_S = 0.08
MAX_ABS_VY_M_S = 0.08
MAX_ABS_WZ_RAD_S = 0.30
TERMINAL_ZERO_COMMAND_COUNT = 5


@dataclass(frozen=True)
class MotionCommand:
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    wz_rad_s: float = 0.0


@dataclass(frozen=True)
class CommissioningPhase:
    name: str
    duration_sec: float
    command: MotionCommand


ZERO_COMMAND = MotionCommand()

DEFAULT_PHASES: tuple[CommissioningPhase, ...] = (
    CommissioningPhase("settle_initial", 10.0, ZERO_COMMAND),
    CommissioningPhase("forward", 4.0, MotionCommand(vx_m_s=0.06)),
    CommissioningPhase("settle_1", 3.0, ZERO_COMMAND),
    CommissioningPhase("reverse", 4.0, MotionCommand(vx_m_s=-0.06)),
    CommissioningPhase("settle_2", 3.0, ZERO_COMMAND),
    CommissioningPhase("strafe_left", 4.0, MotionCommand(vy_m_s=0.05)),
    CommissioningPhase("settle_3", 3.0, ZERO_COMMAND),
    CommissioningPhase("strafe_right", 4.0, MotionCommand(vy_m_s=-0.05)),
    CommissioningPhase("settle_4", 3.0, ZERO_COMMAND),
    CommissioningPhase("rotate_ccw", 4.0, MotionCommand(wz_rad_s=0.20)),
    CommissioningPhase("settle_5", 3.0, ZERO_COMMAND),
    CommissioningPhase("rotate_cw", 4.0, MotionCommand(wz_rad_s=-0.20)),
    CommissioningPhase("settle_final", 10.0, ZERO_COMMAND),
)


def validate_phase(phase: CommissioningPhase) -> None:
    """Reject non-conservative commands and simultaneous-axis motion."""
    if not phase.name.strip():
        raise ValueError("commissioning phase name must not be empty")
    if phase.duration_sec <= 0.0:
        raise ValueError(f"phase {phase.name!r} duration must be positive")

    command = phase.command
    nonzero_axes = sum(value != 0.0 for value in (command.vx_m_s, command.vy_m_s, command.wz_rad_s))
    if nonzero_axes > 1:
        raise ValueError(f"phase {phase.name!r} commands multiple nonzero axes")
    if abs(command.vx_m_s) > MAX_ABS_VX_M_S:
        raise ValueError(f"phase {phase.name!r} vx exceeds {MAX_ABS_VX_M_S} m/s")
    if abs(command.vy_m_s) > MAX_ABS_VY_M_S:
        raise ValueError(f"phase {phase.name!r} vy exceeds {MAX_ABS_VY_M_S} m/s")
    if abs(command.wz_rad_s) > MAX_ABS_WZ_RAD_S:
        raise ValueError(f"phase {phase.name!r} wz exceeds {MAX_ABS_WZ_RAD_S} rad/s")


def validate_sequence(phases: Sequence[CommissioningPhase]) -> None:
    if not phases:
        raise ValueError("commissioning sequence must contain at least one phase")
    for phase in phases:
        validate_phase(phase)


def total_duration_sec(phases: Sequence[CommissioningPhase]) -> float:
    validate_sequence(phases)
    return sum(phase.duration_sec for phase in phases)


def phase_markers(phases: Sequence[CommissioningPhase]) -> tuple[str, ...]:
    validate_sequence(phases)
    return tuple(phase.name for phase in phases)


def execute_sequence(
    phases: Sequence[CommissioningPhase],
    *,
    publish_command: Callable[[MotionCommand], None],
    publish_phase: Callable[[str], None],
    is_running: Callable[[], bool],
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
    command_rate_hz: float = COMMAND_RATE_HZ,
    terminal_zero_command_count: int = TERMINAL_ZERO_COMMAND_COUNT,
) -> bool:
    """Publish one sequence and always finish with several zero commands.

    Returns ``True`` only after every phase completed. This function has no
    repeat path; callers choose whether to run it at all.
    """
    validate_sequence(phases)
    if command_rate_hz <= 0.0:
        raise ValueError("command_rate_hz must be positive")
    if terminal_zero_command_count <= 0:
        raise ValueError("terminal_zero_command_count must be positive")

    period_sec = 1.0 / command_rate_hz
    completed = False
    try:
        for phase in phases:
            deadline = monotonic() + phase.duration_sec
            while True:
                if not is_running():
                    return False
                publish_phase(phase.name)
                publish_command(phase.command)
                remaining_sec = deadline - monotonic()
                if remaining_sec <= 0.0:
                    break
                sleep(min(period_sec, remaining_sec))
        completed = True
        return completed
    finally:
        # Keep this best-effort: an external ROS shutdown can invalidate a
        # publisher, but Ctrl-C and normal exceptions still get safe commands.
        for _ in range(terminal_zero_command_count):
            try:
                publish_phase("terminal_zero")
                publish_command(ZERO_COMMAND)
            except Exception:  # noqa: BLE001 - terminal safety is best effort
                pass
            try:
                sleep(period_sec)
            except Exception:  # noqa: BLE001 - preserve the original failure
                pass
