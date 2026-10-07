import unittest
from contextlib import redirect_stdout
from io import StringIO

from omniseer_experiments.commissioning_sequence import (
    DEFAULT_PHASES,
    MAX_ABS_VX_M_S,
    MotionCommand,
    CommissioningPhase,
    execute_sequence,
    phase_markers,
    total_duration_sec,
    validate_phase,
)
from omniseer_experiments.commissioning_motion import main


class _FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_sec: float) -> None:
        self.now += duration_sec


class CommissioningSequenceTests(unittest.TestCase):
    def test_default_phase_order_values_and_duration(self) -> None:
        self.assertEqual(
            phase_markers(DEFAULT_PHASES),
            (
                "settle_initial",
                "forward",
                "settle_1",
                "reverse",
                "settle_2",
                "strafe_left",
                "settle_3",
                "strafe_right",
                "settle_4",
                "rotate_ccw",
                "settle_5",
                "rotate_cw",
                "settle_final",
            ),
        )
        self.assertEqual(DEFAULT_PHASES[1].command, MotionCommand(vx_m_s=0.06))
        self.assertEqual(DEFAULT_PHASES[5].command, MotionCommand(vy_m_s=0.05))
        self.assertEqual(DEFAULT_PHASES[9].command, MotionCommand(wz_rad_s=0.20))
        self.assertEqual(total_duration_sec(DEFAULT_PHASES), 59.0)

    def test_rejects_multi_axis_and_excessive_velocity(self) -> None:
        with self.assertRaisesRegex(ValueError, "multiple nonzero axes"):
            validate_phase(CommissioningPhase("unsafe", 1.0, MotionCommand(vx_m_s=0.01, wz_rad_s=0.01)))
        with self.assertRaisesRegex(ValueError, "vx exceeds"):
            validate_phase(CommissioningPhase("too_fast", 1.0, MotionCommand(vx_m_s=MAX_ABS_VX_M_S + 0.01)))

    def test_phase_marker_generation_rejects_invalid_phase(self) -> None:
        with self.assertRaisesRegex(ValueError, "duration must be positive"):
            phase_markers((CommissioningPhase("bad", 0.0, MotionCommand()),))

    def test_terminal_zero_commands_follow_normal_completion(self) -> None:
        clock = _FakeClock()
        commands: list[MotionCommand] = []
        phases: list[str] = []
        completed = execute_sequence(
            (CommissioningPhase("move", 0.1, MotionCommand(vx_m_s=0.01)),),
            publish_command=commands.append,
            publish_phase=phases.append,
            is_running=lambda: True,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
            command_rate_hz=20.0,
            terminal_zero_command_count=3,
        )
        self.assertTrue(completed)
        self.assertEqual(commands[-3:], [MotionCommand(), MotionCommand(), MotionCommand()])
        self.assertEqual(phases[-3:], ["terminal_zero", "terminal_zero", "terminal_zero"])

    def test_terminal_zero_commands_follow_abort(self) -> None:
        commands: list[MotionCommand] = []
        phases: list[str] = []
        completed = execute_sequence(
            (CommissioningPhase("move", 1.0, MotionCommand(vx_m_s=0.01)),),
            publish_command=commands.append,
            publish_phase=phases.append,
            is_running=lambda: False,
            monotonic=lambda: 0.0,
            sleep=lambda _duration_sec: None,
            terminal_zero_command_count=2,
        )
        self.assertFalse(completed)
        self.assertEqual(commands, [MotionCommand(), MotionCommand()])
        self.assertEqual(phases, ["terminal_zero", "terminal_zero"])

    def test_dry_run_prints_timeline_without_ros_initialization(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            self.assertEqual(main(["--dry-run"]), 0)
        self.assertIn("settle_initial", output.getvalue())
        self.assertIn("Total duration: 59.0 s", output.getvalue())
