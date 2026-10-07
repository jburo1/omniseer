"""One-shot real-hardware motion commissioning runner."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from omniseer_experiments.commissioning_sequence import (
    COMMAND_RATE_HZ,
    DEFAULT_PHASES,
    MotionCommand,
    phase_markers,
    total_duration_sec,
)

def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the one-shot real-hardware commissioning motion sequence.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the validated sequence and duration without initializing ROS or publishing commands",
    )
    return parser


def _format_command(command: MotionCommand) -> str:
    return f"vx={command.vx_m_s:+.2f} m/s, vy={command.vy_m_s:+.2f} m/s, wz={command.wz_rad_s:+.2f} rad/s"


def print_dry_run() -> None:
    print(f"Commissioning sequence ({COMMAND_RATE_HZ:.0f} Hz; one shot):")
    for phase in DEFAULT_PHASES:
        print(f"  {phase.name:15s} {phase.duration_sec:4.1f} s  {_format_command(phase.command)}")
    print(f"Phase markers: {', '.join(phase_markers(DEFAULT_PHASES))}")
    print(f"Total duration: {total_duration_sec(DEFAULT_PHASES):.1f} s")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.dry_run:
        print_dry_run()
        return 0

    import rclpy

    from omniseer_experiments.commissioning_runner import CommissioningMotionRunner

    rclpy.init(args=None)
    runner = CommissioningMotionRunner()
    try:
        runner.get_logger().warning(
            "starting one-shot commissioning motion sequence; publishes only to /cmd_vel_autonomy"
        )
        completed = runner.run()
        if not completed:
            runner.get_logger().warning("commissioning sequence interrupted; terminal zero commands requested")
            return 1
        runner.get_logger().info("commissioning sequence complete; terminal zero commands published")
        return 0
    except KeyboardInterrupt:
        runner.get_logger().warning("commissioning sequence interrupted by Ctrl-C; terminal zero commands requested")
        return 130
    except Exception as exc:  # noqa: BLE001 - report after execute_sequence safety finally
        runner.get_logger().error(f"commissioning sequence failed: {exc}")
        return 1
    finally:
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
