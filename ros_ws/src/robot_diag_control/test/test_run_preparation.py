import sys
import unittest
from pathlib import Path

from robot_diag_control.run_commands import (
    RUN_BACKEND_RUNTIME,
    RUN_TYPE_AUTONOMY_CENTER,
    RobotConnection,
    RunConfig,
)
from robot_diag_control.run_preparation import prepare_remote_run, run_command_checked


def _connection() -> RobotConnection:
    return RobotConnection(
        host="10.0.0.2",
        ssh_user="radxa",
        remote_repo_root="/home/radxa/apps/omniseer",
        remote_runs_root="/home/radxa/apps/omniseer/runs",
    )


class RunPreparationTests(unittest.TestCase):
    def test_prepare_remote_run_creates_directory_and_returns_start_command_without_classes(self):
        calls: list[tuple[str, list[str]]] = []

        def runner(command: list[str], action: str) -> None:
            calls.append((action, command))

        start_command = prepare_remote_run(
            connection=_connection(),
            run_config=RunConfig(run_id="operator_001", backend=RUN_BACKEND_RUNTIME),
            cwd=Path("/repo"),
            command_runner=runner,
        )

        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "create remote run directory")
        self.assertEqual(
            calls[0][1],
            ["ssh", "radxa@10.0.0.2", "mkdir -p /home/radxa/apps/omniseer/runs/operator_001"],
        )
        self.assertEqual(start_command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime record --tag robot-verified --run-id operator_001", start_command[3])

    def test_prepare_remote_run_uploads_class_file_before_start_command(self):
        calls: list[tuple[str, list[str]]] = []

        def runner(command: list[str], action: str) -> None:
            calls.append((action, command))
            if action == "upload classes":
                class_file = Path(command[1])
                self.assertEqual(class_file.read_text(encoding="utf-8"), "person\nfire extinguisher\n")

        start_command = prepare_remote_run(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_RUNTIME,
                classes=("person", "fire extinguisher"),
                run_type=RUN_TYPE_AUTONOMY_CENTER,
            ),
            cwd=Path("/repo"),
            command_runner=runner,
        )

        self.assertEqual([action for action, _command in calls], ["create remote run directory", "upload classes"])
        self.assertEqual(calls[1][1][0], "scp")
        self.assertIn("classes_path:=/runs/operator_001/classes.txt", start_command[3])

    def test_run_command_checked_reports_subprocess_failure_detail(self):
        with self.assertRaisesRegex(OSError, "test action failed: bad"):
            run_command_checked(
                [sys.executable, "-c", "import sys; print('bad', file=sys.stderr); raise SystemExit(3)"],
                "test action",
                cwd=Path.cwd(),
            )
