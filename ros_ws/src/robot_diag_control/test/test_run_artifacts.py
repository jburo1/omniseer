import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from robot_diag_control.run_artifacts import (
    RunArtifactContext,
    build_pull_command,
    build_report_command,
    generate_run_report,
    imported_run_dir,
    inspect_run_artifacts,
    report_path,
    retrieve_run_artifacts,
)
from robot_diag_control.run_commands import RobotConnection


def _context() -> RunArtifactContext:
    return RunArtifactContext(
        repo_root=Path("/repo"),
        connection=RobotConnection(
            host="10.0.0.2",
            ssh_user="radxa",
            remote_repo_root="/home/radxa/apps/omniseer",
            remote_runs_root="/home/radxa/apps/omniseer/runs",
        ),
        local_import_root=Path("/repo/runs/imported"),
    )


def _context_for_repo(repo_root: Path) -> RunArtifactContext:
    return RunArtifactContext(
        repo_root=repo_root,
        connection=RobotConnection(
            host="10.0.0.2",
            ssh_user="radxa",
            remote_repo_root="/home/radxa/apps/omniseer",
            remote_runs_root="/home/radxa/apps/omniseer/runs",
        ),
        local_import_root=repo_root / "runs" / "imported",
    )


def _write_completed_bundle(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "manifest.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                'run_id: "operator_001"',
                'started_at: "2026-07-19T12:00:00+00:00"',
                'ended_at: "2026-07-19T12:01:05+00:00"',
                "classes: []",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (run_dir / "detections.jsonl").write_text("", encoding="utf-8")
    (run_dir / "perf.jsonl").write_text("", encoding="utf-8")
    (run_dir / "summary.json").write_text(
        json.dumps(
            {
                "run_id": "operator_001",
                "duration_sec": 65.0,
                "message_counts": {"detections": 0, "perf": 0, "system": 0},
                "errors": {},
                "perf": {},
                "dropped_records": {},
            }
        ),
        encoding="utf-8",
    )


def _write_in_progress_bundle(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "manifest.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                'run_id: "operator_001"',
                'started_at: "2026-07-19T12:00:00+00:00"',
                "ended_at: null",
                "classes: []",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (run_dir / "detections.jsonl").write_text("", encoding="utf-8")
    (run_dir / "perf.jsonl").write_text("", encoding="utf-8")


def _write_report(run_dir: Path) -> None:
    report = run_dir / "report" / "index.html"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("<!doctype html>\n", encoding="utf-8")


class RunArtifactsTests(unittest.TestCase):
    def test_imported_run_dir_uses_context_import_root(self):
        self.assertEqual(imported_run_dir(_context(), "operator_001"), Path("/repo/runs/imported/operator_001"))

    def test_report_path_targets_generated_html_report(self):
        self.assertEqual(
            report_path(_context(), "operator_001"),
            Path("/repo/runs/imported/operator_001/report/index.html"),
        )

    def test_build_pull_command_uses_artifact_context(self):
        command = build_pull_command(_context(), run_id="operator_001")

        self.assertEqual(command[0:4], ["/repo/scripts/omni", "runs", "pull", "operator_001"])
        self.assertIn("--host", command)
        self.assertIn("10.0.0.2", command)
        self.assertIn("--user", command)
        self.assertIn("radxa", command)
        self.assertIn("--remote-root", command)
        self.assertIn("/home/radxa/apps/omniseer/runs", command)
        self.assertIn("--import-root", command)
        self.assertIn("/repo/runs/imported", command)
        self.assertIn("--overwrite", command)

    def test_build_report_command_targets_imported_run(self):
        command = build_report_command(_context(), run_id="operator_001")

        self.assertEqual(
            command,
            ["/repo/scripts/omni", "runs", "report", "/repo/runs/imported/operator_001", "--overwrite"],
        )

    def test_retrieve_run_artifacts_executes_pull_command_and_reports_success(self):
        commands: list[list[str]] = []

        def command_executor(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
            commands.append(command)
            _write_completed_bundle(cwd / "runs" / "imported" / "operator_001")
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            context = _context_for_repo(Path(tmp))

            result = retrieve_run_artifacts(context, run_id="operator_001", command_executor=command_executor)

        self.assertTrue(result.success)
        self.assertEqual(result.command, commands[0])
        self.assertEqual(result.path, Path(tmp) / "runs" / "imported" / "operator_001")
        self.assertEqual(result.message, f"retrieved run: {Path(tmp) / 'runs' / 'imported' / 'operator_001'}")
        self.assertEqual(result.issues, ())

    def test_retrieve_run_artifacts_forwards_optional_progress_callback(self):
        progress_values = []

        def progress_command_executor(command: list[str], cwd: Path, on_output):
            self.assertIn("--progress", command)
            _write_completed_bundle(cwd / "runs" / "imported" / "operator_001")
            on_output("OMNISEER_RSYNC_PROGRESS 67 851443712 1270811510\n")
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            context = _context_for_repo(Path(tmp))
            result = retrieve_run_artifacts(
                context,
                run_id="operator_001",
                progress_callback=progress_values.append,
                progress_command_executor=progress_command_executor,
            )

        self.assertTrue(result.success)
        self.assertEqual(len(progress_values), 1)
        self.assertEqual(progress_values[0].percent, 67)
        self.assertEqual(progress_values[0].transferred_bytes, 851_443_712)

    def test_inspect_run_artifacts_reports_incomplete_bundle_with_existing_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            context = _context_for_repo(Path(tmp))
            run_dir = imported_run_dir(context, "operator_001")
            _write_in_progress_bundle(run_dir)
            _write_report(run_dir)

            inspection = inspect_run_artifacts(context, "operator_001", require_report=True)

        self.assertEqual(inspection.state, "in_progress")
        self.assertTrue(inspection.report_exists)
        self.assertFalse(inspection.complete)
        self.assertTrue(any("run_open: manifest ended_at is null" in issue for issue in inspection.issues))
        self.assertTrue(any("missing_summary: summary.json is missing" in issue for issue in inspection.issues))

    def test_retrieve_run_artifacts_reports_command_failure(self):
        result = retrieve_run_artifacts(
            _context(),
            run_id="operator_001",
            command_executor=lambda command, _cwd: subprocess.CompletedProcess(
                command,
                42,
                stdout="",
                stderr="rsync failed\n",
            ),
        )

        self.assertFalse(result.success)
        self.assertEqual(result.message, "retrieve failed: rsync failed")

    def test_retrieve_run_artifacts_reports_launch_failure(self):
        def command_executor(_command: list[str], _cwd: Path) -> subprocess.CompletedProcess[str]:
            raise OSError("missing scripts/omni")

        result = retrieve_run_artifacts(_context(), run_id="operator_001", command_executor=command_executor)

        self.assertFalse(result.success)
        self.assertEqual(result.message, "retrieve failed: missing scripts/omni")

    def test_generate_run_report_executes_report_command_and_reports_success(self):
        commands: list[list[str]] = []

        def command_executor(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
            commands.append(command)
            run_dir = cwd / "runs" / "imported" / "operator_001"
            _write_completed_bundle(run_dir)
            _write_report(run_dir)
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            context = _context_for_repo(Path(tmp))

            result = generate_run_report(context, run_id="operator_001", command_executor=command_executor)

        self.assertTrue(result.success)
        self.assertEqual(result.command, commands[0])
        expected_path = Path(tmp) / "runs" / "imported" / "operator_001" / "report" / "index.html"
        self.assertEqual(result.path, expected_path)
        self.assertEqual(result.message, f"report generated: {expected_path}")
        self.assertEqual(result.issues, ())

    def test_generate_run_report_surfaces_incomplete_bundle_warnings(self):
        def command_executor(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
            run_dir = cwd / "runs" / "imported" / "operator_001"
            _write_in_progress_bundle(run_dir)
            _write_report(run_dir)
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            context = _context_for_repo(Path(tmp))

            result = generate_run_report(context, run_id="operator_001", command_executor=command_executor)

        self.assertTrue(result.success)
        self.assertTrue(result.message.startswith("report generated with artifact warnings: "))
        self.assertTrue(any("run_open: manifest ended_at is null" in issue for issue in result.issues))
        self.assertTrue(any("missing_summary: summary.json is missing" in issue for issue in result.issues))

    def test_generate_run_report_reports_command_failure(self):
        result = generate_run_report(
            _context(),
            run_id="operator_001",
            command_executor=lambda command, _cwd: subprocess.CompletedProcess(
                command,
                42,
                stdout="",
                stderr="report failed\n",
            ),
        )

        self.assertFalse(result.success)
        self.assertEqual(result.message, "report generation failed: report failed")

    def test_generate_run_report_reports_launch_failure(self):
        def command_executor(_command: list[str], _cwd: Path) -> subprocess.CompletedProcess[str]:
            raise OSError("missing report tool")

        result = generate_run_report(_context(), run_id="operator_001", command_executor=command_executor)

        self.assertFalse(result.success)
        self.assertEqual(result.message, "failed to generate report: missing report tool")
