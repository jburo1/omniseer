import sys
import unittest
from pathlib import Path

from robot_diag_control.monitor_gui import (
    DEFAULT_ROBOT_HOST,
    DEFAULT_ROBOT_USER,
    RUN_BACKEND_DEVCONTAINER,
    RUN_BACKEND_RUNTIME,
    _build_overlay_viewer_command,
    _build_parser,
    _build_preview_viewer_command,
    _build_pull_run_command,
    _build_remote_mkdir_command,
    _build_remote_start_command,
    _build_report_command,
    _build_upload_classes_command,
    _local_import_dir_for,
    _parse_run_classes,
    _remote_class_list_path_for,
    _resolved_preview_host,
    _sanitize_run_id,
    _teleop_command_for_action,
)


class MonitorGuiTests(unittest.TestCase):
    def test_parser_defaults(self):
        parser = _build_parser()

        args = parser.parse_args([])

        self.assertEqual(args.host, DEFAULT_ROBOT_HOST)
        self.assertEqual(args.port, 50051)
        self.assertEqual(args.poll_interval_seconds, 1.0)
        self.assertFalse(args.refresh_on_start)
        self.assertEqual(args.repo_root, ".")
        self.assertEqual(args.ssh_user, DEFAULT_ROBOT_USER)
        self.assertEqual(args.remote_repo_root, "/home/radxa/apps/omniseer")
        self.assertEqual(args.local_import_root, "runs/imported")
        self.assertEqual(args.run_backend, RUN_BACKEND_RUNTIME)

    def test_resolved_preview_host_defaults_to_gateway_host(self):
        parser = _build_parser()
        args = parser.parse_args(["--host", "10.0.0.8"])

        self.assertEqual(_resolved_preview_host(args), "10.0.0.8")

    def test_build_preview_viewer_command_uses_current_python(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--host",
                "10.0.0.2",
                "--port",
                "50070",
                "--preview-host",
                "10.0.0.3",
                "--preview-port",
                "7010",
                "--preview-latency-ms",
                "150",
            ]
        )

        command = _build_preview_viewer_command(
            args,
            profile_name="balanced",
            leave_preview_running=True,
        )

        self.assertEqual(command[:3], [sys.executable, "-m", "robot_diag_control.preview_viewer"])
        self.assertIn("--preview-host", command)
        self.assertIn("10.0.0.3", command)
        self.assertIn("--leave-preview-running", command)

    def test_build_overlay_viewer_command_uses_current_python(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--host",
                "10.0.0.2",
                "--port",
                "50070",
                "--preview-host",
                "10.0.0.3",
                "--preview-port",
                "7010",
                "--preview-latency-ms",
                "150",
            ]
        )

        command = _build_overlay_viewer_command(
            args,
            profile_name="balanced",
            leave_preview_running=True,
        )

        self.assertEqual(command[:3], [sys.executable, "-m", "robot_diag_control.overlay_viewer"])
        self.assertIn("--preview-host", command)
        self.assertIn("10.0.0.3", command)
        self.assertIn("--leave-preview-running", command)

    def test_teleop_command_for_action_maps_to_bounded_steps(self):
        self.assertEqual(
            _teleop_command_for_action(
                "forward",
                linear_step_mps=0.12,
                angular_step_rad_s=0.35,
            ),
            (0.12, 0.0, 0.0),
        )
        self.assertEqual(
            _teleop_command_for_action(
                "turn_right",
                linear_step_mps=0.12,
                angular_step_rad_s=0.35,
            ),
            (0.0, 0.0, -0.35),
        )

    def test_run_class_parser_splits_and_deduplicates_tokens(self):
        self.assertEqual(
            _parse_run_classes("person, chair\nfire extinguisher"),
            ["person", "chair", "fire extinguisher"],
        )

    def test_run_id_sanitizer_keeps_path_safe_subset(self):
        self.assertEqual(_sanitize_run_id(" demo run/01 "), "demo_run_01")

    def test_build_remote_mkdir_command_targets_robot_run_directory(self):
        command = _build_remote_mkdir_command(
            ssh_user="radxa",
            host="10.0.0.2",
            remote_run_dir="/home/radxa/apps/omniseer/runs/operator_001",
        )

        self.assertEqual(
            command,
            ["ssh", "radxa@10.0.0.2", "mkdir -p /home/radxa/apps/omniseer/runs/operator_001"],
        )

    def test_build_upload_classes_command_targets_robot_class_file(self):
        command = _build_upload_classes_command(
            ssh_user="radxa",
            host="10.0.0.2",
            local_path=Path("/tmp/classes.txt"),
            remote_class_path="/home/radxa/apps/omniseer/runs/operator_001/classes.txt",
        )

        self.assertEqual(
            command,
            [
                "scp",
                "/tmp/classes.txt",
                "radxa@10.0.0.2:/home/radxa/apps/omniseer/runs/operator_001/classes.txt",
            ],
        )

    def test_runtime_backend_uses_container_visible_class_path(self):
        command = _build_remote_start_command(
            backend=RUN_BACKEND_RUNTIME,
            ssh_user="radxa",
            host="10.0.0.2",
            remote_repo_root="/home/radxa/apps/omniseer",
            run_id="operator_001",
            classes=["person", "fire extinguisher"],
            notes="lighting changed",
            devcontainer_exec_template="ignored {command}",
        )

        self.assertEqual(command[0:2], ["ssh", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime record", command[2])
        self.assertIn("--record-notes 'lighting changed'", command[2])
        self.assertIn("--record-classes 'person,fire extinguisher'", command[2])
        self.assertIn("classes_path:=/runs/operator_001/classes.txt", command[2])

    def test_devcontainer_backend_uses_robot_host_visible_class_path(self):
        command = _build_remote_start_command(
            backend=RUN_BACKEND_DEVCONTAINER,
            ssh_user="radxa",
            host="10.0.0.2",
            remote_repo_root="/home/radxa/apps/omniseer",
            run_id="operator_001",
            classes=["person", "fire extinguisher"],
            notes="",
            devcontainer_exec_template="docker exec omniseer-dev bash -lc {command}",
        )

        self.assertEqual(command[0:2], ["ssh", "radxa@10.0.0.2"])
        self.assertIn("docker exec omniseer-dev bash -lc", command[2])
        self.assertIn("scripts/omni run real --profile operator", command[2])
        class_path = _remote_class_list_path_for("/home/radxa/apps/omniseer", "operator_001")
        self.assertIn(f"classes_path:={class_path}", command[2])

    def test_build_pull_command_uses_import_root_and_remote_root(self):
        command = _build_pull_run_command(
            repo_root=Path("/repo"),
            ssh_user="radxa",
            host="10.0.0.2",
            remote_runs_root="/home/radxa/apps/omniseer/runs",
            local_import_root=Path("/repo/runs/imported"),
            run_id="operator_001",
        )

        self.assertEqual(command[0:4], ["/repo/scripts/omni", "runs", "pull", "operator_001"])
        self.assertIn("--host", command)
        self.assertIn("10.0.0.2", command)
        self.assertIn("--remote-root", command)
        self.assertIn("/home/radxa/apps/omniseer/runs", command)
        self.assertIn("--import-root", command)
        self.assertIn("/repo/runs/imported", command)
        self.assertIn("--overwrite", command)

    def test_build_report_command_targets_imported_run(self):
        run_dir = _local_import_dir_for(Path("/repo/runs/imported"), "operator_001")
        command = _build_report_command(repo_root=Path("/repo"), run_dir=run_dir)

        self.assertEqual(
            command,
            ["/repo/scripts/omni", "runs", "report", "/repo/runs/imported/operator_001", "--overwrite"],
        )
