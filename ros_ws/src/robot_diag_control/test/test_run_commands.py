import unittest
from pathlib import Path

from robot_diag_control.run_commands import (
    RUN_BACKEND_DEVCONTAINER,
    RUN_BACKEND_RUNTIME,
    RobotConnection,
    RunConfig,
    build_pull_run_command,
    build_remote_run_mkdir_command,
    build_remote_runtime_stop_command,
    build_remote_start_command,
    build_report_command,
    build_upload_classes_command,
    local_import_dir_for,
    parse_run_classes,
    remote_class_list_path_for,
    sanitize_run_id,
)


def _connection() -> RobotConnection:
    return RobotConnection(
        host="10.0.0.2",
        ssh_user="radxa",
        remote_repo_root="/home/radxa/apps/omniseer",
        remote_runs_root="/home/radxa/apps/omniseer/runs",
    )


class RunCommandsTests(unittest.TestCase):
    def test_run_class_parser_splits_and_deduplicates_tokens(self):
        self.assertEqual(
            parse_run_classes("person, chair\nfire extinguisher"),
            ["person", "chair", "fire extinguisher"],
        )

    def test_run_id_sanitizer_keeps_path_safe_subset(self):
        self.assertEqual(sanitize_run_id(" demo run/01 "), "demo_run_01")

    def test_build_remote_mkdir_command_targets_robot_run_directory(self):
        command = build_remote_run_mkdir_command(
            connection=_connection(),
            run_config=RunConfig(run_id="operator_001", backend=RUN_BACKEND_RUNTIME),
        )

        self.assertEqual(
            command,
            ["ssh", "radxa@10.0.0.2", "mkdir -p /home/radxa/apps/omniseer/runs/operator_001"],
        )

    def test_build_upload_classes_command_targets_robot_class_file(self):
        command = build_upload_classes_command(
            connection=_connection(),
            run_config=RunConfig(run_id="operator_001", backend=RUN_BACKEND_RUNTIME),
            local_path=Path("/tmp/classes.txt"),
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
        command = build_remote_start_command(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_RUNTIME,
                classes=("person", "fire extinguisher"),
                notes="lighting changed",
                devcontainer_exec_template="ignored {command}",
            ),
        )

        self.assertEqual(command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime record", command[3])
        self.assertIn("--record-notes 'lighting changed'", command[3])
        self.assertIn("--record-classes 'person,fire extinguisher'", command[3])
        self.assertIn("classes_path:=/runs/operator_001/classes.txt", command[3])

    def test_devcontainer_backend_uses_robot_host_visible_class_path(self):
        command = build_remote_start_command(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_DEVCONTAINER,
                classes=("person", "fire extinguisher"),
                devcontainer_exec_template="docker exec omniseer-dev bash -lc {command}",
            ),
        )

        self.assertEqual(command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIn("docker exec omniseer-dev bash -lc", command[3])
        self.assertIn("scripts/omni run real --profile operator", command[3])
        class_path = remote_class_list_path_for("/home/radxa/apps/omniseer", "operator_001")
        self.assertIn(f"classes_path:={class_path}", command[3])

    def test_runtime_stop_command_targets_named_runtime_record_container(self):
        command = build_remote_runtime_stop_command(
            connection=_connection(),
            run_id="operator_001",
        )

        self.assertEqual(command[0:2], ["ssh", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime stop --run-id operator_001", command[2])

    def test_build_pull_command_uses_import_root_and_remote_root(self):
        command = build_pull_run_command(
            repo_root=Path("/repo"),
            connection=_connection(),
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
        run_dir = local_import_dir_for(Path("/repo/runs/imported"), "operator_001")
        command = build_report_command(repo_root=Path("/repo"), run_dir=run_dir)

        self.assertEqual(
            command,
            ["/repo/scripts/omni", "runs", "report", "/repo/runs/imported/operator_001", "--overwrite"],
        )
