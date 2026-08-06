import unittest
from pathlib import Path

from robot_diag_control.run_commands import (
    DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
    RUN_BACKEND_DEVCONTAINER,
    RUN_BACKEND_RUNTIME,
    RUN_TYPE_AUTONOMY_CENTER,
    RobotConnection,
    RunConfig,
    build_pull_run_command,
    build_remote_run_mkdir_command,
    build_remote_runtime_stop_command,
    build_remote_start_command,
    build_report_command,
    build_upload_classes_command,
    devcontainer_workspace_root_for,
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

    def test_devcontainer_workspace_root_uses_repo_basename(self):
        self.assertEqual(devcontainer_workspace_root_for("/home/radxa/apps/omniseer"), "/omniseer")
        self.assertEqual(devcontainer_workspace_root_for("/home/radxa/apps/omniseer/"), "/omniseer")

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
                detector_score_threshold="0.31",
                detector_nms_iou_threshold="0.52",
                detector_max_detections="42",
            ),
        )

        self.assertEqual(command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime record", command[3])
        self.assertIn("--record-notes 'lighting changed'", command[3])
        self.assertIn("--record-classes 'person,fire extinguisher'", command[3])
        self.assertIn("--record-experiment-config 'Perception recording'", command[3])
        self.assertIn("--record-experiment-parameter postprocess.score_threshold=0.31", command[3])
        self.assertIn("--record-experiment-parameter postprocess.nms_iou_threshold=0.52", command[3])
        self.assertIn("--record-experiment-parameter postprocess.max_detections=42", command[3])
        self.assertIn("experiment_overwrite:=true", command[3])
        self.assertIn("classes_path:=/runs/operator_001/classes.txt", command[3])
        self.assertIn("postprocess_score_threshold:=0.31", command[3])
        self.assertIn("postprocess_nms_iou_threshold:=0.52", command[3])
        self.assertIn("postprocess_max_detections:=42", command[3])
        self.assertLess(command[3].index("--record-experiment-config"), command[3].index(" -- "))
        self.assertLess(command[3].index("--record-experiment-parameter"), command[3].index(" -- "))
        self.assertGreater(command[3].index("postprocess_score_threshold:=0.31"), command[3].index(" -- "))
        self.assertNotIn("start_autonomy:=true", command[3])

    def test_runtime_backend_adds_autonomy_launch_args_for_centering_run(self):
        command = build_remote_start_command(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_RUNTIME,
                classes=("backpack", "chair"),
                run_type=RUN_TYPE_AUTONOMY_CENTER,
                autonomy_bbox_area_min_ratio="0.10",
                autonomy_approach_stop_area_ratio="0.18",
                autonomy_bbox_area_max_ratio="0.28",
                autonomy_forward_speed_m_s="0.06",
                autonomy_reverse_speed_m_s="0.03",
                autonomy_stable_framed_frames="7",
                autonomy_proximity_stop_m="0.42",
                autonomy_capture_timeout_sec="3.5",
                autonomy_min_target_confidence="0.65",
                autonomy_max_target_center_jump_ratio="0.15",
                autonomy_evidence_interval_sec="0.20",
            ),
        )

        self.assertIn("start_autonomy:=true", command[3])
        self.assertIn("--record-classes backpack,chair", command[3])
        self.assertIn("--record-experiment-config 'Autonomy: frame and capture target'", command[3])
        self.assertIn("classes_path:=/runs/operator_001/classes.txt", command[3])
        self.assertIn("autonomy_target_class:=backpack", command[3])
        self.assertIn("autonomy_run_dir:=/runs/operator_001", command[3])
        self.assertIn("autonomy_bbox_area_min_ratio:=0.10", command[3])
        self.assertIn("autonomy_approach_stop_area_ratio:=0.18", command[3])
        self.assertIn("autonomy_bbox_area_max_ratio:=0.28", command[3])
        self.assertIn("autonomy_forward_speed_m_s:=0.06", command[3])
        self.assertIn("autonomy_reverse_speed_m_s:=0.03", command[3])
        self.assertIn("autonomy_stable_framed_frames:=7", command[3])
        self.assertIn("autonomy_proximity_stop_m:=0.42", command[3])
        self.assertIn("autonomy_capture_timeout_sec:=3.5", command[3])
        self.assertIn("autonomy_min_target_confidence:=0.65", command[3])
        self.assertIn("autonomy_max_target_center_jump_ratio:=0.15", command[3])
        self.assertIn("evidence_interval_sec:=0.20", command[3])

    def test_autonomy_run_requires_a_class(self):
        with self.assertRaisesRegex(ValueError, "requires at least one target class"):
            build_remote_start_command(
                connection=_connection(),
                run_config=RunConfig(
                    run_id="operator_001",
                    backend=RUN_BACKEND_RUNTIME,
                    run_type=RUN_TYPE_AUTONOMY_CENTER,
                ),
            )

    def test_devcontainer_backend_uses_container_visible_workspace_paths(self):
        command = build_remote_start_command(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_DEVCONTAINER,
                classes=("person", "fire extinguisher"),
                devcontainer_exec_template="docker exec omniseer-dev bash -lc {command}",
                detector_score_threshold="0.31",
                detector_nms_iou_threshold="0.52",
                detector_max_detections="42",
            ),
        )

        self.assertEqual(command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIn("docker exec omniseer-dev bash -lc", command[3])
        self.assertIn("cd /omniseer && scripts/omni run real --profile operator", command[3])
        self.assertIn("--record-out /omniseer/runs/operator_001", command[3])
        self.assertIn("--record-classes", command[3])
        self.assertIn("person,fire extinguisher", command[3])
        self.assertIn("--record-experiment-config", command[3])
        self.assertIn("Perception recording", command[3])
        self.assertIn("--record-experiment-parameter postprocess.score_threshold=0.31", command[3])
        self.assertIn("experiment_overwrite:=true", command[3])
        class_path = remote_class_list_path_for("/omniseer", "operator_001")
        self.assertIn(f"classes_path:={class_path}", command[3])
        self.assertIn("postprocess_score_threshold:=0.31", command[3])
        self.assertIn("postprocess_nms_iou_threshold:=0.52", command[3])
        self.assertIn("postprocess_max_detections:=42", command[3])
        self.assertLess(command[3].index("--record-experiment-config"), command[3].index(" bringup "))
        self.assertLess(command[3].index("--record-experiment-parameter"), command[3].index(" bringup "))
        self.assertGreater(command[3].index("postprocess_score_threshold:=0.31"), command[3].index(" bringup "))

    def test_devcontainer_backend_default_exec_template_uses_running_container_label(self):
        command = build_remote_start_command(
            connection=_connection(),
            run_config=RunConfig(
                run_id="operator_001",
                backend=RUN_BACKEND_DEVCONTAINER,
                classes=("person",),
                devcontainer_exec_template=DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
            ),
        )

        self.assertIn("docker ps --filter label=devcontainer.local_folder=/home/radxa/apps/omniseer", command[3])
        self.assertIn("--format '{{.Names}}'", command[3])
        self.assertIn('docker exec -it "$container" bash -lc', command[3])
        self.assertIn("cd /omniseer && scripts/omni run real --profile operator", command[3])
        self.assertIn("--record-out /omniseer/runs/operator_001", command[3])
        self.assertIn("experiment_overwrite:=true", command[3])
        self.assertIn("classes_path:=/omniseer/runs/operator_001/classes.txt", command[3])
        self.assertNotIn("devcontainer exec", command[3])

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

    def test_optional_recording_flags_are_opt_in_for_both_backends(self):
        for backend in (RUN_BACKEND_RUNTIME, RUN_BACKEND_DEVCONTAINER):
            default_command = build_remote_start_command(
                connection=_connection(), run_config=RunConfig(run_id="operator_001", backend=backend)
            )
            enabled_command = build_remote_start_command(
                connection=_connection(),
                run_config=RunConfig(run_id="operator_001", backend=backend, record_rosbag=True),
            )

            self.assertNotIn("--record-video", default_command[3])
            self.assertNotIn("--record-rosbag", default_command[3])
            self.assertIn("--record-rosbag", enabled_command[3])
