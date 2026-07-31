import unittest
from pathlib import Path

from robot_diag_control.run_commands import (
    RUN_BACKEND_DEVCONTAINER,
    RUN_BACKEND_LABELS,
    RUN_BACKEND_RUNTIME,
    RUN_TYPE_AUTONOMY_CENTER,
    RUN_TYPE_LABELS,
    RUN_TYPE_PERCEPTION,
)
from robot_diag_control.run_settings import (
    DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
    DEFAULT_LOCAL_IMPORT_ROOT,
    DEFAULT_REMOTE_REPO_ROOT,
    RunFormValues,
    default_remote_runs_root,
    local_import_root,
    normalized_remote_repo_root,
    normalized_remote_runs_root,
    resolve_run_form,
    resolved_run_id,
    selected_run_backend,
    selected_run_type,
)


def _values(**overrides: str) -> RunFormValues:
    values = {
        "robot_host": "10.0.0.2",
        "ssh_user": " radxa ",
        "remote_repo_root": "/home/radxa/apps/omniseer/",
        "remote_runs_root": "",
        "local_import_root": DEFAULT_LOCAL_IMPORT_ROOT,
        "run_id": " operator 001 ",
        "backend_label": RUN_BACKEND_LABELS[RUN_BACKEND_RUNTIME],
        "run_type_label": RUN_TYPE_LABELS[RUN_TYPE_PERCEPTION],
        "classes_text": "person, cup\nperson",
        "notes": "  trial notes  ",
        "devcontainer_exec_template": "",
        "autonomy_bbox_area_min_ratio": " 0.10 ",
        "autonomy_bbox_area_max_ratio": " 0.28 ",
        "autonomy_forward_speed_m_s": " 0.06 ",
        "autonomy_reverse_speed_m_s": " 0.03 ",
        "autonomy_stable_framed_frames": " 7 ",
        "autonomy_proximity_stop_m": " 0.42 ",
        "autonomy_capture_timeout_sec": " 3.5 ",
        "autonomy_evidence_interval_sec": " 0.20 ",
        "detector_score_threshold": " 0.31 ",
        "detector_nms_iou_threshold": " 0.52 ",
        "detector_max_detections": " 42 ",
    }
    values.update(overrides)
    return RunFormValues(**values)


class RunSettingsTests(unittest.TestCase):
    def test_selected_run_backend_maps_display_label_to_backend_id(self):
        self.assertEqual(selected_run_backend(RUN_BACKEND_LABELS[RUN_BACKEND_DEVCONTAINER]), RUN_BACKEND_DEVCONTAINER)

    def test_selected_run_backend_rejects_unknown_label(self):
        with self.assertRaisesRegex(ValueError, "unsupported run backend"):
            selected_run_backend("unknown backend")

    def test_selected_run_type_maps_display_label_to_run_type_id(self):
        self.assertEqual(selected_run_type(RUN_TYPE_LABELS[RUN_TYPE_AUTONOMY_CENTER]), RUN_TYPE_AUTONOMY_CENTER)

    def test_selected_run_type_rejects_unknown_label(self):
        with self.assertRaisesRegex(ValueError, "unsupported run type"):
            selected_run_type("unknown run type")

    def test_normalized_remote_paths_use_defaults_and_strip_trailing_slashes(self):
        self.assertEqual(normalized_remote_repo_root(""), DEFAULT_REMOTE_REPO_ROOT)
        self.assertEqual(normalized_remote_repo_root("/robot/repo/"), "/robot/repo")
        self.assertEqual(normalized_remote_runs_root("", remote_repo_root="/robot/repo"), "/robot/repo/runs")
        self.assertEqual(normalized_remote_runs_root("/robot/runs/", remote_repo_root="/robot/repo"), "/robot/runs")
        self.assertEqual(default_remote_runs_root("/robot/repo/"), "/robot/repo/runs")

    def test_local_import_root_resolves_relative_paths_against_repo_root(self):
        self.assertEqual(local_import_root("", repo_root=Path("/repo")), Path("/repo/runs/imported"))
        self.assertEqual(local_import_root("runs/custom", repo_root=Path("/repo")), Path("/repo/runs/custom"))
        self.assertEqual(local_import_root("/tmp/imported", repo_root=Path("/repo")), Path("/tmp/imported"))

    def test_resolved_run_id_sanitizes_or_uses_default(self):
        self.assertEqual(resolved_run_id(" operator 001 ", default_run_id=lambda: "default"), "operator_001")
        self.assertEqual(resolved_run_id("...", default_run_id=lambda: "default"), "default")

    def test_resolve_run_form_builds_domain_objects_from_form_values(self):
        selection = resolve_run_form(
            _values(),
            repo_root=Path("/repo"),
            default_run_id=lambda: "operator_default",
        )

        self.assertEqual(selection.run_id, "operator_001")
        self.assertEqual(selection.connection.host, "10.0.0.2")
        self.assertEqual(selection.connection.ssh_user, "radxa")
        self.assertEqual(selection.connection.remote_repo_root, "/home/radxa/apps/omniseer")
        self.assertEqual(selection.connection.remote_runs_root, "/home/radxa/apps/omniseer/runs")
        self.assertEqual(selection.run_config.run_id, "operator_001")
        self.assertEqual(selection.run_config.backend, RUN_BACKEND_RUNTIME)
        self.assertEqual(selection.run_config.classes, ("person", "cup"))
        self.assertEqual(selection.run_config.notes, "trial notes")
        self.assertEqual(selection.run_config.run_type, RUN_TYPE_PERCEPTION)
        self.assertEqual(selection.run_config.devcontainer_exec_template, DEFAULT_DEVCONTAINER_EXEC_TEMPLATE)
        self.assertEqual(selection.run_config.autonomy_bbox_area_min_ratio, "0.10")
        self.assertEqual(selection.run_config.autonomy_bbox_area_max_ratio, "0.28")
        self.assertEqual(selection.run_config.autonomy_forward_speed_m_s, "0.06")
        self.assertEqual(selection.run_config.autonomy_reverse_speed_m_s, "0.03")
        self.assertEqual(selection.run_config.autonomy_stable_framed_frames, "7")
        self.assertEqual(selection.run_config.autonomy_proximity_stop_m, "0.42")
        self.assertEqual(selection.run_config.autonomy_capture_timeout_sec, "3.5")
        self.assertEqual(selection.run_config.autonomy_evidence_interval_sec, "0.20")
        self.assertEqual(selection.run_config.detector_score_threshold, "0.31")
        self.assertEqual(selection.run_config.detector_nms_iou_threshold, "0.52")
        self.assertEqual(selection.run_config.detector_max_detections, "42")
        self.assertEqual(selection.artifact_context.repo_root, Path("/repo"))
        self.assertEqual(selection.artifact_context.connection, selection.connection)
        self.assertEqual(selection.artifact_context.local_import_root, Path("/repo/runs/imported"))

    def test_resolve_run_form_defaults_blank_detector_parameters(self):
        selection = resolve_run_form(
            _values(
                detector_score_threshold=" ",
                detector_nms_iou_threshold=" ",
                detector_max_detections=" ",
            ),
            repo_root=Path("/repo"),
            default_run_id=lambda: "operator_default",
        )

        self.assertEqual(selection.run_config.detector_score_threshold, "0.25")
        self.assertEqual(selection.run_config.detector_nms_iou_threshold, "0.45")
        self.assertEqual(selection.run_config.detector_max_detections, "100")

    def test_resolve_run_form_rejects_invalid_detector_thresholds(self):
        with self.assertRaisesRegex(ValueError, "score threshold must be a number from 0.0 to 1.0"):
            resolve_run_form(
                _values(detector_score_threshold="1.5"),
                repo_root=Path("/repo"),
                default_run_id=lambda: "operator_default",
            )

        with self.assertRaisesRegex(ValueError, "NMS IoU must be a number from 0.0 to 1.0"):
            resolve_run_form(
                _values(detector_nms_iou_threshold="nan"),
                repo_root=Path("/repo"),
                default_run_id=lambda: "operator_default",
            )

    def test_resolve_run_form_rejects_invalid_max_detections(self):
        with self.assertRaisesRegex(ValueError, "max detections must be a positive integer"):
            resolve_run_form(
                _values(detector_max_detections="0"),
                repo_root=Path("/repo"),
                default_run_id=lambda: "operator_default",
            )
