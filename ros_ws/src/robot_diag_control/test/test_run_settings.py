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
        self.assertEqual(selection.artifact_context.repo_root, Path("/repo"))
        self.assertEqual(selection.artifact_context.connection, selection.connection)
        self.assertEqual(selection.artifact_context.local_import_root, Path("/repo/runs/imported"))
