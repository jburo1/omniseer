import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import grpc
from robot_diag_control.monitor_gui import (
    DEFAULT_ROBOT_HOST,
    DEFAULT_ROBOT_USER,
    RUN_BACKEND_RUNTIME,
    RUN_TYPE_AUTONOMY_CENTER,
    RUN_TYPE_LABELS,
    RobotMonitorGui,
    _build_overlay_viewer_command,
    _build_parser,
    _build_preview_viewer_command,
    _format_request_error,
    _resolved_preview_host,
    _teleop_command_for_action,
)
from robot_diag_control.run_lifecycle import RunPhase

try:
    import tkinter as tk
except ModuleNotFoundError:  # pragma: no cover - depends on host Tk install
    tk = None  # type: ignore[assignment]


class _FakeRpcError(grpc.RpcError):
    def code(self):
        return grpc.StatusCode.UNAVAILABLE

    def details(self):
        return "failed to connect to all addresses"


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

    def test_format_request_error_compacts_rpc_error(self):
        message = _format_request_error("status refresh", _FakeRpcError())

        self.assertEqual(message, "status refresh failed: UNAVAILABLE; failed to connect to all addresses")
        self.assertNotIn("_InactiveRpcError", message)
        self.assertNotIn("debug_error_string", message)

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_run_form_selection_builds_run_domain_objects(self):
        assert tk is not None
        with tempfile.TemporaryDirectory() as repo_root:
            root = tk.Tk()
            root.withdraw()
            try:
                args = _build_parser().parse_args(
                    [
                        "--host",
                        "10.0.0.2",
                        "--repo-root",
                        repo_root,
                        "--remote-repo-root",
                        "/robot/repo/",
                        "--local-import-root",
                        "runs/custom",
                    ]
                )
                gui = RobotMonitorGui(root, args)
                gui._ssh_user_var.set(" radxa ")
                gui._run_type_var.set(RUN_TYPE_LABELS[RUN_TYPE_AUTONOMY_CENTER])
                gui._run_id_var.set(" demo run/01 ")
                gui._run_classes_var.set("person, cup\nperson")
                gui._run_notes_text.delete("1.0", tk.END)
                gui._run_notes_text.insert("1.0", "  trial notes  ")

                selection = gui._run_form_selection()

                self.assertEqual(selection.run_id, "demo_run_01")
                self.assertEqual(selection.connection.host, "10.0.0.2")
                self.assertEqual(selection.connection.ssh_user, "radxa")
                self.assertEqual(selection.connection.remote_repo_root, "/robot/repo")
                self.assertEqual(selection.connection.remote_runs_root, "/robot/repo/runs")
                self.assertEqual(selection.run_config.classes, ("person", "cup"))
                self.assertEqual(selection.run_config.run_type, RUN_TYPE_AUTONOMY_CENTER)
                self.assertEqual(selection.run_config.notes, "trial notes")
                self.assertEqual(selection.artifact_context.repo_root, Path(repo_root).resolve())
                self.assertEqual(
                    selection.artifact_context.local_import_root,
                    Path(repo_root).resolve() / "runs/custom",
                )
            finally:
                root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_run_form_selection_writes_default_run_id_back_to_entry(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))
            gui._run_id_var.set("...")

            with patch("robot_diag_control.monitor_gui._default_run_id", return_value="operator_default"):
                selection = gui._run_form_selection()

            self.assertEqual(selection.run_id, "operator_default")
            self.assertEqual(gui._run_id_var.get(), "operator_default")
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_connection_settings_resolve_from_tk_variables(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))
            gui._host_var.set(" 10.0.0.2 ")
            gui._port_var.set(" 50070 ")
            gui._preview_host_var.set(" ")
            gui._preview_port_var.set(" 7010 ")
            gui._preview_latency_var.set(" 150 ")
            gui._gst_launch_path_var.set(" gst-launch-1.0 ")
            gui._poll_interval_var.set(" 0.5 ")

            settings = gui._connection_settings()

            self.assertEqual(settings.host, "10.0.0.2")
            self.assertEqual(settings.port, 50070)
            self.assertIsNone(settings.preview_host)
            self.assertEqual(settings.preview_port, 7010)
            self.assertEqual(settings.preview_latency_ms, 150)
            self.assertEqual(settings.gst_launch_path, "gst-launch-1.0")
            self.assertEqual(settings.poll_interval_seconds, 0.5)
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_teleop_step_settings_resolve_from_tk_variables(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))
            gui._teleop_linear_step_var.set(" 0.2 ")
            gui._teleop_angular_step_var.set(" 0.45 ")

            settings = gui._teleop_step_settings()

            self.assertEqual(settings.linear_step_mps, 0.2)
            self.assertEqual(settings.angular_step_rad_s, 0.45)
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_sections_are_collapsible(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))
            self.assertEqual(set(gui._sections), {"connection", "actions", "teleop", "run", "status", "log"})

            section = gui._sections["run"]
            root.update_idletasks()
            self.assertTrue(section.expanded)
            self.assertEqual(section.body.winfo_manager(), "pack")

            section.toggle()
            root.update_idletasks()
            self.assertFalse(section.expanded)
            self.assertEqual(section.body.winfo_manager(), "")

            section.toggle()
            root.update_idletasks()
            self.assertTrue(section.expanded)
            self.assertEqual(section.body.winfo_manager(), "pack")
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_run_actions_do_not_include_standalone_generate_report(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))

            self.assertEqual(set(gui._run_buttons), {"new_id", "start", "stop", "retrieve", "open_report"})
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_header_mode_uses_run_state_not_last_log_message(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))

            self.assertEqual(gui._mode_var.get(), "idle")
            gui._append_log("status refresh failed: UNAVAILABLE")
            self.assertEqual(gui._mode_var.get(), "idle")

            gui._set_run_state(RunPhase.RUNNING, run_id="operator_001")
            self.assertEqual(gui._mode_var.get(), "running operator_001")

            gui._append_log("status refresh failed: UNAVAILABLE")
            self.assertEqual(gui._mode_var.get(), "running operator_001")
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_user_actions_write_clear_activity_messages(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))

            gui.stop_watch()
            gui.new_run_id()

            activity = gui._log_text.get("1.0", tk.END)
            self.assertIn("stop watch requested", activity)
            self.assertIn("watch not running", activity)
            self.assertIn("new run id generated", activity)
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_teleop_keys_are_silent_while_teleop_is_disabled(self):
        assert tk is not None
        root = tk.Tk()
        root.withdraw()
        try:
            gui = RobotMonitorGui(root, _build_parser().parse_args([]))

            gui.send_teleop_action("forward")
            gui.send_teleop_action("right")

            activity = gui._log_text.get("1.0", tk.END)
            self.assertEqual(activity.strip(), "")
            self.assertNotIn("teleop forward requested", activity)
            self.assertNotIn("teleop command failed", activity)

            gui._update_teleop_enabled(True)
            gui._update_teleop_enabled(False)
            gui.send_teleop_action("left")

            activity = gui._log_text.get("1.0", tk.END)
            self.assertEqual(activity.strip(), "")
        finally:
            root.destroy()

    @unittest.skipIf(tk is None, "tkinter is unavailable")
    def test_gui_open_report_logs_artifact_warnings_for_incomplete_bundle(self):
        assert tk is not None
        with tempfile.TemporaryDirectory() as repo_root:
            run_dir = Path(repo_root) / "runs" / "imported" / "operator_001"
            run_dir.mkdir(parents=True)
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
            report = run_dir / "report" / "index.html"
            report.parent.mkdir(parents=True)
            report.write_text("<!doctype html>\n", encoding="utf-8")

            root = tk.Tk()
            root.withdraw()
            try:
                args = _build_parser().parse_args(["--repo-root", repo_root])
                gui = RobotMonitorGui(root, args)
                gui._run_id_var.set("operator_001")

                with patch("robot_diag_control.monitor_gui.webbrowser.open") as browser_open:
                    gui.open_report()

                activity = gui._log_text.get("1.0", tk.END)
                self.assertIn("open report requested", activity)
                self.assertIn("artifact warning: run_open: manifest ended_at is null; run did not finalize", activity)
                self.assertIn("artifact warning: missing_summary: summary.json is missing", activity)
                browser_open.assert_called_once_with(report.as_uri())
            finally:
                root.destroy()
