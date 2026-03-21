import sys
import unittest

from robot_diag_control.monitor_gui import (
    _build_parser,
    _build_preview_viewer_command,
    _resolved_preview_host,
)


class MonitorGuiTests(unittest.TestCase):
    def test_parser_defaults(self):
        parser = _build_parser()

        args = parser.parse_args([])

        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 50051)
        self.assertEqual(args.poll_interval_seconds, 1.0)
        self.assertFalse(args.refresh_on_start)

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
