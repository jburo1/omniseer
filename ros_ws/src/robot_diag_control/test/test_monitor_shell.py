import unittest

from robot_diag_control.monitor_shell import _build_parser, _build_preview_viewer_args


class MonitorShellTests(unittest.TestCase):
    def test_parser_defaults(self):
        parser = _build_parser()

        args = parser.parse_args([])

        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 50051)
        self.assertEqual(args.poll_interval_seconds, 1.0)

    def test_preview_viewer_args_inherit_shell_connection_defaults(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--host",
                "10.0.0.2",
                "--port",
                "50070",
                "--preview-port",
                "7009",
                "--preview-latency-ms",
                "150",
            ]
        )

        viewer_args = _build_preview_viewer_args(
            args,
            ["--mode", "capture_ts", "--output-path", "/tmp/preview.ts"],
        )

        self.assertEqual(
            viewer_args[:10],
            [
                "--host",
                "10.0.0.2",
                "--port",
                "50070",
                "--preview-host",
                "10.0.0.2",
                "--preview-port",
                "7009",
                "--preview-latency-ms",
                "150",
            ],
        )
        self.assertIn("--mode", viewer_args)
        self.assertIn("/tmp/preview.ts", viewer_args)
