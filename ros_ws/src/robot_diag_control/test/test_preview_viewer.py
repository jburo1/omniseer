import unittest

from robot_diag_control.preview_viewer import _build_parser, _build_player_command


class PreviewViewerTests(unittest.TestCase):
    def test_parser_defaults_to_display_mode(self):
        parser = _build_parser()

        args = parser.parse_args([])

        self.assertEqual(args.mode, "display")
        self.assertEqual(args.profile, "balanced")
        self.assertEqual(args.preview_port, 7001)

    def test_build_player_command_uses_preview_host_override(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--host",
                "10.0.0.2",
                "--preview-host",
                "10.0.0.3",
                "--mode",
                "fakesink",
            ]
        )

        command = _build_player_command(args)

        self.assertEqual(command[0], "gst-launch-1.0")
        self.assertIn("uri=srt://10.0.0.3:7001?mode=caller", command)
        self.assertIn("fakesink", command)

    def test_capture_ts_requires_output_path(self):
        parser = _build_parser()
        args = parser.parse_args(["--mode", "capture_ts"])

        with self.assertRaisesRegex(ValueError, "output-path"):
            _build_player_command(args)

    def test_capture_ts_builds_filesink_pipeline(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--mode",
                "capture_ts",
                "--output-path",
                "/tmp/preview.ts",
            ]
        )

        command = _build_player_command(args)

        self.assertIn("filesink", command)
        self.assertIn("location=/tmp/preview.ts", command)
