import unittest
from unittest import mock

from robot_diag_control.preview_viewer import (
    _build_parser,
    _build_player_command,
    _select_h264_decoder_element,
    _validate_gstreamer_support,
)


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

    def test_build_player_command_accepts_decoder_override(self):
        parser = _build_parser()
        args = parser.parse_args([])

        command = _build_player_command(args, decoder_element="openh264dec")

        self.assertIn("openh264dec", command)

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

    @mock.patch("robot_diag_control.preview_viewer.subprocess.run")
    @mock.patch("robot_diag_control.preview_viewer._resolve_gst_inspect_path")
    def test_validate_gstreamer_support_rejects_missing_srtsrc(
        self,
        mock_resolve_gst_inspect_path,
        mock_run,
    ):
        parser = _build_parser()
        args = parser.parse_args([])
        mock_resolve_gst_inspect_path.return_value = "/usr/bin/gst-inspect-1.0"
        mock_run.return_value = mock.Mock(returncode=1)

        with self.assertRaisesRegex(RuntimeError, "srtsrc"):
            _validate_gstreamer_support(args)

    @mock.patch("robot_diag_control.preview_viewer._gst_element_available")
    def test_select_h264_decoder_element_prefers_first_available(self, mock_available):
        mock_available.side_effect = lambda _path, element: element == "openh264dec"

        decoder = _select_h264_decoder_element("/usr/bin/gst-inspect-1.0")

        self.assertEqual(decoder, "openh264dec")

    @mock.patch("robot_diag_control.preview_viewer._gst_element_available")
    @mock.patch("robot_diag_control.preview_viewer._resolve_gst_inspect_path")
    def test_validate_gstreamer_support_returns_available_decoder(
        self,
        mock_resolve_gst_inspect_path,
        mock_available,
    ):
        parser = _build_parser()
        args = parser.parse_args([])
        mock_resolve_gst_inspect_path.return_value = "/usr/bin/gst-inspect-1.0"
        mock_available.side_effect = lambda _path, element: element in {"srtsrc", "openh264dec"}

        decoder = _validate_gstreamer_support(args)

        self.assertEqual(decoder, "openh264dec")
