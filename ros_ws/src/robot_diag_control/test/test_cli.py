import unittest

from robot_diag_control.cli import _build_parser


class CliTests(unittest.TestCase):
    def test_parser_exposes_status_and_preview_commands(self):
        parser = _build_parser()

        status_args = parser.parse_args(["status"])
        self.assertEqual(status_args.command, "status")

        preview_args = parser.parse_args(["preview", "on", "--profile", "low_bw"])
        self.assertEqual(preview_args.command, "preview")
        self.assertEqual(preview_args.mode, "on")
        self.assertEqual(preview_args.profile, "low_bw")
