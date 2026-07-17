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

        teleop_args = parser.parse_args(["teleop", "cmd", "--linear-x-mps", "0.1"])
        self.assertEqual(teleop_args.command, "teleop")
        self.assertEqual(teleop_args.mode, "cmd")
        self.assertEqual(teleop_args.linear_x_mps, 0.1)
