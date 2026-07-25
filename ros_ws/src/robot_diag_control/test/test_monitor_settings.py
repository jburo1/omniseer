import unittest

from robot_diag_control.monitor_settings import (
    MonitorConnectionValues,
    TeleopStepValues,
    resolve_monitor_connection,
    resolve_teleop_steps,
)


def _values(**overrides: str) -> MonitorConnectionValues:
    values = {
        "host": " 10.0.0.2 ",
        "port": " 50070 ",
        "preview_host": " 10.0.0.3 ",
        "preview_port": " 7010 ",
        "preview_latency_ms": " 150 ",
        "gst_launch_path": " gst-launch-1.0 ",
        "poll_interval_seconds": " 0.5 ",
    }
    values.update(overrides)
    return MonitorConnectionValues(**values)


class MonitorSettingsTests(unittest.TestCase):
    def test_resolve_monitor_connection_trims_and_parses_values(self):
        settings = resolve_monitor_connection(_values())

        self.assertEqual(settings.host, "10.0.0.2")
        self.assertEqual(settings.port, 50070)
        self.assertEqual(settings.preview_host, "10.0.0.3")
        self.assertEqual(settings.preview_port, 7010)
        self.assertEqual(settings.preview_latency_ms, 150)
        self.assertEqual(settings.gst_launch_path, "gst-launch-1.0")
        self.assertEqual(settings.poll_interval_seconds, 0.5)

    def test_resolve_monitor_connection_keeps_blank_preview_host_as_none(self):
        settings = resolve_monitor_connection(_values(preview_host=" "))

        self.assertIsNone(settings.preview_host)

    def test_resolve_monitor_connection_rejects_invalid_numeric_value(self):
        with self.assertRaises(ValueError):
            resolve_monitor_connection(_values(port="not-a-port"))

    def test_resolve_teleop_steps_trims_and_parses_values(self):
        settings = resolve_teleop_steps(TeleopStepValues(linear_step_mps=" 0.12 ", angular_step_rad_s=" 0.35 "))

        self.assertEqual(settings.linear_step_mps, 0.12)
        self.assertEqual(settings.angular_step_rad_s, 0.35)

    def test_resolve_teleop_steps_rejects_invalid_numeric_value(self):
        with self.assertRaises(ValueError):
            resolve_teleop_steps(TeleopStepValues(linear_step_mps="fast", angular_step_rad_s="0.35"))
