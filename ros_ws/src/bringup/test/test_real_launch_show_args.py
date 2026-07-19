import pathlib
import shutil
import subprocess
import unittest


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[4]


def _launch_file(name: str) -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1] / "launch" / name


class RealLaunchShowArgsTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("ros2"), "ros2 is required for launch argument checks")
    def test_real_launch_show_args_lists_teensy_gating_arguments(self) -> None:
        result = subprocess.run(
            ["ros2", "launch", str(_launch_file("real.launch.py")), "--show-args"],
            cwd=_repo_root(),
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("pre_launch_cleanup", result.stdout)
        self.assertIn("micro_ros_serial_device", result.stdout)
        self.assertIn(
            "/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00",
            result.stdout,
        )
        self.assertIn("require_teensy", result.stdout)
        self.assertIn("teensy_preflight_timeout_sec", result.stdout)
        self.assertIn("boundary_topics_timeout_sec", result.stdout)
        self.assertIn("allow_teensy_power_cycle", result.stdout)
        self.assertIn("start_experiment_recording", result.stdout)
        self.assertIn("experiment_run_id", result.stdout)
        self.assertIn("experiment_out_dir", result.stdout)
        self.assertIn("experiment_duration_sec", result.stdout)

    @unittest.skipUnless(shutil.which("ros2"), "ros2 is required for launch argument checks")
    def test_real_io_launch_show_args_lists_teensy_gating_arguments(self) -> None:
        result = subprocess.run(
            ["ros2", "launch", str(_launch_file("real_io.launch.py")), "--show-args"],
            cwd=_repo_root(),
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("micro_ros_serial_device", result.stdout)
        self.assertIn(
            "/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00",
            result.stdout,
        )
        self.assertIn("require_teensy", result.stdout)
        self.assertIn("teensy_preflight_timeout_sec", result.stdout)
        self.assertIn("allow_teensy_power_cycle", result.stdout)
