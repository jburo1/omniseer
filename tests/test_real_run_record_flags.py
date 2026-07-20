import os
import pathlib
import subprocess
import tempfile
import unittest


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


class RealRunRecordFlagsTests(unittest.TestCase):
    def test_record_run_omits_empty_optional_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--phase",
                    "3",
                    "--record-run",
                    "demo_001",
                    "bringup",
                    "start_vision:=false",
                ],
                cwd=_repo_root(),
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("start_experiment_recording:=true", result.stdout)
        self.assertIn("experiment_run_id:=demo_001", result.stdout)
        self.assertIn("pipeline_telemetry_path:=runs/demo_001/pipeline_telemetry.jsonl", result.stdout)
        self.assertNotIn("experiment_notes:=", result.stdout)
        self.assertNotIn("experiment_classes:=", result.stdout)

    def test_record_run_flags_map_to_real_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--phase",
                    "3",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    "/tmp/demo_001",
                    "--record-duration-sec",
                    "5",
                    "--record-notes",
                    "note text",
                    "--record-classes",
                    "chair backpack",
                    "bringup",
                    "start_vision:=false",
                ],
                cwd=_repo_root(),
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("start_experiment_recording:=true", result.stdout)
        self.assertIn("experiment_run_id:=demo_001", result.stdout)
        self.assertIn("experiment_out_dir:=/tmp/demo_001", result.stdout)
        self.assertIn("pipeline_telemetry_path:=/tmp/demo_001/pipeline_telemetry.jsonl", result.stdout)
        self.assertIn("experiment_duration_sec:=5", result.stdout)
        self.assertIn("experiment_notes:=note\\ text", result.stdout)
        self.assertIn("experiment_classes:=chair\\ backpack", result.stdout)

    def test_record_flags_rejected_for_verify_mode(self) -> None:
        result = subprocess.run(
            ["scripts/run/real.sh", "--phase", "3", "--record-run", "demo_001", "verify"],
            cwd=_repo_root(),
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("recording flags require a mode that launches real bringup", result.stderr)
