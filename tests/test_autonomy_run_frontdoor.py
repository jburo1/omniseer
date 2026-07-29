import os
import pathlib
import subprocess
import tempfile
import unittest


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


class AutonomyRunFrontdoorTests(unittest.TestCase):
    def test_target_maps_to_real_autonomy_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            setup_file = tmp_path / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = tmp_path / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = tmp_path / "chair_run"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/omni",
                    "run",
                    "autonomy",
                    "--target",
                    "chair",
                    "--run-id",
                    "chair_run",
                    "--out",
                    str(run_dir),
                    "--notes",
                    "frontdoor test",
                    "--",
                    "start_vision:=false",
                ],
                cwd=_repo_root(),
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=30.0,
            )
            classes_text = (run_dir / "classes.txt").read_text(encoding="utf-8")

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(classes_text, "chair\n")
        self.assertIn("start_gateway:=true", result.stdout)
        self.assertIn("start_experiment_recording:=true", result.stdout)
        self.assertIn("experiment_overwrite:=true", result.stdout)
        self.assertIn("experiment_run_id:=chair_run", result.stdout)
        self.assertIn(f"experiment_out_dir:={run_dir}", result.stdout)
        self.assertIn("experiment_classes:=chair", result.stdout)
        self.assertIn("experiment_notes:=frontdoor\\ test", result.stdout)
        self.assertIn(f"classes_path:={run_dir}/classes.txt", result.stdout)
        self.assertIn("start_autonomy:=true", result.stdout)
        self.assertIn("autonomy_target_class:=chair", result.stdout)
        self.assertIn(f"autonomy_run_dir:={run_dir}", result.stdout)
        self.assertIn("evidence_interval_sec:=0.25", result.stdout)
        self.assertIn("start_vision:=false", result.stdout)

    def test_positional_target_is_supported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = pathlib.Path(tmp)
            setup_file = tmp_path / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = tmp_path / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = tmp_path / "backpack_run"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/omni",
                    "run",
                    "autonomy",
                    "backpack",
                    "--run-id",
                    "backpack_run",
                    "--out",
                    str(run_dir),
                    "--",
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
        self.assertIn("autonomy_target_class:=backpack", result.stdout)


if __name__ == "__main__":
    unittest.main()
