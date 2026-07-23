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
            run_dir = pathlib.Path(tmp) / "demo_001"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    str(run_dir),
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
            run_dir_exists = run_dir.is_dir()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("start_gateway:=true", result.stdout)
        self.assertIn("start_experiment_recording:=true", result.stdout)
        self.assertIn("experiment_run_id:=demo_001", result.stdout)
        self.assertIn(f"pipeline_telemetry_path:={run_dir}/pipeline_telemetry.jsonl", result.stdout)
        self.assertIn(f"evidence_dir:={run_dir}/evidence", result.stdout)
        self.assertIn("experiment_launch_profile:=operator", result.stdout)
        self.assertIn("experiment_launch_mode:=bringup", result.stdout)
        self.assertIn("experiment_launch_command:=scripts/omni\\ run\\ real", result.stdout)
        self.assertIn("experiment_launch_args:=start_nav:=false", result.stdout)
        self.assertIn("start_vision:=false", result.stdout)
        self.assertNotIn("experiment_notes:=", result.stdout)
        self.assertNotIn("experiment_classes:=", result.stdout)
        self.assertTrue(run_dir_exists)

    def test_record_run_flags_map_to_real_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = pathlib.Path(tmp) / "demo_001"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--profile",
                    "perception",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    str(run_dir),
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
        self.assertIn("start_gateway:=false", result.stdout)
        self.assertIn("start_experiment_recording:=true", result.stdout)
        self.assertIn("experiment_run_id:=demo_001", result.stdout)
        self.assertIn(f"experiment_out_dir:={run_dir}", result.stdout)
        self.assertIn(f"pipeline_telemetry_path:={run_dir}/pipeline_telemetry.jsonl", result.stdout)
        self.assertIn(f"evidence_dir:={run_dir}/evidence", result.stdout)
        self.assertIn("experiment_duration_sec:=5", result.stdout)
        self.assertIn("experiment_notes:=note\\ text", result.stdout)
        self.assertIn("experiment_classes:=chair\\ backpack", result.stdout)
        self.assertIn("experiment_launch_profile:=perception", result.stdout)
        self.assertIn("experiment_launch_mode:=bringup", result.stdout)

    def test_record_run_provenance_flags_map_to_real_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = pathlib.Path(tmp) / "demo_001"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--profile",
                    "perception",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    str(run_dir),
                    "--record-container-image-ref",
                    "ghcr.io/acme/omniseer:robot-v2",
                    "--record-container-image-digest",
                    "sha256:0123456789abcdef",
                    "--record-experiment-config",
                    "experiments/container-smoke.yaml",
                    "--record-experiment-parameter",
                    "profile=perception",
                    "--record-experiment-parameter",
                    "camera=/dev/video11",
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
        self.assertIn("experiment_container_image_ref:=ghcr.io/acme/omniseer:robot-v2", result.stdout)
        self.assertIn("experiment_container_image_digest:=sha256:0123456789abcdef", result.stdout)
        self.assertIn("experiment_config:=experiments/container-smoke.yaml", result.stdout)
        self.assertIn("experiment_parameters:=profile=perception\\,camera=/dev/video11", result.stdout)

    def test_record_run_uses_provenance_env_fallbacks(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = pathlib.Path(tmp) / "demo_001"

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)
            env["OMNISEER_CONTAINER_IMAGE_REF"] = "ghcr.io/acme/omniseer:env"
            env["OMNISEER_CONTAINER_IMAGE_DIGEST"] = "sha256:envdigest"
            env["OMNISEER_EXPERIMENT_CONFIG"] = "experiments/env.yaml"
            env["OMNISEER_EXPERIMENT_PARAMETERS"] = "profile=operator"

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    str(run_dir),
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
        self.assertIn("experiment_container_image_ref:=ghcr.io/acme/omniseer:env", result.stdout)
        self.assertIn("experiment_container_image_digest:=sha256:envdigest", result.stdout)
        self.assertIn("experiment_config:=experiments/env.yaml", result.stdout)
        self.assertIn("experiment_parameters:=profile=operator", result.stdout)

    def test_record_overwrite_prepares_clean_directory_before_launch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            setup_file = pathlib.Path(tmp) / "setup.bash"
            setup_file.write_text("# test setup shim\n", encoding="utf-8")
            fake_ros2 = pathlib.Path(tmp) / "ros2"
            fake_ros2.write_text("#!/usr/bin/env bash\nprintf '%q\\n' \"$@\"\n", encoding="utf-8")
            fake_ros2.chmod(0o755)
            run_dir = pathlib.Path(tmp) / "demo_001"
            run_dir.mkdir()
            (run_dir / "stale.jsonl").write_text("{}\n", encoding="utf-8")

            env = os.environ.copy()
            env["PATH"] = f"{tmp}:{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_file)
            env["OMNISEER_WS_SETUP"] = str(setup_file)

            result = subprocess.run(
                [
                    "scripts/run/real.sh",
                    "--record-run",
                    "demo_001",
                    "--record-out",
                    str(run_dir),
                    "--record-overwrite",
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
            run_dir_exists = run_dir.is_dir()
            stale_exists = (run_dir / "stale.jsonl").exists()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(run_dir_exists)
        self.assertFalse(stale_exists)
        self.assertIn("experiment_overwrite:=false", result.stdout)

    def test_record_flags_rejected_for_verify_mode(self) -> None:
        result = subprocess.run(
            ["scripts/run/real.sh", "--record-run", "demo_001", "verify"],
            cwd=_repo_root(),
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("recording flags require a mode that launches real bringup", result.stderr)
