import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class RealScriptRecordingTests(unittest.TestCase):
    def test_autonomy_script_uses_first_class_as_target_and_records_full_class_list(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            run_dir = tmp_path / "runs" / "autonomy_chair_001"
            ros_args_path = tmp_path / "ros2.args"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")
            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 1; fi',
                        'printf "%s\\n" "fake launch output"',
                        f'printf "%s\\n" "$@" >"{ros_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            ros2_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["OMNISEER_WS_SETUP"] = str(setup_path)

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "run",
                    "autonomy",
                    "--classes",
                    "chair, backpack, bottle",
                    "--run-id",
                    "autonomy_chair_001",
                    "--out",
                    str(run_dir),
                    "--",
                    "start_vision:=false",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = ros_args_path.read_text(encoding="utf-8").splitlines()
            self.assertIn(f"classes_path:={run_dir}/classes.txt", args)
            self.assertIn("autonomy_target_class:=chair", args)
            self.assertIn(f"autonomy_run_dir:={run_dir}", args)
            self.assertIn("start_vision:=false", args)
            self.assertEqual((run_dir / "classes.txt").read_text(encoding="utf-8"), "chair\nbackpack\nbottle\n")

    def test_record_overwrite_reaches_launch_after_run_dir_preparation(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            run_dir = tmp_path / "runs" / "autonomy_v1"
            ros_args_path = tmp_path / "ros2.args"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")
            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 1; fi',
                        'printf "%s\\n" "fake launch output"',
                        f'printf "%s\\n" "$@" >"{ros_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            ros2_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["OMNISEER_WS_SETUP"] = str(setup_path)

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "run",
                    "real",
                    "--profile",
                    "operator",
                    "--record-run",
                    "autonomy_v1",
                    "--record-out",
                    str(run_dir),
                    "--record-classes",
                    "plant",
                    "--record-overwrite",
                    "bringup",
                    "start_vision:=false",
                    "start_autonomy:=true",
                    "autonomy_target_class:=plant",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = ros_args_path.read_text(encoding="utf-8").splitlines()
            self.assertIn("experiment_overwrite:=true", args)
            self.assertEqual((run_dir / "classes.txt").read_text(encoding="utf-8"), "plant\n")
            self.assertEqual((run_dir / "logs" / "bringup.log").read_text(encoding="utf-8"), "fake launch output\n")

    def test_record_without_overwrite_preserves_existing_launch_default(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            run_dir = tmp_path / "runs" / "operator_001"
            ros_args_path = tmp_path / "ros2.args"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")
            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 1; fi',
                        'printf "%s\\n" "fake launch output"',
                        f'printf "%s\\n" "$@" >"{ros_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            ros2_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["OMNISEER_WS_SETUP"] = str(setup_path)

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "run",
                    "real",
                    "--profile",
                    "operator",
                    "--record-run",
                    "operator_001",
                    "--record-out",
                    str(run_dir),
                    "bringup",
                    "start_vision:=false",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = ros_args_path.read_text(encoding="utf-8").splitlines()
            self.assertIn("experiment_overwrite:=false", args)
            self.assertEqual((run_dir / "logs" / "bringup.log").read_text(encoding="utf-8"), "fake launch output\n")


if __name__ == "__main__":
    unittest.main()
