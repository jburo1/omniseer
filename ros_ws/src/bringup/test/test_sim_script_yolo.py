import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class SimScriptYoloTests(unittest.TestCase):
    def test_autonomy_flag_starts_visual_autonomy_and_yolo(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            ros_args_path = tmp_path / "ros2.args"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")

            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 0; fi',
                        f'printf "%s\\n" "$@" >"{ros_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            ros2_stub.chmod(0o755)

            python_stub = bin_dir / "python3"
            python_stub.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            python_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["OMNISEER_WS_SETUP"] = str(setup_path)

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "run",
                    "sim",
                    "--autonomy",
                    "--yolo-device",
                    "cpu",
                    "headless:=true",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = ros_args_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(args[:3], ["launch", "bringup", "sim.launch.py"])
            self.assertIn("headless:=true", args)
            self.assertIn("start_autonomy:=true", args)
            self.assertIn("start_yolo:=true", args)
            self.assertIn("yolo_device:=cpu", args)

    def test_yolo_flag_forwards_sim_launch_arguments(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            ros_args_path = tmp_path / "ros2.args"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")

            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 0; fi',
                        f'printf "%s\\n" "$@" >"{ros_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            ros2_stub.chmod(0o755)

            python_stub = bin_dir / "python3"
            python_stub.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            python_stub.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["OMNISEER_WS_SETUP"] = str(setup_path)

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "run",
                    "sim",
                    "--yolo",
                    "--yolo-device",
                    "cpu",
                    "headless:=true",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = ros_args_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(args[:3], ["launch", "bringup", "sim.launch.py"])
            self.assertIn("headless:=true", args)
            self.assertIn("start_yolo:=true", args)
            self.assertIn("yolo_device:=cpu", args)
            self.assertIn(f"yolo_model:={repo_root / 'ros_ws' / 'yolov8s-worldv2.pt'}", args)
            self.assertIn("yolo_image_reliability:=2", args)

    def test_yolo_flag_reports_missing_yolo_packages(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            setup_path.write_text("# test setup\n", encoding="utf-8")

            ros2_stub = bin_dir / "ros2"
            ros2_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then exit 1; fi',
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

            result = subprocess.run(
                [str(repo_root / "scripts" / "omni"), "run", "sim", "--yolo"],
                cwd=repo_root,
                env=env,
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("scripts/omni build ros --with-yolo", result.stderr)


if __name__ == "__main__":
    unittest.main()
