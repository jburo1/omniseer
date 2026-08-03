import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class BuildScriptYoloTests(unittest.TestCase):
    def test_with_yolo_builds_with_virtualenv_python(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            venv_bin = tmp_path / "venv" / "bin"
            setup_path = tmp_path / "setup.bash"
            colcon_args_path = tmp_path / "colcon.args"
            bin_dir.mkdir()
            venv_bin.mkdir(parents=True)
            setup_path.write_text("# test setup\n", encoding="utf-8")

            colcon_stub = bin_dir / "colcon"
            colcon_stub.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            colcon_stub.chmod(0o755)

            venv_python = venv_bin / "python"
            venv_python.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "-m" && "$2" == "colcon" && "$3" == "--help" ]]; then exit 0; fi',
                        'if [[ "$1" == "-c" ]]; then exit 0; fi',
                        'if [[ "$1" == "-m" && "$2" == "colcon" ]]; then',
                        "  shift 2",
                        f'  printf "%s\\n" "$@" >"{colcon_args_path}"',
                        "  exit 0",
                        "fi",
                        "exit 1",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            venv_python.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["VIRTUAL_ENV"] = str(tmp_path / "venv")

            subprocess.run(
                [str(repo_root / "scripts" / "omni"), "build", "ros", "--with-yolo"],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = colcon_args_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(args[:2], ["build", "--merge-install"])
            self.assertIn("yolo_ros", args)
            self.assertIn("yolo_bringup", args)
            self.assertIn("--cmake-args", args)
            self.assertIn(f"-DPython3_EXECUTABLE={venv_python}", args)
            self.assertIn("-DPython3_FIND_VIRTUALENV=ONLY", args)

    def test_with_yolo_rejects_build_python_without_yolo_dependencies(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            venv_bin = tmp_path / "venv" / "bin"
            setup_path = tmp_path / "setup.bash"
            bin_dir.mkdir()
            venv_bin.mkdir(parents=True)
            setup_path.write_text("# test setup\n", encoding="utf-8")

            colcon_stub = bin_dir / "colcon"
            colcon_stub.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            colcon_stub.chmod(0o755)

            venv_python = venv_bin / "python"
            venv_python.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "-m" && "$2" == "colcon" && "$3" == "--help" ]]; then exit 0; fi',
                        'if [[ "$1" == "-c" ]]; then exit 1; fi',
                        "exit 1",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            venv_python.chmod(0o755)

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_ROS_SETUP"] = str(setup_path)
            env["VIRTUAL_ENV"] = str(tmp_path / "venv")

            result = subprocess.run(
                [str(repo_root / "scripts" / "omni"), "build", "ros", "--with-yolo"],
                cwd=repo_root,
                env=env,
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("YOLO sim support requires Python 'torch' and 'ultralytics'", result.stderr)


if __name__ == "__main__":
    unittest.main()
