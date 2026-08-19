import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class RuntimeScriptRecordingTests(unittest.TestCase):
    def test_runtime_record_forwards_experiment_overwrite_launch_arg(self) -> None:
        repo_root = Path(__file__).resolve().parents[4]

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bin_dir = tmp_path / "bin"
            run_root = tmp_path / "runs"
            docker_args_path = tmp_path / "docker.args"
            bin_dir.mkdir()
            docker_stub = bin_dir / "docker"
            docker_stub.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env bash",
                        'if [[ "$1" == "image" && "$2" == "inspect" ]]; then',
                        '  printf "%s\\n" "sha256:test"',
                        "  exit 0",
                        "fi",
                        f'printf "%s\\n" "$@" >"{docker_args_path}"',
                        "exit 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            docker_stub.chmod(0o755)

            latest_dir = tmp_path / "runtime"
            latest_dir.mkdir()
            (latest_dir / "latest-build.env").write_text(
                "\n".join(
                    [
                        "IMAGE_BASE=ghcr.io/test/omniseer",
                        "TAG=robot-test",
                        "IMAGE_REF=ghcr.io/test/omniseer:robot-test",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            env = os.environ.copy()
            env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
            env["OMNISEER_RUNTIME_METADATA_DIR"] = str(latest_dir)
            env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(run_root)
            env["OMNISEER_RUNTIME_RUNS_LOCAL_ROOT"] = str(run_root)
            env["OMNISEER_RUNTIME_DOCKER_TTY"] = "never"

            subprocess.run(
                [
                    str(repo_root / "scripts" / "omni"),
                    "runtime",
                    "record",
                    "--run-id",
                    "autonomy_v2",
                    "--record-classes",
                    "plant",
                    "--",
                    "classes_path:=/runs/autonomy_v2/classes.txt",
                    "start_autonomy:=true",
                ],
                cwd=repo_root,
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )

            args = docker_args_path.read_text(encoding="utf-8").splitlines()
            self.assertIn("run", args)
            self.assertIn("--record-overwrite", args)
            self.assertIn("--record-runtime-backend", args)
            self.assertIn("robot_runtime_container", args)
            self.assertIn("--record-container-image-ref", args)
            self.assertIn("ghcr.io/jburo1/omniseer-robot-runtime:robot-test", args)
            self.assertIn("--record-container-image-digest", args)
            self.assertIn("--record-container-image-id", args)
            self.assertIn("sha256:test", args)
            self.assertIn("classes_path:=/runs/autonomy_v2/classes.txt", args)
            self.assertEqual((run_root / "autonomy_v2" / "classes.txt").read_text(encoding="utf-8"), "plant\n")


if __name__ == "__main__":
    unittest.main()
