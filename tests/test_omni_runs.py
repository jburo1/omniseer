import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_fake_setup(path: Path) -> None:
    path.write_text("# test setup shim\n", encoding="utf-8")


def _write_fake_retrieve_runs(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "printf 'retrieve_runs'",
                "for arg in \"$@\"; do",
                "  printf ' %s' \"$arg\"",
                "done",
                "printf '\\n'",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def test_omni_runs_list_dispatches_to_retrieve_runs(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    retrieve_runs = tmp_path / "retrieve_runs"
    _write_fake_setup(setup_file)
    _write_fake_retrieve_runs(retrieve_runs)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "list", "--host", "robot.local"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "retrieve_runs list --host robot.local\n"


def test_omni_runs_pull_dispatches_to_retrieve_runs(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    retrieve_runs = tmp_path / "retrieve_runs"
    _write_fake_setup(setup_file)
    _write_fake_retrieve_runs(retrieve_runs)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        [
            "scripts/omni",
            "runs",
            "pull",
            "demo_001",
            "--host",
            "robot.local",
            "--out",
            "runs/imported/demo_001",
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "retrieve_runs pull demo_001 --host robot.local --out runs/imported/demo_001\n"
