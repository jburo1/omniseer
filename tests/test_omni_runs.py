import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_fake_setup(path: Path) -> None:
    path.write_text("# test setup shim\n", encoding="utf-8")


def _write_fake_ros2(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "printf 'ros2'",
                'for arg in "$@"; do',
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
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "list"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        "ros2 run omniseer_experiments retrieve_runs list --host 192.168.1.178 "
        "--user radxa --remote-root /home/radxa/apps/omniseer/runs\n"
    )


def test_omni_runs_pull_dispatches_to_retrieve_runs(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
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
    assert (
        result.stdout == "ros2 run omniseer_experiments retrieve_runs pull demo_001 --host robot.local "
        "--out runs/imported/demo_001 --user radxa --remote-root /home/radxa/apps/omniseer/runs\n"
    )


def test_omni_runs_preserves_explicit_host_and_user(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        [
            "scripts/omni",
            "runs",
            "list",
            "--host=robot.local",
            "--user=operator",
            "--remote-root=/tmp/runs",
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (
        result.stdout == "ros2 run omniseer_experiments retrieve_runs list --host=robot.local "
        "--user=operator --remote-root=/tmp/runs\n"
    )


def test_omni_runs_inspect_dispatches_to_inspect_run(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "inspect", "runs/imported/demo_001", "--json"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "ros2 run omniseer_experiments inspect_run runs/imported/demo_001 --json\n"


def test_omni_runs_annotate_dispatches_to_annotate_evidence(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "annotate", "runs/imported/demo_001", "--overwrite"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "ros2 run omniseer_experiments annotate_evidence runs/imported/demo_001 --overwrite\n"


def test_omni_runs_report_dispatches_to_report_run(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "report", "runs/imported/demo_001", "--overwrite"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "ros2 run omniseer_experiments report_run runs/imported/demo_001 --overwrite\n"


def test_omni_runs_local_list_dispatches_to_list_runs(tmp_path: Path) -> None:
    setup_file = tmp_path / "setup.bash"
    ros2 = tmp_path / "ros2"
    _write_fake_setup(setup_file)
    _write_fake_ros2(ros2)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["OMNISEER_ROS_SETUP"] = str(setup_file)
    env["OMNISEER_WS_SETUP"] = str(setup_file)

    result = subprocess.run(
        ["scripts/omni", "runs", "local-list", "--root", "runs"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "ros2 run omniseer_experiments list_runs --root runs\n"
