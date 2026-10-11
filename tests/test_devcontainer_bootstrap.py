import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


def run_bootstrap(tmp_path: Path, *args: str, layout: str | None = None, failure: str = ""):
    for directory in (".devcontainer", "scripts/lib", "ros_ws/install", "uros_ws/install", "home"):
        (tmp_path / directory).mkdir(parents=True)
    shutil.copy(REPO_ROOT / ".devcontainer/bootstrap.sh", tmp_path / ".devcontainer/bootstrap.sh")
    shutil.copy(REPO_ROOT / "scripts/lib/common.sh", tmp_path / "scripts/lib/common.sh")
    if layout is not None:
        (tmp_path / "uros_ws/install/.colcon_install_layout").write_text(layout)
    (tmp_path / "home/.bashrc").touch()
    omni = tmp_path / "scripts/omni"
    omni.write_text(
        '#!/bin/bash\nprintf "omni %s\\n" "$*" >> "$BOOTSTRAP_LOG"\nif [[ "$1" == "$FAILURE" ]]; then exit 23; fi\n'
    )
    omni.chmod(0o755)
    mock_env = tmp_path / "mock-env.sh"
    mock_env.write_text(
        'source() { case "$1" in */scripts/lib/common.sh) builtin source "$1" ;; esac; }\n'
        'sudo() { shift; "$@"; }\n'
        "pkg-config() { return 1; }\n"
        'rosdep() { printf "rosdep %s\\n" "$*" >> "$BOOTSTRAP_LOG"; }\n'
        'python() { printf "python %s\\n" "$*" >> "$BOOTSTRAP_LOG"; '
        'if [[ "$2" == "$FAILURE" ]]; then return 23; fi; }\n'
    )
    env = os.environ.copy()
    env.update(
        HOME=str(tmp_path / "home"),
        BASH_ENV=str(mock_env),
        BOOTSTRAP_LOG=str(tmp_path / "commands.log"),
        ROS_DISTRO="kilted",
        FAILURE=failure,
    )
    result = subprocess.run(
        ["bash", str(tmp_path / ".devcontainer/bootstrap.sh"), *args],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    return result, (tmp_path / "commands.log").read_text()


def test_laptop_bootstrap_uses_portable_workflow_without_micro_ros(tmp_path: Path) -> None:
    result, log = run_bootstrap(tmp_path)
    assert result.returncode == 0, result.stderr
    assert "omni setup ros-deps ros_ws/src/omniseer_gz_assets" in log
    assert "omni build ros --symlink-install" in log
    assert "omniseer_vision_bridge" not in log
    assert "colcon build" not in log


def test_yolo_bootstrap_includes_provider_dependencies_and_build_flag(tmp_path: Path) -> None:
    result, log = run_bootstrap(tmp_path, "--with-yolo")
    assert result.returncode == 0, result.stderr
    assert "ros_ws/src/yolo_ros/yolo_ros ros_ws/src/yolo_ros/yolo_bringup" in log
    assert "omni build ros --with-yolo" in log


@pytest.mark.parametrize("layout", [None, "isolated", "merged"])
def test_optional_micro_ros_preserves_install_layout(tmp_path: Path, layout: str | None) -> None:
    result, log = run_bootstrap(tmp_path, "--with-micro-ros", layout=layout)
    assert result.returncode == 0, result.stderr
    micro_build = next(line for line in log.splitlines() if "colcon build" in line)
    assert ("--merge-install" in micro_build) == (layout == "merged")


@pytest.mark.parametrize("failure", ["setup", "pip", "build", "colcon"])
def test_bootstrap_propagates_failures_before_writing_overlay(tmp_path: Path, failure: str) -> None:
    result, log = run_bootstrap(tmp_path, "--with-micro-ros", failure=failure)
    assert result.returncode == 23
    assert (tmp_path / "home/.bashrc").read_text() == ""
    if failure in ("setup", "pip"):
        assert "omni build" not in log
    if failure == "build":
        assert "colcon build" not in log
