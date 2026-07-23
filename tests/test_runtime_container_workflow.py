import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_fake_docker(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -euo pipefail",
                'printf "docker" >>"${DOCKER_LOG}"',
                'for arg in "$@"; do printf " %q" "${arg}" >>"${DOCKER_LOG}"; done',
                'printf "\\n" >>"${DOCKER_LOG}"',
                'if [[ "${1:-}" == "image" && "${2:-}" == "inspect" ]]; then',
                '  if [[ "$*" == *RepoDigests* ]]; then',
                '    printf "%s@sha256:repo_digest\\n" "${@: -1}"',
                "  else",
                '    printf "sha256:local_image_id\\n"',
                "  fi",
                "fi",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_fake_findmnt(path: Path, *, target: str, source: str) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -euo pipefail",
                'if [[ "$*" == *"TARGET"* ]]; then',
                f'  printf "%s\\n" "{target}"',
                'elif [[ "$*" == *"SOURCE"* ]]; then',
                f'  printf "%s\\n" "{source}"',
                "fi",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _runtime_env(tmp_path: Path) -> dict[str, str]:
    docker = tmp_path / "docker"
    _write_fake_docker(docker)
    rknn_include = tmp_path / "rknn_api.h"
    rknn_lib = tmp_path / "librknnrt.so"
    rknn_include.write_text("// fake RKNN header\n", encoding="utf-8")
    rknn_lib.write_text("fake RKNN runtime\n", encoding="utf-8")

    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["DOCKER_LOG"] = str(tmp_path / "docker.log")
    env["OMNISEER_RUNTIME_METADATA_DIR"] = str(tmp_path / "metadata")
    env["OMNISEER_RKNN_INCLUDE"] = str(rknn_include)
    env["OMNISEER_RKNN_LIB"] = str(rknn_lib)
    env["OMNISEER_RUNTIME_SAFE_SMOKE_SEC"] = "1"
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = f"{REPO_ROOT}/runs"
    return env


def _docker_log(env: dict[str, str]) -> str:
    return Path(env["DOCKER_LOG"]).read_text(encoding="utf-8")


def test_robot_runtime_packages_vision_testdata_at_configured_path() -> None:
    dockerfile = (REPO_ROOT / "docker/runtime/Dockerfile").read_text(encoding="utf-8")
    config = (REPO_ROOT / "ros_ws/src/bringup/config/vision_bridge.real.paths.yaml").read_text(encoding="utf-8")

    assert "COPY --from=robot-builder /opt/omniseer-src/vision/testdata /omniseer/vision/testdata" in dockerfile
    assert "classes.path: /omniseer/vision/testdata/text_embeddings/classes_person_bus.txt" in config


def test_runtime_image_carries_source_git_sha() -> None:
    dockerfile = (REPO_ROOT / "docker/runtime/Dockerfile").read_text(encoding="utf-8")
    build_script = (REPO_ROOT / "scripts/build/runtime_container.sh").read_text(encoding="utf-8")

    assert "ARG OMNISEER_GIT_SHA=unknown" in dockerfile
    assert 'LABEL org.opencontainers.image.revision="${OMNISEER_GIT_SHA}"' in dockerfile
    assert "OMNISEER_GIT_SHA=${OMNISEER_GIT_SHA}" in dockerfile
    assert '--build-arg "OMNISEER_GIT_SHA=${git_sha}"' in build_script


def test_robot_runtime_installs_rockchip_v4l_packages_for_preview() -> None:
    dockerfile = (REPO_ROOT / "docker/runtime/Dockerfile").read_text(encoding="utf-8")

    for package in ("libv4l-0t64", "libv4l2rds0t64", "libv4lconvert0t64", "v4l-utils"):
        assert dockerfile.count(package) >= 2
    assert dockerfile.count("/etc/ld.so.conf.d/rockchip-v4l-mplane.conf") == 2
    assert dockerfile.count("/usr/lib/aarch64-linux-gnu/libv4l/plugins") == 2


def test_runtime_build_dispatches_to_robot_runtime_build_with_default_image(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "build", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker build" in log
    assert "--target robot-runtime" in log
    assert "--tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" in log
    assert "--build-arg OMNISEER_GIT_SHA=" in log
    assert (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "build-runtime-test.env").is_file()


def test_runtime_build_default_tag_uses_robot_candidate_name(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "build"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--tag ghcr.io/jburo1/omniseer-robot-runtime:robot-candidate-" in _docker_log(env)


def test_runtime_run_uses_robot_container_flags_and_provenance(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "run", "--tag", "runtime-test", "run", "real", "bringup"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker image inspect" in log
    assert "docker run" in log
    assert "-it" not in log
    assert "--privileged" in log
    assert "--network=host" in log
    assert "--pid=host" in log
    assert "--ipc=host" in log
    assert "/dev:/dev" in log
    assert "/run/udev:/run/udev:ro" in log
    assert f"{REPO_ROOT}/runs:/runs" in log
    assert "OMNISEER_CONTAINER_IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" in log
    assert (
        "OMNISEER_CONTAINER_IMAGE_DIGEST=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test@sha256:repo_digest" in log
    )
    assert "OMNISEER_RUNTIME_CONTAINER_COMMAND=run\\ real\\ bringup" in log


def test_runtime_run_uses_devcontainer_workspace_source_for_runs_bind(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env.pop("OMNISEER_RUNTIME_RUNS_HOST_ROOT")
    _write_fake_findmnt(
        tmp_path / "findmnt",
        target=str(REPO_ROOT),
        source="/dev/nvme0n1p3[/home/radxa/apps/omniseer]",
    )

    result = subprocess.run(
        ["scripts/omni", "runtime", "run", "--tag", "runtime-test", "run", "real", "bringup"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "/home/radxa/apps/omniseer/runs:/runs" in _docker_log(env)


def test_runtime_run_can_force_docker_tty_for_interactive_sessions(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_DOCKER_TTY"] = "always"

    result = subprocess.run(
        ["scripts/omni", "runtime", "run", "--tag", "runtime-test", "run", "real", "bringup"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "docker run -it --rm" in _docker_log(env)


def test_runtime_record_runs_operator_recording_with_defaults(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_DOCKER_TTY"] = "always"

    result = subprocess.run(
        ["scripts/omni", "runtime", "record", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker run" in log
    assert "-it" not in log
    assert "run real --profile operator" in log
    assert "--record-run operator_" in log
    assert "--record-out /runs/operator_" in log
    assert "--record-overwrite" in log
    assert "--record-system-interval-sec 1.0" in log
    assert "--record-experiment-config operator-runtime" in log
    assert "--record-experiment-parameter stage=manual-operator" in log
    assert "--record-experiment-parameter stage=manual-operator bringup" in log
    assert "Run bundle path:" in result.stderr


def test_runtime_record_accepts_options_and_launch_args(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        [
            "scripts/omni",
            "runtime",
            "record",
            "--tag",
            "runtime-test",
            "--run-id",
            "operator_debug",
            "--system-interval-sec",
            "0.5",
            "--experiment-config",
            "operator-runtime-debug",
            "--experiment-parameter",
            "note=desk",
            "--",
            "start_lidar:=false",
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "--record-run operator_debug" in log
    assert "--record-out /runs/operator_debug" in log
    assert "--record-system-interval-sec 0.5" in log
    assert "--record-experiment-config operator-runtime-debug" in log
    assert "--record-experiment-parameter note=desk" in log
    assert "bringup start_lidar:=false" in log


def test_runtime_verify_safe_smoke_treats_timeout_as_pass(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    fake_timeout = tmp_path / "timeout"
    fake_timeout.write_text("#!/usr/bin/env bash\nexit 124\n", encoding="utf-8")
    fake_timeout.chmod(0o755)

    result = subprocess.run(
        ["scripts/omni", "runtime", "verify", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "verify-smoke-runtime-test.env").is_file()


def test_runtime_verify_full_records_run_with_provenance(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_DOCKER_TTY"] = "always"

    result = subprocess.run(
        ["scripts/omni", "runtime", "verify", "--tag", "runtime-test", "--stage", "full"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker run" in log
    assert "-it" not in log
    assert "--record-run" in log
    assert "--record-out /runs/runtime_full_" in log
    assert "--record-experiment-config runtime-container-full" in log
    assert "--record-experiment-parameter stage=full" in log
    assert (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "verify-full-runtime-test.env").is_file()


def test_runtime_push_requires_full_verification_metadata(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "full verification metadata is missing" in result.stderr


def test_runtime_push_promotes_verified_image(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    metadata_dir = Path(env["OMNISEER_RUNTIME_METADATA_DIR"])
    metadata_dir.mkdir(parents=True)
    (metadata_dir / "verify-full-runtime-test.env").write_text(
        "\n".join(
            [
                "IMAGE_BASE=ghcr.io/jburo1/omniseer-robot-runtime",
                "TAG=runtime-test",
                "IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test",
                "IMAGE_ID=sha256:local_image_id",
                "STAGE=full",
                "STATUS=passed",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" in log
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test "
        "ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-runtime-test"
    ) in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-runtime-test" in log
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test "
        "ghcr.io/jburo1/omniseer-robot-runtime:robot-verified"
    ) in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified" in log


def test_runtime_push_promotes_candidate_to_matching_verified_tag(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    metadata_dir = Path(env["OMNISEER_RUNTIME_METADATA_DIR"])
    metadata_dir.mkdir(parents=True)
    tag = "robot-candidate-20260723T052827Z-g437c10907531"
    (metadata_dir / f"verify-full-{tag}.env").write_text(
        "\n".join(
            [
                "IMAGE_BASE=ghcr.io/jburo1/omniseer-robot-runtime",
                f"TAG={tag}",
                f"IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:{tag}",
                "IMAGE_ID=sha256:local_image_id",
                "STAGE=full",
                "STATUS=passed",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", tag],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:robot-candidate-20260723T052827Z-g437c10907531 "
        "ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-20260723T052827Z-g437c10907531"
    ) in log
    assert ("docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-20260723T052827Z-g437c10907531") in log


def test_runtime_pull_defaults_to_robot_verified(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "pull"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "docker pull ghcr.io/jburo1/omniseer-robot-runtime:robot-verified" in _docker_log(env)
