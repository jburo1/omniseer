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
    return env


def _docker_log(env: dict[str, str]) -> str:
    return Path(env["DOCKER_LOG"]).read_text(encoding="utf-8")


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
    assert (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "build-runtime-test.env").is_file()


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
    assert "--privileged" in log
    assert "--network=host" in log
    assert "--pid=host" in log
    assert "--ipc=host" in log
    assert "/dev:/dev" in log
    assert "/run/udev:/run/udev:ro" in log
    assert f"{REPO_ROOT}/runs:/runs" in log
    assert "OMNISEER_CONTAINER_IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" in log
    assert (
        "OMNISEER_CONTAINER_IMAGE_DIGEST=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test@sha256:repo_digest"
        in log
    )


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
        "ghcr.io/jburo1/omniseer-robot-runtime:robot-verified"
    ) in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified" in log


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
