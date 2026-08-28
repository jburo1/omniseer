import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FAKE_GIT_SHA = "437c109075310102030405060708090a0b0c0d0e"


def _write_fake_docker(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -euo pipefail",
                'printf "docker" >>"${DOCKER_LOG}"',
                'for arg in "$@"; do printf " %q" "${arg}" >>"${DOCKER_LOG}"; done',
                'printf "\\n" >>"${DOCKER_LOG}"',
                'if [[ "${OMNISEER_FAKE_RUNTIME_FULL_VIDEO:-0}" == "1" && "${1:-}" == "run" ]]; then',
                '  args=("$@")',
                "  for ((index = 0; index < ${#args[@]}; index++)); do",
                '    if [[ "${args[index]}" == gateway_preview_record_path:=/runs/* ]]; then',
                '      preview_path="${args[index]#gateway_preview_record_path:=/runs/}"',
                '      runs_roots=("${OMNISEER_RUNTIME_RUNS_HOST_ROOT}")',
                '      runs_roots+=("${OMNISEER_RUNTIME_RUNS_LOCAL_ROOT:-}")',
                '      for runs_root in "${runs_roots[@]}"; do',
                '        [[ -n "${runs_root}" ]] || continue',
                '        output_path="${runs_root}/${preview_path}"',
                '        mkdir -p "$(dirname "${output_path}")"',
                '        printf "fake transport stream\\n" >"${output_path}"',
                "      done",
                "      break",
                "    fi",
                "  done",
                "fi",
                'if [[ "${1:-}" == "buildx" && "${2:-}" == "imagetools" && "${3:-}" == "inspect" ]]; then',
                '  printf "sha256:registry_digest\\n"',
                "  exit 0",
                "fi",
                'if [[ "${1:-}" == "image" && "${2:-}" == "inspect" ]]; then',
                '  if [[ "$*" == *RepoDigests* ]]; then',
                '    printf "%s@sha256:repo_digest\\n" "${@: -1}"',
                "  else",
                '    printf "%s\\n" "${OMNISEER_FAKE_DOCKER_IMAGE_ID:-sha256:local_image_id}"',
                "  fi",
                "fi",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_fake_git(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -euo pipefail",
                'while [[ "${1:-}" == "-C" ]]; do',
                "  shift 2",
                "done",
                'case "${1:-} ${2:-} ${3:-}" in',
                '  "rev-parse HEAD ")',
                '    printf "%s\\n" "${OMNISEER_FAKE_GIT_SHA}"',
                "    ;;",
                '  "rev-parse --short=12 HEAD")',
                '    printf "%.12s\\n" "${OMNISEER_FAKE_GIT_SHA}"',
                "    ;;",
                '  "status --porcelain ")',
                '    printf "%s" "${OMNISEER_FAKE_GIT_STATUS:-}"',
                "    ;;",
                "  *)",
                '    printf "unexpected fake git invocation: %q" "$@" >&2',
                "    exit 2",
                "    ;;",
                "esac",
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
    git = tmp_path / "git"
    _write_fake_git(git)
    rknn_include = tmp_path / "rknn_api.h"
    rknn_lib = tmp_path / "librknnrt.so"
    rknn_include.write_text("// fake RKNN header\n", encoding="utf-8")
    rknn_lib.write_text("fake RKNN runtime\n", encoding="utf-8")
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("\n", encoding="utf-8")
    ws_setup = tmp_path / "ws_setup.bash"
    ws_setup.write_text("\n", encoding="utf-8")
    ros2 = tmp_path / "ros2"
    ros2.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'printf "ros2 %q\\n" "$*" >>"${ROS2_LOG}"\n'
        'exit "${OMNISEER_FAKE_INSPECT_STATUS:-0}"\n',
        encoding="utf-8",
    )
    ros2.chmod(0o755)
    ffprobe = tmp_path / "ffprobe"
    ffprobe.write_text(
        '#!/usr/bin/env bash\nset -euo pipefail\nprintf "%s\\n" "${OMNISEER_FAKE_FFPROBE_CODEC:-h264}"\n',
        encoding="utf-8",
    )
    ffprobe.chmod(0o755)
    runs_root = tmp_path / "runs"

    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["DOCKER_LOG"] = str(tmp_path / "docker.log")
    env["ROS2_LOG"] = str(tmp_path / "ros2.log")
    env["OMNISEER_FAKE_GIT_SHA"] = FAKE_GIT_SHA
    env["OMNISEER_RUNTIME_METADATA_DIR"] = str(tmp_path / "metadata")
    env["OMNISEER_RKNN_INCLUDE"] = str(rknn_include)
    env["OMNISEER_RKNN_LIB"] = str(rknn_lib)
    env["OMNISEER_RUNTIME_SAFE_SMOKE_SEC"] = "1"
    env["OMNISEER_ROS_SETUP"] = str(ros_setup)
    env["OMNISEER_WS_SETUP"] = str(ws_setup)
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(runs_root)
    env["OMNISEER_RUNTIME_RUNS_LOCAL_ROOT"] = str(runs_root)
    return env


def _docker_log(env: dict[str, str]) -> str:
    return Path(env["DOCKER_LOG"]).read_text(encoding="utf-8")


def _write_full_verify_metadata(
    env: dict[str, str],
    *,
    tag: str = "runtime-test",
    image_id: str = "sha256:local_image_id",
    git_sha: str = FAKE_GIT_SHA,
) -> Path:
    metadata_dir = Path(env["OMNISEER_RUNTIME_METADATA_DIR"])
    metadata_dir.mkdir(parents=True)
    verify_file = metadata_dir / f"verify-full-{tag}.env"
    verify_file.write_text(
        "\n".join(
            [
                "IMAGE_BASE=ghcr.io/jburo1/omniseer-robot-runtime",
                f"TAG={tag}",
                f"IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:{tag}",
                f"IMAGE_ID={image_id}",
                f"GIT_SHA={git_sha}",
                "STAGE=full",
                "STATUS=passed",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return verify_file


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


def test_runtime_image_installs_rosbag_recording_dependencies() -> None:
    dockerfile = (REPO_ROOT / "docker/runtime/Dockerfile").read_text(encoding="utf-8")

    assert "ros-${ROS_DISTRO}-rosbag2" in dockerfile
    assert "ros-${ROS_DISTRO}-rosbag2-storage-default-plugins" in dockerfile


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
    assert "--sig-proxy=true" in log
    assert "--privileged" in log
    assert "--network=host" in log
    assert "--pid=host" in log
    assert "--ipc=host" in log
    assert "/dev:/dev" in log
    assert "/run/udev:/run/udev:ro" in log
    assert f"{env['OMNISEER_RUNTIME_RUNS_HOST_ROOT']}:/runs" in log
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
    assert "--name omniseer-runtime-record-operator_" in log
    assert "--label org.omniseer.kind=runtime-record" in log
    assert "run real --profile operator" in log
    assert "--record-run operator_" in log
    assert "--record-out /runs/operator_" in log
    assert "--record-overwrite" in log
    assert "--record-system-interval-sec 1.0" in log
    assert "--record-experiment-config operator-runtime" in log
    assert "--record-experiment-parameter stage=manual-operator" in log
    assert "--record-experiment-parameter stage=manual-operator bringup" in log
    assert "Run bundle path:" in result.stderr


def test_runtime_record_respects_forced_docker_tty(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_DOCKER_TTY"] = "always"

    result = subprocess.run(
        ["scripts/omni", "runtime", "record", "--tag", "runtime-test", "--run-id", "operator_debug"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker run -it --rm" in log
    assert "--record-run operator_debug" in log


def test_runtime_stop_stops_named_record_container(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)

    result = subprocess.run(
        ["scripts/omni", "runtime", "stop", "--run-id", "operator_debug", "--time", "7"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "docker container inspect omniseer-runtime-record-operator_debug" in log
    assert "docker stop --time 7 omniseer-runtime-record-operator_debug" in log


def test_runtime_record_accepts_options_and_launch_args(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(tmp_path / "runs")

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
            "--record-notes",
            "lighting changed",
            "--record-classes",
            "person,fire extinguisher",
            "--record-rosbag",
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
    assert "--record-notes lighting\\ changed" in log
    assert "--record-classes person\\,fire\\ extinguisher" in log
    assert "--record-rosbag" in log
    assert "bringup classes_path:=/runs/operator_debug/classes.txt start_lidar:=false" in log


def test_runtime_record_writes_class_file_in_runs_bind_root(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    runs_root = tmp_path / "runs"
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(runs_root)

    result = subprocess.run(
        [
            "scripts/omni",
            "runtime",
            "record",
            "--tag",
            "runtime-test",
            "--run-id",
            "operator_debug",
            "--record-classes",
            "person, fire extinguisher",
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (runs_root / "operator_debug/classes.txt").read_text(encoding="utf-8") == "person\nfire extinguisher\n"
    log = _docker_log(env)
    assert f"{runs_root}:/runs" in log
    assert "classes_path:=/runs/operator_debug/classes.txt" in log


def test_runtime_record_restores_runbundle_ownership_for_sudo_caller(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    runs_root = tmp_path / "runs"
    (runs_root / "operator_debug").mkdir(parents=True)
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(runs_root)
    env["SUDO_UID"] = "1001"
    env["SUDO_GID"] = "1002"

    result = subprocess.run(
        ["scripts/omni", "runtime", "record", "--tag", "runtime-test", "--run-id", "operator_debug"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert "--entrypoint /bin/chown" in log
    assert "-R 1001:1002 /runs/operator_debug" in log


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


def test_runtime_verify_full_records_rockchip_preview_artifact(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_RUNTIME_DOCKER_TTY"] = "always"
    env["OMNISEER_FAKE_RUNTIME_FULL_VIDEO"] = "1"

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
    assert "--record-run" not in log
    assert "--record-out" not in log
    assert "--record-video" not in log
    assert "--record-experiment-config" not in log
    assert "gateway_preview_encoder:=rockchip" in log
    assert "gateway_preview_record_path:=/runs/runtime_full_" in log
    assert not Path(env["ROS2_LOG"]).exists()
    preview_artifacts = list(Path(env["OMNISEER_RUNTIME_RUNS_LOCAL_ROOT"]).glob("runtime_full_*.ts"))
    assert len(preview_artifacts) == 1
    assert preview_artifacts[0].stat().st_size > 0
    metadata_file = Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "verify-full-runtime-test.env"
    assert metadata_file.is_file()
    metadata = metadata_file.read_text(encoding="utf-8")
    assert "IMAGE_REF=ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" in metadata
    assert "TAG=runtime-test" in metadata
    assert "IMAGE_ID=sha256:local_image_id" in metadata
    assert f"GIT_SHA={FAKE_GIT_SHA}" in metadata
    assert f"PREVIEW_ARTIFACT={preview_artifacts[0]}" in metadata


def test_runtime_verify_full_validates_caller_visible_preview_artifact(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    host_runs_root = tmp_path / "docker-host-runs"
    local_runs_root = tmp_path / "workspace-runs"
    env["OMNISEER_RUNTIME_RUNS_HOST_ROOT"] = str(host_runs_root)
    env["OMNISEER_RUNTIME_RUNS_LOCAL_ROOT"] = str(local_runs_root)
    env["OMNISEER_FAKE_RUNTIME_FULL_VIDEO"] = "1"

    result = subprocess.run(
        ["scripts/omni", "runtime", "verify", "--tag", "runtime-test", "--stage", "full"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    preview_artifacts = list(local_runs_root.glob("runtime_full_*.ts"))
    assert len(preview_artifacts) == 1
    assert preview_artifacts[0].stat().st_size > 0
    assert not Path(env["ROS2_LOG"]).exists()


def test_runtime_verify_full_does_not_depend_on_runbundle_inspection(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_FAKE_RUNTIME_FULL_VIDEO"] = "1"
    env["OMNISEER_FAKE_INSPECT_STATUS"] = "1"

    result = subprocess.run(
        ["scripts/omni", "runtime", "verify", "--tag", "runtime-test", "--stage", "full"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "verify-full-runtime-test.env").exists()
    assert not Path(env["ROS2_LOG"]).exists()


def test_runtime_verify_full_accepts_first_h264_codec_from_duplicate_ffprobe_output(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_FAKE_RUNTIME_FULL_VIDEO"] = "1"
    env["OMNISEER_FAKE_FFPROBE_CODEC"] = "h264\nh264"

    result = subprocess.run(
        ["scripts/omni", "runtime", "verify", "--tag", "runtime-test", "--stage", "full"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_runtime_verify_full_fails_when_preview_artifact_is_missing_or_not_h264(tmp_path: Path) -> None:
    for video_enabled, codec in ((False, "h264"), (True, "hevc")):
        case_dir = tmp_path / f"video_{video_enabled}_{codec}"
        case_dir.mkdir()
        env = _runtime_env(case_dir)
        env["OMNISEER_FAKE_RUNTIME_FULL_VIDEO"] = "1" if video_enabled else "0"
        env["OMNISEER_FAKE_FFPROBE_CODEC"] = codec

        result = subprocess.run(
            ["scripts/omni", "runtime", "verify", "--tag", "runtime-test", "--stage", "full"],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )

        assert result.returncode != 0
        assert not (Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / "verify-full-runtime-test.env").exists()


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
    _write_full_verify_metadata(env)

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
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test "
        f"ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-g{FAKE_GIT_SHA}"
    ) in log
    assert f"docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-g{FAKE_GIT_SHA}" in log
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test "
        "ghcr.io/jburo1/omniseer-robot-runtime:robot-verified"
    ) in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified" in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:runtime-test" not in log
    release_metadata = (
        Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / f"release-robot-verified-g{FAKE_GIT_SHA}.env"
    ).read_text(encoding="utf-8")
    assert f"GIT_SHA={FAKE_GIT_SHA}" in release_metadata
    assert "IMAGE_ID=sha256:local_image_id" in release_metadata
    assert "REGISTRY_DIGEST=ghcr.io/jburo1/omniseer-robot-runtime@sha256:registry_digest" in release_metadata
    assert f"PUSHED_TAGS=robot-verified-g{FAKE_GIT_SHA}\\ robot-verified" in release_metadata


def test_runtime_push_promotes_candidate_to_full_commit_verified_tag(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    tag = "robot-candidate-20260723T052827Z-g437c10907531"
    _write_full_verify_metadata(env, tag=tag)

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
        f"ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-g{FAKE_GIT_SHA}"
    ) in log
    assert f"docker push ghcr.io/jburo1/omniseer-robot-runtime:robot-verified-g{FAKE_GIT_SHA}" in log


def test_runtime_push_accepts_optional_release_tag(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    _write_full_verify_metadata(env)

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test", "--release-tag", "bench-release"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    log = _docker_log(env)
    assert (
        "docker tag ghcr.io/jburo1/omniseer-robot-runtime:runtime-test "
        "ghcr.io/jburo1/omniseer-robot-runtime:bench-release"
    ) in log
    assert "docker push ghcr.io/jburo1/omniseer-robot-runtime:bench-release" in log
    release_metadata = (
        Path(env["OMNISEER_RUNTIME_METADATA_DIR"]) / f"release-robot-verified-g{FAKE_GIT_SHA}.env"
    ).read_text(encoding="utf-8")
    assert f"PUSHED_TAGS=robot-verified-g{FAKE_GIT_SHA}\\ bench-release\\ robot-verified" in release_metadata


def test_runtime_push_requires_clean_git_tree(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    env["OMNISEER_FAKE_GIT_STATUS"] = " M scripts/runtime.sh\n"
    _write_full_verify_metadata(env)

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "git working tree must be clean" in result.stderr


def test_runtime_push_rejects_mismatched_verified_image_id(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    _write_full_verify_metadata(env, image_id="sha256:different_image_id")

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "does not match verified image ID" in result.stderr


def test_runtime_push_rejects_mismatched_verified_commit(tmp_path: Path) -> None:
    env = _runtime_env(tmp_path)
    _write_full_verify_metadata(env, git_sha="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

    result = subprocess.run(
        ["scripts/omni", "runtime", "push", "--tag", "runtime-test"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "verification commit does not match current git commit" in result.stderr


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
