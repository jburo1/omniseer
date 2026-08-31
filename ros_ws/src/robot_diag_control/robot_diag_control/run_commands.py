from __future__ import annotations

import shlex
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

RUN_BACKEND_RUNTIME = "robot_runtime_container"
RUN_BACKEND_DEVCONTAINER = "robot_devcontainer"
RUN_BACKEND_LABELS = {
    RUN_BACKEND_RUNTIME: "Robot runtime container",
    RUN_BACKEND_DEVCONTAINER: "Robot devcontainer",
}
DEFAULT_DEVCONTAINER_EXEC_TEMPLATE = (
    "container=$(docker ps --filter label=devcontainer.local_folder={remote_repo_root} "
    "--format '{{{{.Names}}}}' | head -n 1); "
    'if [ -z "$container" ]; then '
    'echo "no running Omniseer devcontainer found for {remote_repo_root}" >&2; exit 127; '
    "fi; "
    'docker exec -it "$container" bash -lc {command}'
)
RUN_TYPE_PERCEPTION = "perception_recording"
RUN_TYPE_AUTONOMY_CENTER = "autonomy_center_first_class"
RUN_TYPE_LABELS = {
    RUN_TYPE_PERCEPTION: "Perception: 360° environment scan",
    RUN_TYPE_AUTONOMY_CENTER: "Autonomy: frame and capture target",
}
PREVIEW_ENCODER_ROCKCHIP = "rockchip"
PREVIEW_ENCODER_SOFTWARE = "software"
PREVIEW_ENCODER_LABELS = {
    PREVIEW_ENCODER_ROCKCHIP: "Rockchip hardware",
    PREVIEW_ENCODER_SOFTWARE: "Software x264",
}
DEFAULT_RUNTIME_TAG = "robot-verified"
RUNTIME_DEFAULT_MODEL_LABEL = "Runtime default"


@dataclass(frozen=True)
class DetectorModelChoice:
    variant: str
    precision: str
    artifact_filename: str
    family: str = "yolo-world"
    backend: str = "rknn"


DETECTOR_MODEL_CHOICES: dict[str, DetectorModelChoice | None] = {
    RUNTIME_DEFAULT_MODEL_LABEL: None,
    "YOLO-World v2-S INT8 (recalibrated)": DetectorModelChoice("v2s", "int8", "yolo_world_v2_s_i8.rknn"),
    "YOLO-World v2-S FP": DetectorModelChoice("v2s", "fp", "yolo_world_v2_s_fp.rknn"),
    "YOLO-World v2-M INT8 (recalibrated)": DetectorModelChoice("v2m", "int8", "yolo_world_v2_m_i8.rknn"),
    "YOLO-World v2-M FP": DetectorModelChoice("v2m", "fp", "yolo_world_v2_m_fp.rknn"),
    "YOLO-World v2-L FP": DetectorModelChoice("v2l", "fp", "yolo_world_v2_l_fp.rknn"),
    "YOLO-World v2-L INT8 (recalibrated)": DetectorModelChoice("v2l", "int8", "yolo_world_v2_l_i8.rknn"),
}


@dataclass(frozen=True)
class RobotConnection:
    host: str
    ssh_user: str
    remote_repo_root: str
    remote_runs_root: str


@dataclass(frozen=True)
class RunConfig:
    run_id: str
    backend: str
    classes: tuple[str, ...] = ()
    notes: str = ""
    runtime_tag: str = DEFAULT_RUNTIME_TAG
    devcontainer_exec_template: str = ""
    run_type: str = RUN_TYPE_PERCEPTION
    experiment_config: str = ""
    autonomy_bbox_area_min_ratio: str = "0.08"
    autonomy_approach_stop_area_ratio: str = "0.10"
    autonomy_bbox_area_max_ratio: str = "0.35"
    autonomy_forward_speed_m_s: str = "0.05"
    autonomy_reverse_speed_m_s: str = "0.04"
    autonomy_stable_framed_frames: str = "10"
    autonomy_success_miss_tolerance_updates: str = "2"
    autonomy_proximity_stop_m: str = "0.30"
    autonomy_capture_timeout_sec: str = "2.0"
    autonomy_min_target_confidence: str = "0.50"
    autonomy_max_target_center_jump_ratio: str = "0.20"
    autonomy_evidence_interval_sec: str = "0.25"
    perception_scan_yaw_rate_rad_s: str = "0.20"
    detector_score_threshold: str = "0.25"
    detector_nms_iou_threshold: str = "0.45"
    detector_max_detections: str = "100"
    detector_model_artifact: str = ""
    experiment_model_family: str = ""
    experiment_model_variant: str = ""
    experiment_model_precision: str = ""
    experiment_model_backend: str = ""
    preview_encoder: str = PREVIEW_ENCODER_ROCKCHIP
    record_video: bool = False
    record_rosbag: bool = False


def sanitize_run_id(run_id: str) -> str:
    sanitized = "".join(char if char.isalnum() or char in {"_", "-", "."} else "_" for char in run_id.strip())
    return sanitized.strip("._-")


def parse_run_classes(raw_text: str) -> list[str]:
    classes: list[str] = []
    seen: set[str] = set()
    if "," in raw_text or "\n" in raw_text:
        raw_tokens = raw_text.replace("\n", ",").split(",")
    else:
        raw_tokens = raw_text.split()
    for token in raw_tokens:
        normalized = token.strip()
        if not normalized or normalized in seen:
            continue
        classes.append(normalized)
        seen.add(normalized)
    return classes


def remote_run_dir_for(remote_repo_root: str, run_id: str) -> str:
    return f"{remote_repo_root.rstrip('/')}/runs/{run_id}"


def remote_class_list_path_for(remote_repo_root: str, run_id: str) -> str:
    return f"{remote_run_dir_for(remote_repo_root, run_id)}/classes.txt"


def devcontainer_workspace_root_for(remote_repo_root: str) -> str:
    workspace_name = PurePosixPath(remote_repo_root.rstrip("/") or "/workspace").name
    return f"/{workspace_name or 'workspace'}"


def _runtime_container_class_list_path(run_id: str) -> str:
    return f"/runs/{run_id}/classes.txt"


def _runtime_container_run_dir(run_id: str) -> str:
    return f"/runs/{run_id}"


def _runtime_container_model_path(artifact_filename: str) -> str:
    return f"/runs/model_artifacts/{artifact_filename}"


def _model_launch_args(run_config: RunConfig, *, model_path: str) -> list[str]:
    if not run_config.detector_model_artifact:
        return []
    return [
        f"detector_model_path:={model_path}",
        f"experiment_model_family:={run_config.experiment_model_family}",
        f"experiment_model_variant:={run_config.experiment_model_variant}",
        f"experiment_model_precision:={run_config.experiment_model_precision}",
        f"experiment_model_backend:={run_config.experiment_model_backend}",
    ]


def _omni_command(repo_root: Path) -> str:
    return str(repo_root / "scripts" / "omni")


def _ssh_target(user: str, host: str) -> str:
    user = user.strip()
    host = host.strip()
    return f"{user}@{host}" if user else host


def shell_join(command: list[str]) -> str:
    return shlex.join(command)


def _build_remote_mkdir_command(*, ssh_user: str, host: str, remote_run_dir: str) -> list[str]:
    return ["ssh", _ssh_target(ssh_user, host), f"mkdir -p {shlex.quote(remote_run_dir)}"]


def build_remote_run_mkdir_command(*, connection: RobotConnection, run_config: RunConfig) -> list[str]:
    return _build_remote_mkdir_command(
        ssh_user=connection.ssh_user,
        host=connection.host,
        remote_run_dir=remote_run_dir_for(connection.remote_repo_root, run_config.run_id),
    )


def build_upload_classes_command(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
    local_path: Path,
) -> list[str]:
    remote_class_path = remote_class_list_path_for(connection.remote_repo_root, run_config.run_id)
    return ["scp", str(local_path), f"{_ssh_target(connection.ssh_user, connection.host)}:{remote_class_path}"]


def _build_runtime_record_inner_command(
    *,
    run_id: str,
    classes: Sequence[str],
    notes: str,
    run_config: RunConfig,
) -> list[str]:
    if run_config.run_type == RUN_TYPE_PERCEPTION:
        classes = ()
    command = [
        "scripts/omni",
        "runtime",
        "record",
        "--tag",
        run_config.runtime_tag,
        "--run-id",
        run_id,
    ]
    if classes:
        command.extend(["--record-classes", ",".join(classes)])
    if notes.strip():
        command.extend(["--record-notes", notes.strip()])
    if _records_video(run_config):
        command.append("--record-video")
    if run_config.record_rosbag:
        command.append("--record-rosbag")
    command.extend(_record_experiment_config_args(run_config))
    command.extend(_detector_experiment_parameters(run_config))
    command.append("--")
    command.append("experiment_overwrite:=true")
    command.append(f"gateway_preview_encoder:={run_config.preview_encoder}")
    if classes:
        command.append(f"classes_path:={_runtime_container_class_list_path(run_id)}")
    if run_config.run_type == RUN_TYPE_AUTONOMY_CENTER:
        command.extend(
            _model_launch_args(
                run_config,
                model_path=_runtime_container_model_path(run_config.detector_model_artifact),
            )
        )
    command.extend(_detector_parameter_launch_args(run_config))
    command.extend(_perception_scan_launch_args(run_config))
    command.extend(
        _autonomy_launch_args(
            classes=classes,
            run_type=run_config.run_type,
            run_dir=_runtime_container_run_dir(run_id),
        )
    )
    command.extend(_autonomy_parameter_launch_args(run_config))
    return command


def _build_devcontainer_record_inner_command(
    *,
    remote_repo_root: str,
    run_id: str,
    classes: Sequence[str],
    notes: str,
    run_config: RunConfig,
) -> list[str]:
    container_repo_root = devcontainer_workspace_root_for(remote_repo_root)
    container_run_dir = remote_run_dir_for(container_repo_root, run_id)
    if run_config.run_type == RUN_TYPE_PERCEPTION:
        classes = ()
    command = [
        "scripts/omni",
        "run",
        "real",
        "--profile",
        "operator",
        "--record-run",
        run_id,
        "--record-out",
        container_run_dir,
        "--record-overwrite",
    ]
    if classes:
        command.extend(["--record-classes", ",".join(classes)])
    if notes.strip():
        command.extend(["--record-notes", notes.strip()])
    if _records_video(run_config):
        command.append("--record-video")
    if run_config.record_rosbag:
        command.append("--record-rosbag")
    command.extend(_record_experiment_config_args(run_config))
    command.extend(_detector_experiment_parameters(run_config))
    command.append("bringup")
    command.append("experiment_overwrite:=true")
    command.append(f"gateway_preview_encoder:={run_config.preview_encoder}")
    if classes:
        command.append(f"classes_path:={remote_class_list_path_for(container_repo_root, run_id)}")
    if run_config.run_type == RUN_TYPE_AUTONOMY_CENTER:
        command.extend(
            _model_launch_args(
                run_config,
                model_path=f"{container_repo_root}/runs/model_artifacts/{run_config.detector_model_artifact}",
            )
        )
    command.extend(_detector_parameter_launch_args(run_config))
    command.extend(_perception_scan_launch_args(run_config))
    command.extend(_autonomy_launch_args(classes=classes, run_type=run_config.run_type, run_dir=container_run_dir))
    command.extend(_autonomy_parameter_launch_args(run_config))
    return command


def _detector_parameter_launch_args(run_config: RunConfig) -> list[str]:
    if run_config.run_type == RUN_TYPE_PERCEPTION:
        return []
    return [
        f"postprocess_score_threshold:={run_config.detector_score_threshold}",
        f"postprocess_nms_iou_threshold:={run_config.detector_nms_iou_threshold}",
        f"postprocess_max_detections:={run_config.detector_max_detections}",
    ]


def _detector_experiment_parameters(run_config: RunConfig) -> list[str]:
    if run_config.run_type == RUN_TYPE_PERCEPTION:
        return ["--record-experiment-parameter", f"preview.encoder={run_config.preview_encoder}"]
    return [
        "--record-experiment-parameter",
        f"preview.encoder={run_config.preview_encoder}",
        "--record-experiment-parameter",
        f"postprocess.score_threshold={run_config.detector_score_threshold}",
        "--record-experiment-parameter",
        f"postprocess.nms_iou_threshold={run_config.detector_nms_iou_threshold}",
        "--record-experiment-parameter",
        f"postprocess.max_detections={run_config.detector_max_detections}",
    ]


def _record_experiment_config_args(run_config: RunConfig) -> list[str]:
    config = run_config.experiment_config.strip() or RUN_TYPE_LABELS.get(run_config.run_type, run_config.run_type)
    return ["--record-experiment-config", config]


def _records_video(run_config: RunConfig) -> bool:
    return run_config.record_video or run_config.run_type == RUN_TYPE_PERCEPTION


def _perception_scan_launch_args(run_config: RunConfig) -> list[str]:
    if run_config.run_type != RUN_TYPE_PERCEPTION:
        return []
    return [
        "start_perception_scan:=true",
        "start_vision:=false",
        "gateway_require_vision:=false",
        f"perception_scan_yaw_rate_rad_s:={run_config.perception_scan_yaw_rate_rad_s}",
    ]


def _autonomy_launch_args(*, classes: Sequence[str], run_type: str, run_dir: str) -> list[str]:
    if run_type == RUN_TYPE_PERCEPTION:
        return []
    if run_type != RUN_TYPE_AUTONOMY_CENTER:
        raise ValueError(f"unsupported run type: {run_type}")
    if not classes:
        raise ValueError("autonomy run requires at least one target class")
    target_class = classes[0]
    return [
        "start_autonomy:=true",
        f"autonomy_target_class:={target_class}",
        f"autonomy_run_dir:={run_dir}",
    ]


def _autonomy_parameter_launch_args(run_config: RunConfig) -> list[str]:
    if run_config.run_type == RUN_TYPE_PERCEPTION:
        return []
    if run_config.run_type != RUN_TYPE_AUTONOMY_CENTER:
        raise ValueError(f"unsupported run type: {run_config.run_type}")
    return [
        f"autonomy_bbox_area_min_ratio:={run_config.autonomy_bbox_area_min_ratio}",
        f"autonomy_approach_stop_area_ratio:={run_config.autonomy_approach_stop_area_ratio}",
        f"autonomy_bbox_area_max_ratio:={run_config.autonomy_bbox_area_max_ratio}",
        f"autonomy_forward_speed_m_s:={run_config.autonomy_forward_speed_m_s}",
        f"autonomy_reverse_speed_m_s:={run_config.autonomy_reverse_speed_m_s}",
        f"autonomy_stable_framed_frames:={run_config.autonomy_stable_framed_frames}",
        f"autonomy_success_miss_tolerance_updates:={run_config.autonomy_success_miss_tolerance_updates}",
        f"autonomy_proximity_stop_m:={run_config.autonomy_proximity_stop_m}",
        f"autonomy_capture_timeout_sec:={run_config.autonomy_capture_timeout_sec}",
        f"autonomy_min_target_confidence:={run_config.autonomy_min_target_confidence}",
        f"autonomy_max_target_center_jump_ratio:={run_config.autonomy_max_target_center_jump_ratio}",
        f"evidence_interval_sec:={run_config.autonomy_evidence_interval_sec}",
    ]


def _format_devcontainer_exec_template(
    *,
    template: str,
    command: str,
    remote_repo_root: str,
    run_id: str,
) -> str:
    values = {
        "command": shlex.quote(command),
        "remote_repo_root": shlex.quote(remote_repo_root),
        "remote_run_dir": shlex.quote(remote_run_dir_for(remote_repo_root, run_id)),
        "run_id": shlex.quote(run_id),
    }
    formatted = template.format(**values)
    if "{command}" not in template:
        formatted = f"{formatted} {shlex.quote(command)}"
    return formatted


def build_remote_start_command(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
) -> list[str]:
    if run_config.backend == RUN_BACKEND_RUNTIME:
        inner = _build_runtime_record_inner_command(
            run_id=run_config.run_id,
            classes=run_config.classes,
            notes=run_config.notes,
            run_config=run_config,
        )
        remote_command = f"cd {shlex.quote(connection.remote_repo_root)} && {shell_join(inner)}"
    elif run_config.backend == RUN_BACKEND_DEVCONTAINER:
        container_repo_root = devcontainer_workspace_root_for(connection.remote_repo_root)
        inner = _build_devcontainer_record_inner_command(
            remote_repo_root=connection.remote_repo_root,
            run_id=run_config.run_id,
            classes=run_config.classes,
            notes=run_config.notes,
            run_config=run_config,
        )
        inner_shell = f"cd {shlex.quote(container_repo_root)} && {shell_join(inner)}"
        remote_command = _format_devcontainer_exec_template(
            template=run_config.devcontainer_exec_template,
            command=inner_shell,
            remote_repo_root=connection.remote_repo_root,
            run_id=run_config.run_id,
        )
    else:
        raise ValueError(f"unsupported run backend: {run_config.backend}")
    return ["ssh", "-tt", _ssh_target(connection.ssh_user, connection.host), remote_command]


def build_remote_runtime_stop_command(
    *,
    connection: RobotConnection,
    run_id: str,
) -> list[str]:
    inner = ["scripts/omni", "runtime", "stop", "--run-id", run_id]
    remote_command = f"cd {shlex.quote(connection.remote_repo_root)} && {shell_join(inner)}"
    return ["ssh", _ssh_target(connection.ssh_user, connection.host), remote_command]


def build_pull_run_command(
    *,
    repo_root: Path,
    connection: RobotConnection,
    local_import_root: Path,
    run_id: str,
    overwrite: bool = True,
) -> list[str]:
    command = [
        _omni_command(repo_root),
        "runs",
        "pull",
        run_id,
        "--host",
        connection.host,
        "--user",
        connection.ssh_user,
        "--remote-root",
        connection.remote_runs_root,
        "--import-root",
        str(local_import_root),
    ]
    if overwrite:
        command.append("--overwrite")
    return command


def local_import_dir_for(local_import_root: Path, run_id: str) -> Path:
    return local_import_root / run_id


def build_report_command(*, repo_root: Path, run_dir: Path, overwrite: bool = True) -> list[str]:
    command = [_omni_command(repo_root), "runs", "report", str(run_dir)]
    if overwrite:
        command.append("--overwrite")
    return command


def build_video_command(*, repo_root: Path, run_dir: Path) -> list[str]:
    return [_omni_command(repo_root), "runs", "video", str(run_dir)]
