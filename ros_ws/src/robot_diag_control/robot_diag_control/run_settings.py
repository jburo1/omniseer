from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from robot_diag_control.run_artifacts import RunArtifactContext
from robot_diag_control.run_commands import (
    DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
    PREVIEW_ENCODER_LABELS,
    RUN_BACKEND_LABELS,
    RUN_TYPE_LABELS,
    RobotConnection,
    RunConfig,
    parse_run_classes,
    sanitize_run_id,
)

DEFAULT_REMOTE_REPO_ROOT = "/home/radxa/apps/omniseer"
DEFAULT_LOCAL_IMPORT_ROOT = "runs/imported"


@dataclass(frozen=True)
class RunFormValues:
    robot_host: str
    ssh_user: str
    remote_repo_root: str
    remote_runs_root: str
    local_import_root: str
    run_id: str
    backend_label: str
    run_type_label: str
    classes_text: str
    notes: str
    devcontainer_exec_template: str
    autonomy_bbox_area_min_ratio: str = "0.08"
    autonomy_approach_stop_area_percent: str = "125"
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
    detector_score_threshold: str = "0.25"
    detector_nms_iou_threshold: str = "0.45"
    detector_max_detections: str = "100"
    preview_encoder_label: str = "Rockchip hardware"
    record_video: bool = False
    record_rosbag: bool = False


@dataclass(frozen=True)
class RunFormSelection:
    run_id: str
    connection: RobotConnection
    run_config: RunConfig
    artifact_context: RunArtifactContext


def default_remote_runs_root(remote_repo_root: str) -> str:
    return f"{normalized_remote_repo_root(remote_repo_root)}/runs"


def selected_run_backend(selected_label: str) -> str:
    for backend, label in RUN_BACKEND_LABELS.items():
        if selected_label == label:
            return backend
    raise ValueError(f"unsupported run backend: {selected_label}")


def selected_run_type(selected_label: str) -> str:
    for run_type, label in RUN_TYPE_LABELS.items():
        if selected_label == label:
            return run_type
    raise ValueError(f"unsupported run type: {selected_label}")


def selected_preview_encoder(selected_label: str) -> str:
    for encoder, label in PREVIEW_ENCODER_LABELS.items():
        if selected_label == label:
            return encoder
    raise ValueError(f"unsupported preview encoder: {selected_label}")


def normalized_remote_repo_root(value: str) -> str:
    return value.strip().rstrip("/") or DEFAULT_REMOTE_REPO_ROOT


def normalized_remote_runs_root(value: str, *, remote_repo_root: str) -> str:
    return value.strip().rstrip("/") or f"{remote_repo_root}/runs"


def local_import_root(value: str, *, repo_root: Path) -> Path:
    path = Path(value.strip() or DEFAULT_LOCAL_IMPORT_ROOT).expanduser()
    if path.is_absolute():
        return path
    return repo_root / path


def resolved_run_id(raw_run_id: str, *, default_run_id: Callable[[], str]) -> str:
    run_id = sanitize_run_id(raw_run_id)
    return run_id or default_run_id()


def validated_unit_interval(value: str, *, name: str, default: str) -> str:
    normalized = value.strip() or default
    try:
        parsed = float(normalized)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number from 0.0 to 1.0") from exc
    if not math.isfinite(parsed) or parsed < 0.0 or parsed > 1.0:
        raise ValueError(f"{name} must be a number from 0.0 to 1.0")
    return normalized


def validated_positive_int(value: str, *, name: str, default: str) -> str:
    normalized = value.strip() or default
    try:
        parsed = int(normalized)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive integer") from exc
    if parsed <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return normalized


def validated_nonnegative_int(value: str, *, name: str, default: str) -> str:
    normalized = value.strip() or default
    try:
        parsed = int(normalized)
    except ValueError as exc:
        raise ValueError(f"{name} must be a non-negative integer") from exc
    if parsed < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return normalized


def validated_positive_unit_interval(value: str, *, name: str, default: str) -> str:
    normalized = validated_unit_interval(value, name=name, default=default)
    if float(normalized) <= 0.0:
        raise ValueError(f"{name} must be a number greater than 0.0 and no greater than 1.0")
    return normalized


def approach_stop_area_ratio(*, bbox_area_min_ratio: str, bbox_area_max_ratio: str, percent: str) -> str:
    minimum = float(
        validated_unit_interval(
            bbox_area_min_ratio,
            name="bbox minimum area",
            default="0.08",
        )
    )
    maximum = float(
        validated_unit_interval(
            bbox_area_max_ratio,
            name="bbox maximum area",
            default="0.35",
        )
    )
    if maximum <= minimum:
        raise ValueError("bbox maximum area must be greater than bbox minimum area")

    normalized_percent = percent.strip() or "125"
    try:
        multiplier = float(normalized_percent) / 100.0
    except ValueError as exc:
        raise ValueError("approach stop percentage must be a number of at least 100") from exc
    if not math.isfinite(multiplier) or multiplier < 1.0:
        raise ValueError("approach stop percentage must be a number of at least 100")

    ratio = minimum * multiplier
    if ratio > maximum:
        raise ValueError("approach stop percentage produces an area greater than bbox maximum area")
    return format(ratio, ".12g")


def resolve_run_form(
    values: RunFormValues,
    *,
    repo_root: Path,
    default_run_id: Callable[[], str],
) -> RunFormSelection:
    run_id = resolved_run_id(values.run_id, default_run_id=default_run_id)
    run_type = selected_run_type(values.run_type_label)
    remote_repo_root = normalized_remote_repo_root(values.remote_repo_root)
    remote_runs_root = normalized_remote_runs_root(values.remote_runs_root, remote_repo_root=remote_repo_root)
    connection = RobotConnection(
        host=values.robot_host.strip(),
        ssh_user=values.ssh_user.strip(),
        remote_repo_root=remote_repo_root,
        remote_runs_root=remote_runs_root,
    )
    autonomy_bbox_area_min_ratio = values.autonomy_bbox_area_min_ratio.strip() or "0.08"
    autonomy_bbox_area_max_ratio = values.autonomy_bbox_area_max_ratio.strip() or "0.35"
    run_config = RunConfig(
        run_id=run_id,
        backend=selected_run_backend(values.backend_label),
        classes=tuple(parse_run_classes(values.classes_text)),
        notes=values.notes.strip(),
        devcontainer_exec_template=values.devcontainer_exec_template.strip() or DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
        run_type=run_type,
        experiment_config=RUN_TYPE_LABELS[run_type],
        autonomy_bbox_area_min_ratio=autonomy_bbox_area_min_ratio,
        autonomy_approach_stop_area_ratio=approach_stop_area_ratio(
            bbox_area_min_ratio=autonomy_bbox_area_min_ratio,
            bbox_area_max_ratio=autonomy_bbox_area_max_ratio,
            percent=values.autonomy_approach_stop_area_percent,
        ),
        autonomy_bbox_area_max_ratio=autonomy_bbox_area_max_ratio,
        autonomy_forward_speed_m_s=values.autonomy_forward_speed_m_s.strip() or "0.05",
        autonomy_reverse_speed_m_s=values.autonomy_reverse_speed_m_s.strip() or "0.04",
        autonomy_stable_framed_frames=values.autonomy_stable_framed_frames.strip() or "10",
        autonomy_success_miss_tolerance_updates=validated_nonnegative_int(
            values.autonomy_success_miss_tolerance_updates,
            name="success miss tolerance updates",
            default="2",
        ),
        autonomy_proximity_stop_m=values.autonomy_proximity_stop_m.strip() or "0.30",
        autonomy_capture_timeout_sec=values.autonomy_capture_timeout_sec.strip() or "2.0",
        autonomy_min_target_confidence=validated_unit_interval(
            values.autonomy_min_target_confidence,
            name="minimum target confidence",
            default="0.50",
        ),
        autonomy_max_target_center_jump_ratio=validated_positive_unit_interval(
            values.autonomy_max_target_center_jump_ratio,
            name="maximum target center jump ratio",
            default="0.20",
        ),
        autonomy_evidence_interval_sec=values.autonomy_evidence_interval_sec.strip() or "0.25",
        detector_score_threshold=validated_unit_interval(
            values.detector_score_threshold,
            name="score threshold",
            default="0.25",
        ),
        detector_nms_iou_threshold=validated_unit_interval(
            values.detector_nms_iou_threshold,
            name="NMS IoU",
            default="0.45",
        ),
        detector_max_detections=validated_positive_int(
            values.detector_max_detections,
            name="max detections",
            default="100",
        ),
        preview_encoder=selected_preview_encoder(values.preview_encoder_label),
        record_video=values.record_video,
        record_rosbag=values.record_rosbag,
    )
    artifact_context = RunArtifactContext(
        repo_root=repo_root,
        connection=connection,
        local_import_root=local_import_root(values.local_import_root, repo_root=repo_root),
    )
    return RunFormSelection(
        run_id=run_id,
        connection=connection,
        run_config=run_config,
        artifact_context=artifact_context,
    )
