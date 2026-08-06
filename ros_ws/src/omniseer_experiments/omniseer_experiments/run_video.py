"""Build RunBundle videos from exact inference-frame associations."""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.run_inspection import _parse_generated_manifest

TARGET_COLOR = (0, 255, 0)
NON_TARGET_COLOR = (0, 0, 255)
DEFAULT_VIDEO_FPS = 30.0


@dataclass(frozen=True)
class RecordedFrame:
    """A model-input frame and detections produced from that exact frame."""

    image_path: Path
    detections: tuple[dict[str, Any], ...]
    remap_scale: float
    remap_pad_x: float
    remap_pad_y: float
    capture_ts_real_ns: int


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _recorded_image_path(video_dir: Path, value: object) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts or relative.suffix.lower() not in {".jpg", ".jpeg"}:
        return None
    return video_dir / relative


def read_recorded_frames(run_dir: Path) -> list[RecordedFrame]:
    """Read complete inference-frame records in capture order.

    `video/frames.jsonl` is written by the vision bridge from its consumer
    callback, where the model input, capture timestamp, and postprocessed
    detections are still one atomic association.
    """
    video_dir = run_dir / "video"
    records_path = video_dir / "frames.jsonl"
    frames: list[RecordedFrame] = []
    for line in records_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(record, dict):
            continue
        image_path = _recorded_image_path(video_dir, record.get("image_path"))
        detections = record.get("detections")
        remap = record.get("remap")
        timestamp = record.get("capture_ts_real_ns")
        if (
            image_path is None
            or not image_path.is_file()
            or not isinstance(detections, list)
            or not isinstance(remap, dict)
            or isinstance(timestamp, bool)
            or not isinstance(timestamp, int)
        ):
            continue
        scale = _number(remap.get("scale"))
        pad_x = _number(remap.get("pad_x"))
        pad_y = _number(remap.get("pad_y"))
        if scale is None or scale <= 0.0 or pad_x is None or pad_y is None:
            continue
        frames.append(
            RecordedFrame(
                image_path=image_path,
                detections=tuple(item for item in detections if isinstance(item, dict)),
                remap_scale=scale,
                remap_pad_x=pad_x,
                remap_pad_y=pad_y,
                capture_ts_real_ns=timestamp,
            )
        )
    return frames


def _target_class_from_manifest(manifest_path: Path) -> str:
    """Return the target class configured for this recorded run, if available."""
    try:
        manifest = _parse_generated_manifest(manifest_path.read_text(encoding="utf-8").splitlines())
    except (FileNotFoundError, OSError):
        return ""

    experiment = manifest.get("experiment")
    if isinstance(experiment, dict):
        parameters = experiment.get("parameters")
        if isinstance(parameters, dict):
            target = parameters.get("autonomy_target_class")
            if isinstance(target, str):
                return target
    launch = manifest.get("launch")
    if not isinstance(launch, dict):
        return ""
    args = launch.get("args")
    if not isinstance(args, list):
        return ""
    for arg in args:
        if not isinstance(arg, str):
            continue
        name, separator, value = arg.partition(":=")
        if name == "autonomy_target_class" and separator:
            return value
    return ""


def recorded_frame_rate(frames: Sequence[RecordedFrame]) -> float:
    """Use the median capture interval; source and overlay share this rate."""
    intervals = [
        (current.capture_ts_real_ns - previous.capture_ts_real_ns) / 1_000_000_000
        for previous, current in zip(frames, frames[1:])
        if current.capture_ts_real_ns > previous.capture_ts_real_ns
    ]
    if not intervals:
        return DEFAULT_VIDEO_FPS
    return max(1.0, min(120.0, 1.0 / statistics.median(intervals)))


def transcode_video(rendered_mp4: Path, output_mp4: Path, *, runner: Callable[..., Any] = subprocess.run) -> None:
    """Encode an OpenCV-rendered video for browser playback without changing cadence."""
    runner(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(rendered_mp4),
            "-map",
            "0:v:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_mp4),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def build_run_video(
    run_dir: Path, *, runner: Callable[..., Any] = subprocess.run, cv2_module: Any | None = None
) -> None:
    frames = read_recorded_frames(run_dir)
    if not frames:
        raise FileNotFoundError(f"no complete inference video frames found: {run_dir / 'video' / 'frames.jsonl'}")
    video_dir = run_dir / "video"
    video_dir.mkdir(exist_ok=True)
    render_videos(
        video_dir / "source.mp4",
        video_dir / "overlay.mp4",
        frames,
        target_class=_target_class_from_manifest(run_dir / "manifest.yaml"),
        runner=runner,
        cv2_module=cv2_module,
    )


def render_videos(
    source_mp4: Path,
    overlay_mp4: Path,
    frames: Sequence[RecordedFrame],
    *,
    target_class: str,
    runner: Callable[..., Any] = subprocess.run,
    cv2_module: Any | None = None,
) -> None:
    """Write source and overlay together so their frame streams are identical."""
    if not frames:
        raise ValueError("at least one recorded inference frame is required")
    source_rendered = source_mp4.with_name(f"{source_mp4.stem}.rendered.mp4")
    overlay_rendered = overlay_mp4.with_name(f"{overlay_mp4.stem}.rendered.mp4")
    try:
        render_video_frames(
            source_rendered,
            overlay_rendered,
            frames,
            target_class=target_class,
            cv2_module=cv2_module,
        )
        transcode_video(source_rendered, source_mp4, runner=runner)
        transcode_video(overlay_rendered, overlay_mp4, runner=runner)
    finally:
        source_rendered.unlink(missing_ok=True)
        overlay_rendered.unlink(missing_ok=True)


def render_video_frames(
    rendered_source_mp4: Path,
    rendered_overlay_mp4: Path,
    frames: Sequence[RecordedFrame],
    *,
    target_class: str,
    cv2_module: Any | None = None,
) -> None:
    cv2 = cv2_module or _import_cv2()
    source_writer: Any | None = None
    overlay_writer: Any | None = None
    try:
        for association in frames:
            source = cv2.imread(str(association.image_path), cv2.IMREAD_COLOR)
            if source is None:
                raise RuntimeError(f"failed to read recorded inference frame: {association.image_path}")
            height, width = source.shape[:2]
            if source_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                fps = recorded_frame_rate(frames)
                source_writer = cv2.VideoWriter(str(rendered_source_mp4), fourcc, fps, (width, height))
                overlay_writer = cv2.VideoWriter(str(rendered_overlay_mp4), fourcc, fps, (width, height))
                if not source_writer.isOpened() or not overlay_writer.isOpened():
                    raise RuntimeError("failed to create rendered RunBundle videos")
            overlay = source.copy()
            _draw_detections(cv2, overlay, association.detections, association, target_class=target_class)
            source_writer.write(source)
            overlay_writer.write(overlay)
    finally:
        if source_writer is not None:
            source_writer.release()
        if overlay_writer is not None:
            overlay_writer.release()


def _draw_detections(
    cv2: Any,
    frame: Any,
    detections: Sequence[dict[str, Any]],
    association: RecordedFrame,
    *,
    target_class: str,
) -> None:
    frame_height, frame_width = frame.shape[:2]
    for detection in detections:
        bbox = detection.get("bbox")
        if not isinstance(bbox, dict):
            continue
        source_x1, source_y1 = _number(bbox.get("x1")), _number(bbox.get("y1"))
        source_x2, source_y2 = _number(bbox.get("x2")), _number(bbox.get("y2"))
        if None in {source_x1, source_y1, source_x2, source_y2}:
            continue
        x1 = max(
            0,
            min(
                frame_width - 1,
                round(min(source_x1, source_x2) * association.remap_scale + association.remap_pad_x),
            ),
        )
        y1 = max(
            0,
            min(
                frame_height - 1,
                round(min(source_y1, source_y2) * association.remap_scale + association.remap_pad_y),
            ),
        )
        x2 = max(
            0,
            min(
                frame_width - 1,
                round(max(source_x1, source_x2) * association.remap_scale + association.remap_pad_x),
            ),
        )
        y2 = max(
            0,
            min(
                frame_height - 1,
                round(max(source_y1, source_y2) * association.remap_scale + association.remap_pad_y),
            ),
        )
        if x2 <= x1 or y2 <= y1:
            continue
        class_name = detection.get("class_name", "")
        class_label = class_name if isinstance(class_name, str) and class_name else str(detection.get("class_id", 0))
        score = _number(detection.get("score"))
        label = f"{class_label} {score:.2f}" if score is not None else class_label
        color = TARGET_COLOR if class_label == target_class else NON_TARGET_COLOR
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def _import_cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - host dependency
        raise RuntimeError("OpenCV is required to build RunBundle videos") from exc
    return cv2


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Build frame-aligned source and overlay videos for a RunBundle.")
    parser.add_argument("run_dir", help="path to a local RunBundle")
    args = parser.parse_args(argv)
    build_run_video(Path(args.run_dir))
