"""RunBundle video generation from frame-associated inference evidence."""

from __future__ import annotations

import argparse
import json
import subprocess
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.run_inspection import _parse_generated_manifest

TARGET_COLOR = (0, 255, 0)
NON_TARGET_COLOR = (0, 0, 255)


@dataclass(frozen=True)
class InferenceFrame:
    """One model-input image and the detections produced from that same image."""

    image_path: Path
    detections: tuple[dict[str, Any], ...]
    remap_scale: float
    remap_pad_x: float
    remap_pad_y: float
    target_class: str = ""


def read_inference_frames(run_dir: Path) -> list[InferenceFrame]:
    """Read only complete, self-contained inference-frame records.

    An incomplete record can occur if a run ends while the evidence writer still
    has work queued.  It is deliberately omitted instead of borrowing a
    detection from another frame.
    """
    evidence_path = run_dir / "evidence" / "evidence.jsonl"
    target_class = _target_class_from_manifest(run_dir / "manifest.yaml")
    frames: list[InferenceFrame] = []
    for line in evidence_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(record, dict) or record.get("artifact_type") != "sampled_frame":
            continue
        image_path = _evidence_image_path(run_dir, record.get("image_path"))
        detections = record.get("detections")
        remap = record.get("remap")
        if (
            image_path is None
            or not image_path.is_file()
            or not isinstance(detections, list)
            or not isinstance(remap, dict)
        ):
            continue
        scale = _number(remap.get("scale"))
        pad_x = _number(remap.get("pad_x"))
        pad_y = _number(remap.get("pad_y"))
        if scale is None or scale <= 0.0 or pad_x is None or pad_y is None:
            continue
        frames.append(
            InferenceFrame(
                image_path=image_path,
                detections=tuple(item for item in detections if isinstance(item, dict)),
                remap_scale=scale,
                remap_pad_x=pad_x,
                remap_pad_y=pad_y,
                target_class=target_class,
            )
        )
    return frames


def _target_class_from_manifest(manifest_path: Path) -> str:
    """Return the target class configured for this recorded run, if available."""
    try:
        manifest = _parse_generated_manifest(manifest_path.read_text(encoding="utf-8").splitlines())
    except (FileNotFoundError, OSError):
        return ""

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


def _evidence_image_path(run_dir: Path, value: object) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts or relative.suffix.lower() not in {".jpg", ".jpeg"}:
        return None
    return run_dir / relative


def remux_source_video(source_ts: Path, source_mp4: Path, *, runner: Callable[..., Any] = subprocess.run) -> None:
    runner(
        ["ffmpeg", "-y", "-i", str(source_ts), "-c", "copy", str(source_mp4)],
        check=True,
        capture_output=True,
        text=True,
    )


def transcode_overlay_video(
    rendered_overlay_mp4: Path, overlay_mp4: Path, *, runner: Callable[..., Any] = subprocess.run
) -> None:
    """Encode an OpenCV-rendered overlay for HTML5 video playback."""
    runner(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(rendered_overlay_mp4),
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
            str(overlay_mp4),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def build_run_video(
    run_dir: Path, *, runner: Callable[..., Any] = subprocess.run, cv2_module: Any | None = None
) -> None:
    video_dir = run_dir / "video"
    source_ts = video_dir / "source.ts"
    video_dir.mkdir(exist_ok=True)
    if source_ts.is_file():
        remux_source_video(source_ts, video_dir / "source.mp4", runner=runner)

    frames = read_inference_frames(run_dir)
    if not frames:
        raise FileNotFoundError("no complete inference evidence frames found")
    render_overlay(video_dir / "overlay.mp4", frames, runner=runner, cv2_module=cv2_module)


def render_overlay(
    overlay_mp4: Path,
    frames: Sequence[InferenceFrame],
    *,
    runner: Callable[..., Any] = subprocess.run,
    cv2_module: Any | None = None,
) -> None:
    rendered_overlay_mp4 = overlay_mp4.with_name(f"{overlay_mp4.stem}.rendered.mp4")
    try:
        render_overlay_frames(rendered_overlay_mp4, frames, cv2_module=cv2_module)
        transcode_overlay_video(rendered_overlay_mp4, overlay_mp4, runner=runner)
    finally:
        rendered_overlay_mp4.unlink(missing_ok=True)


def render_overlay_frames(
    rendered_overlay_mp4: Path,
    frames: Sequence[InferenceFrame],
    *,
    max_output_width: int = 640,
    output_fps: float = 1.0,
    cv2_module: Any | None = None,
) -> None:
    if not frames:
        raise ValueError("at least one inference frame is required")
    cv2 = cv2_module or _import_cv2()
    writer = None
    try:
        for frame_record in frames:
            frame = cv2.imread(str(frame_record.image_path), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            height, width = frame.shape[:2]
            output_width = min(width, max_output_width)
            output_height = max(2, round(height * output_width / width / 2) * 2)
            if writer is None:
                writer = cv2.VideoWriter(
                    str(rendered_overlay_mp4),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    output_fps,
                    (output_width, output_height),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"failed to create overlay video: {rendered_overlay_mp4}")
            _draw_detections(cv2, frame, frame_record.detections, frame_record)
            if (output_width, output_height) != (width, height):
                frame = cv2.resize(frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
            writer.write(frame)
        if writer is None:
            raise RuntimeError("no readable inference evidence frames found")
    finally:
        if writer is not None:
            writer.release()


def _draw_detections(cv2: Any, frame: Any, detections: Sequence[dict[str, Any]], association: InferenceFrame) -> None:
    frame_height, frame_width = frame.shape[:2]
    for detection in detections:
        bbox = detection.get("bbox")
        if not isinstance(bbox, dict):
            continue
        source_x1, source_y1 = _number(bbox.get("x1")), _number(bbox.get("y1"))
        source_x2, source_y2 = _number(bbox.get("x2")), _number(bbox.get("y2"))
        if None in {source_x1, source_y1, source_x2, source_y2}:
            continue
        # Detections are source-camera pixels; the saved image is the model input.
        x1 = max(
            0,
            min(frame_width - 1, round(min(source_x1, source_x2) * association.remap_scale + association.remap_pad_x)),
        )
        y1 = max(
            0,
            min(frame_height - 1, round(min(source_y1, source_y2) * association.remap_scale + association.remap_pad_y)),
        )
        x2 = max(
            0,
            min(frame_width - 1, round(max(source_x1, source_x2) * association.remap_scale + association.remap_pad_x)),
        )
        y2 = max(
            0,
            min(frame_height - 1, round(max(source_y1, source_y2) * association.remap_scale + association.remap_pad_y)),
        )
        if x2 <= x1 or y2 <= y1:
            continue
        class_id = detection.get("class_id", 0)
        class_name = str(detection.get("class_name", "") or class_id)
        score = _number(detection.get("score")) or 0.0
        color = TARGET_COLOR if class_name == association.target_class else NON_TARGET_COLOR
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            f"{class_name} {score:.2f}",
            (x1, max(16, y1)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )


def _number(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _import_cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("overlay rendering requires python3-opencv") from exc
    return cv2


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Build source and frame-aligned detection-overlay videos for a RunBundle."
    )
    parser.add_argument("run_dir", help="path to a local RunBundle")
    args = parser.parse_args(argv)
    build_run_video(Path(args.run_dir))


if __name__ == "__main__":
    main()
