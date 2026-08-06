"""Laptop-side RunBundle video remuxing and approximate detection overlays."""

from __future__ import annotations

import argparse
import json
import subprocess
from bisect import bisect_right
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

MAX_DETECTION_AGE_SEC = 0.5


@dataclass(frozen=True)
class TimedDetections:
    timestamp_sec: float
    detections: tuple[dict[str, Any], ...]


def detection_header_timestamp(record: dict[str, Any]) -> float | None:
    stamp = record.get("header_stamp")
    if not isinstance(stamp, dict):
        return None
    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec")
    if isinstance(sec, bool) or isinstance(nanosec, bool) or not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    return sec + nanosec / 1_000_000_000


def read_timed_detections(path: Path) -> list[TimedDetections]:
    records: list[TimedDetections] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if not isinstance(record, dict):
            continue
        timestamp = detection_header_timestamp(record)
        detections = record.get("detections")
        if timestamp is None or not isinstance(detections, list):
            continue
        records.append(TimedDetections(timestamp, tuple(item for item in detections if isinstance(item, dict))))
    return records


def nearest_detections(
    records: Sequence[TimedDetections],
    *,
    video_time_sec: float,
    max_age_sec: float = MAX_DETECTION_AGE_SEC,
) -> TimedDetections | None:
    if not records:
        return None
    first_timestamp = records[0].timestamp_sec
    timestamps = [item.timestamp_sec - first_timestamp for item in records]
    index = bisect_right(timestamps, video_time_sec) - 1
    if index < 0 or video_time_sec - timestamps[index] > max_age_sec:
        return None
    return records[index]


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
    if not source_ts.is_file():
        raise FileNotFoundError(f"recorded source video not found: {source_ts}")
    detections_path = run_dir / "detections.jsonl"
    if not detections_path.is_file():
        raise FileNotFoundError(f"detections not found: {detections_path}")

    video_dir.mkdir(exist_ok=True)
    source_mp4 = video_dir / "source.mp4"
    remux_source_video(source_ts, source_mp4, runner=runner)
    render_overlay(
        source_mp4,
        video_dir / "overlay.mp4",
        read_timed_detections(detections_path),
        runner=runner,
        cv2_module=cv2_module,
    )


def render_overlay(
    source_mp4: Path,
    overlay_mp4: Path,
    records: Sequence[TimedDetections],
    *,
    runner: Callable[..., Any] = subprocess.run,
    cv2_module: Any | None = None,
) -> None:
    rendered_overlay_mp4 = overlay_mp4.with_name(f"{overlay_mp4.stem}.rendered.mp4")
    try:
        render_overlay_frames(
            source_mp4,
            rendered_overlay_mp4,
            records,
            cv2_module=cv2_module,
        )
        transcode_overlay_video(rendered_overlay_mp4, overlay_mp4, runner=runner)
    finally:
        rendered_overlay_mp4.unlink(missing_ok=True)


def render_overlay_frames(
    source_mp4: Path,
    rendered_overlay_mp4: Path,
    records: Sequence[TimedDetections],
    *,
    max_output_width: int = 640,
    max_output_fps: float = 30.0,
    cv2_module: Any | None = None,
) -> None:
    cv2 = cv2_module or _import_cv2()
    capture = cv2.VideoCapture(str(source_mp4))
    if not capture.isOpened():
        raise RuntimeError(f"failed to open remuxed video: {source_mp4}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    output_width = min(width, max_output_width)
    output_height = max(2, round(height * output_width / width / 2) * 2)
    frame_step = max(1, round(fps / max_output_fps))
    output_fps = fps / frame_step
    writer = cv2.VideoWriter(
        str(rendered_overlay_mp4),
        cv2.VideoWriter_fourcc(*"mp4v"),
        output_fps,
        (output_width, output_height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"failed to create overlay video: {rendered_overlay_mp4}")
    try:
        frame_index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index % frame_step:
                frame_index += 1
                continue
            match = nearest_detections(records, video_time_sec=capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
            if match is not None:
                _draw_detections(cv2, frame, match.detections)
            if (output_width, output_height) != (width, height):
                frame = cv2.resize(frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
            writer.write(frame)
            frame_index += 1
    finally:
        capture.release()
        writer.release()


def _draw_detections(cv2: Any, frame: Any, detections: Sequence[dict[str, Any]]) -> None:
    frame_height, frame_width = frame.shape[:2]
    for detection in detections:
        bbox = detection.get("bbox")
        if not isinstance(bbox, dict):
            continue
        center_x, center_y = _number(bbox.get("center_x")), _number(bbox.get("center_y"))
        size_x, size_y = _number(bbox.get("size_x")), _number(bbox.get("size_y"))
        if None in {center_x, center_y, size_x, size_y}:
            continue
        # DetectionArray coordinates are remapped by vision into source-camera pixels.
        x1 = max(0, min(frame_width - 1, round(center_x - size_x / 2)))
        y1 = max(0, min(frame_height - 1, round(center_y - size_y / 2)))
        x2 = max(0, min(frame_width - 1, round(center_x + size_x / 2)))
        y2 = max(0, min(frame_height - 1, round(center_y + size_y / 2)))
        if x2 <= x1 or y2 <= y1:
            continue
        class_id = detection.get("class_id", 0)
        class_name = str(detection.get("class_name", "") or class_id)
        score = _number(detection.get("score")) or 0.0
        color = (0, 255, 0)
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
    parser = argparse.ArgumentParser(description="Build source and detection-overlay videos for a RunBundle.")
    parser.add_argument("run_dir", help="path to a local RunBundle")
    args = parser.parse_args(argv)
    build_run_video(Path(args.run_dir))


if __name__ == "__main__":
    main()
