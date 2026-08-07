"""Laptop-side RunBundle video remuxing and source-timed detection overlays."""

from __future__ import annotations

import argparse
import json
import subprocess
import warnings
from bisect import bisect_left
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.run_inspection import _parse_generated_manifest

MAX_DETECTION_TIME_DELTA_SEC = 0.10
TARGET_COLOR = (0, 255, 0)
NON_TARGET_COLOR = (0, 0, 255)


@dataclass(frozen=True)
class TimedDetections:
    timestamp_sec: float
    detections: tuple[dict[str, Any], ...]
    target_class: str = ""


def detection_header_timestamp(record: dict[str, Any]) -> float | None:
    stamp = record.get("header_stamp")
    if not isinstance(stamp, dict):
        return None
    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec")
    if isinstance(sec, bool) or isinstance(nanosec, bool) or not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    return sec + nanosec / 1_000_000_000


def read_timed_detections(path: Path, *, target_class: str = "") -> list[TimedDetections]:
    records: list[TimedDetections] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(record, dict):
            continue
        timestamp = detection_header_timestamp(record)
        detections = record.get("detections")
        if timestamp is None or not isinstance(detections, list):
            continue
        records.append(
            TimedDetections(timestamp, tuple(item for item in detections if isinstance(item, dict)), target_class)
        )
    return sorted(records, key=lambda item: item.timestamp_sec)


def load_video_start_time(timing_path: Path) -> float | None:
    """Load the robot realtime timestamp corresponding to video time zero."""
    try:
        timing = json.loads(timing_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    if not isinstance(timing, dict):
        return None
    timestamp_ns = timing.get("video_start_time_ns")
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
        return None
    return timestamp_ns / 1_000_000_000


def video_relative_to_robot_time(video_start_time_sec: float, video_relative_time_sec: float) -> float:
    return video_start_time_sec + video_relative_time_sec


def frame_robot_time(
    video_relative_time_sec: float,
    records: Sequence[TimedDetections],
    *,
    video_start_time_sec: float | None = None,
) -> float:
    if video_start_time_sec is not None:
        return video_relative_to_robot_time(video_start_time_sec, video_relative_time_sec)
    return records[0].timestamp_sec + video_relative_time_sec if records else video_relative_time_sec


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


def nearest_detections(
    records: Sequence[TimedDetections],
    *,
    frame_robot_time_sec: float,
    max_time_delta_sec: float = MAX_DETECTION_TIME_DELTA_SEC,
) -> TimedDetections | None:
    if not records:
        return None
    timestamps = [item.timestamp_sec for item in records]
    insertion_index = bisect_left(timestamps, frame_robot_time_sec)
    candidate_indices = (insertion_index - 1, insertion_index)
    valid_indices = [index for index in candidate_indices if 0 <= index < len(records)]
    if not valid_indices:
        return None
    nearest_index = min(valid_indices, key=lambda index: abs(timestamps[index] - frame_robot_time_sec))
    if abs(timestamps[nearest_index] - frame_robot_time_sec) > max_time_delta_sec:
        return None
    return records[nearest_index]


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
    video_start_time_sec = load_video_start_time(video_dir / "timing.json")
    if video_start_time_sec is None:
        warnings.warn(
            "video/timing.json is missing or invalid; using legacy first-detection-relative overlay timing",
            RuntimeWarning,
            stacklevel=2,
        )
    source_mp4 = video_dir / "source.mp4"
    remux_source_video(source_ts, source_mp4, runner=runner)
    render_overlay(
        source_mp4,
        video_dir / "overlay.mp4",
        read_timed_detections(
            detections_path,
            target_class=_target_class_from_manifest(run_dir / "manifest.yaml"),
        ),
        video_start_time_sec=video_start_time_sec,
        runner=runner,
        cv2_module=cv2_module,
    )


def render_overlay(
    source_mp4: Path,
    overlay_mp4: Path,
    records: Sequence[TimedDetections],
    *,
    video_start_time_sec: float | None = None,
    runner: Callable[..., Any] = subprocess.run,
    cv2_module: Any | None = None,
) -> None:
    rendered_overlay_mp4 = overlay_mp4.with_name(f"{overlay_mp4.stem}.rendered.mp4")
    try:
        render_overlay_frames(
            source_mp4,
            rendered_overlay_mp4,
            records,
            video_start_time_sec=video_start_time_sec,
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
    video_start_time_sec: float | None = None,
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
            video_relative_time_sec = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            frame_robot_time_sec = frame_robot_time(
                video_relative_time_sec,
                records,
                video_start_time_sec=video_start_time_sec,
            )
            match = nearest_detections(records, frame_robot_time_sec=frame_robot_time_sec)
            if match is not None:
                _draw_detections(cv2, frame, match.detections, target_class=match.target_class)
            if (output_width, output_height) != (width, height):
                frame = cv2.resize(frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
            writer.write(frame)
            frame_index += 1
    finally:
        capture.release()
        writer.release()


def _draw_detections(cv2: Any, frame: Any, detections: Sequence[dict[str, Any]], *, target_class: str = "") -> None:
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
        color = TARGET_COLOR if class_name == target_class else NON_TARGET_COLOR
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
