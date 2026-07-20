"""Offline annotation tooling for JPEG evidence frames in run bundles."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class AnnotationIssue:
    code: str
    message: str
    path: str = ""
    line: int | None = None

    def format(self) -> str:
        location = self.path
        if self.line is not None:
            location = f"{location}:{self.line}" if location else f"line {self.line}"
        return f"{self.code}: {self.message}" + (f" ({location})" if location else "")


@dataclass(frozen=True)
class AnnotationSummary:
    run_dir: Path
    annotated_dir: Path
    frames_written: int
    detections_drawn: int
    empty_frames_written: int
    issues: tuple[AnnotationIssue, ...]


def annotate_evidence(run_dir: Path, *, overwrite: bool = False) -> AnnotationSummary:
    cv2 = _import_cv2()
    evidence_path = run_dir / "evidence" / "evidence.jsonl"
    annotated_dir = run_dir / "evidence" / "annotated"
    records, issues = _read_evidence_records(evidence_path)
    if issues:
        return AnnotationSummary(run_dir, annotated_dir, 0, 0, 0, tuple(issues))

    annotated_dir.mkdir(parents=True, exist_ok=True)
    frames_written = 0
    detections_drawn = 0
    empty_frames_written = 0

    for line, record in records:
        if record.get("artifact_type") != "sampled_frame":
            continue
        frame_result = _annotate_record(
            cv2=cv2,
            run_dir=run_dir,
            annotated_dir=annotated_dir,
            record=record,
            line=line,
            overwrite=overwrite,
        )
        if frame_result.issues:
            issues.extend(frame_result.issues)
            continue
        frames_written += 1
        detections_drawn += frame_result.detections_drawn
        if frame_result.detections_drawn == 0:
            empty_frames_written += 1

    return AnnotationSummary(
        run_dir=run_dir,
        annotated_dir=annotated_dir,
        frames_written=frames_written,
        detections_drawn=detections_drawn,
        empty_frames_written=empty_frames_written,
        issues=tuple(issues),
    )


def format_annotation_summary(summary: AnnotationSummary) -> str:
    lines = [
        f"Run: {summary.run_dir}",
        f"Annotated evidence: {summary.annotated_dir}",
        f"Frames written: {summary.frames_written}",
        f"Detections drawn: {summary.detections_drawn}",
        f"Empty frames written: {summary.empty_frames_written}",
    ]
    if summary.issues:
        lines.append("Issues:")
        lines.extend(f"- {issue.format()}" for issue in summary.issues)
    return "\n".join(lines)


def annotate_evidence_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate annotated JPEGs from run-bundle evidence frames.")
    parser.add_argument("run_dir", help="path to a runs/<run_id> bundle")
    parser.add_argument("--overwrite", action="store_true", help="replace existing evidence/annotated JPEGs")
    args = parser.parse_args(argv)

    try:
        summary = annotate_evidence(Path(args.run_dir), overwrite=args.overwrite)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    print(format_annotation_summary(summary))
    if summary.issues:
        raise SystemExit(1)


@dataclass(frozen=True)
class _FrameAnnotationResult:
    detections_drawn: int = 0
    issues: tuple[AnnotationIssue, ...] = ()


def _import_cv2() -> Any:
    try:
        import cv2  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("OpenCV Python bindings are required to annotate evidence frames") from exc
    return cv2


def _read_evidence_records(path: Path) -> tuple[list[tuple[int, dict[str, Any]]], list[AnnotationIssue]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return [], [AnnotationIssue("missing_evidence_jsonl", "evidence.jsonl is missing", str(path))]
    except OSError as exc:
        return [], [AnnotationIssue("unreadable_evidence_jsonl", f"evidence.jsonl could not be read: {exc}", str(path))]

    records: list[tuple[int, dict[str, Any]]] = []
    issues: list[AnnotationIssue] = []
    for index, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            issues.append(
                AnnotationIssue(
                    "malformed_evidence_jsonl",
                    f"evidence.jsonl contains malformed JSONL: {exc.msg}",
                    str(path),
                    index,
                )
            )
            continue
        if not isinstance(record, dict):
            issues.append(
                AnnotationIssue("invalid_evidence_record", "evidence record is not an object", str(path), index)
            )
            continue
        records.append((index, record))
    return records, issues


def _annotate_record(
    *,
    cv2: Any,
    run_dir: Path,
    annotated_dir: Path,
    record: dict[str, Any],
    line: int,
    overwrite: bool,
) -> _FrameAnnotationResult:
    evidence_path = run_dir / "evidence" / "evidence.jsonl"
    image_path_value = record.get("image_path")
    relative_image_path = _valid_relative_image_path(image_path_value)
    if relative_image_path is None:
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue(
                    "invalid_evidence_path",
                    "sampled evidence image_path must be a relative JPEG path inside the run bundle",
                    str(evidence_path),
                    line,
                ),
            )
        )

    source_path = run_dir / relative_image_path
    if not source_path.is_file():
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue(
                    "missing_evidence_image",
                    f"sampled evidence image is missing: {image_path_value}",
                    str(evidence_path),
                    line,
                ),
            )
        )

    output_path = annotated_dir / relative_image_path.name
    if output_path.exists() and not overwrite:
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue(
                    "annotated_image_exists",
                    f"annotated image already exists: {output_path}; pass --overwrite to replace it",
                    str(output_path),
                ),
            )
        )

    frame = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
    if frame is None:
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue(
                    "unreadable_evidence_image",
                    f"could not read evidence image: {source_path}",
                    str(source_path),
                ),
            )
        )

    dimensions_issue = _validate_dimensions(record, frame_shape=frame.shape, evidence_path=evidence_path, line=line)
    if dimensions_issue is not None:
        return _FrameAnnotationResult(issues=(dimensions_issue,))

    transform = _read_remap_transform(record, evidence_path=evidence_path, line=line)
    if isinstance(transform, AnnotationIssue):
        return _FrameAnnotationResult(issues=(transform,))

    detections = record.get("detections")
    if not isinstance(detections, list):
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue(
                    "invalid_evidence_record",
                    "sampled evidence record detections must be a list",
                    str(evidence_path),
                    line,
                ),
            )
        )

    detections_drawn = _draw_detections(cv2, frame, detections, transform, evidence_path=evidence_path, line=line)
    if isinstance(detections_drawn, AnnotationIssue):
        return _FrameAnnotationResult(issues=(detections_drawn,))
    if detections_drawn == 0:
        _draw_empty_marker(cv2, frame)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), frame):
        return _FrameAnnotationResult(
            issues=(
                AnnotationIssue("write_failed", f"could not write annotated image: {output_path}", str(output_path)),
            )
        )
    return _FrameAnnotationResult(detections_drawn=detections_drawn)


def _valid_relative_image_path(value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.suffix.lower() not in {".jpg", ".jpeg"}:
        return None
    return path


def _read_remap_transform(
    record: dict[str, Any], *, evidence_path: Path, line: int
) -> tuple[float, float, float] | AnnotationIssue:
    remap = record.get("remap")
    if not isinstance(remap, dict):
        return AnnotationIssue(
            "invalid_evidence_record",
            "sampled evidence record is missing remap",
            str(evidence_path),
            line,
        )
    scale = _float_value(remap.get("scale"))
    pad_x = _float_value(remap.get("pad_x"))
    pad_y = _float_value(remap.get("pad_y"))
    if scale is None or scale <= 0.0 or pad_x is None or pad_y is None:
        return AnnotationIssue(
            "invalid_evidence_record",
            "sampled evidence record has invalid remap scale/pad_x/pad_y",
            str(evidence_path),
            line,
        )
    return scale, pad_x, pad_y


def _validate_dimensions(
    record: dict[str, Any], *, frame_shape: tuple[int, ...], evidence_path: Path, line: int
) -> AnnotationIssue | None:
    model_input = record.get("model_input")
    source_image = record.get("source_image")
    if not isinstance(model_input, dict) or not _is_positive_int(model_input.get("width")) or not _is_positive_int(
        model_input.get("height")
    ):
        return AnnotationIssue(
            "invalid_evidence_record",
            "sampled evidence record has invalid model_input width/height",
            str(evidence_path),
            line,
        )
    if not isinstance(source_image, dict) or not _is_positive_int(source_image.get("width")) or not _is_positive_int(
        source_image.get("height")
    ):
        return AnnotationIssue(
            "invalid_evidence_record",
            "sampled evidence record has invalid source_image width/height",
            str(evidence_path),
            line,
        )
    image_height, image_width = frame_shape[:2]
    if image_width != model_input["width"] or image_height != model_input["height"]:
        return AnnotationIssue(
            "invalid_evidence_image",
            "evidence image dimensions do not match model_input metadata",
            str(evidence_path),
            line,
        )
    return None


def _draw_detections(
    cv2: Any,
    frame: Any,
    detections: Sequence[Any],
    transform: tuple[float, float, float],
    *,
    evidence_path: Path,
    line: int,
) -> int | AnnotationIssue:
    drawn = 0
    for index, detection in enumerate(detections):
        if not isinstance(detection, dict):
            return AnnotationIssue(
                "invalid_detection",
                f"detection {index} is not an object",
                str(evidence_path),
                line,
            )
        box = _model_box(detection.get("bbox"), transform)
        if box is None:
            return AnnotationIssue(
                "invalid_detection",
                f"detection {index} is missing source-space x1/y1/x2/y2 bbox",
                str(evidence_path),
                line,
            )
        color = _class_color(str(detection.get("class_name", detection.get("class_id", index))))
        score = _float_value(detection.get("score"))
        label = str(detection.get("class_name") or f"class_{detection.get('class_id', index)}")
        if score is not None:
            label = f"{label} {score:.2f}"
        _draw_box_label(cv2, frame, box, label, color)
        drawn += 1
    return drawn


def _model_box(value: Any, transform: tuple[float, float, float]) -> tuple[int, int, int, int] | None:
    if not isinstance(value, dict):
        return None
    x1 = _float_value(value.get("x1"))
    y1 = _float_value(value.get("y1"))
    x2 = _float_value(value.get("x2"))
    y2 = _float_value(value.get("y2"))
    if x1 is None or y1 is None or x2 is None or y2 is None:
        return None
    scale, pad_x, pad_y = transform
    return (
        round((min(x1, x2) * scale) + pad_x),
        round((min(y1, y2) * scale) + pad_y),
        round((max(x1, x2) * scale) + pad_x),
        round((max(y1, y2) * scale) + pad_y),
    )


def _draw_box_label(
    cv2: Any,
    frame: Any,
    box: tuple[int, int, int, int],
    label: str,
    color: tuple[int, int, int],
) -> None:
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = _clamp_box(box, width=width, height=height)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 1
    (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    label_x = max(0, min(x1, width - text_w - 4))
    label_y = max(text_h + baseline + 4, y1)
    background_top = max(0, label_y - text_h - baseline - 4)
    cv2.rectangle(frame, (label_x, background_top), (label_x + text_w + 4, label_y + baseline), color, -1)
    cv2.putText(frame, label, (label_x + 2, label_y - 3), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)


def _draw_empty_marker(cv2: Any, frame: Any) -> None:
    label = "no detections"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    x = 12
    y = 12 + text_h
    cv2.rectangle(frame, (8, 8), (x + text_w + 8, y + baseline + 6), (80, 80, 80), -1)
    cv2.putText(frame, label, (x, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)


def _clamp_box(box: tuple[int, int, int, int], *, width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(0, min(width - 1, x2))
    y2 = max(0, min(height - 1, y2))
    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)
    return x1, y1, x2, y2


def _class_color(key: str) -> tuple[int, int, int]:
    seed = 0
    for char in key:
        seed = (seed * 33 + ord(char)) & 0xFFFFFFFF
    palette = (
        (42, 157, 244),
        (76, 175, 80),
        (233, 150, 43),
        (156, 92, 214),
        (39, 188, 203),
        (196, 70, 70),
    )
    return palette[seed % len(palette)]


def _float_value(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return float(value)
    return None


def _is_positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0
