"""Static comparison report generation for physical trials and controlled replay evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.run_report import (
    _attr,
    _css,
    _format_duration,
    _format_optional_float,
    _format_optional_ms,
    _key_value_table,
    _last_matching_record,
    _mean_field,
    _p95_field,
    _percentile,
    _read_jsonl,
    _read_manifest,
    _relative_href,
    _section,
    _table,
    _target_loss_episodes,
)


@dataclass(frozen=True)
class _ModelSpec:
    label: str
    detections_filename: str
    variant: str
    precision: str


# This mirrors the canonical ordering and persisted JSONL names owned by the native comparator.
MODEL_SPECS = (
    _ModelSpec("v2-S FP", "v2s_fp.jsonl", "s", "fp"),
    _ModelSpec("v2-S INT8", "v2s_int8.jsonl", "s", "int8"),
    _ModelSpec("v2-M FP", "v2m_fp.jsonl", "m", "fp"),
    _ModelSpec("v2-M INT8", "v2m_int8.jsonl", "m", "int8"),
)


@dataclass(frozen=True)
class ReplayDetection:
    class_name: str
    score: float


@dataclass(frozen=True)
class ReplayFrame:
    frame_index: int
    timestamp_sec: float
    detections: tuple[ReplayDetection, ...]


@dataclass(frozen=True)
class ReplayComparison:
    name: str
    directory: Path
    provenance: dict[str, Any]
    streams: dict[str, tuple[ReplayFrame, ...]]


@dataclass(frozen=True)
class ClassReplayMetrics:
    frames_with_detection: int
    total_detections: int
    median_confidence: float | None
    max_confidence: float | None


@dataclass(frozen=True)
class ReplayMetrics:
    frames_evaluated: int
    total_detections: int
    unique_detected_classes: int
    median_confidence: float | None
    by_class: dict[str, ClassReplayMetrics]


@dataclass(frozen=True)
class SceneTruth:
    present: dict[str, tuple[tuple[int, int], ...]]
    absent: tuple[str, ...]


@dataclass(frozen=True)
class PresentTruthMetrics:
    visible_frames: int
    visible_frames_detected: int
    first_detection_delay: int | None
    detection_gaps: int


@dataclass(frozen=True)
class AbsentTruthMetrics:
    frames_falsely_containing: int
    total_false_detections: int
    max_false_confidence: float | None


@dataclass(frozen=True)
class TrialMetrics:
    spec: _ModelSpec
    run_dir: Path
    run_id: str
    manifest: dict[str, Any]
    values: dict[str, str]


@dataclass(frozen=True)
class ComparisonReportSummary:
    reference_run: Path
    output_path: Path
    comparison_name: str
    replay_frames: int
    has_scene_truth: bool
    has_coco80: bool


def parse_scene_truth(path: Path) -> SceneTruth:
    """Read the deliberately small frame-range scene-truth format."""

    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"scene truth file is missing: {path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"scene truth file is invalid: {path}: {exc}") from exc
    schema_version = value.get("schema_version") if isinstance(value, dict) else None
    if isinstance(schema_version, bool) or schema_version != 1:
        raise ValueError("scene truth schema_version must be 1")

    present_value = value.get("present", {})
    absent_value = value.get("absent", [])
    if not isinstance(present_value, dict):
        raise ValueError("scene truth present must be an object")
    if not isinstance(absent_value, list):
        raise ValueError("scene truth absent must be a list")

    present: dict[str, tuple[tuple[int, int], ...]] = {}
    for class_name, ranges_value in present_value.items():
        _validate_truth_class_name(class_name, "present")
        if not isinstance(ranges_value, list) or not ranges_value:
            raise ValueError(f"scene truth present[{class_name!r}] must contain frame ranges")
        ranges: list[tuple[int, int]] = []
        for item in ranges_value:
            if not isinstance(item, list) or len(item) != 2:
                raise ValueError(f"scene truth present[{class_name!r}] ranges must be [start, end]")
            start, end = item
            if (
                isinstance(start, bool)
                or isinstance(end, bool)
                or not isinstance(start, int)
                or not isinstance(end, int)
            ):
                raise ValueError(f"scene truth present[{class_name!r}] frame ranges must use integers")
            if start < 0 or end < 0:
                raise ValueError(f"scene truth present[{class_name!r}] frame ranges cannot be negative")
            if start > end:
                raise ValueError(f"scene truth present[{class_name!r}] range start must not exceed end")
            ranges.append((start, end))
        present[class_name] = tuple(ranges)

    absent: list[str] = []
    for class_name in absent_value:
        _validate_truth_class_name(class_name, "absent")
        if class_name in absent:
            raise ValueError(f"scene truth absent class is duplicated: {class_name!r}")
        absent.append(class_name)
    overlap = set(present).intersection(absent)
    if overlap:
        raise ValueError(f"scene truth classes cannot be both present and absent: {sorted(overlap)!r}")
    return SceneTruth(present=present, absent=tuple(absent))


def load_replay_comparison(reference_run: Path, comparison_name: str) -> ReplayComparison:
    """Load one required named comparison and verify its four frame-aligned streams."""

    if not _safe_comparison_name(comparison_name):
        raise ValueError("comparison name must contain 1-64 ASCII letters, digits, '_' or '-' only")
    directory = reference_run / "video" / "comparison" / comparison_name
    provenance_path = directory / "provenance.json"
    try:
        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"missing comparison provenance: {provenance_path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid comparison provenance: {provenance_path}: {exc}") from exc
    if not isinstance(provenance, dict):
        raise ValueError(f"invalid comparison provenance: {provenance_path}: expected object")
    _validate_comparison_artifacts(reference_run, directory, comparison_name, provenance)

    streams = {spec.label: _read_replay_jsonl(directory / spec.detections_filename) for spec in MODEL_SPECS}
    _validate_replay_alignment(streams)
    return ReplayComparison(
        name=comparison_name,
        directory=directory,
        provenance=provenance,
        streams=streams,
    )


def _validate_comparison_artifacts(
    reference_run: Path, directory: Path, comparison_name: str, provenance: dict[str, Any]
) -> None:
    """Require the complete, named output contract before reporting replay evidence."""

    provenance_path = directory / "provenance.json"
    if provenance.get("schema_version") != 2:
        raise ValueError(f"invalid comparison provenance schema: {provenance_path}")
    if provenance.get("comparison_name") != comparison_name:
        raise ValueError(f"comparison provenance name does not match {comparison_name!r}: {provenance_path}")

    expected_relative_dir = Path("video") / "comparison" / comparison_name
    _validate_hashed_artifact(
        reference_run,
        provenance.get("source"),
        expected_path=Path("video") / "source.ts",
        description="raw comparison source",
        provenance_path=provenance_path,
    )
    _validate_hashed_artifact(
        reference_run,
        provenance.get("output"),
        expected_path=expected_relative_dir / "comparison.mp4",
        description="comparison video",
        provenance_path=provenance_path,
    )

    expected_streams = [(expected_relative_dir / spec.detections_filename).as_posix() for spec in MODEL_SPECS]
    if provenance.get("detection_jsonl") != expected_streams:
        raise ValueError(f"comparison provenance lists unexpected replay JSONLs: {provenance_path}")


def _validate_hashed_artifact(
    reference_run: Path,
    value: object,
    *,
    expected_path: Path,
    description: str,
    provenance_path: Path,
) -> None:
    if not isinstance(value, dict) or value.get("path") != expected_path.as_posix():
        raise ValueError(f"comparison provenance has invalid {description} path: {provenance_path}")
    expected_sha256 = value.get("sha256")
    if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
        raise ValueError(f"comparison provenance has invalid {description} SHA-256: {provenance_path}")
    try:
        int(expected_sha256, 16)
    except ValueError as exc:
        raise ValueError(f"comparison provenance has invalid {description} SHA-256: {provenance_path}") from exc

    artifact = reference_run / expected_path
    if not artifact.is_file():
        raise ValueError(f"missing {description}: {artifact}")
    actual_sha256 = _sha256_file(artifact)
    if actual_sha256 != expected_sha256.lower():
        raise ValueError(f"{description} SHA-256 does not match provenance: {artifact}")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ValueError(f"unable to hash comparison artifact: {path}: {exc}") from exc
    return digest.hexdigest()


def aggregate_replay(frames: Sequence[ReplayFrame]) -> ReplayMetrics:
    class_frames: dict[str, set[int]] = defaultdict(set)
    class_scores: dict[str, list[float]] = defaultdict(list)
    total_detections = 0
    for frame in frames:
        for detection in frame.detections:
            total_detections += 1
            class_frames[detection.class_name].add(frame.frame_index)
            class_scores[detection.class_name].append(detection.score)
    by_class = {
        class_name: ClassReplayMetrics(
            frames_with_detection=len(class_frames[class_name]),
            total_detections=len(scores),
            median_confidence=statistics.median(scores),
            max_confidence=max(scores),
        )
        for class_name, scores in sorted(class_scores.items())
    }
    all_scores = [detection.score for frame in frames for detection in frame.detections]
    return ReplayMetrics(
        frames_evaluated=len(frames),
        total_detections=total_detections,
        unique_detected_classes=len(by_class),
        median_confidence=statistics.median(all_scores) if all_scores else None,
        by_class=by_class,
    )


def present_truth_metrics(frames: Sequence[ReplayFrame], truth: SceneTruth) -> dict[str, PresentTruthMetrics]:
    evaluated_indices = tuple(frame.frame_index for frame in frames)
    detections_by_frame = _detected_classes_by_frame(frames)
    result: dict[str, PresentTruthMetrics] = {}
    for class_name, ranges in truth.present.items():
        visible_indices = tuple(index for index in evaluated_indices if _in_ranges(index, ranges))
        detected_indices = tuple(index for index in visible_indices if class_name in detections_by_frame[index])
        first_delay = detected_indices[0] - visible_indices[0] if detected_indices and visible_indices else None
        result[class_name] = PresentTruthMetrics(
            visible_frames=len(visible_indices),
            visible_frames_detected=len(detected_indices),
            first_detection_delay=first_delay,
            detection_gaps=_count_detection_gaps(visible_indices, detections_by_frame, class_name, ranges),
        )
    return result


def absent_truth_metrics(frames: Sequence[ReplayFrame], truth: SceneTruth) -> dict[str, AbsentTruthMetrics]:
    result: dict[str, AbsentTruthMetrics] = {}
    for class_name in truth.absent:
        false_frames = 0
        scores: list[float] = []
        for frame in frames:
            matching = [detection.score for detection in frame.detections if detection.class_name == class_name]
            if matching:
                false_frames += 1
                scores.extend(matching)
        result[class_name] = AbsentTruthMetrics(
            frames_falsely_containing=false_frames,
            total_false_detections=len(scores),
            max_false_confidence=max(scores) if scores else None,
        )
    return result


def resolve_trial_metrics(trial_dirs: Sequence[Path]) -> dict[str, TrialMetrics]:
    if len(trial_dirs) != len(MODEL_SPECS):
        raise ValueError("exactly four physical --trial RunBundles are required")
    resolved: dict[str, TrialMetrics] = {}
    for run_dir in trial_dirs:
        manifest = _read_manifest(run_dir / "manifest.yaml")
        spec = _resolve_trial_spec(manifest)
        if spec is None:
            raise ValueError(f"cannot identify canonical detector configuration for trial: {run_dir}")
        if spec.label in resolved:
            raise ValueError(f"duplicate physical trial configuration: {spec.label}")
        resolved[spec.label] = _trial_metrics(run_dir, manifest, spec)
    missing = [spec.label for spec in MODEL_SPECS if spec.label not in resolved]
    if missing:
        raise ValueError(f"missing physical trial configuration(s): {', '.join(missing)}")
    return resolved


def write_comparison_report(
    reference_run: Path,
    *,
    trial_dirs: Sequence[Path],
    comparison_name: str = "task",
    truth_path: Path | None = None,
    overwrite: bool = False,
) -> ComparisonReportSummary:
    replay = load_replay_comparison(reference_run, comparison_name)
    truth = parse_scene_truth(truth_path) if truth_path is not None else None
    if truth is not None:
        _validate_truth_vocabulary(truth, replay)
    trials = resolve_trial_metrics(trial_dirs)
    output_path = reference_run / "report" / "comparison.html"
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"comparison report already exists: {output_path}; pass --overwrite to replace it")

    coco80: ReplayComparison | None = None
    if comparison_name != "coco80":
        try:
            coco80 = load_replay_comparison(reference_run, "coco80")
        except ValueError:
            coco80 = None
    html_text = _render_comparison_report(
        reference_run=reference_run,
        replay=replay,
        trials=trials,
        truth=truth,
        coco80=coco80,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding="utf-8")
    return ComparisonReportSummary(
        reference_run=reference_run,
        output_path=output_path,
        comparison_name=comparison_name,
        replay_frames=len(replay.streams[MODEL_SPECS[0].label]),
        has_scene_truth=truth is not None,
        has_coco80=coco80 is not None,
    )


def comparison_report_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate a static detector comparison report.")
    parser.add_argument("reference_run", help="controlled RunBundle containing video/comparison/<name>")
    parser.add_argument(
        "--trial",
        action="append",
        default=[],
        help="physical trial RunBundle (repeat exactly four times)",
    )
    parser.add_argument("--comparison", default="task", help="named controlled comparison (default: task)")
    parser.add_argument("--truth", help="optional scene-truth JSON using inclusive frame ranges")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing report/comparison.html")
    args = parser.parse_args(argv)
    try:
        summary = write_comparison_report(
            Path(args.reference_run),
            trial_dirs=tuple(Path(path) for path in args.trial),
            comparison_name=args.comparison,
            truth_path=Path(args.truth) if args.truth else None,
            overwrite=args.overwrite,
        )
    except (FileExistsError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    print(f"Comparison report: {summary.output_path}")
    print(f"Controlled replay frames: {summary.replay_frames}")
    print(f"Scene truth: {'yes' if summary.has_scene_truth else 'no'}")
    print(f"Exploratory COCO-80: {'yes' if summary.has_coco80 else 'no'}")


def _validate_truth_class_name(value: object, location: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"scene truth {location} class names must be non-empty strings")


def _validate_truth_vocabulary(truth: SceneTruth, replay: ReplayComparison) -> None:
    classes = replay.provenance.get("classes")
    if not isinstance(classes, list) or any(not isinstance(item, str) or not item for item in classes):
        raise ValueError("comparison provenance classes must be a non-empty list of strings")
    vocabulary = set(classes)
    unknown = sorted((set(truth.present) | set(truth.absent)) - vocabulary)
    if unknown:
        raise ValueError(
            "scene truth names class(es) absent from the selected comparison vocabulary: " + ", ".join(unknown)
        )


def _safe_comparison_name(name: str) -> bool:
    return bool(name) and len(name) <= 64 and all(char.isascii() and (char.isalnum() or char in "_-") for char in name)


def _read_replay_jsonl(path: Path) -> tuple[ReplayFrame, ...]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError as exc:
        raise ValueError(f"missing replay detections: {path}") from exc
    except OSError as exc:
        raise ValueError(f"unreadable replay detections: {path}: {exc}") from exc
    frames: list[ReplayFrame] = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            raise ValueError(f"invalid replay JSONL {path}:{line_number}: blank records are not allowed")
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid replay JSONL {path}:{line_number}: {exc.msg}") from exc
        if not isinstance(record, dict):
            raise ValueError(f"invalid replay JSONL {path}:{line_number}: expected object")
        frame_index = _nonnegative_integer(record.get("frame_index"), path, line_number, "frame_index")
        timestamp = _finite_number(record.get("timestamp_sec"), path, line_number, "timestamp_sec")
        detections_value = record.get("detections")
        if not isinstance(detections_value, list):
            raise ValueError(f"invalid replay JSONL {path}:{line_number}: detections must be a list")
        detections = tuple(_parse_replay_detection(item, path, line_number) for item in detections_value)
        frames.append(ReplayFrame(frame_index=frame_index, timestamp_sec=timestamp, detections=detections))
    if not frames:
        raise ValueError(f"replay detections contain no frames: {path}")
    return tuple(frames)


def _parse_replay_detection(value: object, path: Path, line_number: int) -> ReplayDetection:
    if not isinstance(value, dict):
        raise ValueError(f"invalid replay JSONL {path}:{line_number}: detection must be an object")
    class_name = value.get("class_name")
    if not isinstance(class_name, str) or not class_name:
        raise ValueError(f"invalid replay JSONL {path}:{line_number}: detection class_name is required")
    _nonnegative_integer(value.get("class_id"), path, line_number, "detection class_id")
    score = _finite_number(value.get("score"), path, line_number, "detection score")
    bbox = value.get("bbox")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError(f"invalid replay JSONL {path}:{line_number}: detection bbox must have four values")
    for coordinate in bbox:
        _finite_number(coordinate, path, line_number, "detection bbox coordinate")
    return ReplayDetection(class_name=class_name, score=score)


def _nonnegative_integer(value: object, path: Path, line_number: int, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"invalid replay JSONL {path}:{line_number}: {field} must be a non-negative integer")
    return value


def _finite_number(value: object, path: Path, line_number: int, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"invalid replay JSONL {path}:{line_number}: {field} must be finite")
    return float(value)


def _validate_replay_alignment(streams: dict[str, tuple[ReplayFrame, ...]]) -> None:
    reference_label = MODEL_SPECS[0].label
    reference = streams[reference_label]
    for label, frames in streams.items():
        if len(frames) != len(reference):
            raise ValueError(
                f"replay streams are not aligned: {label} has {len(frames)} frames; "
                f"{reference_label} has {len(reference)}"
            )
        for expected, actual in zip(reference, frames, strict=True):
            if actual.frame_index != expected.frame_index:
                raise ValueError(f"replay streams are not aligned: frame index differs for {label}")
            if actual.timestamp_sec != expected.timestamp_sec:
                raise ValueError(f"replay streams are not aligned: timestamp differs for {label}")


def _detected_classes_by_frame(frames: Sequence[ReplayFrame]) -> dict[int, set[str]]:
    return {frame.frame_index: {detection.class_name for detection in frame.detections} for frame in frames}


def _in_ranges(frame_index: int, ranges: Iterable[tuple[int, int]]) -> bool:
    return any(start <= frame_index <= end for start, end in ranges)


def _count_detection_gaps(
    visible_indices: Sequence[int],
    detections_by_frame: dict[int, set[str]],
    class_name: str,
    ranges: Sequence[tuple[int, int]],
) -> int:
    gaps = 0
    in_gap = False
    previous_index: int | None = None
    for index in visible_indices:
        starts_visibility_range = any(index == start for start, _ in ranges)
        contiguous_visible = previous_index is not None and index == previous_index + 1 and not starts_visibility_range
        if not contiguous_visible:
            in_gap = False
        if class_name not in detections_by_frame[index]:
            if not in_gap:
                gaps += 1
                in_gap = True
        else:
            in_gap = False
        previous_index = index
    return gaps


def _resolve_trial_spec(manifest: dict[str, Any]) -> _ModelSpec | None:
    model = manifest.get("model")
    if not isinstance(model, dict):
        return None
    variant = str(model.get("variant") or "").lower().replace("-", "").replace("_", "")
    precision = str(model.get("precision") or "").lower().replace("-", "").replace("_", "")
    if variant not in {"v2s", "v2m"} or precision not in {"fp", "fp16", "float", "float16", "int8", "i8"}:
        identity = " ".join(str(model.get(key) or "") for key in ("detector_model_path", "detector"))
        normalized = identity.lower().replace("-", "_")
        if "v2_s" in normalized or "v2s" in normalized:
            variant = "v2s"
        elif "v2_m" in normalized or "v2m" in normalized:
            variant = "v2m"
        if "int8" in normalized or "_i8" in normalized:
            precision = "int8"
        elif "fp16" in normalized or "_fp" in normalized:
            precision = "fp"
    normalized_precision = "fp" if precision in {"fp", "fp16", "float", "float16"} else precision
    return next(
        (spec for spec in MODEL_SPECS if variant == f"v2{spec.variant}" and normalized_precision == spec.precision),
        None,
    )


def _trial_metrics(run_dir: Path, manifest: dict[str, Any], spec: _ModelSpec) -> TrialMetrics:
    autonomy = _read_jsonl(run_dir / "autonomy.jsonl", required=False).records
    perf = _read_jsonl(run_dir / "perf.jsonl", required=False).records
    pipeline = _read_jsonl(run_dir / "pipeline_telemetry.jsonl", required=False).records
    system = _read_jsonl(run_dir / "system.jsonl", required=False).records
    terminal = _last_matching_record(autonomy, lambda record: record.get("state") in {"success", "failed"})
    first_detection = next((record for record in autonomy if record.get("event") == "first_detection"), None)
    first_success = next((record for record in autonomy if record.get("event") == "succeeded"), None)
    source_age_p95 = _p95_field(pipeline, "source_age_end_ns")
    thermal_values = [
        thermal.get("throttled")
        for record in system
        if isinstance((thermal := record.get("thermal")), dict) and isinstance(thermal.get("throttled"), bool)
    ]
    values = {
        "Outcome": str(terminal.get("state")) if terminal else "-",
        "Time to first detection": _format_duration(_number_or_none(first_detection, "time_sec")),
        "Time to success": _format_duration(_number_or_none(first_success, "time_sec")),
        "Target-loss episodes": str(len(_target_loss_episodes(autonomy))) if autonomy else "-",
        "Mean consumer FPS": _format_optional_unit(_mean_field(perf, "consumer_fps"), "fps"),
        "Inference p50": _format_optional_ms(_p50_field(perf, "last_infer_ms")),
        "Inference p95": _format_optional_ms(_p95_field(perf, "last_infer_ms")),
        "Source-age p95": _format_optional_ms(source_age_p95 / 1_000_000.0 if source_age_p95 is not None else None),
        "CPU p95": _format_optional_unit(_p95_field(system, "cpu_percent"), "%"),
        "Memory-used p95": _format_optional_unit(_p95_field(system, "memory_used_mb"), "MB"),
        "SoC temperature p95": _format_optional_unit(_p95_field(system, "soc_temp_c"), "C"),
        "Thermal throttling observed": "yes" if any(thermal_values) else "no" if thermal_values else "-",
    }
    run_id = manifest.get("run_id") if isinstance(manifest.get("run_id"), str) else run_dir.name
    return TrialMetrics(spec=spec, run_dir=run_dir, run_id=run_id, manifest=manifest, values=values)


def _number_or_none(record: dict[str, Any] | None, field: str) -> float | None:
    value = record.get(field) if record else None
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _p50_field(records: Sequence[dict[str, Any]], field: str) -> float | None:
    values = [
        float(value)
        for record in records
        if isinstance((value := record.get(field)), (int, float)) and not isinstance(value, bool)
    ]
    return _percentile(values, 50) if values else None


def _format_optional_unit(value: float | None, unit: str) -> str:
    return "-" if value is None else f"{_format_optional_float(value)} {unit}"


def _render_comparison_report(
    *,
    reference_run: Path,
    replay: ReplayComparison,
    trials: dict[str, TrialMetrics],
    truth: SceneTruth | None,
    coco80: ReplayComparison | None,
) -> str:
    report_dir = reference_run / "report"
    primary_metrics = {label: aggregate_replay(frames) for label, frames in replay.streams.items()}
    sections = [
        _overview_section(reference_run, replay, truth),
        _methodology_section(),
        _physical_trials_section(trials, report_dir),
        _controlled_replay_section(replay, primary_metrics, truth, report_dir),
    ]
    if coco80 is not None:
        coco_metrics = {label: aggregate_replay(frames) for label, frames in coco80.streams.items()}
        sections.append(_coco80_section(coco80, coco_metrics, report_dir))
    sections.append(_provenance_section(replay, trials))
    body = "\n".join(section.html_text for section in sections)
    return (
        '<!doctype html>\n<html lang="en">\n<head>\n'
        '  <meta charset="utf-8">\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1">\n'
        "  <title>Omniseer Detector Configuration Comparison</title>\n"
        f"  <style>{_css()}</style>\n"
        "</head>\n<body>\n  <main>\n"
        "    <h1>Omniseer Detector Configuration Comparison</h1>\n"
        f"{body}\n"
        "  </main>\n</body>\n</html>\n"
    )


def _overview_section(reference_run: Path, replay: ReplayComparison, truth: SceneTruth | None):
    classes = replay.provenance.get("classes")
    vocabulary_size = len(classes) if isinstance(classes, list) else 0
    frames = len(replay.streams[MODEL_SPECS[0].label])
    body = "<p>Four closed-loop robot trials + controlled same-video replay.</p>" + _key_value_table(
        [
            ("Reference RunBundle", reference_run.name),
            ("Selected comparison", replay.name),
            ("Replay frames", str(frames)),
            ("Class vocabulary size", str(vocabulary_size)),
            ("Manual scene truth", "supplied" if truth is not None else "not supplied"),
        ]
    )
    return _section("Overview", body, open_by_default=True)


def _methodology_section():
    body = (
        "<h3>Physical trials</h3><p>Measure end-to-end behavior with each detector in the autonomy loop.</p>"
        "<h3>Controlled replay</h3><p>Holds source video, classes, preprocessing, "
        "postprocessing, and thresholds fixed; "
        "only detector configuration changes.</p>"
        '<p class="table-note">The replay uses immutable source.ts, deterministic 8px repair in memory, '
        "the same corrected frame, and four resident RKNN detectors sequentially. Offline replay timing is not "
        "production runtime performance.</p>"
    )
    return _section("Methodology", body, open_by_default=True)


def _physical_trials_section(trials: dict[str, TrialMetrics], report_dir: Path):
    metric_names = tuple(next(iter(trials.values())).values)
    rows = [[metric, *(trials[spec.label].values[metric] for spec in MODEL_SPECS)] for metric in metric_names]
    links = []
    for spec in MODEL_SPECS:
        trial = trials[spec.label]
        index = trial.run_dir / "report" / "index.html"
        if index.is_file():
            href = _relative_href(report_dir, index)
            links.append(f'<li>{spec.label}: <a href="{_attr(href)}">{trial.run_id} single-run report</a></li>')
        else:
            links.append(f"<li>{spec.label}: {trial.run_id} (single-run report unavailable)</li>")
    body = (
        "<p>These are single physical trials per configuration and are presented as end-to-end case-study evidence, "
        "not population-level estimates.</p>"
        + _table(["Metric", *(spec.label for spec in MODEL_SPECS)], rows)
        + "<h3>Trial reports</h3><ul>"
        + "".join(links)
        + "</ul>"
    )
    return _section("Closed-loop Robot Trials", body, open_by_default=True)


def _controlled_replay_section(
    replay: ReplayComparison,
    metrics: dict[str, ReplayMetrics],
    truth: SceneTruth | None,
    report_dir: Path,
):
    video_path = replay.directory / "comparison.mp4"
    video_html = _comparison_video(video_path, report_dir, "Controlled replay video")
    body = video_html + "<h3>Per-model replay summary</h3>" + _replay_summary_table(metrics)
    body += _per_class_tables(metrics)
    if truth is None:
        body += (
            '<p class="table-note">No manual scene truth was supplied; controlled replay metrics are descriptive.</p>'
        )
    else:
        body += _truth_tables(replay, truth)
    return _section("Controlled Perception Replay", body, open_by_default=True)


def _coco80_section(coco80: ReplayComparison, metrics: dict[str, ReplayMetrics], report_dir: Path):
    body = (
        "<p>This section is descriptive only. Classes without explicit scene-truth labels are not scored as correct "
        "or incorrect.</p>"
        + _comparison_video(coco80.directory / "comparison.mp4", report_dir, "Exploratory COCO-80 replay video")
        + "<h3>Per-model replay summary</h3>"
        + _replay_summary_table(metrics)
        + _per_class_tables(metrics)
    )
    return _section("Exploratory COCO-80", body)


def _comparison_video(video_path: Path, report_dir: Path, title: str) -> str:
    if not video_path.is_file():
        return f"<p>{title} is unavailable.</p>"
    href = _relative_href(report_dir, video_path)
    return (
        f'<h3>{title}</h3><video controls preload="metadata" src="{_attr(href)}"></video>'
        f'<p><a href="{_attr(href)}">Open comparison video</a></p>'
    )


def _replay_summary_table(metrics: dict[str, ReplayMetrics]) -> str:
    rows = [
        [
            spec.label,
            str(metrics[spec.label].frames_evaluated),
            str(metrics[spec.label].total_detections),
            str(metrics[spec.label].unique_detected_classes),
            _format_optional_float(metrics[spec.label].median_confidence),
        ]
        for spec in MODEL_SPECS
    ]
    return _table(
        ["Configuration", "Frames evaluated", "Total detections", "Unique classes", "Median confidence"],
        rows,
    )


def _per_class_tables(metrics: dict[str, ReplayMetrics]) -> str:
    parts = ["<h3>Per-class detections</h3>"]
    for spec in MODEL_SPECS:
        summary = metrics[spec.label]
        rows = [
            [
                class_name,
                str(item.frames_with_detection),
                _fraction(item.frames_with_detection, summary.frames_evaluated),
                str(item.total_detections),
                _format_optional_float(item.median_confidence),
                _format_optional_float(item.max_confidence),
            ]
            for class_name, item in summary.by_class.items()
        ]
        parts.append(f"<h3>{spec.label}</h3>")
        parts.append(
            _table(
                [
                    "Class",
                    "Frames with >=1",
                    "Frame fraction",
                    "Total detections",
                    "Median confidence",
                    "Max confidence",
                ],
                rows,
            )
            if rows
            else "<p>No detections.</p>"
        )
    return "".join(parts)


def _truth_tables(replay: ReplayComparison, truth: SceneTruth) -> str:
    present_by_model = {label: present_truth_metrics(frames, truth) for label, frames in replay.streams.items()}
    absent_by_model = {label: absent_truth_metrics(frames, truth) for label, frames in replay.streams.items()}
    parts = ["<h3>Manual scene-truth visibility</h3>"]
    for spec in MODEL_SPECS:
        rows = [
            [
                class_name,
                str(value.visible_frames),
                str(value.visible_frames_detected),
                _fraction(value.visible_frames_detected, value.visible_frames),
                str(value.first_detection_delay) if value.first_detection_delay is not None else "-",
                str(value.detection_gaps),
            ]
            for class_name, value in present_by_model[spec.label].items()
        ]
        parts.append(f"<h3>{spec.label} present classes</h3>")
        parts.append(
            _table(
                [
                    "Class",
                    "Visible frames",
                    "Visible detected",
                    "Coverage",
                    "First delay (frames)",
                    "Detection gaps",
                ],
                rows,
            )
        )
        absent_rows = [
            [
                class_name,
                str(value.frames_falsely_containing),
                str(value.total_false_detections),
                _format_optional_float(value.max_false_confidence),
            ]
            for class_name, value in absent_by_model[spec.label].items()
        ]
        if absent_rows:
            parts.append(f"<h3>{spec.label} known-absent classes</h3>")
            parts.append(
                _table(
                    [
                        "Class",
                        "Frames falsely containing",
                        "Total false detections",
                        "Max false confidence",
                    ],
                    absent_rows,
                )
            )
    return "".join(parts)


def _provenance_section(replay: ReplayComparison, trials: dict[str, TrialMetrics]):
    source = replay.provenance.get("source")
    source_hash = source.get("sha256") if isinstance(source, dict) else "-"
    postprocess = replay.provenance.get("postprocess")
    postprocess_value = (
        f"score={postprocess.get('score_threshold')}, nms={postprocess.get('nms_iou_threshold')}, "
        f"max={postprocess.get('max_detections')}"
        if isinstance(postprocess, dict)
        else "-"
    )
    classes = replay.provenance.get("classes")
    class_list = ", ".join(str(item) for item in classes) if isinstance(classes, list) else "-"
    rows = [
        ("Reference source SHA-256", str(source_hash)),
        ("Comparison schema/name", f"{replay.provenance.get('schema_version', '-')} / {replay.name}"),
        ("Selected class list", class_list),
        ("Score/NMS/max detections", postprocess_value),
    ]
    model_rows = []
    models = replay.provenance.get("models")
    if isinstance(models, list):
        for model in models:
            if isinstance(model, dict):
                model_rows.append(
                    [
                        str(model.get("label", "-")),
                        str(model.get("artifact_name", "-")),
                        str(model.get("sha256", "-")),
                    ]
                )
    trial_rows = []
    for spec in MODEL_SPECS:
        trial = trials[spec.label]
        container = trial.manifest.get("container")
        image = container.get("image_digest") if isinstance(container, dict) else "-"
        trial_rows.append([spec.label, trial.run_id, str(trial.manifest.get("git_sha", "-")), str(image or "-")])
    body = _key_value_table(rows)
    if model_rows:
        body += "<h3>Controlled replay model artifacts</h3>" + _table(
            ["Configuration", "Artifact", "SHA-256"], model_rows
        )
    body += "<h3>Physical trial identities</h3>" + _table(
        ["Configuration", "RunBundle ID", "Git SHA", "Container identity"], trial_rows
    )
    return _section("Provenance", body)


def _fraction(numerator: int, denominator: int) -> str:
    return f"{(numerator / denominator) * 100.0:.1f}%" if denominator else "-"
