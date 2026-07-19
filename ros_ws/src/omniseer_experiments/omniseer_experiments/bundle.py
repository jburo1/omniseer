"""Pure run-bundle writing and summary logic."""

from __future__ import annotations

import json
import os
import shutil
import statistics
import subprocess
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
DEFAULT_DETECTIONS_TOPIC = "/yolo/detections"
DEFAULT_PERF_TOPIC = "/vision/perf"

ERROR_FIELDS = (
    "no_writable_buffer",
    "capture_retryable",
    "capture_fatal",
    "preprocess",
    "infer",
)


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def isoformat_utc(value: datetime) -> str:
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc).isoformat()


def default_run_id(now: datetime | None = None) -> str:
    value = now or utc_now()
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def resolve_git_sha(repo_root: Path | None = None) -> str:
    cwd = repo_root or Path.cwd()
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return "unknown"
    return result.stdout.strip() or "unknown"


@dataclass(frozen=True)
class RunBundleConfig:
    run_id: str
    out_dir: Path
    classes: tuple[str, ...] = ()
    notes: str = ""
    overwrite: bool = False
    project: str = "omniseer"
    robot: str = "omniseer"
    sbc: str = "ROCK 5B+"
    ros_distro: str = field(default_factory=lambda: os.environ.get("ROS_DISTRO", "unknown"))
    git_sha: str = field(default_factory=resolve_git_sha)
    detector: str = "yolo-world-rknn"
    vision_params_file: str = ""
    detector_model_path: str = ""
    clip_model_path: str = ""
    clip_vocab_path: str = ""
    classes_path: str = ""
    detections_topic: str = DEFAULT_DETECTIONS_TOPIC
    perf_topic: str = DEFAULT_PERF_TOPIC


class SummaryAccumulator:
    """Track summary stats without rereading completed JSONL files."""

    def __init__(self, run_id: str) -> None:
        self.run_id = run_id
        self.detection_message_count = 0
        self.perf_message_count = 0
        self.detections_by_class: Counter[str] = Counter()
        self._confidence_by_class: dict[str, list[float]] = defaultdict(list)
        self._producer_fps: list[float] = []
        self._consumer_fps: list[float] = []
        self._infer_ms: list[float] = []
        self._errors: dict[str, int] = {field_name: 0 for field_name in ERROR_FIELDS}
        self._dropped_records: Counter[str] = Counter()

    def add_detection_record(self, record: dict[str, Any]) -> None:
        self.detection_message_count += 1
        for detection in record.get("detections", []):
            if not isinstance(detection, dict):
                continue
            class_key = _class_key(detection)
            self.detections_by_class[class_key] += 1
            score = _as_float(detection.get("score"))
            if score is not None:
                self._confidence_by_class[class_key].append(score)

    def add_perf_record(self, record: dict[str, Any]) -> None:
        self.perf_message_count += 1
        _append_float(self._producer_fps, record.get("producer_fps"))
        _append_float(self._consumer_fps, record.get("consumer_fps"))
        _append_float(self._infer_ms, record.get("last_infer_ms"))

        error_counts = record.get("error_counts", {})
        if isinstance(error_counts, dict):
            for field_name in ERROR_FIELDS:
                value = _as_int(error_counts.get(field_name))
                if value is not None:
                    self._errors[field_name] = max(self._errors[field_name], value)

    def record_drop(self, stream: str, count: int = 1) -> None:
        if count > 0:
            self._dropped_records[stream] += count

    def build_summary(self, duration_sec: float) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "duration_sec": duration_sec,
            "message_counts": {
                "detections": self.detection_message_count,
                "perf": self.perf_message_count,
            },
            "detections_by_class": dict(sorted(self.detections_by_class.items())),
            "confidence_by_class": {
                class_name: _describe_float(values)
                for class_name, values in sorted(self._confidence_by_class.items())
            },
            "perf": {
                "producer_fps_mean": _mean_or_zero(self._producer_fps),
                "consumer_fps_mean": _mean_or_zero(self._consumer_fps),
                "infer_ms_mean": _mean_or_zero(self._infer_ms),
                "infer_ms_p95": _p95_or_zero(self._infer_ms),
            },
            "errors": dict(self._errors),
            "dropped_records": dict(sorted(self._dropped_records.items())),
        }


class RunBundleWriter:
    """Create and finalize a local perception run bundle."""

    def __init__(self, config: RunBundleConfig, *, started_at: datetime | None = None) -> None:
        self.config = config
        self.started_at = started_at or utc_now()
        self.ended_at: datetime | None = None
        self.summary = SummaryAccumulator(config.run_id)
        self._closed = False

        self._prepare_run_dir()
        self.evidence_dir.mkdir(exist_ok=True)
        self._write_manifest()
        self._detections_handle = self.detections_path.open("a", encoding="utf-8")
        self._perf_handle = self.perf_path.open("a", encoding="utf-8")

    @property
    def run_dir(self) -> Path:
        return self.config.out_dir

    @property
    def manifest_path(self) -> Path:
        return self.run_dir / "manifest.yaml"

    @property
    def detections_path(self) -> Path:
        return self.run_dir / "detections.jsonl"

    @property
    def perf_path(self) -> Path:
        return self.run_dir / "perf.jsonl"

    @property
    def summary_path(self) -> Path:
        return self.run_dir / "summary.json"

    @property
    def evidence_dir(self) -> Path:
        return self.run_dir / "evidence"

    def write_detection_record(self, record: dict[str, Any]) -> None:
        self._write_jsonl(self._detections_handle, record)
        self.summary.add_detection_record(record)

    def write_perf_record(self, record: dict[str, Any]) -> None:
        self._write_jsonl(self._perf_handle, record)
        self.summary.add_perf_record(record)

    def record_drop(self, stream: str, count: int = 1) -> None:
        self.summary.record_drop(stream, count)

    def finalize(self, *, ended_at: datetime | None = None) -> dict[str, Any]:
        if self._closed and self.summary_path.exists():
            return self.summary_from_disk()

        self.ended_at = ended_at or utc_now()
        self._detections_handle.flush()
        self._perf_handle.flush()
        self._detections_handle.close()
        self._perf_handle.close()

        duration_sec = max(0.0, (self.ended_at - self.started_at).total_seconds())
        summary = self.summary.build_summary(duration_sec)
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._write_manifest()
        self._closed = True
        return summary

    def flush(self) -> None:
        self._detections_handle.flush()
        self._perf_handle.flush()

    def summary_from_disk(self) -> dict[str, Any]:
        return json.loads(self.summary_path.read_text(encoding="utf-8"))

    def close(self) -> None:
        if not self._closed:
            self.finalize()

    def __enter__(self) -> RunBundleWriter:
        return self

    def __exit__(self, _exc_type: object, _exc: object, _tb: object) -> None:
        self.close()

    def _prepare_run_dir(self) -> None:
        if self.run_dir.exists():
            if not self.config.overwrite:
                raise FileExistsError(f"run directory already exists: {self.run_dir}")
            if not self.run_dir.is_dir():
                raise NotADirectoryError(f"run output path is not a directory: {self.run_dir}")
            shutil.rmtree(self.run_dir)
            self.run_dir.mkdir(parents=True)
        else:
            self.run_dir.mkdir(parents=True)

    def _manifest(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.config.run_id,
            "project": self.config.project,
            "robot": self.config.robot,
            "sbc": self.config.sbc,
            "started_at": isoformat_utc(self.started_at),
            "ended_at": isoformat_utc(self.ended_at) if self.ended_at else None,
            "git_sha": self.config.git_sha,
            "ros_distro": self.config.ros_distro,
            "model": {
                "detector": self.config.detector,
                "vision_params_file": self.config.vision_params_file,
                "detector_model_path": self.config.detector_model_path,
                "clip_model_path": self.config.clip_model_path,
                "clip_vocab_path": self.config.clip_vocab_path,
            },
            "classes_path": self.config.classes_path,
            "classes": list(self.config.classes),
            "topics": {
                "detections": self.config.detections_topic,
                "perf": self.config.perf_topic,
            },
            "notes": self.config.notes,
        }

    def _write_manifest(self) -> None:
        self.manifest_path.write_text(_to_yaml(self._manifest()), encoding="utf-8")

    @staticmethod
    def _write_jsonl(handle: Any, record: dict[str, Any]) -> None:
        handle.write(json.dumps(record, separators=(",", ":"), sort_keys=True, allow_nan=False) + "\n")


def make_detection_record(
    *,
    recv_ts_ns: int,
    header_stamp: dict[str, int],
    frame_id: str,
    detections: list[dict[str, Any]],
    topic: str = DEFAULT_DETECTIONS_TOPIC,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "topic": topic,
        "recv_ts_ns": recv_ts_ns,
        "header_stamp": header_stamp,
        "frame_id": frame_id,
        "detections": detections,
    }


def make_perf_record(
    *,
    recv_ts_ns: int,
    header_stamp: dict[str, int],
    frame_id: str,
    producer_fps: float,
    consumer_fps: float,
    last_preprocess_ms: float,
    last_infer_ms: float,
    last_postprocess_ms: float,
    last_publish_ms: float,
    last_producer_total_ms: float,
    last_consumer_total_ms: float,
    produced_count: int,
    consumed_count: int,
    error_counts: dict[str, int],
    topic: str = DEFAULT_PERF_TOPIC,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "topic": topic,
        "recv_ts_ns": recv_ts_ns,
        "header_stamp": header_stamp,
        "frame_id": frame_id,
        "producer_fps": producer_fps,
        "consumer_fps": consumer_fps,
        "last_preprocess_ms": last_preprocess_ms,
        "last_infer_ms": last_infer_ms,
        "last_postprocess_ms": last_postprocess_ms,
        "last_publish_ms": last_publish_ms,
        "last_producer_total_ms": last_producer_total_ms,
        "last_consumer_total_ms": last_consumer_total_ms,
        "produced_count": produced_count,
        "consumed_count": consumed_count,
        "error_counts": error_counts,
    }


def _class_key(detection: dict[str, Any]) -> str:
    class_name = detection.get("class_name")
    if isinstance(class_name, str) and class_name:
        return class_name
    class_id = detection.get("class_id")
    return f"class_id:{class_id}" if class_id is not None else "unknown"


def _append_float(values: list[float], value: object) -> None:
    converted = _as_float(value)
    if converted is not None:
        values.append(converted)


def _as_float(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _as_int(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _mean_or_zero(values: list[float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def _p95_or_zero(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = round((len(ordered) - 1) * 0.95)
    return float(ordered[index])


def _describe_float(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "mean": float(statistics.fmean(values)),
        "max": max(values),
    }


def _to_yaml(value: dict[str, Any]) -> str:
    return "\n".join(_yaml_lines(value, 0)) + "\n"


def _yaml_lines(value: Any, indent: int) -> list[str]:
    pad = " " * indent
    if isinstance(value, dict):
        lines: list[str] = []
        for key, item in value.items():
            if isinstance(item, list) and not item:
                lines.append(f"{pad}{key}: []")
            elif isinstance(item, dict) and not item:
                lines.append(f"{pad}{key}: {{}}")
            elif isinstance(item, (dict, list)):
                lines.append(f"{pad}{key}:")
                lines.extend(_yaml_lines(item, indent + 2))
            else:
                lines.append(f"{pad}{key}: {_yaml_scalar(item)}")
        return lines
    if isinstance(value, list):
        if not value:
            return [f"{pad}[]"]
        lines = []
        for item in value:
            if isinstance(item, (dict, list)):
                lines.append(f"{pad}-")
                lines.extend(_yaml_lines(item, indent + 2))
            else:
                lines.append(f"{pad}- {_yaml_scalar(item)}")
        return lines
    return [f"{pad}{_yaml_scalar(value)}"]


def _yaml_scalar(value: object) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    return json.dumps(str(value))
