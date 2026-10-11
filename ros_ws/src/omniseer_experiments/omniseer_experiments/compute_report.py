"""Offline compute-performance report for stationary perception RunBundles."""
# ruff: noqa: E501

from __future__ import annotations

import argparse
import json
import statistics
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import timezone
from pathlib import Path
from typing import Any

from omniseer_experiments.run_inspection import inspect_run
from omniseer_experiments.run_report import (
    _as_float,
    _ChartSeries,
    _css,
    _format_duration,
    _format_float,
    _key_value_table,
    _line_chart,
    _manifest_model_value,
    _manifest_nested_dict,
    _manifest_start_ns,
    _npu_charts,
    _parse_iso_datetime,
    _percentile,
    _read_jsonl,
    _read_manifest,
    _section,
    _table,
)


@dataclass(frozen=True)
class ComputeReportSummary:
    run_dir: Path
    output_path: Path
    metrics_path: Path
    issues: tuple[str, ...]


def is_compute_performance_run(run_dir: Path) -> bool:
    """Return whether the bundle was recorded by the stationary profiling workflow."""
    manifest = _read_manifest(run_dir / "manifest.yaml")
    experiment = manifest.get("experiment")
    return isinstance(experiment, dict) and experiment.get("config") == "Stationary Perception Profiling"


def write_compute_report(run_dir: Path, *, overwrite: bool = False) -> ComputeReportSummary:
    inspection = inspect_run(run_dir)
    manifest = _read_manifest(run_dir / "manifest.yaml")
    report_dir = run_dir / "report"
    output_path, metrics_path = report_dir / "compute.html", report_dir / "compute_metrics.json"
    if (output_path.exists() or metrics_path.exists()) and not overwrite:
        raise FileExistsError(f"compute report already exists: {output_path}; pass --overwrite to replace it")
    pipeline = _read_jsonl(run_dir / "pipeline_telemetry.jsonl", required=False)
    perf = _read_jsonl(run_dir / "perf.jsonl", required=False)
    system = _read_jsonl(run_dir / "system.jsonl", required=False)
    issues = tuple(
        dict.fromkeys(
            [
                *(f"{item.code}: {item.message}" for item in inspection.issues),
                *pipeline.issues,
                *perf.issues,
                *system.issues,
            ]
        )
    )
    report_dir.mkdir(parents=True, exist_ok=True)
    start, end = _interval(manifest)
    measurement_pipeline = [
        record for record in pipeline.records if record.get("source") == "consumer" and _in_interval(record, start, end)
    ]
    measurement_perf = _interval_records(perf.records, start, end)
    measurement_system = _interval_records(system.records, start, end)
    metrics = _metrics(manifest, inspection, measurement_pipeline, measurement_perf, measurement_system, issues)
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    output_path.write_text(
        _render(manifest, inspection, metrics, measurement_pipeline, measurement_system), encoding="utf-8"
    )
    return ComputeReportSummary(run_dir, output_path, metrics_path, issues)


def compute_report_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate an offline compute performance report for a RunBundle.")
    parser.add_argument("run_dir")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary = write_compute_report(Path(args.run_dir), overwrite=args.overwrite)
    except FileExistsError as exc:
        raise SystemExit(str(exc)) from exc
    print(f"Compute report: {summary.output_path}")
    print(f"Metrics: {summary.metrics_path}")
    print(f"Issues: {len(summary.issues)}")


def _interval(manifest: dict[str, Any]) -> tuple[int | None, int | None]:
    params = _manifest_nested_dict(manifest, "experiment", "parameters")

    def parse(key: str) -> int | None:
        value = params.get(key)
        parsed = _parse_iso_datetime(value) if isinstance(value, str) else None
        if parsed is None:
            return None
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return int(parsed.timestamp() * 1_000_000_000)

    return parse("performance.measurement_interval_start_at"), parse("performance.measurement_interval_end_at")


def _in_interval(record: dict[str, Any], start: int | None, end: int | None) -> bool:
    timestamp = record.get("consumer_end_ts_real_ns") or record.get("event_ts_real_ns")
    if not isinstance(timestamp, int):
        return False
    return (start is None or timestamp >= start) and (end is None or timestamp <= end)


def _interval_records(records: Sequence[dict[str, Any]], start: int | None, end: int | None) -> list[dict[str, Any]]:
    if start is None and end is None:
        return list(records)
    return [
        record
        for record in records
        if isinstance(record.get("recv_ts_ns"), int)
        and (start is None or record["recv_ts_ns"] >= start)
        and (end is None or record["recv_ts_ns"] <= end)
    ]


def _values(records: Sequence[dict[str, Any]], field: str, *, nested: bool = False) -> list[float]:
    result = []
    for record in records:
        value = record.get(field)
        if nested:
            duration = record.get("dur_ns")
            value = duration.get(field) if isinstance(duration, dict) else None
        numeric = _as_float(value)
        if numeric is not None:
            result.append(numeric)
    return result


def _describe(values: Sequence[float], *, scale: float = 1.0) -> dict[str, Any] | None:
    if not values:
        return None
    scaled = [value * scale for value in values]
    return {
        "samples": len(scaled),
        "p50": _percentile(scaled, 50),
        "p95": _percentile(scaled, 95),
        "p99": _percentile(scaled, 99),
        "mean": statistics.fmean(scaled),
        "max": max(scaled),
    }


def _metrics(
    manifest: dict[str, Any],
    inspection: Any,
    pipeline: Sequence[dict[str, Any]],
    perf: Sequence[dict[str, Any]],
    system: Sequence[dict[str, Any]],
    issues: Sequence[str],
) -> dict[str, Any]:
    start, end = _interval(manifest)
    consumers = [record for record in pipeline if record.get("source") == "consumer"]
    params = _manifest_nested_dict(manifest, "experiment", "parameters")
    native = {
        name: _describe(_values(consumers, name, nested=True), scale=1 / 1_000_000.0)
        for name in ("infer", "acquire_read", "postprocess", "total")
    }
    latest = perf[-1] if perf else {}
    npu = _npu_metrics(system)
    thermal = [
        record.get("thermal", {}).get("throttled") for record in system if isinstance(record.get("thermal"), dict)
    ]
    errors = {
        key: value
        for key, value in (latest.get("error_counts", {}) if isinstance(latest, dict) else {}).items()
        if isinstance(value, int) and value > 0
    }
    native_errors = sum(
        1
        for record in consumers
        if record.get("infer_status") not in (None, "ok")
        or record.get("consumer_status") not in (None, "ok")
        or record.get("infer_errno") not in (None, 0)
    )
    return {
        "schema_version": 1,
        "workloads": [
            {
                "id": "yolo-world",
                "model": _model_label(manifest),
                "precision": _manifest_model_value(manifest, "precision") or None,
                "input_resolution": _input_resolution(manifest),
                "npu_core_mask": params.get("runner.core_mask"),
                "native_inference_ms": native["infer"],
                "native_stage_ms": {key: value for key, value in native.items() if key != "infer"},
            }
        ],
        "pipeline": {
            "measurement_consumer_frames": len(consumers),
            "throughput_fps": _describe(_values(perf, "consumer_fps")),
            "preprocess_ms": native["acquire_read"],
            "postprocess_ms": native["postprocess"],
            "source_frame_age_ms": _describe(_values(consumers, "source_age_end_ns"), scale=1 / 1_000_000.0),
            "superseded_frames": _delta(latest, "produced_count", "consumed_count"),
            "errors": {"periodic_counters": errors, "native_non_ok_frames": native_errors},
        },
        "npu": npu,
        "system": {
            "cpu_percent": _describe(_values(system, "cpu_percent")),
            "memory_used_mb": _describe(_values(system, "memory_used_mb")),
            "temperature_c": _describe(_values(system, "soc_temp_c")),
            "major_process_consumers": _processes(system, inspection.duration_sec),
            "thermal_throttling": _throttle(thermal),
        },
        "experiment": {
            "duration_sec": inspection.duration_sec,
            "measurement_interval_start_ns": start,
            "measurement_interval_end_ns": end,
            "completion_status": inspection.state,
            "issues": list(issues),
            "provenance_paths": ["manifest.yaml", "provenance"],
            "observed_performance_change": _change(_values(consumers, "infer", nested=True)),
        },
    }


def _delta(record: dict[str, Any], produced: str, consumed: str) -> int | None:
    left, right = record.get(produced), record.get(consumed)
    return left - right if isinstance(left, int) and isinstance(right, int) else None


def _change(values: Sequence[float]) -> str:
    if len(values) < 2:
        return "Unavailable: fewer than two native inference samples in the measurement interval."
    change_ms = (values[-1] - values[0]) / 1_000_000.0
    direction = "increased" if change_ms > 0 else "decreased" if change_ms < 0 else "did not change"
    return f"Native inference {direction} by {abs(change_ms):.2f} ms from first to last measured frame; this is descriptive, not a throttling attribution."


def _npu_metrics(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    cores = {}
    for core in ("core0", "core1", "core2"):
        values = [
            _as_float(record.get("npu", {}).get("utilization_percent", {}).get(core))
            for record in records
            if isinstance(record.get("npu"), dict)
        ]
        cores[core] = _describe([value for value in values if value is not None])
    freq = [
        _as_float(record.get("npu", {}).get("frequency_hz"))
        for record in records
        if isinstance(record.get("npu"), dict)
    ]
    return {
        "per_core_utilization_percent": cores,
        "frequency_mhz": _describe([value for value in freq if value is not None], scale=1 / 1_000_000.0),
        "scope": "host-wide; not attributable to an individual model",
    }


def _processes(records: Sequence[dict[str, Any]], duration: float | None) -> list[dict[str, Any]]:
    items: dict[tuple[Any, Any], dict[str, Any]] = {}
    for record in records:
        for sample in record.get("process_cpu", []) if isinstance(record.get("process_cpu"), list) else []:
            if not isinstance(sample, dict) or not isinstance(sample.get("cpu_seconds_delta"), (int, float)):
                continue
            key = (sample.get("pid"), sample.get("start_time_ticks"))
            item = items.setdefault(
                key, {"pid": sample.get("pid"), "name": sample.get("name", "unknown"), "cpu_seconds": 0.0}
            )
            item["cpu_seconds"] += sample["cpu_seconds_delta"]
    return [
        {**item, "mean_cores": item["cpu_seconds"] / duration if duration else None}
        for item in sorted(items.values(), key=lambda item: item["cpu_seconds"], reverse=True)[:5]
    ]


def _throttle(values: Sequence[Any]) -> str:
    known = [value for value in values if isinstance(value, bool)]
    if not known:
        return "unavailable"
    return "observed" if any(known) else "not_observed_in_samples"


def _model_label(manifest: dict[str, Any]) -> str:
    return (
        " ".join(
            value
            for value in (_manifest_model_value(manifest, "family"), _manifest_model_value(manifest, "variant"))
            if value
        )
        or "YOLO-World (active workload)"
    )


def _input_resolution(manifest: dict[str, Any]) -> str | None:
    # RunBundle provenance may contain a resolved config, but it is not a stable schema yet.
    params = _manifest_nested_dict(manifest, "experiment", "parameters")
    value = params.get("input_resolution") or params.get("runner.input_resolution")
    return str(value) if isinstance(value, (str, int, float)) else None


def _render(
    manifest: dict[str, Any],
    inspection: Any,
    metrics: dict[str, Any],
    pipeline: Sequence[dict[str, Any]],
    system: Sequence[dict[str, Any]],
) -> str:
    workload = metrics["workloads"][0]
    inference = workload["native_inference_ms"]
    pipeline_metrics, experiment = metrics["pipeline"], metrics["experiment"]
    summary = _section(
        "Compute Performance Summary",
        _key_value_table(
            [
                ("Completion status", experiment["completion_status"]),
                ("Run duration", _format_duration(experiment["duration_sec"])),
                ("Measurement frames", str(pipeline_metrics["measurement_consumer_frames"])),
                ("Inference p95", _stat(inference, "p95", " ms")),
                ("Observed change", experiment["observed_performance_change"]),
                ("NPU attribution", "Host-wide telemetry only; no per-model utilization claim."),
            ]
        ),
        open_by_default=True,
    )
    workload_section = _section(
        "Neural Workload",
        _key_value_table(
            [
                ("Active workload", workload["model"]),
                ("Precision", workload["precision"] or "Unavailable"),
                ("Input resolution", workload["input_resolution"] or "Unavailable"),
                ("NPU core mask", str(workload["npu_core_mask"] or "Unavailable")),
            ]
        )
        + _table(
            ["Native inference", "Samples", "p50 ms", "p95 ms", "p99 ms"],
            [
                [
                    "Per-frame consumer telemetry",
                    str(inference["samples"]) if inference else "0",
                    _stat(inference, "p50"),
                    _stat(inference, "p95"),
                    _stat(inference, "p99"),
                ]
            ],
        ),
        open_by_default=True,
    )
    consumer = [record for record in pipeline if record.get("source") == "consumer"]
    base = _manifest_start_ns(manifest)
    latency_chart = _line_chart(
        "Native Consumer Stage Latency Over Time",
        tuple(_pipeline_series(consumer, name, base) for name in ("infer", "postprocess", "total")),
        "ms",
        zero_floor=True,
    )
    pipe_section = _section(
        "Pipeline",
        _key_value_table(
            [
                ("Consumer throughput", _stat(pipeline_metrics["throughput_fps"], "mean", " fps")),
                ("Source-frame age p95", _stat(pipeline_metrics["source_frame_age_ms"], "p95", " ms")),
                (
                    "Frames superseded",
                    str(
                        pipeline_metrics["superseded_frames"]
                        if pipeline_metrics["superseded_frames"] is not None
                        else "Unavailable"
                    ),
                ),
                ("Native non-OK frames", str(pipeline_metrics["errors"]["native_non_ok_frames"])),
            ]
        )
        + latency_chart,
        open_by_default=True,
    )
    npu = metrics["npu"]
    npu_rows = [
        [core, _stat(value, "samples"), _stat(value, "mean"), _stat(value, "p95")]
        for core, value in npu["per_core_utilization_percent"].items()
    ]
    npu_section = _section(
        "NPU",
        _table(["Core", "Samples", "Mean utilization %", "p95 utilization %"], npu_rows)
        + _key_value_table([("Frequency mean", _stat(npu["frequency_mhz"], "mean", " MHz")), ("Scope", npu["scope"])])
        + _npu_charts(system, experiment_start_ns=base),
        open_by_default=True,
    )
    system_metrics = metrics["system"]
    sys_section = _section(
        "System",
        _key_value_table(
            [
                ("CPU p95", _stat(system_metrics["cpu_percent"], "p95", " %")),
                ("Memory p95", _stat(system_metrics["memory_used_mb"], "p95", " MB")),
                ("Temperature p95", _stat(system_metrics["temperature_c"], "p95", " C")),
                ("Thermal throttling evidence", system_metrics["thermal_throttling"]),
            ]
        )
        + _table(
            ["Process", "CPU seconds", "Mean cores"],
            [
                [str(item["name"]), _format_float(item["cpu_seconds"]), _stat(item, "mean_cores")]
                for item in system_metrics["major_process_consumers"]
            ],
        ),
        open_by_default=True,
    )
    provenance = _section(
        "Experiment & Provenance",
        _key_value_table(
            [
                ("Measurement interval start", str(experiment["measurement_interval_start_ns"] or "Unavailable")),
                ("Measurement interval end", str(experiment["measurement_interval_end_ns"] or "Unavailable")),
                ("Git revision", str(manifest.get("git_sha") or "Unavailable")),
                ("Raw evidence", "pipeline_telemetry.jsonl, perf.jsonl, system.jsonl, manifest.yaml, provenance/"),
            ]
        )
        + (
            "<p>Issues: " + "; ".join(experiment["issues"]) + "</p>"
            if experiment["issues"]
            else "<p>No inspection issues recorded.</p>"
        ),
    )
    sections = (summary, workload_section, pipe_section, npu_section, sys_section, provenance)
    nav = "".join(f'<li><a href="#{section.section_id}">{section.title}</a></li>' for section in sections)
    return (
        f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Omniseer Compute Performance Report: {inspection.run_id}</title><style>{_css()}</style></head><body><main><h1>Omniseer Compute Performance Report: {inspection.run_id}</h1><nav class="toc"><h2>Contents</h2><ol>{nav}</ol></nav>'
        + "\n".join(section.html_text for section in sections)
        + "</main></body></html>"
    )


def _pipeline_series(records: Sequence[dict[str, Any]], name: str, base: int | None) -> _ChartSeries:
    points = []
    for record in records:
        timestamp = record.get("consumer_end_ts_real_ns")
        duration = record.get("dur_ns")
        value = duration.get(name) if isinstance(duration, dict) else None
        if isinstance(timestamp, int) and isinstance(value, (int, float)) and base is not None:
            points.append(((timestamp - base) / 1_000_000_000.0, value / 1_000_000.0))
    return _ChartSeries(name, tuple(points), "ms")


def _stat(value: dict[str, Any] | None, key: str, suffix: str = "") -> str:
    if not value or value.get(key) is None:
        return "Unavailable"
    raw = value[key]
    return str(raw) if key == "samples" else _format_float(float(raw)) + suffix
