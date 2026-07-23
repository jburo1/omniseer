"""Static HTML report generation for local Omniseer run bundles."""

from __future__ import annotations

import argparse
import html
import json
import os
import statistics
from collections import Counter, defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from omniseer_experiments.evidence_annotation import annotate_evidence
from omniseer_experiments.run_inspection import RunInspection, _parse_generated_manifest, inspect_run


@dataclass(frozen=True)
class ReportSummary:
    run_dir: Path
    output_path: Path
    evidence_items: int
    issues: tuple[str, ...]


@dataclass(frozen=True)
class _JsonlRead:
    records: tuple[dict[str, Any], ...]
    issues: tuple[str, ...]


@dataclass(frozen=True)
class _EvidenceItem:
    frame_id: str
    sequence: str
    capture_reason: str
    relative_time: str
    image_href: str
    source_href: str
    labels: tuple[str, ...]
    detection_count: int
    top_score: str
    uses_annotation: bool


@dataclass(frozen=True)
class _NumericSummary:
    samples: int
    first: float
    last: float
    min_value: float
    mean: float
    max_value: float


def write_run_report(run_dir: Path, *, overwrite: bool = False) -> ReportSummary:
    inspection = inspect_run(run_dir)
    manifest = _read_manifest(run_dir / "manifest.yaml")
    report_dir = run_dir / "report"
    output_path = report_dir / "index.html"
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"report already exists: {output_path}; pass --overwrite to replace it")

    annotation_issues = _annotate_evidence_for_report(run_dir, overwrite=overwrite)
    detections = _read_jsonl(run_dir / "detections.jsonl", required=True)
    perf = _read_jsonl(run_dir / "perf.jsonl", required=True)
    system = _read_jsonl(run_dir / "system.jsonl", required=False)
    pipeline = _read_jsonl(run_dir / "pipeline_telemetry.jsonl", required=False)
    evidence = _read_jsonl(run_dir / "evidence" / "evidence.jsonl", required=False)
    evidence_items = _evidence_items(run_dir, report_dir, evidence.records)

    issues = [
        *[f"{issue.code}: {issue.message}" for issue in inspection.issues],
        *detections.issues,
        *perf.issues,
        *system.issues,
        *pipeline.issues,
        *evidence.issues,
        *annotation_issues,
    ]
    html_text = _render_report(
        inspection=inspection,
        manifest=manifest,
        detections=detections.records,
        perf=perf.records,
        system=system.records,
        pipeline=pipeline.records,
        evidence_items=evidence_items,
        issues=tuple(dict.fromkeys(issues)),
    )

    report_dir.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding="utf-8")
    return ReportSummary(
        run_dir=run_dir,
        output_path=output_path,
        evidence_items=len(evidence_items),
        issues=tuple(dict.fromkeys(issues)),
    )


def _annotate_evidence_for_report(run_dir: Path, *, overwrite: bool) -> tuple[str, ...]:
    if not (run_dir / "evidence" / "evidence.jsonl").exists():
        return ()
    try:
        summary = annotate_evidence(run_dir, overwrite=overwrite)
    except RuntimeError as exc:
        return (f"annotation_failed: {exc}",)
    return tuple(issue.format() for issue in summary.issues if issue.code != "annotated_image_exists")


def report_run_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate a static HTML report for a local run bundle.")
    parser.add_argument("run_dir", help="path to a runs/<run_id> bundle")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing report/index.html")
    args = parser.parse_args(argv)

    try:
        summary = write_run_report(Path(args.run_dir), overwrite=args.overwrite)
    except FileExistsError as exc:
        raise SystemExit(str(exc)) from exc

    print(f"Report: {summary.output_path}")
    print(f"Evidence items: {summary.evidence_items}")
    print(f"Issues: {len(summary.issues)}")


def _read_jsonl(path: Path, *, required: bool) -> _JsonlRead:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        if required:
            return _JsonlRead(records=(), issues=(f"missing file: {path}",))
        return _JsonlRead(records=(), issues=())
    except OSError as exc:
        return _JsonlRead(records=(), issues=(f"unreadable file: {path}: {exc}",))

    records: list[dict[str, Any]] = []
    issues: list[str] = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            issues.append(f"malformed JSONL: {path}:{line_number}: {exc.msg}")
            continue
        if not isinstance(record, dict):
            issues.append(f"invalid JSONL record: {path}:{line_number}: expected object")
            continue
        records.append(record)
    return _JsonlRead(records=tuple(records), issues=tuple(issues))


def _render_report(
    *,
    inspection: RunInspection,
    manifest: dict[str, Any],
    detections: Sequence[dict[str, Any]],
    perf: Sequence[dict[str, Any]],
    system: Sequence[dict[str, Any]],
    pipeline: Sequence[dict[str, Any]],
    evidence_items: Sequence[_EvidenceItem],
    issues: Sequence[str],
) -> str:
    title = f"Omniseer Run Report: {inspection.run_id}"
    sections = [
        _evidence_summary_section(
            inspection=inspection,
            manifest=manifest,
            detections=detections,
            perf=perf,
            evidence_items=evidence_items,
            issues=issues,
        ),
        _summary_section(inspection),
        _configuration_section(manifest),
        _health_section(inspection, evidence_items=evidence_items, pipeline=pipeline, issues=issues),
        _detections_section(detections, configured_classes=inspection.configured_classes),
        _perf_section("Performance", perf),
        _pipeline_section(pipeline),
        _system_section(system, manifest=manifest),
        _errors_section(inspection),
        _evidence_section(evidence_items),
        _issues_section(issues),
    ]
    body = "\n".join(section for section in sections if section)
    return (
        "<!doctype html>\n"
        '<html lang="en">\n'
        "<head>\n"
        '  <meta charset="utf-8">\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1">\n'
        f"  <title>{_esc(title)}</title>\n"
        f"  <style>{_css()}</style>\n"
        "</head>\n"
        "<body>\n"
        "  <main>\n"
        f"    <h1>{_esc(title)}</h1>\n"
        f"{body}\n"
        "  </main>\n"
        "</body>\n"
        "</html>\n"
    )


def _evidence_summary_section(
    *,
    inspection: RunInspection,
    manifest: dict[str, Any],
    detections: Sequence[dict[str, Any]],
    perf: Sequence[dict[str, Any]],
    evidence_items: Sequence[_EvidenceItem],
    issues: Sequence[str],
) -> str:
    total_detections = sum(1 for _ in _iter_detections(detections))
    observed_classes = _join_or_dash(tuple(sorted(inspection.detections_by_class)))
    configured_classes = _join_or_dash(inspection.configured_classes)
    consumer_total_p95 = _p95_field(perf, "last_consumer_total_ms")
    infer_p95 = _p95_field(perf, "last_infer_ms")
    detector = _manifest_model_value(manifest, "detector")
    hardware = _manifest_string(manifest, "sbc")

    claims = [
        f"Run state: {inspection.state}; duration {_format_duration(inspection.duration_sec)}; issues {len(issues)}.",
        f"Target: {_display(hardware)} running {_display(detector)} with configured classes: {configured_classes}.",
        f"Observed {total_detections} detections across {len(detections)} detection messages; "
        f"classes observed: {observed_classes}.",
        f"Performance samples: {len(perf)}; consumer total p95 {_format_optional_ms(consumer_total_p95)}; "
        f"inference p95 {_format_optional_ms(infer_p95)}.",
        f"Visual evidence: {len(evidence_items)} sampled frames, "
        f"{sum(1 for item in evidence_items if item.uses_annotation)} annotated.",
    ]
    items = "".join(f"<li>{_esc(claim)}</li>" for claim in claims)
    return _section("Evidence Summary", f'<ul class="summary-list">{items}</ul>')


def _summary_section(inspection: RunInspection) -> str:
    rows = [
        ("Run ID", inspection.run_id),
        ("State", inspection.state),
        ("Path", str(inspection.path)),
        ("Started", _display(inspection.started_at)),
        ("Ended", _display(inspection.ended_at)),
        ("Duration", _format_duration(inspection.duration_sec)),
        ("Configured classes", _join_or_dash(inspection.configured_classes)),
    ]
    return _section("Run Summary", _key_value_table(rows))


def _configuration_section(manifest: dict[str, Any]) -> str:
    experiment_parameters = _manifest_nested_dict(manifest, "experiment", "parameters")
    launch_args = _manifest_nested_list(manifest, "launch", "args")
    model_rows = [
        ("Robot", _manifest_string(manifest, "robot")),
        ("SBC", _manifest_string(manifest, "sbc")),
        ("ROS distro", _manifest_string(manifest, "ros_distro")),
        ("Git SHA", _manifest_string(manifest, "git_sha")),
        ("Launch command", _manifest_nested_string(manifest, "launch", "command")),
        ("Launch profile", _manifest_nested_string(manifest, "launch", "profile")),
        ("Launch mode", _manifest_nested_string(manifest, "launch", "mode")),
        ("Launch args", _join_or_dash(launch_args)),
        ("Runtime image ref", _manifest_nested_string(manifest, "container", "image_ref")),
        ("Runtime image digest", _manifest_nested_string(manifest, "container", "image_digest")),
        ("Experiment config", _manifest_nested_string(manifest, "experiment", "config")),
        ("Experiment parameters", _format_mapping(experiment_parameters)),
        ("Detector", _manifest_model_value(manifest, "detector")),
        ("Detector model", _path_basename(_manifest_model_value(manifest, "detector_model_path"))),
        ("CLIP model", _path_basename(_manifest_model_value(manifest, "clip_model_path"))),
        ("CLIP vocab", _path_basename(_manifest_model_value(manifest, "clip_vocab_path"))),
        ("Classes file", _path_basename(_manifest_string(manifest, "classes_path"))),
        ("Detections topic", _manifest_topic_value(manifest, "detections")),
        ("Performance topic", _manifest_topic_value(manifest, "perf")),
        ("Notes", _manifest_string(manifest, "notes")),
    ]
    rows = [(key, _display(value)) for key, value in model_rows]
    return _section("Configuration", _key_value_table(rows))


def _health_section(
    inspection: RunInspection,
    *,
    evidence_items: Sequence[_EvidenceItem],
    pipeline: Sequence[dict[str, Any]],
    issues: Sequence[str],
) -> str:
    annotated_count = sum(1 for item in evidence_items if item.uses_annotation)
    rows = [
        ("Detection messages", str(inspection.message_counts.get("detections", 0))),
        ("Perf samples", str(inspection.message_counts.get("perf", 0))),
        ("System samples", str(inspection.message_counts.get("system", 0))),
        ("Pipeline telemetry samples", str(len(pipeline))),
        ("Evidence items", str(len(evidence_items))),
        ("Annotated evidence items", str(annotated_count)),
        ("Issues", str(len(issues))),
    ]
    return _section("Health", _key_value_table(rows))


def _detections_section(records: Sequence[dict[str, Any]], *, configured_classes: Sequence[str]) -> str:
    scores_by_class: dict[str, list[float]] = defaultdict(list)
    counts: Counter[str] = Counter()
    messages_with_detections = 0
    messages_without_detections = 0
    total_detections = 0
    observed_classes: set[str] = set()
    configured_class_set = set(configured_classes)
    for detection in _iter_detections(records):
        class_name = _class_name(detection)
        counts[class_name] += 1
        observed_classes.add(class_name)
        total_detections += 1
        score = _as_float(detection.get("score"))
        if score is not None:
            scores_by_class[class_name].append(score)

    for record in records:
        detections = record.get("detections")
        if isinstance(detections, list) and detections:
            messages_with_detections += 1
        else:
            messages_without_detections += 1

    if not counts:
        summary = _key_value_table(
            [
                ("Detection messages", str(len(records))),
                ("Messages with detections", str(messages_with_detections)),
                ("Messages without detections", str(messages_without_detections)),
                ("Total detections", "0"),
            ]
        )
        return _section("Detections", summary + "<p>No detections recorded.</p>")

    rows = []
    for class_name, count in sorted(counts.items()):
        scores = scores_by_class[class_name]
        rows.append(
            [
                class_name,
                str(count),
                _format_float(min(scores)) if scores else "-",
                _format_float(statistics.fmean(scores)) if scores else "-",
                _format_float(max(scores)) if scores else "-",
            ]
        )
    summary = _key_value_table(
        [
            ("Detection messages", str(len(records))),
            ("Messages with detections", str(messages_with_detections)),
            ("Messages without detections", str(messages_without_detections)),
            ("Total detections", str(total_detections)),
            ("Observed classes", _join_or_dash(tuple(sorted(observed_classes)))),
            ("Configured classes not observed", _join_or_dash(tuple(sorted(configured_class_set - observed_classes)))),
        ]
    )
    return _section("Detections", summary + _table(["Class", "Count", "Min Score", "Mean Score", "Max Score"], rows))


def _pipeline_section(records: Sequence[dict[str, Any]]) -> str:
    if not records:
        return _section("Pipeline Telemetry", "<p>No native pipeline telemetry recorded.</p>")

    body_parts = [
        _pipeline_status_tables(records),
        _pipeline_stage_table(records, source="producer"),
        _pipeline_stage_table(records, source="consumer"),
        _pipeline_age_table(records),
    ]
    return _section("Pipeline Telemetry", "".join(part for part in body_parts if part))


def _pipeline_status_tables(records: Sequence[dict[str, Any]]) -> str:
    fields = (
        ("producer_status", "Producer status"),
        ("capture_status", "Capture status"),
        ("preprocess_status", "Preprocess status"),
        ("consumer_status", "Consumer status"),
        ("infer_status", "Infer status"),
        ("postprocess_status", "Postprocess status"),
    )
    rows = []
    for field_name, label in fields:
        counts: Counter[str] = Counter()
        for record in records:
            value = record.get(field_name)
            if isinstance(value, str) and value:
                counts[value] += 1
        if counts:
            rows.append([label, ", ".join(f"{key}={value}" for key, value in sorted(counts.items()))])
    if not rows:
        return ""
    return "<h3>Status Distribution</h3>" + _table(["Status Field", "Counts"], rows)


def _pipeline_stage_table(records: Sequence[dict[str, Any]], *, source: str) -> str:
    subset = [record for record in records if record.get("source") == source]
    if not subset:
        return ""
    fields = (
        ("dequeue", "dequeue"),
        ("acquire_write", "acquire_write"),
        ("preprocess", "preprocess"),
        ("publish_ready", "publish_ready"),
        ("requeue", "requeue"),
        ("acquire_read", "acquire_read"),
        ("infer", "infer"),
        ("postprocess", "postprocess"),
        ("publish", "publish"),
        ("release", "release"),
        ("total", "total"),
    )
    rows = []
    for field_name, label in fields:
        samples = []
        for record in subset:
            dur_ns = record.get("dur_ns")
            if not isinstance(dur_ns, dict):
                continue
            value = _as_float(dur_ns.get(field_name))
            if value is not None:
                samples.append(value / 1_000_000.0)
        if samples:
            rows.append(
                [
                    label,
                    str(len(samples)),
                    _format_float(_percentile(samples, 50)),
                    _format_float(_percentile(samples, 95)),
                    _format_float(max(samples)),
                ]
            )
    if not rows:
        return ""
    return f"<h3>{source.title()} Stage Timings</h3>" + _table(["Stage", "Samples", "p50 ms", "p95 ms", "Max ms"], rows)


def _pipeline_age_table(records: Sequence[dict[str, Any]]) -> str:
    fields = (
        ("source_age_dequeue_ns", "source age at producer dequeue"),
        ("source_age_publish_ready_ns", "source age at producer publish-ready"),
        ("source_age_start_ns", "source age at consumer start"),
        ("source_age_end_ns", "source age at consumer end"),
    )
    rows = []
    for field_name, label in fields:
        samples = [
            value / 1_000_000.0
            for value in (_as_float(record.get(field_name)) for record in records)
            if value is not None
        ]
        if samples:
            rows.append(
                [
                    label,
                    str(len(samples)),
                    _format_float(_percentile(samples, 50)),
                    _format_float(_percentile(samples, 95)),
                    _format_float(max(samples)),
                ]
            )
    if not rows:
        return ""
    return "<h3>Source Age</h3>" + _table(["Metric", "Samples", "p50 ms", "p95 ms", "Max ms"], rows)


def _perf_section(title: str, records: Sequence[dict[str, Any]]) -> str:
    fields = _numeric_fields(records, suffixes=("_ms", "_fps"), names=("producer_fps", "consumer_fps"))
    if not fields:
        return _section(title, "<p>No numeric samples recorded.</p>") if title == "Performance" else ""

    rows = []
    for field_name in fields:
        values = [_as_float(record.get(field_name)) for record in records]
        samples = [value for value in values if value is not None]
        if not samples:
            continue
        rows.append(
            [
                field_name,
                str(len(samples)),
                _format_float(_percentile(samples, 50)),
                _format_float(_percentile(samples, 95)),
                _format_float(max(samples)),
            ]
        )
    return _section(title, _table(["Metric", "Samples", "p50", "p95", "Max"], rows))


def _system_section(records: Sequence[dict[str, Any]], *, manifest: dict[str, Any]) -> str:
    fields = ("cpu_percent", "memory_used_mb", "memory_available_mb", "soc_temp_c")
    rows = []
    for field_name in fields:
        samples = [_as_float(record.get(field_name)) for record in records]
        values = [value for value in samples if value is not None]
        if not values:
            continue
        rows.append(
            [
                field_name,
                str(len(values)),
                _format_float(min(values)),
                _format_float(statistics.fmean(values)),
                _format_float(max(values)),
            ]
        )
    if not rows:
        return _section("System", "<p>No system telemetry recorded.</p>")
    body = (
        _system_sample_table(records, manifest=manifest)
        + _table(["Metric", "Samples", "Min", "Mean", "Max"], rows)
        + _platform_summary_tables(records)
    )
    return _section("System", body)


def _system_sample_table(records: Sequence[dict[str, Any]], *, manifest: dict[str, Any]) -> str:
    first_ts = _record_recv_ts(records[0]) if records else None
    latest_ts = _record_recv_ts(records[-1]) if records else None
    started_at = _parse_iso_datetime(_manifest_string(manifest, "started_at"))
    return _key_value_table(
        [
            ("Samples", str(len(records))),
            ("First sample", _format_ns_timestamp(first_ts)),
            ("Latest sample", _format_ns_timestamp(latest_ts)),
            ("Latest sample offset", _format_sample_offset(latest_ts, started_at)),
        ]
    )


def _platform_summary_tables(records: Sequence[dict[str, Any]]) -> str:
    return _thermal_summary_table(records) + _network_summary_table(records) + _battery_summary_table(records)


def _thermal_summary_table(records: Sequence[dict[str, Any]]) -> str:
    thermal_records = [record.get("thermal") for record in records if isinstance(record.get("thermal"), dict)]
    if not thermal_records:
        return ""

    rows = []
    soc_values = [_as_float(thermal.get("soc_temp_c")) for thermal in thermal_records]
    soc_summary = _numeric_summary([value for value in soc_values if value is not None])
    if soc_summary is not None:
        rows.append(["SoC temperature C", *_numeric_summary_cells(soc_summary)])

    zones: dict[str, list[float]] = defaultdict(list)
    for thermal in thermal_records:
        zone_items = thermal.get("zones")
        if not isinstance(zone_items, list):
            continue
        for zone in zone_items:
            if not isinstance(zone, dict):
                continue
            value = _as_float(zone.get("temp_c"))
            if value is None:
                continue
            zones[_thermal_zone_label(zone)].append(value)
    for label, values in sorted(zones.items()):
        summary = _numeric_summary(values)
        if summary is not None:
            rows.append([label, *_numeric_summary_cells(summary)])

    if not rows:
        return ""

    summary = _key_value_table(
        [
            ("Available samples", _count_bool(thermal.get("available") for thermal in thermal_records)),
            ("Throttled states", _count_display(thermal.get("throttled") for thermal in thermal_records)),
        ]
    )
    return "<h3>Thermal Summary</h3>" + summary + _table(_numeric_summary_headers("Metric"), rows)


def _thermal_zone_label(zone: dict[str, Any]) -> str:
    name = zone.get("name")
    zone_type = zone.get("type")
    name_text = name if isinstance(name, str) and name else "zone"
    type_text = zone_type if isinstance(zone_type, str) and zone_type else ""
    return f"{type_text} ({name_text})" if type_text else name_text


def _network_summary_table(records: Sequence[dict[str, Any]]) -> str:
    network_records = [record.get("network") for record in records if isinstance(record.get("network"), dict)]
    if not network_records:
        return ""

    rows = []
    for field_name, label in (
        ("wifi_signal_dbm", "WiFi signal dBm"),
        ("link_quality_percent", "Link quality percent"),
    ):
        values = [_as_float(network.get(field_name)) for network in network_records]
        summary = _numeric_summary([value for value in values if value is not None])
        if summary is not None:
            rows.append([label, *_numeric_summary_cells(summary)])

    summary = _key_value_table(
        [
            ("Available samples", _count_bool(network.get("available") for network in network_records)),
            ("Connected samples", _count_bool(network.get("connected") for network in network_records)),
            (
                "Interfaces",
                _join_or_dash(tuple(sorted(_string_values(network.get("interface") for network in network_records)))),
            ),
        ]
    )
    body = summary
    if rows:
        body += _table(_numeric_summary_headers("Metric"), rows)
    return "<h3>Network Summary</h3>" + body


def _battery_summary_table(records: Sequence[dict[str, Any]]) -> str:
    battery_sources = (
        ("LiPo", "lipo_battery"),
        ("Onboard", "onboard_battery"),
    )
    state_rows = []
    numeric_rows = []
    for label, field_name in battery_sources:
        snapshots = [record.get(field_name) for record in records if isinstance(record.get(field_name), dict)]
        if not snapshots:
            continue
        state_rows.append(
            [
                label,
                _count_bool(snapshot.get("available") for snapshot in snapshots),
                _count_bool(snapshot.get("present") for snapshot in snapshots),
                _count_display(snapshot.get("charging") for snapshot in snapshots),
                _join_or_dash(tuple(sorted(_string_values(snapshot.get("source") for snapshot in snapshots)))),
            ]
        )
        metrics = [("voltage", "Voltage")]
        if field_name != "lipo_battery":
            metrics.append(("percentage", "Percentage"))
        for metric_name, metric_label in metrics:
            values = [_as_float(snapshot.get(metric_name)) for snapshot in snapshots]
            summary = _numeric_summary([value for value in values if value is not None])
            if summary is not None:
                numeric_rows.append([label, metric_label, *_numeric_summary_cells(summary)])

    if not state_rows:
        return ""

    body = _table(["Battery", "Available", "Present", "Charging", "Sources"], state_rows)
    if numeric_rows:
        body += _table(_numeric_summary_headers("Battery", "Metric"), numeric_rows)
    return "<h3>Battery Summary</h3>" + body


def _errors_section(inspection: RunInspection) -> str:
    error_rows = [[name, str(count)] for name, count in sorted(inspection.errors.items())]
    drop_rows = [[name, str(count)] for name, count in sorted(inspection.dropped_records.items())]
    if not error_rows:
        error_rows = [["-", "0"]]
    if not drop_rows:
        drop_rows = [["-", "0"]]
    body = (
        "<h3>Pipeline Errors</h3>"
        + _table(["Error Counter", "Count"], error_rows)
        + "<h3>Dropped Records</h3>"
        + _table(["Stream", "Count"], drop_rows)
    )
    return _section("Errors And Drops", body)


def _evidence_section(items: Sequence[_EvidenceItem]) -> str:
    if not items:
        return _section("Evidence", "<p>No evidence images recorded.</p>")

    cards = []
    for item in items:
        label_text = _join_or_dash(item.labels)
        source_note = "annotated" if item.uses_annotation else "clean frame"
        cards.append(
            '<article class="evidence-card">'
            f'<a href="{_attr(item.image_href)}"><img src="{_attr(item.image_href)}" alt="frame {item.frame_id}"></a>'
            "<div>"
            f"<strong>Frame {_esc(item.frame_id)}</strong>"
            f"<span>Sequence {_esc(item.sequence)} &middot; {_esc(item.capture_reason)} "
            f"&middot; {_esc(source_note)}</span>"
            f"<span>t+{_esc(item.relative_time)} &middot; detections {item.detection_count} "
            f"&middot; top score {_esc(item.top_score)}</span>"
            f"<span>{_esc(label_text)}</span>"
            f'<a href="{_attr(item.source_href)}">clean frame</a>'
            "</div>"
            "</article>"
        )
    intro = (
        "<p>Annotated images are derived review artifacts. Clean frames are the canonical captured evidence; "
        "boxes are projected back into source image coordinates.</p>"
    )
    return _section("Evidence", f'{intro}<div class="evidence-grid">{"".join(cards)}</div>')


def _issues_section(issues: Sequence[str]) -> str:
    if not issues:
        return _section("Issues", "<p>No issues found by local report inputs.</p>")
    items = "".join(f"<li>{_esc(issue)}</li>" for issue in issues)
    return _section("Issues", f"<ul>{items}</ul>")


def _evidence_items(run_dir: Path, report_dir: Path, records: Sequence[dict[str, Any]]) -> tuple[_EvidenceItem, ...]:
    items: list[_EvidenceItem] = []
    valid_records = [record for record in records if record.get("artifact_type") == "sampled_frame"]
    base_capture_ts = min(
        (
            value
            for value in (_as_float(record.get("capture_ts_real_ns")) for record in valid_records)
            if value is not None
        ),
        default=None,
    )
    for record in records:
        if record.get("artifact_type") != "sampled_frame":
            continue
        image_path_value = record.get("image_path")
        if not isinstance(image_path_value, str) or not image_path_value:
            continue
        relative_image_path = Path(image_path_value)
        if relative_image_path.is_absolute() or ".." in relative_image_path.parts:
            continue
        source_path = run_dir / image_path_value
        annotated_path = run_dir / "evidence" / "annotated" / source_path.name
        display_path = annotated_path if annotated_path.is_file() else source_path
        if not display_path.is_file():
            continue
        detections = record.get("detections")
        detection_count = len(detections) if isinstance(detections, list) else 0
        capture_ts = _as_float(record.get("capture_ts_real_ns"))
        items.append(
            _EvidenceItem(
                frame_id=str(record.get("frame_id", "-")),
                sequence=str(record.get("sequence", "-")),
                capture_reason=str(record.get("capture_reason", "-")),
                relative_time=_relative_time(capture_ts, base_capture_ts),
                image_href=_relative_href(report_dir, display_path),
                source_href=_relative_href(report_dir, source_path),
                labels=_detection_labels(record.get("detections")),
                detection_count=detection_count,
                top_score=_top_score(record.get("detections")),
                uses_annotation=display_path == annotated_path,
            )
        )
    return tuple(items)


def _read_manifest(path: Path) -> dict[str, Any]:
    try:
        return _parse_generated_manifest(path.read_text(encoding="utf-8").splitlines())
    except (FileNotFoundError, OSError):
        return {}


def _manifest_string(manifest: dict[str, Any], key: str) -> str:
    value = manifest.get(key)
    return value if isinstance(value, str) else ""


def _manifest_model_value(manifest: dict[str, Any], key: str) -> str:
    return _manifest_nested_string(manifest, "model", key)


def _manifest_topic_value(manifest: dict[str, Any], key: str) -> str:
    return _manifest_nested_string(manifest, "topics", key)


def _manifest_nested_string(manifest: dict[str, Any], section: str, key: str) -> str:
    nested = manifest.get(section)
    if not isinstance(nested, dict):
        return ""
    value = nested.get(key)
    return value if isinstance(value, str) else ""


def _manifest_nested_dict(manifest: dict[str, Any], section: str, key: str) -> dict[str, Any]:
    nested = manifest.get(section)
    if not isinstance(nested, dict):
        return {}
    value = nested.get(key)
    return value if isinstance(value, dict) else {}


def _manifest_nested_list(manifest: dict[str, Any], section: str, key: str) -> tuple[str, ...]:
    nested = manifest.get(section)
    if not isinstance(nested, dict):
        return ()
    value = nested.get(key)
    if not isinstance(value, list):
        return ()
    return tuple(str(item) for item in value if item)


def _format_mapping(value: dict[str, Any]) -> str:
    if not value:
        return ""
    return ", ".join(f"{key}={item}" for key, item in sorted(value.items()))


def _path_basename(value: str) -> str:
    return Path(value).name if value else ""


def _detection_labels(value: object) -> tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    labels = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            continue
        label = _class_name(item, index=index)
        score = _as_float(item.get("score"))
        labels.append(f"{label} {_format_float(score)}" if score is not None else label)
    return tuple(labels)


def _top_score(value: object) -> str:
    if not isinstance(value, list):
        return "-"
    scores = [
        score
        for score in (_as_float(item.get("score")) for item in value if isinstance(item, dict))
        if score is not None
    ]
    return _format_float(max(scores)) if scores else "-"


def _relative_time(capture_ts_ns: float | None, base_capture_ts_ns: float | None) -> str:
    if capture_ts_ns is None or base_capture_ts_ns is None:
        return "-"
    return _format_duration(max(0.0, (capture_ts_ns - base_capture_ts_ns) / 1_000_000_000.0))


def _iter_detections(records: Sequence[dict[str, Any]]) -> Iterable[dict[str, Any]]:
    for record in records:
        detections = record.get("detections")
        if not isinstance(detections, list):
            continue
        for detection in detections:
            if isinstance(detection, dict):
                yield detection


def _class_name(detection: dict[str, Any], *, index: int = 0) -> str:
    class_name = detection.get("class_name")
    if isinstance(class_name, str) and class_name:
        return class_name
    class_id = detection.get("class_id")
    if class_id is not None:
        return f"class_{class_id}"
    return f"detection_{index}"


def _numeric_fields(
    records: Sequence[dict[str, Any]],
    *,
    suffixes: Sequence[str],
    names: Sequence[str],
) -> tuple[str, ...]:
    fields: set[str] = set()
    for record in records:
        for key, value in record.items():
            if _as_float(value) is None:
                continue
            if key in names or any(key.endswith(suffix) for suffix in suffixes):
                fields.add(key)
    return tuple(sorted(fields))


def _percentile(values: Sequence[float], percentile: int) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = round((len(ordered) - 1) * (percentile / 100.0))
    return ordered[index]


def _p95_field(records: Sequence[dict[str, Any]], field_name: str) -> float | None:
    samples = [value for value in (_as_float(record.get(field_name)) for record in records) if value is not None]
    if not samples:
        return None
    return _percentile(samples, 95)


def _numeric_summary(values: Sequence[float]) -> _NumericSummary | None:
    if not values:
        return None
    return _NumericSummary(
        samples=len(values),
        first=values[0],
        last=values[-1],
        min_value=min(values),
        mean=statistics.fmean(values),
        max_value=max(values),
    )


def _numeric_summary_headers(*leading: str) -> list[str]:
    return [*leading, "Samples", "First", "Last", "Min", "Mean", "Max"]


def _numeric_summary_cells(summary: _NumericSummary) -> list[str]:
    return [
        str(summary.samples),
        _format_float(summary.first),
        _format_float(summary.last),
        _format_float(summary.min_value),
        _format_float(summary.mean),
        _format_float(summary.max_value),
    ]


def _count_bool(values: Iterable[object]) -> str:
    counts: Counter[str] = Counter()
    for value in values:
        if isinstance(value, bool):
            counts["true" if value else "false"] += 1
    return _format_counts(counts)


def _count_display(values: Iterable[object]) -> str:
    counts: Counter[str] = Counter(_count_value_label(value) for value in values)
    return _format_counts(counts)


def _count_value_label(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None or value == "":
        return "unavailable"
    return str(value)


def _format_counts(counts: Counter[str]) -> str:
    if not counts:
        return "-"
    return ", ".join(f"{key}={count}" for key, count in sorted(counts.items()))


def _string_values(values: Iterable[object]) -> set[str]:
    return {value for value in values if isinstance(value, str) and value}


def _as_float(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _record_recv_ts(record: dict[str, Any]) -> int | None:
    value = record.get("recv_ts_ns")
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _parse_iso_datetime(value: str) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value)
    except ValueError:
        return None


def _format_ns_timestamp(value: int | None) -> str:
    if value is None:
        return "-"
    return datetime.fromtimestamp(value / 1_000_000_000.0, tz=timezone.utc).isoformat()


def _format_sample_offset(value: int | None, started_at: datetime | None) -> str:
    if value is None or started_at is None:
        return "-"
    if started_at.tzinfo is None:
        started_at = started_at.replace(tzinfo=timezone.utc)
    sample_at = datetime.fromtimestamp(value / 1_000_000_000.0, tz=timezone.utc)
    return _format_duration(max(0.0, (sample_at - started_at.astimezone(timezone.utc)).total_seconds()))


def _format_float(value: float) -> str:
    return f"{value:.2f}"


def _format_optional_ms(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{_format_float(value)} ms"


def _format_optional_float(value: float | None) -> str:
    return "-" if value is None else _format_float(value)


def _format_duration(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.1f}s"


def _display(value: object) -> str:
    if value is None or value == "":
        return "-"
    return str(value)


def _join_or_dash(values: Sequence[str]) -> str:
    return ", ".join(values) if values else "-"


def _relative_href(from_dir: Path, target: Path) -> str:
    return Path(os.path.relpath(target, start=from_dir)).as_posix()


def _section(title: str, body: str) -> str:
    return f"    <section>\n      <h2>{_esc(title)}</h2>\n      {body}\n    </section>"


def _key_value_table(rows: Sequence[tuple[str, str]]) -> str:
    table_rows = "".join(f"<tr><th>{_esc(key)}</th><td>{_esc(value)}</td></tr>" for key, value in rows)
    return f'<table class="kv"><tbody>{table_rows}</tbody></table>'


def _table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    header_html = "".join(f"<th>{_esc(header)}</th>" for header in headers)
    row_html = "".join("<tr>" + "".join(f"<td>{_esc(value)}</td>" for value in row) + "</tr>" for row in rows)
    return f"<table><thead><tr>{header_html}</tr></thead><tbody>{row_html}</tbody></table>"


def _esc(value: object) -> str:
    return html.escape(str(value), quote=False)


def _attr(value: object) -> str:
    return html.escape(str(value), quote=True)


def _css() -> str:
    return """
:root { color-scheme: light; font-family: Arial, sans-serif; background: #f6f7f9; color: #1f2933; }
body { margin: 0; }
main { max-width: 1120px; margin: 0 auto; padding: 28px 20px 48px; }
h1 { margin: 0 0 22px; font-size: 28px; font-weight: 700; }
h2 { margin: 0 0 12px; font-size: 19px; font-weight: 700; }
h3 { margin: 16px 0 8px; font-size: 15px; font-weight: 700; color: #3d4a5c; }
h2 + h3 { margin-top: 0; }
section { margin: 0 0 18px; padding: 16px; background: #ffffff; border: 1px solid #d8dee6; border-radius: 6px; }
table { width: 100%; border-collapse: collapse; font-size: 14px; }
th, td { padding: 8px 10px; border-bottom: 1px solid #e6eaf0; text-align: left; vertical-align: top; }
th { color: #3d4a5c; font-weight: 700; background: #f1f4f8; }
tbody tr:last-child th, tbody tr:last-child td { border-bottom: 0; }
.kv th { width: 220px; }
p, ul { margin: 0; font-size: 14px; }
ul { padding-left: 20px; }
.evidence-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 14px; }
p + .evidence-grid { margin-top: 12px; }
.evidence-card { border: 1px solid #d8dee6; border-radius: 6px; overflow: hidden; background: #fbfcfe; }
.evidence-card img { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: contain; background: #111827; }
.evidence-card div { display: grid; gap: 5px; padding: 10px; font-size: 13px; }
.evidence-card span { color: #526173; }
.evidence-card a { color: #185abc; }
""".strip()
