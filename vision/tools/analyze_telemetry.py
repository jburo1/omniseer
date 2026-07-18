#!/usr/bin/env python3

from __future__ import annotations

import argparse
import collections
import json
import math
import statistics
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path


def load_jsonl(path: Path) -> list[dict]:
    records: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for lineno, raw in enumerate(handle, start=1):
            line = raw.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{lineno}: invalid JSON: {exc}") from exc
            if not isinstance(obj, dict):
                raise ValueError(f"{path}:{lineno}: expected JSON object")
            records.append(obj)
    return records


def quantile(values: list[int], p: float) -> int:
    if not values:
        raise ValueError("quantile() requires non-empty values")
    idx = round((len(values) - 1) * p)
    idx = max(0, min(len(values) - 1, idx))
    return values[idx]


def describe(values: Iterable[int]) -> dict[str, float | int] | None:
    ordered = sorted(int(v) for v in values)
    if not ordered:
        return None
    return {
        "n": len(ordered),
        "min": ordered[0],
        "p50": quantile(ordered, 0.50),
        "p95": quantile(ordered, 0.95),
        "p99": quantile(ordered, 0.99),
        "max": ordered[-1],
        "mean": statistics.mean(ordered),
    }


def as_int(value: object) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    return None


def get_duration(record: dict, key: str) -> int | None:
    return as_int(record.get("dur_ns", {}).get(key))


def get_field(record: dict, key: str) -> int | None:
    return as_int(record.get(key))


def safe_delta(a: int | None, b: int | None) -> int | None:
    if a is None or b is None:
        return None
    value = b - a
    if value < 0:
        return None
    return value


def format_ns_ms(value: float | int | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value) / 1.0e6:.3f}"


def format_count(value: int | None) -> str:
    return "n/a" if value is None else str(value)


def format_value(value: float | int | None, unit: str) -> str:
    if unit == "ns":
        return format_ns_ms(value)
    if unit == "count":
        if value is None:
            return "n/a"
        return str(round(float(value)))
    raise ValueError(f"unsupported unit: {unit}")


def percent_change(base: float | int | None, other: float | int | None) -> str:
    if base is None or other is None:
        return "n/a"
    base_f = float(base)
    other_f = float(other)
    if math.isclose(base_f, 0.0):
        return "n/a"
    return f"{((other_f - base_f) / base_f) * 100.0:+.1f}%"


def emit_section(title: str) -> None:
    print()
    print(title)
    print("-" * len(title))


def print_metric_table(rows: list[tuple[str, dict[str, float | int] | None]], unit: str = "ns") -> None:
    if not rows:
        return
    name_width = max(len(name) for name, _ in rows)
    unit_label = "ms" if unit == "ns" else unit
    print(
        f"{'metric'.ljust(name_width)}  {'n':>6} "
        f"{('p50 ' + unit_label):>10} {('p95 ' + unit_label):>10} "
        f"{('p99 ' + unit_label):>10} {('mean ' + unit_label):>10} {('max ' + unit_label):>10}"
    )
    for name, stats in rows:
        if stats is None:
            print(f"{name.ljust(name_width)}  {'-':>6} {'n/a':>10} {'n/a':>10} {'n/a':>10} {'n/a':>10} {'n/a':>10}")
            continue
        print(
            f"{name.ljust(name_width)}  "
            f"{format_count(int(stats['n'])):>6} "
            f"{format_value(stats['p50'], unit):>10} "
            f"{format_value(stats['p95'], unit):>10} "
            f"{format_value(stats['p99'], unit):>10} "
            f"{format_value(stats['mean'], unit):>10} "
            f"{format_value(stats['max'], unit):>10}"
        )


def print_count_table(title: str, counter: collections.Counter[str]) -> None:
    emit_section(title)
    if not counter:
        print("none")
        return
    width = max(len(name) for name in counter)
    for name, count in sorted(counter.items()):
        print(f"{name.ljust(width)}  {count}")


@dataclass
class RunSummary:
    path: Path
    schema_versions: list[int]
    total_records: int
    producer_records: list[dict]
    consumer_records: list[dict]
    produced_records: list[dict]
    consumed_records: list[dict]
    producer_status_counts: collections.Counter[str]
    consumer_status_counts: collections.Counter[str]
    producer_metrics: dict[str, dict[str, float | int] | None]
    consumer_metrics: dict[str, dict[str, float | int] | None]
    freshness_metrics: dict[str, dict[str, float | int] | None]
    delta_metrics: dict[str, dict[str, float | int] | None]
    time_span_ns: int | None


def filter_status(records: list[dict], field: str, status: str, skip_first: int) -> list[dict]:
    selected = [rec for rec in records if rec.get(field) == status]
    if skip_first <= 0:
        return selected
    return selected[skip_first:]


def compute_time_span_ns(records: list[dict]) -> int | None:
    timestamps: list[int] = []
    for rec in records:
        for key in ("event_ts_real_ns", "consumer_end_ts_real_ns"):
            value = get_field(rec, key)
            if value is not None and value > 0:
                timestamps.append(value)
    if len(timestamps) < 2:
        return None
    return max(timestamps) - min(timestamps)


def consecutive_deltas(records: list[dict], key: str) -> list[int]:
    values = [get_field(rec, key) for rec in records]
    deltas: list[int] = []
    previous: int | None = None
    for value in values:
        if value is None:
            continue
        if previous is not None:
            delta = value - previous
            if delta >= 0:
                deltas.append(delta)
        previous = value
    return deltas


def build_summary(path: Path, skip_first: int) -> RunSummary:
    records = load_jsonl(path)
    producer_records = [rec for rec in records if rec.get("source") == "producer"]
    consumer_records = [rec for rec in records if rec.get("source") == "consumer"]

    produced_records = filter_status(producer_records, "producer_status", "produced", skip_first)
    consumed_records = filter_status(consumer_records, "consumer_status", "consumed", skip_first)

    producer_by_frame = {
        frame_id: rec for rec in produced_records if (frame_id := get_field(rec, "frame_id")) is not None
    }
    consumer_by_frame = {
        frame_id: rec for rec in consumed_records if (frame_id := get_field(rec, "frame_id")) is not None
    }
    matched_frame_ids = sorted(producer_by_frame.keys() & consumer_by_frame.keys())

    freshness_capture_to_dequeue: list[int] = []
    freshness_dequeue_to_publish: list[int] = []
    freshness_publish_to_consumer: list[int] = []
    freshness_consumer_to_end: list[int] = []

    for frame_id in matched_frame_ids:
        producer = producer_by_frame[frame_id]
        consumer = consumer_by_frame[frame_id]
        age_dq = get_field(producer, "source_age_dequeue_ns")
        age_pr = get_field(producer, "source_age_publish_ready_ns")
        age_cs = get_field(consumer, "source_age_start_ns")
        age_ce = get_field(consumer, "source_age_end_ns")

        if age_dq is not None:
            freshness_capture_to_dequeue.append(age_dq)
        delta = safe_delta(age_dq, age_pr)
        if delta is not None:
            freshness_dequeue_to_publish.append(delta)
        delta = safe_delta(age_pr, age_cs)
        if delta is not None:
            freshness_publish_to_consumer.append(delta)
        delta = safe_delta(age_cs, age_ce)
        if delta is not None:
            freshness_consumer_to_end.append(delta)

    producer_metrics = {
        "source_age_dequeue_ns": describe(
            value for rec in produced_records if (value := get_field(rec, "source_age_dequeue_ns")) is not None
        ),
        "source_age_publish_ready_ns": describe(
            value for rec in produced_records if (value := get_field(rec, "source_age_publish_ready_ns")) is not None
        ),
        "dequeue_ns": describe(
            value for rec in produced_records if (value := get_duration(rec, "dequeue")) is not None
        ),
        "acquire_write_ns": describe(
            value for rec in produced_records if (value := get_duration(rec, "acquire_write")) is not None
        ),
        "preprocess_ns": describe(
            value for rec in produced_records if (value := get_duration(rec, "preprocess")) is not None
        ),
        "publish_ready_ns": describe(
            value for rec in produced_records if (value := get_duration(rec, "publish_ready")) is not None
        ),
        "requeue_ns": describe(
            value for rec in produced_records if (value := get_duration(rec, "requeue")) is not None
        ),
        "total_ns": describe(value for rec in produced_records if (value := get_duration(rec, "total")) is not None),
    }

    consumer_metrics = {
        "source_age_start_ns": describe(
            value for rec in consumed_records if (value := get_field(rec, "source_age_start_ns")) is not None
        ),
        "source_age_end_ns": describe(
            value for rec in consumed_records if (value := get_field(rec, "source_age_end_ns")) is not None
        ),
        "acquire_read_ns": describe(
            value for rec in consumed_records if (value := get_duration(rec, "acquire_read")) is not None
        ),
        "infer_ns": describe(value for rec in consumed_records if (value := get_duration(rec, "infer")) is not None),
        "postprocess_ns": describe(
            value for rec in consumed_records if (value := get_duration(rec, "postprocess")) is not None
        ),
        "publish_ns": describe(
            value for rec in consumed_records if (value := get_duration(rec, "publish")) is not None
        ),
        "release_ns": describe(
            value for rec in consumed_records if (value := get_duration(rec, "release")) is not None
        ),
        "total_ns": describe(value for rec in consumed_records if (value := get_duration(rec, "total")) is not None),
    }

    freshness_metrics = {
        "capture_to_dequeue_ns": describe(freshness_capture_to_dequeue),
        "dequeue_to_publish_ready_ns": describe(freshness_dequeue_to_publish),
        "publish_ready_to_consumer_start_ns": describe(freshness_publish_to_consumer),
        "consumer_start_to_end_ns": describe(freshness_consumer_to_end),
    }

    delta_metrics = {
        "producer_frame_delta": describe(consecutive_deltas(produced_records, "frame_id")),
        "producer_sequence_delta": describe(consecutive_deltas(produced_records, "sequence")),
        "consumer_frame_delta": describe(consecutive_deltas(consumed_records, "frame_id")),
        "consumer_sequence_delta": describe(consecutive_deltas(consumed_records, "sequence")),
    }

    schema_versions = sorted({int(value) for rec in records if (value := get_field(rec, "schema_version")) is not None})

    return RunSummary(
        path=path,
        schema_versions=schema_versions,
        total_records=len(records),
        producer_records=producer_records,
        consumer_records=consumer_records,
        produced_records=produced_records,
        consumed_records=consumed_records,
        producer_status_counts=collections.Counter(str(rec.get("producer_status")) for rec in producer_records),
        consumer_status_counts=collections.Counter(str(rec.get("consumer_status")) for rec in consumer_records),
        producer_metrics=producer_metrics,
        consumer_metrics=consumer_metrics,
        freshness_metrics=freshness_metrics,
        delta_metrics=delta_metrics,
        time_span_ns=compute_time_span_ns(records),
    )


def print_overview(summary: RunSummary, skip_first: int) -> None:
    emit_section(f"Run: {summary.path}")
    schemas = ", ".join(str(v) for v in summary.schema_versions) if summary.schema_versions else "n/a"
    print(f"schema_version(s): {schemas}")
    print(f"records: {summary.total_records}")
    print(f"producer samples: {len(summary.producer_records)} total, {len(summary.produced_records)} analyzed")
    print(f"consumer samples: {len(summary.consumer_records)} total, {len(summary.consumed_records)} analyzed")
    print(f"skip-first: {skip_first}")
    if summary.time_span_ns is not None:
        print(f"timespan_s: {summary.time_span_ns / 1.0e9:.3f}")


def print_summary(summary: RunSummary, skip_first: int) -> None:
    print_overview(summary, skip_first)
    print_count_table("Producer Status Counts", summary.producer_status_counts)
    emit_section("Producer Metrics")
    print_metric_table(
        [
            ("age@dequeue", summary.producer_metrics["source_age_dequeue_ns"]),
            ("age@publish_ready", summary.producer_metrics["source_age_publish_ready_ns"]),
            ("dequeue", summary.producer_metrics["dequeue_ns"]),
            ("acquire_write", summary.producer_metrics["acquire_write_ns"]),
            ("preprocess", summary.producer_metrics["preprocess_ns"]),
            ("publish_ready", summary.producer_metrics["publish_ready_ns"]),
            ("requeue", summary.producer_metrics["requeue_ns"]),
            ("total", summary.producer_metrics["total_ns"]),
        ]
    )

    print_count_table("Consumer Status Counts", summary.consumer_status_counts)
    emit_section("Consumer Metrics")
    print_metric_table(
        [
            ("age@consumer_start", summary.consumer_metrics["source_age_start_ns"]),
            ("age@consumer_end", summary.consumer_metrics["source_age_end_ns"]),
            ("acquire_read", summary.consumer_metrics["acquire_read_ns"]),
            ("infer", summary.consumer_metrics["infer_ns"]),
            ("postprocess", summary.consumer_metrics["postprocess_ns"]),
            ("publish", summary.consumer_metrics["publish_ns"]),
            ("release", summary.consumer_metrics["release_ns"]),
            ("total", summary.consumer_metrics["total_ns"]),
        ]
    )

    emit_section("Freshness Breakdown")
    print_metric_table(
        [
            ("capture->dequeue", summary.freshness_metrics["capture_to_dequeue_ns"]),
            ("dequeue->publish", summary.freshness_metrics["dequeue_to_publish_ready_ns"]),
            ("publish->consumer", summary.freshness_metrics["publish_ready_to_consumer_start_ns"]),
            ("consumer->end", summary.freshness_metrics["consumer_start_to_end_ns"]),
        ]
    )

    emit_section("Delta Behavior")
    print_metric_table(
        [
            ("producer frame delta", summary.delta_metrics["producer_frame_delta"]),
            ("producer seq delta", summary.delta_metrics["producer_sequence_delta"]),
            ("consumer frame delta", summary.delta_metrics["consumer_frame_delta"]),
            ("consumer seq delta", summary.delta_metrics["consumer_sequence_delta"]),
        ],
        unit="count",
    )


def metric_p(summary: RunSummary, section: str, metric: str, stat: str) -> float | int | None:
    stats = getattr(summary, section).get(metric)
    if stats is None:
        return None
    return stats.get(stat)


def print_compare(a: RunSummary, b: RunSummary, label_a: str, label_b: str) -> None:
    emit_section(f"Compare: {label_a} vs {label_b}")
    print(
        f"{'metric':36}  {label_a + ' p50':>12} {label_b + ' p50':>12} {'delta':>10}  "
        f"{label_a + ' p95':>12} {label_b + ' p95':>12} {'delta':>10}"
    )

    metrics: list[tuple[str, str, str, str]] = [
        ("producer age@dequeue", "producer_metrics", "source_age_dequeue_ns", "ns"),
        ("producer age@publish_ready", "producer_metrics", "source_age_publish_ready_ns", "ns"),
        ("producer total", "producer_metrics", "total_ns", "ns"),
        ("producer preprocess", "producer_metrics", "preprocess_ns", "ns"),
        ("consumer age@start", "consumer_metrics", "source_age_start_ns", "ns"),
        ("consumer age@end", "consumer_metrics", "source_age_end_ns", "ns"),
        ("consumer total", "consumer_metrics", "total_ns", "ns"),
        ("consumer infer", "consumer_metrics", "infer_ns", "ns"),
        ("consumer publish", "consumer_metrics", "publish_ns", "ns"),
        ("capture->dequeue", "freshness_metrics", "capture_to_dequeue_ns", "ns"),
        ("dequeue->publish", "freshness_metrics", "dequeue_to_publish_ready_ns", "ns"),
        ("publish->consumer", "freshness_metrics", "publish_ready_to_consumer_start_ns", "ns"),
        ("consumer->end", "freshness_metrics", "consumer_start_to_end_ns", "ns"),
        ("producer seq delta", "delta_metrics", "producer_sequence_delta", "count"),
        ("consumer frame delta", "delta_metrics", "consumer_frame_delta", "count"),
        ("consumer seq delta", "delta_metrics", "consumer_sequence_delta", "count"),
    ]

    for label, section, metric, unit in metrics:
        a50 = metric_p(a, section, metric, "p50")
        b50 = metric_p(b, section, metric, "p50")
        a95 = metric_p(a, section, metric, "p95")
        b95 = metric_p(b, section, metric, "p95")
        print(
            f"{label:36}  "
            f"{format_value(a50, unit):>12} {format_value(b50, unit):>12} {percent_change(a50, b50):>10}  "
            f"{format_value(a95, unit):>12} {format_value(b95, unit):>12} {percent_change(a95, b95):>10}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize and compare vision telemetry JSONL.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    summary = subparsers.add_parser("summary", help="Summarize one telemetry JSONL run.")
    summary.add_argument("jsonl", type=Path)
    summary.add_argument(
        "--skip-first", type=int, default=0, help="skip the first N produced/consumed samples before analysis"
    )

    compare = subparsers.add_parser("compare", help="Compare two telemetry JSONL runs.")
    compare.add_argument("run_a", type=Path)
    compare.add_argument("run_b", type=Path)
    compare.add_argument(
        "--skip-first", type=int, default=0, help="skip the first N produced/consumed samples in both runs"
    )
    compare.add_argument("--label-a", default="A")
    compare.add_argument("--label-b", default="B")

    return parser


def main(argv: list[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "summary":
            summary = build_summary(args.jsonl, args.skip_first)
            print_summary(summary, args.skip_first)
            return 0

        if args.command == "compare":
            run_a = build_summary(args.run_a, args.skip_first)
            run_b = build_summary(args.run_b, args.skip_first)
            print_summary(run_a, args.skip_first)
            print()
            print_summary(run_b, args.skip_first)
            print_compare(run_a, run_b, args.label_a, args.label_b)
            return 0o00
    except (OSError, ValueError) as exc:
        print(f"analyze_telemetry: {exc}", file=sys.stderr)
        return 1

    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
