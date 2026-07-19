"""Read-only inspection CLI for local Omniseer perception run bundles."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.bundle import ERROR_FIELDS, SCHEMA_VERSION, SummaryAccumulator

STATE_COMPLETE = "complete"
STATE_IN_PROGRESS = "in_progress"
STATE_INCOMPLETE = "incomplete"


@dataclass(frozen=True)
class InspectionIssue:
    code: str
    message: str
    path: str = ""
    line: int | None = None

    def to_json(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "code": self.code,
            "message": self.message,
        }
        if self.path:
            result["path"] = self.path
        if self.line is not None:
            result["line"] = self.line
        return result


@dataclass(frozen=True)
class JsonlScan:
    message_count: int
    issues: tuple[InspectionIssue, ...]
    records: tuple[dict[str, Any], ...]


@dataclass(frozen=True)
class RunInspection:
    run_id: str
    path: Path
    state: str
    issues: tuple[InspectionIssue, ...]
    started_at: str | None
    ended_at: str | None
    duration_sec: float | None
    message_counts: dict[str, int]
    configured_classes: tuple[str, ...]
    detections_by_class: dict[str, int]
    perf: dict[str, Any]
    errors: dict[str, int]
    dropped_records: dict[str, int]

    def to_json(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "path": str(self.path),
            "state": self.state,
            "issues": [issue.to_json() for issue in self.issues],
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "duration_sec": self.duration_sec,
            "message_counts": dict(self.message_counts),
            "configured_classes": list(self.configured_classes),
            "detections_by_class": dict(self.detections_by_class),
            "perf": dict(self.perf),
            "errors": dict(self.errors),
            "dropped_records": dict(self.dropped_records),
        }


def inspect_run(run_dir: Path) -> RunInspection:
    path = run_dir
    issues: list[InspectionIssue] = []
    manifest = _read_manifest(path / "manifest.yaml", issues)
    detections_scan = _scan_jsonl(path / "detections.jsonl", "detections")
    perf_scan = _scan_jsonl(path / "perf.jsonl", "perf")
    issues.extend(detections_scan.issues)
    issues.extend(perf_scan.issues)

    fallback_summary = _fallback_summary(
        run_id=_manifest_string(manifest, "run_id") or path.name,
        detections_records=detections_scan.records,
        perf_records=perf_scan.records,
    )
    summary = _read_summary(path / "summary.json", issues)
    summary_missing = summary is None and not (path / "summary.json").exists()

    started_at = _manifest_string(manifest, "started_at")
    ended_at = _manifest_string(manifest, "ended_at")
    manifest_missing = any(issue.code == "missing_manifest" for issue in issues)
    jsonl_has_issues = bool(detections_scan.issues or perf_scan.issues)
    summary_has_issues = any(issue.code in {"invalid_summary", "unreadable_summary"} for issue in issues)

    if ended_at is None and not manifest_missing:
        issues.append(
            InspectionIssue(
                code="run_open",
                message="manifest ended_at is null; run did not finalize",
                path=str(path / "manifest.yaml"),
            )
        )
    if summary_missing:
        issues.append(
            InspectionIssue(
                code="missing_summary",
                message="summary.json is missing",
                path=str(path / "summary.json"),
            )
        )

    effective_summary = summary or fallback_summary
    state = _state_from_issues(
        ended_at=ended_at,
        manifest_missing=manifest_missing,
        summary_missing=summary_missing,
        summary_has_issues=summary_has_issues,
        jsonl_has_issues=jsonl_has_issues,
    )

    return RunInspection(
        run_id=_summary_string(effective_summary, "run_id") or _manifest_string(manifest, "run_id") or path.name,
        path=path,
        state=state,
        issues=tuple(issues),
        started_at=started_at,
        ended_at=ended_at,
        duration_sec=_summary_float(effective_summary, "duration_sec"),
        message_counts=_summary_counts(effective_summary),
        configured_classes=tuple(_manifest_list(manifest, "classes")),
        detections_by_class=_summary_int_map(effective_summary, "detections_by_class"),
        perf=_summary_dict(effective_summary, "perf"),
        errors=_summary_errors(effective_summary),
        dropped_records=_summary_int_map(effective_summary, "dropped_records"),
    )


def list_runs(root: Path) -> list[RunInspection]:
    if not root.exists() or not root.is_dir():
        return []
    run_dirs = [path for path in sorted(root.iterdir(), key=lambda item: item.name) if path.is_dir()]
    return [inspect_run(path) for path in run_dirs]


def format_run_summary(inspection: RunInspection) -> str:
    lines = [
        f"Run: {inspection.run_id}",
        f"Path: {inspection.path}",
        f"State: {inspection.state}",
        f"Started: {_display_value(inspection.started_at)}",
        f"Ended: {_display_value(inspection.ended_at)}",
        f"Duration: {_format_duration(inspection.duration_sec)}",
        f"Messages: detections={inspection.message_counts.get('detections', 0)} "
        f"perf={inspection.message_counts.get('perf', 0)}",
        f"Configured classes: {_format_sequence(inspection.configured_classes)}",
        f"Observed classes: {_format_counts(inspection.detections_by_class)}",
        f"Perf: {_format_perf(inspection.perf)}",
        f"Errors: {_format_counts(inspection.errors)}",
        f"Dropped records: {_format_counts(inspection.dropped_records)}",
    ]
    if inspection.issues:
        lines.append("Issues:")
        lines.extend(f"- {_format_issue(issue)}" for issue in inspection.issues)
    return "\n".join(lines)


def format_run_list(inspections: Sequence[RunInspection], *, root: Path) -> str:
    if not inspections:
        return f"no runs found under {root}"

    rows = [
        "run_id     state        started_at                 ended_at                   duration  detections  perf  "
        "classes  issues"
    ]
    for item in inspections:
        class_count = len(set(item.configured_classes) | set(item.detections_by_class))
        rows.append(
            f"{_clip(item.run_id, 10):<10} {_clip(item.state, 11):<11} "
            f"{_clip(_display_value(item.started_at), 26):<26} "
            f"{_clip(_display_value(item.ended_at), 26):<26} "
            f"{_format_duration(item.duration_sec):>8} "
            f"{item.message_counts.get('detections', 0):>10} "
            f"{item.message_counts.get('perf', 0):>5} "
            f"{class_count:>7} "
            f"{_format_issue_codes(item.issues)}"
        )
    return "\n".join(rows)


def inspect_run_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Inspect one local Omniseer perception run bundle.")
    parser.add_argument("run_dir", help="path to a runs/<run_id> bundle")
    parser.add_argument("--json", action="store_true", help="emit a stable JSON object")
    args = parser.parse_args(argv)

    inspection = inspect_run(Path(args.run_dir))
    if args.json:
        print(json.dumps(inspection.to_json(), indent=2, sort_keys=True))
    else:
        print(format_run_summary(inspection))


def list_runs_main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="List local Omniseer perception run bundles.")
    parser.add_argument("--root", default="runs", help="directory containing run bundle directories")
    args = parser.parse_args(argv)
    root = Path(args.root)

    print(format_run_list(list_runs(root), root=root))


def _read_manifest(path: Path, issues: list[InspectionIssue]) -> dict[str, Any]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        issues.append(InspectionIssue(code="missing_manifest", message="manifest.yaml is missing", path=str(path)))
        return {}
    except OSError as exc:
        issues.append(
            InspectionIssue(
                code="unreadable_manifest",
                message=f"manifest.yaml could not be read: {exc}",
                path=str(path),
            )
        )
        return {}
    return _parse_generated_manifest(lines)


def _parse_generated_manifest(lines: Sequence[str]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    section: str | None = None
    for raw in lines:
        if not raw.strip():
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        stripped = raw.strip()

        if indent == 0:
            key, raw_value = _split_yaml_pair(stripped)
            section = key if raw_value == "" else None
            if raw_value == "":
                result[key] = [] if key == "classes" else {}
            else:
                result[key] = _parse_yaml_scalar(raw_value)
            continue

        if section == "classes" and stripped.startswith("- "):
            classes = result.setdefault("classes", [])
            if isinstance(classes, list):
                classes.append(_parse_yaml_scalar(stripped[2:].strip()))
            continue

        if section and indent == 2 and ":" in stripped:
            key, raw_value = _split_yaml_pair(stripped)
            nested = result.setdefault(section, {})
            if isinstance(nested, dict):
                nested[key] = _parse_yaml_scalar(raw_value)

    return result


def _split_yaml_pair(value: str) -> tuple[str, str]:
    if ":" not in value:
        return value, ""
    key, raw_value = value.split(":", 1)
    return key.strip(), raw_value.strip()


def _parse_yaml_scalar(value: str) -> Any:
    if value == "null":
        return None
    if value == "[]":
        return []
    if value == "{}":
        return {}
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        pass
    if value in {"true", "false"}:
        return value == "true"
    try:
        return int(value)
    except ValueError:
        return value


def _scan_jsonl(path: Path, stream: str) -> JsonlScan:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return JsonlScan(
            message_count=0,
            issues=(
                InspectionIssue(code=f"missing_{stream}_jsonl", message=f"{path.name} is missing", path=str(path)),
            ),
            records=(),
        )
    except OSError as exc:
        return JsonlScan(
            message_count=0,
            issues=(
                InspectionIssue(
                    code=f"unreadable_{stream}_jsonl",
                    message=f"{path.name} could not be read: {exc}",
                    path=str(path),
                ),
            ),
            records=(),
        )

    issues: list[InspectionIssue] = []
    records: list[dict[str, Any]] = []
    for index, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            issues.append(
                InspectionIssue(
                    code=f"malformed_{stream}_jsonl",
                    message=f"{path.name} contains malformed JSONL: {exc.msg}",
                    path=str(path),
                    line=index,
                )
            )
            continue
        if not isinstance(record, dict):
            issues.append(
                InspectionIssue(
                    code=f"invalid_{stream}_jsonl_record",
                    message=f"{path.name} line is not a JSON object",
                    path=str(path),
                    line=index,
                )
            )
            continue
        records.append(record)
    return JsonlScan(message_count=len(records), issues=tuple(issues), records=tuple(records))


def _read_summary(path: Path, issues: list[InspectionIssue]) -> dict[str, Any] | None:
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:
        issues.append(
            InspectionIssue(code="unreadable_summary", message=f"summary.json could not be read: {exc}", path=str(path))
        )
        return None

    try:
        summary = json.loads(raw)
    except json.JSONDecodeError as exc:
        issues.append(
            InspectionIssue(
                code="invalid_summary",
                message=f"summary.json contains malformed JSON: {exc.msg}",
                path=str(path),
                line=exc.lineno,
            )
        )
        return None
    if not isinstance(summary, dict):
        issues.append(
            InspectionIssue(code="invalid_summary", message="summary.json root is not a JSON object", path=str(path))
        )
        return None
    return summary


def _fallback_summary(
    *, run_id: str, detections_records: Sequence[dict[str, Any]], perf_records: Sequence[dict[str, Any]]
) -> dict[str, Any]:
    accumulator = SummaryAccumulator(run_id)
    for record in detections_records:
        accumulator.add_detection_record(record)
    for record in perf_records:
        accumulator.add_perf_record(record)
    return accumulator.build_summary(0.0)


def _state_from_issues(
    *,
    ended_at: str | None,
    manifest_missing: bool,
    summary_missing: bool,
    summary_has_issues: bool,
    jsonl_has_issues: bool,
) -> str:
    if (
        not manifest_missing
        and ended_at is None
        and summary_missing
        and not summary_has_issues
        and not jsonl_has_issues
    ):
        return STATE_IN_PROGRESS
    if manifest_missing or ended_at is None or summary_missing or summary_has_issues or jsonl_has_issues:
        return STATE_INCOMPLETE
    return STATE_COMPLETE


def _manifest_string(manifest: dict[str, Any], key: str) -> str | None:
    value = manifest.get(key)
    return value if isinstance(value, str) and value else None


def _manifest_list(manifest: dict[str, Any], key: str) -> list[str]:
    value = manifest.get(key)
    if not isinstance(value, list):
        return []
    return [item for item in value if isinstance(item, str) and item]


def _summary_string(summary: dict[str, Any], key: str) -> str | None:
    value = summary.get(key)
    return value if isinstance(value, str) and value else None


def _summary_float(summary: dict[str, Any], key: str) -> float | None:
    value = summary.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _summary_counts(summary: dict[str, Any]) -> dict[str, int]:
    counts = _summary_int_map(summary, "message_counts")
    return {
        "detections": counts.get("detections", 0),
        "perf": counts.get("perf", 0),
    }


def _summary_dict(summary: dict[str, Any], key: str) -> dict[str, Any]:
    value = summary.get(key)
    return dict(value) if isinstance(value, dict) else {}


def _summary_int_map(summary: dict[str, Any], key: str) -> dict[str, int]:
    value = summary.get(key)
    if not isinstance(value, dict):
        return {}
    return {
        str(item_key): int(item_value)
        for item_key, item_value in value.items()
        if isinstance(item_value, int) and not isinstance(item_value, bool)
    }


def _summary_errors(summary: dict[str, Any]) -> dict[str, int]:
    errors = _summary_int_map(summary, "errors")
    return {field_name: errors.get(field_name, 0) for field_name in ERROR_FIELDS}


def _display_value(value: object) -> str:
    if value is None or value == "":
        return "-"
    return str(value)


def _format_duration(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.1f}s"


def _format_sequence(values: Sequence[str]) -> str:
    return ", ".join(values) if values else "-"


def _format_counts(values: dict[str, int]) -> str:
    nonzero = {key: value for key, value in values.items() if value != 0}
    if not nonzero:
        return "-"
    return ", ".join(f"{key}={value}" for key, value in sorted(nonzero.items()))


def _format_perf(perf: dict[str, Any]) -> str:
    if not perf:
        return "-"
    fields = ("producer_fps_mean", "consumer_fps_mean", "infer_ms_mean", "infer_ms_p95")
    parts = []
    for field_name in fields:
        value = perf.get(field_name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        parts.append(f"{field_name}={float(value):.2f}")
    return ", ".join(parts) if parts else "-"


def _format_issue(issue: InspectionIssue) -> str:
    location = issue.path
    if issue.line is not None:
        location = f"{location}:{issue.line}" if location else f"line {issue.line}"
    return f"{issue.code}: {issue.message}" + (f" ({location})" if location else "")


def _format_issue_codes(issues: Sequence[InspectionIssue]) -> str:
    if not issues:
        return "-"
    return ",".join(issue.code for issue in issues)


def _clip(value: str, width: int) -> str:
    if len(value) <= width:
        return value
    if width <= 1:
        return value[:width]
    return value[: width - 1] + "+"


__all__ = [
    "InspectionIssue",
    "RunInspection",
    "format_run_list",
    "format_run_summary",
    "inspect_run",
    "inspect_run_main",
    "list_runs",
    "list_runs_main",
]
