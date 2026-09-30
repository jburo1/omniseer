#!/usr/bin/env python3
# ruff: noqa: E501
"""Render the detector-comparison page's static charts from retained evidence.

The controlled charts are computed from the saved replay JSONLs, visibility
annotations, and replay provenance.  The runtime values come from the tracked
physical-run table in ``results.md``: raw physical bundles are intentionally
not public.  The physical-trials poster is extracted from the tracked grid
video, so the documentation does not depend on a browser-selected video frame.

Run from any directory:

    python3 studies/detector_comparison/scan_final_recal/evidence/render_docs_assets.py
"""

from __future__ import annotations

import html
import json
import subprocess
from collections import OrderedDict
from pathlib import Path

STUDY_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = STUDY_DIR.parents[2]
OUTPUT_DIR = REPO_ROOT / "docs/assets/evidence/detector-comparison"
VISIBILITY_PATH = STUDY_DIR / "visibility.txt"
PROVENANCE_PATH = STUDY_DIR / "evidence/replay_provenance.json"
JSONL_DIR = STUDY_DIR / "evidence/replay_jsonl"
RESULTS_PATH = STUDY_DIR / "results.md"
PHYSICAL_GRID_PATH = STUDY_DIR / "evidence/physical_trials_grid_2x3.mp4"
PHYSICAL_POSTER_PATH = REPO_ROOT / "docs/assets/evidence/detector-comparison-physical-poster.webp"

MODELS = OrderedDict(
    (
        ("v2-S FP", "v2s_fp.jsonl"),
        ("v2-S INT8", "v2s_int8.jsonl"),
        ("v2-M FP", "v2m_fp.jsonl"),
        ("v2-M INT8", "v2m_int8.jsonl"),
        ("v2-L FP", "v2l_fp.jsonl"),
        ("v2-L classifier-path localization probe", "v2l_hybrid.jsonl"),
    )
)

PHYSICAL_CONFIGURATIONS = (
    "v2-S FP",
    "v2-S INT8",
    "v2-M FP",
    "v2-M INT8",
    "v2-L FP",
    "v2-L hybrid (artifact identity unproven)",
)

INK = "#20242b"
MUTED = "#58616f"
GRID = "#d8dde6"
ACCENT = "#4057a5"
ACCENT_LIGHT = "#c8d1f1"
MIDDLE = "#a76416"
HIGH = "#216b4f"
FONT = "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif"


def svg_document(width: int, height: int, title: str, description: str, body: str) -> str:
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
  <title id="title">{html.escape(title)}</title>
  <desc id="desc">{html.escape(description)}</desc>
  <rect width="100%" height="100%" fill="white"/>
  <style>text {{ font-family: {FONT}; }} .muted {{ fill: {MUTED}; }} .small {{ font-size: 13px; }} .label {{ font-size: 14px; }} .value {{ font-size: 14px; font-weight: 650; }}</style>
{body}
</svg>
'''


def write_svg(name: str, width: int, height: int, title: str, description: str, body: str) -> None:
    (OUTPUT_DIR / name).write_text(svg_document(width, height, title, description, body), encoding="utf-8")


def parse_visibility() -> OrderedDict[str, list[tuple[int, int]]]:
    active: dict[str, int] = {}
    intervals: OrderedDict[str, list[tuple[int, int]]] = OrderedDict()
    for line in VISIBILITY_PATH.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 3 or parts[-2] not in {"start", "end"}:
            continue
        class_name, marker, raw_frame = " ".join(parts[:-2]), parts[-2], parts[-1]
        if marker == "start":
            if class_name in active:
                raise ValueError(f"overlapping visibility annotation for {class_name}")
            active[class_name] = int(raw_frame)
            intervals.setdefault(class_name, [])
        elif marker == "end":
            start = active.pop(class_name, None)
            if start is None:
                raise ValueError(f"end without start for {class_name}")
            intervals[class_name].append((start, int(raw_frame)))
    if active:
        raise ValueError(f"unclosed visibility annotations: {', '.join(active)}")
    return intervals


def read_detection_frames(filename: str) -> dict[str, set[int]]:
    detections: dict[str, set[int]] = {}
    with (JSONL_DIR / filename).open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            frame = record["frame_index"]
            for detection in record["detections"]:
                detections.setdefault(detection["class_name"], set()).add(frame)
    return detections


def in_intervals(frame: int, intervals: list[tuple[int, int]]) -> bool:
    return any(start <= frame <= end for start, end in intervals)


def controlled_metrics() -> tuple[
    OrderedDict[str, int], OrderedDict[str, dict[str, int]], OrderedDict[str, list[tuple[int, int]]]
]:
    annotations = parse_visibility()
    provenance = json.loads(PROVENANCE_PATH.read_text(encoding="utf-8"))
    vocabulary = set(provenance["classes"])
    missing = [label for label in annotations if label not in vocabulary]
    if missing != []:
        raise ValueError(f"annotations outside replay vocabulary: {missing}")
    per_model: OrderedDict[str, dict[str, int]] = OrderedDict()
    totals: OrderedDict[str, int] = OrderedDict()
    for model, filename in MODELS.items():
        detected_frames = read_detection_frames(filename)
        class_hits = {
            label: sum(in_intervals(frame, ranges) for frame in detected_frames.get(label, set()))
            for label, ranges in annotations.items()
        }
        per_model[model] = class_hits
        totals[model] = sum(class_hits.values())
    return totals, per_model, annotations


def physical_metrics() -> OrderedDict[str, tuple[float, float, float]]:
    """Return FPS, p50 ms, p95 ms from the retained public summary table."""
    rows: OrderedDict[str, tuple[float, float, float]] = OrderedDict()
    in_table = False
    for line in RESULTS_PATH.read_text(encoding="utf-8").splitlines():
        if line.startswith("| Configuration | Outcome | First detection"):
            in_table = True
            continue
        if not in_table:
            continue
        if not line.startswith("|"):
            break
        if set(line.replace("|", "").strip()) <= {"-", ":", " "}:
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) != 13:
            raise ValueError(f"unexpected physical summary row: {line}")
        model, outcome, _, _, target_loss, fps, p50, p95, *_ = cells
        if outcome != "success" or target_loss != "0":
            raise ValueError(f"physical summary is not an all-success/no-loss run: {line}")
        rows[model] = (float(fps), float(p50.removesuffix(" ms")), float(p95.removesuffix(" ms")))
    if list(rows) != list(PHYSICAL_CONFIGURATIONS):
        raise ValueError(f"physical summary models differ from replay models: {list(rows)}")
    return rows


def render_coverage(totals: OrderedDict[str, int], opportunities: int) -> None:
    width, height = 960, 355
    left, right, top, bottom = 160, 70, 72, 58
    plot_width, plot_height = width - left - right, height - top - bottom
    max_rate = 45
    parts = [
        '<text x="32" y="35" font-size="21" font-weight="700">Controlled visible class-frame coverage</text>',
        '<text x="32" y="57" class="muted small">Same 1,222 source frames; 5,679 class-frame opportunities per configuration</text>',
    ]
    for tick in range(0, 46, 10):
        x = left + plot_width * tick / max_rate
        parts.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_height}" stroke="{GRID}"/>')
        parts.append(f'<text x="{x:.1f}" y="{height - 25}" text-anchor="middle" class="muted small">{tick}%</text>')
    row_step = plot_height / len(totals)
    for index, (model, hits) in enumerate(totals.items()):
        y = top + index * row_step + 6
        rate = hits / opportunities * 100
        fill = HIGH if model == "v2-M FP" else MIDDLE if model == "v2-M INT8" else ACCENT
        parts.append(f'<text x="{left - 14}" y="{y + 20:.1f}" text-anchor="end" class="label">{model}</text>')
        parts.append(
            f'<rect x="{left}" y="{y:.1f}" width="{plot_width * rate / max_rate:.1f}" height="28" rx="3" fill="{fill}"/>'
        )
        parts.append(
            f'<text x="{left + plot_width * rate / max_rate + 9:.1f}" y="{y + 20:.1f}" class="value">{rate:.1f}%</text>'
        )
    write_svg(
        "controlled-coverage.svg",
        width,
        height,
        "Controlled visible class-frame coverage by configuration",
        "Bar chart of visible class-frame coverage across six configurations. v2-M FP leads at 41.7 percent.",
        "\n".join(parts),
    )


def render_runtime(physical: OrderedDict[str, tuple[float, float, float]]) -> None:
    width, height = 960, 430
    left, right, top, bottom = 152, 55, 105, 66
    gap = 56
    panel_width = (width - left - right - gap) / 2
    plot_height = height - top - bottom
    parts = [
        '<text x="32" y="35" font-size="21" font-weight="700">Independent physical-run runtime observations</text>',
        '<text x="32" y="57" class="muted small">One ROCK 5B+ closed-loop run per configuration</text>',
    ]
    panels = (
        ("Mean consumer throughput (FPS)", 18, (0, 4, 8, 12, 16), 0),
        ("RKNN inference p50 / p95 (ms)", 450, (0, 100, 200, 300, 400), 1),
    )
    row_step = plot_height / len(physical)
    for heading, maximum, ticks, panel in panels:
        x0 = left + panel * (panel_width + gap)
        parts.append(f'<text x="{x0}" y="{top - 24}" class="value">{heading}</text>')
        for value in ticks:
            x = x0 + panel_width * value / maximum
            parts.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_height}" stroke="{GRID}"/>')
            parts.append(
                f'<text x="{x:.1f}" y="{height - 28}" text-anchor="middle" class="muted small">{value:.0f}</text>'
            )
        for index, (model, values) in enumerate(physical.items()):
            fps, p50, p95 = values
            y = top + index * row_step + 5
            if panel == 0:
                fill = HIGH if model == "v2-S INT8" else MIDDLE if model == "v2-M INT8" else ACCENT
                parts.append(f'<text x="{x0 - 12}" y="{y + 18:.1f}" text-anchor="end" class="label">{model}</text>')
                parts.append(
                    f'<rect x="{x0}" y="{y:.1f}" width="{panel_width * fps / maximum:.1f}" height="25" rx="3" fill="{fill}"/>'
                )
                parts.append(
                    f'<text x="{x0 + panel_width * fps / maximum + 8:.1f}" y="{y + 18:.1f}" class="value">{fps:.2f}</text>'
                )
            else:
                p50_width = panel_width * p50 / maximum
                p95_width = panel_width * p95 / maximum
                parts.append(
                    f'<rect x="{x0}" y="{y:.1f}" width="{p95_width:.1f}" height="25" rx="3" fill="{ACCENT_LIGHT}"/>'
                )
                parts.append(f'<rect x="{x0}" y="{y:.1f}" width="{p50_width:.1f}" height="25" rx="3" fill="{ACCENT}"/>')
                parts.append(
                    f'<text x="{x0 + p95_width + 8:.1f}" y="{y + 18:.1f}" class="value">{p50:.0f} / {p95:.0f}</text>'
                )
    parts.extend(
        [
            f'<rect x="{width - 260}" y="{height - 58}" width="14" height="14" fill="{ACCENT}"/><text x="{width - 240}" y="{height - 46}" class="muted small">p50</text>',
            f'<rect x="{width - 180}" y="{height - 58}" width="14" height="14" fill="{ACCENT_LIGHT}"/><text x="{width - 160}" y="{height - 46}" class="muted small">p95</text>',
        ]
    )
    write_svg(
        "physical-runtime.svg",
        width,
        height,
        "Physical-run inference and throughput comparison",
        "Two-panel chart showing mean consumer throughput and RKNN inference p50 and p95 from one physical run per configuration.",
        "\n".join(parts),
    )


def render_scatter(
    totals: OrderedDict[str, int], physical: OrderedDict[str, tuple[float, float, float]], opportunities: int
) -> None:
    width, height = 960, 470
    left, right, top, bottom = 95, 70, 90, 84
    plot_width, plot_height = width - left - right, height - top - bottom
    x_min, x_max, y_min, y_max = 24, 43, 0, 18
    parts = [
        '<text x="32" y="35" font-size="21" font-weight="700">Coverage versus physical-run throughput</text>',
        '<text x="32" y="57" class="muted small">Axes combine separate evidence streams: controlled replay coverage and one physical-run FPS</text>',
    ]
    for tick in (25, 30, 35, 40):
        x = left + (tick - x_min) / (x_max - x_min) * plot_width
        parts.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_height}" stroke="{GRID}"/>')
        parts.append(f'<text x="{x:.1f}" y="{height - 44}" text-anchor="middle" class="muted small">{tick}%</text>')
    for tick in (0, 4, 8, 12, 16):
        y = top + plot_height - (tick - y_min) / (y_max - y_min) * plot_height
        parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="{GRID}"/>')
        parts.append(f'<text x="{left - 12}" y="{y + 5:.1f}" text-anchor="end" class="muted small">{tick}</text>')
    parts.append(
        f'<line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="{INK}"/>'
    )
    parts.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="{INK}"/>')
    parts.append(f'<text x="{left}" y="{top - 10}" class="label">Physical-run mean consumer FPS</text>')
    parts.append(
        f'<text x="{left + plot_width / 2}" y="{height - 14}" text-anchor="middle" class="label">Controlled visible class-frame coverage</text>'
    )
    offsets = {
        "v2-S FP": (10, -10),
        "v2-S INT8": (-10, -13),
        "v2-M FP": (10, 20),
        "v2-M INT8": (10, -10),
        "v2-L FP": (-8, 19),
    }
    for model, hits in list(totals.items())[:5]:
        rate = hits / opportunities * 100
        fps = physical[model][0]
        x = left + (rate - x_min) / (x_max - x_min) * plot_width
        y = top + plot_height - (fps - y_min) / (y_max - y_min) * plot_height
        fill = HIGH if model in {"v2-M FP", "v2-S INT8"} else MIDDLE if model == "v2-M INT8" else ACCENT
        dx, dy = offsets[model]
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="{fill}" stroke="white" stroke-width="2"/>')
        anchor = "end" if dx < 0 else "start"
        parts.append(f'<text x="{x + dx:.1f}" y="{y + dy:.1f}" text-anchor="{anchor}" class="value">{model}</text>')
    write_svg(
        "coverage-throughput.svg",
        width,
        height,
        "Controlled coverage versus physical-run throughput",
        "Scatter plot that combines controlled replay visible class-frame coverage on the x axis with mean consumer FPS from independent physical runs on the y axis.",
        "\n".join(parts),
    )


def render_heatmap(
    per_model: OrderedDict[str, dict[str, int]], annotations: OrderedDict[str, list[tuple[int, int]]]
) -> None:
    displayed_annotations = OrderedDict((label, ranges) for label, ranges in annotations.items() if label != "plant")
    width, height = 1060, 710
    left, right, top, bottom = 188, 40, 90, 48
    plot_width, plot_height = width - left - right, height - top - bottom
    cell_width = plot_width / len(MODELS)
    cell_height = plot_height / len(displayed_annotations)
    parts = [
        '<text x="32" y="35" font-size="21" font-weight="700">Per-class controlled visible-frame coverage</text>',
        '<text x="32" y="57" class="muted small">Each cell is detected annotated frames ÷ visible class-frame opportunities; literal labels are matched</text>',
    ]
    for col, model in enumerate(MODELS):
        x = left + (col + 0.5) * cell_width
        parts.append(f'<text x="{x:.1f}" y="{top - 14}" text-anchor="middle" class="small">{model}</text>')
    for row, (label, ranges) in enumerate(displayed_annotations.items()):
        y = top + row * cell_height
        opportunities = sum(end - start + 1 for start, end in ranges)
        parts.append(
            f'<text x="{left - 10}" y="{y + cell_height * 0.68:.1f}" text-anchor="end" fill="{INK}" class="label">{label}</text>'
        )
        for col, model in enumerate(MODELS):
            rate = per_model[model][label] / opportunities
            shade = int(246 - rate * 130)
            fill = f"rgb({max(62, shade - 25)}, {max(85, shade - 5)}, {max(120, shade)})"
            x = left + col * cell_width
            value_colour = "white" if rate > 0.52 else INK
            parts.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{cell_width - 2:.1f}" height="{cell_height - 2:.1f}" fill="{fill}"/>'
            )
            parts.append(
                f'<text x="{x + cell_width / 2:.1f}" y="{y + cell_height * 0.68:.1f}" text-anchor="middle" fill="{value_colour}" class="small">{rate * 100:.0f}%</text>'
            )
    write_svg(
        "per-class-coverage.svg",
        width,
        height,
        "Per-class controlled visible-frame detection heatmap",
        "Heatmap of visible-frame coverage by annotated class and configuration. The mismatched plant annotation is omitted from this class-level view.",
        "\n".join(parts),
    )


def render_physical_poster() -> None:
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-ss",
            "00:00:25",
            "-i",
            str(PHYSICAL_GRID_PATH),
            "-frames:v",
            "1",
            "-vf",
            "scale=1280:-2",
            "-q:v",
            "72",
            str(PHYSICAL_POSTER_PATH),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    totals, per_model, annotations = controlled_metrics()
    physical = physical_metrics()
    opportunities = sum(end - start + 1 for ranges in annotations.values() for start, end in ranges)
    expected = {
        "v2-S FP": 2126,
        "v2-S INT8": 1853,
        "v2-M FP": 2367,
        "v2-M INT8": 2055,
        "v2-L FP": 2363,
        "v2-L classifier-path localization probe": 1440,
    }
    if opportunities != 5679 or totals != expected:
        raise ValueError(f"controlled metrics no longer match retained report: {opportunities=}, {totals=}")
    render_coverage(totals, opportunities)
    render_runtime(physical)
    render_scatter(totals, physical, opportunities)
    render_heatmap(per_model, annotations)
    render_physical_poster()


if __name__ == "__main__":
    main()
