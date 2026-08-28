import hashlib
import json
from pathlib import Path

import pytest
from omniseer_experiments.comparison_report import (
    MODEL_SPECS,
    ReplayDetection,
    ReplayFrame,
    absent_truth_metrics,
    aggregate_replay,
    load_replay_comparison,
    parse_scene_truth,
    present_truth_metrics,
    resolve_trial_metrics,
    write_comparison_report,
)


def _detection(class_name: str, score: float) -> dict:
    return {
        "class_id": 0,
        "class_name": class_name,
        "score": score,
        "bbox": [0.0, 0.0, 10.0, 10.0],
    }


def _frames() -> list[dict]:
    return [
        {"frame_index": 0, "timestamp_sec": 0.0, "detections": [_detection("chair", 0.5), _detection("chair", 0.9)]},
        {"frame_index": 1, "timestamp_sec": 1.0 / 30.0, "detections": [_detection("book", 0.7)]},
        {"frame_index": 2, "timestamp_sec": 2.0 / 30.0, "detections": [_detection("dog", 0.2)]},
    ]


def _write_comparison(reference: Path, name: str = "task", streams: dict[str, list[dict]] | None = None) -> None:
    directory = reference / "video" / "comparison" / name
    directory.mkdir(parents=True, exist_ok=True)
    source = reference / "video" / "source.ts"
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_bytes(b"fixture-raw-source")
    video = directory / "comparison.mp4"
    video.write_bytes(b"fixture-video")
    relative_directory = f"video/comparison/{name}"
    provenance = {
        "schema_version": 2,
        "comparison_name": name,
        "source": {"path": "video/source.ts", "sha256": _sha256(source)},
        "classes": ["chair", "book", "dog"],
        "postprocess": {"score_threshold": 0.25, "nms_iou_threshold": 0.45, "max_detections": 100},
        "models": [
            {"label": spec.label, "artifact_name": f"{spec.label}.rknn", "sha256": f"hash-{index}"}
            for index, spec in enumerate(MODEL_SPECS)
        ],
        "detection_jsonl": [f"{relative_directory}/{spec.detections_filename}" for spec in MODEL_SPECS],
        "output": {"path": f"{relative_directory}/comparison.mp4", "sha256": _sha256(video)},
    }
    (directory / "provenance.json").write_text(json.dumps(provenance), encoding="utf-8")
    for spec in MODEL_SPECS:
        records = streams[spec.label] if streams else _frames()
        (directory / spec.detections_filename).write_text(
            "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
        )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_trial(run_dir: Path, variant: str, precision: str, *, telemetry: bool = True) -> None:
    run_dir.mkdir(parents=True)
    run_dir.joinpath("manifest.yaml").write_text(
        "\n".join(
            [
                f'run_id: "{run_dir.name}"',
                'git_sha: "trial-git"',
                "model:",
                '  family: "yolo-world"',
                f'  variant: "{variant}"',
                f'  precision: "{precision}"',
                '  backend: "rknn"',
                f'  detector_model_path: "models/yolo_world_{variant}_{precision}.rknn"',
                "container:",
                '  image_digest: "sha256:trial-image"',
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    if not telemetry:
        return
    run_dir.joinpath("autonomy.jsonl").write_text(
        "\n".join(
            [
                json.dumps({"time_sec": 0.2, "event": "first_detection", "state": "lock"}),
                json.dumps({"time_sec": 0.5, "event": "target_lost", "state": "center"}),
                json.dumps({"time_sec": 1.0, "event": "succeeded", "state": "success"}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    run_dir.joinpath("perf.jsonl").write_text(
        "\n".join(
            [
                json.dumps({"consumer_fps": 20.0, "last_infer_ms": 8.0}),
                json.dumps({"consumer_fps": 18.0, "last_infer_ms": 12.0}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    run_dir.joinpath("pipeline_telemetry.jsonl").write_text(
        json.dumps({"source_age_end_ns": 12_000_000}) + "\n", encoding="utf-8"
    )
    run_dir.joinpath("system.jsonl").write_text(
        json.dumps(
            {
                "cpu_percent": 40.0,
                "memory_used_mb": 800.0,
                "soc_temp_c": 62.0,
                "thermal": {"throttled": False},
            }
        )
        + "\n",
        encoding="utf-8",
    )


def _write_all_trials(root: Path) -> list[Path]:
    trials = []
    for spec in MODEL_SPECS:
        run_dir = root / spec.label.replace(" ", "_").replace("-", "").lower()
        _write_trial(run_dir, f"v2{spec.variant}", spec.precision)
        trials.append(run_dir)
    return trials


def test_scene_truth_accepts_multiple_ranges_and_rejects_invalid_values(tmp_path: Path) -> None:
    truth_path = tmp_path / "truth.json"
    truth_path.write_text(
        json.dumps({"schema_version": 1, "present": {"chair": [[0, 3], [8, 9]]}, "absent": ["dog"]}),
        encoding="utf-8",
    )
    truth = parse_scene_truth(truth_path)
    assert truth.present["chair"] == ((0, 3), (8, 9))
    assert truth.absent == ("dog",)

    for value, message in (
        ({"schema_version": 1, "present": {"chair": [[-1, 0]]}}, "negative"),
        ({"schema_version": 1, "present": {"chair": [[2, 1]]}}, "start"),
        ({"schema_version": 1, "present": {"chair": [[0, 1]]}, "absent": ["chair"]}, "both"),
        ({"schema_version": True}, "schema_version"),
    ):
        truth_path.write_text(json.dumps(value), encoding="utf-8")
        with pytest.raises(ValueError, match=message):
            parse_scene_truth(truth_path)


def test_replay_alignment_and_aggregation_metrics(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    _write_comparison(reference)
    replay = load_replay_comparison(reference, "task")
    metrics = aggregate_replay(replay.streams["v2-S FP"])
    assert metrics.frames_evaluated == 3
    assert metrics.total_detections == 4
    assert metrics.by_class["chair"].frames_with_detection == 1
    assert metrics.by_class["chair"].total_detections == 2
    assert metrics.median_confidence == pytest.approx(0.6)
    assert metrics.by_class["chair"].max_confidence == pytest.approx(0.9)
    assert metrics.unique_detected_classes == 3


@pytest.mark.parametrize(
    ("changed", "message"),
    [
        (lambda records: records.pop(), "frames"),
        (lambda records: records.__setitem__(1, {**records[1], "frame_index": 9}), "frame index"),
        (lambda records: records.__setitem__(1, {**records[1], "timestamp_sec": 9.0}), "timestamp"),
    ],
)
def test_replay_alignment_rejects_required_mismatches(tmp_path: Path, changed, message: str) -> None:
    reference = tmp_path / "reference"
    streams = {spec.label: _frames() for spec in MODEL_SPECS}
    changed(streams["v2-M INT8"])
    _write_comparison(reference, streams=streams)
    with pytest.raises(ValueError, match=message):
        load_replay_comparison(reference, "task")


def test_replay_rejects_missing_or_tampered_required_artifacts(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    _write_comparison(reference)
    comparison_video = reference / "video" / "comparison" / "task" / "comparison.mp4"
    comparison_video.unlink()
    with pytest.raises(ValueError, match="missing comparison video"):
        load_replay_comparison(reference, "task")

    _write_comparison(reference)
    comparison_video.write_bytes(b"tampered")
    with pytest.raises(ValueError, match="SHA-256 does not match"):
        load_replay_comparison(reference, "task")


def test_truth_metrics_count_coverage_gaps_known_absent_only(tmp_path: Path) -> None:
    truth_path = tmp_path / "truth.json"
    truth_path.write_text(
        json.dumps({"schema_version": 1, "present": {"chair": [[0, 4]]}, "absent": ["dog"]}),
        encoding="utf-8",
    )
    frames = (
        ReplayFrame(0, 0.0, (ReplayDetection("chair", 0.9), ReplayDetection("cat", 0.7))),
        ReplayFrame(1, 0.1, (ReplayDetection("dog", 0.3), ReplayDetection("dog", 0.8))),
        ReplayFrame(2, 0.2, (ReplayDetection("chair", 0.7),)),
        ReplayFrame(3, 0.3, ()),
        ReplayFrame(4, 0.4, ()),
    )
    truth = parse_scene_truth(truth_path)
    present = present_truth_metrics(frames, truth)["chair"]
    absent = absent_truth_metrics(frames, truth)["dog"]
    assert (
        present.visible_frames,
        present.visible_frames_detected,
        present.first_detection_delay,
        present.detection_gaps,
    ) == (5, 2, 0, 2)
    assert (absent.frames_falsely_containing, absent.total_false_detections, absent.max_false_confidence) == (1, 2, 0.8)
    assert "cat" not in absent_truth_metrics(frames, truth)


def test_trial_resolution_is_order_independent_and_requires_canonical_set(tmp_path: Path) -> None:
    trials = _write_all_trials(tmp_path)
    resolved = resolve_trial_metrics(tuple(reversed(trials)))
    assert set(resolved) == {spec.label for spec in MODEL_SPECS}
    assert resolved["v2-S FP"].values["Inference p50"] == "8.00 ms"

    with pytest.raises(ValueError, match="exactly four"):
        resolve_trial_metrics(trials[:3])
    duplicate = tmp_path / "duplicate"
    _write_trial(duplicate, "v2s", "fp")
    with pytest.raises(ValueError, match="duplicate"):
        resolve_trial_metrics([*trials[:3], duplicate])

    sparse = tmp_path / "sparse"
    _write_trial(sparse, "v2s", "fp", telemetry=False)
    sparse_metrics = resolve_trial_metrics([sparse, *trials[1:]])["v2-S FP"].values
    assert sparse_metrics["Mean consumer FPS"] == "-"


def test_html_uses_existing_report_style_and_conditional_sections(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    _write_comparison(reference, "task")
    trials = _write_all_trials(tmp_path / "trials")
    summary = write_comparison_report(reference, trial_dirs=trials)
    output = summary.output_path.read_text(encoding="utf-8")
    assert "Omniseer Detector Configuration Comparison" in output
    assert ":root { color-scheme: light" in output
    assert "Closed-loop Robot Trials" in output
    assert "Controlled Perception Replay" in output
    assert "../video/comparison/task/comparison.mp4" in output
    assert str(reference) not in output
    assert "Manual scene-truth visibility" not in output
    assert "Exploratory COCO-80" not in output
    assert _sha256(reference / "video" / "source.ts") in output
    assert "hash-0" in output
    with pytest.raises(FileExistsError):
        write_comparison_report(reference, trial_dirs=trials)

    _write_comparison(reference, "coco80")
    truth_path = reference / "scene_truth.json"
    truth_path.write_text(
        json.dumps({"schema_version": 1, "present": {"chair": [[0, 2]]}, "absent": ["dog"]}),
        encoding="utf-8",
    )
    output = write_comparison_report(
        reference,
        trial_dirs=trials,
        truth_path=truth_path,
        overwrite=True,
    ).output_path.read_text(encoding="utf-8")
    assert "Manual scene-truth visibility" in output
    assert "Exploratory COCO-80" in output


def test_scene_truth_rejects_classes_outside_selected_vocabulary(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    _write_comparison(reference)
    trials = _write_all_trials(tmp_path / "trials")
    truth_path = reference / "scene_truth.json"
    truth_path.write_text(
        json.dumps({"schema_version": 1, "present": {"not-a-class": [[0, 2]]}}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="absent from the selected comparison vocabulary"):
        write_comparison_report(reference, trial_dirs=trials, truth_path=truth_path)
