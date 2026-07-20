import contextlib
import io
import json
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from omniseer_experiments.bundle import (
    RunBundleConfig,
    RunBundleWriter,
    make_detection_record,
    make_perf_record,
    make_system_record,
)
from omniseer_experiments.run_inspection import (
    STATE_COMPLETE,
    STATE_IN_PROGRESS,
    STATE_INCOMPLETE,
    format_run_list,
    inspect_run,
    inspect_run_main,
    list_runs,
    list_runs_main,
)

STARTED_AT = datetime(2026, 7, 19, 12, 0, 0, tzinfo=timezone.utc)
ENDED_AT = datetime(2026, 7, 19, 12, 1, 5, tzinfo=timezone.utc)


def _config(out_dir: Path, *, run_id: str = "demo_001") -> RunBundleConfig:
    return RunBundleConfig(
        run_id=run_id,
        out_dir=out_dir,
        classes=("chair", "backpack"),
        notes="inspection fixture",
        ros_distro="kilted",
        git_sha="abc123",
        vision_params_file="/configs/vision.yaml",
        detector_model_path="/models/detector.rknn",
        clip_model_path="/models/clip.rknn",
        clip_vocab_path="/models/clip_vocab.bpe",
        classes_path="/models/classes.txt",
    )


def _detection_record() -> dict:
    return make_detection_record(
        recv_ts_ns=100,
        header_stamp={"sec": 1, "nanosec": 2},
        frame_id="camera_frame",
        detections=[
            {
                "class_id": 0,
                "class_name": "chair",
                "score": 0.8,
                "bbox": {"center_x": 10.0, "center_y": 20.0, "size_x": 30.0, "size_y": 40.0},
            },
            {
                "class_id": 1,
                "class_name": "backpack",
                "score": 0.6,
                "bbox": {"center_x": 50.0, "center_y": 60.0, "size_x": 70.0, "size_y": 80.0},
            },
        ],
    )


def _perf_record() -> dict:
    return make_perf_record(
        recv_ts_ns=200,
        header_stamp={"sec": 3, "nanosec": 4},
        frame_id="camera_frame",
        producer_fps=20.0,
        consumer_fps=19.0,
        last_preprocess_ms=1.0,
        last_infer_ms=8.0,
        last_postprocess_ms=2.0,
        last_publish_ms=0.5,
        last_producer_total_ms=3.0,
        last_consumer_total_ms=9.0,
        produced_count=10,
        consumed_count=9,
        error_counts={
            "no_writable_buffer": 0,
            "capture_retryable": 1,
            "capture_fatal": 0,
            "preprocess": 0,
            "infer": 2,
        },
    )


def _system_record() -> dict:
    return make_system_record(
        recv_ts_ns=300,
        cpu_percent=38.2,
        memory_used_mb=812.0,
        memory_available_mb=7200.0,
        soc_temp_c=61.4,
    )


def _write_completed_bundle(run_dir: Path, *, run_id: str = "demo_001", include_system: bool = False) -> None:
    writer = RunBundleWriter(_config(run_dir, run_id=run_id), started_at=STARTED_AT)
    writer.write_detection_record(_detection_record())
    writer.write_perf_record(_perf_record())
    if include_system:
        writer.write_system_record(_system_record())
    writer.finalize(ended_at=ENDED_AT)


def _write_in_progress_bundle(run_dir: Path) -> None:
    run_dir.mkdir(parents=True)
    (run_dir / "manifest.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                'run_id: "demo_001"',
                'started_at: "2026-07-19T12:00:00+00:00"',
                "ended_at: null",
                "classes:",
                '- "chair"',
                '- "backpack"',
                "topics:",
                '  detections: "/yolo/detections"',
                '  perf: "/vision/perf"',
                "",
            ]
        ),
        encoding="utf-8",
    )
    (run_dir / "detections.jsonl").write_text(json.dumps(_detection_record()) + "\n", encoding="utf-8")
    (run_dir / "perf.jsonl").write_text(json.dumps(_perf_record()) + "\n", encoding="utf-8")


class RunInspectionTests(unittest.TestCase):
    def test_inspects_completed_bundle_created_by_writer(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "runs" / "demo_001"
            _write_completed_bundle(run_dir)

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_COMPLETE)
        self.assertEqual(inspection.run_id, "demo_001")
        self.assertEqual(inspection.started_at, "2026-07-19T12:00:00+00:00")
        self.assertEqual(inspection.ended_at, "2026-07-19T12:01:05+00:00")
        self.assertEqual(inspection.duration_sec, 65.0)
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})
        self.assertEqual(inspection.configured_classes, ("chair", "backpack"))
        self.assertEqual(inspection.detections_by_class, {"backpack": 1, "chair": 1})
        self.assertEqual(inspection.perf["infer_ms_mean"], 8.0)
        self.assertEqual(inspection.errors["infer"], 2)
        self.assertEqual(inspection.issues, ())

    def test_inspect_run_human_output_is_compact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)

            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                inspect_run_main([str(run_dir)])

        output = stream.getvalue()
        self.assertIn("Run: demo_001", output)
        self.assertIn("State: complete", output)
        self.assertIn("Messages: detections=1 perf=1 system=0", output)
        self.assertIn("Configured classes: chair, backpack", output)
        self.assertIn("Observed classes: backpack=1, chair=1", output)
        self.assertNotIn("Traceback", output)

    def test_inspect_run_json_output_is_stable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)

            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                inspect_run_main([str(run_dir), "--json"])

        payload = json.loads(stream.getvalue())
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["run_id"], "demo_001")
        self.assertEqual(payload["state"], "complete")
        self.assertEqual(payload["issues"], [])
        self.assertEqual(payload["message_counts"], {"detections": 1, "perf": 1, "system": 0})
        self.assertEqual(payload["configured_classes"], ["chair", "backpack"])

    def test_missing_summary_after_finalized_manifest_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            (run_dir / "summary.json").unlink()

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_INCOMPLETE)
        self.assertIn("missing_summary", {issue.code for issue in inspection.issues})
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})
        self.assertEqual(inspection.detections_by_class, {"backpack": 1, "chair": 1})

    def test_ended_at_null_and_missing_summary_is_in_progress(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_in_progress_bundle(run_dir)

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_IN_PROGRESS)
        self.assertEqual({issue.code for issue in inspection.issues}, {"missing_summary", "run_open"})
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})

    def test_malformed_jsonl_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            with (run_dir / "detections.jsonl").open("a", encoding="utf-8") as handle:
                handle.write("{bad json\n")
            with (run_dir / "perf.jsonl").open("a", encoding="utf-8") as handle:
                handle.write("[]\n")

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_INCOMPLETE)
        self.assertIn("malformed_detections_jsonl", {issue.code for issue in inspection.issues})
        self.assertIn("invalid_perf_jsonl_record", {issue.code for issue in inspection.issues})

    def test_malformed_summary_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            (run_dir / "summary.json").write_text("{bad json\n", encoding="utf-8")

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_INCOMPLETE)
        self.assertIn("invalid_summary", {issue.code for issue in inspection.issues})
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})

    def test_missing_system_jsonl_remains_complete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            (run_dir / "system.jsonl").unlink()

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_COMPLETE)
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})
        self.assertEqual(inspection.issues, ())

    def test_valid_system_jsonl_is_counted(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir, include_system=True)

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_COMPLETE)
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 1})
        self.assertEqual(inspection.issues, ())

    def test_valid_pipeline_telemetry_jsonl_is_optional(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            (run_dir / "pipeline_telemetry.jsonl").write_text(
                '{"schema_version":3,"source":"producer","tick_id":1}\n',
                encoding="utf-8",
            )

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_COMPLETE)
        self.assertEqual(inspection.message_counts, {"detections": 1, "perf": 1, "system": 0})
        self.assertEqual(inspection.issues, ())

    def test_malformed_pipeline_telemetry_jsonl_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            with (run_dir / "pipeline_telemetry.jsonl").open("a", encoding="utf-8") as handle:
                handle.write("{bad json\n")

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_INCOMPLETE)
        self.assertIn("malformed_pipeline_telemetry_jsonl", {issue.code for issue in inspection.issues})

    def test_malformed_system_jsonl_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            with (run_dir / "system.jsonl").open("a", encoding="utf-8") as handle:
                handle.write("{bad json\n")

            inspection = inspect_run(run_dir)

        self.assertEqual(inspection.state, STATE_INCOMPLETE)
        self.assertIn("malformed_system_jsonl", {issue.code for issue in inspection.issues})

    def test_list_runs_sorts_and_formats_children(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            _write_completed_bundle(root / "b_run", run_id="b_run")
            _write_in_progress_bundle(root / "a_run")

            inspections = list_runs(root)
            output = format_run_list(inspections, root=root)

        self.assertEqual([item.path.name for item in inspections], ["a_run", "b_run"])
        self.assertIn("run_id", output)
        self.assertIn("in_progress", output)
        self.assertIn("complete", output)
        self.assertIn("run_open,missing_summary", output)

    def test_empty_root_reports_no_runs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            root.mkdir()

            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                list_runs_main(["--root", str(root)])

        self.assertIn("no runs found under", stream.getvalue())
