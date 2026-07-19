import json
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from omniseer_experiments.bundle import (
    RunBundleConfig,
    RunBundleWriter,
    SummaryAccumulator,
    default_run_id,
    make_detection_record,
    make_perf_record,
)

STARTED_AT = datetime(2026, 7, 19, 12, 0, 0, tzinfo=timezone.utc)
ENDED_AT = datetime(2026, 7, 19, 12, 1, 0, tzinfo=timezone.utc)


def _config(out_dir: Path, *, overwrite: bool = False) -> RunBundleConfig:
    return RunBundleConfig(
        run_id="demo_001",
        out_dir=out_dir,
        classes=("chair", "backpack"),
        notes="first slice",
        overwrite=overwrite,
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


def _perf_record(*, infer_ms: float, capture_fatal: int = 0) -> dict:
    return make_perf_record(
        recv_ts_ns=200,
        header_stamp={"sec": 3, "nanosec": 4},
        frame_id="camera_frame",
        producer_fps=20.0,
        consumer_fps=19.0,
        last_preprocess_ms=1.0,
        last_infer_ms=infer_ms,
        last_postprocess_ms=2.0,
        last_publish_ms=0.5,
        last_producer_total_ms=3.0,
        last_consumer_total_ms=9.0,
        produced_count=10,
        consumed_count=9,
        error_counts={
            "no_writable_buffer": 0,
            "capture_retryable": 1,
            "capture_fatal": capture_fatal,
            "preprocess": 0,
            "infer": 2,
        },
    )


class RunBundleWriterTests(unittest.TestCase):
    def test_creates_run_directory_and_start_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "runs" / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            try:
                self.assertTrue((run_dir / "evidence").is_dir())
                self.assertTrue((run_dir / "manifest.yaml").is_file())
                manifest = (run_dir / "manifest.yaml").read_text(encoding="utf-8")
                self.assertIn("schema_version: 1", manifest)
                self.assertIn('run_id: "demo_001"', manifest)
                self.assertIn('started_at: "2026-07-19T12:00:00+00:00"', manifest)
                self.assertIn("ended_at: null", manifest)
                self.assertIn('vision_params_file: "/configs/vision.yaml"', manifest)
                self.assertIn('clip_vocab_path: "/models/clip_vocab.bpe"', manifest)
                self.assertIn('classes_path: "/models/classes.txt"', manifest)
                self.assertIn('- "chair"', manifest)
                self.assertNotIn("verified_prerequisite", manifest)
            finally:
                writer.close()

    def test_writes_jsonl_and_final_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)

            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record(infer_ms=8.0))
            writer.write_perf_record(_perf_record(infer_ms=12.0, capture_fatal=1))
            summary = writer.finalize(ended_at=ENDED_AT)

            detections = _load_jsonl(run_dir / "detections.jsonl")
            perf = _load_jsonl(run_dir / "perf.jsonl")
            self.assertEqual(len(detections), 1)
            self.assertEqual(len(perf), 2)
            self.assertEqual(summary["duration_sec"], 60.0)
            self.assertEqual(summary["message_counts"], {"detections": 1, "perf": 2})
            self.assertEqual(summary["detections_by_class"], {"backpack": 1, "chair": 1})
            self.assertEqual(summary["confidence_by_class"]["chair"], {"min": 0.8, "mean": 0.8, "max": 0.8})
            self.assertEqual(summary["perf"]["producer_fps_mean"], 20.0)
            self.assertEqual(summary["perf"]["consumer_fps_mean"], 19.0)
            self.assertEqual(summary["perf"]["infer_ms_mean"], 10.0)
            self.assertEqual(summary["perf"]["infer_ms_p95"], 12.0)
            self.assertEqual(summary["errors"]["capture_retryable"], 1)
            self.assertEqual(summary["errors"]["capture_fatal"], 1)
            self.assertEqual(summary["errors"]["infer"], 2)

            persisted_summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(persisted_summary, summary)
            manifest = (run_dir / "manifest.yaml").read_text(encoding="utf-8")
            self.assertIn('ended_at: "2026-07-19T12:01:00+00:00"', manifest)

    def test_empty_run_finalizes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "empty"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)

            summary = writer.finalize(ended_at=STARTED_AT)

            self.assertEqual(summary["duration_sec"], 0.0)
            self.assertEqual(summary["message_counts"], {"detections": 0, "perf": 0})
            self.assertEqual(summary["detections_by_class"], {})
            self.assertEqual(summary["confidence_by_class"], {})
            self.assertEqual(summary["perf"]["infer_ms_mean"], 0.0)
            self.assertTrue((run_dir / "summary.json").is_file())

    def test_existing_run_requires_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "existing"
            run_dir.mkdir()

            with self.assertRaises(FileExistsError):
                RunBundleWriter(_config(run_dir))

            stale = run_dir / "detections.jsonl"
            stale.write_text('{"old":true}\n', encoding="utf-8")
            writer = RunBundleWriter(_config(run_dir, overwrite=True), started_at=STARTED_AT)
            writer.finalize(ended_at=ENDED_AT)

            self.assertEqual((run_dir / "detections.jsonl").read_text(encoding="utf-8"), "")

    def test_manifest_renders_empty_classes_inline(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "empty_classes"
            config = RunBundleConfig(run_id="empty_classes", out_dir=run_dir, git_sha="abc123", ros_distro="kilted")
            writer = RunBundleWriter(config, started_at=STARTED_AT)
            try:
                manifest = (run_dir / "manifest.yaml").read_text(encoding="utf-8")
                self.assertIn("classes: []", manifest)
            finally:
                writer.close()

    def test_default_run_id_is_timestamp_safe(self) -> None:
        self.assertEqual(default_run_id(STARTED_AT), "20260719T120000Z")


class SummaryAccumulatorTests(unittest.TestCase):
    def test_drop_accounting(self) -> None:
        summary = SummaryAccumulator("demo_001")

        summary.record_drop("detections", 2)
        summary.record_drop("perf")

        result = summary.build_summary(5.0)
        self.assertEqual(result["dropped_records"], {"detections": 2, "perf": 1})


def _load_jsonl(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]
