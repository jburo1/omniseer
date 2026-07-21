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
from omniseer_experiments.run_report import report_run_main, write_run_report

STARTED_AT = datetime(2026, 7, 19, 12, 0, 0, tzinfo=timezone.utc)
ENDED_AT = datetime(2026, 7, 19, 12, 1, 5, tzinfo=timezone.utc)


def _config(out_dir: Path) -> RunBundleConfig:
    return RunBundleConfig(
        run_id="demo_001",
        out_dir=out_dir,
        classes=("chair", "backpack"),
        notes="report fixture",
        ros_distro="kilted",
        git_sha="abc123",
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


def _write_completed_bundle(run_dir: Path) -> None:
    writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
    writer.write_detection_record(_detection_record())
    writer.write_perf_record(_perf_record())
    writer.write_system_record(_system_record())
    writer.finalize(ended_at=ENDED_AT)


def _write_evidence(run_dir: Path) -> None:
    frames_dir = run_dir / "evidence" / "frames"
    annotated_dir = run_dir / "evidence" / "annotated"
    frames_dir.mkdir(parents=True, exist_ok=True)
    annotated_dir.mkdir(parents=True, exist_ok=True)
    (frames_dir / "frame_1.jpg").write_bytes(b"clean-jpeg-fixture")
    (annotated_dir / "frame_1.jpg").write_bytes(b"annotated-jpeg-fixture")
    record = {
        "schema_version": 1,
        "artifact_type": "sampled_frame",
        "image_path": "evidence/frames/frame_1.jpg",
        "jpeg_quality": 85,
        "capture_reason": "periodic",
        "frame_id": 1,
        "sequence": 10,
        "capture_ts_real_ns": 1000,
        "model_input": {"width": 64, "height": 64},
        "source_image": {"width": 128, "height": 108},
        "remap": {"scale": 0.5, "pad_x": 0, "pad_y": 5, "resized_w": 64, "resized_h": 54},
        "detections": [
            {
                "class_id": 0,
                "class_name": "chair",
                "score": 0.91,
                "bbox": {"x1": 20.0, "y1": 20.0, "x2": 80.0, "y2": 80.0},
            }
        ],
    }
    (run_dir / "evidence" / "evidence.jsonl").write_text(json.dumps(record) + "\n", encoding="utf-8")


class RunReportTests(unittest.TestCase):
    def test_writes_static_html_report_with_evidence_gallery(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_evidence(run_dir)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertEqual(summary.evidence_items, 1)
            self.assertEqual(summary.issues, ())
            self.assertIn("Omniseer Run Report: demo_001", output)
            self.assertIn("<h2>Run Summary</h2>", output)
            self.assertIn("<h2>Detections</h2>", output)
            self.assertIn("chair", output)
            self.assertIn("../evidence/annotated/frame_1.jpg", output)
            self.assertIn("../evidence/frames/frame_1.jpg", output)

    def test_existing_report_requires_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)

            first_summary = write_run_report(run_dir)

            with self.assertRaises(FileExistsError):
                write_run_report(run_dir)
            overwrite_summary = write_run_report(run_dir, overwrite=True)
            self.assertEqual(first_summary.output_path, overwrite_summary.output_path)

    def test_report_cli_outputs_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            stream = io.StringIO()

            with contextlib.redirect_stdout(stream):
                report_run_main([str(run_dir)])

            self.assertIn(f"Report: {run_dir / 'report' / 'index.html'}", stream.getvalue())
            self.assertIn("Evidence items: 0", stream.getvalue())
