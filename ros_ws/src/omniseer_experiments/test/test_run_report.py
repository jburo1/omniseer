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

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - dependency availability is environment-specific.
    cv2 = None
    np = None

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
        launch_command="run real --profile operator bringup",
        launch_profile="operator",
        launch_mode="bringup",
        launch_args=("start_gateway:=true", "camera_device:=/dev/video11"),
        container_image_ref="ghcr.io/acme/omniseer:robot-v2",
        container_image_digest="ghcr.io/acme/omniseer@sha256:0123456789abcdef",
        experiment_config="runtime-container-full",
        experiment_parameters={"profile": "operator", "stage": "full"},
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
        thermal={"available": True, "soc_temp_c": 61.4, "throttled": False, "zones": []},
        network={
            "available": True,
            "connected": True,
            "interface": "wlan0",
            "wifi_signal_dbm": -41,
            "link_quality_percent": 90,
        },
        onboard_battery={
            "available": False,
            "source": "",
            "present": False,
            "voltage": None,
            "percentage": None,
            "charging": None,
        },
        lipo_battery={
            "available": True,
            "source": "/battery",
            "present": True,
            "voltage": 8.34,
            "percentage": 0.0,
            "charging": False,
        },
    )


def _later_system_record() -> dict:
    return make_system_record(
        recv_ts_ns=400,
        cpu_percent=42.0,
        memory_used_mb=900.0,
        memory_available_mb=7100.0,
        soc_temp_c=64.2,
        thermal={
            "available": True,
            "soc_temp_c": 64.2,
            "throttled": True,
            "zones": [
                {"name": "thermal_zone0", "temp_c": 64.2, "type": "soc-thermal"},
                {"name": "thermal_zone1", "temp_c": 66.0, "type": "bigcore0-thermal"},
            ],
        },
        network={
            "available": True,
            "connected": False,
            "interface": "wlan1",
            "wifi_signal_dbm": -55,
            "link_quality_percent": 70,
        },
        onboard_battery={
            "available": True,
            "source": "BAT0",
            "present": True,
            "voltage": 4.8,
            "percentage": 88.0,
            "charging": True,
        },
        lipo_battery={
            "available": True,
            "source": "/battery",
            "present": True,
            "voltage": 8.10,
            "percentage": 48.0,
            "charging": False,
        },
    )


def _write_completed_bundle(run_dir: Path) -> None:
    writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
    writer.write_detection_record(_detection_record())
    writer.write_perf_record(_perf_record())
    writer.write_system_record(_system_record())
    writer.finalize(ended_at=ENDED_AT)


def _write_evidence(run_dir: Path, *, annotated: bool = True, valid_jpeg: bool = False) -> None:
    frames_dir = run_dir / "evidence" / "frames"
    annotated_dir = run_dir / "evidence" / "annotated"
    frames_dir.mkdir(parents=True, exist_ok=True)
    if valid_jpeg:
        assert cv2 is not None
        assert np is not None
        image = np.zeros((64, 64, 3), dtype=np.uint8)
        assert cv2.imwrite(str(frames_dir / "frame_1.jpg"), image)
    else:
        (frames_dir / "frame_1.jpg").write_bytes(b"clean-jpeg-fixture")
    if annotated:
        annotated_dir.mkdir(parents=True, exist_ok=True)
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


def _write_pipeline_telemetry(run_dir: Path) -> None:
    records = [
        {
            "schema_version": 3,
            "source": "producer",
            "frame_id": 1,
            "tick_id": 1,
            "sequence": 10,
            "event_ts_real_ns": 1000,
            "source_age_dequeue_ns": 2_000_000,
            "source_age_publish_ready_ns": 3_000_000,
            "producer_status": "produced",
            "capture_status": "ok",
            "preprocess_status": "ok",
            "capture_errno": 0,
            "stage_mask": 31,
            "dur_ns": {
                "dequeue": 100_000,
                "acquire_write": 20_000,
                "preprocess": 1_100_000,
                "publish_ready": 30_000,
                "requeue": 80_000,
                "total": 1_330_000,
            },
        },
        {
            "schema_version": 3,
            "source": "consumer",
            "frame_id": 1,
            "tick_id": 1,
            "sequence": 10,
            "event_ts_real_ns": 1000,
            "consumer_start_ts_real_ns": 2000,
            "consumer_end_ts_real_ns": 3000,
            "source_age_start_ns": 4_000_000,
            "source_age_end_ns": 12_000_000,
            "consumer_status": "consumed",
            "infer_status": "ok",
            "postprocess_status": "ok",
            "infer_errno": 0,
            "stage_mask": 31,
            "dur_ns": {
                "acquire_read": 10_000,
                "infer": 8_000_000,
                "postprocess": 200_000,
                "publish": 300_000,
                "release": 10_000,
                "total": 8_520_000,
            },
        },
    ]
    text = "\n".join(json.dumps(record) for record in records) + "\n"
    (run_dir / "pipeline_telemetry.jsonl").write_text(text, encoding="utf-8")


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
            self.assertIn("<h2>Evidence Summary</h2>", output)
            self.assertIn("<h2>Configuration</h2>", output)
            self.assertIn("run real --profile operator bringup", output)
            self.assertIn("start_gateway:=true, camera_device:=/dev/video11", output)
            self.assertIn("Runtime image ref", output)
            self.assertIn("Runtime image digest", output)
            self.assertIn("ghcr.io/acme/omniseer:robot-v2", output)
            self.assertIn("ghcr.io/acme/omniseer@sha256:0123456789abcdef", output)
            self.assertIn("runtime-container-full", output)
            self.assertIn("profile=operator, stage=full", output)
            self.assertIn("<h2>Errors And Drops</h2>", output)
            self.assertIn("First sample", output)
            self.assertIn("Latest sample", output)
            self.assertIn("1970-01-01T00:00:00", output)
            self.assertIn("Latest sample offset", output)
            self.assertIn("Network Summary", output)
            self.assertIn("wlan0", output)
            self.assertIn("Battery Summary", output)
            self.assertIn("8.34", output)
            self.assertIn("<h2>Detections</h2>", output)
            self.assertIn("Messages with detections", output)
            self.assertIn("Configured classes not observed", output)
            self.assertIn("chair", output)
            self.assertIn("../evidence/annotated/frame_1.jpg", output)
            self.assertIn("../evidence/frames/frame_1.jpg", output)

    def test_system_platform_tables_summarize_entire_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.write_system_record(_system_record())
            writer.write_system_record(_later_system_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("Thermal Summary", output)
            self.assertIn("SoC temperature C", output)
            self.assertIn("soc-thermal (thermal_zone0)", output)
            self.assertIn("bigcore0-thermal (thermal_zone1)", output)
            self.assertIn("Network Summary", output)
            self.assertIn("Connected samples", output)
            self.assertIn("false=1, true=1", output)
            self.assertIn("wlan0, wlan1", output)
            self.assertIn("WiFi signal dBm", output)
            self.assertIn("-55.00", output)
            self.assertIn("Battery Summary", output)
            self.assertIn("LiPo", output)
            self.assertIn("Onboard", output)
            self.assertIn("4.80", output)
            self.assertIn("88.00", output)
            self.assertNotIn("<td>LiPo</td><td>Percentage</td>", output)
            self.assertIn("<td>Onboard</td><td>Percentage</td>", output)
            self.assertNotIn("Thermal Snapshot", output)
            self.assertNotIn("Network Snapshot", output)
            self.assertNotIn("Battery Snapshot", output)

    @unittest.skipIf(cv2 is None or np is None, "OpenCV and NumPy are required for auto-annotation checks")
    def test_report_generates_missing_annotations_before_rendering(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_evidence(run_dir, annotated=False, valid_jpeg=True)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertEqual(summary.issues, ())
            self.assertTrue((run_dir / "evidence" / "annotated" / "frame_1.jpg").is_file())
            self.assertIn("../evidence/annotated/frame_1.jpg", output)

    def test_writes_native_pipeline_telemetry_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_pipeline_telemetry(run_dir)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("<h2>Pipeline Telemetry</h2>", output)
            self.assertIn("Status Distribution", output)
            self.assertIn("Producer Stage Timings", output)
            self.assertIn("Consumer Stage Timings", output)
            self.assertIn("Source Age", output)
            self.assertIn("producer status", output.lower())
            self.assertIn("produced=1", output)
            self.assertIn("source age at consumer end", output)
            self.assertIn("<td>infer</td>", output)

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
