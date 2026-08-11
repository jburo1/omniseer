import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from dataclasses import replace
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


def _config(
    out_dir: Path,
    *,
    launch_args: tuple[str, ...] = ("start_gateway:=true", "camera_device:=/dev/video11"),
    detector_model_path: str = "",
    clip_model_path: str = "",
    clip_vocab_path: str = "",
    classes_path: str = "",
    experiment_parameters: dict[str, object] | None = None,
) -> RunBundleConfig:
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
        launch_args=launch_args,
        detector_model_path=detector_model_path,
        clip_model_path=clip_model_path,
        clip_vocab_path=clip_vocab_path,
        classes_path=classes_path,
        container_image_ref="ghcr.io/acme/omniseer:robot-v2",
        container_image_digest="ghcr.io/acme/omniseer@sha256:0123456789abcdef",
        experiment_config="runtime-container-full",
        experiment_parameters=experiment_parameters or {"profile": "operator", "stage": "full"},
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


def _later_detection_record() -> dict:
    return make_detection_record(
        recv_ts_ns=1_000_000_100,
        header_stamp={"sec": 2, "nanosec": 2},
        frame_id="camera_frame",
        detections=[
            {
                "class_id": 0,
                "class_name": "chair",
                "score": 0.7,
                "bbox": {"center_x": 12.0, "center_y": 22.0, "size_x": 32.0, "size_y": 42.0},
            }
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


def _later_perf_record() -> dict:
    return make_perf_record(
        recv_ts_ns=1_000_000_200,
        header_stamp={"sec": 4, "nanosec": 4},
        frame_id="camera_frame",
        producer_fps=21.0,
        consumer_fps=18.0,
        last_preprocess_ms=1.2,
        last_infer_ms=9.0,
        last_postprocess_ms=2.2,
        last_publish_ms=0.6,
        last_producer_total_ms=3.4,
        last_consumer_total_ms=10.0,
        produced_count=20,
        consumed_count=18,
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
    writer.write_detection_record(_later_detection_record())
    writer.write_perf_record(_perf_record())
    writer.write_perf_record(_later_perf_record())
    writer.write_system_record(_system_record())
    writer.write_system_record(_later_system_record())
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
        {
            "schema_version": 3,
            "source": "producer",
            "frame_id": 2,
            "tick_id": 2,
            "sequence": 11,
            "event_ts_real_ns": 1_000_001_000,
            "source_age_dequeue_ns": 2_500_000,
            "source_age_publish_ready_ns": 3_500_000,
            "producer_status": "produced",
            "capture_status": "ok",
            "preprocess_status": "ok",
            "capture_errno": 0,
            "stage_mask": 31,
            "dur_ns": {
                "dequeue": 120_000,
                "acquire_write": 30_000,
                "preprocess": 1_200_000,
                "publish_ready": 40_000,
                "requeue": 90_000,
                "total": 1_480_000,
            },
        },
        {
            "schema_version": 3,
            "source": "consumer",
            "frame_id": 2,
            "tick_id": 2,
            "sequence": 11,
            "event_ts_real_ns": 1_000_001_000,
            "consumer_start_ts_real_ns": 1_000_002_000,
            "consumer_end_ts_real_ns": 1_000_003_000,
            "source_age_start_ns": 5_000_000,
            "source_age_end_ns": 13_000_000,
            "consumer_status": "consumed",
            "infer_status": "ok",
            "postprocess_status": "ok",
            "infer_errno": 0,
            "stage_mask": 31,
            "dur_ns": {
                "acquire_read": 20_000,
                "infer": 9_000_000,
                "postprocess": 300_000,
                "publish": 400_000,
                "release": 20_000,
                "total": 9_740_000,
            },
        },
    ]
    text = "\n".join(json.dumps(record) for record in records) + "\n"
    (run_dir / "pipeline_telemetry.jsonl").write_text(text, encoding="utf-8")


def _write_autonomy(run_dir: Path) -> None:
    records = [
        {
            "schema_version": 1,
            "time_sec": 0.0,
            "state": "scan",
            "event": "started",
            "target_loss_count": 0,
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": 0.2,
        },
        {
            "schema_version": 1,
            "time_sec": 0.4,
            "state": "scan",
            "event": "first_detection",
            "target_loss_count": 0,
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": 0.1,
            "target": {"class_name": "backpack", "confidence": 0.73},
        },
        {
            "schema_version": 1,
            "time_sec": 0.8,
            "state": "scan",
            "event": "target_lost",
            "reason": "no_target",
            "target_loss_count": 1,
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": 0.0,
        },
        {
            "schema_version": 1,
            "time_sec": 1.0,
            "state": "center",
            "event": "centering_started",
            "target_loss_count": 1,
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": -0.1,
            "target": {"class_name": "backpack", "confidence": 0.80},
        },
        {
            "schema_version": 1,
            "time_sec": 1.2,
            "state": "center",
            "event": "centered_first_frame",
            "normalized_error": 0.03,
            "target_loss_count": 1,
            "linear_x_m_s": 0.05,
            "angular_z_rad_s": 0.0,
            "target": {"class_name": "backpack", "confidence": 0.82},
        },
        {
            "schema_version": 1,
            "time_sec": 1.6,
            "state": "success",
            "event": "succeeded",
            "reason": "centered",
            "normalized_error": 0.02,
            "target_loss_count": 1,
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": 0.0,
            "target": {"class_name": "backpack", "confidence": 0.87},
        },
    ]
    (run_dir / "autonomy.jsonl").write_text(
        "\n".join(json.dumps(record) for record in records) + "\n",
        encoding="utf-8",
    )


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
            self.assertIn('<nav class="toc"', output)
            self.assertIn('<span class="toc-chip"><strong>State</strong>complete</span>', output)
            self.assertIn('<span class="toc-chip"><strong>Issues</strong>0</span>', output)
            self.assertIn('<span class="toc-chip"><strong>Evidence</strong>1</span>', output)
            self.assertIn('<span class="toc-chip"><strong>Duration</strong>65.0s</span>', output)
            self.assertIn('<a href="#run-summary">Run Summary</a>', output)
            self.assertIn('<details id="run-summary" open>', output)
            self.assertIn('<details id="configuration">', output)
            self.assertIn('<details id="evidence" open>', output)
            self.assertIn('<details id="issues">', output)
            self.assertNotIn('<details id="issues" open>', output)
            self.assertIn("<h2>Run Summary</h2>", output)
            self.assertIn("<h2>Experiment Outcome</h2>", output)
            self.assertIn("<h2>Key Metrics</h2>", output)
            self.assertIn("<h2>Data Coverage</h2>", output)
            self.assertNotIn("<h2>Health</h2>", output)
            self.assertIn("<th>Experiment config</th><td>runtime-container-full</td>", output)
            self.assertIn("<h2>Run Artifacts</h2>", output)
            self.assertIn('<a href="../manifest.yaml">manifest.yaml</a>', output)
            self.assertIn('<a href="../summary.json">summary.json</a>', output)
            self.assertIn('<a href="../detections.jsonl">detections.jsonl</a>', output)
            self.assertIn('<a href="../perf.jsonl">perf.jsonl</a>', output)
            self.assertIn('<a href="../system.jsonl">system.jsonl</a>', output)
            self.assertIn('<a href="../evidence/evidence.jsonl">evidence/evidence.jsonl</a>', output)
            self.assertNotIn('href="../pipeline_telemetry.jsonl"', output)
            self.assertNotIn('href="../autonomy.jsonl"', output)
            self.assertNotIn('href="../logs/bringup.log"', output)
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
            self.assertIn('class="chart"', output)
            self.assertIn("<svg", output)
            self.assertIn("Detection Count Over Time", output)
            self.assertIn("Top Detection Score Over Time", output)
            self.assertNotIn("Detection Activity Over Time", output)
            self.assertIn("Vision FPS Over Time", output)
            self.assertIn("Vision Latency Over Time", output)
            self.assertIn('class="metric-bars"', output)
            self.assertIn("Frame rate p95 visual summary", output)
            self.assertIn("Latency p95 visual summary", output)
            self.assertIn('class="metric-bar-fill"', output)
            self.assertIn("System CPU Over Time", output)
            self.assertIn("System Memory Over Time", output)
            self.assertIn("System Temperature Over Time", output)
            self.assertIn("WiFi Signal Over Time", output)
            self.assertIn("Network Link Quality Over Time", output)
            self.assertIn("LiPo Voltage Over Time", output)
            self.assertIn("seconds since experiment start", output)
            self.assertIn("consumer_fps", output)
            self.assertIn("last_infer_ms", output)
            self.assertIn("../evidence/annotated/frame_1.jpg", output)
            self.assertIn("../evidence/frames/frame_1.jpg", output)
            self.assertIn("Open clean frame", output)

    def test_collapsible_sections_open_issues_when_present(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(
                _config(run_dir, launch_args=("start_autonomy:=true", "autonomy_target_class:=chair")),
                started_at=STARTED_AT,
            )
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn('<details id="issues" open>', output)

    def test_configuration_expands_known_from_config_launch_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(
                _config(
                    run_dir,
                    launch_args=(
                        "detector_model_path:=__from_config__",
                        "clip_model_path:=__from_config__",
                        "clip_vocab_path:=__from_config__",
                        "classes_path:=__from_config__",
                    ),
                    detector_model_path="/models/detector.rknn",
                    clip_model_path="/models/clip.rknn",
                    clip_vocab_path="/models/clip_vocab.bpe",
                    classes_path="/models/classes.txt",
                ),
                started_at=STARTED_AT,
            )
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("detector_model_path:=/models/detector.rknn (from config)", output)
            self.assertIn("clip_model_path:=/models/clip.rknn (from config)", output)
            self.assertIn("clip_vocab_path:=/models/clip_vocab.bpe (from config)", output)
            self.assertIn("classes_path:=/models/classes.txt (from config)", output)
            self.assertNotIn("__from_config__", output)

    def test_run_artifact_links_include_optional_existing_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_pipeline_telemetry(run_dir)
            _write_autonomy(run_dir)
            logs_dir = run_dir / "logs"
            logs_dir.mkdir()
            (logs_dir / "bringup.log").write_text("bringup fixture\n", encoding="utf-8")

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn('<a href="../pipeline_telemetry.jsonl">pipeline_telemetry.jsonl</a>', output)
            self.assertIn('<a href="../autonomy.jsonl">autonomy.jsonl</a>', output)
            self.assertIn('<a href="../logs/bringup.log">logs/bringup.log</a>', output)

    def test_writes_autonomy_summary_when_autonomy_jsonl_exists(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_autonomy(run_dir)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertEqual(summary.issues, ())
            self.assertIn("<h2>Autonomy</h2>", output)
            self.assertIn("<th>Terminal state</th><td>success</td>", output)
            self.assertIn("<th>Failure reason</th><td>-</td>", output)
            self.assertIn("<th>Time to first detection</th><td>0.4s</td>", output)
            self.assertIn("<th>Time to centered</th><td>1.2s</td>", output)
            self.assertIn("<th>Time to framed/success</th><td>1.6s</td>", output)
            self.assertNotIn("Time to centered/framed", output)
            self.assertIn("<th>Final error</th><td>0.02</td>", output)
            self.assertIn("<th>Final confidence</th><td>0.87</td>", output)
            self.assertIn("<th>Target-loss episodes</th><td>1</td>", output)
            self.assertIn("<th>Missing-target updates</th><td>1</td>", output)
            self.assertIn("Autonomy Error Over Time", output)
            self.assertIn("Target Confidence Over Time", output)
            self.assertIn("Target Loss Count Over Time", output)
            self.assertIn("Linear Command Over Time", output)
            self.assertIn("Angular Command Over Time", output)
            self.assertIn("Command Summary", output)
            self.assertIn("State Timeline", output)
            self.assertIn("Target Loss Episodes", output)
            self.assertIn("Event Timeline", output)
            self.assertIn("target_lost", output)
            self.assertIn('class="outcome-banner outcome-success"', output)
            self.assertIn("<th>Target class</th><td>backpack</td>", output)
            self.assertIn("<th>Final target area</th><td>-</td>", output)
            self.assertIn("Consumer FPS p95", output)

    def test_report_uses_one_time_extent_for_duration_events_and_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_autonomy(run_dir)
            autonomy_path = run_dir / "autonomy.jsonl"
            autonomy_records = [json.loads(line) for line in autonomy_path.read_text(encoding="utf-8").splitlines()]
            autonomy_records[-1]["time_sec"] = 70.0
            autonomy_path.write_text(
                "\n".join(json.dumps(record) for record in autonomy_records) + "\n", encoding="utf-8"
            )
            _write_evidence(run_dir)
            evidence_path = run_dir / "evidence" / "evidence.jsonl"
            evidence_record = json.loads(evidence_path.read_text(encoding="utf-8"))
            evidence_record["capture_ts_real_ns"] = int(STARTED_AT.timestamp() * 1_000_000_000) + 2_000_000_000
            evidence_path.write_text(json.dumps(evidence_record) + "\n", encoding="utf-8")

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<strong>Duration</strong>70.0s", output)
            self.assertIn("<th>Run duration</th><td>70.0s</td>", output)
            self.assertIn("<th>Time to framed/success</th><td>70.0s</td>", output)
            self.assertIn("t+2.0s", output)
            self.assertIn("Frames superseded before inference", output)
            self.assertIn("Supersession ratio</th><td>10.0%", output)
            self.assertIn("Normal latest-frame behavior; not an error", output)

    def test_report_marks_missing_runtime_provenance_unavailable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            config = replace(_config(run_dir), container_image_ref="", container_image_digest="")
            writer = RunBundleWriter(config, started_at=STARTED_AT)
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<th>Runtime image ref</th><td>Unavailable (missing)</td>", output)
            self.assertIn("<th>Runtime image digest</th><td>Unavailable (missing)</td>", output)
            self.assertIn("<th>Runtime image identity</th><td>Unavailable</td>", output)

    def test_report_uses_runtime_image_ref_when_digest_is_unavailable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            config = replace(_config(run_dir), container_image_digest="")
            writer = RunBundleWriter(config, started_at=STARTED_AT)
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<th>Runtime image identity</th><td>ghcr.io/acme/omniseer:robot-v2</td>", output)

    def test_report_collapses_target_loss_updates_into_episodes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            records = [
                {"time_sec": 0.5, "state": "center", "event": "target_locked"},
                {"time_sec": 1.0, "state": "center", "event": "target_lost", "reason": "no_target"},
                {"time_sec": 1.5, "state": "center", "event": "target_lost", "reason": "no_target"},
                {"time_sec": 2.0, "state": "center", "event": "target_lost", "reason": "no_target"},
                {"time_sec": 5.0, "state": "frame", "event": "framing_started"},
                {"time_sec": 7.0, "state": "frame", "event": "target_lost", "reason": "stale_target"},
                {"time_sec": 8.0, "state": "frame", "event": "target_lost", "reason": "stale_target"},
                {"time_sec": 9.0, "state": "failed", "event": "failed"},
            ]
            (run_dir / "autonomy.jsonl").write_text(
                "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
            )

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<th>Target-loss episodes</th><td>2</td>", output)
            self.assertIn("<th>Missing-target updates</th><td>5</td>", output)
            self.assertIn("<th>Target-loss duration max</th><td>4.0s</td>", output)
            self.assertIn("<th>Target-loss duration median</th><td>3.0s</td>", output)
            self.assertIn("<td>1.0s</td><td>5.0s</td><td>4.0s</td>", output)
            self.assertIn("<td>7.0s</td><td>Run ended</td><td>2.0s</td>", output)

    def test_report_adds_phase_timings_only_when_both_events_exist(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            records = [
                {"time_sec": 1.0, "state": "lock", "event": "target_locked"},
                {"time_sec": 2.0, "state": "center", "event": "centered_first_frame"},
                {"time_sec": 3.0, "state": "frame", "event": "framing_started"},
                {"time_sec": 5.0, "state": "success", "event": "succeeded"},
            ]
            (run_dir / "autonomy.jsonl").write_text(
                "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
            )

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<th>Lock → success</th><td>4.0s</td>", output)
            self.assertIn("<th>Centered → success</th><td>3.0s</td>", output)
            self.assertIn("<th>Framing start → success</th><td>2.0s</td>", output)

            (run_dir / "autonomy.jsonl").write_text(
                json.dumps({"time_sec": 5.0, "state": "success", "event": "succeeded"}) + "\n",
                encoding="utf-8",
            )
            output = write_run_report(run_dir, overwrite=True).output_path.read_text(encoding="utf-8")
            self.assertNotIn("Lock → success", output)
            self.assertNotIn("Centered → success", output)
            self.assertNotIn("Framing start → success", output)

    def test_report_shows_hashed_model_artifact_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            model = root / "detector.rknn"
            model.write_bytes(b"model artifact")
            run_dir = root / "demo_001"
            writer = RunBundleWriter(replace(_config(run_dir), detector_model_path=str(model)), started_at=STARTED_AT)
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn(
                f"<th>Detector model SHA256</th><td>{hashlib.sha256(model.read_bytes()).hexdigest()}</td>", output
            )

    def test_report_flags_missing_autonomy_jsonl_for_autonomy_launch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(
                _config(run_dir, launch_args=("start_autonomy:=true", "autonomy_target_class:=chair")),
                started_at=STARTED_AT,
            )
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertEqual(
                summary.issues,
                ("missing_autonomy_jsonl: autonomy.jsonl is missing despite start_autonomy:=true",),
            )
            self.assertIn("Run state: incomplete", output)
            self.assertIn("missing_autonomy_jsonl", output)

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

    def test_system_summary_excludes_unavailable_samples(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.write_system_record(
                make_system_record(
                    recv_ts_ns=300,
                    cpu_percent=None,
                    memory_used_mb=None,
                    memory_available_mb=None,
                    soc_temp_c=None,
                    thermal={"available": False, "soc_temp_c": None, "throttled": None, "zones": []},
                    onboard_battery={
                        "available": False,
                        "source": "",
                        "present": False,
                        "voltage": None,
                        "percentage": None,
                        "charging": None,
                    },
                )
            )
            writer.write_system_record(_later_system_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("<td>cpu_percent</td><td>1</td><td>42.00</td><td>42.00</td><td>42.00</td>", output)
            self.assertIn(
                "<td>memory_used_mb</td><td>1</td><td>900.00</td><td>900.00</td><td>900.00</td>",
                output,
            )
            self.assertIn("<th>Available samples</th><td>false=1, true=1</td>", output)

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
            self.assertIn("produced=2", output)
            self.assertIn("source age at consumer end", output)
            self.assertIn("<td>infer</td>", output)
            self.assertIn("Pipeline Source Age Over Time", output)
            self.assertIn("source age at producer dequeue", output)
            self.assertIn('class="chart"', output)
            self.assertIn('class="metric-bars"', output)
            self.assertIn("Stage timing p95 visual summary", output)
            self.assertIn("Source age p95 visual summary", output)
            self.assertIn("<svg", output)

    def test_sparse_or_missing_streams_do_not_render_empty_charts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.write_system_record(_system_record())
            writer.finalize(ended_at=ENDED_AT)

            summary = write_run_report(run_dir)

            output = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("<h2>Detections</h2>", output)
            self.assertIn("<h2>Performance</h2>", output)
            self.assertIn("<h2>System</h2>", output)
            self.assertIn("<h2>Pipeline Telemetry</h2>", output)
            self.assertIn("No native pipeline telemetry recorded.", output)
            self.assertNotIn('class="chart"', output)

    def test_report_ranks_sampled_cpu_consumers(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.write_system_record(
                make_system_record(
                    recv_ts_ns=100,
                    cpu_percent=1.0,
                    memory_used_mb=1.0,
                    memory_available_mb=1.0,
                    soc_temp_c=None,
                    process_cpu=[
                        {
                            "pid": 20,
                            "start_time_ticks": 200,
                            "name": "python3",
                            "cmdline": "python3 recorder.py",
                            "cpu_seconds_delta": 2.0,
                            "cpu_cores": 2.0,
                        },
                        {
                            "pid": 10,
                            "start_time_ticks": 100,
                            "name": "vision_node",
                            "cmdline": "",
                            "cpu_seconds_delta": 5.0,
                            "cpu_cores": 5.0,
                        },
                    ],
                )
            )
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn("<h2>CPU Consumers</h2>", output)
            self.assertIn("CPU share (sampled processes)", output)
            self.assertIn("not total machine utilization", output)
            self.assertLess(output.index("vision_node"), output.index("python3 — python3 recorder.py"))
            self.assertIn("71.4%", output)
            self.assertIn("approximately 1 Hz", output)

    def test_existing_report_requires_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)

            first_summary = write_run_report(run_dir)

            with self.assertRaises(FileExistsError):
                write_run_report(run_dir)
            overwrite_summary = write_run_report(run_dir, overwrite=True)
            self.assertEqual(first_summary.output_path, overwrite_summary.output_path)

    def test_report_includes_only_detection_overlay_video(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            video_dir = run_dir / "video"
            video_dir.mkdir()
            (video_dir / "source.ts").write_bytes(b"recorded")
            waiting = write_run_report(run_dir)
            self.assertNotIn('<details id="video"', waiting.output_path.read_text(encoding="utf-8"))

            (video_dir / "source.mp4").write_bytes(b"mp4")
            (video_dir / "overlay.mp4").write_bytes(b"mp4")
            complete = write_run_report(run_dir, overwrite=True)
            output = complete.output_path.read_text(encoding="utf-8")
            self.assertIn("video/overlay.mp4", output)
            self.assertIn('<details id="video" open>', output)
            self.assertNotIn("video/source.mp4", output)
            self.assertNotIn("Source video", output)
            self.assertIn("map video time to recorded robot time", output)

    def test_outcome_failure_is_prominent_without_configured_metric_status(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(
                _config(run_dir, experiment_parameters={"min_consumer_fps": 20}),
                started_at=STARTED_AT,
            )
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.finalize(ended_at=ENDED_AT)
            (run_dir / "autonomy.jsonl").write_text(
                json.dumps(
                    {
                        "time_sec": 0.5,
                        "state": "failed",
                        "event": "failed",
                        "reason": "target_timeout",
                        "target_class": "chair",
                        "target_loss_count": 2,
                        "bbox_area_ratio": 0.12,
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            self.assertIn('class="outcome-banner outcome-failed"', output)
            self.assertIn("Terminal state: <strong>failed</strong>", output)
            self.assertIn("<th>Failure reason</th><td>target_timeout</td>", output)
            self.assertIn("<th>Final target area</th><td>0.12</td>", output)
            self.assertIn("<th>Metric</th><th>Value</th>", output)
            self.assertNotIn("Configured status", output)
            self.assertNotIn("threshold 20.00", output)

    def test_report_cli_outputs_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            stream = io.StringIO()

            with contextlib.redirect_stdout(stream):
                report_run_main([str(run_dir)])

            self.assertIn(f"Report: {run_dir / 'report' / 'index.html'}", stream.getvalue())
            self.assertIn("Evidence items: 0", stream.getvalue())
