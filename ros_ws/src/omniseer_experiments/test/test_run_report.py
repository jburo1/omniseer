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
AUTONOMY_LAUNCH_ARGS = (
    "start_autonomy:=true",
    "autonomy_target_class:=chair",
    "autonomy_min_target_confidence:=0.75",
    "autonomy_bbox_area_min_ratio:=0.08",
    "autonomy_approach_stop_area_ratio:=0.10",
    "autonomy_bbox_area_max_ratio:=0.35",
    "autonomy_stable_framed_frames:=10",
    "autonomy_success_miss_tolerance_updates:=2",
    "autonomy_max_target_center_jump_ratio:=0.20",
)


def _config(out_dir: Path, *, launch_args: tuple[str, ...] = ()) -> RunBundleConfig:
    return RunBundleConfig(
        run_id="demo_001",
        out_dir=out_dir,
        classes=("chair", "backpack"),
        ros_distro="kilted",
        git_sha="abc123",
        launch_command="run real --profile operator bringup",
        launch_profile="operator",
        launch_mode="bringup",
        launch_args=launch_args,
        runtime_backend="robot_runtime_container",
        detector_model_path="/models/detector.rknn",
        classes_path="/models/classes.txt",
        container_image_ref="ghcr.io/acme/omniseer:robot-v2",
        container_image_digest="ghcr.io/acme/omniseer@sha256:0123456789abcdef",
        container_image_id="sha256:local-image-id",
        experiment_config="runtime-container-full",
        experiment_parameters={"profile": "operator", "stage": "full"},
        model_family="yolo-world",
        model_variant="v2m",
        model_precision="int8",
        model_backend="rknn",
    )


def _detection_record(*, received_ns: int = 100) -> dict:
    return make_detection_record(
        recv_ts_ns=received_ns,
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


def _perf_record(*, received_ns: int = 200, produced: int = 10, consumed: int = 9) -> dict:
    return make_perf_record(
        recv_ts_ns=received_ns,
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
        produced_count=produced,
        consumed_count=consumed,
        error_counts={"capture_retryable": 1, "infer": 2, "capture_fatal": 0},
    )


def _system_record(*, process_cpu: list[dict] | None = None) -> dict:
    return make_system_record(
        recv_ts_ns=300,
        cpu_percent=38.2,
        memory_used_mb=812.0,
        memory_available_mb=7200.0,
        soc_temp_c=61.4,
        thermal={"available": True, "soc_temp_c": 61.4, "throttled": False, "zones": []},
        process_cpu=process_cpu,
    )


def _write_completed_bundle(run_dir: Path, *, launch_args: tuple[str, ...] = ()) -> None:
    writer = RunBundleWriter(_config(run_dir, launch_args=launch_args), started_at=STARTED_AT)
    writer.write_detection_record(_detection_record())
    writer.write_detection_record(_detection_record(received_ns=1_000_000_100))
    writer.write_perf_record(_perf_record())
    writer.write_perf_record(_perf_record(received_ns=1_000_000_200, produced=20, consumed=18))
    writer.write_system_record(_system_record())
    writer.finalize(ended_at=ENDED_AT)


def _write_autonomy(run_dir: Path, *, failed: bool = False) -> None:
    terminal_state = "failed" if failed else "success"
    terminal_event = "failed" if failed else "succeeded"
    records = [
        {"time_sec": 0.4, "state": "lock", "event": "first_detection", "target_class": "chair"},
        {
            "time_sec": 1.0,
            "state": "center",
            "event": "target_lost",
            "reason": "no_target",
            "normalized_error": 0.20,
            "target": {"class_name": "chair", "confidence": 0.70},
        },
        {
            "time_sec": 1.2,
            "state": "center",
            "event": "centering_resumed",
            "normalized_error": 0.08,
            "target": {"class_name": "chair", "confidence": 0.82},
        },
        {
            "time_sec": 1.6,
            "state": terminal_state,
            "event": terminal_event,
            "reason": "target_timeout" if failed else "centered",
            "normalized_error": 0.02,
            "bbox_area_ratio": 0.12,
            "target_loss_count": 1,
            "target": {"class_name": "chair", "confidence": 0.87},
        },
    ]
    (run_dir / "autonomy.jsonl").write_text(
        "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
    )


def _write_pipeline_telemetry(run_dir: Path) -> None:
    records = [
        {"source": "consumer", "consumer_end_ts_real_ns": 2_000, "source_age_end_ns": 12_000_000},
        {"source": "consumer", "consumer_end_ts_real_ns": 1_000_002_000, "source_age_end_ns": 13_000_000},
    ]
    (run_dir / "pipeline_telemetry.jsonl").write_text(
        "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
    )


def _write_evidence(run_dir: Path, *, count: int = 1) -> None:
    frames_dir = run_dir / "evidence" / "frames"
    annotated_dir = run_dir / "evidence" / "annotated"
    frames_dir.mkdir(parents=True, exist_ok=True)
    annotated_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for index in range(count):
        name = f"frame_{index}.jpg"
        (frames_dir / name).write_bytes(b"clean-jpeg-fixture")
        (annotated_dir / name).write_bytes(b"annotated-jpeg-fixture")
        records.append(
            {
                "artifact_type": "sampled_frame",
                "image_path": f"evidence/frames/{name}",
                "capture_reason": "periodic",
                "frame_id": index,
                "sequence": index,
                "capture_ts_real_ns": int(STARTED_AT.timestamp() * 1_000_000_000) + index,
                "jpeg_quality": 85,
                "detections": [{"class_name": "chair", "score": 0.91}],
            }
        )
    (run_dir / "evidence" / "evidence.jsonl").write_text(
        "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
    )


class RunReportTests(unittest.TestCase):
    def test_renders_concise_decision_focused_sections(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir, launch_args=AUTONOMY_LAUNCH_ARGS)
            _write_autonomy(run_dir)
            _write_pipeline_telemetry(run_dir)
            _write_evidence(run_dir)

            summary = write_run_report(run_dir)
            output = summary.output_path.read_text(encoding="utf-8")

            for title in ("Run Overview", "Behavior", "Performance", "Evidence &amp; Provenance"):
                self.assertIn(f"<h2>{title}</h2>", output)
            self.assertIn("Outcome: <strong>success</strong>", output)
            self.assertIn("<th>Target class</th><td>chair</td>", output)
            self.assertIn("<th>Duration</th><td>65.0s</td>", output)
            self.assertIn("<th>Time to first detection</th><td>0.4s</td>", output)
            self.assertIn("<th>Time to success</th><td>1.6s</td>", output)
            self.assertIn("<th>Source-age p95</th><td>13.00 ms</td>", output)
            self.assertIn("<th>Mean consumer FPS</th><td>19.00</td>", output)
            self.assertNotIn("Consumer FPS p95</th>", output)
            self.assertIn("<td>Consumer FPS</td><td>19.00</td><td>19.00</td><td>19.00</td>", output)
            self.assertIn("YOLO-World · v2-M · INT8 · RKNN", output)
            self.assertIn("Autonomy Decision Parameters", output)
            self.assertIn("Framing area ratio (min / stop / max)", output)
            self.assertIn("Target Reliability", output)
            self.assertIn("<th>Observations passing confidence threshold</th><td>2/3 (66.7%)</td>", output)
            self.assertIn("<th>Missing / invalid target updates</th><td>1</td>", output)
            self.assertIn("<th>Recovered target-loss episodes</th><td>1</td>", output)
            self.assertIn("<th>Runtime backend</th><td>robot_runtime_container</td>", output)
            self.assertIn("<th>Runtime image ID</th><td>sha256:local-image-id</td>", output)
            self.assertIn("State Timeline", output)
            self.assertIn("Target Confidence and Centering Error", output)
            self.assertIn("Target Loss Episodes", output)
            self.assertIn("Detections by Class", output)
            self.assertIn("Latency Summary", output)
            self.assertIn("Latency Over Time", output)
            self.assertIn("Latest-frame Supersession", output)
            self.assertIn("System Summary", output)
            self.assertIn("All RunBundle artifacts", output)

    def test_omits_verbose_sections_and_zero_issue_section(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")

            for title in (
                "Evidence Summary",
                "Run Summary",
                "Data Coverage",
                "Key Metrics",
                "Autonomy",
                "Detections",
                "Pipeline Telemetry",
                "Errors And Drops",
                "Configuration",
                "CPU Consumers",
                "Issues",
            ):
                self.assertNotIn(f"<h2>{title}</h2>", output)
            for removed_detail in (
                "Detection Count Over Time",
                "Linear Command Over Time",
                "Angular Command Over Time",
                "Command Summary",
                "Event Timeline",
                "metric-bar",
                "WiFi",
                "Battery Summary",
                "First sample",
                "Launch command",
                "Launch profile",
                "Detections topic",
            ):
                self.assertNotIn(removed_detail, output)

    def test_shows_issues_only_when_present(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(
                _config(run_dir, launch_args=("start_autonomy:=true", "autonomy_target_class:=chair")),
                started_at=STARTED_AT,
            )
            writer.write_detection_record(_detection_record())
            writer.write_perf_record(_perf_record())
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")
            self.assertIn('<details id="issues" open>', output)
            self.assertIn("missing_autonomy_jsonl", output)

    def test_failure_reason_is_prominent_when_failed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_autonomy(run_dir, failed=True)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")
            self.assertIn('class="outcome-banner outcome-failed"', output)
            self.assertIn("Outcome: <strong>failed</strong>", output)
            self.assertIn("<th>Failure reason</th><td>target_timeout</td>", output)
            self.assertIn("<th>Last valid centering error</th><td>0.02</td>", output)
            self.assertIn("<th>Last valid target area</th><td>0.12</td>", output)
            self.assertNotIn("<th>Final centering error</th>", output)

    def test_collapses_complete_evidence_gallery_while_showing_representative_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            _write_evidence(run_dir, count=13)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")
            self.assertIn("<summary>All captured frames (13)</summary>", output)
            self.assertNotIn('<details class="nested-details" open><summary>All captured frames', output)
            self.assertEqual(output.count('class="evidence-card"'), 17)
            gallery = output.split("<summary>All captured frames (13)</summary>", maxsplit=1)[1].split(
                "</details><h3>Provenance</h3>", maxsplit=1
            )[0]
            self.assertEqual(gallery.count('href="../evidence/frames/frame_'), 13)
            self.assertIn('href="../evidence/frames/frame_12.jpg"', gallery)

    def test_keeps_overlay_video_behavior(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            video_dir = run_dir / "video"
            video_dir.mkdir()
            (video_dir / "source.mp4").write_bytes(b"mp4")
            (video_dir / "overlay.mp4").write_bytes(b"mp4")

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")
            self.assertIn('<details id="video" open>', output)
            self.assertIn("video/overlay.mp4", output)
            self.assertNotIn("Source video", output)

    def test_collapses_top_five_cpu_consumers(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            writer = RunBundleWriter(_config(run_dir), started_at=STARTED_AT)
            writer.write_perf_record(_perf_record())
            writer.write_system_record(
                _system_record(
                    process_cpu=[
                        {
                            "pid": index,
                            "start_time_ticks": index,
                            "name": f"process-{index}",
                            "cmdline": "",
                            "cpu_seconds_delta": float(index),
                        }
                        for index in range(1, 7)
                    ]
                )
            )
            writer.finalize(ended_at=ENDED_AT)

            output = write_run_report(run_dir).output_path.read_text(encoding="utf-8")
            self.assertIn("Top CPU consumers (sampled)", output)
            self.assertIn("process-6", output)
            self.assertNotIn("process-1</td>", output)
            self.assertIn("not total machine utilization", output)

    def test_requires_overwrite_and_cli_reports_output(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_completed_bundle(run_dir)
            summary = write_run_report(run_dir)
            with self.assertRaises(FileExistsError):
                write_run_report(run_dir)
            self.assertEqual(summary.output_path, write_run_report(run_dir, overwrite=True).output_path)

            other_run_dir = Path(tmp) / "cli_run"
            _write_completed_bundle(other_run_dir)
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                report_run_main([str(other_run_dir)])
            self.assertIn(f"Report: {other_run_dir / 'report' / 'index.html'}", stream.getvalue())


if __name__ == "__main__":
    unittest.main()
