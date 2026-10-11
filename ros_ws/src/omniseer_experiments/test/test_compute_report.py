import json
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from omniseer_experiments.bundle import RunBundleConfig, RunBundleWriter, make_perf_record, make_system_record
from omniseer_experiments.compute_report import is_compute_performance_run, write_compute_report

START = datetime(2026, 7, 19, 12, 0, 0, tzinfo=timezone.utc)


class ComputeReportTests(unittest.TestCase):
    def _bundle(self, root: Path) -> Path:
        run_dir = root / "performance_001"
        writer = RunBundleWriter(
            RunBundleConfig(
                run_id="performance_001",
                out_dir=run_dir,
                experiment_config="Stationary Perception Profiling",
                experiment_parameters={
                    "performance.measurement_interval_start_at": "2026-07-19T12:00:10+00:00",
                    "performance.measurement_interval_end_at": "2026-07-19T12:00:20+00:00",
                    "runner.core_mask": "core0_1",
                },
                model_family="yolo-world",
                model_variant="v2s",
                model_precision="int8",
            ),
            started_at=START,
        )
        writer.write_perf_record(
            make_perf_record(
                recv_ts_ns=int(START.timestamp() * 1e9) + 15_000_000_000,
                header_stamp={},
                frame_id="",
                producer_fps=30,
                consumer_fps=20,
                last_preprocess_ms=99,
                last_infer_ms=999,
                last_postprocess_ms=99,
                last_publish_ms=1,
                last_producer_total_ms=1,
                last_consumer_total_ms=1,
                produced_count=100,
                consumed_count=80,
                error_counts={"infer": 1},
            )
        )
        writer.write_system_record(
            make_system_record(
                recv_ts_ns=int(START.timestamp() * 1e9) + 15_000_000_000,
                cpu_percent=40,
                memory_used_mb=800,
                memory_available_mb=1000,
                soc_temp_c=60,
                thermal={"throttled": False},
                process_cpu=[],
                npu={
                    "available": True,
                    "utilization_percent": {"core0": 10, "core1": None, "core2": 30},
                    "frequency_hz": 1_000_000_000,
                    "governor": "ondemand",
                },
            )
        )
        writer.finalize(ended_at=datetime(2026, 7, 19, 12, 0, 25, tzinfo=timezone.utc))
        telemetry = [
            {
                "source": "producer",
                "event_ts_real_ns": int(START.timestamp() * 1e9) + seconds * 1_000_000_000,
                "dur_ns": {"preprocess": duration},
            }
            for seconds, duration in (
                (5, 100_000_000),
                (11, 6_000_000),
                (19, 8_000_000),
                (19, None),
                (21, 200_000_000),
            )
        ] + [
            {
                "source": "consumer",
                "consumer_end_ts_real_ns": int(START.timestamp() * 1e9) + 5_000_000_000,
                "dur_ns": {"infer": 1_000_000},
            },
            {
                "source": "consumer",
                "consumer_end_ts_real_ns": int(START.timestamp() * 1e9) + 11_000_000_000,
                "source_age_end_ns": 4_000_000,
                "dur_ns": {
                    "infer": 10_000_000,
                    "acquire_read": 2_000_000,
                    "postprocess": 3_000_000,
                    "total": 15_000_000,
                },
                "infer_status": "ok",
                "consumer_status": "ok",
                "infer_errno": 0,
            },
            {
                "source": "consumer",
                "consumer_end_ts_real_ns": int(START.timestamp() * 1e9) + 19_000_000_000,
                "source_age_end_ns": 5_000_000,
                "dur_ns": {
                    "infer": 30_000_000,
                    "acquire_read": 2_000_000,
                    "postprocess": 4_000_000,
                    "total": 35_000_000,
                },
                "infer_status": "ok",
                "consumer_status": "ok",
                "infer_errno": 0,
            },
            {
                "source": "consumer",
                "consumer_end_ts_real_ns": int(START.timestamp() * 1e9) + 21_000_000_000,
                "dur_ns": {"infer": 100_000_000},
            },
        ]
        run_dir.joinpath("pipeline_telemetry.jsonl").write_text(
            "\n".join(json.dumps(item) for item in telemetry) + "\n", encoding="utf-8"
        )
        return run_dir

    def test_uses_native_measurement_interval_and_is_deterministic(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._bundle(Path(tmp))
            self.assertTrue(is_compute_performance_run(run_dir))
            first = write_compute_report(run_dir)
            first_html = first.output_path.read_text(encoding="utf-8")
            first_metrics = first.metrics_path.read_bytes()
            metrics = json.loads(first.metrics_path.read_text(encoding="utf-8"))
            self.assertEqual(metrics["workloads"][0]["native_inference_ms"]["samples"], 2)
            self.assertEqual(metrics["workloads"][0]["native_inference_ms"]["p95"], 30.0)
            self.assertEqual(metrics["workloads"][0]["native_inference_ms"]["p99"], 30.0)
            self.assertEqual(metrics["pipeline"]["preprocess_ms"]["samples"], 2)
            self.assertEqual(metrics["pipeline"]["preprocess_ms"]["p50"], 6.0)
            self.assertEqual(metrics["pipeline"]["preprocess_ms"]["p95"], 8.0)
            self.assertEqual(metrics["workloads"][0]["native_stage_ms"]["acquire_read"]["p50"], 2.0)
            self.assertEqual(metrics["experiment"]["measurement_interval_status"], "valid")
            self.assertIsNone(metrics["npu"]["per_core_utilization_percent"]["core1"])
            self.assertIn("Host-wide telemetry only", first_html)
            second = write_compute_report(run_dir, overwrite=True)
            self.assertEqual(first_html, second.output_path.read_text(encoding="utf-8"))
            self.assertEqual(first_metrics, second.metrics_path.read_bytes())

    def test_missing_producer_preprocessing_does_not_use_acquisition_or_periodic_latency(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._bundle(Path(tmp))
            path = run_dir / "pipeline_telemetry.jsonl"
            records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            for record in records:
                if record["source"] == "producer":
                    record["dur_ns"] = {"preprocess": None}
            path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")
            summary = write_compute_report(run_dir)
            metrics = json.loads(summary.metrics_path.read_text(encoding="utf-8"))
            self.assertIsNone(metrics["pipeline"]["preprocess_ms"])
            self.assertIsNotNone(metrics["workloads"][0]["native_stage_ms"]["acquire_read"])

    def test_periodic_counters_are_explicitly_cumulative(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._bundle(Path(tmp))
            summary = write_compute_report(run_dir)
            pipeline = json.loads(summary.metrics_path.read_text(encoding="utf-8"))["pipeline"]
            self.assertEqual(pipeline["measurement_consumer_frames"], 2)
            self.assertEqual(pipeline["cumulative_produced_minus_consumed_frames"], 20)
            self.assertEqual(pipeline["errors"]["periodic_counters"], {"infer": 1})
            self.assertIn("Cumulative since", pipeline["counter_scope"])
            html = summary.output_path.read_text(encoding="utf-8")
            self.assertIn("Cumulative produced minus consumed frames", html)
            self.assertIn("not interval counts or a superseded-frame count", html)
            self.assertNotIn("Frames superseded", html)

    def test_missing_or_invalid_boundaries_do_not_measure_full_or_partial_run(self):
        start_key = "performance.measurement_interval_start_at"
        end_key = "performance.measurement_interval_end_at"
        cases = (
            ({start_key: None, end_key: None}, "missing"),
            ({start_key: None}, "missing"),
            ({end_key: None}, "missing"),
            ({start_key: "not a date"}, "invalid"),
            ({end_key: 123}, "invalid"),
            ({end_key: "2026-07-19T12:00:05+00:00"}, "end must be later"),
            ({end_key: "2026-07-19T12:00:10+00:00"}, "end must be later"),
        )
        for changes, reason in cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as tmp:
                run_dir = self._bundle(Path(tmp))
                manifest_path = run_dir / "manifest.yaml"
                lines = []
                for line in manifest_path.read_text(encoding="utf-8").splitlines():
                    key = line.strip().split(":", 1)[0]
                    if key in changes:
                        if changes[key] is None:
                            continue
                        line = f"    {key}: {json.dumps(changes[key])}"
                    lines.append(line)
                manifest_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                raw_paths = [
                    run_dir / name
                    for name in ("manifest.yaml", "pipeline_telemetry.jsonl", "perf.jsonl", "system.jsonl")
                ]
                raw_before = [path.read_bytes() for path in raw_paths]
                summary = write_compute_report(run_dir)
                metrics = json.loads(summary.metrics_path.read_text(encoding="utf-8"))
                self.assertEqual(metrics["experiment"]["measurement_interval_status"], "unavailable")
                self.assertTrue(any(reason in issue for issue in summary.issues))
                self.assertTrue(
                    any("measurement_interval_unavailable" in issue for issue in metrics["experiment"]["issues"])
                )
                self.assertIsNone(metrics["pipeline"]["measurement_consumer_frames"])
                self.assertIsNone(metrics["pipeline"]["preprocess_ms"])
                self.assertIsNone(metrics["pipeline"]["throughput_fps"])
                self.assertIsNone(metrics["pipeline"]["cumulative_produced_minus_consumed_frames"])
                self.assertIsNone(metrics["workloads"][0]["native_inference_ms"])
                self.assertIsNone(metrics["system"]["cpu_percent"])
                self.assertIsNone(metrics["npu"]["frequency_mhz"])
                self.assertIn("measurement_interval_unavailable", summary.output_path.read_text(encoding="utf-8"))
                self.assertEqual(raw_before, [path.read_bytes() for path in raw_paths])

    def test_missing_optional_telemetry_is_unavailable_not_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._bundle(Path(tmp))
            (run_dir / "pipeline_telemetry.jsonl").unlink()
            (run_dir / "system.jsonl").unlink()
            summary = write_compute_report(run_dir)
            metrics = json.loads(summary.metrics_path.read_text(encoding="utf-8"))
            self.assertIsNone(metrics["workloads"][0]["native_inference_ms"])
            self.assertEqual(metrics["system"]["thermal_throttling"], "unavailable")
