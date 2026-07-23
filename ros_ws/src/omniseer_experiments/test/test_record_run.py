import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from omniseer_experiments.bundle import RunBundleConfig, RunBundleWriter, make_system_record
from omniseer_experiments.record_run import (
    AsyncBundleWriter,
    SystemTelemetryThread,
    battery_state_to_snapshot,
    detection_array_to_record,
    options_from_args,
    perf_summary_to_record,
    unavailable_lipo_battery_snapshot,
)


def _stamp(sec: int = 1, nanosec: int = 2) -> SimpleNamespace:
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _header(frame_id: str = "camera_frame") -> SimpleNamespace:
    return SimpleNamespace(stamp=_stamp(), frame_id=frame_id)


def _detection_message() -> SimpleNamespace:
    bbox = SimpleNamespace(
        center=SimpleNamespace(position=SimpleNamespace(x=10.0, y=20.0)),
        size=SimpleNamespace(x=30.0, y=40.0),
    )
    detection = SimpleNamespace(class_id=7, class_name="chair", score=0.82, bbox=bbox)
    return SimpleNamespace(header=_header(), detections=[detection])


def _perf_message() -> SimpleNamespace:
    return SimpleNamespace(
        header=_header(),
        producer_fps=20.0,
        consumer_fps=19.5,
        last_preprocess_ms=1.0,
        last_infer_ms=8.0,
        last_postprocess_ms=2.0,
        last_publish_ms=0.5,
        last_producer_total_ms=3.0,
        last_consumer_total_ms=10.0,
        produced_count=12,
        consumed_count=11,
        no_writable_buffer_count=1,
        capture_retryable_error_count=2,
        capture_fatal_error_count=0,
        preprocess_error_count=3,
        infer_error_count=4,
    )


def _battery_message() -> SimpleNamespace:
    return SimpleNamespace(
        present=True,
        voltage=8.34,
        percentage=0.72,
        power_supply_status=1,
        POWER_SUPPLY_STATUS_CHARGING=1,
    )


def _system_record(recv_ts_ns: int = 300) -> dict:
    return make_system_record(
        recv_ts_ns=recv_ts_ns,
        cpu_percent=38.2,
        memory_used_mb=812.0,
        memory_available_mb=7200.0,
        soc_temp_c=None,
    )


class _FakeSampler:
    def __init__(self) -> None:
        self.count = 0

    def sample(self) -> dict:
        self.count += 1
        return _system_record(recv_ts_ns=self.count)


class RecordRunConversionTests(unittest.TestCase):
    def test_detection_array_to_record_uses_minimal_bbox_shape(self) -> None:
        record = detection_array_to_record(_detection_message())

        self.assertEqual(record["schema_version"], 1)
        self.assertEqual(record["topic"], "/yolo/detections")
        self.assertEqual(record["header_stamp"], {"sec": 1, "nanosec": 2})
        self.assertEqual(record["frame_id"], "camera_frame")
        self.assertEqual(
            record["detections"],
            [
                {
                    "class_id": 7,
                    "class_name": "chair",
                    "score": 0.82,
                    "bbox": {"center_x": 10.0, "center_y": 20.0, "size_x": 30.0, "size_y": 40.0},
                }
            ],
        )
        self.assertIsInstance(record["recv_ts_ns"], int)

    def test_perf_summary_to_record_maps_error_counters(self) -> None:
        record = perf_summary_to_record(_perf_message())

        self.assertEqual(record["schema_version"], 1)
        self.assertEqual(record["topic"], "/vision/perf")
        self.assertEqual(record["producer_fps"], 20.0)
        self.assertEqual(record["consumer_fps"], 19.5)
        self.assertEqual(record["last_infer_ms"], 8.0)
        self.assertEqual(record["produced_count"], 12)
        self.assertEqual(record["consumed_count"], 11)
        self.assertEqual(
            record["error_counts"],
            {
                "no_writable_buffer": 1,
                "capture_retryable": 2,
                "capture_fatal": 0,
                "preprocess": 3,
                "infer": 4,
            },
        )

    def test_battery_state_to_snapshot_normalizes_percentage(self) -> None:
        snapshot = battery_state_to_snapshot(_battery_message())

        self.assertEqual(snapshot["topic"], "/battery")
        self.assertEqual(snapshot["voltage"], 8.34)
        self.assertEqual(snapshot["percentage"], 72.0)
        self.assertEqual(snapshot["charging"], True)

    def test_unavailable_lipo_battery_snapshot_has_explicit_shape(self) -> None:
        snapshot = unavailable_lipo_battery_snapshot("/custom_battery")

        self.assertEqual(snapshot["available"], False)
        self.assertEqual(snapshot["topic"], "/custom_battery")
        self.assertIsNone(snapshot["voltage"])

    def test_options_strip_program_name_and_ros_args(self) -> None:
        options = options_from_args(
            [
                "record_run",
                "--run-id",
                "demo_001",
                "--classes",
                "chair,backpack",
                "fire_extinguisher",
                "--ros-args",
                "-p",
                "unused:=true",
            ]
        )

        self.assertEqual(options.run_id, "demo_001")
        self.assertEqual(options.out_dir, Path("runs") / "demo_001")
        self.assertEqual(options.classes, ("chair", "backpack", "fire_extinguisher"))

    def test_options_treat_empty_launch_values_as_defaults(self) -> None:
        options = options_from_args(["record_run", "--run-id", "demo_001", "--out", "", "--classes", ""])

        self.assertEqual(options.out_dir, Path("runs") / "demo_001")
        self.assertEqual(options.classes, ())

    def test_options_accept_launch_style_overwrite_value(self) -> None:
        options = options_from_args(["record_run", "--run-id", "demo_001", "--overwrite", "true"])

        self.assertTrue(options.overwrite)

    def test_options_resolve_assets_and_classes_from_vision_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            classes_path = Path(tmp) / "classes.txt"
            classes_path.write_text("person\nbus\n", encoding="utf-8")
            config_path = Path(tmp) / "vision.yaml"
            config_path.write_text(
                "\n".join(
                    [
                        "vision_bridge:",
                        "  ros__parameters:",
                        "    models.detector_model_path: /models/detector.rknn",
                        "    models.clip_model_path: /models/clip.rknn",
                        "    models.clip_vocab_path: /models/clip_vocab.bpe",
                        f"    classes.path: {classes_path}",
                    ]
                ),
                encoding="utf-8",
            )

            options = options_from_args(
                [
                    "record_run",
                    "--run-id",
                    "demo_001",
                    "--vision-params-file",
                    str(config_path),
                    "--detector-model-path",
                    "__from_config__",
                    "--clip-model-path",
                    "__from_config__",
                    "--clip-vocab-path",
                    "__from_config__",
                    "--classes-path",
                    "__from_config__",
                ]
            )

        self.assertEqual(options.detector_model_path, "/models/detector.rknn")
        self.assertEqual(options.clip_model_path, "/models/clip.rknn")
        self.assertEqual(options.clip_vocab_path, "/models/clip_vocab.bpe")
        self.assertEqual(options.classes_path, str(classes_path))
        self.assertEqual(options.classes, ("person", "bus"))

    def test_options_accept_container_and_experiment_provenance(self) -> None:
        options = options_from_args(
            [
                "record_run",
                "--run-id",
                "demo_001",
                "--container-image-ref",
                "ghcr.io/acme/omniseer:robot-v2",
                "--container-image-digest",
                "sha256:0123456789abcdef",
                "--experiment-config",
                "experiments/container-smoke.yaml",
                "--experiment-parameters",
                "profile=operator,camera=/dev/video11",
                "--experiment-parameter",
                "scenario=smoke",
                "--system-interval-sec",
                "0.5",
                "--launch-command",
                "run real --profile operator bringup",
                "--launch-profile",
                "operator",
                "--launch-mode",
                "bringup",
                "--launch-args",
                "start_gateway:=true camera_device:=/dev/video11",
                "--battery-topic",
                "/battery",
            ]
        )

        self.assertEqual(options.container_image_ref, "ghcr.io/acme/omniseer:robot-v2")
        self.assertEqual(options.container_image_digest, "sha256:0123456789abcdef")
        self.assertEqual(options.experiment_config, "experiments/container-smoke.yaml")
        self.assertEqual(
            options.experiment_parameters,
            {"camera": "/dev/video11", "profile": "operator", "scenario": "smoke"},
        )
        self.assertEqual(options.system_interval_sec, 0.5)
        self.assertEqual(options.launch_command, "run real --profile operator bringup")
        self.assertEqual(options.launch_profile, "operator")
        self.assertEqual(options.launch_mode, "bringup")
        self.assertEqual(options.launch_args, ("start_gateway:=true", "camera_device:=/dev/video11"))
        self.assertEqual(options.battery_topic, "/battery")

    def test_options_use_env_fallback_for_container_and_experiment_provenance(self) -> None:
        original_env = {
            "OMNISEER_CONTAINER_IMAGE_REF": os.environ.get("OMNISEER_CONTAINER_IMAGE_REF"),
            "OMNISEER_CONTAINER_IMAGE_DIGEST": os.environ.get("OMNISEER_CONTAINER_IMAGE_DIGEST"),
            "OMNISEER_EXPERIMENT_CONFIG": os.environ.get("OMNISEER_EXPERIMENT_CONFIG"),
            "OMNISEER_EXPERIMENT_PARAMETERS": os.environ.get("OMNISEER_EXPERIMENT_PARAMETERS"),
        }
        try:
            os.environ["OMNISEER_CONTAINER_IMAGE_REF"] = "ghcr.io/acme/omniseer:env"
            os.environ["OMNISEER_CONTAINER_IMAGE_DIGEST"] = "sha256:envdigest"
            os.environ["OMNISEER_EXPERIMENT_CONFIG"] = "experiments/env.yaml"
            os.environ["OMNISEER_EXPERIMENT_PARAMETERS"] = '{"profile":"operator","duration_sec":5}'

            options = options_from_args(["record_run", "--run-id", "demo_001"])
        finally:
            for key, value in original_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

        self.assertEqual(options.container_image_ref, "ghcr.io/acme/omniseer:env")
        self.assertEqual(options.container_image_digest, "sha256:envdigest")
        self.assertEqual(options.experiment_config, "experiments/env.yaml")
        self.assertEqual(options.experiment_parameters, {"duration_sec": "5", "profile": "operator"})


class AsyncBundleWriterTests(unittest.TestCase):
    def test_async_writer_writes_records_and_finalizes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=4, flush_interval_sec=0.01)

            self.assertTrue(writer.submit("detections", detection_array_to_record(_detection_message())))
            self.assertTrue(writer.submit("perf", perf_summary_to_record(_perf_message())))
            self.assertTrue(writer.submit("system", _system_record()))
            summary = writer.close()

            self.assertEqual(summary["message_counts"], {"detections": 1, "perf": 1, "system": 1})
            self.assertEqual(summary["detections_by_class"], {"chair": 1})
            self.assertTrue((run_dir / "summary.json").is_file())
            with (run_dir / "perf.jsonl").open("r", encoding="utf-8") as handle:
                self.assertEqual(len([json.loads(line) for line in handle if line.strip()]), 1)
            with (run_dir / "system.jsonl").open("r", encoding="utf-8") as handle:
                self.assertEqual(len([json.loads(line) for line in handle if line.strip()]), 1)

    def test_async_writer_rejects_submit_after_close(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=1, flush_interval_sec=0.01)

            writer.close()

            self.assertFalse(writer.submit("detections", detection_array_to_record(_detection_message())))

    def test_async_writer_counts_dropped_system_records(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=1, flush_interval_sec=0.01)

            writer.close()

            self.assertFalse(writer.submit("system", _system_record()))
            self.assertEqual(writer.bundle.summary.build_summary(0.0)["dropped_records"], {"system": 1})

    def test_system_telemetry_thread_samples_and_stops(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=8, flush_interval_sec=0.01)
            sampler = _FakeSampler()

            thread = SystemTelemetryThread(
                sampler=sampler,
                writer=writer,
                extra_snapshot=lambda: {"lipo_battery": {"available": False}},
                interval_sec=0.01,
            )
            thread.stop()
            summary = writer.close()
            system_records = _load_jsonl(run_dir / "system.jsonl")

        self.assertGreaterEqual(sampler.count, 1)
        self.assertGreaterEqual(summary["message_counts"]["system"], 1)
        self.assertEqual(system_records[-1]["lipo_battery"], {"available": False})


def _load_jsonl(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]
