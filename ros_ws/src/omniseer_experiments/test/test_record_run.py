import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from omniseer_experiments.bundle import RunBundleConfig, RunBundleWriter
from omniseer_experiments.record_run import (
    AsyncBundleWriter,
    detection_array_to_record,
    options_from_args,
    perf_summary_to_record,
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


class AsyncBundleWriterTests(unittest.TestCase):
    def test_async_writer_writes_records_and_finalizes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=4, flush_interval_sec=0.01)

            self.assertTrue(writer.submit("detections", detection_array_to_record(_detection_message())))
            self.assertTrue(writer.submit("perf", perf_summary_to_record(_perf_message())))
            summary = writer.close()

            self.assertEqual(summary["message_counts"], {"detections": 1, "perf": 1})
            self.assertEqual(summary["detections_by_class"], {"chair": 1})
            self.assertTrue((run_dir / "summary.json").is_file())
            with (run_dir / "perf.jsonl").open("r", encoding="utf-8") as handle:
                self.assertEqual(len([json.loads(line) for line in handle if line.strip()]), 1)

    def test_async_writer_rejects_submit_after_close(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            writer = AsyncBundleWriter(bundle, queue_size=1, flush_interval_sec=0.01)

            writer.close()

            self.assertFalse(writer.submit("detections", detection_array_to_record(_detection_message())))
