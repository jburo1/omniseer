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
    canonicalize_ros_graph,
    capture_ros_graph_safely,
    detection_array_to_record,
    options_from_args,
    perf_summary_to_record,
    unavailable_lipo_battery_snapshot,
    wait_for_stable_ros_graph,
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


class _GraphEndpoint:
    def __init__(self, name: str, namespace: str, topic_type: str, qos_profile=None) -> None:
        self.node_name = name
        self.node_namespace = namespace
        self.topic_type = topic_type
        self.qos_profile = SimpleNamespace(**qos_profile) if isinstance(qos_profile, dict) else qos_profile


class _GraphNode:
    def __init__(self, graph: dict | None = None, error: Exception | None = None) -> None:
        self._graph = graph or {"nodes": [], "topics": []}
        self._error = error

    def get_node_names_and_namespaces(self):
        self._raise_if_needed()
        return [(item["name"], item["namespace"]) for item in self._graph["nodes"]]

    def get_topic_names_and_types(self):
        self._raise_if_needed()
        return [(item["name"], item["types"]) for item in self._graph["topics"]]

    def get_publishers_info_by_topic(self, topic_name: str):
        return self._endpoints(topic_name, "publishers")

    def get_subscriptions_info_by_topic(self, topic_name: str):
        return self._endpoints(topic_name, "subscribers")

    def _endpoints(self, topic_name: str, kind: str):
        self._raise_if_needed()
        topic = next(item for item in self._graph["topics"] if item["name"] == topic_name)
        return [
            _GraphEndpoint(
                endpoint["node_name"],
                endpoint["node_namespace"],
                endpoint["topic_type"],
                endpoint.get("qos"),
            )
            for endpoint in topic[kind]
        ]

    def _raise_if_needed(self) -> None:
        if self._error is not None:
            raise self._error


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

    def test_battery_state_to_snapshot_serializes_nan_percentage_as_unknown(self) -> None:
        message = _battery_message()
        message.percentage = float("nan")

        snapshot = battery_state_to_snapshot(message)

        self.assertEqual(snapshot["voltage"], 8.34)
        self.assertIsNone(snapshot["percentage"])

    def test_battery_state_to_snapshot_preserves_explicit_zero_percentage(self) -> None:
        message = _battery_message()
        message.percentage = 0.0

        snapshot = battery_state_to_snapshot(message)

        self.assertEqual(snapshot["percentage"], 0.0)

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

    def test_options_preserve_comma_delimited_class_phrases(self) -> None:
        options = options_from_args(
            [
                "record_run",
                "--run-id",
                "demo_001",
                "--classes",
                "person, fire extinguisher\ntraffic cone",
            ]
        )

        self.assertEqual(options.classes, ("person", "fire extinguisher", "traffic cone"))

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
                "--container-image-id",
                "sha256:local-image-id",
                "--runtime-backend",
                "robot_runtime_container",
                "--experiment-config",
                "experiments/container-smoke.yaml",
                "--model-family",
                "yolo-world",
                "--model-variant",
                "v2s",
                "--model-precision",
                "int8",
                "--model-backend",
                "rknn",
                "--comparison-id",
                "yolo-world-v2s-baseline",
                "--trial",
                "03",
                "--workload-id",
                "warehouse-aisle-a",
                "--resolved-vision-config-path",
                "/runs/demo_001/provenance/resolved_vision_config.yaml",
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
        self.assertEqual(options.container_image_id, "sha256:local-image-id")
        self.assertEqual(options.runtime_backend, "robot_runtime_container")
        self.assertEqual(options.experiment_config, "experiments/container-smoke.yaml")
        self.assertEqual(options.model_family, "yolo-world")
        self.assertEqual(options.model_variant, "v2s")
        self.assertEqual(options.model_precision, "int8")
        self.assertEqual(options.model_backend, "rknn")
        self.assertEqual(options.comparison_id, "yolo-world-v2s-baseline")
        self.assertEqual(options.trial, "03")
        self.assertEqual(options.workload_id, "warehouse-aisle-a")
        self.assertEqual(
            options.resolved_vision_config_path,
            "/runs/demo_001/provenance/resolved_vision_config.yaml",
        )
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
            os.environ["OMNISEER_EXPERIMENT_PARAMETERS"] = '{"profile":"operator","stage":"smoke"}'

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
        self.assertEqual(options.experiment_parameters, {"profile": "operator", "stage": "smoke"})


class RosGraphCaptureTests(unittest.TestCase):
    def test_canonicalizes_and_serializes_graph_deterministically(self) -> None:
        graph = {
            "nodes": [{"name": "z", "namespace": "/b"}, {"name": "a", "namespace": "/a"}],
            "topics": [
                {
                    "name": "/z",
                    "types": ["pkg/msg/B", "pkg/msg/A"],
                    "publishers": [
                        {"node_name": "z", "node_namespace": "/b", "topic_type": "pkg/msg/A", "qos": {"depth": 1}},
                        {"node_name": "a", "node_namespace": "/a", "topic_type": "pkg/msg/A", "qos": {"depth": 2}},
                    ],
                    "subscribers": [],
                }
            ],
        }
        reordered = {
            "nodes": list(reversed(graph["nodes"])),
            "topics": [
                {
                    **graph["topics"][0],
                    "types": list(reversed(graph["topics"][0]["types"])),
                    "publishers": list(reversed(graph["topics"][0]["publishers"])),
                }
            ],
        }

        canonical = canonicalize_ros_graph(graph)

        self.assertEqual(canonical, canonicalize_ros_graph(reordered))
        self.assertEqual(
            json.dumps(canonical, indent=2, sort_keys=True, allow_nan=False),
            json.dumps(canonicalize_ros_graph(reordered), indent=2, sort_keys=True, allow_nan=False),
        )
        self.assertEqual(canonical["nodes"][0], {"name": "a", "namespace": "/a"})
        self.assertEqual(canonical["topics"][0]["types"], ["pkg/msg/A", "pkg/msg/B"])

    def test_waits_for_consecutive_identical_graph_observations(self) -> None:
        first = {"nodes": [], "topics": [{"name": "/a", "types": ["pkg/msg/A"]}]}
        stable = {"nodes": [], "topics": [{"name": "/b", "types": ["pkg/msg/B"]}]}
        observations = iter([first, stable, stable])

        snapshot = wait_for_stable_ros_graph(
            lambda: next(observations),
            timeout_sec=1.0,
            interval_sec=0.01,
            settle_sec=0.0,
            monotonic=lambda: 0.0,
            sleeper=lambda _duration: None,
        )

        self.assertTrue(snapshot["capture"]["stability_reached"])
        self.assertFalse(snapshot["capture"]["timeout_used"])
        self.assertEqual(snapshot["capture"]["observations"], 3)
        self.assertEqual(snapshot["topics"][0]["name"], "/b")

    def test_uses_latest_graph_when_stability_times_out(self) -> None:
        observations = iter(
            [
                {"nodes": [], "topics": [{"name": "/a", "types": ["pkg/msg/A"]}]},
                {"nodes": [], "topics": [{"name": "/b", "types": ["pkg/msg/B"]}]},
            ]
        )
        monotonic_values = iter([0.0, 0.5, 1.0])

        snapshot = wait_for_stable_ros_graph(
            lambda: next(observations),
            timeout_sec=1.0,
            interval_sec=0.1,
            settle_sec=0.0,
            monotonic=lambda: next(monotonic_values),
            sleeper=lambda _duration: None,
        )

        self.assertFalse(snapshot["capture"]["stability_reached"])
        self.assertTrue(snapshot["capture"]["timeout_used"])
        self.assertEqual(snapshot["capture"]["observations"], 2)
        self.assertEqual(snapshot["topics"][0]["name"], "/b")

    def test_settling_avoids_accepting_an_early_partial_graph(self) -> None:
        partial = {"nodes": [{"name": "perception_run_recorder", "namespace": "/"}], "topics": []}
        complete = {
            "nodes": [
                {"name": "perception_run_recorder", "namespace": "/"},
                {"name": "target_centering_node", "namespace": "/"},
            ],
            "topics": [
                {
                    "name": "/cmd_vel_autonomy",
                    "types": ["geometry_msgs/msg/TwistStamped"],
                    "publishers": [
                        {
                            "node_name": "target_centering_node",
                            "node_namespace": "/",
                            "topic_type": "geometry_msgs/msg/TwistStamped",
                        }
                    ],
                    "subscribers": [],
                }
            ],
        }
        node_only = {"nodes": complete["nodes"], "topics": []}

        def capture_with_settle(settle_sec: float) -> dict:
            observations = iter([partial, partial, partial, node_only, complete, complete, complete])
            times = iter([0.0, 0.0, 0.25, 0.50, 1.00, 1.25, 2.00, 2.25])
            return wait_for_stable_ros_graph(
                lambda: next(observations),
                timeout_sec=5.0,
                interval_sec=0.25,
                settle_sec=settle_sec,
                monotonic=lambda: next(times),
                sleeper=lambda _duration: None,
            )

        old_behavior = capture_with_settle(0.0)
        snapshot = capture_with_settle(2.0)

        self.assertEqual(old_behavior["nodes"], partial["nodes"])
        self.assertEqual(snapshot["nodes"], complete["nodes"])
        self.assertEqual(snapshot["topics"], canonicalize_ros_graph(complete)["topics"])
        self.assertEqual(snapshot["capture"]["observations"], 7)
        self.assertEqual(snapshot["capture"]["settle_sec"], 2.0)
        self.assertTrue(snapshot["capture"]["stability_reached"])

    def test_writes_one_snapshot_at_the_required_path(self) -> None:
        graph = {
            "nodes": [{"name": "recorder", "namespace": "/"}],
            "topics": [
                {
                    "name": "/detections",
                    "types": ["yolo_msgs/msg/DetectionArray"],
                    "publishers": [],
                    "subscribers": [
                        {
                            "node_name": "recorder",
                            "node_namespace": "/",
                            "topic_type": "yolo_msgs/msg/DetectionArray",
                            "qos": {"reliability": "RELIABLE"},
                        }
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            try:
                self.assertTrue(
                    capture_ros_graph_safely(
                        node=_GraphNode(graph),
                        bundle=bundle,
                        warning=lambda _message: self.fail("graph capture should not warn"),
                        timeout_sec=0.1,
                        interval_sec=0.001,
                    )
                )
                snapshot_path = run_dir / "ros_graph" / "topology.json"
                self.assertTrue(snapshot_path.is_file())
                self.assertEqual(len(list((run_dir / "ros_graph").glob("topology.json"))), 1)
                snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                self.assertEqual(snapshot["topics"][0]["subscribers"][0]["qos"], {"reliability": "RELIABLE"})
            finally:
                bundle.close()

    def test_graph_capture_failure_only_warns(self) -> None:
        warnings = []
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            bundle = RunBundleWriter(RunBundleConfig(run_id="demo_001", out_dir=run_dir, git_sha="abc123"))
            try:
                result = capture_ros_graph_safely(
                    node=_GraphNode(error=RuntimeError("graph unavailable")),
                    bundle=bundle,
                    warning=warnings.append,
                    timeout_sec=0.1,
                    interval_sec=0.001,
                )
                self.assertFalse(result)
                self.assertEqual(len(warnings), 1)
                self.assertIn("graph unavailable", warnings[0])
                self.assertFalse((run_dir / "ros_graph" / "topology.json").exists())
            finally:
                bundle.close()


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
