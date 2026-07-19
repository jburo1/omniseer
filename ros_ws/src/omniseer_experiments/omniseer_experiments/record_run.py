"""ROS node for recording local perception run bundles."""

from __future__ import annotations

import argparse
import queue
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from yolo_msgs.msg import DetectionArray

from omniseer_experiments.bundle import (
    DEFAULT_DETECTIONS_TOPIC,
    DEFAULT_PERF_TOPIC,
    ERROR_FIELDS,
    RunBundleConfig,
    RunBundleWriter,
    default_run_id,
    make_detection_record,
    make_perf_record,
)
from omniseer_msgs.msg import VisionPerfSummary

DEFAULT_QUEUE_SIZE = 256
USE_CONFIG_SENTINEL = "__from_config__"


@dataclass(frozen=True)
class RecorderOptions:
    run_id: str
    out_dir: Path
    classes: tuple[str, ...] = ()
    notes: str = ""
    duration_sec: float = 0.0
    overwrite: bool = False
    detections_topic: str = DEFAULT_DETECTIONS_TOPIC
    perf_topic: str = DEFAULT_PERF_TOPIC
    vision_params_file: Path | None = None
    detector_model_path: str = ""
    clip_model_path: str = ""
    clip_vocab_path: str = ""
    classes_path: str = ""
    queue_size: int = DEFAULT_QUEUE_SIZE
    flush_interval_sec: float = 1.0


class AsyncBundleWriter:
    """Own file I/O on a background thread with bounded queueing."""

    def __init__(self, bundle: RunBundleWriter, *, queue_size: int, flush_interval_sec: float) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be > 0")
        if flush_interval_sec <= 0.0:
            raise ValueError("flush_interval_sec must be > 0")

        self._bundle = bundle
        self._queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue(maxsize=queue_size)
        self._flush_interval_sec = flush_interval_sec
        self._stop_requested = threading.Event()
        self._thread = threading.Thread(target=self._run, name="omniseer_run_bundle_writer", daemon=True)
        self._thread.start()

    @property
    def bundle(self) -> RunBundleWriter:
        return self._bundle

    def submit(self, stream: str, record: dict[str, Any]) -> bool:
        if self._stop_requested.is_set():
            self._bundle.record_drop(stream)
            return False
        try:
            self._queue.put_nowait((stream, record))
        except queue.Full:
            self._bundle.record_drop(stream)
            return False
        return True

    def close(self) -> dict[str, Any]:
        self._stop_requested.set()
        self._queue.put(None)
        self._thread.join()
        return self._bundle.finalize()

    def _run(self) -> None:
        next_flush = time.monotonic() + self._flush_interval_sec
        while True:
            timeout = max(0.0, next_flush - time.monotonic())
            try:
                item = self._queue.get(timeout=timeout)
            except queue.Empty:
                self._flush()
                next_flush = time.monotonic() + self._flush_interval_sec
                continue

            if item is None:
                self._queue.task_done()
                break

            self._write_item(item)
            self._queue.task_done()

        self._drain_pending()
        self._flush()

    def _drain_pending(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                return
            try:
                if item is not None:
                    self._write_item(item)
            finally:
                self._queue.task_done()

    def _write_item(self, item: tuple[str, dict[str, Any]]) -> None:
        stream, record = item
        if stream == "detections":
            self._bundle.write_detection_record(record)
        elif stream == "perf":
            self._bundle.write_perf_record(record)
        else:
            self._bundle.record_drop(stream)

    def _flush(self) -> None:
        self._bundle.flush()


class PerceptionRunRecorder(Node):
    """Record perception detections and performance telemetry into a run bundle."""

    def __init__(self, options: RecorderOptions) -> None:
        super().__init__("perception_run_recorder")
        self._options = options
        self._closed = False

        bundle = RunBundleWriter(
            RunBundleConfig(
                run_id=options.run_id,
                out_dir=options.out_dir,
                classes=options.classes,
                notes=options.notes,
                overwrite=options.overwrite,
                vision_params_file=str(options.vision_params_file or ""),
                detector_model_path=options.detector_model_path,
                clip_model_path=options.clip_model_path,
                clip_vocab_path=options.clip_vocab_path,
                classes_path=options.classes_path,
                detections_topic=options.detections_topic,
                perf_topic=options.perf_topic,
            )
        )
        self._writer = AsyncBundleWriter(
            bundle,
            queue_size=options.queue_size,
            flush_interval_sec=options.flush_interval_sec,
        )

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max(1, min(options.queue_size, 10)),
        )
        self.create_subscription(DetectionArray, options.detections_topic, self._on_detections, qos)
        self.create_subscription(VisionPerfSummary, options.perf_topic, self._on_perf, qos)

        if options.duration_sec > 0.0:
            self.create_timer(options.duration_sec, self._finish_duration)

        self.get_logger().info(
            f"recording perception run bundle: run_id={options.run_id} out_dir={options.out_dir}"
        )

    def close(self) -> dict[str, Any]:
        if self._closed:
            return self._writer.bundle.summary_from_disk()
        self._closed = True
        summary = self._writer.close()
        self.get_logger().info(f"finalized perception run bundle: out_dir={self._options.out_dir}")
        return summary

    def _on_detections(self, message: DetectionArray) -> None:
        record = detection_array_to_record(message, topic=self._options.detections_topic)
        accepted = self._writer.submit("detections", record)
        if not accepted:
            self.get_logger().warning("dropped detection record because recorder queue is full")

    def _on_perf(self, message: VisionPerfSummary) -> None:
        record = perf_summary_to_record(message, topic=self._options.perf_topic)
        accepted = self._writer.submit("perf", record)
        if not accepted:
            self.get_logger().warning("dropped perf record because recorder queue is full")

    def _finish_duration(self) -> None:
        self.get_logger().info("recording duration elapsed; shutting down recorder")
        rclpy.shutdown()


def detection_array_to_record(message: DetectionArray, *, topic: str = DEFAULT_DETECTIONS_TOPIC) -> dict[str, Any]:
    detections = []
    for detection in message.detections:
        detections.append(
            {
                "class_id": int(detection.class_id),
                "class_name": str(detection.class_name),
                "score": float(detection.score),
                "bbox": {
                    "center_x": float(detection.bbox.center.position.x),
                    "center_y": float(detection.bbox.center.position.y),
                    "size_x": float(detection.bbox.size.x),
                    "size_y": float(detection.bbox.size.y),
                },
            }
        )

    return make_detection_record(
        topic=topic,
        recv_ts_ns=time.time_ns(),
        header_stamp=stamp_to_dict(message.header.stamp),
        frame_id=str(message.header.frame_id),
        detections=detections,
    )


def perf_summary_to_record(message: VisionPerfSummary, *, topic: str = DEFAULT_PERF_TOPIC) -> dict[str, Any]:
    return make_perf_record(
        topic=topic,
        recv_ts_ns=time.time_ns(),
        header_stamp=stamp_to_dict(message.header.stamp),
        frame_id=str(message.header.frame_id),
        producer_fps=float(message.producer_fps),
        consumer_fps=float(message.consumer_fps),
        last_preprocess_ms=float(message.last_preprocess_ms),
        last_infer_ms=float(message.last_infer_ms),
        last_postprocess_ms=float(message.last_postprocess_ms),
        last_publish_ms=float(message.last_publish_ms),
        last_producer_total_ms=float(message.last_producer_total_ms),
        last_consumer_total_ms=float(message.last_consumer_total_ms),
        produced_count=int(message.produced_count),
        consumed_count=int(message.consumed_count),
        error_counts={
            "no_writable_buffer": int(message.no_writable_buffer_count),
            "capture_retryable": int(message.capture_retryable_error_count),
            "capture_fatal": int(message.capture_fatal_error_count),
            "preprocess": int(message.preprocess_error_count),
            "infer": int(message.infer_error_count),
        },
    )


def stamp_to_dict(stamp: Any) -> dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def options_from_args(argv: list[str] | None = None) -> RecorderOptions:
    parser = _build_parser()
    cli_args = remove_ros_args(args=argv)
    if cli_args and not cli_args[0].startswith("-"):
        cli_args = cli_args[1:]
    args = parser.parse_args(cli_args)
    run_id = args.run_id or default_run_id()
    out_dir = Path(args.out) if args.out else Path("runs") / run_id
    vision_params_file = Path(args.vision_params_file) if args.vision_params_file else None
    vision_params = _load_flat_ros_params(vision_params_file) if vision_params_file else {}
    detector_model_path = _resolve_config_value(args.detector_model_path, vision_params, "models.detector_model_path")
    clip_model_path = _resolve_config_value(args.clip_model_path, vision_params, "models.clip_model_path")
    clip_vocab_path = _resolve_config_value(args.clip_vocab_path, vision_params, "models.clip_vocab_path")
    classes_path = _resolve_config_value(args.classes_path, vision_params, "classes.path")
    classes = _normalize_classes(args.classes) or _load_classes(classes_path)

    return RecorderOptions(
        run_id=run_id,
        out_dir=out_dir,
        classes=tuple(classes),
        notes=args.notes,
        duration_sec=args.duration_sec,
        overwrite=_as_bool(args.overwrite),
        detections_topic=args.detections_topic,
        perf_topic=args.perf_topic,
        vision_params_file=vision_params_file,
        detector_model_path=detector_model_path,
        clip_model_path=clip_model_path,
        clip_vocab_path=clip_vocab_path,
        classes_path=classes_path,
        queue_size=args.queue_size,
        flush_interval_sec=args.flush_interval_sec,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record a local Omniseer perception run bundle.")
    parser.add_argument("--run-id", default="", help="run identifier; defaults to a UTC timestamp")
    parser.add_argument("--out", default="", help="output run directory; defaults to runs/<run-id>")
    parser.add_argument("--classes", nargs="*", default=[], help="semantic classes configured for this run")
    parser.add_argument("--notes", default="", help="free-form run notes")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="stop after this duration; 0 records until shutdown",
    )
    parser.add_argument(
        "--overwrite",
        nargs="?",
        const="true",
        default="false",
        help="replace an existing run directory; accepts true/false when launched",
    )
    parser.add_argument("--detections-topic", default=DEFAULT_DETECTIONS_TOPIC)
    parser.add_argument("--perf-topic", default=DEFAULT_PERF_TOPIC)
    parser.add_argument("--vision-params-file", default="", help="vision bridge parameter file for metadata fallback")
    parser.add_argument("--detector-model-path", default="", help="detector model path or __from_config__")
    parser.add_argument("--clip-model-path", default="", help="CLIP text model path or __from_config__")
    parser.add_argument("--clip-vocab-path", default="", help="CLIP vocab path or __from_config__")
    parser.add_argument("--classes-path", default="", help="class list path or __from_config__")
    parser.add_argument("--queue-size", type=int, default=DEFAULT_QUEUE_SIZE)
    parser.add_argument("--flush-interval-sec", type=float, default=1.0)
    return parser


def _normalize_classes(values: list[str]) -> list[str]:
    classes: list[str] = []
    for value in values:
        for token in value.replace(",", " ").split():
            if token:
                classes.append(token)
    return classes


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return False


def _resolve_config_value(raw_value: str, params: dict[str, str], key: str) -> str:
    if raw_value and raw_value != USE_CONFIG_SENTINEL:
        return raw_value
    return params.get(key, "")


def _load_classes(classes_path: str) -> list[str]:
    if not classes_path:
        return []
    path = Path(classes_path)
    try:
        return [
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
    except OSError:
        return []


def _load_flat_ros_params(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return {}

    params: dict[str, str] = {}
    in_parameters = False
    parameter_indent = 0
    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip())
        if stripped == "ros__parameters:":
            in_parameters = True
            parameter_indent = indent
            continue
        if not in_parameters:
            continue
        if indent <= parameter_indent:
            break
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        params[key.strip()] = _unquote_scalar(value.strip())
    return params


def _unquote_scalar(value: str) -> str:
    if not value:
        return ""
    if value[0] in {"'", '"'} and value[-1:] == value[0]:
        return value[1:-1]
    return value


def main(argv: list[str] | None = None) -> None:
    options = options_from_args(argv)
    rclpy.init(args=argv)
    node = PerceptionRunRecorder(options)
    _install_signal_handlers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _install_signal_handlers() -> None:
    signal.signal(signal.SIGTERM, lambda _signum, _frame: rclpy.shutdown())


__all__ = [
    "ERROR_FIELDS",
    "AsyncBundleWriter",
    "PerceptionRunRecorder",
    "RecorderOptions",
    "detection_array_to_record",
    "main",
    "options_from_args",
    "perf_summary_to_record",
    "stamp_to_dict",
]


if __name__ == "__main__":
    main()
