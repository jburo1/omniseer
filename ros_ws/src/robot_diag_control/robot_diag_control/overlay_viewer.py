from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any

import grpc

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.gateway_client import (
    PROFILE_NAMES,
    PROFILE_TO_PROTO,
    TELEOP_STATE_NAMES,
    create_stub,
    format_preview_response,
    get_overlay_snapshot,
    set_preview_mode,
    target_for,
)
from robot_diag_control.preview_viewer import _validate_gstreamer_support


@dataclass(frozen=True)
class ScaledBox:
    x1: int
    y1: int
    x2: int
    y2: int


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Laptop video overlay viewer for the robot gateway")
    parser.add_argument("--host", default="127.0.0.1", help="gRPC gateway host")
    parser.add_argument("--port", type=int, default=50051, help="gRPC gateway port")
    parser.add_argument(
        "--preview-host",
        default=None,
        help="preview stream host; defaults to --host",
    )
    parser.add_argument("--preview-port", type=int, default=7100)
    parser.add_argument("--preview-latency-ms", type=int, default=125)
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_TO_PROTO),
        default="balanced",
        help="preview profile to request before playback",
    )
    parser.add_argument(
        "--mode",
        choices=("display", "fakesink"),
        default="display",
        help="display a window or consume frames without rendering",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        default=None,
        help="optional runtime limit; useful for smoke tests",
    )
    parser.add_argument(
        "--overlay-poll-hz",
        type=float,
        default=12.0,
        help="overlay snapshot polling rate",
    )
    parser.add_argument(
        "--min-score",
        type=float,
        default=0.0,
        help="minimum score to draw",
    )
    parser.add_argument(
        "--gst-launch-path",
        default="gst-launch-1.0",
        help="path to gst-launch-1.0 on the laptop",
    )
    parser.add_argument(
        "--leave-preview-running",
        action="store_true",
        help="do not disable preview when the helper exits",
    )
    return parser


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    return _build_parser().parse_args(sys.argv[1:] if args is None else args)


def _build_appsink_pipeline(
    parsed: argparse.Namespace,
    *,
    decoder_element: str = "avdec_h264",
) -> str:
    preview_host = parsed.preview_host or parsed.host
    return " ".join(
        [
            "srtsrc",
            f"uri=srt://{preview_host}:{parsed.preview_port}?mode=caller",
            f"latency={parsed.preview_latency_ms}",
            "wait-for-connection=false",
            "!",
            "tsdemux",
            "!",
            "h264parse",
            "!",
            decoder_element,
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=BGR",
            "!",
            "appsink",
            "drop=true",
            "sync=false",
            "max-buffers=1",
        ]
    )


def _scale_detection_box(
    detection: robot_gateway_pb2.OverlayDetection,
    *,
    source_width: int,
    source_height: int,
    frame_width: int,
    frame_height: int,
) -> ScaledBox:
    if source_width <= 0 or source_height <= 0 or frame_width <= 0 or frame_height <= 0:
        return ScaledBox(0, 0, 0, 0)

    scale_x = frame_width / source_width
    scale_y = frame_height / source_height
    left = (detection.bbox_center_x_px - detection.bbox_width_px / 2.0) * scale_x
    top = (detection.bbox_center_y_px - detection.bbox_height_px / 2.0) * scale_y
    right = (detection.bbox_center_x_px + detection.bbox_width_px / 2.0) * scale_x
    bottom = (detection.bbox_center_y_px + detection.bbox_height_px / 2.0) * scale_y
    return ScaledBox(
        max(0, min(frame_width - 1, round(left))),
        max(0, min(frame_height - 1, round(top))),
        max(0, min(frame_width - 1, round(right))),
        max(0, min(frame_height - 1, round(bottom))),
    )


def _class_color(class_name: str, class_id: int) -> tuple[int, int, int]:
    key = class_name or str(class_id)
    seed = sum((index + 1) * ord(char) for index, char in enumerate(key))
    return (
        80 + seed % 140,
        80 + (seed // 7) % 140,
        80 + (seed // 17) % 140,
    )


def _put_label(
    cv2: Any,
    frame: Any,
    text: str,
    origin: tuple[int, int],
    *,
    color: tuple[int, int, int] = (240, 240, 240),
    background: tuple[int, int, int] = (0, 0, 0),
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    baseline = 0
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = origin
    top = max(0, y - text_height - baseline - 6)
    right = min(frame.shape[1] - 1, x + text_width + 8)
    bottom = min(frame.shape[0] - 1, y + baseline + 4)
    cv2.rectangle(frame, (x, top), (right, bottom), background, -1)
    cv2.putText(frame, text, (x + 4, y - 3), font, scale, color, thickness, cv2.LINE_AA)


def _draw_detections(
    cv2: Any,
    frame: Any,
    snapshot: robot_gateway_pb2.OverlaySnapshot,
    *,
    min_score: float,
) -> None:
    detections = snapshot.detections
    if not detections.available or not detections.detections:
        return

    frame_height, frame_width = frame.shape[:2]
    source_width = detections.source_width_px
    source_height = detections.source_height_px
    for detection in detections.detections:
        if detection.score < min_score:
            continue
        box = _scale_detection_box(
            detection,
            source_width=source_width,
            source_height=source_height,
            frame_width=frame_width,
            frame_height=frame_height,
        )
        if box.x2 <= box.x1 or box.y2 <= box.y1:
            continue

        color = _class_color(detection.class_name, detection.class_id)
        thickness = 1 if detections.stale else 2
        cv2.rectangle(frame, (box.x1, box.y1), (box.x2, box.y2), color, thickness)
        label = f"{detection.class_name or detection.class_id} {detection.score:.2f}"
        _put_label(cv2, frame, label, (box.x1, max(16, box.y1)), color=(255, 255, 255), background=color)


def _draw_hud(
    cv2: Any,
    frame: Any,
    snapshot: robot_gateway_pb2.OverlaySnapshot | None,
    *,
    overlay_enabled: bool,
    min_score: float,
) -> None:
    if snapshot is None:
        lines = ["gateway: waiting for overlay snapshot"]
    else:
        status = snapshot.status
        detections = snapshot.detections
        vision = status.vision
        health = status.health
        preview = status.preview
        det_state = "unavailable"
        if detections.available:
            det_state = "stale" if detections.stale else "fresh"
        lines = [
            f"health={health.summary or 'unknown'} teleop={TELEOP_STATE_NAMES.get(status.teleop.state, 'unknown')}",
            "vision="
            f"{'stale' if vision.stale else 'fresh'} "
            f"prod={vision.producer_fps:.1f}Hz cons={vision.consumer_fps:.1f}Hz infer={vision.last_infer_ms:.1f}ms",
            "detections="
            f"{det_state} count={detections.detection_count} age={detections.age_ms}ms min={min_score:.2f}",
            f"preview={PROFILE_NAMES.get(preview.profile, 'unknown')} overlay={'on' if overlay_enabled else 'off'}",
        ]

    x = 10
    y = 24
    for line in lines:
        _put_label(cv2, frame, line, (x, y), color=(245, 245, 245), background=(20, 20, 20))
        y += 24


def _import_cv2() -> Any:
    try:
        import cv2  # type: ignore[import-not-found]
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "OpenCV is not available. Install a GUI-capable OpenCV package on the laptop "
            "(for Ubuntu: python3-opencv)."
        ) from error
    return cv2


def _run_viewer(
    cv2: Any,
    capture: Any,
    stub: Any,
    parsed: argparse.Namespace,
) -> int:
    last_snapshot: robot_gateway_pb2.OverlaySnapshot | None = None
    last_poll = 0.0
    poll_interval = 1.0 / max(parsed.overlay_poll_hz, 0.1)
    started_at = time.monotonic()
    overlay_enabled = True
    min_score = max(0.0, parsed.min_score)

    if parsed.mode == "display":
        cv2.namedWindow("Omniseer Overlay Viewer", cv2.WINDOW_NORMAL)

    while True:
        now = time.monotonic()
        if parsed.duration_seconds is not None and now - started_at >= parsed.duration_seconds:
            return 0

        if now - last_poll >= poll_interval:
            try:
                last_snapshot = get_overlay_snapshot(stub)
            except grpc.RpcError:
                last_snapshot = None
            last_poll = now

        ok, frame = capture.read()
        if not ok:
            time.sleep(0.02)
            continue

        if overlay_enabled and last_snapshot is not None:
            _draw_detections(cv2, frame, last_snapshot, min_score=min_score)
        _draw_hud(cv2, frame, last_snapshot, overlay_enabled=overlay_enabled, min_score=min_score)

        if parsed.mode == "fakesink":
            continue

        cv2.imshow("Omniseer Overlay Viewer", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in {27, ord("q"), ord("x")}:
            return 0
        if key == ord("o"):
            overlay_enabled = not overlay_enabled
        elif key == ord("["):
            min_score = max(0.0, min_score - 0.05)
        elif key == ord("]"):
            min_score = min(1.0, min_score + 0.05)


def run(parsed: argparse.Namespace) -> int:
    decoder_element = _validate_gstreamer_support(parsed)
    pipeline = _build_appsink_pipeline(parsed, decoder_element=decoder_element)
    cv2 = _import_cv2()
    preview_was_enabled = False

    with grpc.insecure_channel(target_for(parsed.host, parsed.port)) as channel:
        stub = create_stub(channel)
        enable_response = set_preview_mode(stub, enabled=True, profile_name=parsed.profile)
        print(format_preview_response(enable_response))
        if not enable_response.accepted:
            return 1
        preview_was_enabled = True

        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        try:
            if not capture.isOpened():
                print("failed to open overlay video pipeline", file=sys.stderr)
                return 1
            return _run_viewer(cv2, capture, stub, parsed)
        finally:
            capture.release()
            if parsed.mode == "display":
                cv2.destroyAllWindows()
            if preview_was_enabled and not parsed.leave_preview_running:
                disable_response = set_preview_mode(stub, enabled=False)
                print(format_preview_response(disable_response))


def main(args: list[str] | None = None) -> int:
    return run(parse_args(args))


if __name__ == "__main__":
    raise SystemExit(main())
