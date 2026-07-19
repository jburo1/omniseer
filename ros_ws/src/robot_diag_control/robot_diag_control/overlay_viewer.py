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


@dataclass
class OverlayLayers:
    perception: bool = True
    motion: bool = True
    system: bool = True
    events: bool = True


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
        "--hide-motion",
        action="store_true",
        help="start with the motion telemetry HUD layer hidden",
    )
    parser.add_argument(
        "--hide-system",
        action="store_true",
        help="start with the platform health HUD layer hidden",
    )
    parser.add_argument(
        "--hide-events",
        action="store_true",
        help="start with the operator event HUD layer hidden",
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
        if detection.track_id:
            label += f" ID:{detection.track_id}"
        label += f" {detections.age_ms}ms"
        _put_label(cv2, frame, label, (box.x1, max(16, box.y1)), color=(255, 255, 255), background=color)


def _hud_lines(
    snapshot: robot_gateway_pb2.OverlaySnapshot | None,
    *,
    layers: OverlayLayers,
    min_score: float,
) -> list[str]:
    if snapshot is None:
        return ["gateway: waiting for overlay snapshot"]

    status = snapshot.status
    detections = snapshot.detections
    vision = status.vision
    health = status.health
    preview = status.preview
    teleop = status.teleop
    platform = status.platform
    compute = platform.compute
    network = platform.network
    lipo = platform.power.lipo_battery
    onboard = platform.power.onboard_battery

    odom_state = "MISSING"
    if health.odom_available:
        odom_state = "STALE" if health.odom_stale else "OK"

    vision_state = "MISSING"
    if vision.available:
        vision_state = "STALE" if vision.stale else "OK"

    det_state = "MISSING"
    if detections.available:
        det_state = "STALE" if detections.stale else "OK"

    fault_parts: list[str] = []
    if health.state == robot_gateway_pb2.ROBOT_HEALTH_DEGRADED or not health.ready:
        fault_parts.append(health.summary or "HEALTH DEGRADED")
    if odom_state != "OK":
        fault_parts.append(f"ODOM {odom_state}")
    if vision_state != "OK":
        fault_parts.append(f"VISION {vision_state}")
    if det_state != "OK":
        fault_parts.append(f"DET {det_state}")
    if vision.capture_fatal_error_count:
        fault_parts.append(f"CAPTURE ERR {vision.capture_fatal_error_count}")
    if vision.infer_error_count:
        fault_parts.append(f"INFER ERR {vision.infer_error_count}")
    if preview.last_error:
        fault_parts.append(f"PREVIEW {preview.last_error}")
    if teleop.last_error:
        fault_parts.append(f"TELEOP {teleop.last_error}")
    if layers.system:
        if not network.available:
            fault_parts.append("Wi-Fi MISSING")
        elif network.stale:
            fault_parts.append("Wi-Fi STALE")
        elif not network.connected:
            fault_parts.append("Wi-Fi DOWN")
        elif network.wifi_signal_available and network.wifi_signal_dbm <= -75:
            fault_parts.append(f"Wi-Fi WEAK {network.wifi_signal_dbm} dBm")
        if not compute.available:
            fault_parts.append("COMPUTE MISSING")
        elif compute.stale:
            fault_parts.append("COMPUTE STALE")
        elif compute.thermal_throttled:
            fault_parts.append("CPU THROTTLING")
        elif compute.cpu_temperature_available and compute.cpu_temperature_c >= 80.0:
            fault_parts.append(f"CPU HOT {compute.cpu_temperature_c:.0f} C")
        if lipo.available and not lipo.stale and lipo.voltage_available and lipo.voltage <= 7.0:
            fault_parts.append(f"LiPo LOW {lipo.voltage:.1f} V")
        if onboard.available and not onboard.stale and onboard.percentage_available and onboard.percentage <= 20.0:
            fault_parts.append(f"ROCK BAT LOW {onboard.percentage:.0f}%")

    lines = [
        "TELEOP "
        f"{TELEOP_STATE_NAMES.get(teleop.state, 'unknown').upper()} | "
        f"{'READY' if health.ready else 'NOT READY'} | "
        f"ODOM {odom_state} {health.odom_age_ms} ms | VISION {vision_state}",
    ]
    if layers.perception:
        lines.append(
            "CAM "
            f"{vision.producer_fps:.1f} FPS | "
            f"DET {vision.consumer_fps:.1f} FPS | "
            f"LAT {vision.last_infer_ms:.0f} ms | "
            f"OBJ {detections.detection_count} | AGE {detections.age_ms} ms"
        )
    if layers.motion:
        lines.append(
            "CMD "
            f"vx {teleop.last_command_vx_mps:+.2f} "
            f"vy {teleop.last_command_vy_mps:+.2f} "
            f"wz {teleop.last_command_wz_rad_s:+.2f} | "
            "MEAS "
            f"vx {health.measured_vx_mps:+.2f} "
            f"vy {health.measured_vy_mps:+.2f} "
            f"wz {health.measured_wz_rad_s:+.2f} | "
            f"AGE {teleop.last_command_age_ms} ms"
        )
    if layers.system:
        lines.append(_platform_line(platform))
    lines.append(
        "PREVIEW "
        f"{PROFILE_NAMES.get(preview.profile, 'unknown')} | "
        f"layers={'P' if layers.perception else '-'}{'M' if layers.motion else '-'}"
        f"{'S' if layers.system else '-'}{'E' if layers.events else '-'} | "
        f"min={min_score:.2f}"
    )
    if fault_parts:
        lines.insert(0, "FAULT " + " | ".join(fault_parts))
    if layers.events:
        for event in list(snapshot.events)[-3:]:
            lines.append(f"EVENT {event.age_ms} ms {event.message}")
    return lines


def _battery_text(label: str, battery: robot_gateway_pb2.BatteryStatus) -> str:
    if not battery.available:
        return f"{label} --"
    if battery.stale:
        return f"{label} STALE"
    if battery.voltage_available:
        return f"{label} {battery.voltage:.1f}V"
    if battery.percentage_available:
        return f"{label} {battery.percentage:.0f}%"
    return f"{label} OK"


def _platform_line(platform: robot_gateway_pb2.PlatformStatus) -> str:
    compute = platform.compute
    network = platform.network
    lipo = platform.power.lipo_battery
    onboard = platform.power.onboard_battery

    wifi = "Wi-Fi --"
    if network.available:
        if network.stale:
            wifi = "Wi-Fi STALE"
        elif network.wifi_signal_available:
            wifi = f"Wi-Fi {network.wifi_signal_dbm} dBm"
        elif network.link_quality_available:
            wifi = f"Wi-Fi {network.link_quality_percent}%"
        else:
            wifi = f"Wi-Fi {network.interface_name or 'up'}"

    cpu = "CPU --"
    ram = "RAM --"
    if compute.available:
        cpu = f"CPU {compute.cpu_percent:.0f}%"
        if compute.cpu_temperature_available:
            cpu += f" {compute.cpu_temperature_c:.0f} C"
        if compute.thermal_throttled:
            cpu += " THROTTLE"
        if compute.ram_total_bytes:
            ram = f"RAM {compute.ram_used_bytes / (1024**3):.1f}/{compute.ram_total_bytes / (1024**3):.0f}G"

    onboard_text = _battery_text("ROCK", onboard)
    if onboard.available and not onboard.stale and onboard.percentage_available:
        onboard_text = f"ROCK {onboard.percentage:.0f}%"
    return f"PWR {_battery_text('LiPo', lipo)} | {onboard_text} | {wifi} | {cpu} | {ram}"


def _draw_hud(
    cv2: Any,
    frame: Any,
    snapshot: robot_gateway_pb2.OverlaySnapshot | None,
    *,
    layers: OverlayLayers,
    min_score: float,
) -> None:
    lines = _hud_lines(snapshot, layers=layers, min_score=min_score)

    x = 10
    y = 24
    for line in lines:
        background = (45, 45, 45)
        if line.startswith("FAULT"):
            background = (35, 35, 180)
        elif line.startswith("EVENT"):
            background = (35, 70, 70)
        _put_label(cv2, frame, line, (x, y), color=(245, 245, 245), background=background)
        y += 24


def _import_cv2() -> Any:
    try:
        import cv2  # type: ignore[import-not-found]
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "OpenCV is not available. Install a GUI-capable OpenCV package on the laptop (for Ubuntu: python3-opencv)."
        ) from error
    return cv2


def _opencv_gstreamer_enabled(cv2: Any) -> bool | None:
    get_build_information = getattr(cv2, "getBuildInformation", None)
    if not callable(get_build_information):
        return None

    try:
        build_information = get_build_information()
    except Exception:
        return None

    for line in str(build_information).splitlines():
        stripped = line.strip()
        if not stripped.startswith("GStreamer:"):
            continue
        value = stripped.split(":", maxsplit=1)[1].strip().lower()
        if value.startswith("yes"):
            return True
        if value.startswith("no"):
            return False

    return None


def _validate_opencv_videoio_support(cv2: Any) -> None:
    if _opencv_gstreamer_enabled(cv2) is not False:
        return

    raise RuntimeError(
        "OpenCV is installed, but it was built without GStreamer video I/O support. "
        "The overlay viewer needs cv2.VideoCapture(..., cv2.CAP_GSTREAMER) to decode "
        "the SRT preview into frames. On Ubuntu, install the distro OpenCV package "
        "(python3-opencv) and make sure a pip opencv-python wheel is not shadowing it. "
        "For a non-overlay preview path, use robot_preview_viewer."
    )


def _format_pipeline_open_error(pipeline: str) -> str:
    return "\n".join(
        [
            "failed to open overlay video pipeline",
            f"pipeline: {pipeline}",
            "Check that the laptop can reach the robot's SRT preview port, that local firewall rules allow it, "
            "and that Python OpenCV was built with GStreamer support.",
            "For transport-only validation, run robot_preview_viewer with --mode=fakesink and a short duration.",
        ]
    )


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
    layers = OverlayLayers(
        perception=True,
        motion=not parsed.hide_motion,
        system=not parsed.hide_system,
        events=not parsed.hide_events,
    )
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

        if layers.perception and last_snapshot is not None:
            _draw_detections(cv2, frame, last_snapshot, min_score=min_score)
        _draw_hud(cv2, frame, last_snapshot, layers=layers, min_score=min_score)

        if parsed.mode == "fakesink":
            continue

        cv2.imshow("Omniseer Overlay Viewer", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in {27, ord("q"), ord("x")}:
            return 0
        if key == ord("o") or key == ord("p"):
            layers.perception = not layers.perception
        elif key == ord("m"):
            layers.motion = not layers.motion
        elif key == ord("s"):
            layers.system = not layers.system
        elif key == ord("e"):
            layers.events = not layers.events
        elif key == ord("["):
            min_score = max(0.0, min_score - 0.05)
        elif key == ord("]"):
            min_score = min(1.0, min_score + 0.05)


def run(parsed: argparse.Namespace) -> int:
    decoder_element = _validate_gstreamer_support(parsed)
    pipeline = _build_appsink_pipeline(parsed, decoder_element=decoder_element)
    cv2 = _import_cv2()
    _validate_opencv_videoio_support(cv2)
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
                print(_format_pipeline_open_error(pipeline), file=sys.stderr)
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
    try:
        return run(parse_args(args))
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
