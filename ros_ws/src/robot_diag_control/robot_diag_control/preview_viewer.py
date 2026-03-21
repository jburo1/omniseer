from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import grpc

from robot_diag_control.gateway_client import (
    PROFILE_TO_PROTO,
    create_stub,
    format_preview_response,
    format_system_status,
    get_system_status,
    set_preview_mode,
    target_for,
)

_H264_DECODER_CANDIDATES = (
    "avdec_h264",
    "openh264dec",
    "vah264dec",
    "v4l2slh264dec",
    "decodebin",
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal host-side preview helper for the robot gateway"
    )
    parser.add_argument("--host", default="127.0.0.1", help="gRPC gateway host")
    parser.add_argument("--port", type=int, default=50051, help="gRPC gateway port")
    parser.add_argument(
        "--preview-host",
        default=None,
        help="preview stream host; defaults to --host",
    )
    parser.add_argument("--preview-port", type=int, default=7001)
    parser.add_argument("--preview-latency-ms", type=int, default=125)
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_TO_PROTO),
        default="balanced",
        help="preview profile to request before playback",
    )
    parser.add_argument(
        "--mode",
        choices=("display", "fakesink", "capture_ts"),
        default="display",
        help="how to consume the preview stream on the laptop",
    )
    parser.add_argument(
        "--output-path",
        default=None,
        help="required when --mode=capture_ts",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        default=None,
        help="optional playback duration; useful for headless smoke tests",
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


def _resolve_gst_inspect_path(gst_launch_path: str) -> str:
    launch_path = shutil.which(gst_launch_path) or gst_launch_path
    launch_binary = Path(launch_path)
    sibling_inspect = launch_binary.with_name("gst-inspect-1.0")
    if sibling_inspect.exists():
        return str(sibling_inspect)

    inspect_path = shutil.which("gst-inspect-1.0")
    if inspect_path:
        return inspect_path

    raise RuntimeError("gst-inspect-1.0 not found; install GStreamer tools on the laptop")


def _gst_element_available(gst_inspect_path: str, element_name: str) -> bool:
    result = subprocess.run(
        [gst_inspect_path, element_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _select_h264_decoder_element(gst_inspect_path: str) -> str | None:
    for decoder_name in _H264_DECODER_CANDIDATES:
        if _gst_element_available(gst_inspect_path, decoder_name):
            return decoder_name
    return None


def _validate_gstreamer_support(parsed: argparse.Namespace) -> str:
    gst_inspect_path = _resolve_gst_inspect_path(parsed.gst_launch_path)
    if not _gst_element_available(gst_inspect_path, "srtsrc"):
        raise RuntimeError(
            "GStreamer element 'srtsrc' is unavailable; install the SRT plugin "
            "(for Ubuntu: gstreamer1.0-plugins-bad)"
        )

    decoder_element = _select_h264_decoder_element(gst_inspect_path)
    if decoder_element is None:
        raise RuntimeError(
            "No supported H.264 decoder is available; install either GStreamer libav "
            "(for Ubuntu: gstreamer1.0-libav) or a plugin set that provides an H.264 decoder"
        )

    return decoder_element


def _build_player_command(
    parsed: argparse.Namespace,
    *,
    decoder_element: str = "avdec_h264",
) -> list[str]:
    preview_host = parsed.preview_host or parsed.host
    command = [
        parsed.gst_launch_path,
        "-q",
        "srtsrc",
        f"uri=srt://{preview_host}:{parsed.preview_port}?mode=caller",
        f"latency={parsed.preview_latency_ms}",
        "wait-for-connection=false",
    ]

    if parsed.mode == "capture_ts":
        if not parsed.output_path:
            raise ValueError("--output-path is required when --mode=capture_ts")
        output_path = Path(parsed.output_path)
        command.extend(
            [
                "!",
                "filesink",
                f"location={output_path}",
            ]
        )
        return command

    command.extend(
        [
            "!",
            "tsdemux",
            "!",
            "h264parse",
            "!",
            decoder_element,
            "!",
            "videoconvert",
            "!",
            "fakesink" if parsed.mode == "fakesink" else "autovideosink",
            "sync=false",
        ]
    )
    return command


def _run_player(command: list[str], duration_seconds: float | None) -> int:
    process = subprocess.Popen(command)
    try:
        if duration_seconds is None:
            return process.wait()

        try:
            return process.wait(timeout=duration_seconds)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
                return 0
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
                return 0
    except KeyboardInterrupt:
        process.terminate()
        try:
            process.wait(timeout=2.0)
            return 0
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
            return 0


def run(parsed: argparse.Namespace) -> int:
    decoder_element = _validate_gstreamer_support(parsed)
    player_command = _build_player_command(parsed, decoder_element=decoder_element)
    preview_was_enabled = False

    with grpc.insecure_channel(target_for(parsed.host, parsed.port)) as channel:
        stub = create_stub(channel)
        enable_response = set_preview_mode(
            stub,
            enabled=True,
            profile_name=parsed.profile,
        )
        print(format_preview_response(enable_response))
        if not enable_response.accepted:
            return 1
        preview_was_enabled = True

        print(format_system_status(get_system_status(stub)))

        try:
            return _run_player(player_command, parsed.duration_seconds)
        finally:
            if preview_was_enabled and not parsed.leave_preview_running:
                disable_response = set_preview_mode(stub, enabled=False)
                print(format_preview_response(disable_response))


def main(args: list[str] | None = None) -> int:
    return run(parse_args(args))


if __name__ == "__main__":
    raise SystemExit(main())
