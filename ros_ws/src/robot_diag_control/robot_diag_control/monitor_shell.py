from __future__ import annotations

import argparse
import shlex
import sys
import time
from datetime import datetime, timezone

import grpc

from robot_diag_control import preview_viewer
from robot_diag_control.gateway_client import (
    PROFILE_TO_PROTO,
    create_stub,
    format_preview_response,
    format_system_status,
    format_system_status_summary,
    get_system_status,
    set_preview_mode,
    target_for,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal host-side monitor shell for the robot gateway"
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
    parser.add_argument("--gst-launch-path", default="gst-launch-1.0")
    parser.add_argument(
        "--poll-interval-seconds",
        type=float,
        default=1.0,
        help="default interval for the watch command",
    )
    return parser


def _help_text() -> str:
    return "\n".join(
        [
            "commands:",
            "  help",
            "  status",
            "  watch [count]",
            "  preview on [low_bw|balanced|high_quality]",
            "  preview off",
            "  viewer [preview_viewer args...]",
            "  quit",
        ]
    )


def _timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%H:%M:%S")


def _build_preview_viewer_args(
    parsed: argparse.Namespace,
    extra_args: list[str],
) -> list[str]:
    preview_host = parsed.preview_host or parsed.host
    return [
        "--host",
        parsed.host,
        "--port",
        str(parsed.port),
        "--preview-host",
        preview_host,
        "--preview-port",
        str(parsed.preview_port),
        "--preview-latency-ms",
        str(parsed.preview_latency_ms),
        "--gst-launch-path",
        parsed.gst_launch_path,
        *extra_args,
    ]


def _print_status(stub) -> None:
    print(format_system_status(get_system_status(stub)))


def _watch_status(stub, interval_seconds: float, count: int | None) -> None:
    emitted = 0
    try:
        while count is None or emitted < count:
            summary = format_system_status_summary(get_system_status(stub))
            print(f"[{_timestamp()}] {summary}")
            emitted += 1
            if count is not None and emitted >= count:
                return
            time.sleep(interval_seconds)
    except KeyboardInterrupt:
        print("watch interrupted")


def _run_shell(parsed: argparse.Namespace) -> int:
    print(f"monitor target={target_for(parsed.host, parsed.port)}")
    print(_help_text())

    with grpc.insecure_channel(target_for(parsed.host, parsed.port)) as channel:
        stub = create_stub(channel)
        while True:
            try:
                raw_line = input("robot-monitor> ")
            except EOFError:
                print()
                return 0
            except KeyboardInterrupt:
                print()
                return 0

            if not raw_line.strip():
                continue

            try:
                tokens = shlex.split(raw_line)
            except ValueError as error:
                print(f"parse error: {error}")
                continue

            command = tokens[0]
            args = tokens[1:]

            try:
                if command in {"quit", "exit"}:
                    return 0

                if command == "help":
                    print(_help_text())
                    continue

                if command == "status":
                    _print_status(stub)
                    continue

                if command == "watch":
                    count = None
                    if args:
                        if len(args) != 1:
                            print("usage: watch [count]")
                            continue
                        count = int(args[0])
                    _watch_status(stub, parsed.poll_interval_seconds, count)
                    continue

                if command == "preview":
                    if not args:
                        print("usage: preview on [profile] | preview off")
                        continue
                    mode = args[0]
                    if mode == "on":
                        profile = args[1] if len(args) > 1 else "balanced"
                        if profile not in PROFILE_TO_PROTO:
                            print(f"unknown profile: {profile}")
                            continue
                        response = set_preview_mode(
                            stub,
                            enabled=True,
                            profile_name=profile,
                        )
                        print(format_preview_response(response))
                        continue
                    if mode == "off":
                        response = set_preview_mode(stub, enabled=False)
                        print(format_preview_response(response))
                        continue
                    print("usage: preview on [profile] | preview off")
                    continue

                if command == "viewer":
                    viewer_args = _build_preview_viewer_args(parsed, args)
                    exit_code = preview_viewer.main(viewer_args)
                    print(f"viewer exited with code {exit_code}")
                    continue

                print(f"unknown command: {command}")
            except grpc.RpcError as error:
                print(f"rpc error: code={error.code().name} details={error.details()}")
            except ValueError as error:
                print(f"error: {error}")


def main(args: list[str] | None = None) -> int:
    parsed = _build_parser().parse_args(sys.argv[1:] if args is None else args)
    return _run_shell(parsed)


if __name__ == "__main__":
    raise SystemExit(main())
