from __future__ import annotations

import argparse
import sys

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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI for the robot operator gateway")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50051)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="fetch gateway system status")

    preview_parser = subparsers.add_parser("preview", help="set preview on or off")
    preview_parser.add_argument("mode", choices=("on", "off"))
    preview_parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_TO_PROTO),
        default="balanced",
        help="preview profile to use when enabling preview",
    )
    return parser


def main(args: list[str] | None = None) -> None:
    parsed = _build_parser().parse_args(sys.argv[1:] if args is None else args)
    target = target_for(parsed.host, parsed.port)

    with grpc.insecure_channel(target) as channel:
        stub = create_stub(channel)
        if parsed.command == "status":
            print(format_system_status(get_system_status(stub)))
            return

        response = set_preview_mode(
            stub,
            enabled=parsed.mode == "on",
            profile_name=parsed.profile if parsed.mode == "on" else None,
        )
        print(format_preview_response(response))
