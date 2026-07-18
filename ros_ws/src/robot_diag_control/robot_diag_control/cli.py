from __future__ import annotations

import argparse
import sys

import grpc

from robot_diag_control.gateway_client import (
    PROFILE_TO_PROTO,
    create_stub,
    format_overlay_snapshot,
    format_preview_response,
    format_system_status,
    format_teleop_response,
    get_overlay_snapshot,
    get_system_status,
    send_teleop_command,
    set_preview_mode,
    set_teleop_enabled,
    target_for,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI for the robot operator gateway")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50051)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="fetch gateway system status")
    subparsers.add_parser("overlay", help="fetch latest gateway overlay snapshot")

    preview_parser = subparsers.add_parser("preview", help="set preview on or off")
    preview_parser.add_argument("mode", choices=("on", "off"))
    preview_parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_TO_PROTO),
        default="balanced",
        help="preview profile to use when enabling preview",
    )

    teleop_parser = subparsers.add_parser("teleop", help="set teleop on/off or send one command")
    teleop_subparsers = teleop_parser.add_subparsers(dest="mode", required=True)
    teleop_subparsers.add_parser("on", help="enable gateway teleop")
    teleop_subparsers.add_parser("off", help="disable gateway teleop and stop")
    command_parser = teleop_subparsers.add_parser("cmd", help="send one bounded teleop command")
    command_parser.add_argument("--linear-x-mps", type=float, default=0.0)
    command_parser.add_argument("--linear-y-mps", type=float, default=0.0)
    command_parser.add_argument("--angular-z-rad-s", type=float, default=0.0)
    teleop_subparsers.add_parser("stop", help="send a zero teleop command")
    return parser


def main(args: list[str] | None = None) -> None:
    parsed = _build_parser().parse_args(sys.argv[1:] if args is None else args)
    target = target_for(parsed.host, parsed.port)

    with grpc.insecure_channel(target) as channel:
        stub = create_stub(channel)
        if parsed.command == "status":
            print(format_system_status(get_system_status(stub)))
            return

        if parsed.command == "overlay":
            print(format_overlay_snapshot(get_overlay_snapshot(stub)))
            return

        if parsed.command == "teleop":
            if parsed.mode == "on":
                print(format_teleop_response(set_teleop_enabled(stub, enabled=True)))
                return
            if parsed.mode == "off":
                print(format_teleop_response(set_teleop_enabled(stub, enabled=False)))
                return
            if parsed.mode == "stop":
                print(format_teleop_response(send_teleop_command(stub)))
                return
            print(
                format_teleop_response(
                    send_teleop_command(
                        stub,
                        linear_x_mps=parsed.linear_x_mps,
                        linear_y_mps=parsed.linear_y_mps,
                        angular_z_rad_s=parsed.angular_z_rad_s,
                    )
                )
            )
            return

        response = set_preview_mode(
            stub,
            enabled=parsed.mode == "on",
            profile_name=parsed.profile if parsed.mode == "on" else None,
        )
        print(format_preview_response(response))
