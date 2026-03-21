from __future__ import annotations

import grpc

from robot_diag_control.api import robot_gateway_pb2, robot_gateway_pb2_grpc

PROFILE_TO_PROTO = {
    "low_bw": robot_gateway_pb2.PREVIEW_PROFILE_LOW_BW,
    "balanced": robot_gateway_pb2.PREVIEW_PROFILE_BALANCED,
    "high_quality": robot_gateway_pb2.PREVIEW_PROFILE_HIGH_QUALITY,
}

STATE_NAMES = {
    robot_gateway_pb2.PREVIEW_STATE_UNSPECIFIED: "unspecified",
    robot_gateway_pb2.PREVIEW_DISABLED: "disabled",
    robot_gateway_pb2.PREVIEW_RUNNING: "running",
}

PROFILE_NAMES = {
    robot_gateway_pb2.PREVIEW_PROFILE_UNSPECIFIED: "unspecified",
    robot_gateway_pb2.PREVIEW_PROFILE_LOW_BW: "low_bw",
    robot_gateway_pb2.PREVIEW_PROFILE_BALANCED: "balanced",
    robot_gateway_pb2.PREVIEW_PROFILE_HIGH_QUALITY: "high_quality",
}

HEALTH_STATE_NAMES = {
    robot_gateway_pb2.ROBOT_HEALTH_STATE_UNSPECIFIED: "unspecified",
    robot_gateway_pb2.ROBOT_HEALTH_OK: "ok",
    robot_gateway_pb2.ROBOT_HEALTH_DEGRADED: "degraded",
}


def target_for(host: str, port: int) -> str:
    return f"{host}:{port}"


def create_stub(channel: grpc.Channel) -> robot_gateway_pb2_grpc.RobotGatewayStub:
    return robot_gateway_pb2_grpc.RobotGatewayStub(channel)


def get_system_status(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
) -> robot_gateway_pb2.SystemStatus:
    return stub.GetSystemStatus(robot_gateway_pb2.GetSystemStatusRequest())


def set_preview_mode(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
    *,
    enabled: bool,
    profile_name: str | None = None,
) -> robot_gateway_pb2.SetPreviewModeResponse:
    if enabled:
        if profile_name is None:
            raise ValueError("profile_name is required when enabling preview")
        profile = PROFILE_TO_PROTO[profile_name]
    else:
        profile = robot_gateway_pb2.PREVIEW_PROFILE_UNSPECIFIED

    return stub.SetPreviewMode(
        robot_gateway_pb2.SetPreviewModeRequest(
            enabled=enabled,
            profile=profile,
        )
    )


def format_system_status(response: robot_gateway_pb2.SystemStatus) -> str:
    health = response.health
    preview = response.preview
    vision = response.vision
    lines = [
        f"gateway: {response.gateway_name} ({response.gateway_version})",
        "health:"
        f" state={HEALTH_STATE_NAMES.get(health.state, 'unknown')}"
        f" ready={str(health.ready).lower()}"
        f" summary={health.summary}",
        "mobility:"
        f" odom={'unavailable' if not health.odom_available else ('stale' if health.odom_stale else 'fresh')}"
        f" linear_speed_mps={health.linear_speed_mps:.2f}"
        f" angular_speed_rad_s={health.angular_speed_rad_s:.2f}",
        "preview:"
        f" state={STATE_NAMES.get(preview.state, 'unknown')}"
        f" profile={PROFILE_NAMES.get(preview.profile, 'unknown')}",
    ]
    if preview.last_error:
        lines.append(f"preview_error: {preview.last_error}")

    if not vision.available:
        lines.append("vision: unavailable")
        return "\n".join(lines)

    stale_suffix = " stale" if vision.stale else ""
    lines.append(
        "vision:"
        f" producer_fps={vision.producer_fps:.2f}"
        f" consumer_fps={vision.consumer_fps:.2f}"
        f" last_infer_ms={vision.last_infer_ms:.2f}"
        f" infer_errors={vision.infer_error_count}"
        f" capture_fatal_errors={vision.capture_fatal_error_count}"
        f"{stale_suffix}"
    )
    return "\n".join(lines)


def format_system_status_summary(response: robot_gateway_pb2.SystemStatus) -> str:
    health = response.health
    preview = response.preview
    vision = response.vision
    summary = (
        f"health={HEALTH_STATE_NAMES.get(health.state, 'unknown')}"
        f" ready={str(health.ready).lower()}"
        f" odom={'unavailable' if not health.odom_available else ('stale' if health.odom_stale else 'fresh')}"
        " "
        f"preview={STATE_NAMES.get(preview.state, 'unknown')}"
        f"/{PROFILE_NAMES.get(preview.profile, 'unknown')}"
    )
    if health.summary:
        summary += f" health_summary={health.summary}"
    if preview.last_error:
        summary += f" preview_error={preview.last_error}"

    if not vision.available:
        return summary + " vision=unavailable"

    freshness = "stale" if vision.stale else "fresh"
    return (
        summary
        + f" vision={freshness}"
        + f" producer_fps={vision.producer_fps:.2f}"
        + f" consumer_fps={vision.consumer_fps:.2f}"
        + f" infer_ms={vision.last_infer_ms:.2f}"
        + f" infer_errors={vision.infer_error_count}"
        + f" capture_fatal_errors={vision.capture_fatal_error_count}"
    )


def format_preview_response(response: robot_gateway_pb2.SetPreviewModeResponse) -> str:
    preview = response.preview
    return (
        f"accepted={response.accepted}"
        f" message={response.message}"
        f" state={STATE_NAMES.get(preview.state, 'unknown')}"
        f" profile={PROFILE_NAMES.get(preview.profile, 'unknown')}"
    )
