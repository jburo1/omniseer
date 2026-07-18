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

TELEOP_STATE_NAMES = {
    robot_gateway_pb2.TELEOP_STATE_UNSPECIFIED: "unspecified",
    robot_gateway_pb2.TELEOP_DISABLED: "disabled",
    robot_gateway_pb2.TELEOP_ENABLED: "enabled",
    robot_gateway_pb2.TELEOP_TIMED_OUT: "timed_out",
}


def target_for(host: str, port: int) -> str:
    return f"{host}:{port}"


def create_stub(channel: grpc.Channel) -> robot_gateway_pb2_grpc.RobotGatewayStub:
    return robot_gateway_pb2_grpc.RobotGatewayStub(channel)


def get_system_status(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
) -> robot_gateway_pb2.SystemStatus:
    return stub.GetSystemStatus(robot_gateway_pb2.GetSystemStatusRequest())


def get_overlay_snapshot(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
) -> robot_gateway_pb2.OverlaySnapshot:
    return stub.GetOverlaySnapshot(robot_gateway_pb2.GetOverlaySnapshotRequest())


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


def set_teleop_enabled(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
    *,
    enabled: bool,
) -> robot_gateway_pb2.SetTeleopEnabledResponse:
    return stub.SetTeleopEnabled(robot_gateway_pb2.SetTeleopEnabledRequest(enabled=enabled))


def send_teleop_command(
    stub: robot_gateway_pb2_grpc.RobotGatewayStub,
    *,
    linear_x_mps: float = 0.0,
    linear_y_mps: float = 0.0,
    angular_z_rad_s: float = 0.0,
) -> robot_gateway_pb2.SendTeleopCommandResponse:
    return stub.SendTeleopCommand(
        robot_gateway_pb2.SendTeleopCommandRequest(
            linear_x_mps=linear_x_mps,
            linear_y_mps=linear_y_mps,
            angular_z_rad_s=angular_z_rad_s,
        )
    )


def format_system_status(response: robot_gateway_pb2.SystemStatus) -> str:
    health = response.health
    preview = response.preview
    vision = response.vision
    teleop = response.teleop
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
        "teleop:"
        f" state={TELEOP_STATE_NAMES.get(teleop.state, 'unknown')}"
        f" enabled={str(teleop.enabled).lower()}"
        f" timed_out={str(teleop.timed_out).lower()}"
        f" last_command_age_ms={teleop.last_command_age_ms}"
        f" bounds=linear:{teleop.max_linear_mps:.2f}mps angular:{teleop.max_angular_rad_s:.2f}radps",
    ]
    if preview.last_error:
        lines.append(f"preview_error: {preview.last_error}")
    if teleop.last_error:
        lines.append(f"teleop_error: {teleop.last_error}")

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
    teleop = response.teleop
    summary = (
        f"health={HEALTH_STATE_NAMES.get(health.state, 'unknown')}"
        f" ready={str(health.ready).lower()}"
        f" odom={'unavailable' if not health.odom_available else ('stale' if health.odom_stale else 'fresh')}"
        " "
        f"preview={STATE_NAMES.get(preview.state, 'unknown')}"
        f"/{PROFILE_NAMES.get(preview.profile, 'unknown')}"
        " "
        f"teleop={TELEOP_STATE_NAMES.get(teleop.state, 'unknown')}"
    )
    if health.summary:
        summary += f" health_summary={health.summary}"
    if preview.last_error:
        summary += f" preview_error={preview.last_error}"
    if teleop.last_error:
        summary += f" teleop_error={teleop.last_error}"

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


def format_teleop_response(
    response: robot_gateway_pb2.SetTeleopEnabledResponse | robot_gateway_pb2.SendTeleopCommandResponse,
) -> str:
    teleop = response.teleop
    return (
        f"accepted={response.accepted}"
        f" message={response.message}"
        f" state={TELEOP_STATE_NAMES.get(teleop.state, 'unknown')}"
        f" enabled={str(teleop.enabled).lower()}"
    )


def format_overlay_snapshot(response: robot_gateway_pb2.OverlaySnapshot) -> str:
    lines = [format_system_status_summary(response.status)]
    detections = response.detections
    if not detections.available:
        lines.append(
            "detections:"
            f" unavailable source={detections.source_width_px}x{detections.source_height_px}"
        )
        return "\n".join(lines)

    freshness = "stale" if detections.stale else "fresh"
    lines.append(
        "detections:"
        f" {freshness}"
        f" age_ms={detections.age_ms}"
        f" count={detections.detection_count}"
        f" source={detections.source_width_px}x{detections.source_height_px}"
    )
    for detection in detections.detections:
        left = detection.bbox_center_x_px - detection.bbox_width_px / 2.0
        top = detection.bbox_center_y_px - detection.bbox_height_px / 2.0
        lines.append(
            "  -"
            f" class={detection.class_name or detection.class_id}"
            f" score={detection.score:.2f}"
            f" bbox=({left:.1f},{top:.1f},{detection.bbox_width_px:.1f},{detection.bbox_height_px:.1f})"
        )
    return "\n".join(lines)
