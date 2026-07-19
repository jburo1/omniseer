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


def _odom_freshness(health: robot_gateway_pb2.RobotHealth) -> str:
    if not health.odom_available:
        return "MISSING"
    if health.odom_stale:
        return "STALE"
    return "OK"


def _vision_freshness(vision: robot_gateway_pb2.VisionStatus) -> str:
    if not vision.available:
        return "MISSING"
    if vision.stale:
        return "STALE"
    return "OK"


def _preview_state(preview: robot_gateway_pb2.PreviewStatus) -> str:
    return STATE_NAMES.get(preview.state, "unknown").upper()


def _teleop_state(teleop: robot_gateway_pb2.TeleopStatus) -> str:
    return TELEOP_STATE_NAMES.get(teleop.state, "unknown").upper()


def _operator_faults(response: robot_gateway_pb2.SystemStatus) -> list[str]:
    faults: list[str] = []
    health = response.health
    preview = response.preview
    vision = response.vision
    teleop = response.teleop
    platform = response.platform
    compute = platform.compute
    network = platform.network
    lipo = platform.power.lipo_battery
    onboard = platform.power.onboard_battery

    if health.state == robot_gateway_pb2.ROBOT_HEALTH_DEGRADED or not health.ready:
        faults.append(health.summary or "robot health degraded")
    if not health.odom_available:
        faults.append("ODOMETRY MISSING")
    elif health.odom_stale:
        faults.append("ODOMETRY STALE")
    if not vision.available:
        faults.append("VISION MISSING")
    elif vision.stale:
        faults.append("VISION STALE")
    if vision.capture_fatal_error_count > 0:
        faults.append(f"CAMERA CAPTURE FATALS {vision.capture_fatal_error_count}")
    if vision.infer_error_count > 0:
        faults.append(f"INFERENCE ERRORS {vision.infer_error_count}")
    if preview.last_error:
        faults.append(f"PREVIEW ERROR: {preview.last_error}")
    if teleop.last_error:
        faults.append(f"TELEOP ERROR: {teleop.last_error}")
    if not network.available:
        faults.append("Wi-Fi MISSING")
    elif network.stale:
        faults.append("Wi-Fi STALE")
    elif not network.connected:
        faults.append("Wi-Fi DOWN")
    elif network.wifi_signal_available and network.wifi_signal_dbm <= -75:
        faults.append(f"Wi-Fi WEAK {network.wifi_signal_dbm} dBm")
    if not compute.available:
        faults.append("COMPUTE MISSING")
    elif compute.stale:
        faults.append("COMPUTE STALE")
    elif compute.thermal_throttled:
        faults.append("CPU THROTTLING")
    elif compute.cpu_temperature_available and compute.cpu_temperature_c >= 80.0:
        faults.append(f"CPU HOT {compute.cpu_temperature_c:.0f} C")
    if lipo.available and not lipo.stale and lipo.voltage_available and lipo.voltage <= 7.0:
        faults.append(f"LiPo LOW {lipo.voltage:.1f} V")
    if onboard.available and not onboard.stale and onboard.percentage_available and onboard.percentage <= 20.0:
        faults.append(f"ROCK BAT LOW {onboard.percentage:.0f}%")

    return faults


def _format_battery(label: str, battery: robot_gateway_pb2.BatteryStatus) -> str:
    if not battery.available:
        return f"{label}=unavailable"
    freshness = "stale" if battery.stale else "fresh"
    parts = [f"{label}={freshness}", f"present={str(battery.present).lower()}"]
    if battery.voltage_available:
        parts.append(f"voltage={battery.voltage:.2f}V")
    if battery.percentage_available:
        parts.append(f"percentage={battery.percentage:.0f}%")
    if battery.charging_available:
        parts.append(f"charging={str(battery.charging).lower()}")
    return " ".join(parts)


def _format_platform_status(platform: robot_gateway_pb2.PlatformStatus) -> list[str]:
    compute = platform.compute
    network = platform.network
    lines: list[str] = []

    if compute.available:
        freshness = "stale" if compute.stale else "fresh"
        line = (
            "compute:"
            f" {freshness}"
            f" age_ms={compute.age_ms}"
            f" cpu_percent={compute.cpu_percent:.1f}"
            f" ram={compute.ram_used_bytes}/{compute.ram_total_bytes}"
            f" ram_percent={compute.ram_used_percent:.1f}"
        )
        if compute.cpu_temperature_available:
            line += f" cpu_temperature_c={compute.cpu_temperature_c:.1f}"
        if compute.thermal_throttled_available:
            line += f" thermal_throttled={str(compute.thermal_throttled).lower()}"
        if compute.disk_available:
            line += f" disk_percent={compute.disk_used_percent:.1f}"
        lines.append(line)
    else:
        lines.append("compute: unavailable")

    if network.available:
        freshness = "stale" if network.stale else "fresh"
        line = (
            "network:"
            f" {freshness}"
            f" age_ms={network.age_ms}"
            f" connected={str(network.connected).lower()}"
            f" interface={network.interface_name}"
        )
        if network.wifi_signal_available:
            line += f" wifi_signal_dbm={network.wifi_signal_dbm}"
        if network.link_quality_available:
            line += f" link_quality_percent={network.link_quality_percent}"
        lines.append(line)
    else:
        lines.append("network: unavailable")

    lines.append("power: " + _format_battery("lipo", platform.power.lipo_battery))
    lines.append("power: " + _format_battery("onboard", platform.power.onboard_battery))
    return lines


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
    platform = response.platform
    lines = [
        f"gateway: {response.gateway_name} ({response.gateway_version})",
        "health:"
        f" state={HEALTH_STATE_NAMES.get(health.state, 'unknown')}"
        f" ready={str(health.ready).lower()}"
        f" summary={health.summary}",
        "mobility:"
        f" odom={'unavailable' if not health.odom_available else ('stale' if health.odom_stale else 'fresh')}"
        f" odom_age_ms={health.odom_age_ms}"
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
        f" last_command=({teleop.last_command_vx_mps:.2f},{teleop.last_command_vy_mps:.2f},"
        f"{teleop.last_command_wz_rad_s:.2f})"
        f" bounds=linear:{teleop.max_linear_mps:.2f}mps angular:{teleop.max_angular_rad_s:.2f}radps",
    ]
    if preview.last_error:
        lines.append(f"preview_error: {preview.last_error}")
    if teleop.last_error:
        lines.append(f"teleop_error: {teleop.last_error}")

    lines.extend(_format_platform_status(platform))

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


def format_operator_status(response: robot_gateway_pb2.SystemStatus) -> str:
    health = response.health
    preview = response.preview
    vision = response.vision
    teleop = response.teleop
    platform = response.platform

    top_strip = (
        f"TELEOP {_teleop_state(teleop)} | "
        f"{'READY' if health.ready else 'NOT READY'} | "
        f"ODOM {_odom_freshness(health)} {health.odom_age_ms} ms | "
        f"VISION {_vision_freshness(vision)} | "
        f"PREVIEW {_preview_state(preview)}"
    )
    perception_strip = (
        "CAM "
        f"{vision.producer_fps:.1f} FPS | "
        f"DET {vision.consumer_fps:.1f} FPS | "
        f"LAT {vision.last_infer_ms:.0f} ms | "
        f"ERR infer={vision.infer_error_count} capture={vision.capture_fatal_error_count}"
        if vision.available
        else "CAM -- FPS | DET -- FPS | LAT -- ms | VISION MISSING"
    )
    motion_strip = (
        f"CMD vx {teleop.last_command_vx_mps:+.2f} vy {teleop.last_command_vy_mps:+.2f} "
        f"wz {teleop.last_command_wz_rad_s:+.2f} | "
        f"MEAS vx {health.measured_vx_mps:+.2f} vy {health.measured_vy_mps:+.2f} "
        f"wz {health.measured_wz_rad_s:+.2f} | "
        f"AGE {teleop.last_command_age_ms} ms"
    )
    bounds_strip = (
        f"BOUNDS vx <= {teleop.max_linear_mps:.2f} m/s | "
        f"wz <= {teleop.max_angular_rad_s:.2f} rad/s | "
        f"PROFILE {PROFILE_NAMES.get(preview.profile, 'unknown')}"
    )
    platform_strip = _operator_platform_strip(platform)

    faults = _operator_faults(response)
    fault_line = "FAULT none" if not faults else "FAULT " + " | ".join(faults)
    return "\n".join([top_strip, perception_strip, motion_strip, platform_strip, bounds_strip, fault_line])


def _operator_platform_strip(platform: robot_gateway_pb2.PlatformStatus) -> str:
    compute = platform.compute
    network = platform.network
    lipo = platform.power.lipo_battery
    onboard = platform.power.onboard_battery

    lipo_text = "LiPo --"
    if lipo.available:
        lipo_text = "LiPo STALE" if lipo.stale else "LiPo OK"
        if not lipo.stale and lipo.voltage_available:
            lipo_text = f"LiPo {lipo.voltage:.1f} V"

    onboard_text = "ROCK --"
    if onboard.available:
        onboard_text = "ROCK STALE" if onboard.stale else "ROCK OK"
        if not onboard.stale and onboard.percentage_available:
            onboard_text = f"ROCK {onboard.percentage:.0f}%"

    wifi_text = "Wi-Fi --"
    if network.available:
        wifi_text = "Wi-Fi STALE" if network.stale else f"Wi-Fi {network.interface_name}"
        if not network.stale and network.wifi_signal_available:
            wifi_text = f"Wi-Fi {network.wifi_signal_dbm} dBm"

    cpu_text = "CPU --"
    ram_text = "RAM --"
    if compute.available:
        cpu_text = f"CPU {compute.cpu_percent:.0f}%"
        if compute.cpu_temperature_available:
            cpu_text += f" {compute.cpu_temperature_c:.0f} C"
        if compute.thermal_throttled:
            cpu_text += " THROTTLE"
        if compute.ram_total_bytes:
            ram_text = f"RAM {compute.ram_used_bytes / (1024**3):.1f}/{compute.ram_total_bytes / (1024**3):.0f} GB"

    return f"PWR {lipo_text} | {onboard_text} | {wifi_text} | {cpu_text} | {ram_text}"


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
        lines.append(f"detections: unavailable source={detections.source_width_px}x{detections.source_height_px}")
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
    for event in response.events:
        lines.append(f"event: seq={event.sequence} age_ms={event.age_ms} message={event.message}")
    return "\n".join(lines)
