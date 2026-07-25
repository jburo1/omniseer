from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MonitorConnectionValues:
    host: str
    port: str
    preview_host: str
    preview_port: str
    preview_latency_ms: str
    gst_launch_path: str
    poll_interval_seconds: str


@dataclass(frozen=True)
class MonitorConnectionSettings:
    host: str
    port: int
    preview_host: str | None
    preview_port: int
    preview_latency_ms: int
    gst_launch_path: str
    poll_interval_seconds: float


@dataclass(frozen=True)
class TeleopStepValues:
    linear_step_mps: str
    angular_step_rad_s: str


@dataclass(frozen=True)
class TeleopStepSettings:
    linear_step_mps: float
    angular_step_rad_s: float


def resolve_monitor_connection(values: MonitorConnectionValues) -> MonitorConnectionSettings:
    return MonitorConnectionSettings(
        host=values.host.strip(),
        port=int(values.port.strip()),
        preview_host=values.preview_host.strip() or None,
        preview_port=int(values.preview_port.strip()),
        preview_latency_ms=int(values.preview_latency_ms.strip()),
        gst_launch_path=values.gst_launch_path.strip(),
        poll_interval_seconds=float(values.poll_interval_seconds.strip()),
    )


def resolve_teleop_steps(values: TeleopStepValues) -> TeleopStepSettings:
    return TeleopStepSettings(
        linear_step_mps=float(values.linear_step_mps.strip()),
        angular_step_rad_s=float(values.angular_step_rad_s.strip()),
    )
