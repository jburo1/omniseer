"""Low-rate Linux system telemetry sampling for perception run bundles."""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omniseer_experiments.bundle import make_system_record

DEFAULT_PROC_STAT = Path("/proc/stat")
DEFAULT_PROC_MEMINFO = Path("/proc/meminfo")
DEFAULT_PROC_NET_WIRELESS = Path("/proc/net/wireless")
DEFAULT_SYS_ROOT = Path("/sys")
DEFAULT_THERMAL_ROOT = Path("/sys/class/thermal")

TimeNs = Callable[[], int]


@dataclass(frozen=True)
class CpuSample:
    idle: int
    total: int


@dataclass(frozen=True)
class MemorySample:
    used_mb: float
    available_mb: float


def parse_proc_stat_cpu(text: str) -> CpuSample | None:
    """Parse the aggregate cpu line from /proc/stat."""

    for line in text.splitlines():
        fields = line.split()
        if not fields or fields[0] != "cpu":
            continue
        try:
            values = [int(value) for value in fields[1:]]
        except ValueError:
            return None
        if len(values) < 4:
            return None

        user = values[0]
        nice = values[1]
        system = values[2]
        idle = values[3]
        iowait = values[4] if len(values) > 4 else 0
        irq = values[5] if len(values) > 5 else 0
        softirq = values[6] if len(values) > 6 else 0
        steal = values[7] if len(values) > 7 else 0
        idle_all = idle + iowait
        non_idle = user + nice + system + irq + softirq + steal
        return CpuSample(idle=idle_all, total=idle_all + non_idle)
    return None


def cpu_percent(previous: CpuSample | None, current: CpuSample | None) -> float:
    if previous is None or current is None:
        return 0.0
    total_delta = current.total - previous.total
    idle_delta = current.idle - previous.idle
    if total_delta <= 0 or idle_delta < 0:
        return 0.0
    busy_delta = max(0, total_delta - idle_delta)
    return busy_delta * 100.0 / total_delta


def parse_proc_meminfo(text: str) -> MemorySample | None:
    values: dict[str, int] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        parts = raw_value.split()
        if not parts:
            continue
        try:
            values[key] = int(parts[0])
        except ValueError:
            continue

    total_kb = values.get("MemTotal")
    if total_kb is None or total_kb <= 0:
        return None

    available_kb = values.get("MemAvailable")
    if available_kb is None:
        free_kb = values.get("MemFree", 0)
        buffers_kb = values.get("Buffers", 0)
        cached_kb = values.get("Cached", 0)
        reclaimable_kb = values.get("SReclaimable", 0)
        available_kb = free_kb + buffers_kb + cached_kb + reclaimable_kb

    available_kb = max(0, min(total_kb, available_kb))
    used_kb = max(0, total_kb - available_kb)
    return MemorySample(used_mb=used_kb / 1024.0, available_mb=available_kb / 1024.0)


def read_temperature_c(paths: Sequence[Path]) -> float | None:
    for path in paths:
        try:
            raw = path.read_text(encoding="utf-8").strip()
        except OSError:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        if abs(value) >= 1000.0:
            value /= 1000.0
        return value
    return None


def default_temperature_paths(root: Path = DEFAULT_THERMAL_ROOT) -> tuple[Path, ...]:
    try:
        return tuple(sorted(root.glob("thermal_zone*/temp")))
    except OSError:
        return ()


def read_first_line(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").splitlines()[0].strip()
    except (IndexError, OSError):
        return None


def read_float(path: Path) -> float | None:
    raw = read_first_line(path)
    if raw is None:
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def read_int(path: Path) -> int | None:
    raw = read_first_line(path)
    if raw is None:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def read_thermal_throttled(sys_root: Path = DEFAULT_SYS_ROOT) -> bool | None:
    value = read_int(sys_root / "devices/system/cpu/cpu0/cpufreq/throttle_stats/throttled_time")
    return None if value is None else value > 0


def read_thermal_snapshot(root: Path = DEFAULT_THERMAL_ROOT, *, sys_root: Path = DEFAULT_SYS_ROOT) -> dict[str, Any]:
    zones: list[dict[str, Any]] = []
    try:
        entries = tuple(sorted(root.glob("thermal_zone*")))
    except OSError:
        entries = ()
    for entry in entries:
        temp_c = read_temperature_c([entry / "temp"])
        if temp_c is None:
            continue
        zones.append(
            {
                "name": entry.name,
                "type": read_first_line(entry / "type") or "",
                "temp_c": temp_c,
            }
        )
    hottest = max((zone["temp_c"] for zone in zones), default=None)
    return {
        "available": bool(zones),
        "soc_temp_c": hottest,
        "throttled": read_thermal_throttled(sys_root),
        "zones": zones,
    }


def select_wireless_interface(sys_root: Path = DEFAULT_SYS_ROOT) -> str | None:
    network_root = sys_root / "class/net"
    try:
        entries = tuple(sorted(network_root.iterdir()))
    except OSError:
        return None
    for entry in entries:
        if not (entry / "wireless").exists():
            continue
        operstate = read_first_line(entry / "operstate") or ""
        if operstate in {"up", "unknown"}:
            return entry.name
    return None


def read_wireless_stats(path: Path, interface_name: str) -> tuple[int | None, int | None]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None, None
    for line in lines:
        if f"{interface_name}:" not in line:
            continue
        parts = line.split(":", 1)[1].split()
        if len(parts) < 3:
            return None, None
        try:
            link = float(parts[1].rstrip("."))
            level = float(parts[2].rstrip("."))
        except ValueError:
            return None, None
        link_quality = round(max(0.0, min(100.0, link * 100.0 / 70.0)))
        return round(level), link_quality
    return None, None


def read_network_snapshot(
    *,
    sys_root: Path = DEFAULT_SYS_ROOT,
    proc_net_wireless_path: Path = DEFAULT_PROC_NET_WIRELESS,
) -> dict[str, Any]:
    interface_name = select_wireless_interface(sys_root)
    if interface_name is None:
        return {
            "available": False,
            "connected": False,
            "interface": "",
            "wifi_signal_dbm": None,
            "link_quality_percent": None,
        }

    signal_dbm, link_quality = read_wireless_stats(proc_net_wireless_path, interface_name)
    return {
        "available": True,
        "connected": True,
        "interface": interface_name,
        "wifi_signal_dbm": signal_dbm,
        "link_quality_percent": link_quality,
    }


def read_onboard_battery_snapshot(sys_root: Path = DEFAULT_SYS_ROOT) -> dict[str, Any]:
    battery_root = sys_root / "class/power_supply"
    try:
        entries = tuple(sorted(battery_root.iterdir()))
    except OSError:
        return _unavailable_battery_snapshot()
    for entry in entries:
        if (read_first_line(entry / "type") or "").lower() != "battery":
            continue
        voltage_raw = read_float(entry / "voltage_now")
        capacity = read_float(entry / "capacity")
        status = read_first_line(entry / "status")
        return {
            "available": True,
            "source": entry.name,
            "present": (read_int(entry / "present") or 1) != 0,
            "voltage": voltage_raw / 1_000_000.0 if voltage_raw is not None else None,
            "percentage": capacity,
            "charging": status.lower() == "charging" if status else None,
        }
    return _unavailable_battery_snapshot()


def _unavailable_battery_snapshot() -> dict[str, Any]:
    return {
        "available": False,
        "source": "",
        "present": False,
        "voltage": None,
        "percentage": None,
        "charging": None,
    }


class SystemTelemetrySampler:
    """Sample host resource state without owning any output files."""

    def __init__(
        self,
        *,
        proc_stat_path: Path = DEFAULT_PROC_STAT,
        proc_meminfo_path: Path = DEFAULT_PROC_MEMINFO,
        proc_net_wireless_path: Path = DEFAULT_PROC_NET_WIRELESS,
        sys_root: Path = DEFAULT_SYS_ROOT,
        temperature_paths: Sequence[Path] | None = None,
        time_ns: TimeNs = time.time_ns,
    ) -> None:
        self._proc_stat_path = proc_stat_path
        self._proc_meminfo_path = proc_meminfo_path
        self._proc_net_wireless_path = proc_net_wireless_path
        self._sys_root = sys_root
        self._temperature_paths = tuple(temperature_paths) if temperature_paths is not None else None
        self._time_ns = time_ns
        self._previous_cpu: CpuSample | None = None

    def sample(self) -> dict[str, object]:
        current_cpu = self._read_cpu()
        cpu = cpu_percent(self._previous_cpu, current_cpu)
        if current_cpu is not None:
            self._previous_cpu = current_cpu

        memory = self._read_memory()
        temperature_paths = self._temperature_paths
        if temperature_paths is None:
            temperature_paths = default_temperature_paths(Path(self._sys_root) / "class/thermal")
        thermal = read_thermal_snapshot(
            Path(self._sys_root) / "class/thermal",
            sys_root=self._sys_root,
        )
        soc_temp_c = read_temperature_c(temperature_paths)
        if soc_temp_c is not None:
            thermal["soc_temp_c"] = soc_temp_c

        return make_system_record(
            recv_ts_ns=self._time_ns(),
            cpu_percent=cpu,
            memory_used_mb=memory.used_mb if memory is not None else 0.0,
            memory_available_mb=memory.available_mb if memory is not None else 0.0,
            soc_temp_c=soc_temp_c,
            thermal=thermal,
            network=read_network_snapshot(
                sys_root=self._sys_root,
                proc_net_wireless_path=self._proc_net_wireless_path,
            ),
            onboard_battery=read_onboard_battery_snapshot(self._sys_root),
        )

    def _read_cpu(self) -> CpuSample | None:
        try:
            return parse_proc_stat_cpu(self._proc_stat_path.read_text(encoding="utf-8"))
        except OSError:
            return None

    def _read_memory(self) -> MemorySample | None:
        try:
            return parse_proc_meminfo(self._proc_meminfo_path.read_text(encoding="utf-8"))
        except OSError:
            return None


__all__ = [
    "CpuSample",
    "MemorySample",
    "SystemTelemetrySampler",
    "cpu_percent",
    "default_temperature_paths",
    "parse_proc_meminfo",
    "parse_proc_stat_cpu",
    "read_network_snapshot",
    "read_onboard_battery_snapshot",
    "read_temperature_c",
    "read_thermal_snapshot",
    "read_wireless_stats",
    "select_wireless_interface",
]
