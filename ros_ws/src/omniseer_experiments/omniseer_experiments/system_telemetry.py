"""Low-rate Linux system telemetry sampling for perception run bundles."""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from omniseer_experiments.bundle import make_system_record

DEFAULT_PROC_STAT = Path("/proc/stat")
DEFAULT_PROC_MEMINFO = Path("/proc/meminfo")
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


class SystemTelemetrySampler:
    """Sample host resource state without owning any output files."""

    def __init__(
        self,
        *,
        proc_stat_path: Path = DEFAULT_PROC_STAT,
        proc_meminfo_path: Path = DEFAULT_PROC_MEMINFO,
        temperature_paths: Sequence[Path] | None = None,
        time_ns: TimeNs = time.time_ns,
    ) -> None:
        self._proc_stat_path = proc_stat_path
        self._proc_meminfo_path = proc_meminfo_path
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
            temperature_paths = default_temperature_paths()

        return make_system_record(
            recv_ts_ns=self._time_ns(),
            cpu_percent=cpu,
            memory_used_mb=memory.used_mb if memory is not None else 0.0,
            memory_available_mb=memory.available_mb if memory is not None else 0.0,
            soc_temp_c=read_temperature_c(temperature_paths),
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
    "read_temperature_c",
]
