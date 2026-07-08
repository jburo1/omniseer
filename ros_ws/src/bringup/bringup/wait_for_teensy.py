"""Wait for a usable Teensy serial device."""

from __future__ import annotations

import argparse
import os
import stat
import sys
import termios
import time
from dataclasses import dataclass

TRUTHY_VALUES = {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class ProbeResult:
    ready: bool
    reason: str


def probe_device(device_path: str) -> ProbeResult:
    try:
        stat_result = os.stat(device_path)
    except FileNotFoundError:
        return ProbeResult(False, f"{device_path} does not exist yet")
    except OSError as exc:
        return ProbeResult(False, f"stat({device_path}) failed: {exc}")

    if not stat.S_ISCHR(stat_result.st_mode):
        return ProbeResult(False, f"{device_path} is not a character device")

    if not os.access(device_path, os.R_OK | os.W_OK):
        return ProbeResult(False, f"{device_path} is not readable and writable by the current user")

    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0)
    try:
        fd = os.open(device_path, flags)
    except OSError as exc:
        return ProbeResult(False, f"open({device_path}) failed: {exc}")

    try:
        if not os.isatty(fd):
            return ProbeResult(False, f"{device_path} opened but is not a tty device")

        try:
            termios.tcgetattr(fd)
        except termios.error as exc:
            return ProbeResult(False, f"termios probe failed for {device_path}: {exc}")
    finally:
        os.close(fd)

    return ProbeResult(True, f"{device_path} is present and openable")


def wait_for_device(device_path: str, timeout_sec: float, poll_interval_sec: float = 0.2) -> ProbeResult:
    deadline = time.monotonic() + max(timeout_sec, 0.0)
    last_reason = f"{device_path} has not been checked yet"

    while True:
        result = probe_device(device_path)
        if result.ready:
            return result

        last_reason = result.reason
        now = time.monotonic()
        if now >= deadline:
            return ProbeResult(
                False,
                f"Timed out after {timeout_sec:.1f}s waiting for {device_path}: {last_reason}",
            )

        time.sleep(min(poll_interval_sec, max(0.0, deadline - now)))


def _nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("timeout must be non-negative")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--timeout-sec", required=True, type=_nonnegative_float)
    args = parser.parse_args()

    result = wait_for_device(args.device, args.timeout_sec)
    stream = sys.stdout if result.ready else sys.stderr
    print(result.reason, file=stream)
    return 0 if result.ready else 1


if __name__ == "__main__":
    sys.exit(main())
