"""Wait for ROS topics to become discoverable and publish a first message."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class ProbeResult:
    ready: bool
    reason: str


Runner = Callable[..., subprocess.CompletedProcess[str]]


def _failure_reason(topic: str, result: subprocess.CompletedProcess[str]) -> str:
    detail = (result.stderr or result.stdout or "no diagnostic output").strip()
    return f"ros2 topic echo --once {topic} exited {result.returncode}: {detail}"


def probe_topic(topic: str, timeout_sec: float, *, runner: Runner = subprocess.run) -> ProbeResult:
    """Probe one topic once, including both graph discovery and a live message."""
    try:
        result = runner(
            ["ros2", "topic", "echo", "--once", topic],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        return ProbeResult(False, f"no message received on {topic} within {timeout_sec:.1f}s")
    except OSError as exc:
        return ProbeResult(False, f"could not execute ros2 while probing {topic}: {exc}")

    if result.returncode == 0:
        return ProbeResult(True, f"received a message on {topic}")
    return ProbeResult(False, _failure_reason(topic, result))


def wait_for_topic(
    topic: str,
    timeout_sec: float,
    *,
    poll_interval_sec: float = 0.2,
    runner: Runner = subprocess.run,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ProbeResult:
    """Wait for a topic to appear and then deliver one message.

    ``ros2 topic echo --once`` exits immediately when a topic is not yet in the
    graph. Retrying that condition avoids treating normal micro-ROS entity
    creation latency as a hardware failure.
    """
    deadline = monotonic() + max(timeout_sec, 0.0)
    last_result = ProbeResult(False, f"{topic} has not been probed yet")

    while True:
        remaining_sec = deadline - monotonic()
        if remaining_sec <= 0.0:
            return ProbeResult(
                False,
                f"Timed out after {timeout_sec:.1f}s waiting for first message on {topic}: "
                f"{last_result.reason}",
            )

        last_result = probe_topic(topic, remaining_sec, runner=runner)
        if last_result.ready:
            return last_result

        remaining_sec = deadline - monotonic()
        if remaining_sec <= 0.0:
            return ProbeResult(
                False,
                f"Timed out after {timeout_sec:.1f}s waiting for first message on {topic}: "
                f"{last_result.reason}",
            )
        sleep(min(poll_interval_sec, remaining_sec))


def _nonnegative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("timeout must be a finite non-negative number")
    return parsed


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", required=True, type=_nonnegative_float)
    parser.add_argument("topics", nargs="+")
    args = parser.parse_args(argv)

    for topic in args.topics:
        result = wait_for_topic(topic, args.timeout_sec)
        if not result.ready:
            print(result.reason, file=sys.stderr)
            return 1
        print(result.reason)
    return 0


if __name__ == "__main__":
    sys.exit(main())
