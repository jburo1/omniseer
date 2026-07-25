from __future__ import annotations

import os
import signal
import subprocess
import threading
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path


class RunPhase(str, Enum):
    IDLE = "idle"
    PREPARING = "preparing"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAILED = "failed"


@dataclass(frozen=True)
class RunState:
    phase: RunPhase
    run_id: str = ""
    message: str = ""


@dataclass(frozen=True)
class RunControlAvailability:
    new_id: bool
    start: bool
    stop: bool
    retrieve: bool
    open_report: bool


def run_state(phase: RunPhase, *, run_id: str = "", message: str = "") -> RunState:
    return RunState(phase=phase, run_id=run_id, message=message)


def run_control_availability(state: RunState) -> RunControlAvailability:
    can_start = state.phase in {RunPhase.IDLE, RunPhase.STOPPED, RunPhase.FAILED}
    can_stop = state.phase in {RunPhase.STARTING, RunPhase.RUNNING, RunPhase.STOPPING}
    has_run = bool(state.run_id)
    can_use_artifacts = has_run and state.phase in {RunPhase.STOPPED, RunPhase.FAILED}
    return RunControlAvailability(
        new_id=can_start,
        start=can_start,
        stop=can_stop,
        retrieve=can_use_artifacts,
        open_report=can_use_artifacts,
    )


@dataclass
class RemoteRunProcess:
    process: subprocess.Popen[str]
    stop_requested: bool = False


def start_remote_run_process(command: list[str], *, cwd: Path) -> RemoteRunProcess:
    return RemoteRunProcess(
        subprocess.Popen(
            command,
            cwd=cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            bufsize=1,
        )
    )


def start_remote_run_log_reader(remote_run: RemoteRunProcess, on_line: Callable[[str], None]) -> None:
    process = remote_run.process
    if process.stdout is None:
        return

    def _read_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            on_line(line.rstrip())

    threading.Thread(target=_read_output, name="omniseer_remote_run_log", daemon=True).start()


def remote_run_is_running(remote_run: RemoteRunProcess | None) -> bool:
    return remote_run is not None and remote_run.process.poll() is None


def request_remote_run_stop(remote_run: RemoteRunProcess | None) -> bool:
    if not remote_run_is_running(remote_run):
        return False
    assert remote_run is not None
    remote_run.stop_requested = True
    process = remote_run.process

    try:
        if process.stdin is None:
            raise OSError("run process stdin is unavailable")
        process.stdin.write("\x03")
        process.stdin.flush()
    except OSError:
        return False

    return True


def interrupt_remote_run(remote_run: RemoteRunProcess | None) -> bool:
    if not remote_run_is_running(remote_run):
        return False
    assert remote_run is not None
    process = remote_run.process

    try:
        os.killpg(process.pid, signal.SIGINT)
    except OSError:
        process.terminate()
    return True


def remote_run_exit_code(remote_run: RemoteRunProcess | None) -> int | None:
    if remote_run is None:
        return None
    return remote_run.process.poll()
