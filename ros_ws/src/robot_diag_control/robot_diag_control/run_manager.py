from __future__ import annotations

import subprocess
import threading
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from robot_diag_control.run_commands import RobotConnection, RunConfig, build_remote_runtime_stop_command
from robot_diag_control.run_lifecycle import (
    RemoteRunProcess,
    RunPhase,
    interrupt_remote_run,
    remote_run_exit_code,
    remote_run_is_running,
    request_remote_run_stop,
    start_remote_run_process,
)
from robot_diag_control.run_preparation import CommandRunner, prepare_remote_run

ProcessStarter = Callable[[list[str], Path], RemoteRunProcess]
StopRequester = Callable[[RemoteRunProcess | None], bool]
RunInterrupter = Callable[[RemoteRunProcess | None], bool]
RuntimeStopRunner = Callable[[list[str], Path], subprocess.CompletedProcess[str]]
RuntimeStopScheduler = Callable[[Callable[[], None]], None]


@dataclass(frozen=True)
class RunStartResult:
    command: list[str]
    remote_run: RemoteRunProcess


@dataclass(frozen=True)
class RunStopResult:
    accepted: bool
    graceful: bool = False
    interrupted: bool = False


@dataclass(frozen=True)
class RunCompletion:
    phase: RunPhase
    run_id: str
    exit_code: int
    state_message: str = ""
    log_message: str = ""


def _default_process_starter(command: list[str], cwd: Path) -> RemoteRunProcess:
    return start_remote_run_process(command, cwd=cwd)


def _default_runtime_stop_runner(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )


def _default_runtime_stop_scheduler(target: Callable[[], None]) -> None:
    threading.Thread(target=target, name="omniseer_runtime_stop", daemon=True).start()


def _format_runtime_stop_message(completed: subprocess.CompletedProcess[str]) -> str:
    detail = (completed.stderr or completed.stdout).strip()
    if completed.returncode == 0:
        if "runtime record container is not present" in detail:
            message = "runtime container already stopped"
        else:
            message = "runtime container stop requested"
        if detail:
            message = f"{message}: {detail}"
        return message
    return f"runtime container stop failed: {detail or completed.returncode}"


class RunManager:
    def __init__(
        self,
        *,
        repo_root: Path,
        command_runner: CommandRunner | None = None,
        process_starter: ProcessStarter = _default_process_starter,
        stop_requester: StopRequester = request_remote_run_stop,
        run_interrupter: RunInterrupter = interrupt_remote_run,
        runtime_stop_runner: RuntimeStopRunner = _default_runtime_stop_runner,
        runtime_stop_scheduler: RuntimeStopScheduler = _default_runtime_stop_scheduler,
    ) -> None:
        self._repo_root = repo_root
        self._command_runner = command_runner
        self._process_starter = process_starter
        self._stop_requester = stop_requester
        self._run_interrupter = run_interrupter
        self._runtime_stop_runner = runtime_stop_runner
        self._runtime_stop_scheduler = runtime_stop_scheduler

    def start(
        self,
        *,
        connection: RobotConnection,
        run_config: RunConfig,
        before_process_start: Callable[[list[str]], None] | None = None,
    ) -> RunStartResult:
        command = prepare_remote_run(
            connection=connection,
            run_config=run_config,
            cwd=self._repo_root,
            command_runner=self._command_runner,
        )
        if before_process_start is not None:
            before_process_start(command)
        remote_run = self._process_starter(command, self._repo_root)
        return RunStartResult(command=command, remote_run=remote_run)

    def request_stop(self, remote_run: RemoteRunProcess | None) -> RunStopResult:
        if not remote_run_is_running(remote_run):
            return RunStopResult(accepted=False)
        if self._stop_requester(remote_run):
            return RunStopResult(accepted=True, graceful=True)
        interrupted = self._run_interrupter(remote_run)
        return RunStopResult(accepted=True, interrupted=interrupted)

    def force_stop(self, remote_run: RemoteRunProcess | None) -> RunStopResult:
        interrupted = self._run_interrupter(remote_run)
        return RunStopResult(accepted=interrupted, interrupted=interrupted)

    def completion(self, remote_run: RemoteRunProcess | None, *, run_id: str) -> RunCompletion | None:
        exit_code = remote_run_exit_code(remote_run)
        if exit_code is None:
            return None
        if remote_run is not None and remote_run.stop_requested:
            return RunCompletion(
                phase=RunPhase.STOPPED,
                run_id=run_id,
                exit_code=exit_code,
                log_message=f"remote run stopped: {run_id}",
            )
        if exit_code == 0:
            return RunCompletion(
                phase=RunPhase.STOPPED,
                run_id=run_id,
                exit_code=exit_code,
                log_message=f"remote run exited with code {exit_code}: {run_id}",
            )
        return RunCompletion(
            phase=RunPhase.FAILED,
            run_id=run_id,
            exit_code=exit_code,
            state_message=f"exit code {exit_code}",
            log_message=f"remote run exited with code {exit_code}: {run_id}",
        )

    def request_runtime_stop(
        self,
        *,
        connection: RobotConnection,
        run_id: str,
        on_command: Callable[[list[str]], None] | None = None,
        on_message: Callable[[str], None] | None = None,
    ) -> bool:
        if not run_id:
            return False

        command = build_remote_runtime_stop_command(connection=connection, run_id=run_id)
        if on_command is not None:
            on_command(command)

        def _stop_remote_container() -> None:
            completed = self._runtime_stop_runner(command, self._repo_root)
            if on_message is not None:
                on_message(_format_runtime_stop_message(completed))

        self._runtime_stop_scheduler(_stop_remote_container)
        return True
