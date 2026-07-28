import subprocess
import unittest
from pathlib import Path

from robot_diag_control.run_commands import RUN_BACKEND_RUNTIME, RobotConnection, RunConfig
from robot_diag_control.run_lifecycle import RemoteRunProcess, RunPhase
from robot_diag_control.run_manager import RunManager


class _FakeProcess:
    stdin = None
    stdout = None
    pid = 12345

    def __init__(self, *, exit_code: int | None = None) -> None:
        self._exit_code = exit_code

    def poll(self) -> int | None:
        return self._exit_code


def _connection() -> RobotConnection:
    return RobotConnection(
        host="10.0.0.2",
        ssh_user="radxa",
        remote_repo_root="/home/radxa/apps/omniseer",
        remote_runs_root="/home/radxa/apps/omniseer/runs",
    )


class RunManagerTests(unittest.TestCase):
    def test_start_prepares_then_calls_before_callback_then_starts_process(self):
        events: list[str] = []

        def command_runner(_command: list[str], action: str) -> None:
            events.append(action)

        def process_starter(command: list[str], cwd: Path) -> RemoteRunProcess:
            events.append(f"start {cwd}")
            self.assertIn("scripts/omni runtime record --run-id operator_001", command[3])
            return RemoteRunProcess(_FakeProcess())  # type: ignore[arg-type]

        manager = RunManager(
            repo_root=Path("/repo"),
            command_runner=command_runner,
            process_starter=process_starter,
        )

        result = manager.start(
            connection=_connection(),
            run_config=RunConfig(run_id="operator_001", backend=RUN_BACKEND_RUNTIME),
            before_process_start=lambda _command: events.append("before start"),
        )

        self.assertEqual(events, ["create remote run directory", "before start", "start /repo"])
        self.assertEqual(result.command[0:3], ["ssh", "-tt", "radxa@10.0.0.2"])
        self.assertIsNotNone(result.remote_run)

    def test_start_does_not_launch_process_when_preparation_fails(self):
        events: list[str] = []

        def command_runner(_command: list[str], action: str) -> None:
            events.append(action)
            raise OSError("mkdir failed")

        def process_starter(_command: list[str], _cwd: Path) -> RemoteRunProcess:
            events.append("start")
            return RemoteRunProcess(_FakeProcess())  # type: ignore[arg-type]

        manager = RunManager(
            repo_root=Path("/repo"),
            command_runner=command_runner,
            process_starter=process_starter,
        )

        with self.assertRaisesRegex(OSError, "mkdir failed"):
            manager.start(
                connection=_connection(),
                run_config=RunConfig(run_id="operator_001", backend=RUN_BACKEND_RUNTIME),
            )

        self.assertEqual(events, ["create remote run directory"])

    def test_request_stop_returns_not_accepted_when_run_is_not_running(self):
        events: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            stop_requester=lambda _run: events.append("stop") == "unused",
            run_interrupter=lambda _run: events.append("interrupt") == "unused",
        )

        result = manager.request_stop(RemoteRunProcess(_FakeProcess(exit_code=0)))  # type: ignore[arg-type]

        self.assertFalse(result.accepted)
        self.assertFalse(result.graceful)
        self.assertFalse(result.interrupted)
        self.assertEqual(events, [])

    def test_request_stop_reports_graceful_stop_when_process_accepts_ctrl_c(self):
        events: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            stop_requester=lambda _run: events.append("stop") is None or True,
            run_interrupter=lambda _run: events.append("interrupt") is None or True,
        )

        result = manager.request_stop(RemoteRunProcess(_FakeProcess()))  # type: ignore[arg-type]

        self.assertTrue(result.accepted)
        self.assertTrue(result.graceful)
        self.assertFalse(result.interrupted)
        self.assertEqual(events, ["stop"])

    def test_request_stop_interrupts_process_when_graceful_stop_fails(self):
        events: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            stop_requester=lambda _run: events.append("stop") is None and False,
            run_interrupter=lambda _run: events.append("interrupt") is None or True,
        )

        result = manager.request_stop(RemoteRunProcess(_FakeProcess()))  # type: ignore[arg-type]

        self.assertTrue(result.accepted)
        self.assertFalse(result.graceful)
        self.assertTrue(result.interrupted)
        self.assertEqual(events, ["stop", "interrupt"])

    def test_force_stop_interrupts_running_process(self):
        events: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            run_interrupter=lambda _run: events.append("interrupt") is None or True,
        )

        result = manager.force_stop(RemoteRunProcess(_FakeProcess()))  # type: ignore[arg-type]

        self.assertTrue(result.accepted)
        self.assertFalse(result.graceful)
        self.assertTrue(result.interrupted)
        self.assertEqual(events, ["interrupt"])

    def test_completion_returns_none_while_process_is_running(self):
        manager = RunManager(repo_root=Path("/repo"))

        self.assertIsNone(manager.completion(RemoteRunProcess(_FakeProcess()), run_id="operator_001"))  # type: ignore[arg-type]

    def test_completion_reports_stopped_when_stop_was_requested(self):
        manager = RunManager(repo_root=Path("/repo"))
        remote_run = RemoteRunProcess(_FakeProcess(exit_code=130), stop_requested=True)  # type: ignore[arg-type]

        completion = manager.completion(remote_run, run_id="operator_001")

        self.assertIsNotNone(completion)
        assert completion is not None
        self.assertEqual(completion.phase, RunPhase.STOPPED)
        self.assertEqual(completion.run_id, "operator_001")
        self.assertEqual(completion.exit_code, 130)
        self.assertEqual(completion.state_message, "")
        self.assertEqual(completion.log_message, "remote run stopped: operator_001")

    def test_completion_reports_stopped_when_process_exits_zero(self):
        manager = RunManager(repo_root=Path("/repo"))
        remote_run = RemoteRunProcess(_FakeProcess(exit_code=0))  # type: ignore[arg-type]

        completion = manager.completion(remote_run, run_id="operator_001")

        self.assertIsNotNone(completion)
        assert completion is not None
        self.assertEqual(completion.phase, RunPhase.STOPPED)
        self.assertEqual(completion.state_message, "")
        self.assertEqual(completion.log_message, "remote run exited with code 0: operator_001")

    def test_completion_reports_failed_when_process_exits_nonzero(self):
        manager = RunManager(repo_root=Path("/repo"))
        remote_run = RemoteRunProcess(_FakeProcess(exit_code=255))  # type: ignore[arg-type]

        completion = manager.completion(remote_run, run_id="operator_001")

        self.assertIsNotNone(completion)
        assert completion is not None
        self.assertEqual(completion.phase, RunPhase.FAILED)
        self.assertEqual(completion.state_message, "exit code 255")
        self.assertEqual(completion.log_message, "remote run exited with code 255: operator_001")

    def test_request_runtime_stop_returns_false_without_run_id(self):
        manager = RunManager(repo_root=Path("/repo"))

        self.assertFalse(
            manager.request_runtime_stop(
                connection=_connection(),
                run_id="",
                on_command=lambda _command: self.fail("command should not be reported"),
                on_message=lambda _message: self.fail("message should not be reported"),
            )
        )

    def test_request_runtime_stop_runs_command_and_reports_success(self):
        commands: list[list[str]] = []
        messages: list[str] = []
        completions: list[tuple[bool, str]] = []

        def runtime_stop_runner(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
            commands.append(command)
            self.assertEqual(cwd, Path("/repo"))
            return subprocess.CompletedProcess(command, 0, stdout="stopped operator_001\n", stderr="")

        manager = RunManager(
            repo_root=Path("/repo"),
            runtime_stop_runner=runtime_stop_runner,
            runtime_stop_scheduler=lambda target: target(),
        )

        accepted = manager.request_runtime_stop(
            connection=_connection(),
            run_id="operator_001",
            on_command=commands.append,
            on_message=messages.append,
            on_completion=lambda success, message: completions.append((success, message)),
        )

        self.assertTrue(accepted)
        self.assertEqual(commands[0], commands[1])
        self.assertEqual(commands[0][0:2], ["ssh", "radxa@10.0.0.2"])
        self.assertIn("scripts/omni runtime stop --run-id operator_001", commands[0][2])
        self.assertEqual(messages, ["runtime container stop requested: stopped operator_001"])
        self.assertEqual(completions, [(True, "runtime container stop requested: stopped operator_001")])

    def test_request_runtime_stop_reports_already_stopped_container(self):
        messages: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            runtime_stop_runner=lambda command, _cwd: subprocess.CompletedProcess(
                command,
                0,
                stdout="runtime record container is not present\n",
                stderr="",
            ),
            runtime_stop_scheduler=lambda target: target(),
        )

        self.assertTrue(
            manager.request_runtime_stop(
                connection=_connection(),
                run_id="operator_001",
                on_message=messages.append,
            )
        )

        self.assertEqual(messages, ["runtime container already stopped: runtime record container is not present"])

    def test_request_runtime_stop_reports_command_failure(self):
        messages: list[str] = []
        manager = RunManager(
            repo_root=Path("/repo"),
            runtime_stop_runner=lambda command, _cwd: subprocess.CompletedProcess(
                command,
                42,
                stdout="",
                stderr="ssh failed\n",
            ),
            runtime_stop_scheduler=lambda target: target(),
        )

        self.assertTrue(
            manager.request_runtime_stop(
                connection=_connection(),
                run_id="operator_001",
                on_message=messages.append,
            )
        )

        self.assertEqual(messages, ["runtime container stop failed: ssh failed"])
