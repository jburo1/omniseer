import sys
import time
import unittest
from pathlib import Path

from robot_diag_control.run_lifecycle import (
    RemoteRunProcess,
    RunPhase,
    interrupt_remote_run,
    remote_run_exit_code,
    remote_run_is_running,
    request_remote_run_stop,
    run_control_availability,
    run_state,
    start_remote_run_log_reader,
    start_remote_run_process,
)


class _FakeStdin:
    def __init__(self) -> None:
        self.written = ""
        self.flushed = False

    def write(self, value: str) -> None:
        self.written += value

    def flush(self) -> None:
        self.flushed = True


class _FakeProcess:
    def __init__(self, *, exit_code: int | None = None) -> None:
        self.stdin = _FakeStdin()
        self._exit_code = exit_code

    def poll(self) -> int | None:
        return self._exit_code


class RunLifecycleTests(unittest.TestCase):
    def test_run_state_records_phase_run_id_and_message(self):
        state = run_state(RunPhase.RUNNING, run_id="operator_001", message="started")

        self.assertEqual(state.phase, RunPhase.RUNNING)
        self.assertEqual(state.run_id, "operator_001")
        self.assertEqual(state.message, "started")

    def test_run_controls_allow_start_only_when_idle_stopped_or_failed(self):
        idle = run_control_availability(run_state(RunPhase.IDLE))
        running = run_control_availability(run_state(RunPhase.RUNNING, run_id="operator_001"))
        stopped = run_control_availability(run_state(RunPhase.STOPPED, run_id="operator_001"))

        self.assertTrue(idle.start)
        self.assertFalse(idle.stop)
        self.assertFalse(idle.retrieve)
        self.assertFalse(running.start)
        self.assertTrue(running.stop)
        self.assertFalse(running.retrieve)
        self.assertTrue(stopped.start)
        self.assertFalse(stopped.stop)
        self.assertTrue(stopped.retrieve)
        self.assertTrue(stopped.open_report)

    def test_run_controls_allow_artifacts_after_failed_run_with_id(self):
        failed = run_control_availability(run_state(RunPhase.FAILED, run_id="operator_001", message="exit code 255"))
        failed_without_id = run_control_availability(run_state(RunPhase.FAILED, message="invalid connection"))

        self.assertTrue(failed.start)
        self.assertTrue(failed.retrieve)
        self.assertFalse(failed.stop)
        self.assertTrue(failed_without_id.start)
        self.assertFalse(failed_without_id.retrieve)

    def test_request_remote_run_stop_writes_ctrl_c_to_stdin(self):
        process = _FakeProcess()
        remote_run = RemoteRunProcess(process)  # type: ignore[arg-type]

        self.assertTrue(request_remote_run_stop(remote_run))

        self.assertTrue(remote_run.stop_requested)
        self.assertEqual(process.stdin.written, "\x03")
        self.assertTrue(process.stdin.flushed)

    def test_request_remote_run_stop_returns_false_when_process_exited(self):
        remote_run = RemoteRunProcess(_FakeProcess(exit_code=0))  # type: ignore[arg-type]

        self.assertFalse(request_remote_run_stop(remote_run))
        self.assertFalse(remote_run.stop_requested)

    def test_start_remote_run_process_streams_stdout_and_reports_exit(self):
        remote_run = start_remote_run_process(
            [sys.executable, "-c", "print('ready', flush=True)"],
            cwd=Path.cwd(),
        )
        lines: list[str] = []
        start_remote_run_log_reader(remote_run, lines.append)

        exit_code = remote_run.process.wait(timeout=5.0)
        deadline = time.monotonic() + 2.0
        while not lines and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual(exit_code, 0)
        self.assertEqual(remote_run_exit_code(remote_run), 0)
        self.assertEqual(lines, ["ready"])

    def test_interrupt_remote_run_stops_running_process(self):
        remote_run = start_remote_run_process(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            cwd=Path.cwd(),
        )
        try:
            self.assertTrue(remote_run_is_running(remote_run))
            self.assertTrue(interrupt_remote_run(remote_run))
            remote_run.process.wait(timeout=5.0)
            self.assertIsNotNone(remote_run_exit_code(remote_run))
        finally:
            if remote_run_is_running(remote_run):
                remote_run.process.kill()
                remote_run.process.wait(timeout=5.0)
