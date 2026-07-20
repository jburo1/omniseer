import json
import shutil
import subprocess
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from omniseer_experiments.bundle import RunBundleConfig, RunBundleWriter, make_detection_record, make_perf_record
from omniseer_experiments.run_inspection import STATE_COMPLETE
from omniseer_experiments.run_retrieval import (
    RemoteConfig,
    RetrievalError,
    build_remote_inspect_command,
    build_remote_list_command,
    build_remote_run_check_command,
    build_rsync_command,
    format_remote_run_list,
    list_remote_runs,
    prepare_destination,
    pull_remote_run,
    remote_run_path,
    remote_run_spec,
)

STARTED_AT = datetime(2026, 7, 19, 12, 0, 0, tzinfo=timezone.utc)
ENDED_AT = datetime(2026, 7, 19, 12, 1, 5, tzinfo=timezone.utc)


def _completed_process(args: list[str], returncode: int = 0, stdout: str = "", stderr: str = ""):
    return subprocess.CompletedProcess(args=args, returncode=returncode, stdout=stdout, stderr=stderr)


def _write_completed_bundle(run_dir: Path, *, run_id: str = "demo_001") -> None:
    writer = RunBundleWriter(
        RunBundleConfig(
            run_id=run_id,
            out_dir=run_dir,
            classes=("chair", "backpack"),
            notes="retrieval fixture",
            ros_distro="kilted",
            git_sha="abc123",
        ),
        started_at=STARTED_AT,
    )
    writer.write_detection_record(
        make_detection_record(
            recv_ts_ns=100,
            header_stamp={"sec": 1, "nanosec": 2},
            frame_id="camera_frame",
            detections=[
                {
                    "class_id": 0,
                    "class_name": "chair",
                    "score": 0.8,
                    "bbox": {"center_x": 10.0, "center_y": 20.0, "size_x": 30.0, "size_y": 40.0},
                }
            ],
        )
    )
    writer.write_perf_record(
        make_perf_record(
            recv_ts_ns=200,
            header_stamp={"sec": 3, "nanosec": 4},
            frame_id="camera_frame",
            producer_fps=20.0,
            consumer_fps=19.0,
            last_preprocess_ms=1.0,
            last_infer_ms=8.0,
            last_postprocess_ms=2.0,
            last_publish_ms=0.5,
            last_producer_total_ms=3.0,
            last_consumer_total_ms=9.0,
            produced_count=10,
            consumed_count=9,
            error_counts={},
        )
    )
    writer.finalize(ended_at=ENDED_AT)


class FakePullRunner:
    def __init__(self, source: Path) -> None:
        self.source = source
        self.calls: list[list[str]] = []

    def __call__(self, args: list[str]) -> subprocess.CompletedProcess[str]:
        self.calls.append(list(args))
        if args[0] == "ssh":
            return _completed_process(args)
        if args[0] == "rsync":
            destination = Path(args[-1])
            shutil.copytree(self.source, destination, dirs_exist_ok=True)
            return _completed_process(args)
        return _completed_process(args, returncode=127, stderr="unexpected command")


class RunRetrievalTests(unittest.TestCase):
    def test_builds_remote_paths_and_commands(self) -> None:
        config = RemoteConfig(host="192.0.2.10", user="robot", remote_root="/omniseer/runs")

        self.assertEqual(remote_run_path(config.remote_root, "demo_001"), "/omniseer/runs/demo_001")
        self.assertEqual(remote_run_spec(config, "demo_001"), "robot@192.0.2.10:/omniseer/runs/demo_001")
        self.assertEqual(
            build_remote_run_check_command(config, "demo_001"),
            ["ssh", "robot@192.0.2.10", "test -d /omniseer/runs/demo_001"],
        )
        self.assertIn("find /omniseer/runs", build_remote_list_command(config)[2])
        self.assertIn("inspect_run /omniseer/runs/demo_001 --json", build_remote_inspect_command(config, "demo_001")[2])
        self.assertEqual(
            build_rsync_command(config, "demo_001", Path("runs/imported/demo_001")),
            ["rsync", "-a", "robot@192.0.2.10:/omniseer/runs/demo_001/", "runs/imported/demo_001/"],
        )

    def test_builds_commands_with_ssh_reuse_args(self) -> None:
        config = RemoteConfig(
            host="192.0.2.10",
            user="robot",
            remote_root="/omniseer/runs",
            ssh_args=(
                "-o",
                "ControlMaster=auto",
                "-o",
                "ControlPersist=60",
                "-o",
                "ControlPath=/tmp/omniseer-%C",
            ),
        )

        self.assertEqual(
            build_remote_list_command(config)[:8],
            [
                "ssh",
                "-o",
                "ControlMaster=auto",
                "-o",
                "ControlPersist=60",
                "-o",
                "ControlPath=/tmp/omniseer-%C",
                "robot@192.0.2.10",
            ],
        )
        self.assertEqual(
            build_rsync_command(config, "demo_001", Path("runs/imported/demo_001")),
            [
                "rsync",
                "-a",
                "-e",
                "ssh -o ControlMaster=auto -o ControlPersist=60 -o ControlPath=/tmp/omniseer-%C",
                "robot@192.0.2.10:/omniseer/runs/demo_001/",
                "runs/imported/demo_001/",
            ],
        )

    def test_invalid_run_id_is_rejected(self) -> None:
        config = RemoteConfig(host="robot.local", user="robot", remote_root="/omniseer/runs")

        with self.assertRaisesRegex(RetrievalError, "one child directory"):
            remote_run_path(config.remote_root, "../demo")

    def test_list_remote_runs_uses_inspection_when_available(self) -> None:
        config = RemoteConfig(host="robot.local", user="robot", remote_root="/omniseer/runs")

        def runner(args: list[str]) -> subprocess.CompletedProcess[str]:
            if args[2].startswith("test -d"):
                return _completed_process(args)
            if args[2].startswith("if [ ! -d"):
                return _completed_process(args, stdout="demo_001\nopen_run\n")
            if "demo_001" in args[2]:
                return _completed_process(
                    args,
                    stdout=json.dumps(
                        {
                            "run_id": "demo_001",
                            "state": "complete",
                            "started_at": "2026-07-19T12:00:00+00:00",
                            "ended_at": "2026-07-19T12:01:05+00:00",
                        }
                    ),
                )
            return _completed_process(args, returncode=127, stderr="inspect unavailable")

        runs = list_remote_runs(config, runner=runner)
        output = format_remote_run_list(runs)

        self.assertEqual([item.run_id for item in runs], ["demo_001", "open_run"])
        self.assertEqual(runs[0].state, "complete")
        self.assertEqual(runs[1].state, "-")
        self.assertIn("remote_path", output)
        self.assertIn("/omniseer/runs/open_run", output)

    def test_prepare_destination_refuses_non_empty_without_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / "demo_001"
            destination.mkdir()
            (destination / "old.txt").write_text("old", encoding="utf-8")

            with self.assertRaisesRegex(RetrievalError, "--overwrite"):
                prepare_destination(destination, overwrite=False)

    def test_prepare_destination_overwrites_non_empty_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / "demo_001"
            destination.mkdir()
            (destination / "old.txt").write_text("old", encoding="utf-8")

            prepare_destination(destination, overwrite=True)

            self.assertTrue(destination.is_dir())
            self.assertEqual(list(destination.iterdir()), [])

    def test_pull_preserves_unknown_files_and_inspects_import(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "remote" / "demo_001"
            destination = Path(tmp) / "runs" / "imported" / "demo_001"
            _write_completed_bundle(source)
            (source / "system.jsonl").write_text('{"cpu": 12}\n', encoding="utf-8")
            (source / "future" / "nested").mkdir(parents=True)
            (source / "future" / "nested" / "artifact.txt").write_text("preserved", encoding="utf-8")
            runner = FakePullRunner(source)
            config = RemoteConfig(host="robot.local", user="robot", remote_root="/omniseer/runs")

            result = pull_remote_run(config, "demo_001", out=destination, runner=runner)

            self.assertEqual(result.inspection.state, STATE_COMPLETE)
            self.assertTrue((destination / "system.jsonl").exists())
            self.assertEqual(
                (destination / "future" / "nested" / "artifact.txt").read_text(encoding="utf-8"),
                "preserved",
            )
            self.assertEqual(runner.calls[-1][0], "rsync")

    def test_pull_reports_missing_remote_run(self) -> None:
        config = RemoteConfig(host="robot.local", user="robot", remote_root="/omniseer/runs")

        def runner(args: list[str]) -> subprocess.CompletedProcess[str]:
            if args[2] == "test -d /omniseer/runs":
                return _completed_process(args)
            return _completed_process(args, returncode=1, stderr="missing")

        with self.assertRaisesRegex(RetrievalError, "failed to check remote run"):
            pull_remote_run(config, "demo_001", out=Path("unused"), runner=runner)

    def test_pull_reports_missing_remote_root(self) -> None:
        config = RemoteConfig(host="robot.local", user="robot", remote_root="/missing/runs")

        def runner(args: list[str]) -> subprocess.CompletedProcess[str]:
            return _completed_process(args, returncode=1, stderr="missing root")

        with self.assertRaisesRegex(RetrievalError, "failed to check remote runs root"):
            pull_remote_run(config, "demo_001", out=Path("unused"), runner=runner)
