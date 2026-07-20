"""Offboard retrieval helpers for robot-created Omniseer run bundles."""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from omniseer_experiments.run_inspection import RunInspection, format_run_summary, inspect_run

DEFAULT_REMOTE_ROOT = "/omniseer/runs"
DEFAULT_IMPORT_ROOT = "runs/imported"
UNKNOWN_VALUE = "-"

CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]


class RetrievalError(RuntimeError):
    """Raised when a retrieval operation cannot be completed."""


@dataclass(frozen=True)
class RemoteConfig:
    host: str
    user: str
    remote_root: str = DEFAULT_REMOTE_ROOT
    ssh_args: tuple[str, ...] = ()

    @property
    def target(self) -> str:
        return ssh_target(self.user, self.host)


@dataclass(frozen=True)
class RemoteRun:
    run_id: str
    remote_path: str
    state: str = UNKNOWN_VALUE
    started_at: str | None = None
    ended_at: str | None = None


@dataclass(frozen=True)
class PullResult:
    run_id: str
    remote_path: str
    local_path: Path
    inspection: RunInspection


def default_robot_user() -> str:
    return os.environ.get("OMNISEER_ROBOT_USER") or os.environ.get("USER") or ""


def default_remote_root() -> str:
    return os.environ.get("OMNISEER_ROBOT_RUNS_ROOT") or DEFAULT_REMOTE_ROOT


def ssh_target(user: str, host: str) -> str:
    host = host.strip()
    user = user.strip()
    if not host:
        raise RetrievalError("--host is required")
    return f"{user}@{host}" if user else host


def validate_run_id(run_id: str) -> str:
    value = run_id.strip()
    if not value:
        raise RetrievalError("run_id is required")
    if value in {".", ".."} or "/" in value:
        raise RetrievalError(f"run_id must name one child directory, got: {run_id}")
    return value


def remote_run_path(remote_root: str, run_id: str) -> str:
    return f"{remote_root.rstrip('/')}/{validate_run_id(run_id)}"


def remote_run_spec(config: RemoteConfig, run_id: str) -> str:
    return f"{config.target}:{remote_run_path(config.remote_root, run_id)}"


def build_remote_root_check_command(config: RemoteConfig) -> list[str]:
    return build_ssh_command(config, f"test -d {shlex.quote(config.remote_root)}")


def build_remote_run_check_command(config: RemoteConfig, run_id: str) -> list[str]:
    return build_ssh_command(config, f"test -d {shlex.quote(remote_run_path(config.remote_root, run_id))}")


def build_remote_list_command(config: RemoteConfig) -> list[str]:
    root = shlex.quote(config.remote_root)
    remote_command = (
        f"if [ ! -d {root} ]; then exit 3; fi; "
        f"find {root} -mindepth 1 -maxdepth 1 -type d -printf '%f\\n' | sort"
    )
    return build_ssh_command(config, remote_command)


def build_remote_inspect_command(config: RemoteConfig, run_id: str) -> list[str]:
    run_path = shlex.quote(remote_run_path(config.remote_root, run_id))
    remote_command = (
        "if command -v ros2 >/dev/null 2>&1; then "
        f"ros2 run omniseer_experiments inspect_run {run_path} --json; "
        "elif command -v inspect_run >/dev/null 2>&1; then "
        f"inspect_run {run_path} --json; "
        "else exit 127; fi"
    )
    return build_ssh_command(config, remote_command)


def build_ssh_command(config: RemoteConfig, remote_command: str) -> list[str]:
    return ["ssh", *config.ssh_args, config.target, remote_command]


def build_rsync_command(config: RemoteConfig, run_id: str, destination: Path) -> list[str]:
    remote_path = remote_run_path(config.remote_root, run_id).rstrip("/") + "/"
    source = f"{config.target}:{shlex.quote(remote_path)}"
    args = ["rsync", "-a"]
    if config.ssh_args:
        args.extend(["-e", shlex.join(["ssh", *config.ssh_args])])
    args.extend([source, f"{destination}/"])
    return args


def run_process(args: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, check=False, capture_output=True, text=True, timeout=60.0)


def require_command(name: str) -> None:
    if shutil.which(name) is None:
        raise RetrievalError(f"{name} not found in PATH")


@contextlib.contextmanager
def ssh_connection_reuse(config: RemoteConfig):
    with tempfile.TemporaryDirectory(prefix="omniseer-ssh-") as socket_dir:
        ssh_args = (
            "-o",
            "ControlMaster=auto",
            "-o",
            "ControlPersist=60",
            "-o",
            f"ControlPath={Path(socket_dir) / '%C'}",
        )
        reusable_config = replace(config, ssh_args=ssh_args)
        try:
            yield reusable_config
        finally:
            subprocess.run(
                ["ssh", *ssh_args, "-O", "exit", reusable_config.target],
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )


def list_remote_runs(config: RemoteConfig, *, runner: CommandRunner = run_process) -> list[RemoteRun]:
    _run_checked(build_remote_root_check_command(config), runner=runner, action="check remote runs root")
    result = _run_checked(build_remote_list_command(config), runner=runner, action="list remote runs")
    run_ids = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return [_remote_run_with_optional_inspection(config, run_id, runner=runner) for run_id in run_ids]


def pull_remote_run(
    config: RemoteConfig,
    run_id: str,
    *,
    import_root: Path = Path(DEFAULT_IMPORT_ROOT),
    out: Path | None = None,
    overwrite: bool = False,
    runner: CommandRunner = run_process,
) -> PullResult:
    normalized_run_id = validate_run_id(run_id)
    remote_path = remote_run_path(config.remote_root, normalized_run_id)
    _run_checked(build_remote_root_check_command(config), runner=runner, action="check remote runs root")
    _run_checked(build_remote_run_check_command(config, normalized_run_id), runner=runner, action="check remote run")

    destination = out if out is not None else import_root / normalized_run_id
    prepare_destination(destination, overwrite=overwrite)
    _run_checked(build_rsync_command(config, normalized_run_id, destination), runner=runner, action="copy remote run")
    inspection = inspect_run(destination)
    return PullResult(
        run_id=inspection.run_id,
        remote_path=remote_path,
        local_path=destination,
        inspection=inspection,
    )


def prepare_destination(destination: Path, *, overwrite: bool) -> None:
    if destination.exists():
        if not destination.is_dir():
            raise RetrievalError(f"local destination exists and is not a directory: {destination}")
        if any(destination.iterdir()):
            if not overwrite:
                raise RetrievalError(f"local destination is not empty; use --overwrite: {destination}")
            shutil.rmtree(destination)
            destination.mkdir(parents=True)
            return
    destination.mkdir(parents=True, exist_ok=True)


def format_remote_run_list(runs: Sequence[RemoteRun]) -> str:
    if not runs:
        return "no remote runs found"
    rows = ["run_id     state        started_at                 ended_at                   remote_path"]
    for item in runs:
        rows.append(
            f"{_clip(item.run_id, 10):<10} {_clip(item.state, 11):<11} "
            f"{_clip(_display_value(item.started_at), 26):<26} "
            f"{_clip(_display_value(item.ended_at), 26):<26} "
            f"{item.remote_path}"
        )
    return "\n".join(rows)


def format_pull_result(result: PullResult) -> str:
    return "\n".join(
        [
            f"Pulled: {result.run_id}",
            f"Remote: {result.remote_path}",
            f"Local: {result.local_path}",
            "",
            format_run_summary(result.inspection),
        ]
    )


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        config = RemoteConfig(host=args.host, user=args.user, remote_root=args.remote_root)
        if args.command == "list":
            require_command("ssh")
            with ssh_connection_reuse(config) as reusable_config:
                print(format_remote_run_list(list_remote_runs(reusable_config)))
            return 0
        if args.command == "pull":
            require_command("ssh")
            require_command("rsync")
            with ssh_connection_reuse(config) as reusable_config:
                result = pull_remote_run(
                    reusable_config,
                    args.run_id,
                    import_root=Path(args.import_root),
                    out=Path(args.out) if args.out else None,
                    overwrite=args.overwrite,
                )
            print(format_pull_result(result))
            return 0
    except RetrievalError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    parser.print_help()
    return 2


def _remote_run_with_optional_inspection(
    config: RemoteConfig, run_id: str, *, runner: CommandRunner
) -> RemoteRun:
    remote_path = remote_run_path(config.remote_root, run_id)
    result = runner(build_remote_inspect_command(config, run_id))
    if result.returncode != 0:
        return RemoteRun(run_id=run_id, remote_path=remote_path)

    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError:
        return RemoteRun(run_id=run_id, remote_path=remote_path)

    return RemoteRun(
        run_id=str(payload.get("run_id") or run_id),
        remote_path=remote_path,
        state=str(payload.get("state") or UNKNOWN_VALUE),
        started_at=_optional_string(payload.get("started_at")),
        ended_at=_optional_string(payload.get("ended_at")),
    )


def _run_checked(
    args: Sequence[str], *, runner: CommandRunner, action: str
) -> subprocess.CompletedProcess[str]:
    result = runner(args)
    if result.returncode == 0:
        return result
    detail = (result.stderr or result.stdout).strip()
    message = f"failed to {action}: {shlex.join(args)}"
    if detail:
        message = f"{message}\n{detail}"
    raise RetrievalError(message)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Retrieve Omniseer perception run bundles from a robot.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list robot-side run bundles")
    _add_remote_args(list_parser)

    pull_parser = subparsers.add_parser("pull", help="pull one robot-side run bundle")
    pull_parser.add_argument("run_id", help="remote run id to retrieve")
    _add_remote_args(pull_parser)
    pull_parser.add_argument("--import-root", default=DEFAULT_IMPORT_ROOT, help="local root for imported runs")
    pull_parser.add_argument("--out", default="", help="exact local destination directory")
    pull_parser.add_argument("--overwrite", action="store_true", help="replace a non-empty local destination")
    return parser


def _add_remote_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", required=True, help="robot hostname or IP address")
    parser.add_argument(
        "--user",
        default=default_robot_user(),
        help="SSH user; defaults to OMNISEER_ROBOT_USER or USER",
    )
    parser.add_argument(
        "--remote-root",
        default=default_remote_root(),
        help="robot directory containing run bundles; defaults to OMNISEER_ROBOT_RUNS_ROOT or /omniseer/runs",
    )


def _display_value(value: str | None) -> str:
    return value if value else UNKNOWN_VALUE


def _optional_string(value: Any) -> str | None:
    if value is None:
        return None
    return str(value)


def _clip(value: str, width: int) -> str:
    if len(value) <= width:
        return value
    if width <= 1:
        return value[:width]
    return value[: width - 1] + "+"


__all__ = [
    "DEFAULT_IMPORT_ROOT",
    "DEFAULT_REMOTE_ROOT",
    "PullResult",
    "RemoteConfig",
    "RemoteRun",
    "RetrievalError",
    "build_remote_inspect_command",
    "build_remote_list_command",
    "build_remote_root_check_command",
    "build_remote_run_check_command",
    "build_rsync_command",
    "build_ssh_command",
    "format_pull_result",
    "format_remote_run_list",
    "list_remote_runs",
    "main",
    "prepare_destination",
    "pull_remote_run",
    "remote_run_path",
    "remote_run_spec",
    "ssh_target",
    "validate_run_id",
]
