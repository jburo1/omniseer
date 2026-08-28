from __future__ import annotations

import subprocess
import tempfile
from collections.abc import Callable
from pathlib import Path

from robot_diag_control.run_commands import (
    RUN_TYPE_AUTONOMY_CENTER,
    RobotConnection,
    RunConfig,
    build_remote_run_mkdir_command,
    build_remote_start_command,
    build_upload_classes_command,
)

CommandRunner = Callable[[list[str], str], None]


def run_command_checked(command: list[str], action: str, *, cwd: Path) -> None:
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise OSError(f"{action} failed: {detail}")


def upload_classes_file(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
    command_runner: CommandRunner,
) -> None:
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as handle:
        local_path = Path(handle.name)
        handle.write("\n".join(run_config.classes) + "\n")
    try:
        upload_command = build_upload_classes_command(
            connection=connection,
            run_config=run_config,
            local_path=local_path,
        )
        command_runner(upload_command, "upload classes")
    finally:
        try:
            local_path.unlink()
        except OSError:
            pass


def prepare_remote_run(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
    cwd: Path,
    command_runner: CommandRunner | None = None,
) -> list[str]:
    runner = command_runner or (lambda command, action: run_command_checked(command, action, cwd=cwd))
    mkdir_command = build_remote_run_mkdir_command(
        connection=connection,
        run_config=run_config,
    )
    runner(mkdir_command, "create remote run directory")
    if run_config.run_type == RUN_TYPE_AUTONOMY_CENTER and run_config.classes:
        upload_classes_file(
            connection=connection,
            run_config=run_config,
            command_runner=runner,
        )
    return build_remote_start_command(
        connection=connection,
        run_config=run_config,
    )
