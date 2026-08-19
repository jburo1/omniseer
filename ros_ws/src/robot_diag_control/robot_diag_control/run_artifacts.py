from __future__ import annotations

import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from omniseer_experiments.run_inspection import STATE_COMPLETE, InspectionIssue, inspect_run
from omniseer_experiments.run_retrieval import RsyncProgress, parse_rsync_progress_event

from robot_diag_control.run_commands import (
    RobotConnection,
    build_pull_run_command,
    local_import_dir_for,
)
from robot_diag_control.run_commands import (
    build_report_command as build_omni_report_command,
)
from robot_diag_control.run_commands import (
    build_video_command as build_omni_video_command,
)

CommandExecutor = Callable[[list[str], Path], subprocess.CompletedProcess[str]]
ProgressCommandExecutor = Callable[[list[str], Path, Callable[[str], None]], subprocess.CompletedProcess[str]]
ProgressCallback = Callable[[RsyncProgress], None]


@dataclass(frozen=True)
class RunArtifactContext:
    repo_root: Path
    connection: RobotConnection
    local_import_root: Path


@dataclass(frozen=True)
class RunArtifactResult:
    command: list[str]
    success: bool
    message: str
    path: Path | None = None
    issues: tuple[str, ...] = ()


@dataclass(frozen=True)
class RunArtifactInspection:
    run_dir: Path
    report_path: Path
    state: str
    issues: tuple[str, ...]
    report_exists: bool

    @property
    def complete(self) -> bool:
        return self.state == STATE_COMPLETE and not self.issues


def _default_command_executor(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )


def _default_progress_command_executor(
    command: list[str],
    cwd: Path,
    on_output: Callable[[str], None],
) -> subprocess.CompletedProcess[str]:
    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    output: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        output.append(line)
        on_output(line)
    return subprocess.CompletedProcess(command, process.wait(), stdout="".join(output), stderr="")


def _completed_detail(completed: subprocess.CompletedProcess[str]) -> str:
    return (completed.stderr or completed.stdout).strip()


def imported_run_dir(context: RunArtifactContext, run_id: str) -> Path:
    return local_import_dir_for(context.local_import_root, run_id)


def report_path(context: RunArtifactContext, run_id: str) -> Path:
    return imported_run_dir(context, run_id) / "report" / "index.html"


def inspect_run_artifacts(
    context: RunArtifactContext,
    run_id: str,
    *,
    require_report: bool = False,
) -> RunArtifactInspection:
    run_dir = imported_run_dir(context, run_id)
    html_report_path = report_path(context, run_id)
    issues: list[str] = []
    state = "missing"

    if not run_dir.exists():
        issues.append(f"run bundle not found: {run_dir}")
    elif not run_dir.is_dir():
        issues.append(f"run bundle path is not a directory: {run_dir}")
    else:
        inspection = inspect_run(run_dir)
        state = inspection.state
        issues.extend(_format_inspection_issue(issue) for issue in inspection.issues)

    if require_report:
        if not html_report_path.is_file():
            issues.append(f"report not found: {html_report_path}")
        else:
            issues.extend(_stale_report_issues(run_dir, html_report_path))

    return RunArtifactInspection(
        run_dir=run_dir,
        report_path=html_report_path,
        state=state,
        issues=tuple(dict.fromkeys(issues)),
        report_exists=html_report_path.is_file(),
    )


def build_pull_command(
    context: RunArtifactContext,
    *,
    run_id: str,
    overwrite: bool = True,
    progress: bool = False,
) -> list[str]:
    command = build_pull_run_command(
        repo_root=context.repo_root,
        connection=context.connection,
        local_import_root=context.local_import_root,
        run_id=run_id,
        overwrite=overwrite,
    )
    if progress:
        command.append("--progress")
    return command


def build_report_command(
    context: RunArtifactContext,
    *,
    run_id: str,
    overwrite: bool = True,
) -> list[str]:
    return build_omni_report_command(
        repo_root=context.repo_root,
        run_dir=imported_run_dir(context, run_id),
        overwrite=overwrite,
    )


def build_video_command(context: RunArtifactContext, *, run_id: str) -> list[str]:
    return build_omni_video_command(repo_root=context.repo_root, run_dir=imported_run_dir(context, run_id))


def build_run_videos(
    context: RunArtifactContext,
    *,
    run_id: str,
    command_executor: CommandExecutor = _default_command_executor,
) -> RunArtifactResult:
    command = build_video_command(context, run_id=run_id)
    video_dir = imported_run_dir(context, run_id) / "video"
    source_path = video_dir / "source.mp4"
    overlay_path = video_dir / "overlay.mp4"
    try:
        completed = command_executor(command, context.repo_root)
    except OSError as error:
        return RunArtifactResult(command=command, success=False, message=f"video build failed: {error}")
    if completed.returncode != 0:
        return RunArtifactResult(
            command=command,
            success=False,
            message=f"video build failed: {_completed_detail(completed)}",
        )
    if not source_path.is_file() or not overlay_path.is_file():
        return RunArtifactResult(
            command=command,
            success=False,
            message=f"video build completed but expected outputs are missing: {source_path}, {overlay_path}",
        )
    return RunArtifactResult(command=command, success=True, path=overlay_path, message=f"videos built: {overlay_path}")


def retrieve_run_artifacts(
    context: RunArtifactContext,
    *,
    run_id: str,
    overwrite: bool = True,
    command_executor: CommandExecutor = _default_command_executor,
    progress_callback: ProgressCallback | None = None,
    progress_command_executor: ProgressCommandExecutor = _default_progress_command_executor,
) -> RunArtifactResult:
    command = build_pull_command(context, run_id=run_id, overwrite=overwrite, progress=progress_callback is not None)
    path = imported_run_dir(context, run_id)
    try:
        if progress_callback is None:
            completed = command_executor(command, context.repo_root)
        else:

            def _on_output(output: str) -> None:
                progress = parse_rsync_progress_event(output)
                if progress is not None:
                    progress_callback(progress)

            completed = progress_command_executor(command, context.repo_root, _on_output)
    except OSError as error:
        return RunArtifactResult(command=command, success=False, message=f"retrieve failed: {error}")

    if completed.returncode != 0:
        detail = _completed_detail(completed)
        return RunArtifactResult(command=command, success=False, message=f"retrieve failed: {detail}")
    inspection = inspect_run_artifacts(context, run_id)
    if not path.is_dir():
        return RunArtifactResult(
            command=command,
            success=False,
            message=f"retrieve completed but run bundle is missing: {path}",
            path=path,
            issues=inspection.issues,
        )
    return RunArtifactResult(
        command=command,
        success=True,
        message=f"retrieved run: {path}",
        path=path,
        issues=inspection.issues,
    )


def generate_run_report(
    context: RunArtifactContext,
    *,
    run_id: str,
    overwrite: bool = True,
    command_executor: CommandExecutor = _default_command_executor,
) -> RunArtifactResult:
    command = build_report_command(context, run_id=run_id, overwrite=overwrite)
    path = report_path(context, run_id)
    try:
        completed = command_executor(command, context.repo_root)
    except OSError as error:
        return RunArtifactResult(command=command, success=False, message=f"failed to generate report: {error}")

    if completed.returncode != 0:
        detail = _completed_detail(completed)
        return RunArtifactResult(command=command, success=False, message=f"report generation failed: {detail}")
    inspection = inspect_run_artifacts(context, run_id, require_report=True)
    if not path.is_file():
        return RunArtifactResult(
            command=command,
            success=False,
            message=f"report generation completed but report is missing: {path}",
            path=path,
            issues=inspection.issues,
        )
    message = f"report generated: {path}"
    if inspection.issues:
        message = f"report generated with artifact warnings: {path}"
    return RunArtifactResult(
        command=command,
        success=True,
        message=message,
        path=path,
        issues=inspection.issues,
    )


def _format_inspection_issue(issue: InspectionIssue) -> str:
    if issue.path:
        return f"{issue.code}: {issue.message} ({issue.path})"
    return f"{issue.code}: {issue.message}"


def _stale_report_issues(run_dir: Path, html_report_path: Path) -> tuple[str, ...]:
    try:
        report_mtime = html_report_path.stat().st_mtime
    except OSError as error:
        return (f"report is not readable: {html_report_path}: {error}",)

    stale_against: list[str] = []
    for source_name in ("manifest.yaml", "summary.json"):
        source_path = run_dir / source_name
        if not source_path.exists():
            continue
        try:
            if source_path.stat().st_mtime > report_mtime:
                stale_against.append(source_name)
        except OSError as error:
            return (f"run artifact is not readable: {source_path}: {error}",)

    if stale_against:
        return (f"report may be stale; regenerate after updated {', '.join(stale_against)}",)
    return ()
