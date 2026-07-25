from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from robot_diag_control.run_artifacts import RunArtifactContext
from robot_diag_control.run_commands import (
    RUN_BACKEND_LABELS,
    RobotConnection,
    RunConfig,
    parse_run_classes,
    sanitize_run_id,
)

DEFAULT_REMOTE_REPO_ROOT = "/home/radxa/apps/omniseer"
DEFAULT_LOCAL_IMPORT_ROOT = "runs/imported"
DEFAULT_DEVCONTAINER_EXEC_TEMPLATE = "devcontainer exec --workspace-folder {remote_repo_root} bash -lc {command}"


@dataclass(frozen=True)
class RunFormValues:
    robot_host: str
    ssh_user: str
    remote_repo_root: str
    remote_runs_root: str
    local_import_root: str
    run_id: str
    backend_label: str
    classes_text: str
    notes: str
    devcontainer_exec_template: str


@dataclass(frozen=True)
class RunFormSelection:
    run_id: str
    connection: RobotConnection
    run_config: RunConfig
    artifact_context: RunArtifactContext


def default_remote_runs_root(remote_repo_root: str) -> str:
    return f"{normalized_remote_repo_root(remote_repo_root)}/runs"


def selected_run_backend(selected_label: str) -> str:
    for backend, label in RUN_BACKEND_LABELS.items():
        if selected_label == label:
            return backend
    raise ValueError(f"unsupported run backend: {selected_label}")


def normalized_remote_repo_root(value: str) -> str:
    return value.strip().rstrip("/") or DEFAULT_REMOTE_REPO_ROOT


def normalized_remote_runs_root(value: str, *, remote_repo_root: str) -> str:
    return value.strip().rstrip("/") or f"{remote_repo_root}/runs"


def local_import_root(value: str, *, repo_root: Path) -> Path:
    path = Path(value.strip() or DEFAULT_LOCAL_IMPORT_ROOT).expanduser()
    if path.is_absolute():
        return path
    return repo_root / path


def resolved_run_id(raw_run_id: str, *, default_run_id: Callable[[], str]) -> str:
    run_id = sanitize_run_id(raw_run_id)
    return run_id or default_run_id()


def resolve_run_form(
    values: RunFormValues,
    *,
    repo_root: Path,
    default_run_id: Callable[[], str],
) -> RunFormSelection:
    run_id = resolved_run_id(values.run_id, default_run_id=default_run_id)
    remote_repo_root = normalized_remote_repo_root(values.remote_repo_root)
    remote_runs_root = normalized_remote_runs_root(values.remote_runs_root, remote_repo_root=remote_repo_root)
    connection = RobotConnection(
        host=values.robot_host.strip(),
        ssh_user=values.ssh_user.strip(),
        remote_repo_root=remote_repo_root,
        remote_runs_root=remote_runs_root,
    )
    run_config = RunConfig(
        run_id=run_id,
        backend=selected_run_backend(values.backend_label),
        classes=tuple(parse_run_classes(values.classes_text)),
        notes=values.notes.strip(),
        devcontainer_exec_template=values.devcontainer_exec_template.strip() or DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
    )
    artifact_context = RunArtifactContext(
        repo_root=repo_root,
        connection=connection,
        local_import_root=local_import_root(values.local_import_root, repo_root=repo_root),
    )
    return RunFormSelection(
        run_id=run_id,
        connection=connection,
        run_config=run_config,
        artifact_context=artifact_context,
    )
