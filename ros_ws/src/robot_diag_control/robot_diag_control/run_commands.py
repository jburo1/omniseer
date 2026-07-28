from __future__ import annotations

import shlex
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

RUN_BACKEND_RUNTIME = "robot_runtime_container"
RUN_BACKEND_DEVCONTAINER = "robot_devcontainer"
RUN_BACKEND_LABELS = {
    RUN_BACKEND_RUNTIME: "Robot runtime container",
    RUN_BACKEND_DEVCONTAINER: "Robot devcontainer",
}
DEFAULT_DEVCONTAINER_EXEC_TEMPLATE = (
    "container=$(docker ps --filter label=devcontainer.local_folder={remote_repo_root} "
    "--format '{{{{.Names}}}}' | head -n 1); "
    'if [ -z "$container" ]; then '
    'echo "no running Omniseer devcontainer found for {remote_repo_root}" >&2; exit 127; '
    "fi; "
    'docker exec -it "$container" bash -lc {command}'
)
RUN_TYPE_PERCEPTION = "perception_recording"
RUN_TYPE_AUTONOMY_CENTER = "autonomy_center_first_class"
RUN_TYPE_LABELS = {
    RUN_TYPE_PERCEPTION: "Perception recording",
    RUN_TYPE_AUTONOMY_CENTER: "Autonomy: center first class",
}


@dataclass(frozen=True)
class RobotConnection:
    host: str
    ssh_user: str
    remote_repo_root: str
    remote_runs_root: str


@dataclass(frozen=True)
class RunConfig:
    run_id: str
    backend: str
    classes: tuple[str, ...] = ()
    notes: str = ""
    devcontainer_exec_template: str = ""
    run_type: str = RUN_TYPE_PERCEPTION


def sanitize_run_id(run_id: str) -> str:
    sanitized = "".join(char if char.isalnum() or char in {"_", "-", "."} else "_" for char in run_id.strip())
    return sanitized.strip("._-")


def parse_run_classes(raw_text: str) -> list[str]:
    classes: list[str] = []
    seen: set[str] = set()
    if "," in raw_text or "\n" in raw_text:
        raw_tokens = raw_text.replace("\n", ",").split(",")
    else:
        raw_tokens = raw_text.split()
    for token in raw_tokens:
        normalized = token.strip()
        if not normalized or normalized in seen:
            continue
        classes.append(normalized)
        seen.add(normalized)
    return classes


def remote_run_dir_for(remote_repo_root: str, run_id: str) -> str:
    return f"{remote_repo_root.rstrip('/')}/runs/{run_id}"


def remote_class_list_path_for(remote_repo_root: str, run_id: str) -> str:
    return f"{remote_run_dir_for(remote_repo_root, run_id)}/classes.txt"


def devcontainer_workspace_root_for(remote_repo_root: str) -> str:
    workspace_name = PurePosixPath(remote_repo_root.rstrip("/") or "/workspace").name
    return f"/{workspace_name or 'workspace'}"


def _runtime_container_class_list_path(run_id: str) -> str:
    return f"/runs/{run_id}/classes.txt"


def _runtime_container_run_dir(run_id: str) -> str:
    return f"/runs/{run_id}"


def _omni_command(repo_root: Path) -> str:
    return str(repo_root / "scripts" / "omni")


def _ssh_target(user: str, host: str) -> str:
    user = user.strip()
    host = host.strip()
    return f"{user}@{host}" if user else host


def shell_join(command: list[str]) -> str:
    return shlex.join(command)


def _build_remote_mkdir_command(*, ssh_user: str, host: str, remote_run_dir: str) -> list[str]:
    return ["ssh", _ssh_target(ssh_user, host), f"mkdir -p {shlex.quote(remote_run_dir)}"]


def build_remote_run_mkdir_command(*, connection: RobotConnection, run_config: RunConfig) -> list[str]:
    return _build_remote_mkdir_command(
        ssh_user=connection.ssh_user,
        host=connection.host,
        remote_run_dir=remote_run_dir_for(connection.remote_repo_root, run_config.run_id),
    )


def build_upload_classes_command(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
    local_path: Path,
) -> list[str]:
    remote_class_path = remote_class_list_path_for(connection.remote_repo_root, run_config.run_id)
    return ["scp", str(local_path), f"{_ssh_target(connection.ssh_user, connection.host)}:{remote_class_path}"]


def _build_runtime_record_inner_command(
    *,
    run_id: str,
    classes: Sequence[str],
    notes: str,
    run_type: str,
) -> list[str]:
    command = [
        "scripts/omni",
        "runtime",
        "record",
        "--run-id",
        run_id,
    ]
    if classes:
        command.extend(["--record-classes", ",".join(classes)])
    if notes.strip():
        command.extend(["--record-notes", notes.strip()])
    command.append("--")
    command.append("experiment_overwrite:=true")
    if classes:
        command.append(f"classes_path:={_runtime_container_class_list_path(run_id)}")
    command.extend(
        _autonomy_launch_args(
            classes=classes,
            run_type=run_type,
            run_dir=_runtime_container_run_dir(run_id),
        )
    )
    return command


def _build_devcontainer_record_inner_command(
    *,
    remote_repo_root: str,
    run_id: str,
    classes: Sequence[str],
    notes: str,
    run_type: str,
) -> list[str]:
    container_repo_root = devcontainer_workspace_root_for(remote_repo_root)
    container_run_dir = remote_run_dir_for(container_repo_root, run_id)
    command = [
        "scripts/omni",
        "run",
        "real",
        "--profile",
        "operator",
        "--record-run",
        run_id,
        "--record-out",
        container_run_dir,
        "--record-overwrite",
    ]
    if classes:
        command.extend(["--record-classes", ",".join(classes)])
    if notes.strip():
        command.extend(["--record-notes", notes.strip()])
    command.append("bringup")
    if classes:
        command.append(f"classes_path:={remote_class_list_path_for(container_repo_root, run_id)}")
    command.extend(_autonomy_launch_args(classes=classes, run_type=run_type, run_dir=container_run_dir))
    return command


def _autonomy_launch_args(*, classes: Sequence[str], run_type: str, run_dir: str) -> list[str]:
    if run_type == RUN_TYPE_PERCEPTION:
        return []
    if run_type != RUN_TYPE_AUTONOMY_CENTER:
        raise ValueError(f"unsupported run type: {run_type}")
    if not classes:
        raise ValueError("autonomy run requires at least one target class")
    target_class = classes[0]
    return [
        "start_autonomy:=true",
        f"autonomy_target_class:={target_class}",
        f"autonomy_run_dir:={run_dir}",
        "evidence_interval_sec:=0.25",
    ]


def _format_devcontainer_exec_template(
    *,
    template: str,
    command: str,
    remote_repo_root: str,
    run_id: str,
) -> str:
    values = {
        "command": shlex.quote(command),
        "remote_repo_root": shlex.quote(remote_repo_root),
        "remote_run_dir": shlex.quote(remote_run_dir_for(remote_repo_root, run_id)),
        "run_id": shlex.quote(run_id),
    }
    formatted = template.format(**values)
    if "{command}" not in template:
        formatted = f"{formatted} {shlex.quote(command)}"
    return formatted


def build_remote_start_command(
    *,
    connection: RobotConnection,
    run_config: RunConfig,
) -> list[str]:
    if run_config.backend == RUN_BACKEND_RUNTIME:
        inner = _build_runtime_record_inner_command(
            run_id=run_config.run_id,
            classes=run_config.classes,
            notes=run_config.notes,
            run_type=run_config.run_type,
        )
        remote_command = f"cd {shlex.quote(connection.remote_repo_root)} && {shell_join(inner)}"
    elif run_config.backend == RUN_BACKEND_DEVCONTAINER:
        container_repo_root = devcontainer_workspace_root_for(connection.remote_repo_root)
        inner = _build_devcontainer_record_inner_command(
            remote_repo_root=connection.remote_repo_root,
            run_id=run_config.run_id,
            classes=run_config.classes,
            notes=run_config.notes,
            run_type=run_config.run_type,
        )
        inner_shell = f"cd {shlex.quote(container_repo_root)} && {shell_join(inner)}"
        remote_command = _format_devcontainer_exec_template(
            template=run_config.devcontainer_exec_template,
            command=inner_shell,
            remote_repo_root=connection.remote_repo_root,
            run_id=run_config.run_id,
        )
    else:
        raise ValueError(f"unsupported run backend: {run_config.backend}")
    return ["ssh", "-tt", _ssh_target(connection.ssh_user, connection.host), remote_command]


def build_remote_runtime_stop_command(
    *,
    connection: RobotConnection,
    run_id: str,
) -> list[str]:
    inner = ["scripts/omni", "runtime", "stop", "--run-id", run_id]
    remote_command = f"cd {shlex.quote(connection.remote_repo_root)} && {shell_join(inner)}"
    return ["ssh", _ssh_target(connection.ssh_user, connection.host), remote_command]


def build_pull_run_command(
    *,
    repo_root: Path,
    connection: RobotConnection,
    local_import_root: Path,
    run_id: str,
    overwrite: bool = True,
) -> list[str]:
    command = [
        _omni_command(repo_root),
        "runs",
        "pull",
        run_id,
        "--host",
        connection.host,
        "--user",
        connection.ssh_user,
        "--remote-root",
        connection.remote_runs_root,
        "--import-root",
        str(local_import_root),
    ]
    if overwrite:
        command.append("--overwrite")
    return command


def local_import_dir_for(local_import_root: Path, run_id: str) -> Path:
    return local_import_root / run_id


def build_report_command(*, repo_root: Path, run_dir: Path, overwrite: bool = True) -> list[str]:
    command = [_omni_command(repo_root), "runs", "report", str(run_dir)]
    if overwrite:
        command.append("--overwrite")
    return command
