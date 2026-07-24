from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import webbrowser
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import grpc

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.gateway_client import (
    PROFILE_TO_PROTO,
    create_stub,
    format_operator_status,
    format_preview_response,
    format_system_status,
    format_teleop_response,
    get_system_status,
    send_teleop_command,
    set_preview_mode,
    set_teleop_enabled,
    target_for,
)

_TK_IMPORT_ERROR: ModuleNotFoundError | None = None
try:
    import tkinter as tk
    from tkinter import scrolledtext, ttk
except ModuleNotFoundError as error:  # pragma: no cover - exercised in runtime environments without Tk
    tk = None  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]
    scrolledtext = None  # type: ignore[assignment]
    _TK_IMPORT_ERROR = error

RUN_BACKEND_RUNTIME = "robot_runtime_container"
RUN_BACKEND_DEVCONTAINER = "robot_devcontainer"
RUN_BACKEND_LABELS = {
    RUN_BACKEND_RUNTIME: "Robot runtime container",
    RUN_BACKEND_DEVCONTAINER: "Robot devcontainer",
}
DEFAULT_REMOTE_REPO_ROOT = "/home/radxa/apps/omniseer"
DEFAULT_LOCAL_IMPORT_ROOT = "runs/imported"
DEFAULT_DEVCONTAINER_EXEC_TEMPLATE = "devcontainer exec --workspace-folder {remote_repo_root} bash -lc {command}"
DEFAULT_ROBOT_HOST = "192.168.1.178"
DEFAULT_ROBOT_USER = "radxa"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Minimal Tk monitor GUI for the robot gateway")
    parser.add_argument(
        "--host", default=os.environ.get("OMNISEER_ROBOT_HOST", DEFAULT_ROBOT_HOST), help="gRPC gateway host"
    )
    parser.add_argument("--port", type=int, default=50051, help="gRPC gateway port")
    parser.add_argument(
        "--preview-host",
        default=None,
        help="preview stream host; defaults to --host",
    )
    parser.add_argument("--preview-port", type=int, default=7100)
    parser.add_argument("--preview-latency-ms", type=int, default=125)
    parser.add_argument("--gst-launch-path", default="gst-launch-1.0")
    parser.add_argument(
        "--poll-interval-seconds",
        type=float,
        default=1.0,
        help="default watch interval",
    )
    parser.add_argument(
        "--refresh-on-start",
        action="store_true",
        help="fetch status once immediately after the window opens",
    )
    parser.add_argument(
        "--auto-close-seconds",
        type=float,
        default=None,
        help="optional auto-close timer for headless smoke tests",
    )
    parser.add_argument(
        "--repo-root",
        default=".",
        help="local repository root used for retrieval and report commands",
    )
    parser.add_argument(
        "--ssh-user", default=os.environ.get("OMNISEER_ROBOT_USER", DEFAULT_ROBOT_USER), help="robot SSH user"
    )
    parser.add_argument(
        "--remote-repo-root",
        default=os.environ.get("OMNISEER_ROBOT_REPO_ROOT", DEFAULT_REMOTE_REPO_ROOT),
        help="robot-side repository root",
    )
    parser.add_argument(
        "--remote-runs-root",
        default=os.environ.get("OMNISEER_ROBOT_RUNS_ROOT", ""),
        help="robot-side run bundle root; defaults to <remote-repo-root>/runs",
    )
    parser.add_argument(
        "--local-import-root",
        default=os.environ.get("OMNISEER_LOCAL_IMPORT_ROOT", DEFAULT_LOCAL_IMPORT_ROOT),
        help="local root for pulled robot run bundles",
    )
    parser.add_argument(
        "--run-backend",
        choices=tuple(RUN_BACKEND_LABELS),
        default=RUN_BACKEND_RUNTIME,
        help="robot-side run backend used by the run starter",
    )
    parser.add_argument(
        "--devcontainer-exec-template",
        default=os.environ.get("OMNISEER_DEVCONTAINER_EXEC_TEMPLATE", DEFAULT_DEVCONTAINER_EXEC_TEMPLATE),
        help="remote shell template for executing a command inside the robot devcontainer; use {command}",
    )
    return parser


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    return _build_parser().parse_args(sys.argv[1:] if args is None else args)


def _resolved_preview_host(parsed: argparse.Namespace) -> str:
    return parsed.preview_host or parsed.host


def _default_run_id() -> str:
    return "operator_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _sanitize_run_id(run_id: str) -> str:
    sanitized = "".join(char if char.isalnum() or char in {"_", "-", "."} else "_" for char in run_id.strip())
    return sanitized.strip("._-")


def _parse_run_classes(raw_text: str) -> list[str]:
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


def _remote_run_dir_for(remote_repo_root: str, run_id: str) -> str:
    return f"{remote_repo_root.rstrip('/')}/runs/{run_id}"


def _remote_class_list_path_for(remote_repo_root: str, run_id: str) -> str:
    return f"{_remote_run_dir_for(remote_repo_root, run_id)}/classes.txt"


def _runtime_container_class_list_path(run_id: str) -> str:
    return f"/runs/{run_id}/classes.txt"


def _omni_command(repo_root: Path) -> str:
    return str(repo_root / "scripts" / "omni")


def _ssh_target(user: str, host: str) -> str:
    user = user.strip()
    host = host.strip()
    return f"{user}@{host}" if user else host


def _shell_join(command: list[str]) -> str:
    return shlex.join(command)


def _build_remote_mkdir_command(*, ssh_user: str, host: str, remote_run_dir: str) -> list[str]:
    return ["ssh", _ssh_target(ssh_user, host), f"mkdir -p {shlex.quote(remote_run_dir)}"]


def _build_upload_classes_command(*, ssh_user: str, host: str, local_path: Path, remote_class_path: str) -> list[str]:
    return ["scp", str(local_path), f"{_ssh_target(ssh_user, host)}:{remote_class_path}"]


def _build_runtime_record_inner_command(
    *,
    run_id: str,
    classes: list[str],
    notes: str,
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
    if classes:
        command.append(f"classes_path:={_runtime_container_class_list_path(run_id)}")
    return command


def _build_devcontainer_record_inner_command(
    *,
    remote_repo_root: str,
    run_id: str,
    classes: list[str],
    notes: str,
) -> list[str]:
    remote_run_dir = _remote_run_dir_for(remote_repo_root, run_id)
    command = [
        "scripts/omni",
        "run",
        "real",
        "--profile",
        "operator",
        "--record-run",
        run_id,
        "--record-out",
        remote_run_dir,
        "--record-overwrite",
    ]
    if classes:
        command.extend(["--record-classes", ",".join(classes)])
    if notes.strip():
        command.extend(["--record-notes", notes.strip()])
    command.append("bringup")
    if classes:
        command.append(f"classes_path:={_remote_class_list_path_for(remote_repo_root, run_id)}")
    return command


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
        "remote_run_dir": shlex.quote(_remote_run_dir_for(remote_repo_root, run_id)),
        "run_id": shlex.quote(run_id),
    }
    formatted = template.format(**values)
    if "{command}" not in template:
        formatted = f"{formatted} {shlex.quote(command)}"
    return formatted


def _build_remote_start_command(
    *,
    backend: str,
    ssh_user: str,
    host: str,
    remote_repo_root: str,
    run_id: str,
    classes: list[str],
    notes: str,
    devcontainer_exec_template: str,
) -> list[str]:
    if backend == RUN_BACKEND_RUNTIME:
        inner = _build_runtime_record_inner_command(run_id=run_id, classes=classes, notes=notes)
        remote_command = f"cd {shlex.quote(remote_repo_root)} && {_shell_join(inner)}"
    elif backend == RUN_BACKEND_DEVCONTAINER:
        inner = _build_devcontainer_record_inner_command(
            remote_repo_root=remote_repo_root,
            run_id=run_id,
            classes=classes,
            notes=notes,
        )
        inner_shell = f"cd {shlex.quote(remote_repo_root)} && {_shell_join(inner)}"
        remote_command = _format_devcontainer_exec_template(
            template=devcontainer_exec_template,
            command=inner_shell,
            remote_repo_root=remote_repo_root,
            run_id=run_id,
        )
    else:
        raise ValueError(f"unsupported run backend: {backend}")
    return ["ssh", _ssh_target(ssh_user, host), remote_command]


def _build_pull_run_command(
    *,
    repo_root: Path,
    ssh_user: str,
    host: str,
    remote_runs_root: str,
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
        host,
        "--user",
        ssh_user,
        "--remote-root",
        remote_runs_root,
        "--import-root",
        str(local_import_root),
    ]
    if overwrite:
        command.append("--overwrite")
    return command


def _local_import_dir_for(local_import_root: Path, run_id: str) -> Path:
    return local_import_root / run_id


def _build_report_command(*, repo_root: Path, run_dir: Path, overwrite: bool = True) -> list[str]:
    command = [_omni_command(repo_root), "runs", "report", str(run_dir)]
    if overwrite:
        command.append("--overwrite")
    return command


def _build_preview_viewer_command(
    parsed: argparse.Namespace,
    *,
    profile_name: str,
    leave_preview_running: bool,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "robot_diag_control.preview_viewer",
        "--host",
        parsed.host,
        "--port",
        str(parsed.port),
        "--preview-host",
        _resolved_preview_host(parsed),
        "--preview-port",
        str(parsed.preview_port),
        "--preview-latency-ms",
        str(parsed.preview_latency_ms),
        "--gst-launch-path",
        parsed.gst_launch_path,
        "--profile",
        profile_name,
        "--mode",
        "display",
    ]
    if leave_preview_running:
        command.append("--leave-preview-running")
    return command


def _build_overlay_viewer_command(
    parsed: argparse.Namespace,
    *,
    profile_name: str,
    leave_preview_running: bool,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "robot_diag_control.overlay_viewer",
        "--host",
        parsed.host,
        "--port",
        str(parsed.port),
        "--preview-host",
        _resolved_preview_host(parsed),
        "--preview-port",
        str(parsed.preview_port),
        "--preview-latency-ms",
        str(parsed.preview_latency_ms),
        "--gst-launch-path",
        parsed.gst_launch_path,
        "--profile",
        profile_name,
        "--mode",
        "display",
    ]
    if leave_preview_running:
        command.append("--leave-preview-running")
    return command


def _teleop_command_for_action(
    action: str,
    *,
    linear_step_mps: float,
    angular_step_rad_s: float,
) -> tuple[float, float, float]:
    commands = {
        "forward": (linear_step_mps, 0.0, 0.0),
        "back": (-linear_step_mps, 0.0, 0.0),
        "left": (0.0, linear_step_mps, 0.0),
        "right": (0.0, -linear_step_mps, 0.0),
        "turn_left": (0.0, 0.0, angular_step_rad_s),
        "turn_right": (0.0, 0.0, -angular_step_rad_s),
        "stop": (0.0, 0.0, 0.0),
    }
    if action not in commands:
        raise ValueError(f"unknown teleop action: {action}")
    return commands[action]


class RobotMonitorGui:
    def __init__(self, root: Any, parsed: argparse.Namespace) -> None:
        self._root = root
        self._parsed = parsed
        self._watch_after_id: str | None = None
        self._viewer_process: subprocess.Popen[str] | None = None
        self._run_process: subprocess.Popen[str] | None = None
        self._active_run_id: str | None = None
        self._last_status: robot_gateway_pb2.SystemStatus | None = None
        self._last_fault_line: str | None = None

        self._host_var = tk.StringVar(value=parsed.host)
        self._port_var = tk.StringVar(value=str(parsed.port))
        self._preview_host_var = tk.StringVar(value=_resolved_preview_host(parsed))
        self._preview_port_var = tk.StringVar(value=str(parsed.preview_port))
        self._preview_latency_var = tk.StringVar(value=str(parsed.preview_latency_ms))
        self._gst_launch_path_var = tk.StringVar(value=parsed.gst_launch_path)
        self._poll_interval_var = tk.StringVar(value=str(parsed.poll_interval_seconds))
        self._profile_var = tk.StringVar(value="balanced")
        self._teleop_linear_step_var = tk.StringVar(value="0.12")
        self._teleop_angular_step_var = tk.StringVar(value="0.35")
        self._run_backend_var = tk.StringVar(value=RUN_BACKEND_LABELS[parsed.run_backend])
        self._ssh_user_var = tk.StringVar(value=parsed.ssh_user)
        self._remote_repo_root_var = tk.StringVar(value=parsed.remote_repo_root)
        remote_runs_root = parsed.remote_runs_root or f"{parsed.remote_repo_root.rstrip('/')}/runs"
        self._remote_runs_root_var = tk.StringVar(value=remote_runs_root)
        self._local_import_root_var = tk.StringVar(value=parsed.local_import_root)
        self._devcontainer_exec_template_var = tk.StringVar(value=parsed.devcontainer_exec_template)
        self._run_id_var = tk.StringVar(value=_default_run_id())
        self._run_classes_var = tk.StringVar(value="person")
        self._run_notes_text: Any | None = None
        self._action_var = tk.StringVar(value="Ready")

        self._root.title("Robot Monitor")
        self._root.geometry("920x680")
        self._root.minsize(820, 560)
        self._configure_style()
        self._build_layout()

        if parsed.refresh_on_start:
            self._root.after(50, self.refresh_status)
        if parsed.auto_close_seconds is not None:
            self._root.after(int(parsed.auto_close_seconds * 1000), self._root.destroy)

    def _configure_style(self) -> None:
        style = ttk.Style(self._root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("TkDefaultFont", 16, "bold"))

    def _build_layout(self) -> None:
        container = ttk.Frame(self._root, padding=14)
        container.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(container)
        header.pack(fill=tk.X)
        ttk.Label(header, text="Robot Monitor", style="Title.TLabel").pack(side=tk.LEFT)

        connection_frame = ttk.LabelFrame(container, text="Connection", padding=10)
        connection_frame.pack(fill=tk.X, pady=(12, 10))
        self._add_labeled_entry(connection_frame, "Host", self._host_var, 0, 0)
        self._add_labeled_entry(connection_frame, "Port", self._port_var, 0, 2, width=10)
        self._add_labeled_entry(connection_frame, "Preview Host", self._preview_host_var, 1, 0)
        self._add_labeled_entry(connection_frame, "Preview Port", self._preview_port_var, 1, 2, width=10)
        self._add_labeled_entry(connection_frame, "Latency ms", self._preview_latency_var, 2, 0, width=10)
        self._add_labeled_entry(connection_frame, "gst-launch", self._gst_launch_path_var, 2, 2)
        self._add_labeled_entry(connection_frame, "Poll Interval (s)", self._poll_interval_var, 3, 0, width=10)
        ttk.Label(connection_frame, text="Profile").grid(row=3, column=2, sticky=tk.W, pady=(8, 0))
        profile_box = ttk.Combobox(
            connection_frame,
            textvariable=self._profile_var,
            values=tuple(PROFILE_TO_PROTO),
            state="readonly",
            width=18,
        )
        profile_box.grid(row=3, column=3, sticky=tk.W, padx=(8, 0), pady=(8, 0))

        for column in range(4):
            connection_frame.columnconfigure(column, weight=1 if column in {1, 3} else 0)

        controls = ttk.Frame(container)
        controls.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(controls, text="Refresh", command=self.refresh_status).pack(side=tk.LEFT)
        ttk.Button(controls, text="Start Watch", command=self.start_watch).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Stop Watch", command=self.stop_watch).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Preview On", command=self.preview_on).pack(side=tk.LEFT, padx=(24, 0))
        ttk.Button(controls, text="Preview Off", command=self.preview_off).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Open Viewer", command=self.open_viewer).pack(side=tk.LEFT, padx=(24, 0))
        ttk.Button(controls, text="Open Overlay", command=self.open_overlay).pack(side=tk.LEFT, padx=(8, 0))

        body = ttk.Panedwindow(container, orient=tk.VERTICAL)
        body.pack(fill=tk.BOTH, expand=True)

        teleop_frame = ttk.LabelFrame(body, text="Teleop", padding=8)
        self._build_teleop_controls(teleop_frame)
        body.add(teleop_frame, weight=1)

        run_frame = ttk.LabelFrame(body, text="Perception Run", padding=8)
        self._build_run_controls(run_frame)
        body.add(run_frame, weight=1)

        status_frame = ttk.LabelFrame(body, text="Operator Overview", padding=8)
        self._status_text = scrolledtext.ScrolledText(
            status_frame,
            wrap=tk.WORD,
            height=14,
            state=tk.DISABLED,
        )
        self._status_text.pack(fill=tk.BOTH, expand=True)
        body.add(status_frame, weight=3)

        log_frame = ttk.LabelFrame(body, text="Activity", padding=8)
        self._log_text = scrolledtext.ScrolledText(
            log_frame,
            wrap=tk.WORD,
            height=10,
            state=tk.DISABLED,
        )
        self._log_text.pack(fill=tk.BOTH, expand=True)
        body.add(log_frame, weight=2)

        footer = ttk.Frame(container)
        footer.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(footer, textvariable=self._action_var).pack(side=tk.LEFT)

        self._root.bind("<KeyPress-w>", lambda _event: self.send_teleop_action("forward"))
        self._root.bind("<KeyPress-s>", lambda _event: self.send_teleop_action("back"))
        self._root.bind("<KeyPress-a>", lambda _event: self.send_teleop_action("left"))
        self._root.bind("<KeyPress-d>", lambda _event: self.send_teleop_action("right"))
        self._root.bind("<KeyPress-q>", lambda _event: self.send_teleop_action("turn_left"))
        self._root.bind("<KeyPress-e>", lambda _event: self.send_teleop_action("turn_right"))
        self._root.bind("<space>", lambda _event: self.send_teleop_action("stop"))

    def _build_teleop_controls(self, parent: Any) -> None:
        settings = ttk.Frame(parent)
        settings.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 14))
        self._add_labeled_entry(settings, "Linear step", self._teleop_linear_step_var, 0, 0, width=8)
        self._add_labeled_entry(settings, "Angular step", self._teleop_angular_step_var, 1, 0, width=8)
        ttk.Button(settings, text="Enable", command=self.teleop_enable).grid(
            row=2, column=0, sticky=tk.EW, pady=(10, 0)
        )
        ttk.Button(settings, text="Disable", command=self.teleop_disable).grid(
            row=2, column=1, sticky=tk.EW, padx=(8, 0), pady=(10, 0)
        )

        pad = ttk.Frame(parent)
        pad.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Button(pad, text="Forward", command=lambda: self.send_teleop_action("forward")).grid(
            row=0, column=1, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Left", command=lambda: self.send_teleop_action("left")).grid(
            row=1, column=0, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Stop", command=lambda: self.send_teleop_action("stop")).grid(
            row=1, column=1, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Right", command=lambda: self.send_teleop_action("right")).grid(
            row=1, column=2, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Back", command=lambda: self.send_teleop_action("back")).grid(
            row=2, column=1, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Turn Left", command=lambda: self.send_teleop_action("turn_left")).grid(
            row=0, column=0, sticky=tk.EW, padx=4, pady=4
        )
        ttk.Button(pad, text="Turn Right", command=lambda: self.send_teleop_action("turn_right")).grid(
            row=0, column=2, sticky=tk.EW, padx=4, pady=4
        )
        for column in range(3):
            pad.columnconfigure(column, weight=1)

    def _build_run_controls(self, parent: Any) -> None:
        form = ttk.Frame(parent)
        form.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 14))
        ttk.Label(form, text="Backend").grid(row=0, column=0, sticky=tk.W, pady=(8, 0))
        backend_box = ttk.Combobox(
            form,
            textvariable=self._run_backend_var,
            values=tuple(RUN_BACKEND_LABELS.values()),
            state="readonly",
        )
        backend_box.grid(row=0, column=1, sticky=tk.EW, padx=(8, 0), pady=(8, 0))
        self._add_labeled_entry(form, "SSH User", self._ssh_user_var, 0, 2, width=12)
        self._add_labeled_entry(form, "Remote Repo", self._remote_repo_root_var, 1, 0)
        self._add_labeled_entry(form, "Remote Runs", self._remote_runs_root_var, 1, 2)
        self._add_labeled_entry(form, "Local Import", self._local_import_root_var, 2, 0)
        self._add_labeled_entry(form, "Devcontainer Exec", self._devcontainer_exec_template_var, 2, 2)
        self._add_labeled_entry(form, "Run ID", self._run_id_var, 3, 0)
        self._add_labeled_entry(form, "Classes", self._run_classes_var, 3, 2)
        ttk.Label(form, text="Notes").grid(row=4, column=0, sticky=tk.NW, pady=(8, 0))
        self._run_notes_text = tk.Text(form, height=3, width=40, wrap=tk.WORD)
        self._run_notes_text.grid(row=4, column=1, columnspan=3, sticky=tk.EW, padx=(8, 0), pady=(8, 0))
        for column in range(4):
            form.columnconfigure(column, weight=1 if column in {1, 3} else 0)

        actions = ttk.Frame(parent)
        actions.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(actions, text="New ID", command=self.new_run_id).pack(fill=tk.X)
        ttk.Button(actions, text="Start Run", command=self.start_run).pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions, text="Stop Run", command=self.stop_run).pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions, text="Retrieve Results", command=self.retrieve_results).pack(fill=tk.X, pady=(18, 0))
        ttk.Button(actions, text="Generate Report", command=self.generate_report).pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions, text="Open Report", command=self.open_report).pack(fill=tk.X, pady=(8, 0))

    def _add_labeled_entry(
        self,
        parent: Any,
        label: str,
        variable: Any,
        row: int,
        column: int,
        *,
        width: int | None = None,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky=tk.W, pady=(8, 0))
        entry = ttk.Entry(parent, textvariable=variable, width=width)
        entry.grid(row=row, column=column + 1, sticky=tk.EW, padx=(8, 0), pady=(8, 0))

    def _connection_namespace(self) -> argparse.Namespace:
        return argparse.Namespace(
            host=self._host_var.get().strip(),
            port=int(self._port_var.get().strip()),
            preview_host=self._preview_host_var.get().strip() or None,
            preview_port=int(self._preview_port_var.get().strip()),
            preview_latency_ms=int(self._preview_latency_var.get().strip()),
            gst_launch_path=self._gst_launch_path_var.get().strip(),
            poll_interval_seconds=float(self._poll_interval_var.get().strip()),
        )

    def _repo_root(self) -> Path:
        return Path(self._parsed.repo_root).expanduser().resolve()

    def _selected_run_backend(self) -> str:
        selected = self._run_backend_var.get()
        for backend, label in RUN_BACKEND_LABELS.items():
            if selected == label:
                return backend
        raise ValueError(f"unsupported run backend: {selected}")

    def _remote_repo_root(self) -> str:
        return self._remote_repo_root_var.get().strip().rstrip("/") or DEFAULT_REMOTE_REPO_ROOT

    def _remote_runs_root(self) -> str:
        return self._remote_runs_root_var.get().strip().rstrip("/") or f"{self._remote_repo_root()}/runs"

    def _local_import_root(self) -> Path:
        value = Path(self._local_import_root_var.get().strip() or DEFAULT_LOCAL_IMPORT_ROOT).expanduser()
        if value.is_absolute():
            return value
        return self._repo_root() / value

    def _append_log_threadsafe(self, message: str) -> None:
        self._root.after(0, lambda: self._append_log(message))

    def _stub(self) -> Any:
        settings = self._connection_namespace()
        channel = grpc.insecure_channel(target_for(settings.host, settings.port))
        return channel, create_stub(channel)

    def _append_log(self, message: str) -> None:
        self._log_text.configure(state=tk.NORMAL)
        self._log_text.insert(tk.END, message + "\n")
        self._log_text.see(tk.END)
        self._log_text.configure(state=tk.DISABLED)
        self._action_var.set(message)

    def _set_status_text(self, text: str) -> None:
        self._status_text.configure(state=tk.NORMAL)
        self._status_text.delete("1.0", tk.END)
        self._status_text.insert("1.0", text)
        self._status_text.configure(state=tk.DISABLED)

    def _run_notes(self) -> str:
        if self._run_notes_text is None:
            return ""
        return self._run_notes_text.get("1.0", tk.END).strip()

    def _resolved_run_id(self) -> str:
        run_id = _sanitize_run_id(self._run_id_var.get())
        if not run_id:
            run_id = _default_run_id()
            self._run_id_var.set(run_id)
        return run_id

    def new_run_id(self) -> None:
        self._run_id_var.set(_default_run_id())
        self._append_log("new run id generated")

    def start_run(self) -> None:
        if self._run_process is not None and self._run_process.poll() is None:
            self._append_log("run already running")
            return

        try:
            run_id = self._resolved_run_id()
            classes = _parse_run_classes(self._run_classes_var.get())
            backend = self._selected_run_backend()
            settings = self._connection_namespace()
            ssh_user = self._ssh_user_var.get().strip()
            remote_repo_root = self._remote_repo_root()
            remote_run_dir = _remote_run_dir_for(remote_repo_root, run_id)
            remote_class_path = _remote_class_list_path_for(remote_repo_root, run_id)
            mkdir_command = _build_remote_mkdir_command(
                ssh_user=ssh_user,
                host=settings.host,
                remote_run_dir=remote_run_dir,
            )
            self._run_command_checked(mkdir_command, "create remote run directory")
            if classes:
                self._upload_classes_file(
                    ssh_user=ssh_user,
                    host=settings.host,
                    remote_class_path=remote_class_path,
                    classes=classes,
                )
            command = _build_remote_start_command(
                backend=backend,
                ssh_user=ssh_user,
                host=settings.host,
                remote_repo_root=remote_repo_root,
                run_id=run_id,
                classes=classes,
                notes=self._run_notes(),
                devcontainer_exec_template=self._devcontainer_exec_template_var.get().strip()
                or DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
            )
        except (OSError, ValueError) as error:
            self._append_log(f"failed to prepare run: {error}")
            return

        try:
            self._run_process = subprocess.Popen(
                command,
                cwd=self._repo_root(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                bufsize=1,
            )
        except OSError as error:
            self._append_log(f"failed to start run: {error}")
            return

        self._active_run_id = run_id
        class_text = ", ".join(classes) if classes else "from config"
        self._append_log(f"run started remotely: {run_id}; backend={RUN_BACKEND_LABELS[backend]}; classes={class_text}")
        self._append_log("$ " + _shell_join(command))
        self._start_run_log_reader()
        self._poll_run_process()

    def _run_command_checked(self, command: list[str], action: str) -> None:
        completed = subprocess.run(
            command,
            cwd=self._repo_root(),
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            raise OSError(f"{action} failed: {detail}")

    def _upload_classes_file(self, *, ssh_user: str, host: str, remote_class_path: str, classes: list[str]) -> None:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as handle:
            local_path = Path(handle.name)
            handle.write("\n".join(classes) + "\n")
        try:
            upload_command = _build_upload_classes_command(
                ssh_user=ssh_user,
                host=host,
                local_path=local_path,
                remote_class_path=remote_class_path,
            )
            self._run_command_checked(upload_command, "upload classes")
        finally:
            try:
                local_path.unlink()
            except OSError:
                pass

    def _start_run_log_reader(self) -> None:
        process = self._run_process
        if process is None or process.stdout is None:
            return

        def _read_output() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self._append_log_threadsafe(line.rstrip())

        threading.Thread(target=_read_output, name="omniseer_remote_run_log", daemon=True).start()

    def stop_run(self) -> None:
        if self._run_process is None or self._run_process.poll() is not None:
            self._append_log("run not running")
            return

        try:
            os.killpg(self._run_process.pid, signal.SIGINT)
        except OSError:
            self._run_process.terminate()
        self._append_log("run stop requested")

    def _poll_run_process(self) -> None:
        if self._run_process is None:
            return
        exit_code = self._run_process.poll()
        if exit_code is None:
            self._root.after(1000, self._poll_run_process)
            return

        run_id = self._active_run_id or self._resolved_run_id()
        self._append_log(f"remote run exited with code {exit_code}: {run_id}")
        self._run_process = None

    def retrieve_results(self) -> None:
        run_id = self._active_run_id or self._resolved_run_id()
        settings = self._connection_namespace()
        command = _build_pull_run_command(
            repo_root=self._repo_root(),
            ssh_user=self._ssh_user_var.get().strip(),
            host=settings.host,
            remote_runs_root=self._remote_runs_root(),
            local_import_root=self._local_import_root(),
            run_id=run_id,
            overwrite=True,
        )
        self._append_log("$ " + _shell_join(command))
        completed = subprocess.run(
            command,
            cwd=self._repo_root(),
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            self._append_log(f"retrieve failed: {detail}")
            return
        self._append_log(f"retrieved run: {_local_import_dir_for(self._local_import_root(), run_id)}")
        self.generate_report()

    def generate_report(self) -> None:
        run_id = self._active_run_id or self._resolved_run_id()
        repo_root = self._repo_root()
        local_run_dir = _local_import_dir_for(self._local_import_root(), run_id)
        command = _build_report_command(repo_root=repo_root, run_dir=local_run_dir, overwrite=True)
        try:
            completed = subprocess.run(
                command,
                cwd=repo_root,
                check=False,
                capture_output=True,
                text=True,
            )
        except OSError as error:
            self._append_log(f"failed to generate report: {error}")
            return

        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            self._append_log(f"report generation failed: {detail}")
            return

        report_path = local_run_dir / "report" / "index.html"
        self._append_log(f"report generated: {report_path}")
        self.open_report()

    def open_report(self) -> None:
        run_id = self._active_run_id or self._resolved_run_id()
        report_path = _local_import_dir_for(self._local_import_root(), run_id) / "report" / "index.html"
        if not report_path.exists():
            self._append_log(f"report not found: {report_path}")
            return
        webbrowser.open(report_path.as_uri())
        self._append_log(f"report opened: {report_path}")

    def refresh_status(self) -> None:
        try:
            with grpc.insecure_channel(
                target_for(self._connection_namespace().host, self._connection_namespace().port)
            ) as channel:
                status = get_system_status(create_stub(channel))
        except (grpc.RpcError, ValueError) as error:
            self._append_log(f"status refresh failed: {error}")
            return

        self._last_status = status
        operator_status = format_operator_status(status)
        self._set_status_text(operator_status + "\n\nRaw status\n" + format_system_status(status))
        fault_line = operator_status.splitlines()[-1] if operator_status else None
        if fault_line and fault_line != "FAULT none" and fault_line != self._last_fault_line:
            self._append_log(fault_line)
        self._last_fault_line = fault_line
        self._append_log("status refreshed")

    def start_watch(self) -> None:
        if self._watch_after_id is not None:
            self._append_log("watch already running")
            return
        self._append_log("watch started")
        self._watch_tick()

    def _watch_tick(self) -> None:
        self.refresh_status()
        interval_ms = int(self._connection_namespace().poll_interval_seconds * 1000)
        self._watch_after_id = self._root.after(max(interval_ms, 100), self._watch_tick)

    def stop_watch(self) -> None:
        if self._watch_after_id is None:
            self._append_log("watch not running")
            return
        self._root.after_cancel(self._watch_after_id)
        self._watch_after_id = None
        self._append_log("watch stopped")

    def preview_on(self) -> None:
        self._set_preview_mode(True)

    def preview_off(self) -> None:
        self._set_preview_mode(False)

    def _set_preview_mode(self, enabled: bool) -> None:
        profile_name = self._profile_var.get() if enabled else None
        try:
            settings = self._connection_namespace()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                response = set_preview_mode(
                    create_stub(channel),
                    enabled=enabled,
                    profile_name=profile_name,
                )
        except (grpc.RpcError, ValueError) as error:
            self._append_log(f"preview request failed: {error}")
            return

        self._append_log(format_preview_response(response))
        self.refresh_status()

    def teleop_enable(self) -> None:
        self._set_teleop_enabled(True)

    def teleop_disable(self) -> None:
        self._set_teleop_enabled(False)

    def _set_teleop_enabled(self, enabled: bool) -> None:
        try:
            with grpc.insecure_channel(
                target_for(self._connection_namespace().host, self._connection_namespace().port)
            ) as channel:
                response = set_teleop_enabled(create_stub(channel), enabled=enabled)
        except (grpc.RpcError, ValueError) as error:
            self._append_log(f"teleop request failed: {error}")
            return

        self._append_log(format_teleop_response(response))
        self.refresh_status()

    def send_teleop_action(self, action: str) -> None:
        try:
            linear_x, linear_y, angular_z = _teleop_command_for_action(
                action,
                linear_step_mps=float(self._teleop_linear_step_var.get().strip()),
                angular_step_rad_s=float(self._teleop_angular_step_var.get().strip()),
            )
            with grpc.insecure_channel(
                target_for(self._connection_namespace().host, self._connection_namespace().port)
            ) as channel:
                response = send_teleop_command(
                    create_stub(channel),
                    linear_x_mps=linear_x,
                    linear_y_mps=linear_y,
                    angular_z_rad_s=angular_z,
                )
        except (grpc.RpcError, ValueError) as error:
            self._append_log(f"teleop command failed: {error}")
            return

        self._append_log(format_teleop_response(response))
        self.refresh_status()

    def close(self) -> None:
        if self._run_process is not None and self._run_process.poll() is None:
            try:
                os.killpg(self._run_process.pid, signal.SIGINT)
            except OSError:
                self._run_process.terminate()
        try:
            with grpc.insecure_channel(
                target_for(self._connection_namespace().host, self._connection_namespace().port)
            ) as channel:
                set_teleop_enabled(create_stub(channel), enabled=False)
        except (grpc.RpcError, ValueError):
            pass
        self._root.destroy()

    def open_viewer(self) -> None:
        self._open_viewer_process("viewer", _build_preview_viewer_command)

    def open_overlay(self) -> None:
        self._open_viewer_process("overlay", _build_overlay_viewer_command)

    def _open_viewer_process(self, label: str, command_builder: Any) -> None:
        if self._viewer_process is not None and self._viewer_process.poll() is None:
            self._append_log("viewer already running")
            return

        settings = self._connection_namespace()
        profile_name = self._profile_var.get()
        leave_preview_running = (
            self._last_status is not None and self._last_status.preview.state == robot_gateway_pb2.PREVIEW_RUNNING
        )
        command = command_builder(
            argparse.Namespace(
                host=settings.host,
                port=settings.port,
                preview_host=settings.preview_host,
                preview_port=settings.preview_port,
                preview_latency_ms=settings.preview_latency_ms,
                gst_launch_path=settings.gst_launch_path,
            ),
            profile_name=profile_name,
            leave_preview_running=leave_preview_running,
        )

        try:
            self._viewer_process = subprocess.Popen(command)
        except OSError as error:
            self._append_log(f"failed to launch {label}: {error}")
            return

        self._append_log(f"{label} launched with pid={self._viewer_process.pid}")
        self._poll_viewer_process()

    def _poll_viewer_process(self) -> None:
        if self._viewer_process is None:
            return
        exit_code = self._viewer_process.poll()
        if exit_code is None:
            self._root.after(500, self._poll_viewer_process)
            return

        self._append_log(f"viewer exited with code {exit_code}")
        self._viewer_process = None
        self.refresh_status()


def main(args: list[str] | None = None) -> int:
    if _TK_IMPORT_ERROR is not None:
        print(
            "Tk is not available in this Python environment. Install python3-tk "
            "or rebuild the devcontainer after the Dockerfile update.",
            file=sys.stderr,
        )
        return 1

    parsed = parse_args(args)
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(
            f"failed to start Tk GUI: {error}. Use a desktop session or xvfb-run for headless smoke tests.",
            file=sys.stderr,
        )
        return 1

    app = RobotMonitorGui(root, parsed)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
    app.stop_watch() if app._watch_after_id is not None else None
    if app._viewer_process is not None and app._viewer_process.poll() is None:
        app._viewer_process.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
