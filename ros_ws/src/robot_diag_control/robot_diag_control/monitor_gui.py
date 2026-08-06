from __future__ import annotations

import argparse
import os
import subprocess
import sys
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
from robot_diag_control.monitor_settings import (
    MonitorConnectionSettings,
    MonitorConnectionValues,
    TeleopStepSettings,
    TeleopStepValues,
    resolve_monitor_connection,
    resolve_teleop_steps,
)
from robot_diag_control.run_artifacts import (
    RunArtifactContext,
    RunArtifactResult,
    build_run_videos,
    generate_run_report,
    inspect_run_artifacts,
    report_path,
    retrieve_run_artifacts,
)
from robot_diag_control.run_commands import (
    RUN_BACKEND_LABELS,
    RUN_BACKEND_RUNTIME,
    RUN_TYPE_AUTONOMY_CENTER,
    RUN_TYPE_LABELS,
    RUN_TYPE_PERCEPTION,
    RobotConnection,
    RunConfig,
    sanitize_run_id,
    shell_join,
)
from robot_diag_control.run_lifecycle import (
    RemoteRunProcess,
    RunPhase,
    RunState,
    remote_run_is_running,
    run_control_availability,
    run_state,
    start_remote_run_log_reader,
)
from robot_diag_control.run_manager import RunManager
from robot_diag_control.run_settings import (
    DEFAULT_DEVCONTAINER_EXEC_TEMPLATE,
    DEFAULT_LOCAL_IMPORT_ROOT,
    DEFAULT_REMOTE_REPO_ROOT,
    RunFormSelection,
    RunFormValues,
    default_remote_runs_root,
    resolve_run_form,
    selected_run_backend,
    selected_run_type,
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


if ttk is not None:

    class CollapsibleSection(ttk.Frame):
        def __init__(self, parent: Any, title: str, *, padding: int = 0, expanded: bool = True) -> None:
            super().__init__(parent)
            self._title = title
            self._expanded = expanded
            self._toggle_button = ttk.Button(self, text=self._button_text(), command=self.toggle)
            self._toggle_button.pack(fill=tk.X)
            self.body = ttk.Frame(self, padding=padding)
            if self._expanded:
                self.body.pack(fill=tk.BOTH, expand=True)

        @property
        def expanded(self) -> bool:
            return self._expanded

        def toggle(self) -> None:
            self.set_expanded(not self._expanded)

        def set_expanded(self, expanded: bool) -> None:
            if expanded == self._expanded:
                return
            self._expanded = expanded
            self._toggle_button.configure(text=self._button_text())
            if expanded:
                self.body.pack(fill=tk.BOTH, expand=True)
            else:
                self.body.pack_forget()

        def _button_text(self) -> str:
            prefix = "[-]" if self._expanded else "[+]"
            return f"{prefix} {self._title}"

else:

    class CollapsibleSection:  # type: ignore[no-redef]
        pass


DEFAULT_ROBOT_HOST = "192.168.1.178"
DEFAULT_ROBOT_USER = "radxa"
RUN_STOP_GRACE_MS = 8000


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


def _format_request_error(action: str, error: grpc.RpcError | ValueError) -> str:
    if isinstance(error, grpc.RpcError):
        code = error.code()
        details = error.details()
        code_name = code.name if code is not None else "UNKNOWN"
        message = f"{action} failed: {code_name}"
        if details:
            message = f"{message}; {details}"
        return message
    return f"{action} failed: {error}"


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
        self._run_manager = RunManager(repo_root=self._repo_root())
        self._watch_after_id: str | None = None
        self._viewer_process: subprocess.Popen[str] | None = None
        self._run_process: RemoteRunProcess | None = None
        self._run_state: RunState = run_state(RunPhase.IDLE)
        self._active_run_id: str | None = None
        self._run_stop_after_id: str | None = None
        self._run_poll_after_id: str | None = None
        self._runtime_stop_pending_run_id: str | None = None
        self._run_generation = 0
        self._run_buttons: dict[str, Any] = {}
        self._sections: dict[str, CollapsibleSection] = {}
        self._teleop_enabled = False
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
        self._run_type_var = tk.StringVar(value=RUN_TYPE_LABELS[RUN_TYPE_PERCEPTION])
        self._ssh_user_var = tk.StringVar(value=parsed.ssh_user)
        self._remote_repo_root_var = tk.StringVar(value=parsed.remote_repo_root)
        remote_runs_root = parsed.remote_runs_root or default_remote_runs_root(parsed.remote_repo_root)
        self._remote_runs_root_var = tk.StringVar(value=remote_runs_root)
        self._local_import_root_var = tk.StringVar(value=parsed.local_import_root)
        self._devcontainer_exec_template_var = tk.StringVar(value=parsed.devcontainer_exec_template)
        self._run_id_var = tk.StringVar(value=_default_run_id())
        self._run_classes_var = tk.StringVar(value="person")
        self._record_video_var = tk.BooleanVar(value=False)
        self._record_rosbag_var = tk.BooleanVar(value=False)
        self._autonomy_bbox_area_min_ratio_var = tk.StringVar(value="0.08")
        self._autonomy_bbox_area_max_ratio_var = tk.StringVar(value="0.35")
        self._autonomy_forward_speed_var = tk.StringVar(value="0.05")
        self._autonomy_reverse_speed_var = tk.StringVar(value="0.04")
        self._autonomy_stable_frames_var = tk.StringVar(value="10")
        self._autonomy_proximity_stop_var = tk.StringVar(value="0.30")
        self._autonomy_capture_timeout_var = tk.StringVar(value="2.0")
        self._autonomy_min_target_confidence_var = tk.StringVar(value="0.50")
        self._autonomy_max_target_center_jump_ratio_var = tk.StringVar(value="0.20")
        self._autonomy_evidence_interval_var = tk.StringVar(value="0.25")
        self._detector_score_threshold_var = tk.StringVar(value="0.25")
        self._detector_nms_iou_threshold_var = tk.StringVar(value="0.45")
        self._detector_max_detections_var = tk.StringVar(value="100")
        self._run_notes_text: Any | None = None
        self._run_experiment_frames: dict[str, Any] = {}
        self._mode_var = tk.StringVar(value=self._format_run_mode(self._run_state))

        self._root.title("Robot Monitor")
        self._root.geometry("920x780")
        self._root.minsize(820, 640)
        self._configure_style()
        self._build_layout()
        self._sync_run_experiment_fields()
        self._update_run_controls()

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
        ttk.Label(header, text="Mode").pack(side=tk.LEFT, padx=(24, 4))
        ttk.Label(header, textvariable=self._mode_var, anchor=tk.W, width=44).pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True,
        )

        connection_section = CollapsibleSection(container, "Connection", padding=10)
        connection_section.pack(fill=tk.X, pady=(12, 10))
        self._sections["connection"] = connection_section
        connection_frame = connection_section.body
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

        actions_section = CollapsibleSection(container, "Actions", padding=8, expanded=False)
        actions_section.pack(fill=tk.X, pady=(0, 10))
        self._sections["actions"] = actions_section
        controls = actions_section.body
        ttk.Button(controls, text="Refresh", command=self.refresh_status).pack(side=tk.LEFT)
        ttk.Button(controls, text="Start Watch", command=self.start_watch).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Stop Watch", command=self.stop_watch).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Preview On", command=self.preview_on).pack(side=tk.LEFT, padx=(24, 0))
        ttk.Button(controls, text="Preview Off", command=self.preview_off).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Open Viewer", command=self.open_viewer).pack(side=tk.LEFT, padx=(24, 0))
        ttk.Button(controls, text="Open Overlay", command=self.open_overlay).pack(side=tk.LEFT, padx=(8, 0))

        body = ttk.Panedwindow(container, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)

        control_column = ttk.Frame(body)
        review_column = ttk.Frame(body)
        body.add(control_column, weight=2)
        body.add(review_column, weight=3)

        teleop_section = CollapsibleSection(control_column, "Teleop", padding=8, expanded=False)
        self._sections["teleop"] = teleop_section
        self._build_teleop_controls(teleop_section.body)
        teleop_section.pack(fill=tk.X)

        run_section = CollapsibleSection(control_column, "Run", padding=8)
        self._sections["run"] = run_section
        self._build_run_controls(run_section.body)
        run_section.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        status_section = CollapsibleSection(review_column, "Operator Overview", padding=8)
        self._sections["status"] = status_section
        self._status_text = scrolledtext.ScrolledText(
            status_section.body,
            wrap=tk.WORD,
            height=8,
            state=tk.DISABLED,
        )
        self._status_text.pack(fill=tk.BOTH, expand=True)
        status_section.pack(fill=tk.BOTH, expand=True)

        log_section = CollapsibleSection(review_column, "Activity", padding=8)
        self._sections["log"] = log_section
        self._log_text = scrolledtext.ScrolledText(
            log_section.body,
            wrap=tk.WORD,
            height=6,
            state=tk.DISABLED,
        )
        self._log_text.pack(fill=tk.BOTH, expand=True)
        log_section.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

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

        ttk.Separator(form).grid(row=3, column=0, columnspan=4, sticky=tk.EW, pady=(8, 0))
        experiment_holder = ttk.Frame(form)
        experiment_holder.grid(row=4, column=0, columnspan=4, sticky=tk.EW, pady=(8, 0))
        ttk.Label(experiment_holder, text="Experiment").grid(row=0, column=0, sticky=tk.W)
        run_type_box = ttk.Combobox(
            experiment_holder,
            textvariable=self._run_type_var,
            values=tuple(RUN_TYPE_LABELS.values()),
            state="readonly",
        )
        run_type_box.grid(row=0, column=1, columnspan=3, sticky=tk.EW, padx=(8, 0))
        run_type_box.bind("<<ComboboxSelected>>", lambda _event: self._sync_run_experiment_fields())

        self._add_labeled_entry(experiment_holder, "Class List", self._run_classes_var, 1, 0)
        ttk.Checkbutton(experiment_holder, text="Record video", variable=self._record_video_var).grid(
            row=1, column=2, sticky=tk.W, pady=(8, 0)
        )
        ttk.Checkbutton(experiment_holder, text="Record rosbag", variable=self._record_rosbag_var).grid(
            row=1, column=3, sticky=tk.W, pady=(8, 0)
        )
        self._add_labeled_entry(
            experiment_holder,
            "Score Threshold",
            self._detector_score_threshold_var,
            2,
            0,
            width=8,
        )
        self._add_labeled_entry(
            experiment_holder,
            "NMS IoU",
            self._detector_nms_iou_threshold_var,
            2,
            2,
            width=8,
        )
        self._add_labeled_entry(
            experiment_holder,
            "Max Detections",
            self._detector_max_detections_var,
            3,
            0,
            width=8,
        )

        experiment_fields = ttk.Frame(experiment_holder)
        experiment_fields.grid(row=4, column=0, columnspan=4, sticky=tk.EW)
        perception_frame = ttk.Frame(experiment_fields)
        perception_frame.columnconfigure(1, weight=1)
        self._run_experiment_frames[RUN_TYPE_PERCEPTION] = perception_frame

        autonomy_frame = ttk.Frame(experiment_fields)
        self._add_labeled_entry(autonomy_frame, "BBox Min Area", self._autonomy_bbox_area_min_ratio_var, 0, 0, width=8)
        self._add_labeled_entry(autonomy_frame, "BBox Max Area", self._autonomy_bbox_area_max_ratio_var, 0, 2, width=8)
        self._add_labeled_entry(autonomy_frame, "Forward m/s", self._autonomy_forward_speed_var, 1, 0, width=8)
        self._add_labeled_entry(autonomy_frame, "Reverse m/s", self._autonomy_reverse_speed_var, 1, 2, width=8)
        self._add_labeled_entry(autonomy_frame, "Stable Frames", self._autonomy_stable_frames_var, 2, 0, width=8)
        self._add_labeled_entry(autonomy_frame, "Proximity Stop m", self._autonomy_proximity_stop_var, 2, 2, width=8)
        self._add_labeled_entry(autonomy_frame, "Capture Timeout s", self._autonomy_capture_timeout_var, 3, 0, width=8)
        self._add_labeled_entry(
            autonomy_frame,
            "Evidence Interval s",
            self._autonomy_evidence_interval_var,
            3,
            2,
            width=8,
        )
        self._add_labeled_entry(
            autonomy_frame,
            "Min Target Confidence",
            self._autonomy_min_target_confidence_var,
            4,
            0,
            width=8,
        )
        self._add_labeled_entry(
            autonomy_frame,
            "Max Center Jump Ratio",
            self._autonomy_max_target_center_jump_ratio_var,
            4,
            2,
            width=8,
        )
        for column in range(4):
            autonomy_frame.columnconfigure(column, weight=1 if column in {1, 3} else 0)
        self._run_experiment_frames[RUN_TYPE_AUTONOMY_CENTER] = autonomy_frame
        for column in range(4):
            experiment_holder.columnconfigure(column, weight=1 if column in {1, 3} else 0)

        self._add_labeled_entry(form, "Run ID", self._run_id_var, 5, 0)
        ttk.Label(form, text="Notes").grid(row=6, column=0, sticky=tk.NW, pady=(8, 0))
        self._run_notes_text = tk.Text(form, height=3, width=40, wrap=tk.WORD)
        self._run_notes_text.grid(row=6, column=1, columnspan=3, sticky=tk.EW, padx=(8, 0), pady=(8, 0))
        for column in range(4):
            form.columnconfigure(column, weight=1 if column in {1, 3} else 0)

        actions = ttk.Frame(parent)
        actions.pack(side=tk.LEFT, fill=tk.Y)
        self._run_buttons["new_id"] = ttk.Button(actions, text="New ID", command=self.new_run_id)
        self._run_buttons["new_id"].pack(fill=tk.X)
        self._run_buttons["start"] = ttk.Button(actions, text="Start Run", command=self.start_run)
        self._run_buttons["start"].pack(fill=tk.X, pady=(8, 0))
        self._run_buttons["stop"] = ttk.Button(actions, text="Stop Run", command=self.stop_run)
        self._run_buttons["stop"].pack(fill=tk.X, pady=(8, 0))
        self._run_buttons["retrieve"] = ttk.Button(actions, text="Retrieve Results", command=self.retrieve_results)
        self._run_buttons["retrieve"].pack(fill=tk.X, pady=(18, 0))
        self._run_buttons["open_report"] = ttk.Button(actions, text="Open Report", command=self.open_report)
        self._run_buttons["open_report"].pack(fill=tk.X, pady=(8, 0))
        self._run_buttons["build_video"] = ttk.Button(actions, text="Build Videos", command=self.build_videos)
        self._run_buttons["build_video"].pack(fill=tk.X, pady=(8, 0))
        self._run_buttons["open_source_video"] = ttk.Button(
            actions, text="Open Source Video", command=self.open_source_video
        )
        self._run_buttons["open_source_video"].pack(fill=tk.X, pady=(8, 0))
        self._run_buttons["open_overlay_video"] = ttk.Button(
            actions, text="Open Overlay Video", command=self.open_overlay_video
        )
        self._run_buttons["open_overlay_video"].pack(fill=tk.X, pady=(8, 0))

    def _sync_run_experiment_fields(self) -> None:
        try:
            selected = selected_run_type(self._run_type_var.get())
        except ValueError:
            selected = RUN_TYPE_PERCEPTION
        for run_type, frame in self._run_experiment_frames.items():
            if run_type == selected:
                frame.pack(fill=tk.X)
            else:
                frame.pack_forget()

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

    def _connection_values(self) -> MonitorConnectionValues:
        return MonitorConnectionValues(
            host=self._host_var.get(),
            port=self._port_var.get(),
            preview_host=self._preview_host_var.get(),
            preview_port=self._preview_port_var.get(),
            preview_latency_ms=self._preview_latency_var.get(),
            gst_launch_path=self._gst_launch_path_var.get(),
            poll_interval_seconds=self._poll_interval_var.get(),
        )

    def _connection_settings(self) -> MonitorConnectionSettings:
        return resolve_monitor_connection(self._connection_values())

    def _teleop_step_values(self) -> TeleopStepValues:
        return TeleopStepValues(
            linear_step_mps=self._teleop_linear_step_var.get(),
            angular_step_rad_s=self._teleop_angular_step_var.get(),
        )

    def _teleop_step_settings(self) -> TeleopStepSettings:
        return resolve_teleop_steps(self._teleop_step_values())

    def _repo_root(self) -> Path:
        return Path(self._parsed.repo_root).expanduser().resolve()

    def _run_form_values(self, run_id: str | None = None) -> RunFormValues:
        return RunFormValues(
            robot_host=self._connection_settings().host,
            ssh_user=self._ssh_user_var.get(),
            remote_repo_root=self._remote_repo_root_var.get(),
            remote_runs_root=self._remote_runs_root_var.get(),
            local_import_root=self._local_import_root_var.get(),
            run_id=run_id or self._run_id_var.get(),
            backend_label=self._run_backend_var.get(),
            run_type_label=self._run_type_var.get(),
            classes_text=self._run_classes_var.get(),
            notes=self._run_notes(),
            devcontainer_exec_template=self._devcontainer_exec_template_var.get(),
            autonomy_bbox_area_min_ratio=self._autonomy_bbox_area_min_ratio_var.get(),
            autonomy_bbox_area_max_ratio=self._autonomy_bbox_area_max_ratio_var.get(),
            autonomy_forward_speed_m_s=self._autonomy_forward_speed_var.get(),
            autonomy_reverse_speed_m_s=self._autonomy_reverse_speed_var.get(),
            autonomy_stable_framed_frames=self._autonomy_stable_frames_var.get(),
            autonomy_proximity_stop_m=self._autonomy_proximity_stop_var.get(),
            autonomy_capture_timeout_sec=self._autonomy_capture_timeout_var.get(),
            autonomy_min_target_confidence=self._autonomy_min_target_confidence_var.get(),
            autonomy_max_target_center_jump_ratio=self._autonomy_max_target_center_jump_ratio_var.get(),
            autonomy_evidence_interval_sec=self._autonomy_evidence_interval_var.get(),
            detector_score_threshold=self._detector_score_threshold_var.get(),
            detector_nms_iou_threshold=self._detector_nms_iou_threshold_var.get(),
            detector_max_detections=self._detector_max_detections_var.get(),
            record_video=bool(self._record_video_var.get()),
            record_rosbag=bool(self._record_rosbag_var.get()),
        )

    def _run_form_selection(self, run_id: str | None = None) -> RunFormSelection:
        selection = resolve_run_form(
            self._run_form_values(run_id=run_id),
            repo_root=self._repo_root(),
            default_run_id=_default_run_id,
        )
        if run_id is None and selection.run_id != self._run_id_var.get():
            self._run_id_var.set(selection.run_id)
        return selection

    def _robot_connection(self) -> RobotConnection:
        return self._run_form_selection().connection

    def _run_config(self, run_id: str | None = None) -> RunConfig:
        return self._run_form_selection(run_id=run_id).run_config

    def _artifact_context(self) -> RunArtifactContext:
        return self._run_form_selection().artifact_context

    def _append_log_threadsafe(self, message: str) -> None:
        self._root.after(0, lambda: self._append_log(message))

    def _stub(self) -> Any:
        settings = self._connection_settings()
        channel = grpc.insecure_channel(target_for(settings.host, settings.port))
        return channel, create_stub(channel)

    def _append_log(self, message: str) -> None:
        self._log_text.configure(state=tk.NORMAL)
        self._log_text.insert(tk.END, message + "\n")
        self._log_text.see(tk.END)
        self._log_text.configure(state=tk.DISABLED)

    def _append_artifact_warnings(self, issues: tuple[str, ...]) -> None:
        for issue in issues:
            self._append_log(f"artifact warning: {issue}")

    def _append_artifact_result(self, result: RunArtifactResult) -> None:
        self._append_log("$ " + shell_join(result.command))
        self._append_log(result.message)
        self._append_artifact_warnings(result.issues)

    def _format_run_mode(self, state: RunState) -> str:
        label = state.phase.value.replace("_", " ")
        if state.run_id:
            label = f"{label} {state.run_id}"
        if state.message:
            label = f"{label}: {state.message}"
        return label

    def _set_run_state(self, phase: RunPhase, *, run_id: str = "", message: str = "") -> None:
        self._run_state = run_state(phase, run_id=run_id, message=message)
        self._update_run_controls()
        self._mode_var.set(self._format_run_mode(self._run_state))

    def _update_teleop_enabled(self, enabled: bool) -> None:
        self._teleop_enabled = enabled

    def _update_run_controls(self) -> None:
        availability = run_control_availability(self._run_state)
        states = {
            "new_id": availability.new_id,
            "start": availability.start,
            "stop": availability.stop,
            "retrieve": availability.retrieve,
            "open_report": availability.open_report,
            "build_video": availability.open_report,
            "open_source_video": availability.open_report,
            "open_overlay_video": availability.open_report,
        }
        for name, enabled in states.items():
            button = self._run_buttons.get(name)
            if button is not None:
                button.configure(state=tk.NORMAL if enabled else tk.DISABLED)

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
        run_id = sanitize_run_id(self._run_id_var.get())
        if not run_id:
            run_id = _default_run_id()
            self._run_id_var.set(run_id)
        return run_id

    def _current_run_id(self) -> str:
        return self._active_run_id or self._resolved_run_id()

    def _cancel_run_stop_timer(self) -> None:
        if self._run_stop_after_id is None:
            return
        self._root.after_cancel(self._run_stop_after_id)
        self._run_stop_after_id = None

    def _cancel_run_poll(self) -> None:
        if self._run_poll_after_id is None:
            return
        self._root.after_cancel(self._run_poll_after_id)
        self._run_poll_after_id = None

    def new_run_id(self) -> None:
        self._run_id_var.set(_default_run_id())
        self._append_log("new run id generated")

    def start_run(self) -> None:
        self._append_log("start run requested")
        if remote_run_is_running(self._run_process):
            self._append_log("run already running")
            return

        try:
            connection = self._robot_connection()
            run_config = self._run_config()
            self._set_run_state(RunPhase.PREPARING, run_id=run_config.run_id)
            start_result = self._run_manager.start(
                connection=connection,
                run_config=run_config,
                before_process_start=lambda _command: self._set_run_state(
                    RunPhase.STARTING,
                    run_id=run_config.run_id,
                ),
            )
        except (OSError, ValueError) as error:
            failed_run_id = run_config.run_id if "run_config" in locals() else ""
            failed_action = "start" if self._run_state.phase == RunPhase.STARTING else "prepare"
            self._set_run_state(RunPhase.FAILED, run_id=failed_run_id, message=str(error))
            self._append_log(f"failed to {failed_action} run: {error}")
            return

        self._run_process = start_result.remote_run
        self._active_run_id = run_config.run_id
        self._run_generation += 1
        self._set_run_state(RunPhase.RUNNING, run_id=run_config.run_id)
        class_text = ", ".join(run_config.classes) if run_config.classes else "from config"
        autonomy_enabled = run_config.run_type == RUN_TYPE_AUTONOMY_CENTER
        target_text = run_config.classes[0] if autonomy_enabled and run_config.classes else "-"
        self._append_log(
            f"run started remotely: {run_config.run_id}; "
            f"backend={RUN_BACKEND_LABELS[run_config.backend]}; classes={class_text}; "
            f"autonomy={str(autonomy_enabled).lower()}; target={target_text}; "
            f"score_threshold={run_config.detector_score_threshold}; "
            f"nms_iou={run_config.detector_nms_iou_threshold}; "
            f"max_detections={run_config.detector_max_detections}"
        )
        self._append_log("$ " + shell_join(start_result.command))
        self._start_run_log_reader()
        self._poll_run_process(
            remote_run=start_result.remote_run,
            run_id=run_config.run_id,
            generation=self._run_generation,
        )

    def _start_run_log_reader(self) -> None:
        remote_run = self._run_process
        if remote_run is None:
            return
        start_remote_run_log_reader(remote_run, self._append_log_threadsafe)

    def stop_run(self) -> None:
        self._append_log("stop run requested")
        if not self._request_run_stop():
            runtime_stop_requested = self._request_remote_runtime_stop(finalize_on_success=True)
            if runtime_stop_requested and self._run_state.phase != RunPhase.STOPPED:
                run_id = self._active_run_id or sanitize_run_id(self._run_id_var.get())
                self._set_run_state(RunPhase.STOPPING, run_id=run_id)
            if not runtime_stop_requested:
                self._append_log("run not running")
                return
            self._append_log("waiting for remote run shutdown")
            return

        self._append_log("waiting for remote run shutdown")

    def _request_run_stop(self, *, finalize_runtime_stop: bool = True) -> bool:
        stop_result = self._run_manager.request_stop(self._run_process)
        if not stop_result.accepted:
            return False

        run_id = self._current_run_id()
        self._set_run_state(RunPhase.STOPPING, run_id=run_id)
        if finalize_runtime_stop:
            self._request_remote_runtime_stop(finalize_on_success=True)
        if stop_result.graceful and self._run_state.phase != RunPhase.STOPPED and self._run_stop_after_id is None:
            self._run_stop_after_id = self._root.after(RUN_STOP_GRACE_MS, self._force_stop_run_process)
        elif stop_result.interrupted:
            self._append_log("remote run did not accept graceful stop; local ssh session interrupted")
        return True

    def _force_stop_run_process(self) -> None:
        self._run_stop_after_id = None
        self._request_remote_runtime_stop(finalize_on_success=True)

        stop_result = self._run_manager.force_stop(self._run_process)
        if stop_result.interrupted:
            self._set_run_state(RunPhase.STOPPING, run_id=self._current_run_id())
            self._append_log("remote run did not exit after graceful stop; local ssh session interrupted")

    def _request_remote_runtime_stop(self, *, finalize_on_success: bool = False) -> bool:
        try:
            if selected_run_backend(self._run_backend_var.get()) != RUN_BACKEND_RUNTIME:
                return False
            run_id = self._active_run_id or sanitize_run_id(self._run_id_var.get())
            if not run_id:
                return False
            if self._runtime_stop_pending_run_id == run_id:
                return True

            def _on_completion(success: bool, message: str) -> None:
                self._root.after(0, lambda: self._complete_runtime_stop(success, message, run_id))

            self._runtime_stop_pending_run_id = run_id
            accepted = self._run_manager.request_runtime_stop(
                connection=self._robot_connection(),
                run_id=run_id,
                on_command=lambda command: self._append_log("$ " + shell_join(command)),
                on_message=self._append_log_threadsafe,
                on_completion=_on_completion if finalize_on_success else None,
            )
            if not accepted and self._runtime_stop_pending_run_id == run_id:
                self._runtime_stop_pending_run_id = None
            return accepted
        except ValueError as error:
            if "run_id" in locals() and self._runtime_stop_pending_run_id == run_id:
                self._runtime_stop_pending_run_id = None
            self._append_log(f"runtime stop command failed: {error}")
            return False

    def _complete_runtime_stop(self, success: bool, _message: str, run_id: str) -> None:
        if self._runtime_stop_pending_run_id == run_id:
            self._runtime_stop_pending_run_id = None
        if not success:
            return
        self._finish_stopped_run(run_id=run_id, log_message=f"remote runtime container stopped: {run_id}")

    def _finish_stopped_run(self, *, run_id: str, log_message: str) -> None:
        self._cancel_run_stop_timer()
        self._cancel_run_poll()
        remote_run = self._run_process
        self._run_generation += 1
        self._run_process = None
        self._active_run_id = run_id
        if self._runtime_stop_pending_run_id == run_id:
            self._runtime_stop_pending_run_id = None
        self._set_run_state(RunPhase.STOPPED, run_id=run_id)
        self._append_log(log_message)
        if remote_run_is_running(remote_run):
            self._run_manager.force_stop(remote_run)

    def _poll_run_process(
        self,
        *,
        remote_run: RemoteRunProcess | None = None,
        run_id: str | None = None,
        generation: int | None = None,
    ) -> None:
        remote_run = remote_run or self._run_process
        if remote_run is None:
            return
        if remote_run is not self._run_process:
            return
        if generation is not None and generation != self._run_generation:
            return
        run_id = run_id or self._current_run_id()
        completion = self._run_manager.completion(remote_run, run_id=run_id)
        if completion is None:
            self._run_poll_after_id = self._root.after(
                1000,
                lambda: self._poll_run_process(
                    remote_run=remote_run,
                    run_id=run_id,
                    generation=generation,
                ),
            )
            return

        self._cancel_run_stop_timer()
        self._run_poll_after_id = None
        self._set_run_state(completion.phase, run_id=completion.run_id, message=completion.state_message)
        self._append_log(completion.log_message)
        self._run_process = None

    def retrieve_results(self) -> None:
        self._append_log("retrieve results requested")
        run_id = self._current_run_id()
        result = retrieve_run_artifacts(
            self._artifact_context(),
            run_id=run_id,
            overwrite=True,
        )
        self._append_artifact_result(result)
        if not result.success:
            return
        self._generate_and_open_report()

    def _generate_and_open_report(self) -> None:
        self._append_log("generating report after retrieval")
        run_id = self._current_run_id()
        result = generate_run_report(self._artifact_context(), run_id=run_id, overwrite=True)
        self._append_artifact_result(result)
        if not result.success:
            return

        self.open_report()

    def open_report(self) -> None:
        self._append_log("open report requested")
        run_id = self._current_run_id()
        context = self._artifact_context()
        inspection = inspect_run_artifacts(context, run_id, require_report=True)
        self._append_artifact_warnings(inspection.issues)
        path = report_path(context, run_id)
        if not inspection.report_exists:
            return
        webbrowser.open(path.as_uri())
        self._append_log(f"report opened: {path}")

    def build_videos(self) -> None:
        run_id = self._current_run_id()
        result = build_run_videos(self._artifact_context(), run_id=run_id)
        self._append_artifact_result(result)
        if result.success:
            self._append_artifact_result(generate_run_report(self._artifact_context(), run_id=run_id, overwrite=True))

    def _open_video(self, name: str) -> None:
        path = self._artifact_context().local_import_root / self._current_run_id() / "video" / name
        if not path.is_file():
            self._append_log(f"video not found: {path}")
            return
        webbrowser.open(path.as_uri())
        self._append_log(f"video opened: {path}")

    def open_source_video(self) -> None:
        self._open_video("source.mp4")

    def open_overlay_video(self) -> None:
        self._open_video("overlay.mp4")

    def refresh_status(self) -> None:
        self._append_log("status refresh requested")
        try:
            settings = self._connection_settings()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                status = get_system_status(create_stub(channel))
        except (grpc.RpcError, ValueError) as error:
            self._append_log(_format_request_error("status refresh", error))
            return

        self._last_status = status
        self._update_teleop_enabled(status.teleop.enabled)
        operator_status = format_operator_status(status)
        self._set_status_text(operator_status + "\n\nRaw status\n" + format_system_status(status))
        fault_line = operator_status.splitlines()[-1] if operator_status else None
        if fault_line and fault_line != "FAULT none" and fault_line != self._last_fault_line:
            self._append_log(fault_line)
        self._last_fault_line = fault_line
        self._append_log("status refreshed")

    def start_watch(self) -> None:
        self._append_log("start watch requested")
        if self._watch_after_id is not None:
            self._append_log("watch already running")
            return
        self._append_log("watch started")
        self._watch_tick()

    def _watch_tick(self) -> None:
        self.refresh_status()
        interval_ms = int(self._connection_settings().poll_interval_seconds * 1000)
        self._watch_after_id = self._root.after(max(interval_ms, 100), self._watch_tick)

    def stop_watch(self) -> None:
        self._append_log("stop watch requested")
        if self._watch_after_id is None:
            self._append_log("watch not running")
            return
        self._root.after_cancel(self._watch_after_id)
        self._watch_after_id = None
        self._append_log("watch stopped")

    def preview_on(self) -> None:
        self._append_log("preview on requested")
        self._set_preview_mode(True)

    def preview_off(self) -> None:
        self._append_log("preview off requested")
        self._set_preview_mode(False)

    def _set_preview_mode(self, enabled: bool) -> None:
        profile_name = self._profile_var.get() if enabled else None
        try:
            settings = self._connection_settings()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                response = set_preview_mode(
                    create_stub(channel),
                    enabled=enabled,
                    profile_name=profile_name,
                )
        except (grpc.RpcError, ValueError) as error:
            self._append_log(_format_request_error("preview request", error))
            return

        self._append_log(format_preview_response(response))
        self.refresh_status()

    def teleop_enable(self) -> None:
        self._append_log("teleop enable requested")
        self._set_teleop_enabled(True)

    def teleop_disable(self) -> None:
        self._append_log("teleop disable requested")
        self._set_teleop_enabled(False)

    def _set_teleop_enabled(self, enabled: bool) -> None:
        try:
            settings = self._connection_settings()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                response = set_teleop_enabled(create_stub(channel), enabled=enabled)
        except (grpc.RpcError, ValueError) as error:
            self._append_log(_format_request_error("teleop request", error))
            return

        self._update_teleop_enabled(response.teleop.enabled)
        self._append_log(format_teleop_response(response))
        self.refresh_status()

    def send_teleop_action(self, action: str) -> None:
        if not self._teleop_enabled:
            return

        self._append_log(f"teleop {action} requested")
        try:
            teleop_settings = self._teleop_step_settings()
            linear_x, linear_y, angular_z = _teleop_command_for_action(
                action,
                linear_step_mps=teleop_settings.linear_step_mps,
                angular_step_rad_s=teleop_settings.angular_step_rad_s,
            )
            settings = self._connection_settings()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                response = send_teleop_command(
                    create_stub(channel),
                    linear_x_mps=linear_x,
                    linear_y_mps=linear_y,
                    angular_z_rad_s=angular_z,
                )
        except (grpc.RpcError, ValueError) as error:
            self._append_log(_format_request_error("teleop command", error))
            return

        self._append_log(format_teleop_response(response))
        self.refresh_status()

    def close(self) -> None:
        self._request_run_stop(finalize_runtime_stop=False)
        try:
            settings = self._connection_settings()
            with grpc.insecure_channel(target_for(settings.host, settings.port)) as channel:
                set_teleop_enabled(create_stub(channel), enabled=False)
        except (grpc.RpcError, ValueError):
            pass
        self._root.destroy()

    def open_viewer(self) -> None:
        self._append_log("open viewer requested")
        self._open_viewer_process("viewer", _build_preview_viewer_command)

    def open_overlay(self) -> None:
        self._append_log("open overlay requested")
        self._open_viewer_process("overlay", _build_overlay_viewer_command)

    def _open_viewer_process(self, label: str, command_builder: Any) -> None:
        if self._viewer_process is not None and self._viewer_process.poll() is None:
            self._append_log("viewer already running")
            return

        settings = self._connection_settings()
        profile_name = self._profile_var.get()
        leave_preview_running = (
            self._last_status is not None and self._last_status.preview.state == robot_gateway_pb2.PREVIEW_RUNNING
        )
        command = command_builder(
            settings,
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
