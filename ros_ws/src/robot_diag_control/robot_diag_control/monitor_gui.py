from __future__ import annotations

import argparse
import subprocess
import sys
from typing import Any

import grpc

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.gateway_client import (
    PROFILE_TO_PROTO,
    create_stub,
    format_preview_response,
    format_system_status,
    format_system_status_summary,
    get_system_status,
    set_preview_mode,
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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal Tk monitor GUI for the robot gateway"
    )
    parser.add_argument("--host", default="127.0.0.1", help="gRPC gateway host")
    parser.add_argument("--port", type=int, default=50051, help="gRPC gateway port")
    parser.add_argument(
        "--preview-host",
        default=None,
        help="preview stream host; defaults to --host",
    )
    parser.add_argument("--preview-port", type=int, default=7001)
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
    return parser


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    return _build_parser().parse_args(sys.argv[1:] if args is None else args)


def _resolved_preview_host(parsed: argparse.Namespace) -> str:
    return parsed.preview_host or parsed.host


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


class RobotMonitorGui:
    def __init__(self, root: Any, parsed: argparse.Namespace) -> None:
        self._root = root
        self._parsed = parsed
        self._watch_after_id: str | None = None
        self._viewer_process: subprocess.Popen[str] | None = None
        self._last_status: robot_gateway_pb2.SystemStatus | None = None

        self._host_var = tk.StringVar(value=parsed.host)
        self._port_var = tk.StringVar(value=str(parsed.port))
        self._preview_host_var = tk.StringVar(value=_resolved_preview_host(parsed))
        self._preview_port_var = tk.StringVar(value=str(parsed.preview_port))
        self._preview_latency_var = tk.StringVar(value=str(parsed.preview_latency_ms))
        self._gst_launch_path_var = tk.StringVar(value=parsed.gst_launch_path)
        self._poll_interval_var = tk.StringVar(value=str(parsed.poll_interval_seconds))
        self._profile_var = tk.StringVar(value="balanced")
        self._summary_var = tk.StringVar(value="Disconnected")
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
        style.configure("Summary.TLabel", font=("TkDefaultFont", 11, "bold"))

    def _build_layout(self) -> None:
        container = ttk.Frame(self._root, padding=14)
        container.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(container)
        header.pack(fill=tk.X)
        ttk.Label(header, text="Robot Monitor", style="Title.TLabel").pack(side=tk.LEFT)
        ttk.Label(header, textvariable=self._summary_var, style="Summary.TLabel").pack(
            side=tk.RIGHT
        )

        connection_frame = ttk.LabelFrame(container, text="Connection", padding=10)
        connection_frame.pack(fill=tk.X, pady=(12, 10))
        self._add_labeled_entry(connection_frame, "Host", self._host_var, 0, 0)
        self._add_labeled_entry(connection_frame, "Port", self._port_var, 0, 2, width=10)
        self._add_labeled_entry(connection_frame, "Preview Host", self._preview_host_var, 1, 0)
        self._add_labeled_entry(
            connection_frame, "Preview Port", self._preview_port_var, 1, 2, width=10
        )
        self._add_labeled_entry(
            connection_frame, "Latency ms", self._preview_latency_var, 2, 0, width=10
        )
        self._add_labeled_entry(
            connection_frame, "gst-launch", self._gst_launch_path_var, 2, 2
        )
        self._add_labeled_entry(
            connection_frame, "Poll Interval (s)", self._poll_interval_var, 3, 0, width=10
        )
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
        ttk.Button(controls, text="Start Watch", command=self.start_watch).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Button(controls, text="Stop Watch", command=self.stop_watch).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Button(controls, text="Preview On", command=self.preview_on).pack(
            side=tk.LEFT, padx=(24, 0)
        )
        ttk.Button(controls, text="Preview Off", command=self.preview_off).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Button(controls, text="Open Viewer", command=self.open_viewer).pack(
            side=tk.LEFT, padx=(24, 0)
        )

        body = ttk.Panedwindow(container, orient=tk.VERTICAL)
        body.pack(fill=tk.BOTH, expand=True)

        status_frame = ttk.LabelFrame(body, text="System Status", padding=8)
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

    def refresh_status(self) -> None:
        try:
            with grpc.insecure_channel(
                target_for(self._connection_namespace().host, self._connection_namespace().port)
            ) as channel:
                status = get_system_status(create_stub(channel))
        except (grpc.RpcError, ValueError) as error:
            self._summary_var.set("Disconnected")
            self._append_log(f"status refresh failed: {error}")
            return

        self._last_status = status
        self._summary_var.set(format_system_status_summary(status))
        self._set_status_text(format_system_status(status))
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

    def open_viewer(self) -> None:
        if self._viewer_process is not None and self._viewer_process.poll() is None:
            self._append_log("viewer already running")
            return

        settings = self._connection_namespace()
        profile_name = self._profile_var.get()
        leave_preview_running = (
            self._last_status is not None and self._last_status.preview.state == robot_gateway_pb2.PREVIEW_RUNNING
        )
        command = _build_preview_viewer_command(
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
            self._append_log(f"failed to launch viewer: {error}")
            return

        self._append_log(f"viewer launched with pid={self._viewer_process.pid}")
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
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()
    app.stop_watch() if app._watch_after_id is not None else None
    if app._viewer_process is not None and app._viewer_process.poll() is None:
        app._viewer_process.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
