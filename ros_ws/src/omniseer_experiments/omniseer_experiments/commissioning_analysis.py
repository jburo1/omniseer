"""Offline measurements for deterministic motion-commissioning RunBundles.

This module never creates ROS publishers.  It reads the recorded rosbag and
writes only derived files below ``analysis/commissioning`` in the bundle.
Bag imports are deliberately lazy so the numerical analysis can be unit tested
without a ROS installation or physical hardware.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import re
import shutil
import statistics
from collections import defaultdict
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

PHASE_TOPIC = "/commissioning/phase"
TOPICS = frozenset(
    {
        PHASE_TOPIC,
        "/cmd_vel_autonomy",
        "/mecanum_drive_controller/reference",
        "/encoder_counts",
        "/mecanum_drive_controller/odometry",
        "/imu",
        "/odometry/filtered",
    }
)
AXES = ("vx", "vy", "wz")
ANALYSIS_DIR = Path("analysis/commissioning")


def _number(value: Any) -> float:
    return float(value)


def _get(value: Any, *path: str) -> Any:
    for part in path:
        value = getattr(value, part)
    return value


def _stats(samples: Sequence[dict[str, float]], axes: Sequence[str] = AXES) -> dict[str, Any] | None:
    if not samples:
        return None
    result: dict[str, Any] = {"samples": len(samples)}
    for axis in axes:
        values = [sample[axis] for sample in samples]
        result[axis] = {
            "mean": statistics.fmean(values),
            "stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
        }
    return result


def _pose_delta(samples: Sequence[dict[str, float]]) -> dict[str, float] | None:
    if len(samples) < 2:
        return None
    first, last = samples[0], samples[-1]
    return {"x_m": last["x"] - first["x"], "y_m": last["y"] - first["y"], "yaw_rad": _angle_delta(last["yaw"], first["yaw"])}


def _angle_delta(end: float, start: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def _phase_intervals(phase_messages: Sequence[dict[str, Any]], end_time: float) -> list[dict[str, Any]]:
    transitions: list[dict[str, Any]] = []
    for message in sorted(phase_messages, key=lambda item: item["time"]):
        if not transitions or message["name"] != transitions[-1]["name"]:
            transitions.append(message)
    phases = []
    for index, transition in enumerate(transitions):
        end = transitions[index + 1]["time"] if index + 1 < len(transitions) else end_time
        if end >= transition["time"]:
            phases.append({"name": transition["name"], "start_sec": transition["time"], "end_sec": end})
    return phases


def _inside(samples: Iterable[dict[str, float]], start: float, end: float) -> list[dict[str, float]]:
    return [sample for sample in samples if start <= sample["time"] < end]


def _onset_settling(samples: Sequence[dict[str, float]], axis: str, target: float, start: float) -> dict[str, float | None] | None:
    """Return conservative 10%-onset and final-contiguous 20%-band settling.

    It intentionally reports ``null`` where the recorded sampling is too sparse
    to support a timing claim.
    """
    if target == 0.0 or len(samples) < 3:
        return None
    signed = [math.copysign(item[axis], target) for item in samples]
    onset_index = next((i for i, value in enumerate(signed) if value >= abs(target) * 0.10), None)
    in_band = [abs(value - abs(target)) <= max(abs(target) * 0.20, 1.0e-6) for value in signed]
    settle_index: int | None = None
    for index in range(len(in_band)):
        if all(in_band[index:]):
            settle_index = index
            break
    return {
        "onset_sec": None if onset_index is None else samples[onset_index]["time"] - start,
        "settling_sec": None if settle_index is None else samples[settle_index]["time"] - start,
    }


def _cross_axis(raw: Sequence[dict[str, float]], command: dict[str, float]) -> dict[str, Any] | None:
    active = [axis for axis in AXES if command[axis] != 0.0]
    if len(active) != 1 or not raw:
        return None
    commanded_axis = active[0]
    response = _stats(raw)
    assert response is not None
    return {
        "commanded_axis": commanded_axis,
        "other_axes": {axis: response[axis] for axis in AXES if axis != commanded_axis},
    }


def analyze_samples(samples_by_topic: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    """Build a JSON-safe report from normalized bag samples.

    Samples use bag-record timestamps in seconds.  This shared clock is
    necessary because phase messages have no header and should not be mixed with
    potentially different sensor header clock domains.
    """
    phases_input = samples_by_topic.get(PHASE_TOPIC, [])
    all_times = [sample["time"] for samples in samples_by_topic.values() for sample in samples]
    if not phases_input:
        raise ValueError(f"required phase topic {PHASE_TOPIC} contains no messages")
    if not all_times:
        raise ValueError("bag contains no readable commissioning messages")
    phases = _phase_intervals(phases_input, max(all_times))
    if not phases:
        raise ValueError("phase topic contains no usable transitions")

    phase_results: list[dict[str, Any]] = []
    for phase in phases:
        start, end = phase["start_sec"], phase["end_sec"]
        command = _inside(samples_by_topic.get("/cmd_vel_autonomy", []), start, end)
        reference = _inside(samples_by_topic.get("/mecanum_drive_controller/reference", []), start, end)
        raw = _inside(samples_by_topic.get("/mecanum_drive_controller/odometry", []), start, end)
        filtered = _inside(samples_by_topic.get("/odometry/filtered", []), start, end)
        imu = _inside(samples_by_topic.get("/imu", []), start, end)
        encoders = _inside(samples_by_topic.get("/encoder_counts", []), start, end)
        command_stats = _stats(command)
        command_mean = {axis: command_stats[axis]["mean"] for axis in AXES} if command_stats else {axis: 0.0 for axis in AXES}
        imu_stats = _stats(imu, ("wz",))
        phase_results.append(
            {
                **phase,
                "duration_sec": end - start,
                "command": command_stats,
                "controller_reference": _stats(reference),
                "raw_wheel_odometry_velocity": _stats(raw),
                "filtered_odometry_velocity": _stats(filtered),
                "imu_angular_velocity": imu_stats,
                # These are deltas from estimator-reported poses, not external truth.
                "integrated_raw_odometry_pose_change": _pose_delta(raw),
                "integrated_filtered_odometry_pose_change": _pose_delta(filtered),
                "encoder_count_change": _encoder_delta(encoders),
                "cross_axis_response": {
                    "raw_wheel_odometry": _cross_axis(raw, command_mean),
                    "filtered_odometry": _cross_axis(filtered, command_mean),
                },
                "raw_odom_response_timing": _onset_settling(raw, _active_axis(command_mean), _active_target(command_mean), start),
                "filtered_odom_response_timing": _onset_settling(
                    filtered, _active_axis(command_mean), _active_target(command_mean), start
                ),
            }
        )
    origin = phase_results[0]["start_sec"]
    for phase in phase_results:
        phase["start_offset_sec"] = phase["start_sec"] - origin
        phase["end_offset_sec"] = phase["end_sec"] - origin
    return {
        "schema_version": 1,
        "timestamp_domain": "rosbag_record_timestamp_sec",
        "interpretation": {
            "command": "Requested /cmd_vel_autonomy motion; it is not physical ground truth.",
            "reference": "Command accepted at the mecanum controller reference topic.",
            "odometry_and_imu": "Estimator and sensor outputs; no external position or heading ground truth exists in this RunBundle.",
        },
        "phase_count": len(phase_results),
        "phases": phase_results,
        "stationary": _stationary_summary(phase_results, samples_by_topic),
        "positive_negative_symmetry": {
            "raw_wheel_odometry": _symmetry(phase_results, "raw_wheel_odometry_velocity"),
            "filtered_odometry": _symmetry(phase_results, "filtered_odometry_velocity"),
        },
        "available_topics": sorted(topic for topic, values in samples_by_topic.items() if values),
    }


def _active_axis(command: dict[str, float]) -> str:
    return next((axis for axis in AXES if command[axis] != 0.0), "vx")


def _active_target(command: dict[str, float]) -> float:
    return next((command[axis] for axis in AXES if command[axis] != 0.0), 0.0)


def _encoder_delta(samples: Sequence[dict[str, float]]) -> dict[str, float] | None:
    if len(samples) < 2:
        return None
    return {axis: samples[-1][axis] - samples[0][axis] for axis in ("front_left", "front_right", "rear_left", "rear_right")}


def _stationary_summary(
    phases: Sequence[dict[str, Any]], samples_by_topic: dict[str, list[dict[str, Any]]]
) -> dict[str, Any]:
    stationary = []
    for phase in phases:
        command = phase["command"]
        if command and all(command[axis]["mean"] == 0.0 for axis in AXES):
            stationary.append(phase)
    intervals = [(phase["start_sec"], phase["end_sec"]) for phase in stationary]
    def collect(topic: str) -> list[dict[str, float]]:
        return [
            sample
            for sample in samples_by_topic.get(topic, [])
            if any(start <= sample["time"] < end for start, end in intervals)
        ]
    return {
        "phase_names": [phase["name"] for phase in stationary],
        "raw_wheel_odometry_velocity": _stats(collect("/mecanum_drive_controller/odometry")),
        "filtered_odometry_velocity": _stats(collect("/odometry/filtered")),
        "imu_angular_velocity": _stats(collect("/imu"), ("wz",)),
    }


def _symmetry(phases: Sequence[dict[str, Any]], measurement_key: str) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for axis in AXES:
        positive, negative = [], []
        for phase in phases:
            command = phase["command"]
            measurement = phase[measurement_key]
            if not command or not measurement or sum(command[name]["mean"] != 0.0 for name in AXES) != 1:
                continue
            command_value = command[axis]["mean"]
            if command_value > 0:
                positive.append(measurement[axis]["mean"])
            elif command_value < 0:
                negative.append(measurement[axis]["mean"])
        if positive and negative:
            pos, neg = statistics.fmean(positive), statistics.fmean(negative)
            result[axis] = {
                "positive_mean": pos,
                "negative_mean": neg,
                "magnitude_ratio_positive_over_negative": abs(pos) / abs(neg) if neg else None,
            }
    return result


def read_rosbag_samples(bag_dir: Path) -> dict[str, list[dict[str, Any]]]:
    """Read only analyzer topics using standard ROS 2 rosbag APIs."""
    try:
        import rosbag2_py  # type: ignore[import-not-found]
        from rclpy.serialization import deserialize_message  # type: ignore[import-not-found]
        from rosidl_runtime_py.utilities import get_message  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - requires ROS runtime
        raise RuntimeError("ROS bag APIs are unavailable; source the Omniseer ROS workspace") from exc
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"rosbag directory is missing: {bag_dir}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types() if item.name in TOPICS}
    missing = sorted(TOPICS - set(topic_types))
    if missing:
        raise ValueError(f"rosbag is missing required commissioning topics: {', '.join(missing)}")
    messages: dict[str, list[dict[str, Any]]] = defaultdict(list)
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in topic_types:
            continue
        message = deserialize_message(serialized, get_message(topic_types[topic]))
        sample = _normalize_message(topic, message, timestamp_ns * 1.0e-9)
        if sample is not None:
            messages[topic].append(sample)
    return dict(messages)


def _storage_id(bag_dir: Path) -> str:
    """Read the ROS 2 bag storage plugin without adding a YAML dependency."""
    metadata = bag_dir / "metadata.yaml"
    if metadata.is_file():
        match = re.search(r"^\s*storage_identifier:\s*(\S+)\s*$", metadata.read_text(encoding="utf-8"), re.MULTILINE)
        if match:
            return match.group(1)
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def _normalize_message(topic: str, message: Any, timestamp: float) -> dict[str, Any] | None:
    sample: dict[str, Any] = {"time": timestamp}
    if topic == PHASE_TOPIC:
        return {**sample, "name": str(message.data)}
    if topic in {"/cmd_vel_autonomy", "/mecanum_drive_controller/reference"}:
        twist = message.twist
        return {**sample, "vx": _number(twist.linear.x), "vy": _number(twist.linear.y), "wz": _number(twist.angular.z)}
    if topic in {"/mecanum_drive_controller/odometry", "/odometry/filtered"}:
        twist, pose = message.twist.twist, message.pose.pose
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y), 1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2))
        return {**sample, "vx": _number(twist.linear.x), "vy": _number(twist.linear.y), "wz": _number(twist.angular.z), "x": _number(pose.position.x), "y": _number(pose.position.y), "yaw": yaw}
    if topic == "/imu":
        return {**sample, "wz": _number(message.angular_velocity.z)}
    if topic == "/encoder_counts":
        return {**sample, **{name: _number(getattr(message, name)) for name in ("front_left", "front_right", "rear_left", "rear_right")}}
    return None


def write_commissioning_analysis(run_dir: Path, *, overwrite: bool = False) -> Path:
    output_dir = run_dir / ANALYSIS_DIR
    if output_dir.exists():
        if not overwrite:
            raise FileExistsError(f"analysis already exists: {output_dir}; pass --overwrite to replace derived output")
        shutil.rmtree(output_dir)
    samples = read_rosbag_samples(run_dir / "rosbag")
    summary = analyze_samples(samples)
    output_dir.mkdir(parents=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "summary.md").write_text(_markdown(summary), encoding="utf-8")
    _write_plots(output_dir, samples, summary["phases"])
    return output_dir


def _markdown(summary: dict[str, Any]) -> str:
    lines = ["# Commissioning analysis", "", "Offline measurements from recorded rosbag timestamps. Commands, controller references, odometry, and IMU are not external physical ground truth.", "", "| Phase | Duration (s) | Command vx/vy/wz | Raw odom vx/vy/wz | Filtered vx/vy/wz |", "| --- | ---: | --- | --- | --- |"]
    for phase in summary["phases"]:
        lines.append("| {name} | {duration:.3f} | {command} | {raw} | {filtered} |".format(name=phase["name"], duration=phase["duration_sec"], command=_format_metric(phase["command"]), raw=_format_metric(phase["raw_wheel_odometry_velocity"]), filtered=_format_metric(phase["filtered_odometry_velocity"])))
    lines.extend(["", "## Stationary bias/noise", "", "```json", json.dumps(summary["stationary"], indent=2, sort_keys=True), "```", "", "## Positive/negative raw-odometry symmetry", "", "```json", json.dumps(summary["positive_negative_symmetry"], indent=2, sort_keys=True), "```", "", "Plots: `vx.svg`, `vy.svg`, `wz.svg`, `trajectory_xy.svg`, and `yaw.svg`."])
    return "\n".join(lines) + "\n"


def _format_metric(metric: dict[str, Any] | None) -> str:
    if not metric:
        return "n/a"
    return "/".join(f"{metric[axis]['mean']:.4f}" for axis in AXES)


def _write_plots(output_dir: Path, samples: dict[str, list[dict[str, Any]]], phases: Sequence[dict[str, Any]]) -> None:
    start = phases[0]["start_sec"]
    _write_series_svg(output_dir / "vx.svg", "vx (m/s)", [("command", samples.get("/cmd_vel_autonomy", []), "vx"), ("reference", samples.get("/mecanum_drive_controller/reference", []), "vx"), ("raw odom", samples.get("/mecanum_drive_controller/odometry", []), "vx"), ("filtered odom", samples.get("/odometry/filtered", []), "vx")], phases, start)
    _write_series_svg(output_dir / "vy.svg", "vy (m/s)", [("command", samples.get("/cmd_vel_autonomy", []), "vy"), ("reference", samples.get("/mecanum_drive_controller/reference", []), "vy"), ("raw odom", samples.get("/mecanum_drive_controller/odometry", []), "vy"), ("filtered odom", samples.get("/odometry/filtered", []), "vy")], phases, start)
    _write_series_svg(output_dir / "wz.svg", "wz (rad/s)", [("command", samples.get("/cmd_vel_autonomy", []), "wz"), ("reference", samples.get("/mecanum_drive_controller/reference", []), "wz"), ("raw odom", samples.get("/mecanum_drive_controller/odometry", []), "wz"), ("IMU", samples.get("/imu", []), "wz"), ("filtered odom", samples.get("/odometry/filtered", []), "wz")], phases, start)
    _write_xy_svg(output_dir / "trajectory_xy.svg", samples, "x (m)", "y (m)", "x", "y")
    _write_series_svg(output_dir / "yaw.svg", "yaw (rad)", [("raw odom", samples.get("/mecanum_drive_controller/odometry", []), "yaw"), ("filtered odom", samples.get("/odometry/filtered", []), "yaw")], phases, start)


def _write_series_svg(path: Path, title: str, series: Sequence[tuple[str, Sequence[dict[str, Any]], str]], phases: Sequence[dict[str, Any]], start: float) -> None:
    points = [(sample["time"] - start, sample[axis]) for _, samples, axis in series for sample in samples]
    xmax = max((point[0] for point in points), default=1.0) or 1.0
    values = [point[1] for point in points] or [0.0]
    ymin, ymax = min(values), max(values)
    margin = max((ymax - ymin) * 0.08, 1.0e-4)
    ymin, ymax = ymin - margin, ymax + margin
    def xy(point: tuple[float, float]) -> str:
        return f"{70 + 850 * point[0] / xmax:.2f},{450 - 390 * (point[1] - ymin) / (ymax - ymin):.2f}"
    colors = ("#1769aa", "#d05a00", "#228833", "#aa3377", "#6644aa")
    body = _svg_header(title)
    body += f'<text x="70" y="35" class="title">{html.escape(title)}</text><line x1="70" y1="450" x2="920" y2="450" class="axis"/><line x1="70" y1="60" x2="70" y2="450" class="axis"/>'
    for index, phase in enumerate(phases):
        x = 70 + 850 * (phase["start_sec"] - start) / xmax
        body += f'<line x1="{x:.2f}" y1="60" x2="{x:.2f}" y2="450" class="phase"/><text x="{x + 3:.2f}" y="75" class="phase-label">{html.escape(phase["name"])}</text>'
    for index, (name, samples, axis) in enumerate(series):
        line = " ".join(xy((sample["time"] - start, sample[axis])) for sample in samples)
        if line:
            body += f'<polyline points="{line}" fill="none" stroke="{colors[index]}" stroke-width="1.5"/><text x="{700 + (index % 2) * 105}" y="{25 + (index // 2) * 16}" fill="{colors[index]}" class="legend">{html.escape(name)}</text>'
    body += f'<text x="70" y="475" class="label">0 s</text><text x="880" y="475" class="label">{xmax:.2f} s</text><text x="5" y="65" class="label">{ymax:.4f}</text><text x="5" y="450" class="label">{ymin:.4f}</text></svg>'
    path.write_text(body, encoding="utf-8")


def _write_xy_svg(path: Path, samples: dict[str, list[dict[str, Any]]], x_label: str, y_label: str, x_axis: str, y_axis: str) -> None:
    series = [("raw odom", samples.get("/mecanum_drive_controller/odometry", [])), ("filtered odom", samples.get("/odometry/filtered", []))]
    points = [(sample[x_axis], sample[y_axis]) for _, samples in series for sample in samples] or [(0.0, 0.0)]
    xmin, xmax = min(x for x, _ in points), max(x for x, _ in points)
    ymin, ymax = min(y for _, y in points), max(y for _, y in points)
    xpad, ypad = max((xmax - xmin) * 0.08, 1.0e-4), max((ymax - ymin) * 0.08, 1.0e-4)
    xmin, xmax, ymin, ymax = xmin - xpad, xmax + xpad, ymin - ypad, ymax + ypad
    body = _svg_header("raw vs filtered XY trajectory") + '<text x="70" y="35" class="title">raw vs filtered XY trajectory</text><line x1="70" y1="450" x2="920" y2="450" class="axis"/><line x1="70" y1="60" x2="70" y2="450" class="axis"/>'
    for index, (name, values) in enumerate(series):
        line = " ".join(f"{70 + 850 * (sample[x_axis] - xmin) / (xmax - xmin):.2f},{450 - 390 * (sample[y_axis] - ymin) / (ymax - ymin):.2f}" for sample in values)
        if line:
            color = ("#228833", "#aa3377")[index]
            body += f'<polyline points="{line}" fill="none" stroke="{color}" stroke-width="1.5"/><text x="{730 + index * 95}" y="25" fill="{color}" class="legend">{name}</text>'
    path.write_text(body + f'<text x="450" y="490" class="label">{x_label}</text><text x="5" y="65" class="label">{y_label}</text></svg>', encoding="utf-8")


def _svg_header(title: str) -> str:
    return f'<?xml version="1.0" encoding="UTF-8"?><svg xmlns="http://www.w3.org/2000/svg" width="1000" height="510" viewBox="0 0 1000 510"><style>.title{{font:18px sans-serif}}.legend,.label{{font:12px sans-serif}}.phase-label{{font:10px sans-serif;fill:#666}}.axis{{stroke:#333}}.phase{{stroke:#aaa;stroke-dasharray:4 3}}</style><title>{html.escape(title)}</title>'


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Offline analyzer for commissioning RunBundle rosbags; writes derived output only.")
    parser.add_argument("run_dir", help="path to an existing commissioning RunBundle")
    parser.add_argument("--overwrite", action="store_true", help="replace only analysis/commissioning derived output")
    args = parser.parse_args(argv)
    output_dir = write_commissioning_analysis(Path(args.run_dir), overwrite=args.overwrite)
    print(output_dir)
