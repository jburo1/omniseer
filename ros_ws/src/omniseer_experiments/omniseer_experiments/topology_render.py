"""Render captured ROS graph topology JSON as a documentation-friendly SVG."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any

# Documentation filtering belongs here, never in the captured RunBundle evidence.
IGNORED_TOPICS = frozenset(
    {
        "/diagnostics",
        "/events/write_split",
        "/parameter_events",
        "/rosout",
        "/tf",
        "/tf_static",
    }
)
IGNORED_NODE_NAME_PREFIXES = ("transform_listener_impl_",)

# This is a documentation projection of the captured graph, not a runtime
# topology template.  Keep this list narrowly scoped to bounded target
# acquisition so unrelated launched components do not obscure the path.
BOUNDED_AUTONOMY_TOPICS = frozenset(
    {
        "/yolo/detections",
        "/cmd_vel_autonomy",
        "/mecanum_drive_controller/reference",
        "/encoder_counts",
        "/mecanum_drive_controller/odometry",
        "/odometry/filtered",
        "/imu",
        "/range",
    }
)
EVIDENCE_NODES = frozenset({"/perception_run_recorder", "/rosbag2_recorder"})
GATEWAY_NODES = frozenset({"/robot_diag_control_cpp"})
HARDWARE_NODES = frozenset({"/omniseer_teensy"})


def _endpoint_labels(topic: dict[str, Any], endpoint_key: str) -> list[str]:
    """Return sorted captured endpoint labels for one topic direction."""

    return sorted(
        {
            _node_label(endpoint)
            for endpoint in topic.get(endpoint_key, [])
            if isinstance(endpoint, dict) and "node_name" in endpoint and not _is_ignored_node(_node_label(endpoint))
        }
    )


def _node_label(endpoint: dict[str, Any]) -> str:
    """Return the captured fully qualified ROS node name."""

    namespace = str(endpoint.get("node_namespace", endpoint.get("namespace", "/")))
    name = str(endpoint["node_name"] if "node_name" in endpoint else endpoint["name"])
    if namespace in ("", "/"):
        return f"/{name}"
    return f"{namespace.rstrip('/')}/{name}"


def _identifier(kind: str, label: str) -> str:
    digest = hashlib.sha256(label.encode("utf-8")).hexdigest()[:16]
    return f"{kind}_{digest}"


def _quoted(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def _is_ignored_node(label: str) -> bool:
    return Path(label).name.startswith(IGNORED_NODE_NAME_PREFIXES)


def topology_to_dot(topology: dict[str, Any]) -> str:
    """Build DOT from one recorder topology snapshot without repairing it."""

    captured_nodes = {
        _node_label(node)
        for node in topology.get("nodes", [])
        if isinstance(node, dict) and "name" in node and not _is_ignored_node(_node_label(node))
    }
    node_labels = set(captured_nodes)
    topics: list[tuple[str, list[str], list[str]]] = []

    for topic in topology.get("topics", []):
        if not isinstance(topic, dict) or not isinstance(topic.get("name"), str):
            continue
        topic_name = topic["name"]
        if topic_name in IGNORED_TOPICS:
            continue

        publishers = sorted(
            {
                _node_label(endpoint)
                for endpoint in topic.get("publishers", [])
                if isinstance(endpoint, dict)
                and "node_name" in endpoint
                and not _is_ignored_node(_node_label(endpoint))
            }
        )
        subscribers = sorted(
            {
                _node_label(endpoint)
                for endpoint in topic.get("subscribers", [])
                if isinstance(endpoint, dict)
                and "node_name" in endpoint
                and not _is_ignored_node(_node_label(endpoint))
            }
        )
        if not publishers and not subscribers:
            continue

        node_labels.update(publishers)
        node_labels.update(subscribers)
        topics.append((topic_name, publishers, subscribers))

    lines = [
        "digraph ros_topology {",
        "  graph [rankdir=LR, bgcolor=white, pad=0.2, nodesep=0.35, ranksep=0.75];",
        '  node [fontname="Helvetica", fontsize=11, shape=box, style="rounded,filled", '
        'fillcolor="#ffffff", color="#64748b", fontcolor="#1e293b", margin="0.12,0.07"];',
        '  edge [color="#64748b", arrowsize=0.7, penwidth=1.0];',
    ]
    for label in sorted(node_labels):
        lines.append(f"  {_identifier('node', label)} [label={_quoted(label)}];")
    for topic_name, publishers, subscribers in sorted(topics):
        topic_id = _identifier("topic", topic_name)
        is_incomplete = not publishers or not subscribers
        topic_style = 'style="filled" fillcolor="#eff6ff" color="#2563eb" fontcolor="#1e3a8a" penwidth=1.4'
        if is_incomplete:
            topic_style = 'style="filled,dashed" fillcolor="#f8fafc" color="#94a3b8" fontcolor="#64748b"'
        lines.append(f"  {topic_id} [label={_quoted(topic_name)}, shape=ellipse, {topic_style}];")
        publisher_set = set(publishers)
        edge_style = 'color="#2563eb" penwidth=1.5'
        if is_incomplete:
            edge_style = 'color="#94a3b8" style=dashed'
        for publisher in publishers:
            lines.append(f"  {_identifier('node', publisher)} -> {topic_id} [{edge_style}];")
        for subscriber in subscribers:
            # A node publishing and subscribing to one topic is a self-edge at
            # the ROS semantic level. Retain other observed endpoints instead.
            if subscriber not in publisher_set:
                lines.append(f"  {topic_id} -> {_identifier('node', subscriber)} [{edge_style}];")
    lines.append("}")
    return "\n".join(lines) + "\n"


def bounded_autonomy_topology_to_dot(topology: dict[str, Any]) -> str:
    """Build a focused DOT projection from observed bounded-autonomy endpoints.

    One-sided captured topics are retained with dashed styling.  In particular,
    this deliberately does not add a hardware publisher or controller consumer
    when the recorder did not observe one.
    """

    topics = {
        topic["name"]: topic
        for topic in topology.get("topics", [])
        if isinstance(topic, dict) and topic.get("name") in BOUNDED_AUTONOMY_TOPICS
    }
    endpoint_labels = {
        label
        for topic in topics.values()
        for direction in ("publishers", "subscribers")
        for label in _endpoint_labels(topic, direction)
    }
    captured_labels = {
        _node_label(node)
        for node in topology.get("nodes", [])
        if isinstance(node, dict) and "name" in node and not _is_ignored_node(_node_label(node))
    }
    evidence_labels = endpoint_labels & EVIDENCE_NODES

    groups = {
        "Hardware / I/O": {"/encoder_counts", "/imu", "/range"} | HARDWARE_NODES,
        "Perception": {"/vision_bridge", "/yolo/detections"},
        "State Estimation": {
            "/encoder_counts_to_odometry",
            "/mecanum_drive_controller/odometry",
            "/ekf_filter",
            "/odometry/filtered",
        },
        "Autonomy / Control": {
            "/target_centering_node",
            "/cmd_vel_autonomy",
            "/twist_mux",
            "/mecanum_drive_controller/reference",
        },
        "Run Evidence": set(evidence_labels),
    }
    # Do not show a group member merely because it is conceptually expected.
    observed_labels = endpoint_labels | set(topics) | captured_labels
    groups["Hardware / I/O"] &= observed_labels
    groups["Perception"] &= observed_labels
    groups["State Estimation"] &= observed_labels
    groups["Autonomy / Control"] &= observed_labels

    lines = [
        "digraph bounded_autonomy_topology {",
        '  graph [rankdir=LR, bgcolor=white, pad=0.25, nodesep=0.35, ranksep=0.8, fontname="Helvetica"];',
        '  node [fontname="Helvetica", fontsize=11, shape=box, '
        'style="rounded,filled", fillcolor="#ffffff", color="#64748b", '
        'fontcolor="#1e293b", margin="0.12,0.07"];',
        '  edge [color="#64748b", arrowsize=0.7, penwidth=1.0];',
    ]
    for index, (group_name, labels) in enumerate(groups.items()):
        lines.extend(
            [
                f"  subgraph cluster_{index} {{",
                f"    label={_quoted(group_name)};",
                '    color="#cbd5e1"; style="rounded"; penwidth=1.0; '
                'fontname="Helvetica"; fontsize=12; fontcolor="#334155";',
            ]
        )
        for label in sorted(labels):
            is_topic = label in topics
            is_evidence = label in EVIDENCE_NODES
            if is_topic:
                lines.append(
                    f"    {_identifier('topic', label)} [label={_quoted(label)}, shape=ellipse, "
                    'style="filled", fillcolor="#dbeafe", color="#2563eb", fontcolor="#1e3a8a", penwidth=1.5];'
                )
            elif is_evidence:
                lines.append(
                    f'    {_identifier("node", label)} [label={_quoted(label)}, fillcolor="#f8fafc", '
                    'color="#94a3b8", fontcolor="#64748b", style="rounded,dashed,filled"];'
                )
            elif label == "/target_centering_node":
                lines.append(
                    f'    {_identifier("node", label)} [label={_quoted(label)}, fillcolor="#ecfdf5", '
                    'color="#047857", fontcolor="#064e3b", penwidth=2.5];'
                )
            else:
                lines.append(
                    f'    {_identifier("node", label)} [label={_quoted(label)}, fillcolor="#ffffff", '
                    'color="#0f766e", fontcolor="#134e4a", penwidth=1.6];'
                )
        lines.append("  }")

    primary_topics = {
        "/yolo/detections",
        "/cmd_vel_autonomy",
        "/mecanum_drive_controller/reference",
        "/mecanum_drive_controller/odometry",
        "/odometry/filtered",
    }
    for topic_name, topic in sorted(topics.items()):
        topic_id = _identifier("topic", topic_name)
        publishers = _endpoint_labels(topic, "publishers")
        subscribers = _endpoint_labels(topic, "subscribers")
        is_incomplete = not publishers or not subscribers
        for publisher in publishers:
            is_evidence = publisher in EVIDENCE_NODES
            if publisher in GATEWAY_NODES:
                continue
            style = (
                'color="#94a3b8" style=dotted constraint=false'
                if is_evidence
                else (
                    'color="#0f766e" penwidth=2.4' if topic_name in primary_topics else 'color="#2563eb" penwidth=1.5'
                )
            )
            lines.append(f"  {_identifier('node', publisher)} -> {topic_id} [{style}];")
        for subscriber in subscribers:
            if subscriber in publishers:
                continue
            is_evidence = subscriber in EVIDENCE_NODES
            if subscriber in GATEWAY_NODES:
                continue
            # Keep one observed evidence attachment readable without drawing a
            # recorder line for every bounded-autonomy topic it captures.
            if is_evidence and topic_name != "/yolo/detections":
                continue
            style = (
                'color="#94a3b8" style=dotted constraint=false'
                if is_evidence
                else (
                    'color="#0f766e" penwidth=2.4' if topic_name in primary_topics else 'color="#2563eb" penwidth=1.5'
                )
            )
            if is_incomplete and not is_evidence:
                style = 'color="#94a3b8" style=dashed penwidth=1.2'
            lines.append(f"  {topic_id} -> {_identifier('node', subscriber)} [{style}];")
    lines.append("}")
    return "\n".join(lines) + "\n"


def render_topology(input_path: Path, output_svg_path: Path, *, projection: str = "full") -> Path:
    """Write inspectable DOT and render the requested SVG with Graphviz dot."""

    try:
        topology = json.loads(input_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid topology JSON: {input_path}: {exc}") from exc
    if not isinstance(topology, dict):
        raise ValueError(f"topology JSON root must be an object: {input_path}")
    if projection not in {"full", "bounded-autonomy"}:
        raise ValueError(f"unknown topology projection: {projection}")
    if shutil.which("dot") is None:
        raise RuntimeError("Graphviz 'dot' is required to render topology SVGs")

    output_svg_path.parent.mkdir(parents=True, exist_ok=True)
    dot_path = output_svg_path.with_suffix(".dot")
    dot = bounded_autonomy_topology_to_dot(topology) if projection == "bounded-autonomy" else topology_to_dot(topology)
    dot_path.write_text(dot, encoding="utf-8")
    subprocess.run(["dot", "-Tsvg", str(dot_path), "-o", str(output_svg_path)], check=True)
    return dot_path


def topology_render_main() -> int:
    parser = argparse.ArgumentParser(description="Render captured ROS topology JSON with Graphviz dot.")
    parser.add_argument("input", type=Path, help="captured ros_graph/topology.json")
    parser.add_argument("output", type=Path, help="output SVG path; a sibling .dot is also written")
    parser.add_argument(
        "--projection",
        choices=("full", "bounded-autonomy"),
        default="full",
        help="documentation projection to render (default: full)",
    )
    args = parser.parse_args()
    try:
        render_topology(args.input, args.output, projection=args.projection)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(topology_render_main())
