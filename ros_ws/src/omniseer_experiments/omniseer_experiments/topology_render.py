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


def render_topology(input_path: Path, output_svg_path: Path) -> Path:
    """Write inspectable DOT and render the requested SVG with Graphviz dot."""

    try:
        topology = json.loads(input_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid topology JSON: {input_path}: {exc}") from exc
    if not isinstance(topology, dict):
        raise ValueError(f"topology JSON root must be an object: {input_path}")
    if shutil.which("dot") is None:
        raise RuntimeError("Graphviz 'dot' is required to render topology SVGs")

    output_svg_path.parent.mkdir(parents=True, exist_ok=True)
    dot_path = output_svg_path.with_suffix(".dot")
    dot_path.write_text(topology_to_dot(topology), encoding="utf-8")
    subprocess.run(["dot", "-Tsvg", str(dot_path), "-o", str(output_svg_path)], check=True)
    return dot_path


def topology_render_main() -> int:
    parser = argparse.ArgumentParser(description="Render captured ROS topology JSON with Graphviz dot.")
    parser.add_argument("input", type=Path, help="captured ros_graph/topology.json")
    parser.add_argument("output", type=Path, help="output SVG path; a sibling .dot is also written")
    args = parser.parse_args()
    try:
        render_topology(args.input, args.output)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(topology_render_main())
