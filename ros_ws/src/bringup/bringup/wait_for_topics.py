"""Wait for first messages on required ROS topics."""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    type_name: str


def parse_topic_spec(value: str) -> TopicSpec:
    try:
        topic, type_name = value.split(":", 1)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("topic spec must use TOPIC:PACKAGE/msg/TYPE") from exc

    if not topic.startswith("/"):
        raise argparse.ArgumentTypeError("topic must be absolute")

    type_parts = type_name.split("/")
    if len(type_parts) != 3 or type_parts[1] != "msg" or not all(type_parts):
        raise argparse.ArgumentTypeError("message type must use PACKAGE/msg/TYPE")

    return TopicSpec(topic=topic, type_name=type_name)


def resolve_message_type(type_name: str):
    package, namespace, class_name = type_name.split("/")
    module = importlib.import_module(f"{package}.{namespace}")
    return getattr(module, class_name)


def wait_for_messages(specs: list[TopicSpec], timeout_sec: float) -> set[str]:
    rclpy.init(args=None)
    node = Node("wait_for_topics")
    received_topics: set[str] = set()

    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    try:
        subscriptions = []
        for spec in specs:
            message_type = resolve_message_type(spec.type_name)

            def _mark_received(_msg, topic=spec.topic):
                received_topics.add(topic)

            subscriptions.append(node.create_subscription(message_type, spec.topic, _mark_received, qos))

        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while set(spec.topic for spec in specs) - received_topics and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        return set(spec.topic for spec in specs) - received_topics
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def _nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("timeout must be non-negative")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", required=True, type=_nonnegative_float)
    parser.add_argument("--topic", action="append", required=True, type=parse_topic_spec)
    args = parser.parse_args()

    topic_list = ", ".join(spec.topic for spec in args.topic)
    print(f"Waiting up to {args.timeout_sec:.1f}s for first messages on: {topic_list}")

    try:
        missing_topics = wait_for_messages(args.topic, args.timeout_sec)
    except KeyboardInterrupt:
        print("Interrupted while waiting for required topics", file=sys.stderr)
        return 130

    if missing_topics:
        missing_list = ", ".join(sorted(missing_topics))
        print(f"Timed out waiting for first messages on: {missing_list}", file=sys.stderr)
        return 1

    print(f"Received first messages on all required topics: {topic_list}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
