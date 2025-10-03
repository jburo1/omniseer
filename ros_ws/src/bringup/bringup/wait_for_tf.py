"""Wait for a transform to become available before exiting."""

import argparse
import sys
import time
from typing import Optional

from rclpy import init, ok, shutdown, spin_once
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("target")
    args = parser.parse_args()

    init(args=None)
    node: Optional[Node] = None
    found = False
    try:
        node = Node("wait_tf")
        buffer = Buffer()
        _listener = TransformListener(buffer, node)
        deadline = time.time() + 60.0

        while ok() and time.time() < deadline:
            spin_once(node, timeout_sec=0.1)
            if buffer.can_transform(args.source, args.target, Time()):
                found = True
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        shutdown()
        sys.exit(0 if found else 1)


if __name__ == "__main__":
    main()
