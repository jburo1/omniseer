#!/usr/bin/env python3
"""Convert a LaserScan from /sonar into a Range message."""

import math

from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    """Publish a range message using the closest valid beam."""

    def __init__(self) -> None:
        super().__init__("scan_to_range")
        self.declare_parameter("scan_topic", "/sonar")
        self.declare_parameter("range_topic", "/range")

        scan_topic = self.get_parameter("scan_topic").value
        range_topic = self.get_parameter("range_topic").value

        self.publisher = self.create_publisher(Range, range_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"ScanToRange: {scan_topic} -> {range_topic}")
        # self.get_logger().info("ScanToRange: %s -> %s", scan_topic, range_topic)

    def scan_callback(self, scan: LaserScan) -> None:
        best = None
        for value in scan.ranges:
            if not math.isfinite(value):
                continue
            if scan.range_min <= value <= scan.range_max:
                if best is None or value < best:
                    best = value

        message = Range()
        message.header = scan.header
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = scan.angle_max - scan.angle_min
        message.min_range = scan.range_min
        message.max_range = scan.range_max
        message.range = best if best is not None else scan.range_max

        if message.range < message.min_range:
            message.range = message.min_range
        if message.range > message.max_range:
            message.range = message.max_range

        self.publisher.publish(message)


def main() -> None:
    init(args=None)
    node = ScanToRange()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()
