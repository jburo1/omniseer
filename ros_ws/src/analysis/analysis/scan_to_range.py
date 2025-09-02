#!/usr/bin/env python3
"""
Convert a LaserScan from /sonar into a Range message on /range.
"""
from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
import math

class ScanToRange(Node):
    def __init__(self):
        super().__init__('scan_to_range')
        self.declare_parameter('scan_topic', '/sonar')
        self.declare_parameter('range_topic', '/range')

        scan_topic  = self.get_parameter('scan_topic').value
        range_topic = self.get_parameter('range_topic').value

        self.pub = self.create_publisher(Range, range_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.get_logger().info(f'ScanToRange: {scan_topic} -> {range_topic}')

    def scan_cb(self, scan: LaserScan):
        # one pass, no allocations
        best = None
        for v in scan.ranges:
            if math.isfinite(v) and scan.range_min <= v <= scan.range_max:
                best = v if best is None or v < best else best

        rng = Range()
        rng.header         = scan.header
        rng.radiation_type = Range.ULTRASOUND
        rng.field_of_view  = scan.angle_max - scan.angle_min
        rng.min_range      = scan.range_min
        rng.max_range      = scan.range_max
        rng.range          = best if best is not None else scan.range_max  # exact max => clear_on_max_reading works
        # clamp for safety
        if rng.range < rng.min_range: rng.range = rng.min_range
        if rng.range > rng.max_range: rng.range = rng.max_range

        self.pub.publish(rng)

def main():
    init()
    node = ScanToRange()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()

if __name__ == '__main__':
    main()

