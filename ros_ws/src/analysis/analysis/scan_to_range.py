#!/usr/bin/env python3
"""
Convert a LaserScan from /sonar into a Range message on /range.
"""
from rclpy import init, shutdown, spin
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class ScanToRange(Node):
    def __init__(self):
        super().__init__('scan_to_range')

        self.declare_parameter('scan_topic', '/sonar')
        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('queue_size', 5)

        scan_topic  = self.get_parameter('scan_topic').value
        range_topic = self.get_parameter('range_topic').value
        queue_size  = self.get_parameter('queue_size').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size
        )

        self.pub = self.create_publisher(Range, range_topic, 5)
        self.create_subscription(LaserScan, scan_topic,
                                 self.scan_cb, qos)

        self.get_logger().info(
            f'ScanToRange starting: scan_topic={scan_topic}, range_topic={range_topic}')


    def scan_cb(self, scan: LaserScan):
        rng = Range()
        rng.header          = scan.header
        rng.radiation_type  = Range.ULTRASOUND
        rng.field_of_view   = scan.angle_max - scan.angle_min
        rng.min_range       = scan.range_min
        rng.max_range       = scan.range_max
        rng.range           = np.nanmin(scan.ranges)
        finite = [r for r in scan.ranges if np.isfinite(r)]
        rng.range = min(finite) if finite else scan.range_max + 0.01
        self.pub.publish(rng)

def main():
    init(args=None)
    node = ScanToRange()
    try:
        spin(node)
    # swallow Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()

if __name__ == '__main__':
    main()
