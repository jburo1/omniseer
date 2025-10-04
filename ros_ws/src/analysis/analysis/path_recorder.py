#!/usr/bin/env python3
"""Publish recorded odom and simulation paths for comparison."""

from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class PathRecorder(Node):
    """Cache and replay odom and simulation paths."""

    def __init__(self) -> None:
        super().__init__("path_recorder")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("sim_topic", "/gz_odom")
        self.declare_parameter("odom_path_topic", "/odom_path")
        self.declare_parameter("sim_path_topic", "/sim_path")
        self.declare_parameter("queue_size", 100)
        self.declare_parameter("max_path_length", 500)

        odom_topic = self.get_parameter("odom_topic").value
        sim_topic = self.get_parameter("sim_topic").value
        odom_path_topic = self.get_parameter("odom_path_topic").value
        sim_path_topic = self.get_parameter("sim_path_topic").value
        queue_size = self.get_parameter("queue_size").value
        max_path_length = self.get_parameter("max_path_length").value

        self.get_logger().info(f"PathRecorder starting: odom_topic={odom_topic}, sim_topic={sim_topic}")
        # self.get_logger().info(
        #     "PathRecorder starting: odom_topic=%s, sim_topic=%s",
        #     odom_topic,
        #     sim_topic,
        # )

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        self.odom_buffer: deque[PoseStamped] = deque(maxlen=max_path_length)
        self.sim_buffer: deque[PoseStamped] = deque(maxlen=max_path_length)

        self.odom_path = Path()
        self.odom_path.header.frame_id = "odom"
        self.sim_path = Path()
        self.sim_path.header.frame_id = "odom"

        self.odom_publisher = self.create_publisher(Path, odom_path_topic, qos)
        self.sim_publisher = self.create_publisher(Path, sim_path_topic, qos)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)
        self.create_subscription(Odometry, sim_topic, self.sim_callback, qos)

    def odom_callback(self, message: Odometry) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose

        self.odom_path.header.stamp = message.header.stamp
        self.odom_buffer.append(pose)
        self.odom_path.poses = list(self.odom_buffer)
        self.odom_publisher.publish(self.odom_path)

    def sim_callback(self, message: Odometry) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose

        self.sim_path.header.stamp = message.header.stamp
        self.sim_buffer.append(pose)
        self.sim_path.poses = list(self.sim_buffer)
        self.sim_publisher.publish(self.sim_path)


def main() -> None:
    init(args=None)
    node = PathRecorder()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()
