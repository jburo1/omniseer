from rclpy import init, shutdown
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

class PathRecorder(Node):
    def __init__(self, source_frame, target_topic):
        super().__init__('path_recorder')

        self.declare_parameter('source_frame', source_frame)
        self.declare_parameter('target_topic', target_topic)
        self.declare_parameter('rate', 20)

        src = self.get_parameter('source_frame').get_parameter_value().string_value
        topic = self.get_parameter('target_topic').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().integer_value

        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.path = Path()
        self.path.header.frame_id = src
        self.path.poses = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=500
        )

        self.pub = self.create_publisher(Path, topic, qos)

        self.timer = self.create_timer(1 / rate, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        try:
            tf = self.buf.lookup_transform(
                self.path.header.frame_id,
                'base_link',
                now,
                timeout=Duration(seconds=0.1))

            ps = PoseStamped()
            ps.header = tf.header
            ps.pose.position = tf.transform.translation
            ps.pose.orientation = tf.transform.rotation

            self.path.header.stamp = ps.header.stamp
            self.path.poses.append(ps)
            self.pub.publish(self.path)
        except Exception:
            pass

def main():
    init(args=None)
    node_odom = PathRecorder('odom', 'odom_path')
    node_gt = PathRecorder('', 'gr_path')
    executor = MultiThreadedExecutor()
    executor.add_node(node_odom)
    executor.add_node(node_gt)

    try:
        executor.spin()
    # swallow Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node_odom.destroy_node()
        node_gt.destroy_node()
        shutdown()

if __name__ == '__main__':
    main()
