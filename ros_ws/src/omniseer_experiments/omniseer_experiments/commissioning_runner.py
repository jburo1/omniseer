"""ROS publisher implementation for the commissioning motion sequence."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from omniseer_experiments.commissioning_sequence import COMMAND_RATE_HZ, DEFAULT_PHASES, MotionCommand, execute_sequence

COMMAND_TOPIC = "/cmd_vel_autonomy"
PHASE_TOPIC = "/commissioning/phase"


class CommissioningMotionRunner(Node):
    def __init__(self) -> None:
        super().__init__("commissioning_motion_runner")
        self._command_publisher = self.create_publisher(TwistStamped, COMMAND_TOPIC, 10)
        self._phase_publisher = self.create_publisher(String, PHASE_TOPIC, 10)

    def publish_command(self, command: MotionCommand) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = command.vx_m_s
        message.twist.linear.y = command.vy_m_s
        message.twist.angular.z = command.wz_rad_s
        self._command_publisher.publish(message)

    def publish_phase(self, phase_name: str) -> None:
        self._phase_publisher.publish(String(data=phase_name))

    def run(self) -> bool:
        return execute_sequence(
            DEFAULT_PHASES,
            publish_command=self.publish_command,
            publish_phase=self.publish_phase,
            is_running=rclpy.ok,
            monotonic=time.monotonic,
            sleep=time.sleep,
            command_rate_hz=COMMAND_RATE_HZ,
        )
