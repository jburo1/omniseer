import unittest

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.gateway_client import (
    format_system_status,
    format_system_status_summary,
)


class GatewayClientFormattingTests(unittest.TestCase):
    def test_format_system_status_includes_robot_health_and_mobility(self):
        response = robot_gateway_pb2.SystemStatus(
            gateway_name="robot_diag_control_cpp",
            gateway_version="0.1.0",
            health=robot_gateway_pb2.RobotHealth(
                state=robot_gateway_pb2.ROBOT_HEALTH_OK,
                ready=True,
                summary="robot healthy",
                odom_available=True,
                odom_stale=False,
                linear_speed_mps=0.5,
                angular_speed_rad_s=0.2,
            ),
            preview=robot_gateway_pb2.PreviewStatus(
                state=robot_gateway_pb2.PREVIEW_DISABLED,
                profile=robot_gateway_pb2.PREVIEW_PROFILE_BALANCED,
            ),
            vision=robot_gateway_pb2.VisionStatus(
                available=True,
                stale=False,
                producer_fps=22.0,
                consumer_fps=21.5,
                last_infer_ms=6.5,
                infer_error_count=1,
                capture_fatal_error_count=0,
            ),
        )

        formatted = format_system_status(response)

        self.assertIn("health: state=ok ready=true summary=robot healthy", formatted)
        self.assertIn("mobility: odom=fresh linear_speed_mps=0.50 angular_speed_rad_s=0.20", formatted)
        self.assertIn("vision: producer_fps=22.00", formatted)

    def test_format_system_status_summary_includes_health_state(self):
        response = robot_gateway_pb2.SystemStatus(
            health=robot_gateway_pb2.RobotHealth(
                state=robot_gateway_pb2.ROBOT_HEALTH_DEGRADED,
                ready=False,
                summary="waiting for odometry",
                odom_available=False,
            ),
            preview=robot_gateway_pb2.PreviewStatus(
                state=robot_gateway_pb2.PREVIEW_RUNNING,
                profile=robot_gateway_pb2.PREVIEW_PROFILE_LOW_BW,
            ),
            vision=robot_gateway_pb2.VisionStatus(available=False),
        )

        formatted = format_system_status_summary(response)

        self.assertIn("health=degraded ready=false odom=unavailable", formatted)
        self.assertIn("health_summary=waiting for odometry", formatted)
        self.assertIn("preview=running/low_bw", formatted)
        self.assertIn("vision=unavailable", formatted)
