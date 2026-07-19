import unittest

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.gateway_client import (
    format_operator_status,
    format_overlay_snapshot,
    format_system_status,
    format_system_status_summary,
    format_teleop_response,
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
                odom_age_ms=18,
                linear_speed_mps=0.5,
                angular_speed_rad_s=0.2,
                measured_vx_mps=0.3,
                measured_vy_mps=0.4,
                measured_wz_rad_s=0.2,
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
            teleop=robot_gateway_pb2.TeleopStatus(
                state=robot_gateway_pb2.TELEOP_ENABLED,
                enabled=True,
                last_command_age_ms=25,
                max_linear_mps=0.35,
                max_angular_rad_s=0.8,
                last_command_vx_mps=0.12,
                last_command_vy_mps=0.0,
                last_command_wz_rad_s=-0.2,
            ),
        )

        formatted = format_system_status(response)

        self.assertIn("health: state=ok ready=true summary=robot healthy", formatted)
        self.assertIn("mobility: odom=fresh odom_age_ms=18", formatted)
        self.assertIn("teleop: state=enabled enabled=true timed_out=false", formatted)
        self.assertIn("last_command=(0.12,0.00,-0.20)", formatted)
        self.assertIn("vision: producer_fps=22.00", formatted)

    def test_format_operator_status_surfaces_freshness_and_faults(self):
        response = robot_gateway_pb2.SystemStatus(
            health=robot_gateway_pb2.RobotHealth(
                state=robot_gateway_pb2.ROBOT_HEALTH_DEGRADED,
                ready=False,
                summary="waiting for odometry",
                odom_available=True,
                odom_stale=True,
                odom_age_ms=740,
                linear_speed_mps=0.2,
                angular_speed_rad_s=-0.1,
                measured_vx_mps=0.18,
                measured_vy_mps=0.01,
                measured_wz_rad_s=-0.27,
            ),
            preview=robot_gateway_pb2.PreviewStatus(state=robot_gateway_pb2.PREVIEW_RUNNING),
            vision=robot_gateway_pb2.VisionStatus(
                available=True,
                stale=True,
                producer_fps=29.8,
                consumer_fps=8.4,
                last_infer_ms=116.0,
                infer_error_count=2,
            ),
            teleop=robot_gateway_pb2.TeleopStatus(
                state=robot_gateway_pb2.TELEOP_TIMED_OUT,
                enabled=True,
                timed_out=True,
                last_command_age_ms=740,
                max_linear_mps=0.35,
                max_angular_rad_s=0.8,
                last_command_vx_mps=0.2,
                last_command_vy_mps=0.0,
                last_command_wz_rad_s=-0.3,
            ),
        )

        formatted = format_operator_status(response)

        self.assertIn("TELEOP TIMED_OUT | NOT READY | ODOM STALE 740 ms | VISION STALE", formatted)
        self.assertIn("CAM 29.8 FPS | DET 8.4 FPS | LAT 116 ms", formatted)
        self.assertIn("CMD vx +0.20 vy +0.00 wz -0.30", formatted)
        self.assertIn("MEAS vx +0.18 vy +0.01 wz -0.27", formatted)
        self.assertIn("FAULT waiting for odometry | ODOMETRY STALE | VISION STALE", formatted)
        self.assertNotIn("DEADMAN", formatted)

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
            teleop=robot_gateway_pb2.TeleopStatus(
                state=robot_gateway_pb2.TELEOP_TIMED_OUT,
                timed_out=True,
                last_error="teleop deadman timeout",
            ),
        )

        formatted = format_system_status_summary(response)

        self.assertIn("health=degraded ready=false odom=unavailable", formatted)
        self.assertIn("health_summary=waiting for odometry", formatted)
        self.assertIn("preview=running/low_bw", formatted)
        self.assertIn("teleop=timed_out", formatted)
        self.assertIn("teleop_error=teleop deadman timeout", formatted)
        self.assertIn("vision=unavailable", formatted)

    def test_format_teleop_response_includes_state(self):
        response = robot_gateway_pb2.SendTeleopCommandResponse(
            accepted=True,
            message="teleop command sent",
            teleop=robot_gateway_pb2.TeleopStatus(
                state=robot_gateway_pb2.TELEOP_ENABLED,
                enabled=True,
            ),
        )

        formatted = format_teleop_response(response)

        self.assertIn("accepted=True", formatted)
        self.assertIn("message=teleop command sent", formatted)
        self.assertIn("state=enabled", formatted)

    def test_format_overlay_snapshot_includes_status_and_detections(self):
        response = robot_gateway_pb2.OverlaySnapshot(
            status=robot_gateway_pb2.SystemStatus(
                health=robot_gateway_pb2.RobotHealth(state=robot_gateway_pb2.ROBOT_HEALTH_OK, ready=True),
                preview=robot_gateway_pb2.PreviewStatus(
                    state=robot_gateway_pb2.PREVIEW_RUNNING,
                    profile=robot_gateway_pb2.PREVIEW_PROFILE_BALANCED,
                ),
                vision=robot_gateway_pb2.VisionStatus(available=True, producer_fps=20.0),
                teleop=robot_gateway_pb2.TeleopStatus(state=robot_gateway_pb2.TELEOP_ENABLED),
            ),
            detections=robot_gateway_pb2.DetectionOverlayStatus(
                available=True,
                stale=False,
                age_ms=42,
                source_width_px=1280,
                source_height_px=720,
                detection_count=1,
                detections=[
                    robot_gateway_pb2.OverlayDetection(
                        class_id=3,
                        class_name="person",
                        score=0.875,
                        bbox_center_x_px=320.0,
                        bbox_center_y_px=180.0,
                        bbox_width_px=100.0,
                        bbox_height_px=80.0,
                    )
                ],
            ),
            events=[
                robot_gateway_pb2.OperatorEvent(sequence=4, age_ms=35, message="odometry recovered"),
            ],
        )

        formatted = format_overlay_snapshot(response)

        self.assertIn("preview=running/balanced", formatted)
        self.assertIn("detections: fresh age_ms=42 count=1 source=1280x720", formatted)
        self.assertIn("class=person score=0.88 bbox=(270.0,140.0,100.0,80.0)", formatted)
        self.assertIn("event: seq=4 age_ms=35 message=odometry recovered", formatted)

    def test_format_overlay_snapshot_handles_unavailable_detections(self):
        response = robot_gateway_pb2.OverlaySnapshot(
            detections=robot_gateway_pb2.DetectionOverlayStatus(
                available=False,
                source_width_px=1280,
                source_height_px=720,
            )
        )

        formatted = format_overlay_snapshot(response)

        self.assertIn("detections: unavailable source=1280x720", formatted)
