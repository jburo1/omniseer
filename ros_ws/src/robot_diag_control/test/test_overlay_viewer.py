import unittest
from unittest import mock

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.overlay_viewer import (
    ScaledBox,
    _build_appsink_pipeline,
    _build_parser,
    _class_color,
    _format_pipeline_open_error,
    _hud_lines,
    _import_cv2,
    _opencv_gstreamer_enabled,
    _scale_detection_box,
    _validate_opencv_videoio_support,
)


class OverlayViewerTests(unittest.TestCase):
    def test_parser_defaults_to_display_mode(self):
        parser = _build_parser()

        args = parser.parse_args([])

        self.assertEqual(args.mode, "display")
        self.assertEqual(args.profile, "balanced")
        self.assertEqual(args.overlay_poll_hz, 12.0)
        self.assertEqual(args.preview_port, 7100)

    def test_build_appsink_pipeline_uses_preview_host_override(self):
        parser = _build_parser()
        args = parser.parse_args(
            [
                "--host",
                "10.0.0.2",
                "--preview-host",
                "10.0.0.3",
                "--preview-latency-ms",
                "90",
            ]
        )

        pipeline = _build_appsink_pipeline(args, decoder_element="openh264dec")

        self.assertIn("uri=srt://10.0.0.3:7100?mode=caller", pipeline)
        self.assertIn("latency=90", pipeline)
        self.assertIn("openh264dec", pipeline)
        self.assertIn("appsink", pipeline)
        self.assertIn("video/x-raw,format=BGR", pipeline)

    def test_scale_detection_box_maps_source_pixels_to_frame_pixels(self):
        detection = robot_gateway_pb2.OverlayDetection(
            bbox_center_x_px=640.0,
            bbox_center_y_px=360.0,
            bbox_width_px=128.0,
            bbox_height_px=72.0,
        )

        box = _scale_detection_box(
            detection,
            source_width=1280,
            source_height=720,
            frame_width=640,
            frame_height=360,
        )

        self.assertEqual(box, ScaledBox(288, 162, 352, 198))

    def test_scale_detection_box_clamps_to_frame(self):
        detection = robot_gateway_pb2.OverlayDetection(
            bbox_center_x_px=10.0,
            bbox_center_y_px=10.0,
            bbox_width_px=80.0,
            bbox_height_px=80.0,
        )

        box = _scale_detection_box(
            detection,
            source_width=1280,
            source_height=720,
            frame_width=640,
            frame_height=360,
        )

        self.assertEqual(box.x1, 0)
        self.assertEqual(box.y1, 0)

    def test_class_color_is_stable_and_visible(self):
        color = _class_color("person", 0)

        self.assertEqual(color, _class_color("person", 0))
        self.assertTrue(all(80 <= channel <= 219 for channel in color))

    def test_hud_lines_use_operator_view_language(self):
        snapshot = robot_gateway_pb2.OverlaySnapshot(
            status=robot_gateway_pb2.SystemStatus(
                health=robot_gateway_pb2.RobotHealth(
                    state=robot_gateway_pb2.ROBOT_HEALTH_DEGRADED,
                    ready=False,
                    summary="waiting for odometry",
                    odom_available=True,
                    odom_stale=True,
                    odom_age_ms=740,
                    linear_speed_mps=0.18,
                    angular_speed_rad_s=-0.27,
                    measured_vx_mps=0.18,
                    measured_vy_mps=0.01,
                    measured_wz_rad_s=-0.27,
                ),
                preview=robot_gateway_pb2.PreviewStatus(profile=robot_gateway_pb2.PREVIEW_PROFILE_BALANCED),
                vision=robot_gateway_pb2.VisionStatus(
                    available=True,
                    stale=False,
                    producer_fps=30.0,
                    consumer_fps=9.0,
                    last_infer_ms=104.0,
                ),
                teleop=robot_gateway_pb2.TeleopStatus(
                    state=robot_gateway_pb2.TELEOP_TIMED_OUT,
                    enabled=True,
                    timed_out=True,
                    last_command_age_ms=380,
                    last_command_vx_mps=0.2,
                    last_command_vy_mps=0.0,
                    last_command_wz_rad_s=-0.3,
                ),
            ),
            detections=robot_gateway_pb2.DetectionOverlayStatus(
                available=True,
                stale=False,
                age_ms=42,
                detection_count=3,
            ),
            events=[
                robot_gateway_pb2.OperatorEvent(sequence=1, age_ms=18, message="odometry recovered"),
            ],
        )

        lines = _hud_lines(snapshot, overlay_enabled=True, min_score=0.25)

        self.assertEqual(lines[0], "FAULT waiting for odometry | ODOM STALE")
        self.assertIn("TELEOP TIMED_OUT | NOT READY | ODOM STALE 740 ms | VISION OK", lines)
        self.assertIn("CAM 30.0 FPS | DET 9.0 FPS | LAT 104 ms | OBJ 3 | AGE 42 ms", lines)
        self.assertIn("CMD vx +0.20 vy +0.00 wz -0.30", "\n".join(lines))
        self.assertIn("MEAS vx +0.18 vy +0.01 wz -0.27", "\n".join(lines))
        self.assertIn("EVENT 18 ms odometry recovered", lines)
        self.assertNotIn("DEADMAN", "\n".join(lines))

    @mock.patch.dict("sys.modules", {"cv2": None})
    def test_import_cv2_reports_actionable_error(self):
        with self.assertRaisesRegex(RuntimeError, "python3-opencv"):
            _import_cv2()

    def test_opencv_gstreamer_enabled_reads_build_information(self):
        cv2 = mock.Mock()
        cv2.getBuildInformation.return_value = "Video I/O:\n    GStreamer:                   YES (1.20.3)\n"

        self.assertTrue(_opencv_gstreamer_enabled(cv2))

    def test_validate_opencv_videoio_support_rejects_missing_gstreamer(self):
        cv2 = mock.Mock()
        cv2.getBuildInformation.return_value = "Video I/O:\n    GStreamer:                   NO\n"

        with self.assertRaisesRegex(RuntimeError, "built without GStreamer"):
            _validate_opencv_videoio_support(cv2)

    def test_format_pipeline_open_error_includes_pipeline_and_action(self):
        message = _format_pipeline_open_error("srtsrc ! appsink")

        self.assertIn("failed to open overlay video pipeline", message)
        self.assertIn("pipeline: srtsrc ! appsink", message)
        self.assertIn("robot_preview_viewer", message)
