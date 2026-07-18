import unittest
from unittest import mock

from robot_diag_control.api import robot_gateway_pb2
from robot_diag_control.overlay_viewer import (
    ScaledBox,
    _build_appsink_pipeline,
    _build_parser,
    _class_color,
    _import_cv2,
    _scale_detection_box,
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

    @mock.patch.dict("sys.modules", {"cv2": None})
    def test_import_cv2_reports_actionable_error(self):
        with self.assertRaisesRegex(RuntimeError, "python3-opencv"):
            _import_cv2()
