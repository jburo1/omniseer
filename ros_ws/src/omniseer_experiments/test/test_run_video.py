import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.run_video import (
    NON_TARGET_COLOR,
    TARGET_COLOR,
    TimedDetections,
    VideoTimingAnchor,
    _draw_detections,
    _target_class_from_manifest,
    frame_robot_time,
    load_video_start_time,
    load_video_timing_anchor,
    nearest_detections,
    remux_source_video,
    transcode_overlay_video,
    video_relative_to_robot_time,
)


class RunVideoTests(unittest.TestCase):
    def test_remux_uses_ffmpeg_stream_copy(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))

        remux_source_video(Path("video/source.ts"), Path("video/source.mp4"), runner=runner)

        self.assertEqual(calls[0][0][0], ["ffmpeg", "-y", "-i", "video/source.ts", "-c", "copy", "video/source.mp4"])
        self.assertTrue(calls[0][1]["check"])

    def test_overlay_transcode_uses_browser_compatible_h264(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))

        transcode_overlay_video(Path("video/overlay.rendered.mp4"), Path("video/overlay.mp4"), runner=runner)

        self.assertEqual(
            calls[0][0][0],
            [
                "ffmpeg",
                "-y",
                "-i",
                "video/overlay.rendered.mp4",
                "-map",
                "0:v:0",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-crf",
                "18",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                "video/overlay.mp4",
            ],
        )
        self.assertTrue(calls[0][1]["check"])

    def test_overlay_uses_source_camera_coordinates_and_target_colors(self):
        class FakeCv2:
            FONT_HERSHEY_SIMPLEX = 0
            LINE_AA = 0

            def __init__(self):
                self.rectangles = []

            def rectangle(self, _frame, top_left, bottom_right, color, _thickness):
                self.rectangles.append((top_left, bottom_right, color))

            def putText(self, *_args):
                pass

        class Frame:
            shape = (720, 1280, 3)

        cv2 = FakeCv2()
        _draw_detections(
            cv2,
            Frame(),
            [
                {
                    "class_name": "person",
                    "bbox": {"center_x": 640.0, "center_y": 360.0, "size_x": 200.0, "size_y": 100.0},
                },
                {
                    "class_name": "chair",
                    "bbox": {"center_x": 100.0, "center_y": 100.0, "size_x": 40.0, "size_y": 40.0},
                },
            ],
            target_class="person",
        )

        self.assertEqual(
            cv2.rectangles,
            [((540, 310), (740, 410), TARGET_COLOR), ((80, 80), (120, 120), NON_TARGET_COLOR)],
        )

    def test_target_class_is_read_from_manifest(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = Path(temp_dir) / "manifest.yaml"
            manifest_path.write_text("launch:\n  args:\n    - autonomy_target_class:=chair\n", encoding="utf-8")

            self.assertEqual(_target_class_from_manifest(manifest_path), "chair")

    def test_loads_video_start_time(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            timing_path = Path(temp_dir) / "timing.json"
            timing_path.write_text('{"video_start_time_ns": 1786061000123456789}', encoding="utf-8")

            self.assertEqual(load_video_start_time(timing_path), 1786061000.1234567)

    def test_loads_first_recorded_buffer_timing_anchor(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            timing_path = Path(temp_dir) / "timing.json"
            timing_path.write_text(
                '{"anchor_video_time_ns": 125000000, "anchor_robot_time_ns": 1786061000123456789}',
                encoding="utf-8",
            )

            self.assertEqual(
                load_video_timing_anchor(timing_path),
                VideoTimingAnchor(0.125, 1786061000.1234567),
            )

    def test_maps_video_time_from_first_recorded_buffer_to_robot_realtime(self):
        anchor = VideoTimingAnchor(0.125, 1786061000.0)
        self.assertEqual(video_relative_to_robot_time(anchor, 0.250), 1786061000.125)

    def test_frame_time_uses_first_recorded_buffer_anchor(self):
        records = [TimedDetections(100.0, ()), TimedDetections(100.08, ())]
        frame_time = frame_robot_time(
            0.125,
            records,
            timing_anchor=VideoTimingAnchor(0.050, 99.925),
        )

        self.assertEqual(frame_time, 100.0)

    def test_timestamp_matching_selects_nearest_detection_before_frame(self):
        records = [TimedDetections(100.02, ()), TimedDetections(100.09, ())]

        self.assertEqual(nearest_detections(records, frame_robot_time_sec=100.05), records[0])

    def test_timestamp_matching_selects_nearest_detection_after_frame(self):
        records = [TimedDetections(100.02, ()), TimedDetections(100.09, ())]

        self.assertEqual(nearest_detections(records, frame_robot_time_sec=100.06), records[1])

    def test_timestamp_matching_rejects_stale_detections(self):
        records = [TimedDetections(100.0, ())]

        self.assertIsNone(nearest_detections(records, frame_robot_time_sec=100.101))

    def test_missing_timing_metadata_uses_legacy_first_detection_relative_time(self):
        records = [TimedDetections(100.0, ()), TimedDetections(100.08, ())]
        video_start_time_sec = load_video_start_time(Path("missing-timing.json"))
        frame_robot_time_sec = frame_robot_time(
            0.07,
            records,
            video_start_time_sec=video_start_time_sec,
        )

        self.assertIsNone(video_start_time_sec)
        self.assertEqual(nearest_detections(records, frame_robot_time_sec=frame_robot_time_sec), records[1])


if __name__ == "__main__":
    unittest.main()
