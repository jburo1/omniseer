import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.run_video import (
    NON_TARGET_COLOR,
    TARGET_COLOR,
    TimedDetections,
    _draw_detections,
    _target_class_from_manifest,
    nearest_detections,
    remux_source_video,
    transcode_overlay_video,
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

    def test_timestamp_matching_uses_only_current_or_prior_detection(self):
        records = [TimedDetections(100.0, ()), TimedDetections(101.0, ())]

        self.assertIsNone(nearest_detections(records, video_time_sec=3.0, max_age_sec=0.5))
        self.assertIsNone(nearest_detections(records, video_time_sec=0.75, max_age_sec=0.5))
        self.assertEqual(nearest_detections(records, video_time_sec=1.1, max_age_sec=0.5), records[1])


if __name__ == "__main__":
    unittest.main()
