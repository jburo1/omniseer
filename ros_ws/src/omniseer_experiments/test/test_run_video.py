import tempfile
import unittest
from pathlib import Path
from subprocess import CalledProcessError, CompletedProcess
from unittest.mock import patch

from omniseer_experiments.run_video import (
    DETECTION_LABEL_FONT_SCALE,
    DETECTION_LABEL_THICKNESS,
    NON_TARGET_COLOR,
    TARGET_COLOR,
    TimedDetections,
    VideoTimingAnchor,
    _draw_detections,
    _target_class_from_manifest,
    build_run_video,
    frame_robot_time,
    load_video_start_time,
    load_video_timing_anchor,
    nearest_detections,
    remux_source_video,
    repair_preview_wrap_video,
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

    def test_preview_wrap_repair_uses_explicit_circular_filter_and_h264(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))
            if args[0][0] == "ffprobe":
                return CompletedProcess(args[0], 0, stdout='{"streams":[{"width":1280,"height":720}]}')
            return CompletedProcess(args[0], 0, stdout="")

        repair_preview_wrap_video(Path("video/source.mp4"), Path("video/source.corrected.mp4"), runner=runner)

        self.assertEqual(calls[0][0][0][0], "ffprobe")
        self.assertEqual(
            calls[1][0][0],
            [
                "ffmpeg",
                "-y",
                "-i",
                "video/source.mp4",
                "-filter_complex",
                "[0:v]split=2[main_input][wrapped_input];"
                "[main_input]crop=1272:720:8:0[main];"
                "[wrapped_input]crop=8:720:0:0[wrapped];"
                "[main][wrapped]hstack=inputs=2[corrected]",
                "-map",
                "[corrected]",
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
                "-fps_mode",
                "passthrough",
                "video/source.corrected.mp4",
            ],
        )

    def test_preview_wrap_repair_rejects_unsupported_geometry(self):
        def runner(*args, **_kwargs):
            return CompletedProcess(args[0], 0, stdout='{"streams":[{"width":640,"height":480}]}')

        with self.assertRaisesRegex(ValueError, "supports only 1280x720"):
            repair_preview_wrap_video(Path("video/source.mp4"), Path("video/source.corrected.mp4"), runner=runner)

    def test_build_preserves_raw_remux_and_uses_corrected_source_for_overlay(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            source_ts = run_dir / "video" / "source.ts"
            source_ts.parent.mkdir()
            source_ts.write_bytes(b"immutable raw evidence")
            (source_ts.parent / "timing.json").write_text(
                '{"anchor_video_time_ns": 125000000, "anchor_robot_time_ns": 1786061000123456789}',
                encoding="utf-8",
            )
            (run_dir / "detections.jsonl").write_text("", encoding="utf-8")
            raw_before = source_ts.read_bytes()

            with (
                patch("omniseer_experiments.run_video.remux_source_video") as remux,
                patch("omniseer_experiments.run_video.repair_preview_wrap_video") as repair,
                patch("omniseer_experiments.run_video.render_overlay") as render,
            ):
                remux.side_effect = lambda _source, output, **_kwargs: output.write_bytes(b"raw remux")
                repair.side_effect = lambda _source, output, **_kwargs: output.write_bytes(b"corrected derivative")
                build_run_video(run_dir)

            self.assertEqual(source_ts.read_bytes(), raw_before)
            remux.assert_called_once_with(source_ts, run_dir / "video" / "source.mp4", runner=unittest.mock.ANY)
            repair.assert_called_once_with(
                run_dir / "video" / "source.mp4",
                run_dir / "video" / "source.corrected.mp4",
                runner=unittest.mock.ANY,
            )
            self.assertTrue((run_dir / "video" / "source.mp4").is_file())
            self.assertTrue((run_dir / "video" / "source.corrected.mp4").is_file())
            self.assertEqual(render.call_args.args[0], run_dir / "video" / "source.corrected.mp4")
            self.assertEqual(render.call_args.kwargs["timing_anchor"], VideoTimingAnchor(0.125, 1786061000.1234567))

    def test_build_reports_captured_media_stderr(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            source_ts = run_dir / "video" / "source.ts"
            source_ts.parent.mkdir()
            source_ts.write_bytes(b"immutable raw evidence")
            (source_ts.parent / "timing.json").write_text(
                '{"anchor_video_time_ns": 0, "anchor_robot_time_ns": 0}', encoding="utf-8"
            )
            (run_dir / "detections.jsonl").write_text("", encoding="utf-8")

            with (
                patch("omniseer_experiments.run_video.remux_source_video"),
                patch(
                    "omniseer_experiments.run_video.repair_preview_wrap_video",
                    side_effect=CalledProcessError(1, ["ffmpeg"], stderr="Invalid filter graph"),
                ),
            ):
                with self.assertRaisesRegex(RuntimeError, "Invalid filter graph"):
                    build_run_video(run_dir)

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
                self.text = _args

        class Frame:
            shape = (360, 640, 3)

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
            source_width=1280,
            source_height=720,
        )

        self.assertEqual(
            cv2.rectangles,
            [((270, 155), (370, 205), TARGET_COLOR), ((40, 40), (60, 60), NON_TARGET_COLOR)],
        )
        self.assertEqual(cv2.text[4], DETECTION_LABEL_FONT_SCALE)
        self.assertEqual(cv2.text[6], DETECTION_LABEL_THICKNESS)

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
