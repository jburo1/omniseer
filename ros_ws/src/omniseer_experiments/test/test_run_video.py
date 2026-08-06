import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.run_video import (
    InferenceFrame,
    read_inference_frames,
    remux_source_video,
    render_overlay_frames,
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

    def test_overlay_draws_only_the_detection_associated_with_each_inference_frame(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            first = Path(temp_dir) / "first.jpg"
            second = Path(temp_dir) / "second.jpg"
            third = Path(temp_dir) / "third.jpg"
            for path in (first, second, third):
                path.touch()
            frames = (
                InferenceFrame(first, ({"bbox": {"x1": 10, "y1": 20, "x2": 30, "y2": 40}},), 1.0, 0.0, 0.0),
                InferenceFrame(second, ({"bbox": {"x1": 50, "y1": 60, "x2": 70, "y2": 80}},), 1.0, 0.0, 0.0),
                InferenceFrame(third, (), 1.0, 0.0, 0.0),
            )
            cv2 = _FakeCv2()

            render_overlay_frames(Path(temp_dir) / "overlay.mp4", frames, cv2_module=cv2)

        self.assertEqual(cv2.rectangles, [((10, 20), (30, 40)), ((50, 60), (70, 80))])
        self.assertEqual([frame.rectangles for frame in cv2.writer.frames], [1, 1, 0])

    def test_overlay_colors_target_and_non_target_detections(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            image = Path(temp_dir) / "frame.jpg"
            image.touch()
            frames = (
                InferenceFrame(
                    image,
                    (
                        {"class_name": "chair", "bbox": {"x1": 10, "y1": 20, "x2": 30, "y2": 40}},
                        {"class_name": "person", "bbox": {"x1": 50, "y1": 60, "x2": 70, "y2": 80}},
                    ),
                    1.0,
                    0.0,
                    0.0,
                    target_class="chair",
                ),
            )
            cv2 = _FakeCv2()

            render_overlay_frames(Path(temp_dir) / "overlay.mp4", frames, cv2_module=cv2)

        self.assertEqual(cv2.colors, [(0, 255, 0), (0, 0, 255)])

    def test_read_inference_frames_uses_target_class_from_manifest(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            frames_dir = run_dir / "evidence" / "frames"
            frames_dir.mkdir(parents=True)
            (frames_dir / "frame_7.jpg").touch()
            (run_dir / "manifest.yaml").write_text(
                "launch:\n  args:\n    - autonomy_target_class:=chair\n", encoding="utf-8"
            )
            (run_dir / "evidence" / "evidence.jsonl").write_text(
                '{"artifact_type":"sampled_frame","image_path":"evidence/frames/frame_7.jpg",'
                '"remap":{"scale":1,"pad_x":0,"pad_y":0},"detections":[]}\n',
                encoding="utf-8",
            )

            frames = read_inference_frames(run_dir)

        self.assertEqual(frames[0].target_class, "chair")

    def test_incomplete_or_missing_evidence_frames_are_omitted(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            frames_dir = run_dir / "evidence" / "frames"
            frames_dir.mkdir(parents=True)
            (frames_dir / "frame_7.jpg").touch()
            (run_dir / "evidence" / "evidence.jsonl").write_text(
                "\n".join(
                    (
                        '{"artifact_type":"sampled_frame","image_path":"evidence/frames/frame_7.jpg","remap":{"scale":1,"pad_x":0,"pad_y":0},"detections":[]}',
                        '{"artifact_type":"sampled_frame","image_path":"evidence/frames/missing.jpg","remap":{"scale":1,"pad_x":0,"pad_y":0},"detections":[]}',
                        '{"artifact_type":"sampled_frame","image_path":"evidence/frames/frame_7.jpg","detections":[]}',
                    )
                )
                + "\n",
                encoding="utf-8",
            )

            frames = read_inference_frames(run_dir)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].detections, ())


class _Frame:
    shape = (640, 640, 3)

    def __init__(self):
        self.rectangles = 0


class _Writer:
    def __init__(self):
        self.frames = []

    def isOpened(self):
        return True

    def write(self, frame):
        self.frames.append(frame)

    def release(self):
        pass


class _FakeCv2:
    IMREAD_COLOR = 1
    INTER_AREA = 2
    FONT_HERSHEY_SIMPLEX = 0
    LINE_AA = 0

    def __init__(self):
        self.rectangles = []
        self.colors = []
        self.writer = _Writer()

    def imread(self, _path, _mode):
        return _Frame()

    def VideoWriter_fourcc(self, *_args):
        return 0

    def VideoWriter(self, *_args):
        return self.writer

    def rectangle(self, frame, top_left, bottom_right, color, _thickness):
        frame.rectangles += 1
        self.rectangles.append((top_left, bottom_right))
        self.colors.append(color)

    def putText(self, *_args):
        pass

    def resize(self, frame, _size, interpolation):
        self.assertEqual(interpolation, self.INTER_AREA)
        return frame


if __name__ == "__main__":
    unittest.main()
