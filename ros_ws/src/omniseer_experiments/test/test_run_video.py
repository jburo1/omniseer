import json
import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.run_video import (
    NON_TARGET_COLOR,
    TARGET_COLOR,
    RecordedFrame,
    _target_class_from_manifest,
    read_recorded_frames,
    recorded_frame_rate,
    render_video_frames,
    transcode_video,
)


class _Frame:
    shape = (640, 640, 3)

    def __init__(self, name):
        self.name = name
        self.rectangles = []
        self.labels = []

    def copy(self):
        return _Frame(self.name)


class _Writer:
    def __init__(self, path, fps, size):
        self.path = path
        self.fps = fps
        self.size = size
        self.frames = []
        self.released = False

    def isOpened(self):
        return True

    def write(self, frame):
        self.frames.append(frame)

    def release(self):
        self.released = True


class _Cv2:
    IMREAD_COLOR = 1
    FONT_HERSHEY_SIMPLEX = 0
    LINE_AA = 0

    def __init__(self):
        self.writers = []

    def imread(self, path, _mode):
        return _Frame(Path(path).stem)

    def VideoWriter_fourcc(self, *_args):
        return 1

    def VideoWriter(self, path, _fourcc, fps, size):
        writer = _Writer(path, fps, size)
        self.writers.append(writer)
        return writer

    def rectangle(self, frame, top_left, bottom_right, color, _thickness):
        frame.rectangles.append((top_left, bottom_right, color))

    def putText(self, frame, _label, _origin, _font, _scale, color, _thickness, _line_type):
        frame.labels.append(color)


def _frame(path, timestamp, detections=()):
    return RecordedFrame(
        image_path=Path(path),
        detections=tuple(detections),
        remap_scale=1.0,
        remap_pad_x=0.0,
        remap_pad_y=0.0,
        capture_ts_real_ns=timestamp,
    )


class RunVideoTests(unittest.TestCase):
    def test_transcode_uses_browser_compatible_h264(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))

        transcode_video(Path("video/source.rendered.mp4"), Path("video/source.mp4"), runner=runner)

        self.assertEqual(calls[0][0][0][-2:], ["+faststart", "video/source.mp4"])
        self.assertIn("libx264", calls[0][0][0])
        self.assertTrue(calls[0][1]["check"])

    def test_recorded_frame_reader_preserves_exact_order_and_detection_association(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            frames_dir = run_dir / "video" / "frames"
            frames_dir.mkdir(parents=True)
            (frames_dir / "frame_2.jpg").write_bytes(b"jpeg")
            (frames_dir / "frame_1.jpg").write_bytes(b"jpeg")
            records = [
                {
                    "image_path": "frames/frame_2.jpg",
                    "capture_ts_real_ns": 200,
                    "remap": {"scale": 1.0, "pad_x": 0, "pad_y": 0},
                    "detections": [{"class_name": "chair", "bbox": {"x1": 1, "y1": 2, "x2": 3, "y2": 4}}],
                },
                {
                    "image_path": "frames/frame_1.jpg",
                    "capture_ts_real_ns": 300,
                    "remap": {"scale": 1.0, "pad_x": 0, "pad_y": 0},
                    "detections": [{"class_name": "person", "bbox": {"x1": 5, "y1": 6, "x2": 7, "y2": 8}}],
                },
            ]
            (run_dir / "video" / "frames.jsonl").write_text(
                "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
            )

            frames = read_recorded_frames(run_dir)

            self.assertEqual([frame.image_path.name for frame in frames], ["frame_2.jpg", "frame_1.jpg"])
            self.assertEqual([frame.detections[0]["class_name"] for frame in frames], ["chair", "person"])

    def test_source_and_overlay_write_same_order_rate_and_target_colors(self):
        cv2 = _Cv2()
        frames = [
            _frame(
                "frame_1.jpg",
                1_000_000_000,
                [{"class_name": "person", "score": 0.9, "bbox": {"x1": 10, "y1": 20, "x2": 30, "y2": 40}}],
            ),
            _frame(
                "frame_2.jpg",
                1_050_000_000,
                [{"class_name": "chair", "score": 0.8, "bbox": {"x1": 50, "y1": 60, "x2": 70, "y2": 80}}],
            ),
        ]

        render_video_frames(
            Path("source.rendered.mp4"),
            Path("overlay.rendered.mp4"),
            frames,
            target_class="person",
            cv2_module=cv2,
        )

        source, overlay = cv2.writers
        self.assertEqual(source.fps, 20.0)
        self.assertEqual(
            (source.fps, source.size, len(source.frames)),
            (overlay.fps, overlay.size, len(overlay.frames)),
        )
        self.assertEqual([frame.name for frame in source.frames], ["frame_1", "frame_2"])
        self.assertEqual([frame.name for frame in overlay.frames], ["frame_1", "frame_2"])
        self.assertEqual([frame.rectangles[0][2] for frame in overlay.frames], [TARGET_COLOR, NON_TARGET_COLOR])
        self.assertEqual([frame.labels[0] for frame in overlay.frames], [TARGET_COLOR, NON_TARGET_COLOR])

    def test_target_class_is_read_from_recorded_configuration(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = Path(temp_dir) / "manifest.yaml"
            manifest_path.write_text("experiment:\n  parameters:\n    autonomy_target_class: chair\n", encoding="utf-8")
            self.assertEqual(_target_class_from_manifest(manifest_path), "chair")

    def test_default_frame_rate_for_single_frame(self):
        self.assertEqual(recorded_frame_rate([_frame("frame.jpg", 1)]), 30.0)


if __name__ == "__main__":
    unittest.main()
