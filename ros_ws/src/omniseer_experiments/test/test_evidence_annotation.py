import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - dependency availability is environment-specific.
    cv2 = None
    np = None

from omniseer_experiments.evidence_annotation import annotate_evidence, annotate_evidence_main


def _sample_record(*, detections: list[dict] | None = None) -> dict:
    return {
        "schema_version": 1,
        "artifact_type": "sampled_frame",
        "image_path": "evidence/frames/frame_1.jpg",
        "jpeg_quality": 85,
        "capture_reason": "periodic",
        "frame_id": 1,
        "sequence": 10,
        "capture_ts_real_ns": 1000,
        "model_input": {"width": 64, "height": 64},
        "source_image": {"width": 128, "height": 108},
        "remap": {"scale": 0.5, "pad_x": 0, "pad_y": 5, "resized_w": 64, "resized_h": 54},
        "detections": [] if detections is None else detections,
    }


def _detection() -> dict:
    return {
        "class_id": 0,
        "class_name": "person",
        "score": 0.91,
        "bbox": {"x1": 20.0, "y1": 20.0, "x2": 80.0, "y2": 80.0},
    }


@unittest.skipIf(cv2 is None or np is None, "OpenCV and NumPy are required for evidence annotation tests")
class EvidenceAnnotationTests(unittest.TestCase):
    def test_writes_annotated_jpeg_from_evidence_record(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_frame(run_dir)
            _write_evidence_jsonl(run_dir, [_sample_record(detections=[_detection()])])

            summary = annotate_evidence(run_dir)

            output_path = run_dir / "evidence" / "annotated" / "frame_1.jpg"
            self.assertEqual(summary.issues, ())
            self.assertEqual(summary.frames_written, 1)
            self.assertEqual(summary.detections_drawn, 1)
            self.assertTrue(output_path.is_file())
            annotated = cv2.imread(str(output_path), cv2.IMREAD_COLOR)
            self.assertIsNotNone(annotated)
            self.assertGreater(int(annotated.sum()), 0)

    def test_empty_detection_frame_is_still_written(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_frame(run_dir)
            _write_evidence_jsonl(run_dir, [_sample_record()])

            summary = annotate_evidence(run_dir)

            self.assertEqual(summary.issues, ())
            self.assertEqual(summary.frames_written, 1)
            self.assertEqual(summary.detections_drawn, 0)
            self.assertEqual(summary.empty_frames_written, 1)
            self.assertTrue((run_dir / "evidence" / "annotated" / "frame_1.jpg").is_file())

    def test_existing_annotation_requires_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_frame(run_dir)
            _write_evidence_jsonl(run_dir, [_sample_record(detections=[_detection()])])

            first_summary = annotate_evidence(run_dir)
            second_summary = annotate_evidence(run_dir)
            overwrite_summary = annotate_evidence(run_dir, overwrite=True)

            self.assertEqual(first_summary.issues, ())
            self.assertEqual({issue.code for issue in second_summary.issues}, {"annotated_image_exists"})
            self.assertEqual(overwrite_summary.issues, ())

    def test_missing_evidence_image_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            _write_evidence_jsonl(run_dir, [_sample_record(detections=[_detection()])])

            summary = annotate_evidence(run_dir)

            self.assertEqual(summary.frames_written, 0)
            self.assertEqual({issue.code for issue in summary.issues}, {"missing_evidence_image"})

    def test_malformed_evidence_jsonl_exits_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "demo_001"
            (run_dir / "evidence").mkdir(parents=True)
            (run_dir / "evidence" / "evidence.jsonl").write_text("{bad json\n", encoding="utf-8")
            stream = io.StringIO()

            with self.assertRaises(SystemExit) as raised, contextlib.redirect_stdout(stream):
                annotate_evidence_main([str(run_dir)])

            self.assertEqual(raised.exception.code, 1)
            self.assertIn("malformed_evidence_jsonl", stream.getvalue())


def _write_frame(run_dir: Path) -> None:
    frames_dir = run_dir / "evidence" / "frames"
    frames_dir.mkdir(parents=True)
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    self_path = frames_dir / "frame_1.jpg"
    assert cv2.imwrite(str(self_path), image)


def _write_evidence_jsonl(run_dir: Path, records: list[dict]) -> None:
    evidence_dir = run_dir / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    payload = "".join(json.dumps(record, sort_keys=True) + "\n" for record in records)
    (evidence_dir / "evidence.jsonl").write_text(payload, encoding="utf-8")
