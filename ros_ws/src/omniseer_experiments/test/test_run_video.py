import unittest
from pathlib import Path

from omniseer_experiments.run_video import TimedDetections, nearest_detections, remux_source_video


class RunVideoTests(unittest.TestCase):
    def test_remux_uses_ffmpeg_stream_copy(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))

        remux_source_video(Path("video/source.ts"), Path("video/source.mp4"), runner=runner)

        self.assertEqual(calls[0][0][0], ["ffmpeg", "-y", "-i", "video/source.ts", "-c", "copy", "video/source.mp4"])
        self.assertTrue(calls[0][1]["check"])

    def test_timestamp_matching_uses_only_current_or_prior_detection(self):
        records = [TimedDetections(100.0, ()), TimedDetections(101.0, ())]

        self.assertIsNone(nearest_detections(records, video_time_sec=3.0, max_age_sec=0.5))
        self.assertIsNone(nearest_detections(records, video_time_sec=0.75, max_age_sec=0.5))
        self.assertEqual(nearest_detections(records, video_time_sec=1.1, max_age_sec=0.5), records[1])


if __name__ == "__main__":
    unittest.main()
