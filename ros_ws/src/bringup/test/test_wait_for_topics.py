import importlib.util
import subprocess
import sys
import unittest
from pathlib import Path


def _load_wait_for_topics_module():
    module_path = Path(__file__).resolve().parents[1] / "bringup" / "wait_for_topics.py"
    spec = importlib.util.spec_from_file_location("bringup_wait_for_topics", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class WaitForTopicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._module = _load_wait_for_topics_module()

    def test_wait_retries_topic_discovery_until_a_message_arrives(self) -> None:
        outcomes = iter(
            [
                subprocess.CompletedProcess(
                    [], 1, stderr="Could not determine the type for the passed topic\n"
                ),
                subprocess.CompletedProcess([], 0, stderr=""),
            ]
        )
        now = iter([0.0, 0.0, 0.1, 0.1])

        result = self._module.wait_for_topic(
            "/imu",
            5.0,
            runner=lambda *args, **kwargs: next(outcomes),
            monotonic=lambda: next(now),
            sleep=lambda _duration: None,
        )

        self.assertTrue(result.ready, result.reason)
        self.assertEqual(result.reason, "received a message on /imu")

    def test_wait_reports_the_last_ros_failure_after_deadline(self) -> None:
        now = iter([0.0, 0.0, 1.0])
        failure = subprocess.CompletedProcess(
            [], 1, stderr="Could not determine the type for the passed topic\n"
        )

        result = self._module.wait_for_topic(
            "/imu",
            1.0,
            runner=lambda *args, **kwargs: failure,
            monotonic=lambda: next(now),
            sleep=lambda _duration: None,
        )

        self.assertFalse(result.ready)
        self.assertIn("Timed out after 1.0s", result.reason)
        self.assertIn("Could not determine the type", result.reason)

    def test_main_stops_at_the_first_failed_topic(self) -> None:
        original_wait = self._module.wait_for_topic
        self.addCleanup(setattr, self._module, "wait_for_topic", original_wait)
        seen_topics = []

        def fail_first(topic, timeout_sec):
            seen_topics.append((topic, timeout_sec))
            return self._module.ProbeResult(False, "missing /imu")

        self._module.wait_for_topic = fail_first

        exit_code = self._module.main(["--timeout-sec", "2", "/imu", "/scan"])

        self.assertEqual(exit_code, 1)
        self.assertEqual(seen_topics, [("/imu", 2.0)])
