import argparse
import importlib.util
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

    def test_parse_topic_spec_accepts_absolute_topic_and_message_type(self) -> None:
        spec = self._module.parse_topic_spec("/range:sensor_msgs/msg/Range")

        self.assertEqual(spec.topic, "/range")
        self.assertEqual(spec.type_name, "sensor_msgs/msg/Range")

    def test_parse_topic_spec_rejects_relative_topic(self) -> None:
        with self.assertRaises(argparse.ArgumentTypeError):
            self._module.parse_topic_spec("range:sensor_msgs/msg/Range")

    def test_parse_topic_spec_rejects_non_message_type(self) -> None:
        with self.assertRaises(argparse.ArgumentTypeError):
            self._module.parse_topic_spec("/range:sensor_msgs/srv/Range")

    def test_resolve_message_type_loads_ros_message_class(self) -> None:
        message_type = self._module.resolve_message_type("sensor_msgs/msg/Range")

        self.assertEqual(message_type.__name__, "Range")
