import importlib.util
import unittest
from pathlib import Path

from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node


def _load_launch_module(launch_name: str):
    launch_path = Path(__file__).resolve().parents[1] / "launch" / launch_name
    spec = importlib.util.spec_from_file_location(launch_name.replace(".", "_"), launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _flatten_launch_value(value) -> str:
    if isinstance(value, (list, tuple)):
        return "".join(_flatten_launch_value(item) for item in value)
    if hasattr(value, "text"):
        return value.text
    return str(value)


class RealLaunchStructureTests(unittest.TestCase):
    def test_real_io_launch_gates_micro_ros_agent_on_teensy_preflight(self) -> None:
        module = _load_launch_module("real_io.launch.py")
        launch_description = module.generate_launch_description()

        top_level_node_cmds = [str(entity.cmd) for entity in launch_description.entities if isinstance(entity, Node)]
        self.assertFalse(
            any("micro_ros_agent" in cmd for cmd in top_level_node_cmds),
            "micro_ros_agent should only be launched after the Teensy preflight exits successfully",
        )

        teensy_preflight = next(
            (
                entity
                for entity in launch_description.entities
                if isinstance(entity, ExecuteProcess) and "wait_for_teensy.py" in str(entity.cmd)
            ),
            None,
        )
        self.assertIsNotNone(teensy_preflight, "expected a wait_for_teensy launch gate")

        matching_handlers = [
            entity
            for entity in launch_description.entities
            if isinstance(entity, RegisterEventHandler)
            and getattr(entity.event_handler, "_OnActionEventBase__action_matcher", None) is teensy_preflight
        ]
        self.assertTrue(matching_handlers, "expected a process-exit handler for the Teensy preflight gate")

    def test_real_launch_waits_for_live_boundary_messages(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        wait_action = next(
            (
                entity
                for entity in launch_description.entities
                if isinstance(entity, ExecuteProcess)
                and "ros2 topic echo --once" in _flatten_launch_value(entity.cmd)
            ),
            None,
        )
        self.assertIsNotNone(wait_action, "expected a boundary-topic wait process")

        cmd_text = _flatten_launch_value(wait_action.cmd)
        self.assertIn("ros2 topic echo --once", cmd_text)
        self.assertIn("/encoder_counts", cmd_text)
        self.assertNotIn("ros2 topic list", cmd_text)
