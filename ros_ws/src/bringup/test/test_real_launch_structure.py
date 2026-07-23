import importlib.util
import unittest
from pathlib import Path

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
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


def _walk_entities(entities):
    for entity in entities:
        yield entity
        nested_entities = None
        if hasattr(entity, "get_sub_entities"):
            nested_entities = entity.get_sub_entities()
        elif hasattr(entity, "entities"):
            nested_entities = entity.entities

        if nested_entities:
            yield from _walk_entities(nested_entities)


class RealLaunchStructureTests(unittest.TestCase):
    def test_real_launch_runs_pre_launch_cleanup_before_bringup(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        cleanup_action = next(
            (
                entity
                for entity in _walk_entities(launch_description.entities)
                if isinstance(entity, ExecuteProcess) and "pre_launch_cleanup.sh" in _flatten_launch_value(entity.cmd)
            ),
            None,
        )
        self.assertIsNotNone(cleanup_action, "expected a pre-launch cleanup process in real.launch.py")
        self.assertIn("real", _flatten_launch_value(cleanup_action.cmd))

        matching_handlers = [
            entity
            for entity in launch_description.entities
            if isinstance(entity, RegisterEventHandler)
            and getattr(entity.event_handler, "_OnActionEventBase__action_matcher", None) is cleanup_action
        ]
        self.assertTrue(matching_handlers, "expected real bringup to wait for pre-launch cleanup")

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
                for entity in _walk_entities(launch_description.entities)
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
                for entity in _walk_entities(launch_description.entities)
                if isinstance(entity, ExecuteProcess) and "ros2 topic echo --once" in _flatten_launch_value(entity.cmd)
            ),
            None,
        )
        self.assertIsNotNone(wait_action, "expected a boundary-topic wait process")

        cmd_text = _flatten_launch_value(wait_action.cmd)
        self.assertIn("ros2 topic echo --once", cmd_text)
        self.assertIn("/encoder_counts", cmd_text)
        self.assertNotIn("ros2 topic list", cmd_text)

    def test_real_launch_includes_optional_experiment_recorder(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        recorder_nodes = [
            entity
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, Node)
            and "omniseer_experiments" in _flatten_launch_value(entity.node_package)
            and "record_run" in _flatten_launch_value(entity.node_executable)
        ]

        self.assertTrue(recorder_nodes, "expected optional omniseer_experiments record_run node")
        recorder_text = "".join(_flatten_launch_value(getattr(node, "_Node__arguments", [])) for node in recorder_nodes)
        self.assertIn("--run-id", recorder_text)
        self.assertIn("--vision-params-file", recorder_text)
        self.assertIn("--detector-model-path", recorder_text)
        self.assertIn("--classes-path", recorder_text)
        self.assertIn("--container-image-ref", recorder_text)
        self.assertIn("--container-image-digest", recorder_text)
        self.assertIn("--launch-command", recorder_text)
        self.assertIn("--launch-profile", recorder_text)
        self.assertIn("--launch-mode", recorder_text)
        self.assertIn("--launch-args", recorder_text)
        self.assertIn("--experiment-config", recorder_text)
        self.assertIn("--experiment-parameters", recorder_text)
        self.assertIn("--duration-sec", recorder_text)

    def test_real_launch_forwards_pipeline_telemetry_path_to_vision(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("pipeline_telemetry_path", declared_names)

        include_text = "".join(
            str(getattr(entity, "launch_arguments", ""))
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, IncludeLaunchDescription)
        )
        self.assertIn("pipeline_telemetry_path", include_text)
        self.assertIn("evidence_dir", include_text)
        self.assertIn("evidence_interval_sec", include_text)
        self.assertIn("evidence_jpeg_quality", include_text)
        self.assertIn("evidence_storage_budget_mb", include_text)
        self.assertIn("evidence_min_free_mb", include_text)

    def test_real_vision_launch_exposes_pipeline_telemetry_path(self) -> None:
        module = _load_launch_module("real_vision.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("pipeline_telemetry_path", declared_names)
        self.assertIn("evidence_dir", declared_names)
        self.assertIn("evidence_interval_sec", declared_names)
        self.assertIn("evidence_jpeg_quality", declared_names)
        self.assertIn("evidence_storage_budget_mb", declared_names)
        self.assertIn("evidence_min_free_mb", declared_names)

    def test_sim_launch_runs_shared_cleanup_before_launch_group(self) -> None:
        module = _load_launch_module("sim.launch.py")
        launch_description = module.generate_launch_description()

        cleanup_action = next(
            (
                entity
                for entity in _walk_entities(launch_description.entities)
                if isinstance(entity, ExecuteProcess) and "pre_launch_cleanup.sh" in _flatten_launch_value(entity.cmd)
            ),
            None,
        )
        self.assertIsNotNone(cleanup_action, "expected a shared cleanup process in sim.launch.py")
        self.assertIn("sim", _flatten_launch_value(cleanup_action.cmd))

        matching_handlers = [
            entity
            for entity in launch_description.entities
            if isinstance(entity, RegisterEventHandler)
            and getattr(entity.event_handler, "_OnActionEventBase__action_matcher", None) is cleanup_action
        ]
        self.assertTrue(matching_handlers, "expected sim launch to wait for cleanup before bringup")
