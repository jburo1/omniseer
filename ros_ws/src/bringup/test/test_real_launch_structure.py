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
        self.assertIn("--model-family", recorder_text)
        self.assertIn("--model-variant", recorder_text)
        self.assertIn("--model-precision", recorder_text)
        self.assertIn("--model-backend", recorder_text)
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
        self.assertIn("--comparison-id", recorder_text)
        self.assertIn("--trial", recorder_text)
        self.assertIn("--workload-id", recorder_text)
        self.assertIn("--resolved-vision-config-path", recorder_text)
        self.assertIn("--system-interval-sec", recorder_text)

    def test_real_launch_includes_conditioned_rosbag_recorder(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_rosbag_recording", declared_names)

        rosbag_processes = [
            entity
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, ExecuteProcess) and "ros2bagrecord" in _flatten_launch_value(entity.cmd)
        ]
        self.assertEqual(len(rosbag_processes), 1)
        rosbag_process = rosbag_processes[0]
        command_text = _flatten_launch_value(rosbag_process.cmd)
        self.assertIn("--output", command_text)
        self.assertIn("experiment_out_dir", command_text)
        self.assertIn("rosbag", command_text)
        for topic in (
            "/yolo/detections",
            "/vision/perf",
            "/imu",
            "/encoder_counts",
            "/scan",
            "/mecanum_drive_controller/odometry",
            "/mecanum_drive_controller/reference",
            "/cmd_vel_autonomy",
            "/cmd_vel_keyboard",
            "/tf",
            "/tf_static",
        ):
            self.assertIn(topic, command_text)
        launch_source = (Path(__file__).resolve().parents[1] / "launch" / "real.launch.py").read_text(encoding="utf-8")
        self.assertIn('condition=IfCondition(config["start_rosbag_recording"])', launch_source)

    def test_real_launch_includes_optional_autonomy_node(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_autonomy", declared_names)
        self.assertIn("autonomy_target_class", declared_names)
        self.assertIn("autonomy_run_dir", declared_names)
        self.assertIn("autonomy_bbox_area_min_ratio", declared_names)
        self.assertIn("autonomy_approach_stop_area_ratio", declared_names)
        self.assertIn("autonomy_bbox_area_max_ratio", declared_names)
        self.assertIn("autonomy_forward_speed_m_s", declared_names)
        self.assertIn("autonomy_reverse_speed_m_s", declared_names)
        self.assertIn("autonomy_stable_framed_frames", declared_names)
        self.assertIn("autonomy_success_miss_tolerance_updates", declared_names)
        self.assertIn("autonomy_proximity_stop_m", declared_names)
        self.assertIn("autonomy_capture_timeout_sec", declared_names)
        self.assertIn("autonomy_target_lost_timeout_sec", declared_names)
        self.assertIn("autonomy_min_target_confidence", declared_names)
        self.assertIn("autonomy_max_target_center_jump_ratio", declared_names)

        autonomy_nodes = [
            entity
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, Node)
            and "omniseer_autonomy" in _flatten_launch_value(entity.node_package)
            and "target_centering_node" in _flatten_launch_value(entity.node_executable)
        ]
        self.assertTrue(autonomy_nodes, "expected optional omniseer_autonomy target_centering_node")

        launch_text = (Path(__file__).resolve().parents[1] / "launch" / "real.launch.py").read_text(encoding="utf-8")
        self.assertIn("autonomy_completion_shutdown = RegisterEventHandler", launch_text)
        self.assertIn('Shutdown(reason="target centering autonomy completed")', launch_text)
        self.assertIn('"target_class": config["autonomy_target_class"]', launch_text)
        self.assertIn('"run_dir": config["autonomy_run_dir"]', launch_text)
        self.assertIn('"bbox_area_min_ratio": config["autonomy_bbox_area_min_ratio"]', launch_text)
        self.assertIn('"approach_stop_area_ratio": config["autonomy_approach_stop_area_ratio"]', launch_text)
        self.assertIn('"bbox_area_max_ratio": config["autonomy_bbox_area_max_ratio"]', launch_text)
        self.assertIn('"forward_speed_m_s": config["autonomy_forward_speed_m_s"]', launch_text)
        self.assertIn('"reverse_speed_m_s": config["autonomy_reverse_speed_m_s"]', launch_text)
        self.assertIn('"stable_framed_frames": config["autonomy_stable_framed_frames"]', launch_text)
        self.assertIn(
            '"success_miss_tolerance_updates": config["autonomy_success_miss_tolerance_updates"]',
            launch_text,
        )
        self.assertIn('"proximity_stop_m": config["autonomy_proximity_stop_m"]', launch_text)
        self.assertIn('"capture_timeout_sec": config["autonomy_capture_timeout_sec"]', launch_text)
        self.assertIn('"target_lost_timeout_sec": config["autonomy_target_lost_timeout_sec"]', launch_text)
        self.assertIn('"min_target_confidence": config["autonomy_min_target_confidence"]', launch_text)
        self.assertIn(
            '"max_target_center_jump_ratio": config["autonomy_max_target_center_jump_ratio"]',
            launch_text,
        )

    def test_twist_mux_includes_autonomy_below_keyboard_above_nav(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "twist_mux.yaml"
        text = config_path.read_text(encoding="utf-8")

        self.assertIn("topic: /cmd_vel_nav\n        timeout: 0.50\n        priority: 1", text)
        self.assertIn("topic: /cmd_vel_autonomy\n        timeout: 0.50\n        priority: 2", text)
        self.assertIn("topic: /cmd_vel_keyboard\n        timeout: 0.50\n        priority: 3", text)
        self.assertIn("topic: /cmd_vel_emergency_stop", text)
        self.assertIn("priority: 255", text)

    def test_real_launch_forwards_pipeline_telemetry_path_to_vision(self) -> None:
        module = _load_launch_module("real.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("pipeline_telemetry_path", declared_names)
        self.assertIn("resolved_vision_config_path", declared_names)

        include_text = "".join(
            str(getattr(entity, "launch_arguments", ""))
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, IncludeLaunchDescription)
        )
        self.assertIn("pipeline_telemetry_path", include_text)
        self.assertIn("resolved_vision_config_path", include_text)
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
        self.assertIn("resolved_vision_config_path", declared_names)
        self.assertIn("evidence_dir", declared_names)
        self.assertIn("evidence_interval_sec", declared_names)
        self.assertIn("evidence_jpeg_quality", declared_names)
        self.assertIn("evidence_storage_budget_mb", declared_names)
        self.assertIn("evidence_min_free_mb", declared_names)

    def test_real_vision_launch_treats_vision_bridge_exit_as_fault(self) -> None:
        launch_path = Path(__file__).resolve().parents[1] / "launch" / "real_vision.launch.py"
        source = launch_path.read_text(encoding="utf-8")

        self.assertIn("RegisterEventHandler", source)
        self.assertIn("OnProcessExit", source)
        self.assertIn("target_action=vision_bridge_node", source)
        self.assertIn('Shutdown(reason=f"{process_name} exited")', source)

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

    def test_sim_launch_exposes_optional_yolo_provider(self) -> None:
        module = _load_launch_module("sim.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_yolo", declared_names)
        self.assertIn("yolo_model", declared_names)
        self.assertIn("yolo_device", declared_names)
        self.assertIn("yolo_threshold", declared_names)
        self.assertIn("yolo_image_reliability", declared_names)

        sim_source = (Path(__file__).resolve().parents[1] / "launch" / "sim.launch.py").read_text(encoding="utf-8")
        common_source = (Path(__file__).resolve().parents[1] / "launch" / "common.launch.py").read_text(
            encoding="utf-8"
        )
        perception_source = (Path(__file__).resolve().parents[1] / "launch" / "perception.launch.py").read_text(
            encoding="utf-8"
        )
        yolo_source = (Path(__file__).resolve().parents[1] / "launch" / "yolo-world.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"start_yolo": start_yolo', sim_source)
        self.assertIn('"start_yolo": start_yolo', common_source)
        self.assertIn('"image_reliability": yolo_image_reliability', perception_source)
        self.assertIn('"input_image_topic": "/front_camera/image"', perception_source)
        self.assertIn('"classes": LaunchConfiguration("classes", default="chair")', yolo_source)
        self.assertIn('"use_tracking": LaunchConfiguration("use_tracking", default="False")', yolo_source)

    def test_sim_launch_exposes_optional_range_adapter(self) -> None:
        module = _load_launch_module("sim.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_range_adapter", declared_names)

        sim_source = (Path(__file__).resolve().parents[1] / "launch" / "sim.launch.py").read_text(encoding="utf-8")
        self.assertIn('"start_range_adapter": start_range_adapter', sim_source)

    def test_sim_launch_does_not_include_visual_autonomy_node(self) -> None:
        module = _load_launch_module("sim.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertNotIn("start_autonomy", declared_names)
        self.assertNotIn("autonomy_target_class", declared_names)
        self.assertNotIn("autonomy_bbox_area_min_ratio", declared_names)
        self.assertNotIn("autonomy_approach_stop_area_ratio", declared_names)
        self.assertNotIn("autonomy_bbox_area_max_ratio", declared_names)
        self.assertNotIn("autonomy_forward_speed_m_s", declared_names)
        self.assertNotIn("autonomy_reverse_speed_m_s", declared_names)
        self.assertNotIn("autonomy_stable_framed_frames", declared_names)
        self.assertNotIn("autonomy_success_miss_tolerance_updates", declared_names)
        self.assertNotIn("autonomy_proximity_stop_m", declared_names)
        self.assertNotIn("autonomy_capture_timeout_sec", declared_names)
        self.assertNotIn("autonomy_detection_stale_ms", declared_names)
        self.assertNotIn("autonomy_target_lost_timeout_sec", declared_names)
        self.assertNotIn("autonomy_min_target_confidence", declared_names)
        self.assertNotIn("autonomy_max_target_center_jump_ratio", declared_names)
        self.assertIn("sim_camera_width", declared_names)
        self.assertIn("sim_camera_height", declared_names)
        self.assertIn("sim_camera_update_rate", declared_names)

        sim_source = (Path(__file__).resolve().parents[1] / "launch" / "sim.launch.py").read_text(encoding="utf-8")
        common_source = (Path(__file__).resolve().parents[1] / "launch" / "common.launch.py").read_text(
            encoding="utf-8"
        )
        sim_io_source = (Path(__file__).resolve().parents[1] / "launch" / "sim_io.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn('package="omniseer_autonomy"', sim_source)
        self.assertNotIn('executable="target_centering_node"', sim_source)
        self.assertNotIn('package="omniseer_experiments"', sim_source)
        self.assertNotIn("condition=IfCondition(start_autonomy)", sim_source)
        self.assertNotIn("_SIM_AUTONOMY_INPUTS_TIMEOUT_SEC", sim_source)
        self.assertNotIn("wait_sim_autonomy_inputs", sim_source)
        self.assertNotIn("wait_for_topics", sim_source)
        self.assertNotIn("/yolo/detections:yolo_msgs/msg/DetectionArray", sim_source)
        self.assertNotIn("/odometry/filtered:nav_msgs/msg/Odometry", sim_source)
        self.assertNotIn("cmd_vel_autonomy", sim_source)
        self.assertIn('"sim_camera_width": sim_camera_width', sim_source)
        self.assertIn('"sim_camera_height": sim_camera_height', sim_source)
        self.assertIn('"sim_camera_update_rate": sim_camera_update_rate', sim_source)
        self.assertIn('"sim_camera_width": sim_camera_width', common_source)
        self.assertIn('"sim_camera_height": sim_camera_height', common_source)
        self.assertIn('"sim_camera_update_rate": sim_camera_update_rate', common_source)
        self.assertIn('"sim_camera_width": sim_camera_width', sim_io_source)
        self.assertIn('"sim_camera_height": sim_camera_height', sim_io_source)
        self.assertIn('"sim_camera_update_rate": sim_camera_update_rate', sim_io_source)

    def test_sim_io_launch_includes_optional_range_adapter(self) -> None:
        module = _load_launch_module("sim_io.launch.py")
        launch_description = module.generate_launch_description()

        declared_names = {
            _flatten_launch_value(entity.name)
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_range_adapter", declared_names)

        range_adapter_nodes = [
            entity
            for entity in _walk_entities(launch_description.entities)
            if isinstance(entity, Node)
            and "robot_io_adapters" in _flatten_launch_value(entity.node_package)
            and "scan_to_range" in _flatten_launch_value(entity.node_executable)
        ]
        self.assertTrue(range_adapter_nodes, "expected optional robot_io_adapters scan_to_range node")

        sim_io_source = (Path(__file__).resolve().parents[1] / "launch" / "sim_io.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"scan_topic": "/sonar"', sim_io_source)
        self.assertIn('"range_topic": "/range"', sim_io_source)
        self.assertIn("condition=IfCondition(start_range_adapter)", sim_io_source)
