#!/usr/bin/env python3
"""Top-level real bringup with shared common and real IO layers."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_REAL_ARGUMENT_DEFAULTS = [
    ("use_sim_time", "false"),
    ("log_level", "info"),
    ("pre_launch_cleanup", "true"),
    ("start_micro_ros_agent", "true"),
    ("micro_ros_serial_device", "/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00"),
    ("micro_ros_baud", "115200"),
    ("require_teensy", "true"),
    ("teensy_preflight_timeout_sec", "20"),
    ("allow_teensy_power_cycle", "false"),
    ("start_lidar", "true"),
    (
        "lidar_serial_device",
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    ),
    ("lidar_baudrate", "115200"),
    ("lidar_frame_id", "lidar_frame"),
    ("lidar_inverted", "false"),
    ("lidar_angle_compensate", "true"),
    ("encoder_odometry_params_file", "encoder_odometry.yaml"),
    ("ekf_params_file", "ekf_fusion_real.yaml"),
    ("slam_tb_config_file", "slam_toolbox_async_online.yaml"),
    ("nav2_params_file", "nav2_params.yaml"),
    ("start_slam", "true"),
    ("start_rf2o", "true"),
    ("start_nav", "true"),
    ("wait_for_boundary_topics", "true"),
    ("boundary_topics_timeout_sec", "20"),
    ("start_gateway", "false"),
    ("gateway_preview_source_kind", "camera"),
    ("gateway_preview_device", "/dev/video11"),
    ("gateway_preview_record_path", ""),
    ("start_autonomy", "false"),
    ("autonomy_target_class", ""),
    ("autonomy_run_dir", ""),
    ("autonomy_bbox_area_min_ratio", "0.08"),
    ("autonomy_approach_stop_area_ratio", "0.10"),
    ("autonomy_bbox_area_max_ratio", "0.35"),
    ("autonomy_forward_speed_m_s", "0.05"),
    ("autonomy_reverse_speed_m_s", "0.04"),
    ("autonomy_stable_framed_frames", "10"),
    ("autonomy_success_miss_tolerance_updates", "2"),
    ("autonomy_proximity_stop_m", "0.30"),
    ("autonomy_capture_timeout_sec", "2.0"),
    ("autonomy_target_lost_timeout_sec", "0.5"),
    ("autonomy_min_target_confidence", "0.50"),
    ("autonomy_max_target_center_jump_ratio", "0.20"),
    ("start_vision", "true"),
    ("vision_params_file", "vision_bridge.real.yaml"),
    ("camera_device", "__from_config__"),
    ("camera_width", "__from_config__"),
    ("camera_height", "__from_config__"),
    ("camera_buffer_count", "__from_config__"),
    ("pipeline_dst_width", "__from_config__"),
    ("pipeline_dst_height", "__from_config__"),
    ("detector_model_path", "__from_config__"),
    ("clip_model_path", "__from_config__"),
    ("clip_vocab_path", "__from_config__"),
    ("classes_path", "__from_config__"),
    ("classes_pad_token", "__from_config__"),
    ("producer_preflight_capture_wait_ms", "__from_config__"),
    ("runner_warmup_runs", "__from_config__"),
    ("postprocess_score_threshold", "__from_config__"),
    ("postprocess_nms_iou_threshold", "__from_config__"),
    ("postprocess_max_detections", "__from_config__"),
    ("camera_frame_id", "__from_config__"),
    ("pipeline_telemetry_path", ""),
    ("evidence_dir", ""),
    ("video_dir", ""),
    ("evidence_interval_sec", "1.0"),
    ("evidence_jpeg_quality", "85"),
    ("evidence_storage_budget_mb", "1024"),
    ("evidence_min_free_mb", "256"),
    ("start_experiment_recording", "false"),
    ("start_rosbag_recording", "false"),
    ("experiment_run_id", ""),
    ("experiment_out_dir", ""),
    ("experiment_classes", ""),
    ("experiment_notes", ""),
    ("experiment_container_image_ref", ""),
    ("experiment_container_image_digest", ""),
    ("experiment_launch_command", ""),
    ("experiment_launch_profile", ""),
    ("experiment_launch_mode", ""),
    ("experiment_launch_args", ""),
    ("experiment_config", ""),
    ("experiment_parameters", ""),
    ("experiment_system_interval_sec", "1.0"),
    ("experiment_overwrite", "false"),
    ("experiment_queue_size", "256"),
    ("experiment_flush_interval_sec", "1.0"),
]

_REAL_IO_ARGS = [
    "log_level",
    "start_micro_ros_agent",
    "micro_ros_serial_device",
    "micro_ros_baud",
    "require_teensy",
    "teensy_preflight_timeout_sec",
    "allow_teensy_power_cycle",
    "start_lidar",
    "lidar_serial_device",
    "lidar_baudrate",
    "lidar_frame_id",
    "lidar_inverted",
    "lidar_angle_compensate",
    "encoder_odometry_params_file",
]

_REAL_VISION_ARGS = [
    "log_level",
    "vision_params_file",
    "camera_device",
    "camera_width",
    "camera_height",
    "camera_buffer_count",
    "pipeline_dst_width",
    "pipeline_dst_height",
    "detector_model_path",
    "clip_model_path",
    "clip_vocab_path",
    "classes_path",
    "classes_pad_token",
    "producer_preflight_capture_wait_ms",
    "runner_warmup_runs",
    "postprocess_score_threshold",
    "postprocess_nms_iou_threshold",
    "postprocess_max_detections",
    "camera_frame_id",
    "pipeline_telemetry_path",
    "evidence_dir",
    "video_dir",
    "evidence_interval_sec",
    "evidence_jpeg_quality",
    "evidence_storage_budget_mb",
    "evidence_min_free_mb",
]

_COMMON_FORWARD_ARGS = [
    "use_sim_time",
    "log_level",
    "ekf_params_file",
    "slam_tb_config_file",
    "nav2_params_file",
    "start_slam",
    "start_rf2o",
    "start_nav",
    "start_gateway",
    "gateway_preview_source_kind",
    "gateway_preview_device",
    "gateway_preview_record_path",
]


def _declare_real_arguments():
    return [DeclareLaunchArgument(name, default_value=default) for name, default in _REAL_ARGUMENT_DEFAULTS]


def _launch_configurations():
    return {name: LaunchConfiguration(name) for name, _default in _REAL_ARGUMENT_DEFAULTS}


def _selected(config, names):
    return {name: config[name] for name in names}


def _handle_required_process_exit(process_name: str, success_actions, failure_reason: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        return [
            LogInfo(msg=f"{process_name} failed with exit code {event.returncode}"),
            EmitEvent(event=Shutdown(reason=failure_reason)),
        ]

    return _on_exit


def _build_real_bringup_actions(*, pkg_bringup, config):
    real_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "real_io.launch.py"])]),
        launch_arguments=_selected(config, _REAL_IO_ARGS).items(),
    )

    real_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "real_vision.launch.py"])]),
        launch_arguments=_selected(config, _REAL_VISION_ARGS).items(),
        condition=IfCondition(config["start_vision"]),
    )

    experiment_recorder_node = Node(
        package="omniseer_experiments",
        executable="record_run",
        name="perception_run_recorder",
        output="screen",
        arguments=[
            "--run-id",
            config["experiment_run_id"],
            "--out",
            config["experiment_out_dir"],
            "--classes",
            config["experiment_classes"],
            "--vision-params-file",
            PathJoinSubstitution([pkg_bringup, "config", config["vision_params_file"]]),
            "--detector-model-path",
            config["detector_model_path"],
            "--clip-model-path",
            config["clip_model_path"],
            "--clip-vocab-path",
            config["clip_vocab_path"],
            "--classes-path",
            config["classes_path"],
            "--notes",
            config["experiment_notes"],
            "--container-image-ref",
            config["experiment_container_image_ref"],
            "--container-image-digest",
            config["experiment_container_image_digest"],
            "--launch-command",
            config["experiment_launch_command"],
            "--launch-profile",
            config["experiment_launch_profile"],
            "--launch-mode",
            config["experiment_launch_mode"],
            "--launch-args",
            config["experiment_launch_args"],
            "--experiment-config",
            config["experiment_config"],
            "--experiment-parameters",
            config["experiment_parameters"],
            "--system-interval-sec",
            config["experiment_system_interval_sec"],
            "--overwrite",
            config["experiment_overwrite"],
            "--queue-size",
            config["experiment_queue_size"],
            "--flush-interval-sec",
            config["experiment_flush_interval_sec"],
            "--ros-args",
            "--log-level",
            config["log_level"],
        ],
        condition=IfCondition(config["start_experiment_recording"]),
    )

    rosbag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            PathJoinSubstitution([config["experiment_out_dir"], "rosbag"]),
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
        ],
        name="record_rosbag",
        output="screen",
        condition=IfCondition(config["start_rosbag_recording"]),
    )

    autonomy_node = Node(
        package="omniseer_autonomy",
        executable="target_centering_node",
        name="target_centering_node",
        output="screen",
        arguments=["--ros-args", "--log-level", config["log_level"]],
        parameters=[
            {
                "use_sim_time": config["use_sim_time"],
                "target_class": config["autonomy_target_class"],
                "run_dir": config["autonomy_run_dir"],
                "bbox_area_min_ratio": config["autonomy_bbox_area_min_ratio"],
                "approach_stop_area_ratio": config["autonomy_approach_stop_area_ratio"],
                "bbox_area_max_ratio": config["autonomy_bbox_area_max_ratio"],
                "forward_speed_m_s": config["autonomy_forward_speed_m_s"],
                "reverse_speed_m_s": config["autonomy_reverse_speed_m_s"],
                "stable_framed_frames": config["autonomy_stable_framed_frames"],
                "success_miss_tolerance_updates": config["autonomy_success_miss_tolerance_updates"],
                "proximity_stop_m": config["autonomy_proximity_stop_m"],
                "capture_timeout_sec": config["autonomy_capture_timeout_sec"],
                "target_lost_timeout_sec": config["autonomy_target_lost_timeout_sec"],
                "min_target_confidence": config["autonomy_min_target_confidence"],
                "max_target_center_jump_ratio": config["autonomy_max_target_center_jump_ratio"],
            }
        ],
        condition=IfCondition(config["start_autonomy"]),
    )

    autonomy_completion_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=autonomy_node,
            on_exit=[
                LogInfo(msg="target centering autonomy completed; shutting down real launch"),
                EmitEvent(event=Shutdown(reason="target centering autonomy completed")),
            ],
        ),
        condition=IfCondition(config["start_autonomy"]),
    )

    # Keep the teleop command path available even if lidar/nav boundary topics
    # are still missing.
    baseline_twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        arguments=["--ros-args", "--log-level", config["log_level"]],
        parameters=[
            PathJoinSubstitution([pkg_bringup, "config", "twist_mux.yaml"]),
            {"use_sim_time": config["use_sim_time"]},
        ],
        remappings=[("/cmd_vel_out", "/mecanum_drive_controller/reference")],
        condition=IfCondition(config["wait_for_boundary_topics"]),
    )

    common_forward_args = _selected(config, _COMMON_FORWARD_ARGS)
    common_launch_after_wait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            **common_forward_args,
            "start_description": "true",
            "start_perception": "true",
            "start_ekf": "true",
            "start_twist_mux": "false",
        }.items(),
        condition=IfCondition(config["wait_for_boundary_topics"]),
    )

    common_launch_immediate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            **common_forward_args,
            "start_description": "true",
            "start_perception": "true",
            "start_ekf": "true",
            "start_twist_mux": "true",
        }.items(),
        condition=UnlessCondition(config["wait_for_boundary_topics"]),
    )

    wait_boundary_topics = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "set -euo pipefail\n"
                'timeout_sec="$1"\n'
                "check_topic() {\n"
                '  local topic="$1"\n'
                '  if ! timeout "${timeout_sec}" ros2 topic echo --once "${topic}" >/dev/null 2>&1; then\n'
                '    echo "Timed out waiting for first message on ${topic}" >&2\n'
                "    exit 1\n"
                "  fi\n"
                "}\n"
                "check_topic /imu\n"
                "check_topic /encoder_counts\n"
                "check_topic /scan\n"
                "check_topic /mecanum_drive_controller/odometry\n"
            ),
            "bash",
            config["boundary_topics_timeout_sec"],
        ],
        name="wait_real_boundary_topics",
        condition=IfCondition(config["wait_for_boundary_topics"]),
    )

    launch_common_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_boundary_topics,
            on_exit=_handle_required_process_exit(
                process_name="wait_real_boundary_topics",
                success_actions=[GroupAction(actions=[common_launch_after_wait])],
                failure_reason="Timed out waiting for real boundary topics",
            ),
        )
    )

    return [
        real_io_launch,
        real_vision_launch,
        experiment_recorder_node,
        rosbag_recorder,
        autonomy_node,
        autonomy_completion_shutdown,
        baseline_twist_mux_node,
        wait_boundary_topics,
        launch_common_after_wait,
        common_launch_immediate,
    ]


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")
    declared_arguments = _declare_real_arguments()
    config = _launch_configurations()
    pre_launch_cleanup = config["pre_launch_cleanup"]
    cleanup_script = PathJoinSubstitution([pkg_bringup, "scripts", "pre_launch_cleanup.sh"])

    launch_group = GroupAction(actions=_build_real_bringup_actions(pkg_bringup=pkg_bringup, config=config))

    cleanup = ExecuteProcess(
        name="pre_launch_cleanup",
        cmd=["bash", cleanup_script, "real"],
        output="screen",
        condition=IfCondition(pre_launch_cleanup),
    )

    after_cleanup = RegisterEventHandler(OnProcessExit(target_action=cleanup, on_exit=[launch_group]))

    launch_without_cleanup = GroupAction(
        actions=_build_real_bringup_actions(pkg_bringup=pkg_bringup, config=config),
        condition=UnlessCondition(pre_launch_cleanup),
    )

    return LaunchDescription([*declared_arguments, cleanup, after_cleanup, launch_without_cleanup])
