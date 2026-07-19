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


def _handle_required_process_exit(process_name: str, success_actions, failure_reason: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        return [
            LogInfo(msg=f"{process_name} failed with exit code {event.returncode}"),
            EmitEvent(event=Shutdown(reason=failure_reason)),
        ]

    return _on_exit


def _build_real_bringup_actions(
    *,
    pkg_bringup,
    use_sim_time,
    log_level,
    start_micro_ros_agent,
    micro_ros_serial_device,
    micro_ros_baud,
    require_teensy,
    teensy_preflight_timeout_sec,
    allow_teensy_power_cycle,
    start_lidar,
    lidar_serial_device,
    lidar_baudrate,
    lidar_frame_id,
    lidar_inverted,
    lidar_angle_compensate,
    encoder_odometry_params_file,
    ekf_params_file,
    slam_tb_config_file,
    nav2_params_file,
    start_slam,
    start_rf2o,
    start_nav,
    wait_for_boundary_topics,
    boundary_topics_timeout_sec,
    start_gateway,
    gateway_preview_source_kind,
    gateway_preview_device,
    start_vision,
    vision_params_file,
    camera_device,
    camera_width,
    camera_height,
    camera_buffer_count,
    pipeline_dst_width,
    pipeline_dst_height,
    detector_model_path,
    clip_model_path,
    clip_vocab_path,
    classes_path,
    classes_pad_token,
    producer_preflight_capture_wait_ms,
    runner_warmup_runs,
    postprocess_score_threshold,
    postprocess_nms_iou_threshold,
    postprocess_max_detections,
    camera_frame_id,
    start_experiment_recording,
    experiment_run_id,
    experiment_out_dir,
    experiment_classes,
    experiment_notes,
    experiment_duration_sec,
    experiment_overwrite,
    experiment_queue_size,
    experiment_flush_interval_sec,
):
    real_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "real_io.launch.py"])]),
        launch_arguments={
            "log_level": log_level,
            "start_micro_ros_agent": start_micro_ros_agent,
            "micro_ros_serial_device": micro_ros_serial_device,
            "micro_ros_baud": micro_ros_baud,
            "require_teensy": require_teensy,
            "teensy_preflight_timeout_sec": teensy_preflight_timeout_sec,
            "allow_teensy_power_cycle": allow_teensy_power_cycle,
            "start_lidar": start_lidar,
            "lidar_serial_device": lidar_serial_device,
            "lidar_baudrate": lidar_baudrate,
            "lidar_frame_id": lidar_frame_id,
            "lidar_inverted": lidar_inverted,
            "lidar_angle_compensate": lidar_angle_compensate,
            "encoder_odometry_params_file": encoder_odometry_params_file,
        }.items(),
    )

    real_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "real_vision.launch.py"])]),
        launch_arguments={
            "log_level": log_level,
            "vision_params_file": vision_params_file,
            "camera_device": camera_device,
            "camera_width": camera_width,
            "camera_height": camera_height,
            "camera_buffer_count": camera_buffer_count,
            "pipeline_dst_width": pipeline_dst_width,
            "pipeline_dst_height": pipeline_dst_height,
            "detector_model_path": detector_model_path,
            "clip_model_path": clip_model_path,
            "clip_vocab_path": clip_vocab_path,
            "classes_path": classes_path,
            "classes_pad_token": classes_pad_token,
            "producer_preflight_capture_wait_ms": producer_preflight_capture_wait_ms,
            "runner_warmup_runs": runner_warmup_runs,
            "postprocess_score_threshold": postprocess_score_threshold,
            "postprocess_nms_iou_threshold": postprocess_nms_iou_threshold,
            "postprocess_max_detections": postprocess_max_detections,
            "camera_frame_id": camera_frame_id,
        }.items(),
        condition=IfCondition(start_vision),
    )

    experiment_recorder_node = Node(
        package="omniseer_experiments",
        executable="record_run",
        name="perception_run_recorder",
        output="screen",
        arguments=[
            "--run-id",
            experiment_run_id,
            "--out",
            experiment_out_dir,
            "--classes",
            experiment_classes,
            "--vision-params-file",
            PathJoinSubstitution([pkg_bringup, "config", vision_params_file]),
            "--detector-model-path",
            detector_model_path,
            "--clip-model-path",
            clip_model_path,
            "--clip-vocab-path",
            clip_vocab_path,
            "--classes-path",
            classes_path,
            "--notes",
            experiment_notes,
            "--duration-sec",
            experiment_duration_sec,
            "--overwrite",
            experiment_overwrite,
            "--queue-size",
            experiment_queue_size,
            "--flush-interval-sec",
            experiment_flush_interval_sec,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(start_experiment_recording),
    )

    # Keep the teleop command path available even if lidar/nav boundary topics
    # are still missing.
    baseline_twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            PathJoinSubstitution([pkg_bringup, "config", "twist_mux.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", "/mecanum_drive_controller/reference")],
        condition=IfCondition(wait_for_boundary_topics),
    )

    common_launch_after_wait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "ekf_params_file": ekf_params_file,
            "slam_tb_config_file": slam_tb_config_file,
            "nav2_params_file": nav2_params_file,
            "start_description": "true",
            "start_perception": "true",
            "start_ekf": "true",
            "start_twist_mux": "false",
            "start_slam": start_slam,
            "start_rf2o": start_rf2o,
            "start_nav": start_nav,
            "start_gateway": start_gateway,
            "gateway_preview_source_kind": gateway_preview_source_kind,
            "gateway_preview_device": gateway_preview_device,
        }.items(),
        condition=IfCondition(wait_for_boundary_topics),
    )

    common_launch_immediate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "ekf_params_file": ekf_params_file,
            "slam_tb_config_file": slam_tb_config_file,
            "nav2_params_file": nav2_params_file,
            "start_description": "true",
            "start_perception": "true",
            "start_ekf": "true",
            "start_twist_mux": "true",
            "start_slam": start_slam,
            "start_rf2o": start_rf2o,
            "start_nav": start_nav,
            "start_gateway": start_gateway,
            "gateway_preview_source_kind": gateway_preview_source_kind,
            "gateway_preview_device": gateway_preview_device,
        }.items(),
        condition=UnlessCondition(wait_for_boundary_topics),
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
            boundary_topics_timeout_sec,
        ],
        name="wait_real_boundary_topics",
        condition=IfCondition(wait_for_boundary_topics),
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
        baseline_twist_mux_node,
        wait_boundary_topics,
        launch_common_after_wait,
        common_launch_immediate,
    ]


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("pre_launch_cleanup", default_value="true"),
        DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
        DeclareLaunchArgument(
            "micro_ros_serial_device",
            default_value="/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00",
        ),
        DeclareLaunchArgument("micro_ros_baud", default_value="115200"),
        DeclareLaunchArgument("require_teensy", default_value="true"),
        DeclareLaunchArgument("teensy_preflight_timeout_sec", default_value="20"),
        DeclareLaunchArgument("allow_teensy_power_cycle", default_value="false"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "lidar_serial_device",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
        ),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("lidar_frame_id", default_value="lidar_frame"),
        DeclareLaunchArgument("lidar_inverted", default_value="false"),
        DeclareLaunchArgument("lidar_angle_compensate", default_value="true"),
        DeclareLaunchArgument("encoder_odometry_params_file", default_value="encoder_odometry.yaml"),
        DeclareLaunchArgument("ekf_params_file", default_value="ekf_fusion_real.yaml"),
        DeclareLaunchArgument("slam_tb_config_file", default_value="slam_toolbox_async_online.yaml"),
        DeclareLaunchArgument("nav2_params_file", default_value="nav2_params.yaml"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rf2o", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("wait_for_boundary_topics", default_value="true"),
        DeclareLaunchArgument("boundary_topics_timeout_sec", default_value="20"),
        DeclareLaunchArgument("start_gateway", default_value="false"),
        DeclareLaunchArgument("gateway_preview_source_kind", default_value="camera"),
        DeclareLaunchArgument("gateway_preview_device", default_value="/dev/video11"),
        DeclareLaunchArgument("start_vision", default_value="true"),
        DeclareLaunchArgument("vision_params_file", default_value="vision_bridge.real.yaml"),
        DeclareLaunchArgument("camera_device", default_value="__from_config__"),
        DeclareLaunchArgument("camera_width", default_value="__from_config__"),
        DeclareLaunchArgument("camera_height", default_value="__from_config__"),
        DeclareLaunchArgument("camera_buffer_count", default_value="__from_config__"),
        DeclareLaunchArgument("pipeline_dst_width", default_value="__from_config__"),
        DeclareLaunchArgument("pipeline_dst_height", default_value="__from_config__"),
        DeclareLaunchArgument("detector_model_path", default_value="__from_config__"),
        DeclareLaunchArgument("clip_model_path", default_value="__from_config__"),
        DeclareLaunchArgument("clip_vocab_path", default_value="__from_config__"),
        DeclareLaunchArgument("classes_path", default_value="__from_config__"),
        DeclareLaunchArgument("classes_pad_token", default_value="__from_config__"),
        DeclareLaunchArgument("producer_preflight_capture_wait_ms", default_value="__from_config__"),
        DeclareLaunchArgument("runner_warmup_runs", default_value="__from_config__"),
        DeclareLaunchArgument("postprocess_score_threshold", default_value="__from_config__"),
        DeclareLaunchArgument("postprocess_nms_iou_threshold", default_value="__from_config__"),
        DeclareLaunchArgument("postprocess_max_detections", default_value="__from_config__"),
        DeclareLaunchArgument("camera_frame_id", default_value="__from_config__"),
        DeclareLaunchArgument("start_experiment_recording", default_value="false"),
        DeclareLaunchArgument("experiment_run_id", default_value=""),
        DeclareLaunchArgument("experiment_out_dir", default_value=""),
        DeclareLaunchArgument("experiment_classes", default_value=""),
        DeclareLaunchArgument("experiment_notes", default_value=""),
        DeclareLaunchArgument("experiment_duration_sec", default_value="0"),
        DeclareLaunchArgument("experiment_overwrite", default_value="false"),
        DeclareLaunchArgument("experiment_queue_size", default_value="256"),
        DeclareLaunchArgument("experiment_flush_interval_sec", default_value="1.0"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    pre_launch_cleanup = LaunchConfiguration("pre_launch_cleanup")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    micro_ros_serial_device = LaunchConfiguration("micro_ros_serial_device")
    micro_ros_baud = LaunchConfiguration("micro_ros_baud")
    require_teensy = LaunchConfiguration("require_teensy")
    teensy_preflight_timeout_sec = LaunchConfiguration("teensy_preflight_timeout_sec")
    allow_teensy_power_cycle = LaunchConfiguration("allow_teensy_power_cycle")
    start_lidar = LaunchConfiguration("start_lidar")
    lidar_serial_device = LaunchConfiguration("lidar_serial_device")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    encoder_odometry_params_file = LaunchConfiguration("encoder_odometry_params_file")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    slam_tb_config_file = LaunchConfiguration("slam_tb_config_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_slam = LaunchConfiguration("start_slam")
    start_rf2o = LaunchConfiguration("start_rf2o")
    start_nav = LaunchConfiguration("start_nav")
    wait_for_boundary_topics = LaunchConfiguration("wait_for_boundary_topics")
    boundary_topics_timeout_sec = LaunchConfiguration("boundary_topics_timeout_sec")
    start_gateway = LaunchConfiguration("start_gateway")
    gateway_preview_source_kind = LaunchConfiguration("gateway_preview_source_kind")
    gateway_preview_device = LaunchConfiguration("gateway_preview_device")
    start_vision = LaunchConfiguration("start_vision")
    vision_params_file = LaunchConfiguration("vision_params_file")
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_buffer_count = LaunchConfiguration("camera_buffer_count")
    pipeline_dst_width = LaunchConfiguration("pipeline_dst_width")
    pipeline_dst_height = LaunchConfiguration("pipeline_dst_height")
    detector_model_path = LaunchConfiguration("detector_model_path")
    clip_model_path = LaunchConfiguration("clip_model_path")
    clip_vocab_path = LaunchConfiguration("clip_vocab_path")
    classes_path = LaunchConfiguration("classes_path")
    classes_pad_token = LaunchConfiguration("classes_pad_token")
    producer_preflight_capture_wait_ms = LaunchConfiguration("producer_preflight_capture_wait_ms")
    runner_warmup_runs = LaunchConfiguration("runner_warmup_runs")
    postprocess_score_threshold = LaunchConfiguration("postprocess_score_threshold")
    postprocess_nms_iou_threshold = LaunchConfiguration("postprocess_nms_iou_threshold")
    postprocess_max_detections = LaunchConfiguration("postprocess_max_detections")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    start_experiment_recording = LaunchConfiguration("start_experiment_recording")
    experiment_run_id = LaunchConfiguration("experiment_run_id")
    experiment_out_dir = LaunchConfiguration("experiment_out_dir")
    experiment_classes = LaunchConfiguration("experiment_classes")
    experiment_notes = LaunchConfiguration("experiment_notes")
    experiment_duration_sec = LaunchConfiguration("experiment_duration_sec")
    experiment_overwrite = LaunchConfiguration("experiment_overwrite")
    experiment_queue_size = LaunchConfiguration("experiment_queue_size")
    experiment_flush_interval_sec = LaunchConfiguration("experiment_flush_interval_sec")
    cleanup_script = PathJoinSubstitution([pkg_bringup, "scripts", "pre_launch_cleanup.sh"])

    launch_group = GroupAction(
        actions=_build_real_bringup_actions(
            pkg_bringup=pkg_bringup,
            use_sim_time=use_sim_time,
            log_level=log_level,
            start_micro_ros_agent=start_micro_ros_agent,
            micro_ros_serial_device=micro_ros_serial_device,
            micro_ros_baud=micro_ros_baud,
            require_teensy=require_teensy,
            teensy_preflight_timeout_sec=teensy_preflight_timeout_sec,
            allow_teensy_power_cycle=allow_teensy_power_cycle,
            start_lidar=start_lidar,
            lidar_serial_device=lidar_serial_device,
            lidar_baudrate=lidar_baudrate,
            lidar_frame_id=lidar_frame_id,
            lidar_inverted=lidar_inverted,
            lidar_angle_compensate=lidar_angle_compensate,
            encoder_odometry_params_file=encoder_odometry_params_file,
            ekf_params_file=ekf_params_file,
            slam_tb_config_file=slam_tb_config_file,
            nav2_params_file=nav2_params_file,
            start_slam=start_slam,
            start_rf2o=start_rf2o,
            start_nav=start_nav,
            wait_for_boundary_topics=wait_for_boundary_topics,
            boundary_topics_timeout_sec=boundary_topics_timeout_sec,
            start_gateway=start_gateway,
            gateway_preview_source_kind=gateway_preview_source_kind,
            gateway_preview_device=gateway_preview_device,
            start_vision=start_vision,
            vision_params_file=vision_params_file,
            camera_device=camera_device,
            camera_width=camera_width,
            camera_height=camera_height,
            camera_buffer_count=camera_buffer_count,
            pipeline_dst_width=pipeline_dst_width,
            pipeline_dst_height=pipeline_dst_height,
            detector_model_path=detector_model_path,
            clip_model_path=clip_model_path,
            clip_vocab_path=clip_vocab_path,
            classes_path=classes_path,
            classes_pad_token=classes_pad_token,
            producer_preflight_capture_wait_ms=producer_preflight_capture_wait_ms,
            runner_warmup_runs=runner_warmup_runs,
            postprocess_score_threshold=postprocess_score_threshold,
            postprocess_nms_iou_threshold=postprocess_nms_iou_threshold,
            postprocess_max_detections=postprocess_max_detections,
            camera_frame_id=camera_frame_id,
            start_experiment_recording=start_experiment_recording,
            experiment_run_id=experiment_run_id,
            experiment_out_dir=experiment_out_dir,
            experiment_classes=experiment_classes,
            experiment_notes=experiment_notes,
            experiment_duration_sec=experiment_duration_sec,
            experiment_overwrite=experiment_overwrite,
            experiment_queue_size=experiment_queue_size,
            experiment_flush_interval_sec=experiment_flush_interval_sec,
        )
    )

    cleanup = ExecuteProcess(
        name="pre_launch_cleanup",
        cmd=["bash", cleanup_script, "real"],
        output="screen",
        condition=IfCondition(pre_launch_cleanup),
    )

    after_cleanup = RegisterEventHandler(OnProcessExit(target_action=cleanup, on_exit=[launch_group]))

    launch_without_cleanup = GroupAction(
        actions=_build_real_bringup_actions(
            pkg_bringup=pkg_bringup,
            use_sim_time=use_sim_time,
            log_level=log_level,
            start_micro_ros_agent=start_micro_ros_agent,
            micro_ros_serial_device=micro_ros_serial_device,
            micro_ros_baud=micro_ros_baud,
            require_teensy=require_teensy,
            teensy_preflight_timeout_sec=teensy_preflight_timeout_sec,
            allow_teensy_power_cycle=allow_teensy_power_cycle,
            start_lidar=start_lidar,
            lidar_serial_device=lidar_serial_device,
            lidar_baudrate=lidar_baudrate,
            lidar_frame_id=lidar_frame_id,
            lidar_inverted=lidar_inverted,
            lidar_angle_compensate=lidar_angle_compensate,
            encoder_odometry_params_file=encoder_odometry_params_file,
            ekf_params_file=ekf_params_file,
            slam_tb_config_file=slam_tb_config_file,
            nav2_params_file=nav2_params_file,
            start_slam=start_slam,
            start_rf2o=start_rf2o,
            start_nav=start_nav,
            wait_for_boundary_topics=wait_for_boundary_topics,
            boundary_topics_timeout_sec=boundary_topics_timeout_sec,
            start_gateway=start_gateway,
            gateway_preview_source_kind=gateway_preview_source_kind,
            gateway_preview_device=gateway_preview_device,
            start_vision=start_vision,
            vision_params_file=vision_params_file,
            camera_device=camera_device,
            camera_width=camera_width,
            camera_height=camera_height,
            camera_buffer_count=camera_buffer_count,
            pipeline_dst_width=pipeline_dst_width,
            pipeline_dst_height=pipeline_dst_height,
            detector_model_path=detector_model_path,
            clip_model_path=clip_model_path,
            clip_vocab_path=clip_vocab_path,
            classes_path=classes_path,
            classes_pad_token=classes_pad_token,
            producer_preflight_capture_wait_ms=producer_preflight_capture_wait_ms,
            runner_warmup_runs=runner_warmup_runs,
            postprocess_score_threshold=postprocess_score_threshold,
            postprocess_nms_iou_threshold=postprocess_nms_iou_threshold,
            postprocess_max_detections=postprocess_max_detections,
            camera_frame_id=camera_frame_id,
            start_experiment_recording=start_experiment_recording,
            experiment_run_id=experiment_run_id,
            experiment_out_dir=experiment_out_dir,
            experiment_classes=experiment_classes,
            experiment_notes=experiment_notes,
            experiment_duration_sec=experiment_duration_sec,
            experiment_overwrite=experiment_overwrite,
            experiment_queue_size=experiment_queue_size,
            experiment_flush_interval_sec=experiment_flush_interval_sec,
        ),
        condition=UnlessCondition(pre_launch_cleanup),
    )

    return LaunchDescription([*declared_arguments, cleanup, after_cleanup, launch_without_cleanup])
