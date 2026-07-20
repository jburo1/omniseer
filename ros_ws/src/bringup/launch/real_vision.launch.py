#!/usr/bin/env python3
"""Launch the native real-hardware vision bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

USE_CONFIG_SENTINEL = "__from_config__"


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("vision_params_file", default_value="vision_bridge.real.yaml"),
        DeclareLaunchArgument("camera_device", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("camera_width", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("camera_height", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("camera_buffer_count", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("pipeline_dst_width", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("pipeline_dst_height", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("detector_model_path", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("clip_model_path", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("clip_vocab_path", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("classes_path", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("classes_pad_token", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("producer_preflight_capture_wait_ms", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("runner_warmup_runs", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("postprocess_score_threshold", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("postprocess_nms_iou_threshold", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("postprocess_max_detections", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("camera_frame_id", default_value=USE_CONFIG_SENTINEL),
        DeclareLaunchArgument("pipeline_telemetry_path", default_value=""),
        DeclareLaunchArgument("evidence_dir", default_value=""),
        DeclareLaunchArgument("evidence_interval_sec", default_value="1.0"),
        DeclareLaunchArgument("evidence_jpeg_quality", default_value="85"),
        DeclareLaunchArgument("evidence_storage_budget_mb", default_value="1024"),
        DeclareLaunchArgument("evidence_min_free_mb", default_value="256"),
    ]

    def launch_setup(context):
        vision_params_path = PathJoinSubstitution([pkg_bringup, "config", LaunchConfiguration("vision_params_file")])
        override_specs = [
            ("camera.device", "camera_device", str),
            ("camera.width", "camera_width", int),
            ("camera.height", "camera_height", int),
            ("camera.buffer_count", "camera_buffer_count", int),
            ("pipeline.dst_width", "pipeline_dst_width", int),
            ("pipeline.dst_height", "pipeline_dst_height", int),
            ("models.detector_model_path", "detector_model_path", str),
            ("models.clip_model_path", "clip_model_path", str),
            ("models.clip_vocab_path", "clip_vocab_path", str),
            ("classes.path", "classes_path", str),
            ("classes.pad_token", "classes_pad_token", str),
            ("producer.preflight_capture_wait_ms", "producer_preflight_capture_wait_ms", int),
            ("runner.warmup_runs", "runner_warmup_runs", int),
            ("postprocess.score_threshold", "postprocess_score_threshold", float),
            ("postprocess.nms_iou_threshold", "postprocess_nms_iou_threshold", float),
            ("postprocess.max_detections", "postprocess_max_detections", int),
            ("frames.camera_frame_id", "camera_frame_id", str),
            ("telemetry.pipeline_jsonl_path", "pipeline_telemetry_path", str),
            ("evidence.dir", "evidence_dir", str),
            ("evidence.interval_sec", "evidence_interval_sec", float),
            ("evidence.jpeg_quality", "evidence_jpeg_quality", int),
            ("evidence.storage_budget_mb", "evidence_storage_budget_mb", int),
            ("evidence.min_free_mb", "evidence_min_free_mb", int),
        ]

        overrides = {}
        for param_name, argument_name, convert in override_specs:
            raw_value = LaunchConfiguration(argument_name).perform(context)
            if raw_value == USE_CONFIG_SENTINEL:
                continue
            overrides[param_name] = convert(raw_value)

        vision_bridge_node = Node(
            package="omniseer_vision_bridge",
            executable="vision_bridge_node",
            name="vision_bridge",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[ParameterFile(vision_params_path, allow_substs=True), overrides],
        )

        return [vision_bridge_node]

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])
