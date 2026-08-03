#!/usr/bin/env python3
"""Top-level simulation bringup with shared common and sim IO layers."""

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_SIM_AUTONOMY_INPUTS_TIMEOUT_SEC = "60"
_SIM_AUTONOMY_REQUIRED_TOPICS = [
    "/yolo/detections:yolo_msgs/msg/DetectionArray",
    "/range:sensor_msgs/msg/Range",
    "/odometry/filtered:nav_msgs/msg/Odometry",
]


def _handle_required_process_exit(process_name: str, success_actions, failure_reason: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        return [
            LogInfo(msg=f"{process_name} failed with exit code {event.returncode}"),
            EmitEvent(event=Shutdown(reason=failure_reason)),
        ]

    return _on_exit


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("world", default_value="simple_world.world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
        DeclareLaunchArgument("sim_camera_width", default_value="1920"),
        DeclareLaunchArgument("sim_camera_height", default_value="1080"),
        DeclareLaunchArgument("sim_camera_update_rate", default_value="30"),
        DeclareLaunchArgument("ekf_params_file", default_value="ekf_fusion.yaml"),
        DeclareLaunchArgument("slam_tb_config_file", default_value="slam_toolbox_async_online.yaml"),
        DeclareLaunchArgument("nav2_params_file", default_value="nav2_params.yaml"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rf2o", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_yolo", default_value="false"),
        DeclareLaunchArgument("start_range_adapter", default_value="true"),
        DeclareLaunchArgument("start_autonomy", default_value="false"),
        DeclareLaunchArgument("autonomy_target_class", default_value="chair"),
        DeclareLaunchArgument("autonomy_bbox_area_min_ratio", default_value="0.08"),
        DeclareLaunchArgument("autonomy_bbox_area_max_ratio", default_value="0.35"),
        DeclareLaunchArgument("autonomy_forward_speed_m_s", default_value="0.05"),
        DeclareLaunchArgument("autonomy_reverse_speed_m_s", default_value="0.04"),
        DeclareLaunchArgument("autonomy_stable_framed_frames", default_value="10"),
        DeclareLaunchArgument("autonomy_proximity_stop_m", default_value="0.30"),
        DeclareLaunchArgument("autonomy_capture_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("autonomy_detection_stale_ms", default_value="500"),
        DeclareLaunchArgument("autonomy_target_lost_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("yolo_model", default_value="yolov8s-worldv2.pt"),
        DeclareLaunchArgument("yolo_device", default_value="cuda:0"),
        DeclareLaunchArgument("yolo_threshold", default_value="0.5"),
        DeclareLaunchArgument("yolo_image_reliability", default_value="2"),
        DeclareLaunchArgument("start_gateway", default_value="false"),
    ]

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")
    sim_camera_width = LaunchConfiguration("sim_camera_width")
    sim_camera_height = LaunchConfiguration("sim_camera_height")
    sim_camera_update_rate = LaunchConfiguration("sim_camera_update_rate")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    slam_tb_config_file = LaunchConfiguration("slam_tb_config_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_slam = LaunchConfiguration("start_slam")
    start_rf2o = LaunchConfiguration("start_rf2o")
    start_nav = LaunchConfiguration("start_nav")
    start_rviz = LaunchConfiguration("start_rviz")
    start_yolo = LaunchConfiguration("start_yolo")
    start_range_adapter = LaunchConfiguration("start_range_adapter")
    start_autonomy = LaunchConfiguration("start_autonomy")
    autonomy_target_class = LaunchConfiguration("autonomy_target_class")
    autonomy_bbox_area_min_ratio = LaunchConfiguration("autonomy_bbox_area_min_ratio")
    autonomy_bbox_area_max_ratio = LaunchConfiguration("autonomy_bbox_area_max_ratio")
    autonomy_forward_speed_m_s = LaunchConfiguration("autonomy_forward_speed_m_s")
    autonomy_reverse_speed_m_s = LaunchConfiguration("autonomy_reverse_speed_m_s")
    autonomy_stable_framed_frames = LaunchConfiguration("autonomy_stable_framed_frames")
    autonomy_proximity_stop_m = LaunchConfiguration("autonomy_proximity_stop_m")
    autonomy_capture_timeout_sec = LaunchConfiguration("autonomy_capture_timeout_sec")
    autonomy_detection_stale_ms = LaunchConfiguration("autonomy_detection_stale_ms")
    autonomy_target_lost_timeout_sec = LaunchConfiguration("autonomy_target_lost_timeout_sec")
    yolo_model = LaunchConfiguration("yolo_model")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_threshold = LaunchConfiguration("yolo_threshold")
    yolo_image_reliability = LaunchConfiguration("yolo_image_reliability")
    start_gateway = LaunchConfiguration("start_gateway")
    cleanup_script = PathJoinSubstitution([pkg_bringup, "scripts", "pre_launch_cleanup.sh"])

    sim_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "sim_io.launch.py"])]),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "headless": headless,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
            "start_range_adapter": start_range_adapter,
            "sim_camera_width": sim_camera_width,
            "sim_camera_height": sim_camera_height,
            "sim_camera_update_rate": sim_camera_update_rate,
        }.items(),
    )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
            "sim_camera_width": sim_camera_width,
            "sim_camera_height": sim_camera_height,
            "sim_camera_update_rate": sim_camera_update_rate,
            "ekf_params_file": ekf_params_file,
            "slam_tb_config_file": slam_tb_config_file,
            "nav2_params_file": nav2_params_file,
            "start_slam": start_slam,
            "start_rf2o": start_rf2o,
            "start_nav": start_nav,
            "start_yolo": start_yolo,
            "yolo_model": yolo_model,
            "yolo_device": yolo_device,
            "yolo_threshold": yolo_threshold,
            "yolo_image_reliability": yolo_image_reliability,
            "start_gateway": start_gateway,
            "gateway_preview_source_kind": "videotest",
            "gateway_preview_device": "/dev/video11",
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "rviz.launch.py"])]),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_rviz),
    )

    autonomy_node = Node(
        package="omniseer_autonomy",
        executable="target_centering_node",
        name="target_centering_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_class": autonomy_target_class,
                "run_dir": "",
                "image_width_px": sim_camera_width,
                "image_height_px": sim_camera_height,
                "detections_topic": "/yolo/detections",
                "odometry_topic": "/odometry/filtered",
                "range_topic": "/range",
                "command_topic": "/cmd_vel_autonomy",
                "capture_service": "/vision/capture_frame",
                "bbox_area_min_ratio": autonomy_bbox_area_min_ratio,
                "bbox_area_max_ratio": autonomy_bbox_area_max_ratio,
                "forward_speed_m_s": autonomy_forward_speed_m_s,
                "reverse_speed_m_s": autonomy_reverse_speed_m_s,
                "stable_framed_frames": autonomy_stable_framed_frames,
                "proximity_stop_m": autonomy_proximity_stop_m,
                "capture_timeout_sec": autonomy_capture_timeout_sec,
                "detection_stale_ms": autonomy_detection_stale_ms,
                "target_lost_timeout_sec": autonomy_target_lost_timeout_sec,
            }
        ],
        condition=IfCondition(start_autonomy),
    )

    cleanup = ExecuteProcess(
        name="pre_flight_cleanup",
        cmd=["bash", cleanup_script, "sim"],
        output="screen",
    )

    wait_autonomy_inputs = ExecuteProcess(
        name="wait_sim_autonomy_inputs",
        cmd=[
            "ros2",
            "run",
            "bringup",
            "wait_for_topics",
            "--timeout-sec",
            _SIM_AUTONOMY_INPUTS_TIMEOUT_SEC,
            *[item for topic_spec in _SIM_AUTONOMY_REQUIRED_TOPICS for item in ("--topic", topic_spec)],
        ],
        output="screen",
        condition=IfCondition(start_autonomy),
    )

    launch_autonomy_after_inputs = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_autonomy_inputs,
            on_exit=_handle_required_process_exit(
                process_name="wait_sim_autonomy_inputs",
                success_actions=[autonomy_node],
                failure_reason="Timed out waiting for sim autonomy input topics",
            ),
        ),
        condition=IfCondition(start_autonomy),
    )

    launch_group = GroupAction(
        actions=[sim_io_launch, common_launch, rviz_launch, wait_autonomy_inputs, launch_autonomy_after_inputs]
    )
    after_cleanup = RegisterEventHandler(OnProcessExit(target_action=cleanup, on_exit=[launch_group]))

    return LaunchDescription([*declared_arguments, cleanup, after_cleanup])
