#!/usr/bin/env python3
"""Top-level real bringup with shared common and real IO layers."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
        DeclareLaunchArgument("micro_ros_serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("micro_ros_baud", default_value="115200"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("lidar_serial_device", default_value="/dev/ttyUSB0"),
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
        DeclareLaunchArgument("start_gateway", default_value="false"),
        DeclareLaunchArgument("gateway_preview_source_kind", default_value="camera"),
        DeclareLaunchArgument("gateway_preview_device", default_value="/dev/video11"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    micro_ros_serial_device = LaunchConfiguration("micro_ros_serial_device")
    micro_ros_baud = LaunchConfiguration("micro_ros_baud")
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
    start_gateway = LaunchConfiguration("start_gateway")
    gateway_preview_source_kind = LaunchConfiguration("gateway_preview_source_kind")
    gateway_preview_device = LaunchConfiguration("gateway_preview_device")

    real_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "real_io.launch.py"])]),
        launch_arguments={
            "log_level": log_level,
            "start_micro_ros_agent": start_micro_ros_agent,
            "micro_ros_serial_device": micro_ros_serial_device,
            "micro_ros_baud": micro_ros_baud,
            "start_lidar": start_lidar,
            "lidar_serial_device": lidar_serial_device,
            "lidar_baudrate": lidar_baudrate,
            "lidar_frame_id": lidar_frame_id,
            "lidar_inverted": lidar_inverted,
            "lidar_angle_compensate": lidar_angle_compensate,
            "encoder_odometry_params_file": encoder_odometry_params_file,
        }.items(),
    )

    common_launch_after_wait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "ekf_params_file": ekf_params_file,
            "slam_tb_config_file": slam_tb_config_file,
            "nav2_params_file": nav2_params_file,
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
                "until ros2 topic list | grep -qx /imu "
                "&& ros2 topic list | grep -qx /scan "
                "&& ros2 topic list | grep -qx /mecanum_drive_controller/odometry; "
                "do sleep 0.2; done"
            ),
        ],
        name="wait_real_boundary_topics",
        condition=IfCondition(wait_for_boundary_topics),
    )

    launch_common_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_boundary_topics,
            on_exit=[GroupAction(actions=[common_launch_after_wait])],
        )
    )

    return LaunchDescription(
        [
            *declared_arguments,
            real_io_launch,
            wait_boundary_topics,
            launch_common_after_wait,
            common_launch_immediate,
        ]
    )
