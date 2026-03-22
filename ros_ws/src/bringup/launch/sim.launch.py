#!/usr/bin/env python3
"""Top-level simulation bringup with shared common and sim IO layers."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("world", default_value="simple_world.world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
        DeclareLaunchArgument("ekf_params_file", default_value="ekf_fusion.yaml"),
        DeclareLaunchArgument("slam_tb_config_file", default_value="slam_toolbox_async_online.yaml"),
        DeclareLaunchArgument("nav2_params_file", default_value="nav2_params.yaml"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rf2o", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_gateway", default_value="false"),
    ]

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    slam_tb_config_file = LaunchConfiguration("slam_tb_config_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_slam = LaunchConfiguration("start_slam")
    start_rf2o = LaunchConfiguration("start_rf2o")
    start_nav = LaunchConfiguration("start_nav")
    start_rviz = LaunchConfiguration("start_rviz")
    start_gateway = LaunchConfiguration("start_gateway")

    sim_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "sim_io.launch.py"])]),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "headless": headless,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
        }.items(),
    )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "common.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
            "ekf_params_file": ekf_params_file,
            "slam_tb_config_file": slam_tb_config_file,
            "nav2_params_file": nav2_params_file,
            "start_slam": start_slam,
            "start_rf2o": start_rf2o,
            "start_nav": start_nav,
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

    cleanup = ExecuteProcess(
        name="pre_flight_cleanup",
        cmd=[
            "bash",
            "-c",
            r"""
            echo "[cleanup] Gazebo…"
            pkill -TERM -f 'gz[ _](sim|server|client|gui)' || true
            echo "[cleanup] ROS2 Daemon…"
            ros2 daemon stop  || true
            ros2 daemon start || true
            """,
        ],
        output="screen",
    )

    launch_group = GroupAction(actions=[sim_io_launch, common_launch, rviz_launch])
    after_cleanup = RegisterEventHandler(
        OnProcessExit(target_action=cleanup, on_exit=[launch_group])
    )

    return LaunchDescription([*declared_arguments, cleanup, after_cleanup])
