#!/usr/bin/env python3
"""Launch the shared ROS graph above the sim/real IO boundary."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
        DeclareLaunchArgument("ekf_params_file", default_value="ekf_fusion.yaml"),
        DeclareLaunchArgument("slam_tb_config_file", default_value="slam_toolbox_async_online.yaml"),
        DeclareLaunchArgument("nav2_params_file", default_value="nav2_params.yaml"),
        DeclareLaunchArgument("start_description", default_value="true"),
        DeclareLaunchArgument("start_perception", default_value="true"),
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_twist_mux", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rf2o", default_value="true"),
        DeclareLaunchArgument("start_nav", default_value="true"),
        DeclareLaunchArgument("start_gateway", default_value="false"),
        DeclareLaunchArgument("gateway_preview_source_kind", default_value="camera"),
        DeclareLaunchArgument("gateway_preview_device", default_value="/dev/video11"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    slam_tb_config_file = LaunchConfiguration("slam_tb_config_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_description = LaunchConfiguration("start_description")
    start_perception = LaunchConfiguration("start_perception")
    start_ekf = LaunchConfiguration("start_ekf")
    start_twist_mux = LaunchConfiguration("start_twist_mux")
    start_slam = LaunchConfiguration("start_slam")
    start_rf2o = LaunchConfiguration("start_rf2o")
    start_nav = LaunchConfiguration("start_nav")
    start_gateway = LaunchConfiguration("start_gateway")
    gateway_preview_source_kind = LaunchConfiguration("gateway_preview_source_kind")
    gateway_preview_device = LaunchConfiguration("gateway_preview_device")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, "launch", "description.launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
        }.items(),
        condition=IfCondition(start_description),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, "launch", "perception.launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "slam_tb_config_file": slam_tb_config_file,
            "start_yolo": "false",
            "start_slam": start_slam,
            "start_rf2o": start_rf2o,
            "start_scan_to_range": "false",
        }.items(),
        condition=IfCondition(start_perception),
    )

    ekf_params_path = PathJoinSubstitution([pkg_bringup, "config", ekf_params_file])
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[ParameterFile(ekf_params_path, allow_substs=True), {"use_sim_time": use_sim_time}],
        condition=IfCondition(start_ekf),
    )

    nav_launch_after_wait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "nav.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "nav2_params_file": nav2_params_file,
            "start_twist_mux": "false",
        }.items(),
        condition=IfCondition(start_nav),
    )

    wait_nav_inputs = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "until ros2 topic echo --once /odometry/filtered >/dev/null 2>&1 "
                "&& ros2 topic echo --once /map >/dev/null 2>&1; "
                "do sleep 0.2; done"
            ),
        ],
        name="wait_nav_inputs",
        condition=IfCondition(start_nav),
    )

    launch_nav_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_nav_inputs,
            on_exit=[GroupAction(actions=[nav_launch_after_wait])],
        )
    )

    twist_mux_node = Node(
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
        condition=IfCondition(start_twist_mux),
    )

    gateway_node = Node(
        package="robot_diag_control_cpp",
        executable="robot_diag_control_cpp_node",
        name="robot_diag_control_cpp",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "preview_source_kind": gateway_preview_source_kind,
                "preview_device": gateway_preview_device,
            }
        ],
        condition=IfCondition(start_gateway),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            description_launch,
            perception_launch,
            twist_mux_node,
            ekf_node,
            wait_nav_inputs,
            launch_nav_after_wait,
            gateway_node,
        ]
    )
