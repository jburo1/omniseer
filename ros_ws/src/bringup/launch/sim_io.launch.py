#!/usr/bin/env python3
"""Launch simulation-only producers and adapters below the IO boundary."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("world", default_value="simple_world.world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
        DeclareLaunchArgument("start_range_adapter", default_value="true"),
    ]

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")
    start_range_adapter = LaunchConfiguration("start_range_adapter")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "gazebo.launch.py"])]),
        launch_arguments={"world": world, "headless": headless}.items(),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "bridge.launch.py"])]),
        launch_arguments={"use_sim_time": use_sim_time, "log_level": log_level}.items(),
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "spawn_robot.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "use_ci_geometry": use_ci_geometry,
        }.items(),
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "controllers.launch.py"])]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "sim_mode": "true",
            "log_level": log_level,
        }.items(),
    )

    range_adapter_node = Node(
        package="robot_io_adapters",
        executable="scan_to_range",
        name="sim_sonar_to_range",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "scan_topic": "/sonar",
                "range_topic": "/range",
            }
        ],
        condition=IfCondition(start_range_adapter),
    )

    wait_clock = ExecuteProcess(
        cmd=["bash", "-lc", "until ros2 topic list | grep -qx /clock; do sleep 0.2; done"],
        name="wait_clock",
    )

    on_clock = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_clock,
            on_exit=[spawn_robot_launch, controllers_launch],
        )
    )

    return LaunchDescription(
        [
            *declared_arguments,
            gazebo_launch,
            bridge_launch,
            range_adapter_node,
            wait_clock,
            on_clock,
        ]
    )
