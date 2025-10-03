#!/usr/bin/env python3
"""
start robot_state_publisher, then spawn robot URDF into gz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    pkg_omniseer = FindPackageShare("omniseer_description")
    xacro_file = PathJoinSubstitution([pkg_omniseer, "urdf", "xacro", "omniseer.urdf.xacro"])
    robot_description_urdf = Command(["xacro ", xacro_file])

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_urdf, "use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="both",
    )

    robot_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "omniseer",
            "-param",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.25",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
        parameters=[{"robot_description": robot_description_urdf, "use_sim_time": use_sim_time}],
    )

    spawn_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp_node,
            on_start=[robot_spawn_node],
        )
    )

    return LaunchDescription([*declared_arguments, rsp_node, spawn_event_handler])
