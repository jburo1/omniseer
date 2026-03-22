#!/usr/bin/env python3
"""Launch the shared robot description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")

    pkg_omniseer = FindPackageShare("omniseer_description")
    xacro_file = PathJoinSubstitution([pkg_omniseer, "urdf", "xacro", "omniseer.urdf.xacro"])
    robot_description_urdf = Command(["xacro ", xacro_file, " use_ci_geometry:=", use_ci_geometry])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"robot_description": robot_description_urdf, "use_sim_time": use_sim_time}],
    )

    return LaunchDescription([*declared_arguments, robot_state_publisher_node])
