#!/usr/bin/env python3
"""Launch RViz with a focused real-hardware debugging layout."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_bringup = FindPackageShare("bringup")
    rviz_config_path = PathJoinSubstitution([pkg_bringup, "config", "rviz_real_debug_config.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(declared_arguments + [rviz_node])
