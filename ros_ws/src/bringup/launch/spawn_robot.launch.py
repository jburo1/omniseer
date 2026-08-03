#!/usr/bin/env python3
"""
spawn the robot into Gazebo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_ci_geometry", default_value="false"),
        DeclareLaunchArgument("sim_camera_width", default_value="1920"),
        DeclareLaunchArgument("sim_camera_height", default_value="1080"),
        DeclareLaunchArgument("sim_camera_update_rate", default_value="30"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    use_ci_geometry = LaunchConfiguration("use_ci_geometry")
    sim_camera_width = LaunchConfiguration("sim_camera_width")
    sim_camera_height = LaunchConfiguration("sim_camera_height")
    sim_camera_update_rate = LaunchConfiguration("sim_camera_update_rate")

    pkg_omniseer = FindPackageShare("omniseer_description")
    xacro_file = PathJoinSubstitution([pkg_omniseer, "urdf", "xacro", "omniseer.urdf.xacro"])
    robot_description_urdf = Command(
        [
            "xacro ",
            xacro_file,
            " use_ci_geometry:=",
            use_ci_geometry,
            " sim_camera_width:=",
            sim_camera_width,
            " sim_camera_height:=",
            sim_camera_height,
            " sim_camera_update_rate:=",
            sim_camera_update_rate,
        ]
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

    return LaunchDescription([*declared_arguments, robot_spawn_node])
