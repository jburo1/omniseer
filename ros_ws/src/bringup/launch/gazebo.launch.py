#!/usr/bin/env python3
"""
launches gz server with optional gui

Usage examples:
   ros2 launch bringup gazebo.launch.py
   ros2 launch bringup gazebo.launch.py headless:=true world:=my_world.sdf
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ────────────────────────────────
    # launch arguments
    # ────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(name="world", default_value="simple_world.world"),
        DeclareLaunchArgument(name="headless", default_value="false"),
    ]

    world_file = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    pkg_bringup = FindPackageShare("bringup")

    world_path = PathJoinSubstitution([pkg_bringup, "worlds", world_file])
    PathJoinSubstitution([pkg_bringup, "config", "gz_config.config"])

    # ────────────────────────────────
    # gz environment vars
    # ────────────────────────────────
    set_plugin_path = AppendEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=TextSubstitution(text="/opt/ros/kilted/lib:/opt/ros/kilted/opt/gz_sim_vendor/lib/gz-sim-9/plugins"),
    )

    set_res_old = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare("omniseer_gz_assets"), "models"]),
    )

    set_gz_assets = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare("omniseer_gz_assets"), "models/"]),
    )

    set_desc_res = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare("omniseer_description")]),
    )

    # ────────────────────────────────
    # ros_gz_sim launcher
    # ────────────────────────────────
    ros_gz_launch = PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
    )

    gz_ros_gui = IncludeLaunchDescription(
        ros_gz_launch,
        launch_arguments={"gz_args": [TextSubstitution(text="-r -v 1 "), world_path]}.items(),
        condition=UnlessCondition(headless),
    )

    gz_ros_headless = IncludeLaunchDescription(
        ros_gz_launch,
        launch_arguments={
            "gz_args": [
                TextSubstitution(text="-s -r -v 1 "),
                world_path,
                TextSubstitution(text=" --headless-rendering"),
            ]
        }.items(),
        condition=IfCondition(headless),
    )

    ld = LaunchDescription(
        declared_arguments
        + [
            set_plugin_path,
            set_gz_assets,
            set_desc_res,
            set_res_old,
            TimerAction(
                period=1.0,
                actions=[
                    # ExecuteProcess(cmd=['env'],output='screen'),
                    gz_ros_gui,
                    gz_ros_headless,
                ],
            ),
        ]
    )

    return ld
