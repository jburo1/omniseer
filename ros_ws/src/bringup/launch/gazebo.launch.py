#!/usr/bin/env python3
'''
launches gz server with optional gui

Usage examples:
   ros2 launch bringup gazebo.launch.py
   ros2 launch bringup gazebo.launch.py headless:=true world:=my_world.sdf
'''
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # ────────────────────────────────
    # launch arguments
    # ────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            name='world',
            default_value='simple_world.world',
            description='Name of target SDF world file'
        ),
        DeclareLaunchArgument(
            name='headless',
            default_value='false',
            choices=['true', 'false'],
            description='If true, run only gz server with headless rendering'
        )
    ]

    world_file      = LaunchConfiguration('world')
    headless        = LaunchConfiguration('headless')

    pkg_bringup = FindPackageShare('bringup')

    world_path      = PathJoinSubstitution([pkg_bringup, 'worlds', world_file])
    gz_config_path  = PathJoinSubstitution([pkg_bringup, 'config', 'gz_config.config'])

    # ────────────────────────────────
    # gz environment vars
    # ────────────────────────────────
    set_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=TextSubstitution(text=f'/opt/ros/kilted/lib:/opt/ros/kilted/opt/gz_sim_vendor/lib/gz-sim-9/plugins')
    )
    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([FindPackageShare('omniseer_description'), '..'])
    )

    # ────────────────────────────────
    # ros_gz_sim launcher
    # ────────────────────────────────
    ros_gz_launch =  PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),'launch','gz_sim.launch.py'])]
    )

    gz_ros_gui = IncludeLaunchDescription(
        ros_gz_launch,
        launch_arguments={
            'gz_args': [
                ' -r -v 4 ',
                world_path,
                # ' --gui-config ',
                # gz_config_path,
            ]
        }.items(),
        condition=UnlessCondition(headless)
    )

    gz_ros_headless = IncludeLaunchDescription(
        ros_gz_launch,
        launch_arguments={
            'gz_args': [
                '-s -r -v 4 ',
                world_path,
                ' --headless-rendering',
            ]
        }.items(),
        condition=IfCondition(headless)
    )

    ld = LaunchDescription( declared_arguments + [
        # ExecuteProcess(cmd=['env'], output='screen'),
        set_plugin_path,
        set_resource_path,
        gz_ros_gui,
        gz_ros_headless,
    ])

    return ld
