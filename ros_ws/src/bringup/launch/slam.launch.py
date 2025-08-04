#!/usr/bin/env python3
'''
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')

    declared_arguments = [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('config_file', default_value='slam_toolbox_async_online.yaml'),
    ]

    use_sim_time     = LaunchConfiguration("use_sim_time")
    config_file      = LaunchConfiguration("config_file")

    slam_toolbox_node = Node(
        package   = 'slam_toolbox',
        executable= 'async_slam_toolbox_node',
        name      = 'slam_toolbox',
        output    = 'screen',
        parameters= [
            PathJoinSubstitution([
                pkg_bringup, 'config', config_file
            ]),
            {'use_sim_time': use_sim_time}],
        # remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )



    return LaunchDescription(
        declared_arguments + [
            slam_toolbox_node,
        ]
    )
