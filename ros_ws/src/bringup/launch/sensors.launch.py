#!/usr/bin/env python3
'''
launch sensor related nodes
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_bringup  = FindPackageShare('bringup')

    robot_localization_config   = PathJoinSubstitution([pkg_bringup, 'config', 'ekf_fusion.yaml'])

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[{'use_sim_time' : use_sim_time}, robot_localization_config]
    )

    scan_to_range_node = Node(
        package='analysis',
        executable='scan_to_range',
        name='scan_to_range',
        parameters=[{'use_sim_time' : use_sim_time}],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        robot_localization_node,
        scan_to_range_node,
    ])
