#!/usr/bin/env python3
"""
launch rviz
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_bringup = FindPackageShare('bringup')

    rviz_config_path = PathJoinSubstitution([
        pkg_bringup, 'config', 'rviz_config.rviz'
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments = [
            '-d', rviz_config_path
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    path_recorder_node = Node(
        package='analysis',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments +[
        rviz_node,
        path_recorder_node,
    ])
