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
    ]

    use_sim_time     = LaunchConfiguration("use_sim_time")

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                pkg_bringup, 'config', 'twist_mux.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],

    )

    return LaunchDescription(
        declared_arguments + [
            twist_mux_node,
        ]
    )
