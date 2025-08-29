#!/usr/bin/env python3
'''
bridge gz topics to ros
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    pkg_bringup  = FindPackageShare('bringup')

    bridge_config_path = PathJoinSubstitution([
        pkg_bringup, 'config', 'bridge_config.yaml'
    ])

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file' : bridge_config_path,
                     'use_sim_time' : use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    image_bridge_node = Node(
        package    = "ros_gz_image",
        executable = "image_bridge",
        arguments  = ["/front_camera/image",
                      '--ros-args', '--log-level', log_level],
        parameters = [{'front_camera.image.compressed.jpeg_quality': 75,
                       'use_sim_time' : use_sim_time}],
        output = "screen",
    )

    return LaunchDescription(declared_arguments + [
        bridge_node,
        image_bridge_node
    ])
