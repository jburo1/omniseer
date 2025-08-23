#!/usr/bin/env python3
'''
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument('slam_tb_config_file', default_value='slam_toolbox_async_online.yaml'),
    ]

    use_sim_time             = LaunchConfiguration('use_sim_time')
    slam_tb_config_file      = LaunchConfiguration("slam_tb_config_file")    
    
    slam_toolbox_node = Node(
        package   = 'slam_toolbox',
        executable= 'async_slam_toolbox_node',
        name      = 'slam_toolbox',
        output    = 'screen',
        parameters=[ParameterFile(
            PathJoinSubstitution([pkg_bringup, 'config', slam_tb_config_file]),
            allow_substs=True
            ),
            {'use_sim_time': use_sim_time},
        ]
    )
    
    yolo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'yolo-world.launch.py'])]
        ),
    )
    
    yolo_group = GroupAction([
        SetParameter(name='use_sim_time', value=use_sim_time),
        yolo_include
    ])
    
    
    return LaunchDescription(
        declared_arguments + [
            slam_toolbox_node,
            yolo_group
        ]
    )
