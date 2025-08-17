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
        # DeclareLaunchArgument('use_debug', default_value='false',    description='Enable YOLO debug images')
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_tb_config_file      = LaunchConfiguration("slam_tb_config_file")
    # use_debug = LaunchConfiguration('use_debug')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                pkg_bringup, 'config', 'twist_mux.yaml'
            ]),
            {'use_sim_time' : use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],
    )
    
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
            [PathJoinSubstitution([pkg_bringup, 'launch', 'yolov11.launch.py'])]
        ),
    )
    
    yolo_group = GroupAction([
        SetParameter(name='use_sim_time', value=use_sim_time),
        yolo_include
    ])

    lifecycle_mgr = Node(
        package     = 'nav2_lifecycle_manager',
        executable  = 'lifecycle_manager',
        name        = 'lifecycle_manager',
        output      = 'screen',
        parameters  = [{
            'autostart':    True,
            'node_names':   ['slam_toolbox'],
            'bond_timeout': 0.0,
            'bond_heartbeat_period': 0.0,
            'use_sim_time' : False
        }]
    )
    
    return LaunchDescription(
        declared_arguments + [
            twist_mux_node,
            slam_toolbox_node,
            yolo_group,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=slam_toolbox_node,
                    on_start=[TimerAction(period=3.0, actions=[lifecycle_mgr])]
                )
            ),
        ]
    )
