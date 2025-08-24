#!/usr/bin/env python3
'''
launch broadcasters and controllers
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')
    pkg_omniseer = FindPackageShare('omniseer_description')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('sim_mode', default_value='true'),
        DeclareLaunchArgument('controller_yaml', default_value=PathJoinSubstitution([pkg_bringup, 'config', 'controllers.yaml']))
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file   = PathJoinSubstitution([pkg_omniseer, 'urdf', 'xacro', 'omniseer.urdf.xacro'])
    controller_file = PathJoinSubstitution([pkg_bringup, 'config', 'controllers.yaml'])
    robot_description_content = Command(['xacro ', xacro_file])

    sim_mode         = LaunchConfiguration('sim_mode')
    controller_yaml  = LaunchConfiguration("controller_yaml")
    controller_yaml_param_file = ParameterFile(controller_yaml, allow_substs=True)
    
    # only spawn controller for real bot, gz plugin spawns for sim in urdf
    control_node = Node(
        package   = 'controller_manager',
        executable= 'ros2_control_node',
        name='controller_manager',
        parameters= [
            {'robot_description' : robot_description_content,
             'use_sim_time': use_sim_time},
            controller_yaml_param_file],
        output    = 'screen',
        condition = UnlessCondition(sim_mode)
    )
    
    jsb_spawner = TimerAction(
        period=2.0,
        actions= [Node(
            package    = 'controller_manager',
            executable = 'spawner',
            name = 'jsb',
            parameters = [{'use_sim_time' : use_sim_time}],
            arguments  = ['joint_state_broadcaster',
                        '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60.0'],
            output     = 'screen',
        )]
    )

    mecanum_drive_spawner = TimerAction(
        period=3.0,
        actions = [Node(
            package   = 'controller_manager',
            executable= 'spawner',
            parameters = [{'use_sim_time' : use_sim_time}],
            arguments = ['mecanum_drive_controller',
                        '--param-file', controller_file,
                        '--controller-manager-timeout', '60.0'],
            output    = 'screen',
        )]
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            jsb_spawner,
            mecanum_drive_spawner
            # jsb_spawner_node,
            # mecanum_drive_spawner_node
        ]
    )
