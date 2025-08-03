#!/usr/bin/env python3
'''
launch broadcasters and controllers
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition

def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')
    pkg_omniseer = FindPackageShare('omniseer_description')

    declared_arguments = [
            DeclareLaunchArgument('sim_mode', default_value='true',
                description='sim or real'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('controller_yaml',
                default_value=PathJoinSubstitution([pkg_bringup, 'config', 'controllers.yaml'])),
    ]

    xacro_file   = PathJoinSubstitution([pkg_omniseer, 'urdf', 'xacro', 'omniseer.urdf.xacro'])
    controller_file = PathJoinSubstitution([pkg_bringup, 'config', 'controllers.yaml'])
    robot_description_content = Command(['xacro ', xacro_file])

    sim_mode         = LaunchConfiguration('sim_mode')
    use_sim_time     = LaunchConfiguration("use_sim_time")
    controller_yaml  = LaunchConfiguration("controller_yaml")


    # only spawn controller for real bot, gz plugin spawns for sim in urdf
    control_node = Node(
        package   = 'controller_manager',
        executable= 'ros2_control_node',
        name='controller_manager',
        parameters= [
            {'robot_description' : robot_description_content},
            controller_yaml,
            {"use_sim_time" : use_sim_time}
        ],
        output    = 'both',
        condition = UnlessCondition(sim_mode)
    )


    # time gate these spawners making sure that the controller is ready
    jsb_spawner = TimerAction(
        period=2.0,
        actions= [Node(
            package   = 'controller_manager',
            executable= 'spawner',
            name = 'jsb',
            arguments = ['joint_state_broadcaster',
                        '--controller-manager', '/controller_manager'],
            output    = 'screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )]
    )

    mecanum_drive_spawner = TimerAction(
        period=2.0,
        actions = [Node(
            package   = 'controller_manager',
            executable= 'spawner',
            arguments = ['mecanum_drive_controller',
                        '--param-file', controller_file,
                        '--controller-manager-timeout', '10.0'],
            output    = 'screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )]
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            jsb_spawner,
            mecanum_drive_spawner,
        ]
    )
