#!/usr/bin/env python3
"""
bringup for complete Gazebo - ROS2 simulation
alternative headless CI mode

Components:
--------------

GZ server + client
robot state publisher
robot spawn
ros gz bridge


Example usage:
--------------

ros2 launch bringup sim.launch.py                            - Full sim
ros2 launch bringup sim.launch.py gui:=false headless:= true - CI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # args
    declared_arguments = [
        DeclareLaunchArgument('world',    default_value='simple_world.world'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x',        default_value='0'),
        DeclareLaunchArgument('y',        default_value='0'),
        DeclareLaunchArgument('z',        default_value='0.15'),
        DeclareLaunchArgument('rviz',     default_value='false'),
    ]

    bringup_share = FindPackageShare('bringup')
    desc_share    = FindPackageShare('omniseer_description')

    robot_description = Command([
        'xacro ', PathJoinSubstitution([desc_share, 'urdf', 'xacro', 'omniseer.urdf.xacro']),
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share, 'launch', 'gazebo.launch.py'
            ])
        ),

        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time'     : True}],
        output='screen',
        name='rsp',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'omniseer',
                   '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            rsp,
            spawn,
            bridge,
            # Later: Include tele-op, bag recorders, etc.
        ])
