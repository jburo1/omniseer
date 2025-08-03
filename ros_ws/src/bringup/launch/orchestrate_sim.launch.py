#!/usr/bin/env python3
"""
orchestrate bringup for simulation
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world',    default_value='simple_world.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value="false"),
    ]

    world     = LaunchConfiguration('world')
    headless  = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_bringup = FindPackageShare('bringup')

    # ────────────────────────────────
    # gz launch
    # ────────────────────────────────
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={
            'world': world,
            'headless': headless
        }.items()
    )

    # ────────────────────────────────
    # spawn robot
    # ────────────────────────────────
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'spawn_robot.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ────────────────────────────────
    # bridge
    # ────────────────────────────────
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'bridge.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ────────────────────────────────
    # controllers
    # ────────────────────────────────
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'controllers.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ────────────────────────────────
    # sensor fusion
    # ────────────────────────────────
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'sensors.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ────────────────────────────────
    # nav
    # ────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'nav.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    # ────────────────────────────────
    # rviz
    # ────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_bringup, 'launch', 'rviz.launch.py'])]
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )

    return LaunchDescription(declared_arguments + [
        gz_launch,
        spawn_robot_launch,
        bridge_launch,
        sensor_launch,
        controllers_launch,
        nav_launch,
        rviz_launch
    ])

