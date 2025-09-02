#!/usr/bin/env python3
"""
orchestrate bringup for simulation

Usage examples:
    ros2 launch bringup orchestrate_sim.launch.py
    ros2 launch bringup orchestrate_sim.launch.py headless=true -d
    
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/mecanum_drive_controller/reference
    
"""
from launch import LaunchDescription
from launch.actions import (LogInfo, DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world',    default_value='simple_world.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value="false"),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='debug|info|warn|error|fatal'),
    ]

    world           = LaunchConfiguration('world')
    headless        = LaunchConfiguration('headless')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    log_level       = LaunchConfiguration('log_level')
    
    pkg_bringup = FindPackageShare('bringup')

    # Launch descriptions
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={
            'world': world,
            'headless': headless,
        }.items()
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'spawn_robot.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'bridge.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'controllers.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'sensors.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )
    
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'perception.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'nav.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'rviz.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items()
    )
    
    # Poll until /clock exists
    wait_clock = ExecuteProcess(
        cmd=['bash','-lc', 'until ros2 topic list | grep -qx /clock; do sleep 0.2; done'],
        name='wait_clock'
    )

    # Poll until /tf_static exists
    wait_tf_static = ExecuteProcess(
        cmd=['bash','-lc', 'until ros2 topic list | grep -qx /tf_static; do sleep 0.2; done'],
        name='wait_tf_static'
    )

    # Poll until /map exists
    wait_map = ExecuteProcess(
        cmd=['bash','-lc', 'until ros2 topic list | grep -qx /map; do sleep 0.2; done'],
        name='wait_map'
    )
    
    # Poll until /scan exists
    wait_scan = ExecuteProcess(
        cmd=['bash','-lc', 'until ros2 topic list | grep -qx /scan; do sleep 0.2; done'],
        name='wait_scan'
    )
    
    # Poll until map->odom tf exists
    wait_map_base_link_tf = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([pkg_bringup, 'bringup', 'wait_for_tf.py']),
            'map',
            'base_link',
        ],
        name='wait_map_base_link_tf'
    )

    # Events
    on_clock = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_clock,
            on_exit=[
                LogInfo(msg="/clock found"), 
                spawn_robot_launch, 
                wait_tf_static
                ]
        )
    )

    on_tf = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_tf_static,
            on_exit=[
                LogInfo(msg="/tf_static found"), 
                controllers_launch, sensor_launch, perception_launch, 
                wait_map,
                ]
        )
    )

    on_map = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_map,
            on_exit=[
                LogInfo(msg="/map found"), 
                wait_map_base_link_tf, 
                ]
        )
    )
    
    # on_scan = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=wait_scan,
    #         on_exit=[]
    #     )
    # )
    
    on_odom_map_tf = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_map_base_link_tf,
            on_exit=[
                    LogInfo(msg="map->base_link tf found"),
                    nav_launch, rviz_launch
                    ]
        )
    )
    
    return LaunchDescription(declared_arguments + [
        SetEnvironmentVariable('RCUTILS_LOGGING_DEFAULT_LEVEL', log_level),

        gz_launch,
        bridge_launch,
        wait_clock,

        on_clock,
        on_tf,
        on_map,
        # on_scan,
        on_odom_map_tf
    ])

