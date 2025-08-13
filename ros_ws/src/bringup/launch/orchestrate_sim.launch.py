#!/usr/bin/env python3
"""
orchestrate bringup for simulation

Usage examples:
    ros2 launch bringup orchestrate_sim.launch.py
    ros2 launch bringup orchestrate_sim.launch.py headless=true -d
    
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/mecanum_drive_controller/reference
    
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world',    default_value='simple_world.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value="false"),
    ]

    world           = LaunchConfiguration('world')
    headless        = LaunchConfiguration('headless')
    use_sim_time    = LaunchConfiguration('use_sim_time')
        
    pkg_bringup = FindPackageShare('bringup')
    pkg_rf2o    = FindPackageShare('rf2o_laser_odometry')

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
    # slam / nav
    # ────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'nav.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ────────────────────────────────
    # rf2o
    # ────────────────────────────────
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_bringup, 'launch', 'rf2o.launch.py'])]
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
        
    # def _dump_configs(context, *args, **kwargs):
    #     print('\n=== LAUNCH CONFIG DUMP ===')
    #     for k, v in sorted(context.launch_configurations.items()):
    #         print(f"{k}: {type(v).__name__} -> {v!r}")
    #     print('=== END DUMP ===\n')
    #     return []

    # OpaqueFunction(function=_dump_configs)
    
    
    ros_group = GroupAction(actions=[
        bridge_launch,
        spawn_robot_launch,
        sensor_launch,
        controllers_launch,
        # OpaqueFunction(function=_dump_configs),
        TimerAction(period=5.0, actions=[nav_launch]),        
        TimerAction(period=10.0, actions=[rf2o_launch]),
        TimerAction(period=10.0, actions=[rviz_launch])
    ])

    return LaunchDescription(declared_arguments + [
        gz_launch,
        ros_group
    ])

