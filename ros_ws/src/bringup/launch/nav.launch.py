#!/usr/bin/env python3
'''
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument('slam_tb_config_file', default_value='slam_toolbox_async_online.yaml'),
        DeclareLaunchArgument('nav2_params_file', default_value='nav2_params.yaml'),
    ]

    use_sim_time             = LaunchConfiguration('use_sim_time')
    nav2_params_file         = LaunchConfiguration("nav2_params_file")
    
    nav2_params = ParameterFile(
        PathJoinSubstitution([pkg_bringup, "config", nav2_params_file]),
        allow_substs=True,
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_bringup, 'config', 'twist_mux.yaml']),
            {'use_sim_time' : use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],
    )
    
    # name, package, plugin
    nav2_specs = [
        ("planner_server",    "nav2_planner",           "nav2_planner::PlannerServer"),
        ("controller_server", "nav2_controller",        "nav2_controller::ControllerServer"),
        ("smoother_server",   "nav2_smoother",          "nav2_smoother::SmootherServer"),
        ("bt_navigator",      "nav2_bt_navigator",      "nav2_bt_navigator::BtNavigator"),
        # ("behavior_server",   "nav2_behaviors",         "nav2_behaviors::BehaviorServer"),
        # ("waypoint_follower", "nav2_waypoint_follower", "nav2_waypoint_follower::WaypointFollower"),
        # ("velocity_smoother", "nav2_velocity_smoother", "nav2_velocity_smoother::VelocitySmoother"),
    ]
    
    nav2_components = [
        ComposableNode(
            package=pkg,
            plugin=plugin,
            name=name,
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for (name, pkg, plugin) in nav2_specs
    ]
    
    nav2_container = ComposableNodeContainer(
        name="nav2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=nav2_components,
    )
    
    lifecycle_mgr = Node(
        package     = 'nav2_lifecycle_manager',
        executable  = 'lifecycle_manager',
        name        = 'lifecycle_manager_nav',
        output      = 'screen',
        parameters  = [{
            'autostart':    True,
            'node_names':   [name for (name, _, _) in nav2_specs],
            'bond_timeout': 4.0,
            'bond_respawn_max_duration': 10.0,
            'use_sim_time' : False,
            'attempt_respawn_reconnection': True
        }]
    )
    
    delayed_lifecycle = RegisterEventHandler(OnProcessStart(target_action=nav2_container, on_start=[lifecycle_mgr]))
    
    return LaunchDescription(
        declared_arguments + [
            twist_mux_node,
            # nav2_container,
            # delayed_lifecycle,
            lifecycle_mgr
        ]
    )