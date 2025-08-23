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
        DeclareLaunchArgument('nav2_params_file', default_value='nav2_params.yaml'),
        # DeclareLaunchArgument('use_debug', default_value='false',    description='Enable YOLO debug images')
    ]

    use_sim_time             = LaunchConfiguration('use_sim_time')
    slam_tb_config_file      = LaunchConfiguration("slam_tb_config_file")
    nav2_params_file      = LaunchConfiguration("nav2_params_file")
    
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
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # by default reads /cmd_vel, which then goes into velocity_smoother
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(   # if on older Nav2, switch to nav2_recoveries/recoveries_server
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # default topics: /cmd_vel -> /cmd_vel_smoothed
    )
    

    lifecycle_mgr = Node(
        package     = 'nav2_lifecycle_manager',
        executable  = 'lifecycle_manager',
        name        = 'lifecycle_manager',
        output      = 'screen',
        parameters  = [{
            'autostart':    True,
            'node_names':   [
                'slam_toolbox',
                'controller_server',
                'planner_server',
                'smoother_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother',
            ],
            'bond_timeout': 0.0,
            'bond_heartbeat_period': 0.0,
            'use_sim_time' : False
        }]
    )
    
    return LaunchDescription(
        declared_arguments + [
            twist_mux_node,
            slam_toolbox_node,
            controller_server,
            planner_server,
            smoother_server,
            bt_navigator,
            behavior_server,
            waypoint_follower,
            velocity_smoother,
            yolo_group,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=slam_toolbox_node,
                    on_start=[TimerAction(period=3.0, actions=[lifecycle_mgr])]
                )
            ),
        ]
    )


#!/usr/bin/env python3
"""
Unified bringup for SLAM + Nav2 with optional composition.
- Replaces the two provided launch files with one parameterized entrypoint.
- Supports both component composition and classic processes.
- Builds lifecycle node list dynamically. Removes brittle timers.
- Wires cmd_vel -> velocity_smoother -> twist_mux -> base controller.

Tested patterns mirror nav2_bringup conventions. Adjust flags as needed.
"""
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import (
    Node,
    LoadComposableNodes,
    ComposableNodeContainer,
    SetParameter,
)
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _as_bool(cfg: LaunchConfiguration):
    # Helper for building lifecycle list inside OpaqueFunction
    return PythonExpression([cfg])


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_bringup = get_package_share_directory('bringup') 

    # Core args
    namespace          = LaunchConfiguration('namespace')
    use_sim_time       = LaunchConfiguration('use_sim_time')
    autostart          = LaunchConfiguration('autostart')
    log_level          = LaunchConfiguration('log_level')
    use_composition    = LaunchConfiguration('use_composition')
    container_name     = LaunchConfiguration('container_name')
    use_respawn        = LaunchConfiguration('use_respawn')

    # Feature toggles
    enable_slam        = LaunchConfiguration('enable_slam')
    enable_waypoints   = LaunchConfiguration('enable_waypoints')
    enable_smoother    = LaunchConfiguration('enable_smoother')
    enable_vel_smoother= LaunchConfiguration('enable_velocity_smoother')
    enable_collision   = LaunchConfiguration('enable_collision_monitor')
    enable_route       = LaunchConfiguration('enable_route_server')

    # Files
    params_file        = LaunchConfiguration('nav2_params_file')
    slam_tb_config     = LaunchConfiguration('slam_tb_config_file')
    twist_mux_config   = LaunchConfiguration('twist_mux_config_file')

    container_name_full = (namespace, TextSubstitution(text='/'), container_name)

    # Remaps
    remaps_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Inject dynamic params
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declarations
    declared = [
        DeclareLaunchArgument('namespace', default_value='', description='ROS namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='false'),

        # Feature toggles
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('enable_waypoints', default_value='false'),
        DeclareLaunchArgument('enable_smoother', default_value='true'),
        DeclareLaunchArgument('enable_velocity_smoother', default_value='true'),
        DeclareLaunchArgument('enable_collision_monitor', default_value='false'),
        DeclareLaunchArgument('enable_route_server', default_value='false'),
        DeclareLaunchArgument('enable_docking', default_value='false'),
        DeclareLaunchArgument('enable_yolo', default_value='false'),

        # Files
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_tb_config_file',
            default_value='slam_toolbox_async_online.yaml',
            description='File under bringup/config/',
        ),
        DeclareLaunchArgument(
            'twist_mux_config_file',
            default_value='twist_mux.yaml',
            description='File under bringup/config/',
        ),
        DeclareLaunchArgument(
            'yolo_launch_file',
            default_value='yolov11.launch.py',
            description='File under bringup/launch/',
        ),
    ]

    # Twist mux (not lifecycle)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            ParameterFile(os.path.join(my_bringup, 'config', twist_mux_config), allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Classic processes group
    classic = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # SLAM (lifecycle-aware). Only when enabled
            Node(
                condition=IfCondition(enable_slam),
                package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                output='screen',
                parameters=[
                    ParameterFile(os.path.join(my_bringup, 'config', slam_tb_config), allow_substs=True),
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            # Nav2 core
            Node(
                package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level],
                remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                condition=IfCondition(enable_smoother),
                package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level],
                remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                condition=IfCondition(enable_waypoints),
                package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                condition=IfCondition(enable_vel_smoother),
                package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level],
                remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                condition=IfCondition(enable_collision),
                package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                condition=IfCondition(enable_route),
                package='nav2_route', executable='route_server', name='route_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            Node(
                condition=IfCondition(enable_docking),
                package='opennav_docking', executable='opennav_docking', name='docking_server', output='screen',
                parameters=[configured_params], arguments=['--ros-args', '--log-level', log_level], remappings=remaps_tf,
            ),
            # Lifecycle manager
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                output='screen', arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'autostart': autostart,
                    'node_names': [],  # filled by opaque fn
                    'bond_timeout': 0.0,
                    'bond_heartbeat_period': 0.0,
                }],
            ),
        ],
    )

    # Component container and loaders
    container = ComposableNodeContainer(
        condition=IfCondition(use_composition),
        name=container_name,
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded container
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[],  # nodes loaded below
        emulate_tty=True,
    )

    load_components = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            # Nav2 core as components
            ComposableNode(
                package='nav2_planner', plugin='nav2_planner::PlannerServer', name='planner_server',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                package='nav2_controller', plugin='nav2_controller::ControllerServer', name='controller_server',
                parameters=[configured_params], remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                condition=IfCondition(enable_smoother),
                package='nav2_smoother', plugin='nav2_smoother::SmootherServer', name='smoother_server',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator', name='bt_navigator',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                package='nav2_behaviors', plugin='behavior_server::BehaviorServer', name='behavior_server',
                parameters=[configured_params], remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                condition=IfCondition(enable_waypoints),
                package='nav2_waypoint_follower', plugin='nav2_waypoint_follower::WaypointFollower', name='waypoint_follower',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                condition=IfCondition(enable_vel_smoother),
                package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother', name='velocity_smoother',
                parameters=[configured_params], remappings=remaps_tf + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                condition=IfCondition(enable_collision),
                package='nav2_collision_monitor', plugin='nav2_collision_monitor::CollisionMonitor', name='collision_monitor',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                condition=IfCondition(enable_route),
                package='nav2_route', plugin='nav2_route::RouteServer', name='route_server',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            ComposableNode(
                condition=IfCondition(enable_docking),
                package='opennav_docking', plugin='opennav_docking::DockingServer', name='docking_server',
                parameters=[configured_params], remappings=remaps_tf,
            ),
            # Lifecycle manager runs as component too
            ComposableNode(
                package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'autostart': autostart, 'node_names': []}],  # filled by opaque fn
            ),
        ],
    )

    # Optional: Keep SLAM outside container to avoid heavy CPU contention, but still lifecycle-managed
    slam_proc = Node(
        condition=IfCondition(enable_slam),
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen', parameters=[
            ParameterFile(os.path.join(my_bringup, 'config', slam_tb_config), allow_substs=True),
            {'use_sim_time': use_sim_time},
        ], arguments=['--ros-args', '--log-level', log_level],
    )

    # Compute lifecycle node names dynamically based on toggles and mode
    def _inject_lifecycle_nodes(context, *args, **kwargs):
        use_comp = context.launch_configurations['use_composition'].lower() == 'true'
        toggles = {
            'enable_slam': context.launch_configurations.get('enable_slam', 'true') == 'true',
            'enable_smoother': context.launch_configurations.get('enable_smoother', 'true') == 'true',
            'enable_waypoints': context.launch_configurations.get('enable_waypoints', 'false') == 'true',
            'enable_velocity_smoother': context.launch_configurations.get('enable_velocity_smoother', 'true') == 'true',
            'enable_collision_monitor': context.launch_configurations.get('enable_collision_monitor', 'false') == 'true',
            'enable_route_server': context.launch_configurations.get('enable_route_server', 'false') == 'true',
            'enable_docking': context.launch_configurations.get('enable_docking', 'false') == 'true',
        }
        nodes: List[str] = [
            'planner_server', 'controller_server', 'bt_navigator', 'behavior_server'
        ]
        if toggles['enable_smoother']:
            nodes.append('smoother_server')
        if toggles['enable_waypoints']:
            nodes.append('waypoint_follower')
        if toggles['enable_velocity_smoother']:
            nodes.append('velocity_smoother')
        if toggles['enable_collision_monitor']:
            nodes.append('collision_monitor')
        if toggles['enable_route_server']:
            nodes.append('route_server')
        if toggles['enable_docking']:
            nodes.append('docking_server')
        if toggles['enable_slam']:
            nodes.insert(0, 'slam_toolbox')

        # Patch parameter on lifecycle manager depending on composition mode
        # In composition, lifecycle manager is inside container; in classic, it is a separate Node
        from launch_ros.utilities import evaluate_parameters
        # Find the entity by name in current entities
        # Simpler approach: set a global parameter override
        return [SetParameter(name='/lifecycle_manager_navigation.node_names', value=nodes)]

    inject = OpaqueFunction(function=_inject_lifecycle_nodes)

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf)
    for a in declared:
        ld.add_action(a)

    # Always run twist mux and optional YOLO regardless of composition
    ld.add_action(twist_mux)

    # Composition path
    ld.add_action(container)
    ld.add_action(load_components)
    ld.add_action(slam_proc)

    # Classic path
    ld.add_action(classic)

    # Inject lifecycle node list at launch-time
    ld.add_action(inject)

    return ld
