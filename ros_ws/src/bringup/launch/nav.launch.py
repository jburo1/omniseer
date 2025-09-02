'''
'''
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_bringup = FindPackageShare('bringup')
    
    use_sim_time             = LaunchConfiguration('use_sim_time')
    nav2_params_file         = LaunchConfiguration("nav2_params_file")
    log_level                = LaunchConfiguration('log_level')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav2_params_file', default_value='nav2_params.yaml'),
    ]

    nav2_params_path = PathJoinSubstitution([pkg_bringup, "config", nav2_params_file])
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_path,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    rewrite = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )
    
    lifecycle_nodes = [
        'controller_server',
        # 'smoother_server',
        # 'planner_server',
        # 'route_server',
        # 'behavior_server',
        # 'velocity_smoother',
        # 'collision_monitor',
        # 'bt_navigator',
        # 'waypoint_follower',
        # 'docking_server',
    ]
    
    nav2_container = ComposableNodeContainer(
        name="nav2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        arguments=['--ros-args', '--log-level', log_level, '--params-file', rewrite],
    )
    
    load_composable_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=nav2_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=[('cmd_vel', 'cmd_vel_nav')],
                    ),
                    # ComposableNode(
                    #     package='nav2_smoother',
                    #     plugin='nav2_smoother::SmootherServer',
                    #     name='smoother_server',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # ComposableNode(
                    #     package='nav2_planner',
                    #     plugin='nav2_planner::PlannerServer',
                    #     name='planner_server',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # ComposableNode(
                    #     package='nav2_behaviors',
                    #     plugin='behavior_server::BehaviorServer',
                    #     name='behavior_server',
                    #     parameters=[configured_params],
                    #     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    # ),
                    # ComposableNode(
                    #     package='nav2_bt_navigator',
                    #     plugin='nav2_bt_navigator::BtNavigator',
                    #     name='bt_navigator',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # ComposableNode(
                    #     package='nav2_waypoint_follower',
                    #     plugin='nav2_waypoint_follower::WaypointFollower',
                    #     name='waypoint_follower',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # ComposableNode(
                    #     package='nav2_velocity_smoother',
                    #     plugin='nav2_velocity_smoother::VelocitySmoother',
                    #     name='velocity_smoother',
                    #     parameters=[configured_params],
                    #     remappings=remappings
                    #     + [('cmd_vel', 'cmd_vel_nav')],
                    # ),
                    # ComposableNode(
                    #     package='nav2_collision_monitor',
                    #     plugin='nav2_collision_monitor::CollisionMonitor',
                    #     name='collision_monitor',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {
                                'use_sim_time' : use_sim_time,
                                'autostart':    True,
                                'node_names':   lifecycle_nodes,
                                'bond_timeout': 4.0,
                                'bond_respawn_max_duration': 10.0,
                                'attempt_respawn_reconnection': True,
                            }
                        ]
                    ),
                ],
            ),
        ],
    )
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            PathJoinSubstitution([pkg_bringup, 'config', 'twist_mux.yaml']),
            {'use_sim_time' : use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],
    )
    
    return LaunchDescription(
        declared_arguments + [
            stdout_linebuf_envvar,
            twist_mux_node,
            nav2_container,
            load_composable_nodes
        ]
    )