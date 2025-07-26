#!/usr/bin/env python3
"""
bringup for complete Gazebo - ROS2 simulation
alternative headless CI mode

Components:
--------------

GZ server + optional client via ros_gz wrapper launcher
robot spawn via XACRO->URDF
controller_manager via gazebo plugin init in URDF
input mux for teleoperation/nav/emergency stop
ros gz bridge - clock/tf

Example usage:
--------------

ros2 launch bringup sim.launch.py                            - Full sim
ros2 launch bringup sim.launch.py headless:=true             - CI
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world',    default_value='simple_world.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value="false")
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    pkg_bringup  = FindPackageShare('bringup')
    pkg_omniseer = FindPackageShare('omniseer_description')

    # ------------- ROS_GZ ------------- #

    set_sim_system_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        [EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''), ':/opt/ros/kilted/lib']
    )

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('omniseer_description'), '..'])
    )

    world_path = PathJoinSubstitution([
        pkg_bringup, 'worlds',
        LaunchConfiguration('world')
    ])

    gz_config_path = PathJoinSubstitution([
        pkg_bringup, 'config', 'gz_config.config'
    ])

    gz_ros_gui_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments={
            "gz_args": [
                " -r -v 1 ",
                world_path,
                " --gui-config ", gz_config_path,
            ]
        }.items(),
        condition=UnlessCondition(headless)
    )

    gz_ros_headless_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                "-s -r -v 1 ", world_path
            ]
        }.items(),
        condition=IfCondition(headless),
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ------------- RVIZ ------------- #

    rviz_config_path = PathJoinSubstitution([
        pkg_bringup, 'config', 'rviz_config.rviz'
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments = [
            '-d', rviz_config_path
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        # condition=IfCondition(gui),
    )

    # ------------- SPAWN ROBOT ------------- #

    xacro_file   = PathJoinSubstitution([pkg_omniseer, 'urdf', 'xacro', 'omniseer.urdf.xacro'])
    robot_description_urdf = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_urdf,
                     'use_sim_time': use_sim_time
                     }],
        output='both',
    )

    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'omniseer',
                   '-topic', 'robot_description',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.5',
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],

    )

    # ------------- TWIST MUX ------------- #

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                pkg_bringup, 'config', 'twist_mux.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],

    )


    # ------------- CONTROLLERS/BROADCASTERS ------------- #

    jsb_node = Node(
        package   = 'controller_manager',
        executable= 'spawner',
        name = 'jsb',
        arguments = ['joint_state_broadcaster'],
        output    = 'screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    jsb_eh = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_node,
            on_exit=[jsb_node]
        )
    )

    controller_file = PathJoinSubstitution([pkg_bringup, 'config', 'controllers.yaml'])

    mecanum_drive_node = Node(
        package   = 'controller_manager',
        executable= 'spawner',
        arguments = ['mecanum_drive_controller',
                     '--param-file', controller_file,
                     '--controller-ros-args', '-r /mecanum_drive_controller/tf_odometry:=/tf',
                     '--controller-manager-timeout', '10.0'],
        output    = 'screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mecanum_drive_eh = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_node,
            on_exit=[mecanum_drive_node]
        )
    )

    return LaunchDescription(
        declared_arguments + [
        set_sim_system_path,
        set_resource_path,
        gz_ros_gui_ld,
        gz_ros_headless_ld,
        bridge_node,
        robot_spawn_node,
        jsb_eh,
        mecanum_drive_eh,
        rsp_node,
        twist_mux,
        rviz_node,
    ])
