# Launches the bot in simulation
# Components:
# GZ server + client
# robot_state_publisher
# controllers
# ros gz bridge
# Usage: ros2 launch sim.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('omniseer_sim'), 'worlds', 'simple_world.world']
        )
    )

    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    model_arg = DeclareLaunchArgument(
        'model_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('omniseer_description'), 'urdf', 'omniseer.urdf.xacro'])
    )
    bridge_arg = DeclareLaunchArgument('with_bridge', default_value='false')



    return LaunchDescription([
        world_arg, gui_arg, model_arg, bridge_arg,
        SetParameter('use_sim_time', True),
        gz_server, gz_client,
        rsp,
        spawn,
        controllers,
        bridge,
        # Later: Include tele-op, bag recorders, etc.
    ])
