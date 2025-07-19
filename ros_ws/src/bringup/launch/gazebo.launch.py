"""
bringup for gazebo server and optional client

consumed by simulation.launch.py

"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    world    = LaunchConfiguration('world',    default='simple_world.world')
    headless = LaunchConfiguration('headless', default='false')

    bringup_share  = FindPackageShare('bringup')

    world_path = PathJoinSubstitution([
        bringup_share, 'worlds', world
    ])

    config_path = PathJoinSubstitution([
        bringup_share, 'cofig','gz_config.config'
    ])

    # set_plugin_path = SetEnvironmentVariable(
    #         'GAZEBO_PLUGIN_PATH',
    #         PathJoinSubstitution([FindPackageShare('omniseer_gazebo'), 'lib']))

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('omniseer_description'), '..'])
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-v', '--gui-config', config_path],
        output='screen',
        condition=UnlessCondition(headless)
    )

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', world_path, '-v'],
        output='screen',
        condition=IfCondition(headless)
    )

    return LaunchDescription([
        # set_plugin_path
        set_resource_path,
        gz_gui,
        gz_headless
    ])
