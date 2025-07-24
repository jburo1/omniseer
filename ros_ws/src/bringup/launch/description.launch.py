from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='If true, use /clock from simulator.')

    pkg_omniseer = FindPackageShare('omniseer_description')
    xacro_file   = PathJoinSubstitution([pkg_omniseer, 'urdf', 'xacro', 'omniseer.urdf.xacro'])
    robot_description_content = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_robot_description': True
        }],
        output='both'
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
    ])
