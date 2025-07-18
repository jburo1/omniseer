"""



"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_omniseer = FindPackageShare('omniseer_description')

    xacro_path = PathJoinSubstitution([
        pkg_omniseer, 'urdf', 'xacro', 'omniseer.urdf.xacro'
    ])

    rviz_cfg_path = PathJoinSubstitution([
        pkg_omniseer, 'rviz', 'display_omniseer.rviz'
    ])

    robot_description = Command(['xacro ', xacro_path])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp',
        parameters=[{'robot_description': robot_description}],
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='jsp',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_cfg_path],
    )

    return LaunchDescription([
        rsp,
        jsp,
        rviz,
    ])
