from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

desc_pkg = Path(get_package_share_directory('description'))
urdf_file = desc_pkg / 'urdf' / 'omniseer.urdf'

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='rsp',
             parameters=[{'robot_description': urdf_file.read_text()}]),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             name='jsp'),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz',
             arguments=['-d', ''],
             output='screen'),
    ])
