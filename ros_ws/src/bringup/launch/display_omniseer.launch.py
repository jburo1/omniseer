from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare   # nice shorthand

def generate_launch_description():
    # Path to the Xacro in the *install* space
    xacro_path = PathJoinSubstitution([
        FindPackageShare('omniseer_description'),
        'urdf', 'xacro', 'omniseer.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='jsp',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            # arguments=['-d', '<optional RViz config>'],
        ),
    ])
