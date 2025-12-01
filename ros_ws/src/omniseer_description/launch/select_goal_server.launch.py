from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            Node(
                package="omniseer_description",
                executable="select_goal_server",
                name="select_goal_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
