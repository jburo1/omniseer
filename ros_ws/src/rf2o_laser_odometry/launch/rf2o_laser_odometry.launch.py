from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                # 'init_pose_from_topic' : '',
                "freq": 20.0,
            }
        ],
        arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # rf2o_laser_odom_node
        ]
    )
