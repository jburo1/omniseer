#!/usr/bin/env python3
""" """

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("slam_tb_config_file", default_value="slam_toolbox_async_online.yaml"),
        DeclareLaunchArgument("start_yolo", default_value="true"),
        DeclareLaunchArgument("yolo_model", default_value="yolov8s-worldv2.pt"),
        DeclareLaunchArgument("yolo_device", default_value="cuda:0"),
        DeclareLaunchArgument("yolo_threshold", default_value="0.5"),
        DeclareLaunchArgument("yolo_image_reliability", default_value="2"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rf2o", default_value="true"),
    ]

    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_tb_config_file = LaunchConfiguration("slam_tb_config_file")
    start_yolo = LaunchConfiguration("start_yolo")
    yolo_model = LaunchConfiguration("yolo_model")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_threshold = LaunchConfiguration("yolo_threshold")
    yolo_image_reliability = LaunchConfiguration("yolo_image_reliability")
    start_slam = LaunchConfiguration("start_slam")
    start_rf2o = LaunchConfiguration("start_rf2o")

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            ParameterFile(
                PathJoinSubstitution([pkg_bringup, "config", slam_tb_config_file]),
                allow_substs=True,
            ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_slam),
    )

    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["slam_toolbox"],
                "bond_timeout": 0.0,
                "bond_heartbeat_period": 0.0,
                "attempt_respawn_reconnection": True,
            }
        ],
        condition=IfCondition(start_slam),
    )

    yolo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_bringup, "launch", "yolo-world.launch.py"])]),
        launch_arguments={
            "model": yolo_model,
            "device": yolo_device,
            "threshold": yolo_threshold,
            "input_image_topic": "/front_camera/image",
            "image_reliability": yolo_image_reliability,
        }.items(),
    )

    yolo_group = GroupAction(
        [SetParameter(name="use_sim_time", value=use_sim_time), yolo_include],
        condition=IfCondition(start_yolo),
    )

    rf2o_laser_odom_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 8.0,
            }
        ],
        condition=IfCondition(start_rf2o),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            slam_toolbox_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=slam_toolbox_node,
                    on_start=[TimerAction(period=2.0, actions=[lifecycle_manager_slam])],
                ),
                condition=IfCondition(start_slam),
            ),
            yolo_group,
            rf2o_laser_odom_node,
        ]
    )
