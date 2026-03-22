#!/usr/bin/env python3
"""Launch real-hardware producers and adapters below the IO boundary."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
        DeclareLaunchArgument("micro_ros_serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("micro_ros_baud", default_value="115200"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("lidar_serial_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("lidar_frame_id", default_value="lidar_frame"),
        DeclareLaunchArgument("lidar_inverted", default_value="false"),
        DeclareLaunchArgument("lidar_angle_compensate", default_value="true"),
        DeclareLaunchArgument("encoder_odometry_params_file", default_value="encoder_odometry.yaml"),
    ]

    log_level = LaunchConfiguration("log_level")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    micro_ros_serial_device = LaunchConfiguration("micro_ros_serial_device")
    micro_ros_baud = LaunchConfiguration("micro_ros_baud")
    start_lidar = LaunchConfiguration("start_lidar")
    lidar_serial_device = LaunchConfiguration("lidar_serial_device")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    encoder_odometry_params_file = LaunchConfiguration("encoder_odometry_params_file")

    encoder_odometry_params_path = PathJoinSubstitution(
        [pkg_bringup, "config", encoder_odometry_params_file]
    )

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=[
            "serial",
            "--dev",
            micro_ros_serial_device,
            "-b",
            micro_ros_baud,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(start_micro_ros_agent),
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_composition",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "serial_port": ParameterValue(lidar_serial_device, value_type=str),
                "serial_baudrate": ParameterValue(lidar_baudrate, value_type=int),
                "frame_id": ParameterValue(lidar_frame_id, value_type=str),
                "inverted": ParameterValue(lidar_inverted, value_type=bool),
                "angle_compensate": ParameterValue(lidar_angle_compensate, value_type=bool),
            }
        ],
        condition=IfCondition(start_lidar),
    )

    encoder_odometry_node = Node(
        package="robot_io_adapters",
        executable="encoder_counts_to_odometry",
        name="encoder_counts_to_odometry",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[ParameterFile(encoder_odometry_params_path, allow_substs=True)],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            micro_ros_agent_node,
            lidar_node,
            encoder_odometry_node,
        ]
    )
