#!/usr/bin/env python3
"""Launch real-hardware producers and adapters below the IO boundary."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_config_is_true(config: LaunchConfiguration, context) -> bool:
    return config.perform(context).strip().lower() in {"1", "true", "yes", "on"}


def _handle_required_process_exit(process_name: str, success_actions, failure_reason: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return success_actions
        return [
            LogInfo(msg=f"{process_name} failed with exit code {event.returncode}"),
            EmitEvent(event=Shutdown(reason=failure_reason)),
        ]

    return _on_exit


def _handle_teensy_preflight_exit(
    process_name: str,
    success_actions,
    require_teensy: LaunchConfiguration,
):
    def _on_exit(event, context):
        if event.returncode == 0:
            return success_actions

        failure_message = f"{process_name} failed with exit code {event.returncode}"
        if _launch_config_is_true(require_teensy, context):
            return [
                LogInfo(msg=failure_message),
                EmitEvent(event=Shutdown(reason="Teensy preflight failed")),
            ]

        return [
            LogInfo(
                msg=(
                    f"{failure_message}; continuing without micro_ros_agent because "
                    "require_teensy:=false"
                )
            )
        ]

    return _on_exit


def _truthy_expression(config: LaunchConfiguration) -> PythonExpression:
    return PythonExpression(["'", config, "'.lower() in ['1', 'true', 'yes', 'on']"])


def _falsy_expression(config: LaunchConfiguration) -> PythonExpression:
    return PythonExpression(["'", config, "'.lower() not in ['1', 'true', 'yes', 'on']"])


def generate_launch_description():
    pkg_bringup = FindPackageShare("bringup")

    declared_arguments = [
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
        DeclareLaunchArgument("micro_ros_serial_device", default_value="/dev/omniseer_teensy"),
        DeclareLaunchArgument("micro_ros_baud", default_value="115200"),
        DeclareLaunchArgument("require_teensy", default_value="true"),
        DeclareLaunchArgument("teensy_preflight_timeout_sec", default_value="20"),
        DeclareLaunchArgument("allow_teensy_power_cycle", default_value="false"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "lidar_serial_device",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
        ),
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
    require_teensy = LaunchConfiguration("require_teensy")
    teensy_preflight_timeout_sec = LaunchConfiguration("teensy_preflight_timeout_sec")
    allow_teensy_power_cycle = LaunchConfiguration("allow_teensy_power_cycle")
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
    wait_for_teensy_script = PathJoinSubstitution([pkg_bringup, "scripts", "wait_for_teensy.py"])

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

    teensy_power_cycle_guard = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            'echo "allow_teensy_power_cycle:=true is not implemented" >&2; exit 1',
        ],
        name="teensy_power_cycle_not_supported",
        condition=IfCondition(
            PythonExpression(
                [
                    _truthy_expression(start_micro_ros_agent),
                    " and ",
                    _truthy_expression(allow_teensy_power_cycle),
                ]
            )
        ),
    )

    shutdown_for_unsupported_power_cycle = RegisterEventHandler(
        OnProcessExit(
            target_action=teensy_power_cycle_guard,
            on_exit=_handle_required_process_exit(
                process_name="teensy_power_cycle_not_supported",
                success_actions=[],
                failure_reason="allow_teensy_power_cycle:=true is not implemented",
            ),
        )
    )

    teensy_preflight = ExecuteProcess(
        cmd=[
            "python3",
            wait_for_teensy_script,
            "--device",
            micro_ros_serial_device,
            "--timeout-sec",
            teensy_preflight_timeout_sec,
        ],
        name="wait_for_teensy",
        condition=IfCondition(
            PythonExpression(
                [
                    _truthy_expression(start_micro_ros_agent),
                    " and ",
                    _falsy_expression(allow_teensy_power_cycle),
                ]
            )
        ),
    )

    launch_micro_ros_agent_after_preflight = RegisterEventHandler(
        OnProcessExit(
            target_action=teensy_preflight,
            on_exit=_handle_teensy_preflight_exit(
                process_name="wait_for_teensy",
                success_actions=[micro_ros_agent_node],
                require_teensy=require_teensy,
            ),
        )
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
            teensy_power_cycle_guard,
            shutdown_for_unsupported_power_cycle,
            teensy_preflight,
            launch_micro_ros_agent_after_preflight,
            lidar_node,
            encoder_odometry_node,
        ]
    )
