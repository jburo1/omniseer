from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_yolo_bringup = FindPackageShare("yolo_bringup")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_yolo_bringup, "launch", "yolo.launch.py"])]),
                launch_arguments={
                    "model_type": "World",
                    "model": LaunchConfiguration("model", default="yolov8l-worldv2.pt"),
                    "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "threshold": LaunchConfiguration("threshold", default="0.5"),
                    "input_image_topic": LaunchConfiguration("input_image_topic", default="/front_camera/image"),
                    "image_reliability": LaunchConfiguration("image_reliability", default="1"),
                    "namespace": LaunchConfiguration("namespace", default="yolo"),
                }.items(),
            )
        ]
    )
