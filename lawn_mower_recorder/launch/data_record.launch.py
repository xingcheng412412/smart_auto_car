from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_dir",
                default_value="~/fw_mini_pro_control_ros2/recorded_files",
            ),
            Node(
                package="lawn_mower_recorder",
                executable="recorder_enhanced.py",
                name="recorder_enhanced",
                output="screen",
                parameters=[
                    {"base_dir": LaunchConfiguration("base_dir")},
                ],
            ),
        ]
    )
