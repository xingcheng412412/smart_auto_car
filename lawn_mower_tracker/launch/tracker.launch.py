from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    tracker_share = Path(get_package_share_directory("lawn_mower_tracker"))
    driver_share = Path(get_package_share_directory("can_client"))
    default_params = str(tracker_share / "config" / "tracker.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_share / "launch" / "can_client.launch.py"))
        ),
        Node(
            package="lawn_mower_tracker",
            executable="lawn_mower_tracker_node",
            name="lawn_mower_tracker",
            output="screen",
            parameters=[params_file],
        ),
    ])
