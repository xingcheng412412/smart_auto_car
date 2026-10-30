"""Launch file for the RTKGPSLog web viewer node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8080",
        description="HTTP/WebSocket 服务端口",
    )
    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="/rtk_log",
        description="RTKGPSLog topic 名称",
    )

    viewer_node = Node(
        package="robinx_log",
        executable="rtkgpslog_web_viewer",
        name="rtkgpslog_web_viewer",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "topic": LaunchConfiguration("topic"),
            }
        ],
    )

    return LaunchDescription([port_arg, topic_arg, viewer_node])
