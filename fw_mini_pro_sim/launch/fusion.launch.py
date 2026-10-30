"""Online mapping with RTK/SLAM planar fusion and optional path tracker."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('fw_mini_pro_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('tracker',default_value='false'),
        DeclareLaunchArgument('path_file',default_value=str(share/'paths/straight.csv')),
        DeclareLaunchArgument('rtk_params',default_value=str(share/'config/rtk_sim.yaml')),
        DeclareLaunchArgument('fusion_params',default_value=str(share/'config/fusion.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/slam.launch.py')),
            launch_arguments={'gui':LaunchConfiguration('gui'),'rviz':LaunchConfiguration('rviz'),
                              'rtk':'true','fusion':'true','rtk_params':LaunchConfiguration('rtk_params')}.items()),
        Node(package='fw_mini_pro_sim',executable='slam_rtk_fusion',
             parameters=[LaunchConfiguration('fusion_params'),{'use_sim_time':True}],output='screen'),
        Node(package='lawn_mower_tracker',executable='lawn_mower_tracker_node',
             name='lawn_mower_tracker',condition=IfCondition(LaunchConfiguration('tracker')),
             parameters=[str(share/'config/tracker_sim.yaml'),{'use_sim_time':True,
                         'path_file':LaunchConfiguration('path_file'),'odom_topic':'/localization/pose'}],output='screen'),
    ])
