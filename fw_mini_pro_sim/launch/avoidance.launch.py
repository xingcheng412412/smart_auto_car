"""Lidar avoidance simulation; does not start the physical CAN driver."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fw_mini_pro_sim'))
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
                                 '-v', '2', LaunchConfiguration('world')], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': str(share/'config/bridge_slam.yaml')}], output='screen')
    adapter = Node(package='fw_mini_pro_sim', executable='simulation_adapter',
                   parameters=[{'use_sim_time': True}], output='screen')
    tracker = Node(package='lawn_mower_tracker', executable='lawn_mower_tracker_node',
                   name='lawn_mower_tracker', parameters=[str(share/'config/tracker_sim.yaml'),
                     str(share/'config/avoidance_sim.yaml'),
                     {'use_sim_time': True, 'path_file': LaunchConfiguration('path_file')}], output='screen')
    critical = [server, bridge, adapter, tracker]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(share/'worlds/avoidance_ground.sdf')),
        DeclareLaunchArgument('path_file', default_value=str(share/'paths/avoidance.csv')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(share/'models')+os.pathsep+
                               os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        *[RegisterEventHandler(OnProcessExit(target_action=p,
            on_exit=[EmitEvent(event=Shutdown(reason='avoidance simulation process exited'))]))
          for p in critical],
        *critical,
        ExecuteProcess(cmd=['ign', 'gazebo', '-g', '--gui-config', str(share/'config/gui.config')],
                       condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='avoidance_lidar_tf',
             arguments=['--x', '0', '--y', '0', '--z', '0.518976069',
                        '--frame-id', 'base_link', '--child-frame-id', 'fairy_optical'],
             parameters=[{'use_sim_time': True}]),
    ])
