"""Real RSFAIRY + SocketCAN + RTK path tracker with local obstacle avoidance."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('lawn_mower_tracker'))
    can = Node(package='can_client', executable='vehicle_core', name='vehicle_core',
               condition=IfCondition(LaunchConfiguration('with_can')), output='screen',
               parameters=[{'use_sim_time': False, 'can_iface': LaunchConfiguration('can_iface'),
                 'update_rate': 50.0, 'max_linear_speed': 0.15, 'max_angular_speed': 0.8,
                 'command_timeout_sec': ParameterValue(LaunchConfiguration('command_timeout_sec'),
                                                       value_type=float)}])
    lidar = Node(package='rslidar_sdk', executable='rslidar_sdk_node', output='screen',
                 condition=IfCondition(LaunchConfiguration('with_lidar')),
                 parameters=[{'config_path': LaunchConfiguration('driver_config'),
                              'use_sim_time': False}])
    tracker = Node(package='lawn_mower_tracker', executable='lawn_mower_tracker_node',
                   name='lawn_mower_tracker', output='screen',
                   parameters=[LaunchConfiguration('params_file'),
                     {'use_sim_time': False, 'path_file': LaunchConfiguration('path_file'),
                      'odom_topic': LaunchConfiguration('odom_topic')}])
    # Tracker input frame is fixed by the raw driver configuration. No TF lookup is used.
    critical = [can, lidar, tracker]
    return LaunchDescription([
        DeclareLaunchArgument('with_can', default_value='true'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('can_iface', default_value='can0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.25'),
        DeclareLaunchArgument('path_file', default_value=''),
        DeclareLaunchArgument('odom_topic', default_value='/gnss_pose'),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/tracker_real_avoidance.yaml')),
        DeclareLaunchArgument('driver_config', default_value=str(share/'config/fairy_avoidance_driver.yaml')),
        *[RegisterEventHandler(OnProcessExit(target_action=p,
            on_exit=[EmitEvent(event=Shutdown(reason='avoidance process exited'))])) for p in critical],
        *critical,
    ])
