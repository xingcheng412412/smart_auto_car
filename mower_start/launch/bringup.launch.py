"""CAN + RTK + lidar + recorder + obstacle-avoiding tracker; console runs in the calling terminal."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own = Path(get_package_share_directory('mower_start'))
    rtk = Path(get_package_share_directory('rtk_gps_driver_cpp'))
    tracker = Path(get_package_share_directory('lawn_mower_tracker'))
    return LaunchDescription([
        DeclareLaunchArgument('can_iface', default_value='can0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS6'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('gnss_config_file', default_value=str(own / 'param/gnss_init_pose.yaml')),
        DeclareLaunchArgument('path_root', default_value=str(Path.cwd() / 'recorded_files')),
        DeclareLaunchArgument('tracker_params', default_value=str(tracker / 'config/tracker_real_avoidance.yaml')),
        DeclareLaunchArgument('driver_config', default_value=str(tracker / 'config/fairy_avoidance_driver.yaml')),
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='Vehicle backend process exited'))])),
        Node(package='can_client', executable='vehicle_core', name='vehicle_core', output='screen',
             parameters=[{'can_iface': LaunchConfiguration('can_iface'),
                          'command_timeout_sec': 0.25, 'update_rate': 50.0,
                          'max_linear_speed': 0.15, 'max_angular_speed': 0.8}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(rtk / 'launch/rtk_gps_cpp.launch.py')),
            launch_arguments={k: LaunchConfiguration(k) for k in
                              ('serial_port', 'baudrate', 'gnss_config_file')}.items()),
        Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_sdk',
             output='screen', parameters=[{'config_path': LaunchConfiguration('driver_config'),
                                          'use_sim_time': False}]),
        Node(package='lawn_mower_recorder', executable='recorder_enhanced.py',
             name='recorder_enhanced', output='screen',
             parameters=[{'base_dir': LaunchConfiguration('path_root')}]),
        Node(package='lawn_mower_tracker', executable='lawn_mower_tracker_node',
             name='lawn_mower_tracker', output='screen',
             parameters=[LaunchConfiguration('tracker_params'),
                         {'path_file': '', 'path_root': LaunchConfiguration('path_root'),
                          'avoidance_enabled': True, 'use_sim_time': False}]),
    ])
