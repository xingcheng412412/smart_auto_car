from launch import LaunchDescription
import math
from pathlib import Path
import yaml

from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent
from launch.events import Shutdown
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    config_path = Path(LaunchConfiguration('gnss_config_file').perform(context)).expanduser().resolve()
    with config_path.open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('GNSS origin must contain lat/lon/h')
    origin = {key: float(data[key]) for key in ('lat', 'lon', 'h')}
    if (not all(math.isfinite(v) for v in origin.values())
            or not -90 <= origin['lat'] <= 90 or not -180 <= origin['lon'] <= 180):
        raise ValueError('Invalid GNSS origin latitude/longitude/height')
    rtk_gps_node = Node(
        package='rtk_gps_driver_cpp',
        executable='rtk_gps_node_cpp',
        name='rtk_gps_node',
        output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='RTK driver exited'))],
        parameters=[
            origin,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'frame_id': LaunchConfiguration('frame_id'),
                'log_interval': ParameterValue(LaunchConfiguration('log_interval'), value_type=float),
                'pose_publish_rate': ParameterValue(LaunchConfiguration('pose_publish_rate'), value_type=float),
                'pose_timeout': ParameterValue(LaunchConfiguration('pose_timeout'), value_type=float),
                'max_pose_speed': ParameterValue(LaunchConfiguration('max_pose_speed'), value_type=float),
                'map_frame': LaunchConfiguration('map_frame'),
                'pose_frame': LaunchConfiguration('pose_frame'),
                'gnss_config_file': str(config_path),
            }
        ],
        remappings=[
            ('fix', 'fix'),
            ('nmea_sentence', 'nmea_sentence'),
            ('imu_raw', 'imu_raw'),
            ('velocity_enu', 'velocity_enu'),
            ('rtk_log', 'rtk_log'),
            ('gnss_pose', 'gnss_pose'),
        ],
    )

    return [rtk_gps_node]


def generate_launch_description():
    # Declare launch arguments
    gnss_config_file_arg = DeclareLaunchArgument(
        'gnss_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mower_start'), 'param', 'gnss_init_pose.yaml'
        ]),
        description='Pose origin config file containing lat, lon and h'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS6',
        description='Serial port device path'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='460800',
        description='Serial port baudrate'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='Frame ID for GPS messages'
    )

    log_interval_arg = DeclareLaunchArgument(
        'log_interval',
        default_value='0.5',
        description='Log publishing interval in seconds'
    )

    pose_publish_rate_arg = DeclareLaunchArgument(
        'pose_publish_rate',
        default_value='50.0',
        description='Pose publishing rate in Hz'
    )

    pose_timeout_arg = DeclareLaunchArgument(
        'pose_timeout',
        default_value='0.5',
        description='Pose timeout in seconds'
    )

    max_pose_speed_arg = DeclareLaunchArgument(
        'max_pose_speed',
        default_value='10.0',
        description='Maximum pose speed in m/s'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )

    pose_frame_arg = DeclareLaunchArgument(
        'pose_frame',
        default_value='gps',
        description='Pose frame ID'
    )

    return LaunchDescription([
        gnss_config_file_arg,
        serial_port_arg,
        baudrate_arg,
        frame_id_arg,
        log_interval_arg,
        pose_publish_rate_arg,
        pose_timeout_arg,
        max_pose_speed_arg,
        map_frame_arg,
        pose_frame_arg,
        OpaqueFunction(function=launch_setup),
    ])
