"""Real RSFAIRY -> timed point adapter -> FAST-LIO2 -> voxel map."""
import math
from datetime import datetime
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('fw_mini_pro_slam3d'))
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['/**']['ros__parameters']
    extrinsic = params['mapping']
    r = extrinsic['extrinsic_R']
    t = extrinsic['extrinsic_T']
    if len(r) != 9 or len(t) != 3 or not all(math.isfinite(x) for x in r + t):
        raise ValueError('extrinsic_R/T must contain 9/3 finite values')
    for i in range(3):
        for j in range(3):
            dot = sum(r[3*k+i]*r[3*k+j] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-5:
                raise ValueError('extrinsic_R must be orthonormal')
    determinant = (r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6])
                   + r[2]*(r[3]*r[7]-r[4]*r[6]))
    if abs(determinant-1.0) > 1e-5:
        raise ValueError('extrinsic_R must be a proper rotation')
    # ZYX Euler decomposition, including the pitch singularity.
    pitch = math.asin(max(-1.0, min(1.0, -r[6])))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(r[7], r[8]), math.atan2(r[3], r[0])
    else:
        roll, yaw = math.atan2(-r[5], r[4]), 0.0
    map_file = str(Path(LaunchConfiguration('map_file').perform(context)).expanduser().resolve())
    driver = Node(package='rslidar_sdk', executable='rslidar_sdk_node',
                  condition=IfCondition(LaunchConfiguration('with_driver')), output='screen',
                  parameters=[{'config_path': LaunchConfiguration('driver_config'), 'use_sim_time': False}])
    adapter = Node(package='fw_mini_pro_slam3d', executable='fairy_adapter', output='screen',
                   parameters=[{'use_sim_time': False, 'scan_lines': int(params['preprocess']['scan_line'])}])
    mapping = Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
                   parameters=[params_file, {'use_sim_time': False}],
                   remappings=[('/cloud_registered', '/slam3d/registered'),
                               ('/cloud_registered_body', '/slam3d/registered_body'),
                               ('/Odometry', '/slam3d/odometry'), ('/path', '/slam3d/path'),
                               ('map_save', '/slam3d/upstream_map_save_disabled')])
    accumulator = Node(package='fw_mini_pro_slam3d', executable='fairy_map', output='screen',
                       parameters=[{'map_file_path': map_file, 'use_sim_time': False,
                                    'voxel_size': 0.15, 'max_voxels': 1000000}])
    lidar_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='fairy_lidar_tf',
                    arguments=['--x', str(t[0]), '--y', str(t[1]), '--z', str(t[2]),
                               '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                               '--frame-id', 'body', '--child-frame-id', 'rslidar'])
    imu_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='fairy_imu_tf',
                  arguments=['--frame-id', 'body', '--child-frame-id', 'fairy_imu'])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(share/'rviz/fairy.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')))
    critical = [driver, adapter, mapping, accumulator]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='A mapping process exited'))])) for node in critical]
    return handlers + critical + [lidar_tf, imu_tf, rviz]


def generate_launch_description():
    share = Path(get_package_share_directory('fw_mini_pro_slam3d'))
    default_map = Path.cwd()/'maps'/('fairy_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.pcd')
    return LaunchDescription([
        DeclareLaunchArgument('with_driver', default_value='true', description='Start the physical FAIRY driver'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/fairy_fastlio.yaml')),
        DeclareLaunchArgument('driver_config', default_value=str(share/'config/fairy_driver.yaml')),
        DeclareLaunchArgument('map_file', default_value=str(default_map), description='PCD output for /slam3d/save_map'),
        OpaqueFunction(function=setup),
    ])
