import runpy
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[3]


def test_rtk_reads_plain_origin_with_numeric_types(tmp_path):
    origin = tmp_path / 'origin.yaml'
    origin.write_text('lat: 31.2\nlon: 121.4\nh: 8.0\n')
    module = runpy.run_path(str(ROOT / 'src/dy_rtk_gps_driver/launch/rtk_gps_cpp.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update(gnss_config_file=str(origin), serial_port='/dev/test',
        baudrate='460800', frame_id='gps', log_interval='0.5', pose_publish_rate='50.0',
        pose_timeout='0.5', max_pose_speed='10.0', map_frame='map', pose_frame='gps')
    nodes = module['launch_setup'](context)
    params = evaluate_parameters(context, nodes[0]._Node__parameters)
    merged = {k: v for item in params for k, v in item.items()}
    assert merged['lat'] == 31.2
    assert merged['lon'] == 121.4
    assert type(merged['baudrate']) is int
    assert type(merged['pose_timeout']) is float
    origin.write_text('lat: 100\nlon: 121\nh: 8\n')
    with pytest.raises(ValueError):
        module['launch_setup'](context)


def test_bringup_contains_one_of_each_backend(monkeypatch):
    import ament_index_python.packages
    shares = {"mower_start": "mower_start", "rtk_gps_driver_cpp": "dy_rtk_gps_driver", "lawn_mower_tracker": "lawn_mower_tracker"}
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory",
                        lambda name: str(ROOT / "src" / shares[name]))
    module = runpy.run_path(str(ROOT / 'src/mower_start/launch/bringup.launch.py'))
    description = module['generate_launch_description']()
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert sorted(n.node_executable for n in nodes) == sorted([
        'vehicle_core', 'recorder_enhanced.py', 'lawn_mower_tracker_node', 'rslidar_sdk_node'])
    assert len([e for e in description.entities if isinstance(e, RegisterEventHandler)]) == 1

    assert len([e for e in description.entities if isinstance(e, IncludeLaunchDescription)]) == 1

    from launch.actions import DeclareLaunchArgument
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    tracker_node = next(n for n in nodes if n.node_executable == 'lawn_mower_tracker_node')
    tracker_params = evaluate_parameters(context, tracker_node._Node__parameters)
    assert tracker_params[0].name == 'tracker_real_avoidance.yaml'
    assert tracker_params[1]['avoidance_enabled'] is True
    assert tracker_params[1]['path_root'] == context.launch_configurations['path_root']
    can = next(n for n in nodes if n.node_executable == 'vehicle_core')
    can_params = evaluate_parameters(context, can._Node__parameters)[0]
    assert can_params['max_linear_speed'] == 0.15
    assert can_params['max_angular_speed'] == 0.8
    lidar = next(n for n in nodes if n.node_executable == 'rslidar_sdk_node')
    lidar_params = evaluate_parameters(context, lidar._Node__parameters)[0]
    assert Path(lidar_params['config_path']).name == 'fairy_avoidance_driver.yaml'
    assert lidar_params['use_sim_time'] is False
