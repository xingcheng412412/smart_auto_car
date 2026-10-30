from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_ros2_launch_starts_driver_and_tracker():
    launch = (ROOT / "launch" / "tracker.launch.py").read_text()
    assert "can_client.launch.py" in launch
    assert "lawn_mower_tracker_node" in launch
    assert "params_file" in launch


def test_default_parameters_are_safe_and_portable():
    config = (ROOT / "config" / "tracker.yaml").read_text()
    for key in (
        "odom_topic", "vehicle_cmd_topic", "path_status_topic", "control_service",
        "wheelbase", "control_rate_hz", "rtk_timeout_sec", "max_angular_speed",
        "search_back_points", "search_forward_points", "steering_gain",
        "max_slope_tangent", "direction_switch_distance", "log_enabled",
        "log_directory",
        "initial_search_distance", "initial_search_points", "max_start_distance",
        "csv_yaw_column", "terminal_alignment_distance", "terminal_yaw_tolerance",
    ):
        assert key in config
    assert "/home/" not in config
    assert 'path_file: ""' in config
    assert "initial_search_distance: 10.0" in config
    assert 'csv_yaw_column: "yaw"' in config
    assert "terminal_alignment_distance: 3.0" in config
    assert "terminal_yaw_tolerance: 0.174532925" in config


def test_real_avoidance_configuration_matches_mount_and_raw_lidar():
    import yaml
    p = yaml.safe_load((ROOT / 'config/tracker_real_avoidance.yaml').read_text())['lawn_mower_tracker']['ros__parameters']
    assert p['avoidance_enabled'] is True
    assert p['use_sim_time'] is False
    assert p['wheelbase'] == 0.36
    assert p['obstacle_cloud_frame'] == 'rslidar'
    assert p['lidar_origin_height'] == 0.483976069
    assert p['avoidance_max_speed'] <= 0.15
    assert p['avoidance_braking_deceleration'] > 0
    launch = (ROOT / 'launch/avoidance.launch.py').read_text()
    assert 'vehicle_core' in launch and 'rslidar_sdk_node' in launch
    assert 'fw_mini_pro_sim' not in launch and 'gazebo' not in launch
    driver = yaml.safe_load((ROOT / 'config/fairy_avoidance_driver.yaml').read_text())
    assert driver['lidar'][0]['driver']['use_lidar_clock'] is False
    assert driver['lidar'][0]['ros']['ros_frame_id'] == p['obstacle_cloud_frame']
