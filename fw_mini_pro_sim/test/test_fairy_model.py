"""Check physical mounting and lidar coverage in the generated model."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def test_mount_and_sensor():
    model = ET.parse(ROOT/'models/fw_mini_pro_fairy/model.sdf').getroot().find('model')
    link = model.find("link[@name='rslidar']")
    assert abs(float(link.findtext('pose').split()[2]) - .483976069) < 1e-8
    joint = model.find("joint[@name='lidar_mount_joint']")
    assert joint.attrib['type'] == 'fixed'
    assert joint.findtext('parent') == 'base_link'
    sensor = link.find('sensor')
    assert sensor.attrib['type'] == 'gpu_lidar'
    assert float(sensor.findtext('update_rate')) == 10
    scan = sensor.find('lidar/scan')
    assert int(scan.findtext('vertical/samples')) == 96
    assert abs(float(scan.findtext('vertical/min_angle')) - math.radians(-16)) < 1e-8
    assert abs(float(scan.findtext('horizontal/max_angle')) - math.pi) < 1e-8

def test_world_has_mapping_features():
    world = ET.parse(ROOT/'worlds/slam_ground.sdf').getroot().find('world')
    assert len(world.findall('model')) >= 7
    assert world.find("plugin[@name='ignition::gazebo::systems::Sensors']") is not None
