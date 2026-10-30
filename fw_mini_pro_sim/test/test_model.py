from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_physical_model():
    model = ET.parse(ROOT/'models/fw_mini_pro/model.sdf').getroot().find('model')
    assert len(model.findall('joint')) == 8
    assert sum(float(x.text) for x in model.findall('link/inertial/mass')) == pytest.approx(30)
    for link in model.findall('link'):
        inertia = link.find('inertial/inertia')
        assert all(float(inertia.findtext(k)) > 0 for k in ('ixx','iyy','izz'))
    for name,x,y in [('front_left',.18,.14),('front_right',.18,-.14),('rear_left',-.18,.14),('rear_right',-.18,-.14)]:
        wheel = model.find(f"link[@name='{name}_wheel']")
        assert list(map(float,wheel.findtext('pose').split()[:2])) == [x,y]
        assert float(wheel.findtext('collision/geometry/cylinder/radius')) == .07
        assert model.find(f"joint[@name='{name}_steer_joint']/axis/xyz").text == '0 0 1'
        assert model.find(f"joint[@name='{name}_wheel_joint']/axis/xyz").text == '0 1 0'
    names = [p.attrib['name'] for p in model.findall('plugin')]
    assert names.count('ignition::gazebo::systems::JointController') == 4
    assert names.count('ignition::gazebo::systems::JointPositionController') == 4
    assert 'ignition::gazebo::systems::OdometryPublisher' in names
