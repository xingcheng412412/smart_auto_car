import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fw_mini_pro_sim.kinematics import filter_command, wheel_targets, WHEEL_POSITIONS


def test_straight_and_reverse_keep_steering_forward():
    for speed in (0.4, -0.4):
        targets = wheel_targets(speed, 0, 0, [0]*4)
        for angle, rate in targets:
            assert angle == pytest.approx(0)
            assert rate * 0.07 == pytest.approx(speed)


@pytest.mark.parametrize('vx,vy,wz', [(0.4,0,0.8), (0,0,1), (0,0.4,0), (-0.4,0,-0.6)])
def test_each_wheel_matches_rigid_body_velocity(vx, vy, wz):
    targets = wheel_targets(vx, vy, wz, [0]*4)
    for (x,y), (angle,rate) in zip(WHEEL_POSITIONS, targets):
        assert 0.07*rate*math.cos(angle) == pytest.approx(vx-wz*y, abs=1e-9)
        assert 0.07*rate*math.sin(angle) == pytest.approx(vy+wz*x, abs=1e-9)


def test_steering_uses_nearest_equivalent_angle():
    targets = wheel_targets(-0.2, 0.01, 0, [6.28]*4)
    for angle, _ in targets:
        assert abs(angle-6.28) < math.pi/2


def test_stop_preserves_steering():
    angles = [0.1,-0.2,3.2,6.3]
    assert wheel_targets(0,0,0,angles) == [(a,0.0) for a in angles]


def test_mode_filter_and_watchdog():
    assert filter_command(True,6,0.3,0.7,0.5,0.1) == (0.3,0,0.5)
    assert filter_command(True,8,0.3,0.7,0.5,0.1) == (0.3,0.7,0)
    for gear in (0,1,2,5,7,99):
        assert filter_command(True,gear,0.3,0,0.5,0) == (0,0,0)
    assert filter_command(False,6,0.3,0,0.5,0) == (0,0,0)
    assert filter_command(True,6,0.3,0,0.5,0.51) == (0,0,0)
    assert filter_command(True,6,0.3,0,0.5,-1) == (0,0,0)


@pytest.mark.parametrize('value', [float('nan'),float('inf'),-float('inf')])
def test_invalid_commands_stop(value):
    assert filter_command(True,6,value,0,0,0) == (0,0,0)


def test_speed_limits_preserve_curvature():
    vx,vy,wz = filter_command(True,6,6,0,6,0)
    assert vx/wz == pytest.approx(1)
    targets = wheel_targets(vx,vy,wz,[0]*4)
    assert max(abs(rate)*0.07 for _,rate in targets) <= 1.5+1e-9
