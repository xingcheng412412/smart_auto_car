import sys
import math
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fw_mini_pro_sim.rtk_model import RtkModel


def test_origin_and_driver_north_west_axes():
    m = RtkModel(latitude=30., longitude=120., altitude=50., position_std=0., yaw_std=0.)
    assert m.geodetic(0, 0, 0) == pytest.approx((30, 120, 50), abs=1e-7)
    assert m.geodetic(10, 0, 0)[0] > 30
    assert m.geodetic(0, 10, 0)[1] < 120
    assert m.geodetic(0, 0, 10)[2] == pytest.approx(60, abs=1e-6)


def test_rate_dropout_recovery_and_clock_reset():
    m = RtkModel(rate=10., dropout_start=1., dropout_duration=2.)
    assert m.sample(10., (0,0,0), 0) is not None
    assert m.sample(10.01, (0,0,0), 0) is None
    assert m.sample(11., (0,0,0), 0) is None
    assert m.sample(13., (0,0,0), 0) is not None
    assert m.sample(1., (0,0,0), 0) is not None
    assert m.sample(1.2, (0,0,0), 0, forced_dropout=True) is None


def test_noise_reproducible_and_zero_noise_exact():
    a, b = RtkModel(seed=8), RtkModel(seed=8)
    assert a.sample(0., (1,2,3), .4) == b.sample(0., (1,2,3), .4)
    m = RtkModel(position_std=0., yaw_std=0.)
    assert m.sample(0., (1,2,3), .4) == ((1.,2.,3.), .4)


@pytest.mark.parametrize('kwargs', [{'rate':0}, {'position_std':-1}, {'latitude':91}, {'yaw_std':math.nan}, {'dropout_duration':-1}])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        RtkModel(**kwargs)
