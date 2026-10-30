import sys,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fw_mini_pro_sim.fusion_core import compose,inverse,PlanarFilter,OdomHistory


def test_se2_inverse_and_body_prediction():
    p=np.array([2.,3.,math.pi/2])
    assert np.allclose(compose(p,inverse(p)),0,atol=1e-10)
    f=PlanarFilter(p)
    f.predict([1,0,0],.1)
    assert np.allclose(f.x,[2,4,math.pi/2])


def test_yaw_wrap_outlier_and_covariance():
    f=PlanarFilter([0,0,math.pi-.01])
    assert f.correct([0,0,-math.pi+.01],np.diag([.01,.01,.01]))
    before=f.x.copy()
    assert not f.correct([100,100,0],np.eye(3)*.01)
    assert np.allclose(f.x,before)
    assert np.linalg.eigvalsh(f.p).min()>0


def test_delayed_measurement_transport():
    h=OdomHistory()
    h.add(1,[0,0,0]);h.add(2,[1,0,0]);h.add(3,[2,0,0])
    assert np.allclose(h.at(1.5),[.5,0,0])
    assert np.allclose(h.forward([10,0,0],1.5),[11.5,0,0])
    assert h.at(.5) is None
    h.add(0,[0,0,0])
    assert len(h.samples)==1
