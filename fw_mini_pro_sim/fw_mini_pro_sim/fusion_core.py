"""Planar EKF and timestamped SE(2) odometry transport, independent of ROS."""
import math
from collections import deque
import numpy as np


def wrap(a):
    return math.remainder(float(a),2*math.pi)


def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2])])


def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2])
    return np.array([-c*a[0]-s*a[1],s*a[0]-c*a[1],wrap(-a[2])])


class PlanarFilter:
    def __init__(self, pose):
        self.x=np.asarray(pose,dtype=float).copy()
        self.p=np.diag([.1,.1,.05])

    def predict(self,delta,dt):
        theta=self.x[2];c,s=math.cos(theta),math.sin(theta)
        f=np.eye(3)
        f[0,2]=-s*delta[0]-c*delta[1]
        f[1,2]=c*delta[0]-s*delta[1]
        self.x=compose(self.x,delta)
        self.p=f@self.p@f.T+np.diag([.01,.01,.005])*max(dt,0.)

    def correct(self,measurement,covariance,gate=16.27):
        z=np.asarray(measurement,dtype=float);r=np.asarray(covariance,dtype=float)
        if z.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(z).all() or not np.isfinite(r).all():
            return False
        if not np.allclose(r,r.T) or np.linalg.eigvalsh(r).min()<=0:
            return False
        innovation=z-self.x;innovation[2]=wrap(innovation[2])
        total=self.p+r
        if innovation@np.linalg.solve(total,innovation)>gate:
            return False
        k=np.linalg.solve(total,self.p.T).T
        self.x+=k@innovation;self.x[2]=wrap(self.x[2])
        a=np.eye(3)-k
        self.p=a@self.p@a.T+k@r@k.T
        self.p=(self.p+self.p.T)/2
        return True


class OdomHistory:
    def __init__(self):
        self.samples=deque()

    def add(self,t,pose):
        if self.samples and t<self.samples[-1][0]:
            self.samples.clear()
        if self.samples and t==self.samples[-1][0]:
            return
        self.samples.append((t,np.asarray(pose,dtype=float)))
        while len(self.samples)>2 and self.samples[1][0]<t-5.:
            self.samples.popleft()

    def at(self,t):
        if not self.samples or t<self.samples[0][0]-1e-8 or t>self.samples[-1][0]+1e-8:
            return None
        for (ta,a),(tb,b) in zip(self.samples,list(self.samples)[1:]):
            if ta<=t<=tb:
                weight=(t-ta)/(tb-ta)
                result=a+(b-a)*weight
                result[2]=wrap(a[2]+wrap(b[2]-a[2])*weight)
                return result
        return self.samples[-1][1].copy()

    def forward(self,pose,t):
        old=self.at(t)
        if old is None:
            return None
        return compose(pose,compose(inverse(old),self.samples[-1][1]))
