import sys,time,math
from collections import deque
from pathlib import Path
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from fw_mini_pro_sim.fusion_node import FusionNode
from fw_mini_pro_sim.fusion_core import OdomHistory


def harness():
    n=object.__new__(FusionNode)
    n.sensor_timeout=2.;n.odom_timeout=.5;n.max_delay=1.
    n.rtk_position_std=.05;n.rtk_yaw_std=.02;n.slam_position_std=.1;n.slam_yaw_std=.05
    n.history=OdomHistory();n.filter=None;n.alignment=None;n.queue=deque(maxlen=200)
    n.last_used={'slam':-math.inf,'rtk':-math.inf};n.accepted={};n.latest_slam=None
    n.last_odom_wall=-math.inf;n.rtk_quality=None;n.rejected=0
    n.pose_pub=Mock();n.odom_pub=Mock();n.tf_pub=Mock()
    n.now=0.
    n.get_clock=Mock()
    n.get_clock.return_value.now.side_effect=lambda: type('ClockStamp',(),{'nanoseconds':int(n.now*1e9)})()
    return n


def odom(t,x=0.):
    m=Odometry();m.header.stamp.sec=t;m.header.frame_id='odom';m.child_frame_id='base_link'
    m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.
    return m


def sensors(n,t,x=0.,rtk=True,slam=True):
    if slam:
        m=PoseWithCovarianceStamped();m.header.stamp.sec=t;m.header.frame_id='map'
        m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.;n.on_slam(m)
    if rtk:
        n.rtk_quality=(True,time.monotonic(),float(t))
        m=PoseStamped();m.header.stamp.sec=t;m.header.frame_id='rtk_map'
        m.pose.position.x=x+10.;m.pose.orientation.w=1.;n.on_rtk(m)


def test_alignment_dropout_recovery_and_both_sources_lost():
    n=harness();sensors(n,1);deliver(n,odom(1))
    assert n.state=='FUSED'
    assert np.allclose(n.alignment,[-10,0,0])
    sensors(n,4,1.,rtk=False);deliver(n,odom(4,1.))
    assert n.state=='SLAM_ONLY'
    sensors(n,5,2.);deliver(n,odom(5,2.))
    assert n.state=='FUSED'
    count=n.pose_pub.publish.call_count
    deliver(n,odom(8,2.))
    assert n.state=='LOCALIZATION_LOST'
    assert n.pose_pub.publish.call_count==count
    n.last_odom_wall-=1.
    assert n.health()=='ODOM_TIMEOUT'


def test_rejected_measurement_does_not_refresh_sensor_health():
    n=harness();sensors(n,1);deliver(n,odom(1))
    sensors(n,4,100.,slam=False);deliver(n,odom(4))
    assert n.state=='LOCALIZATION_LOST'
    assert n.last_used['rtk']==4.
    assert n.accepted['rtk'][0]==1.


def test_reset_requires_new_alignment_and_old_future_data_rejected():
    n=harness();sensors(n,10);deliver(n,odom(10))
    deliver(n,odom(1));assert n.alignment is None
    sensors(n,10);deliver(n,odom(2))
    assert n.filter is None
    sensors(n,3);deliver(n,odom(3));assert n.state=='FUSED'


def test_repeated_or_invalid_slam_cannot_keep_localization_alive():
    n=harness();sensors(n,1);deliver(n,odom(1))
    sensors(n,1,slam=True,rtk=False);deliver(n,odom(4))
    assert n.state=='LOCALIZATION_LOST'
    m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.covariance[0]=-1.
    n.on_slam(m);assert not n.queue


def deliver(n,msg):
    n.now=float(msg.header.stamp.sec)+msg.header.stamp.nanosec*1e-9
    n.on_odom(msg)


def test_old_advancing_odom_does_not_refresh_control():
    n=harness();sensors(n,1);deliver(n,odom(1))
    count=n.pose_pub.publish.call_count
    sensors(n,2);n.now=10.
    n.on_odom(odom(2))
    assert n.pose_pub.publish.call_count==count
    assert n.history.samples[-1][0]==1.
