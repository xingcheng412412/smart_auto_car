"""Test real message callbacks, including no stale pose after source loss."""
import sys
import time
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from fw_mini_pro_sim.rtk_model import RtkModel
from fw_mini_pro_sim.rtk_simulator import RtkSimulator


def harness():
    n = object.__new__(RtkSimulator)
    n.model = RtkModel(position_std=0.,yaw_std=0.)
    n.map_frame='map'
    n.forced_dropout=False
    n.last_wall=n.last_fix=None
    n.state='WAITING'
    n.pose_pub,n.fix_pub,n.log_pub=Mock(),Mock(),Mock()
    n.get_clock=Mock()
    from builtin_interfaces.msg import Time
    n.get_clock.return_value.now.return_value.to_msg.return_value=Time()
    return n


def pose(t):
    m=PoseStamped()
    m.header.stamp.sec=t
    m.pose.orientation.w=1.
    m.pose.position.x=10.
    return m


def test_outputs_dropout_recovery_and_silence():
    n=harness()
    n.on_pose(pose(1))
    assert n.fix_pub.publish.call_args.args[0].latitude > 0
    assert n.pose_pub.publish.call_args.args[0].pose.position.x == 10.
    assert n.pose_pub.publish.call_args.args[0].header.stamp.sec == 1
    n.publish_log()
    assert n.log_pub.publish.call_args.args[0].gps_diff_status == 50
    n.set_dropout(SetBool.Request(data=True),SetBool.Response())
    n.on_pose(pose(2))
    assert n.pose_pub.publish.call_count == 1
    assert n.fix_pub.publish.call_count == 1
    n.publish_log()
    assert n.log_pub.publish.call_args.args[0].gnss_pose_state == 'TIMEOUT'
    n.set_dropout(SetBool.Request(data=False),SetBool.Response())
    n.on_pose(pose(3))
    assert n.pose_pub.publish.call_count == 2
    n.last_wall=time.monotonic()-1
    n.publish_log()
    assert n.log_pub.publish.call_args.args[0].gnss_pose_state == 'TIMEOUT'
    assert n.pose_pub.publish.call_count == 2


def test_invalid_attitude_is_not_published():
    n=harness()
    msg = PoseStamped()
    msg.pose.orientation.w = 0.
    n.on_pose(msg)
    assert n.state == 'INVALID_SAMPLE'
    assert not n.pose_pub.publish.called
