"""Exercise real ROS message callbacks without starting transport."""
import sys
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nav_msgs.msg import Odometry
from can_client.msg import VehicleCmd
from fw_mini_pro_sim.adapter import SimulationAdapter


def harness():
    node = object.__new__(SimulationAdapter)
    node.pose_frame = 'map'
    node.pose_pub = Mock()
    node.odom_pub = Mock()
    node.feedback_pub = Mock()
    node.tf_pub = Mock()
    node.command = VehicleCmd(enable=True,gear=6,linear_x=0.5)
    node.last_command_wall = 0.0
    node.last_command_sim = 0.0
    node.timeout = 0.5
    node.last_odom_stamp = None
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
    return node


def test_localization_uses_measured_pose_and_feedback_velocity():
    node = harness()
    msg = Odometry()
    msg.header.stamp.sec = 2
    msg.pose.pose.position.x = 3.25
    msg.pose.pose.position.y = -1.5
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.17
    msg.twist.twist.angular.z = -0.2
    node.on_odometry(msg)
    pose = node.pose_pub.publish.call_args.args[0]
    feedback = node.feedback_pub.publish.call_args.args[0]
    assert pose.header.frame_id == 'map'
    assert pose.pose.position.x == 3.25
    assert pose.pose.position.y == -1.5
    assert abs(feedback.linear_x-0.17) < 1e-6
    assert abs(feedback.angular_z+0.2) < 1e-6
    assert feedback.gear == 0  # old command is expired
    assert node.tf_pub.sendTransform.called


def test_time_reset_clears_previous_command():
    node = harness()
    node.last_odom_stamp = 3_000_000_000
    msg = Odometry()
    msg.header.stamp.sec = 1
    msg.pose.pose.orientation.w = 1.0
    node.on_odometry(msg)
    assert node.command is None


def test_actuator_watchdog_stops_all_wheels():
    import time
    node = harness()
    node.angles = [0.0]*4
    node.steer_pubs = [Mock() for _ in range(4)]
    node.wheel_pubs = [Mock() for _ in range(4)]
    node.last_joint_wall = time.monotonic()
    node.last_command_wall = time.monotonic()
    node.last_command_sim = 2.0
    node.update()
    assert all(p.publish.call_args.args[0].data > 0 for p in node.wheel_pubs)
    node.last_command_wall -= 1.0
    node.update()
    assert all(p.publish.call_args.args[0].data == 0 for p in node.wheel_pubs)


def test_missing_joint_feedback_stops_traction():
    import time
    node = harness()
    node.angles = [0.0]*4
    node.steer_pubs = [Mock() for _ in range(4)]
    node.wheel_pubs = [Mock() for _ in range(4)]
    node.last_joint_wall = 0.0
    node.last_command_wall = time.monotonic()
    node.last_command_sim = 2.0
    node.update()
    assert all(p.publish.call_args.args[0].data == 0 for p in node.wheel_pubs)


def test_crab_turns_wheels_before_traction():
    import math
    import time
    node = harness()
    node.angles = [0.0]*4
    node.steer_pubs = [Mock() for _ in range(4)]
    node.wheel_pubs = [Mock() for _ in range(4)]
    node.command = VehicleCmd(enable=True,gear=8,linear_y=0.3)
    node.last_joint_wall = time.monotonic()
    node.last_command_wall = time.monotonic()
    node.last_command_sim = 2.0
    node.update()
    assert all(abs(p.publish.call_args.args[0].data-math.pi/2)<1e-6 for p in node.steer_pubs)
    assert all(p.publish.call_args.args[0].data == 0 for p in node.wheel_pubs)


def test_slam_truth_pose_keeps_odom_frame():
    node = harness()
    node.pose_frame = 'odom'
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    node.on_odometry(msg)
    assert node.pose_pub.publish.call_args.args[0].header.frame_id == 'odom'
