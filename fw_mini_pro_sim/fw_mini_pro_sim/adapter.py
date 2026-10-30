"""ROS VehicleCmd ↔ Gazebo joint targets and measured localization."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from can_client.msg import VehicleCmd, VehicleFeedback

from .kinematics import WHEEL_NAMES, filter_command, wheel_targets


class SimulationAdapter(Node):
    def __init__(self):
        super().__init__('fw_mini_pro_sim_adapter')
        self.timeout = float(self.declare_parameter('command_timeout',0.5).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('command_timeout must be finite and positive')
        self.command = None
        self.last_command_wall = -math.inf
        self.last_command_sim = -math.inf
        self.last_joint_wall = -math.inf
        self.last_odom_stamp = None
        self.angles = [0.0]*4
        self.steer_pubs = [self.create_publisher(Float64,f'/sim/{n}/steer',1) for n in WHEEL_NAMES]
        self.wheel_pubs = [self.create_publisher(Float64,f'/sim/{n}/wheel',1) for n in WHEEL_NAMES]
        self.pose_pub = self.create_publisher(PoseStamped,'/gnss_pose',10)
        self.odom_pub = self.create_publisher(Odometry,'/odom',10)
        self.feedback_pub = self.create_publisher(VehicleFeedback,'/vehicle_feedback',10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.pose_frame = str(self.declare_parameter('pose_frame', 'map').value)
        if self.declare_parameter('publish_map_tf', True).value:
            self.static_tf_pub.sendTransform(transform)
        self.create_subscription(VehicleCmd,'/vehicle_cmd',self.on_command,1)
        self.create_subscription(Odometry,'/sim/odometry',self.on_odometry,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
        # Wall timer keeps sending zero when /clock pauses; simulation age also
        # expires stale commands when the simulator runs faster than real time.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02,self.update,clock=self.wall_clock)
        self.get_logger().info('Simulation adapter ready; waiting for measured joint states')

    def on_command(self,msg):
        self.command = msg
        self.last_command_wall = time.monotonic()
        self.last_command_sim = self.get_clock().now().nanoseconds*1e-9

    def on_joints(self,msg):
        positions = dict(zip(msg.name,msg.position))
        values = [positions.get(n+'_steer_joint') for n in WHEEL_NAMES]
        if all(v is not None and math.isfinite(v) for v in values):
            self.angles = values
            self.last_joint_wall = time.monotonic()

    def command_age(self):
        sim_age = self.get_clock().now().nanoseconds*1e-9-self.last_command_sim
        if sim_age < 0:
            return -1.0
        return max(time.monotonic()-self.last_command_wall,sim_age)

    def update(self):
        cmd = self.command
        twist = (0.0,0.0,0.0)
        if cmd is not None and time.monotonic()-self.last_joint_wall <= self.timeout:
            twist = filter_command(cmd.enable,cmd.gear,cmd.linear_x,cmd.linear_y,
                                   cmd.angular_z,self.command_age(),self.timeout)
        targets = wheel_targets(*twist,self.angles)
        # Steer before applying traction when a new command needs a large turn.
        error = max(abs(math.remainder(a-current,2*math.pi))
                    for (a,_),current in zip(targets,self.angles))
        alignment = max(0.0, math.cos(error)) if error < 0.5 else 0.0
        for (angle,rate),steer,wheel in zip(targets,self.steer_pubs,self.wheel_pubs):
            steer.publish(Float64(data=angle))
            wheel.publish(Float64(data=rate*alignment))

    def on_odometry(self,msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if self.last_odom_stamp is not None and stamp < self.last_odom_stamp:
            self.command = None
        self.last_odom_stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.pose_frame
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_pub.sendTransform(transform)
        feedback = VehicleFeedback()
        feedback.header = msg.header
        cmd = self.command
        age = self.command_age()
        if (cmd is not None and cmd.enable and cmd.gear in (6,8) and
                0 <= age <= self.timeout and
                all(math.isfinite(v) for v in (cmd.linear_x,cmd.linear_y,cmd.angular_z))):
            feedback.gear = cmd.gear
        feedback.linear_x = msg.twist.twist.linear.x
        feedback.linear_y = msg.twist.twist.linear.y
        feedback.angular_z = msg.twist.twist.angular.z
        # Battery fields are illustrative constants, not a battery simulation.
        feedback.battery_voltage = 24.0
        feedback.battery_percent = 100.0
        self.feedback_pub.publish(feedback)

    def stop(self):
        self.command = None
        for publisher in self.wheel_pubs:
            publisher.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = SimulationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
