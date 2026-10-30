"""ROS output simulation of RTK; no physical serial driver or TF broadcaster."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import SetBool
from rtk_gps_driver_cpp.msg import RTKGPSLog
from .rtk_model import RtkModel


class RtkSimulator(Node):
    def __init__(self):
        super().__init__('rtk_simulator')
        defaults = dict(latitude=0., longitude=0., altitude=0., rate=10.,
                        position_std=.02, yaw_std=.005, seed=1,
                        dropout_start=-1., dropout_duration=0.)
        params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.model = RtkModel(**params)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.forced_dropout = False
        self.last_wall = None
        self.state = 'WAITING'
        self.last_fix = None
        self.pose_pub = self.create_publisher(PoseStamped, '/gnss_pose', 10)
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.log_pub = self.create_publisher(RTKGPSLog, '/rtk_log', 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth_pose', self.on_pose, qos_profile_sensor_data)
        self.create_service(SetBool, '/sim/rtk/set_dropout', self.set_dropout)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self.publish_log, clock=self.wall_clock)

    def set_dropout(self, request, response):
        self.forced_dropout = request.data
        if request.data:
            self.state = 'TIMEOUT'
        response.success = True
        response.message = 'Simulated RTK dropout ' + ('enabled' if request.data else 'disabled')
        return response

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        components = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in components):
            self.state = 'INVALID_SAMPLE'
            return
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if norm < 1e-9:
            self.state = 'INVALID_SAMPLE'
            return
        x,y,z,w = (v/norm for v in (q.x,q.y,q.z,q.w))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.last_wall = time.monotonic()
        result = self.model.sample(stamp, (p.x,p.y,p.z), yaw, self.forced_dropout)
        if self.model.dropped:
            self.state = 'TIMEOUT'
        if result is None:
            return
        position, noisy_yaw = result
        pose = copy.deepcopy(msg)
        # Rotate original attitude by the added world-Z heading error.
        half = (noisy_yaw-yaw)/2
        s,c = math.sin(half),math.cos(half)
        pose.pose.orientation.x,pose.pose.orientation.y = c*x-s*y,s*x+c*y
        pose.pose.orientation.z,pose.pose.orientation.w = c*z+s*w,c*w-s*z
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = position
        # Map uses the original simulation-world axes, interpreted as NWU.
        pose.header.frame_id = self.map_frame
        fix = NavSatFix()
        fix.header = copy.deepcopy(msg.header)
        fix.header.frame_id = 'base_link'
        fix.latitude, fix.longitude, fix.altitude = self.model.geodetic(*position)
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance = [self.model.position_std**2 if i in (0,4,8) else 0. for i in range(9)]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pose_pub.publish(pose)
        self.fix_pub.publish(fix)
        self.last_fix = fix
        self.state = 'PUBLISHING'

    def publish_log(self):
        if self.last_wall is not None and time.monotonic()-self.last_wall > .5:
            self.state = 'TIMEOUT'
        log = RTKGPSLog()
        log.header.stamp = self.get_clock().now().to_msg()
        log.header.frame_id = 'base_link'
        good = self.state == 'PUBLISHING'
        log.log_level = 'INFO' if good else 'WARN'
        log.gnss_pose_state = self.state
        log.initial_state = 4 if good else 0
        log.ins_status = 15 if good else 0
        log.gps_diff_status = 50 if good else 0
        log.lat_std = log.lon_std = log.h_std = self.model.position_std
        log.yaw_std = math.degrees(self.model.yaw_std)
        log.error_description = 'SIMULATED RTK; last coordinates retained during dropout'
        if not good:
            log.error_type = 'SIM_' + self.state
        if self.last_fix:
            log.latitude,log.longitude,log.altitude = self.last_fix.latitude,self.last_fix.longitude,self.last_fix.altitude
        self.log_pub.publish(log)


def main(args=None):
    rclpy.init(args=args)
    node = RtkSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
