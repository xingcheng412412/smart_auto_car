"""Simulation planar loose-coupled SLAM/RTK fusion with timestamp gates."""
import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rtk_gps_driver_cpp.msg import RTKGPSLog
from tf2_ros import TransformBroadcaster
from .fusion_core import PlanarFilter, OdomHistory, compose, inverse


def seconds(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9


def planar(p):
    q=p.orientation
    values=np.array([p.position.x,p.position.y,q.x,q.y,q.z,q.w])
    if not np.isfinite(values).all():
        raise ValueError('nonfinite pose')
    norm=np.linalg.norm(values[2:])
    if norm<1e-9:
        raise ValueError('invalid quaternion')
    x,y,z,w=values[2:]/norm
    return np.array([p.position.x,p.position.y,math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def fill_pose(p,x):
    p.position.x,p.position.y=float(x[0]),float(x[1])
    p.orientation.z,p.orientation.w=math.sin(x[2]/2),math.cos(x[2]/2)


class FusionNode(Node):
    def __init__(self):
        super().__init__('slam_rtk_fusion')
        for name,default in [('sensor_timeout',2.),('odom_timeout',.5),
                             ('max_delay',1.),('rtk_position_std',.05),
                             ('rtk_yaw_std',.02),('slam_position_std',.10),
                             ('slam_yaw_std',.05)]:
            value=float(self.declare_parameter(name,default).value)
            if not math.isfinite(value) or value<=0:
                raise ValueError(name+' must be finite and positive')
            setattr(self,name,value)
        self.history=OdomHistory()
        self.filter=None
        self.alignment=None
        self.queue=deque(maxlen=200)
        self.last_used={'slam':-math.inf,'rtk':-math.inf}
        self.accepted={}
        self.latest_slam=None
        self.last_odom_wall=-math.inf
        self.rtk_quality=None
        self.state='INITIALIZING'
        self.rejected=0
        self.pose_pub=self.create_publisher(PoseStamped,'/localization/pose',10)
        self.odom_pub=self.create_publisher(Odometry,'/localization/odometry',10)
        self.status_pub=self.create_publisher(String,'/localization/status',10)
        self.tf_pub=TransformBroadcaster(self)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/slam_toolbox/pose',self.on_slam,10)
        self.create_subscription(PoseStamped,'/gnss_pose',self.on_rtk,qos_profile_sensor_data)
        self.create_subscription(RTKGPSLog,'/rtk_log',self.on_quality,10)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.2,self.status_tick,clock=self.wall_clock)

    def on_quality(self,msg):
        good=(msg.gnss_pose_state=='PUBLISHING' and msg.initial_state==4 and
              msg.gps_diff_status in (48,49,50))
        self.rtk_quality=(good,time.monotonic(),seconds(msg.header))

    def on_slam(self,msg):
        if msg.header.frame_id!='map':
            self.rejected+=1;return
        try:
            pose=planar(msg.pose.pose)
            r=np.array(msg.pose.covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
            if not np.isfinite(r).all() or not np.allclose(r,r.T) or np.linalg.eigvalsh(r).min() < -1e-8:
                raise ValueError('invalid SLAM covariance')
            # Conservative floor for shared-odometry correlation and scan matching.
            r+=np.diag([self.slam_position_std**2]*2+[self.slam_yaw_std**2])
            self.queue.append(('slam',seconds(msg.header),pose,r,time.monotonic()))
        except ValueError:
            self.rejected+=1

    def on_rtk(self,msg):
        if msg.header.frame_id!='rtk_map':
            self.rejected+=1;return
        try:
            pose=planar(msg.pose)
            r=np.diag([self.rtk_position_std**2]*2+[self.rtk_yaw_std**2])
            self.queue.append(('rtk',seconds(msg.header),pose,r,time.monotonic()))
        except ValueError:
            self.rejected+=1

    def process(self):
        now=self.history.samples[-1][0]
        pending=list(self.queue);self.queue.clear()
        # Process SLAM first at equal timestamps to allow initial alignment.
        for source,t,pose,r,wall in sorted(pending,key=lambda m:(m[1],m[0]!='slam')):
            if t>now+1e-8:
                if t-now<self.max_delay:
                    self.queue.append((source,t,pose,r,wall))
                else:
                    self.rejected+=1
                continue
            if t<=self.last_used[source] or now-t>self.max_delay or time.monotonic()-wall>self.sensor_timeout:
                self.rejected+=1;continue
            if source=='rtk':
                quality=self.rtk_quality
                if (quality is None or not quality[0] or time.monotonic()-quality[1]>self.sensor_timeout or
                        abs(now-quality[2])>self.sensor_timeout):
                    continue
                if self.alignment is None:
                    if self.latest_slam is None or abs(t-self.latest_slam[0])>.5:
                        continue
                    slam_now=self.history.forward(self.latest_slam[1],self.latest_slam[0])
                    rtk_now=self.history.forward(pose,t)
                    if slam_now is None or rtk_now is None:
                        continue
                    self.alignment=compose(slam_now,inverse(rtk_now))
                pose=compose(self.alignment,pose)
                a=self.alignment[2];c,s=math.cos(a),math.sin(a)
                rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
                r=rotation@r@rotation.T
            transported=self.history.forward(pose,t)
            if transported is None:
                self.rejected+=1;continue
            # Account for forward transport uncertainty (not a full delayed-state EKF).
            r+=np.diag([.02,.02,.01])*(now-t)
            if self.filter is None:
                if source!='slam':
                    continue
                self.filter=PlanarFilter(transported)
                self.filter.p=r.copy()
                accepted=True
            else:
                accepted=self.filter.correct(transported,r)
            self.last_used[source]=t
            if accepted:
                self.accepted[source]=(t,wall)
                if source=='slam':
                    self.latest_slam=(t,pose.copy())
            else:
                self.rejected+=1

    def health(self):
        wall=time.monotonic()
        if wall-self.last_odom_wall>self.odom_timeout:
            return 'ODOM_TIMEOUT'
        if self.filter is None:
            return 'INITIALIZING'
        now=self.history.samples[-1][0]
        fresh={s for s,(t,w) in self.accepted.items()
               if 0<=now-t<=self.sensor_timeout and wall-w<=self.sensor_timeout}
        if self.alignment is None:
            return 'ALIGNING'  # Require RTK+SLAM calibration before enabling control.
        if fresh=={'slam','rtk'}:return 'FUSED'
        if fresh=={'slam'}:return 'SLAM_ONLY'
        if fresh=={'rtk'}:return 'RTK_ONLY'
        return 'LOCALIZATION_LOST'

    def status_tick(self):
        self.state=self.health()
        self.status_pub.publish(String(data=f'{self.state}; rejected={self.rejected}'))

    def on_odom(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
            return
        try:
            pose=planar(msg.pose.pose)
        except ValueError:
            return
        t=seconds(msg.header)
        clock_age=self.get_clock().now().nanoseconds*1e-9-t
        if clock_age>self.odom_timeout or clock_age < -.2:
            self.last_odom_wall=-math.inf
            self.rejected+=1
            return
        previous=self.history.samples[-1] if self.history.samples else None
        if previous and t<previous[0]:
            self.filter=None;self.alignment=None;self.queue.clear();self.accepted.clear()
            self.latest_slam=None;self.rtk_quality=None
            self.last_used={'slam':-math.inf,'rtk':-math.inf}
        elif previous and t==previous[0]:
            return
        elif previous and self.filter is not None:
            self.filter.predict(compose(inverse(previous[1]),pose),t-previous[0])
        self.history.add(t,pose)
        self.last_odom_wall=time.monotonic()
        self.process()
        self.state=self.health()
        if self.filter is None:
            return
        # TF may continue for mapping while control pose is explicitly gated.
        correction=compose(self.filter.x,inverse(pose))
        self.broadcast(msg.header.stamp,'odom',correction)
        if self.alignment is not None:
            self.broadcast(msg.header.stamp,'rtk_map',self.alignment)
        if self.state not in ('FUSED','SLAM_ONLY','RTK_ONLY'):
            return
        out=PoseStamped();out.header.stamp=msg.header.stamp;out.header.frame_id='map'
        fill_pose(out.pose,self.filter.x)
        self.pose_pub.publish(out)
        odom=Odometry();odom.header=out.header;odom.child_frame_id='base_link'
        odom.pose.pose=out.pose
        covariance=np.zeros((6,6));covariance[2,2]=covariance[3,3]=covariance[4,4]=1e6
        covariance[np.ix_([0,1,5],[0,1,5])]=self.filter.p
        odom.pose.covariance=covariance.ravel().tolist()
        odom.twist=msg.twist
        self.odom_pub.publish(odom)

    def broadcast(self,stamp,child,pose):
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='map';tf.child_frame_id=child
        tf.transform.translation.x,tf.transform.translation.y=float(pose[0]),float(pose[1])
        tf.transform.rotation.z,tf.transform.rotation.w=math.sin(pose[2]/2),math.cos(pose[2]/2)
        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args);node=FusionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
