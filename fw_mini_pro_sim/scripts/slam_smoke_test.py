#!/usr/bin/env python3
"""Verify an already-running isolated slam.launch.py by driving a small circle."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from can_client.msg import VehicleCmd
from tf2_msgs.msg import TFMessage


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=Path('/tmp/fairy_slam_validation'));args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('fairy_slam_acceptance')
    state={'clouds':0,'scans':0,'maps':0,'pose':None,'yaw_sum':0.,'trajectory':[],'cloud_times':[],'map_hashes':set(),'tf_edges':set(),'tf_sources':{},'static_edges':set()}
    def cloud(m):
        state['clouds']+=1;state['last_cloud']=time.monotonic()
        state['cloud_times'].append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9))
        state['cloud_frame']=m.header.frame_id
        state['cloud_size']=[m.height,m.width]
        offset=next(f.offset for f in m.fields if f.name=='x')
        x=np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,offset=offset,strides=(m.row_step,m.point_step))
        state['finite_points']=int(np.isfinite(x).sum())
    def scan(m):
        state['scans']+=1;state['last_scan']=time.monotonic();state['finite_rays']=sum(math.isfinite(v) for v in m.ranges)
    def mapping(m):
        state['maps']+=1;state['last_map']=time.monotonic();state['map_hashes'].add(hash(bytes((x+1 for x in m.data))))
        state['occupied']=sum(v>50 for v in m.data);state['free']=sum(v==0 for v in m.data)
        state['map_size']=[m.info.width,m.info.height]
    def odom(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if state['pose'] is not None:state['yaw_sum']+=math.remainder(yaw-state['pose'][2],2*math.pi)
        state['pose']=(p.x,p.y,yaw);state['trajectory'].append([p.x,p.y])
    def tf(m):
        for t in m.transforms:
            edge=(t.header.frame_id,t.child_frame_id)
            state['tf_edges'].add(edge)

    def static_tf(m):
        for t in m.transforms:
            edge=(t.header.frame_id,t.child_frame_id)
            state['tf_edges'].add(edge);state['static_edges'].add(edge)
    node.create_subscription(PointCloud2,'/rslidar_points',cloud,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/map',mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf',tf,50)
    node.create_subscription(TFMessage,'/tf_static',static_tf,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub=node.create_publisher(VehicleCmd,'/vehicle_cmd',1)
    try:
        deadline=time.monotonic()+40
        while time.monotonic()<deadline and (not state['maps'] or not state['clouds'] or state['pose'] is None):rclpy.spin_once(node,timeout_sec=.05)
        assert state['maps'] and state['clouds'] and state['pose'] is not None,'missing map/cloud/odom'
        counts=(state['clouds'],state['scans'],state['maps'])
        state['map_hashes'].clear()
        start=state['pose'];state['trajectory']=[];yaw0=state['yaw_sum'];deadline=time.monotonic()+100;last_report=0
        while state['yaw_sum']-yaw0 < 2*math.pi:
            assert time.monotonic()<deadline,'circle motion timed out'
            pub.publish(VehicleCmd(enable=True,gear=6,linear_x=.35,angular_z=.3))
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_report>10:
                print('progress',state['pose'],'yaw',state['yaw_sum']-yaw0,'maps',state['maps'],flush=True);last_report=time.monotonic()
        for _ in range(60):pub.publish(VehicleCmd(enable=False));rclpy.spin_once(node,timeout_sec=.03)
        assert state['clouds']-counts[0]>10 and state['scans']-counts[1]>10 and state['maps']-counts[2]>3
        assert all(time.monotonic()-state[key]<3 for key in ('last_cloud','last_scan','last_map'))
        assert state['cloud_frame']=='fairy_optical',state['cloud_frame']
        assert state['finite_points']>1000 and state['finite_rays']>100
        assert state['occupied']>100 and state['free']>100 and len(state['map_hashes'])>1
        for edge in [('map','odom'),('odom','base_link'),('base_link','rslidar'),('rslidar','fairy_optical')]:assert edge in state['tf_edges'],edge
        assert ('map','odom') not in state['static_edges']
        publishers={e.node_name for e in node.get_publishers_info_by_topic('/tf')}
        assert publishers=={'slam_toolbox','fw_mini_pro_sim_adapter'},publishers
        trajectory=state['trajectory']
        distance=sum(math.dist(a,b) for a,b in zip(trajectory,trajectory[1:]))
        excursion=max(math.dist(p,start[:2]) for p in trajectory)
        assert distance>5 and excursion>1.5,(distance,excursion)
        state['distance_travelled_m']=distance;state['max_excursion_m']=excursion
        error=math.hypot(state['pose'][0]-start[0],state['pose'][1]-start[1]);assert error<.5,error
        times=state.pop('cloud_times');duration=times[-1][0]-times[0][0]
        state['cloud_wall_hz']=(len(times)-1)/duration
        state['sim_real_time_ratio']=(times[-1][1]-times[0][1])/duration
        state['return_distance_m']=error
        state['dynamic_tf_nodes']=sorted(publishers)
        state.pop('tf_sources');state['static_edges']=sorted(state['static_edges'])
        state['map_versions']=len(state.pop('map_hashes'));state['tf_edges']=sorted(state['tf_edges'])
        trajectory=state.pop('trajectory');(args.output/'trajectory.json').write_text(json.dumps(trajectory))
        (args.output/'results.json').write_text(json.dumps(state,indent=2));print('PASS',json.dumps(state),flush=True)
    finally:
        pub.publish(VehicleCmd(enable=False));node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
