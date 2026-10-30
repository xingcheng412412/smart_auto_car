#!/usr/bin/env python3
"""Start isolated headless Gazebo; verify online mapping and localization faults."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracker-stop',action='store_true')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='173'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    os.environ['IGN_PARTITION']='fusion_acceptance_'+str(os.getpid())
    logs=Path(tempfile.mkdtemp(prefix='fusion_acceptance_'))
    os.environ['ROS_LOG_DIR']=str(logs/'ros')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid,Odometry
    from lawn_mower_tracker.srv import MowerTrackControl
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from slam_toolbox.srv import Pause
    from can_client.msg import VehicleCmd
    from tf2_msgs.msg import TFMessage
    from rcl_interfaces.srv import GetParameters
    rclpy.init();node=rclpy.create_node('fusion_acceptance')
    state={'poses':0,'maps':0,'status':'','max_step':0.,'tf_sources':set()}
    last=[None]
    def pose(m):
        p=m.pose.position
        if last[0] is not None:
            state['max_step']=max(state['max_step'],math.hypot(p.x-last[0][0],p.y-last[0][1]))
        last[0]=(p.x,p.y);state['poses']+=1
    def mapping(m):state['maps']+=1
    def tf(m):
        for t in m.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                state['tf_sources'].add('map->odom')
    node.create_subscription(PoseStamped,'/localization/pose',pose,10)
    node.create_subscription(String,'/localization/status',lambda m:state.update(status=m.data),10)
    node.create_subscription(OccupancyGrid,'/map',mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage,'/tf',tf,100)
    node.create_subscription(Odometry,'/odom',lambda m:state.update(speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)),10)
    node.create_subscription(VehicleCmd,'/vehicle_cmd',lambda m:state.update(command_enabled=m.enable),10)
    track=node.create_client(MowerTrackControl,'/mowerTrackControl')
    cmd=node.create_publisher(VehicleCmd,'/vehicle_cmd',1)
    drop=node.create_client(SetBool,'/sim/rtk/set_dropout')
    pause=node.create_client(Pause,'/slam_toolbox/pause_new_measurements')
    stream=(logs/'launch.log').open('w')
    process=subprocess.Popen(['ros2','launch','fw_mini_pro_sim','fusion.launch.py','gui:=false','rviz:=false','tracker:='+str(args.tracker_stop).lower()],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    print('Logs:',logs,flush=True)
    def spin(duration,moving=False):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            assert process.poll() is None,'launch exited; see '+str(logs)
            if not args.tracker_stop:
                cmd.publish(VehicleCmd(enable=moving,gear=6,linear_x=.15 if moving else 0.,angular_z=.15 if moving else 0.))
            rclpy.spin_once(node,timeout_sec=.02)
    def call(client,request):
        assert client.wait_for_service(timeout_sec=10),'service missing'
        future=client.call_async(request);deadline=time.monotonic()+10
        while not future.done() and time.monotonic()<deadline:spin(.05)
        assert future.done(),'service timeout'
        return future.result()
    try:
        deadline=time.monotonic()+70
        while time.monotonic()<deadline and (not state['status'].startswith('FUSED') or not state['maps']):spin(.2)
        assert state['status'].startswith('FUSED') and state['maps'],state
        if args.tracker_stop:
            spin(1)
            result=call(track,MowerTrackControl.Request(track_state='start',map_name='',line_id=1))
            assert result.success,'tracker start rejected'
            spin(3)
            assert state.get('speed',0.)>.03,state
            assert call(drop,SetBool.Request(data=True)).success
            call(pause,Pause.Request())
            spin(5)
            assert state['status'].startswith('LOCALIZATION_LOST'),state
            assert state.get('speed',1.)<.03 and not state.get('command_enabled',True),state
            state['tf_sources']=list(state['tf_sources'])
            (logs/'results.json').write_text(json.dumps(state,indent=2))
            print('PASS tracker loss stop',json.dumps(state),flush=True)
            return
        spin(4,True)
        assert call(drop,SetBool.Request(data=True)).success
        spin(5,True)
        assert state['status'].startswith('SLAM_ONLY'),state
        before=state['poses'];spin(2,True);assert state['poses']>before+5,state
        assert call(drop,SetBool.Request(data=False)).success
        spin(4,True);assert state['status'].startswith('FUSED'),state
        assert call(drop,SetBool.Request(data=True)).success
        call(pause,Pause.Request())
        spin(5)
        assert state['status'].startswith('LOCALIZATION_LOST'),state
        before=state['poses'];spin(1);assert state['poses']==before,state
        assert len(state['tf_sources'])==1,state
        params=node.create_client(GetParameters,'/slam_toolbox/get_parameters')
        result=call(params,GetParameters.Request(names=['transform_publish_period']))
        assert result.values[0].double_value==0.,'SLAM TF is not disabled'
        assert state['max_step']<.5,state
        state['tf_sources']=list(state['tf_sources'])
        (logs/'results.json').write_text(json.dumps(state,indent=2))
        print('PASS',json.dumps(state),flush=True)
    finally:
        cmd.publish(VehicleCmd(enable=False))
        os.killpg(process.pid,signal.SIGINT) if process.poll() is None else None
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        stream.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
