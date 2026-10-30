#!/usr/bin/env python3
"""Headless physical acceptance test; starts/stops its own isolated simulation.

Run from a sourced workspace. --tracker tests the three supplied tracking paths.
No commands are sent to the caller's ROS domain or Gazebo partition.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracker',action='store_true')
    parser.add_argument('--rtk',action='store_true',help='Use noisy RTK simulated localization')
    parser.add_argument('--path',choices=['straight','reverse','turn'],help='Run one tracking case')
    parser.add_argument('--path-file',type=Path,help='External CSV for one full tracking test')
    parser.add_argument('--timeout',type=float,default=100,help='Tracking timeout in seconds')
    args = parser.parse_args()
    if args.path_file and (not args.tracker or args.path):
        parser.error('--path-file requires --tracker and cannot be combined with --path')
    if args.path_file and not args.path_file.is_file():
        parser.error('CSV does not exist: '+str(args.path_file))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    logs = Path(tempfile.mkdtemp(prefix='fw_sim_acceptance_'))
    os.environ['ROS_DOMAIN_ID'] = str(100+os.getpid()%100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['IGN_PARTITION'] = 'fw_acceptance_'+uuid.uuid4().hex
    os.environ['ROS_LOG_DIR'] = str(logs/'ros')
    import rclpy
    from nav_msgs.msg import Odometry
    from can_client.msg import VehicleCmd
    from lawn_mower_tracker.msg import PathStatus
    from lawn_mower_tracker.srv import MowerTrackControl
    from ament_index_python.packages import get_package_share_directory
    rclpy.init()
    node = rclpy.create_node('fw_sim_acceptance')
    state = {'odom':None,'status':None}
    trajectory = []
    def on_odometry(msg):
        state['odom'] = msg
        p = msg.pose.pose.position
        trajectory.append([p.x,p.y])
    node.create_subscription(Odometry,'/odom',on_odometry,10)
    node.create_subscription(PathStatus,'/path_status',lambda msg:state.update(status=msg.tracker_state),10)
    pub = node.create_publisher(VehicleCmd,'/vehicle_cmd',1)
    client = node.create_client(MowerTrackControl,'/mowerTrackControl')
    share = Path(get_package_share_directory('fw_mini_pro_sim'))
    results = {}
    print('Acceptance logs:',logs,flush=True)

    def spin(seconds,cmd=None):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            if cmd is not None:
                pub.publish(cmd)
            rclpy.spin_once(node,timeout_sec=.04)

    def pose():
        msg = state['odom']
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return p.x,p.y,yaw

    def path_for(name):
        return args.path_file.resolve() if args.path_file else share/'paths'/f'{name}.csv'

    def start(name,tracker=False):
        state.update(odom=None,status=None)
        trajectory.clear()
        stream = (logs/(name+'.log')).open('w')
        cmd = ['ros2','launch','fw_mini_pro_sim','simulation.launch.py','gui:=false']
        if args.rtk:
            cmd += ['rtk:=true']
        if tracker:
            cmd += ['tracker:=true','path_file:='+str(path_for(name))]
        process = subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        until = time.monotonic()+45
        while state['odom'] is None and time.monotonic()<until and process.poll() is None:
            spin(.1)
        if state['odom'] is None:
            stop(process,stream)
            raise RuntimeError('No odometry received; inspect '+str(logs/(name+'.log')))
        spin(1)
        return process,stream

    def stop(process,stream):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        stream.close()

    def motion(name,gear,vx,vy,wz,duration):
        initial = pose()
        cmd = VehicleCmd(enable=True,gear=gear,linear_x=float(vx),linear_y=float(vy),angular_z=float(wz))
        spin(duration,cmd)
        final = pose()
        dx,dy = final[0]-initial[0],final[1]-initial[1]
        c,s = math.cos(initial[2]),math.sin(initial[2])
        result = {'forward_m':c*dx+s*dy,'left_m':-s*dx+c*dy,
                  'yaw_rad':math.remainder(final[2]-initial[2],2*math.pi)}
        spin(1.5)  # no commands: watchdog must brake the physical wheels
        twist = state['odom'].twist.twist
        result['stopped_speed_mps'] = math.hypot(twist.linear.x,twist.linear.y)
        result['stopped_yaw_rps'] = abs(twist.angular.z)
        assert result['stopped_speed_mps'] < .06,result
        assert result['stopped_yaw_rps'] < .08,result
        results[name] = result
        print(name,json.dumps(result),flush=True)
        return result

    try:
        if not args.tracker:
            process,stream = start('motion')
            try:
                r = motion('forward',6,.3,0,0,4)
                assert .7 < r['forward_m'] < 1.5 and abs(r['left_m']) < .15,r
                r = motion('reverse',6,-.3,0,0,4)
                assert -1.5 < r['forward_m'] < -.7 and abs(r['left_m']) < .15,r
                r = motion('turn',6,.25,0,.3,4)
                assert r['yaw_rad'] > .5 and math.hypot(r['forward_m'],r['left_m']) > .4,r
                r = motion('crab',8,0,.25,0,4)
                assert r['left_m'] > .4 and abs(r['forward_m']) < .2 and abs(r['yaw_rad']) < .2,r
                r = motion('spin',6,0,0,.4,4)
                assert r['yaw_rad'] > .6 and math.hypot(r['forward_m'],r['left_m']) < .2,r
            finally:
                stop(process,stream)
        else:
            for name in ([args.path_file.stem] if args.path_file else ([args.path] if args.path else ('straight','reverse','turn'))):
                process,stream = start(name,True)
                try:
                    assert client.wait_for_service(timeout_sec=10),'Tracker service unavailable'
                    req = MowerTrackControl.Request(track_state='start',map_name='',line_id=1)
                    future = client.call_async(req)
                    until = time.monotonic()+10
                    while not future.done() and time.monotonic()<until:
                        spin(.1)
                    assert future.done() and future.result().success,'Tracker rejected start'
                    if name == 'straight':
                        spin(2)
                        pause = client.call_async(MowerTrackControl.Request(track_state='pause'))
                        until = time.monotonic()+5
                        while not pause.done() and time.monotonic()<until:
                            spin(.1)
                        assert pause.done() and pause.result().success,'Pause rejected'
                        spin(1.5)
                        t = state['odom'].twist.twist
                        assert state['status'] == 4 and math.hypot(t.linear.x,t.linear.y) < .06,'Pause did not stop chassis'
                        resume = client.call_async(MowerTrackControl.Request(track_state='resume'))
                        until = time.monotonic()+5
                        while not resume.done() and time.monotonic()<until:
                            spin(.1)
                        assert resume.done() and resume.result().success,'Resume rejected'
                        results['pause_resume'] = 'passed'
                    task_start = time.monotonic()
                    until = task_start+args.timeout
                    next_report = task_start+30
                    while state['status'] != 6 and time.monotonic()<until:
                        spin(.2)
                        if time.monotonic() >= next_report:
                            print(name,'elapsed_s',round(time.monotonic()-task_start),'pose',pose(),'status',state['status'],flush=True)
                            next_report += 30
                        if state['status'] == 5:
                            (logs/(name+'_trajectory.json')).write_text(json.dumps(trajectory))
                            raise AssertionError('Tracker reported stopped/error: '+name+' final pose='+str(pose()))
                    assert state['status'] == 6,'Tracker did not complete: '+name
                    spin(1)
                    import csv
                    with path_for(name).open() as path_stream:
                        goal = list(csv.DictReader(path_stream))[-1]
                    x,y,yaw = pose()
                    distance = math.hypot(x-float(goal['pose_x']),y-float(goal['pose_y']))
                    yaw_error = abs(math.remainder(yaw-float(goal['yaw']),2*math.pi))
                    assert distance < .23 and yaw_error < .18,(name,distance,yaw_error)
                    results[name] = {'status':state['status'],'final_pose':pose(),
                                     'goal_distance_m':distance,'goal_yaw_error_rad':yaw_error}
                    (logs/(name+'_trajectory.json')).write_text(json.dumps(trajectory))
                    print(name,json.dumps(results[name]),flush=True)
                finally:
                    stop(process,stream)
        (logs/'results.json').write_text(json.dumps(results,indent=2))
        print('PASS',json.dumps(results),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
