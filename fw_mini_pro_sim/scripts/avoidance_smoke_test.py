#!/usr/bin/env python3
"""Launch an isolated Gazebo demo; verify measured bypass and task completion."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as NavPath
from can_client.msg import VehicleCmd
from lawn_mower_tracker.msg import PathStatus
from lawn_mower_tracker.srv import MowerTrackControl


def main():
    output = Path(tempfile.mkdtemp(prefix='avoidance-smoke-'))
    env = os.environ.copy()
    # Isolate both middleware transports, including /clock, from other simulations.
    domain = str(80 + os.getpid() % 100)
    os.environ['ROS_DOMAIN_ID'] = domain
    env.update(ROS_DOMAIN_ID=domain, IGN_PARTITION=f'avoidance_smoke_{os.getpid()}',
               ROS_LOG_DIR=str(output/'ros'), IGN_HOMEDIR=str(output/'ign'),
               XDG_CACHE_HOME=str(output/'cache'))
    log = (output/'gazebo.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'fw_mini_pro_sim', 'avoidance.launch.py', 'gui:=false'],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    client = None
    state = dict(clouds=0, filtered=0, routes=0, enabled=0, poses=[], completed=False)
    print(f'Acceptance logs: {output}', flush=True)
    try:
        rclpy.init()
        node = rclpy.create_node('avoidance_smoke_probe', parameter_overrides=[
            Parameter('use_sim_time', value=True)])

        def on_pose(msg):
            state['poses'].append([msg.pose.position.x, msg.pose.position.y])
            q = msg.pose.orientation
            state['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        def on_cloud(msg):
            state['clouds'] += 1
            state['frame'] = msg.header.frame_id

        def on_status(msg):
            state.update(completed=msg.tracker_state == 6, status=msg.tracker_state)

        def on_filtered(msg):
            state['filtered'] += 1

        def on_route(msg):
            state['routes'] += bool(msg.poses)

        def on_command(msg):
            state['enabled'] += msg.enable

        node.create_subscription(PointCloud2, '/rslidar_points', on_cloud, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/avoidance/filtered_points', on_filtered,
                                 qos_profile_sensor_data)
        node.create_subscription(PoseStamped, '/gnss_pose', on_pose, qos_profile_sensor_data)
        node.create_subscription(NavPath, '/avoidance/local_path', on_route, 10)
        node.create_subscription(VehicleCmd, '/vehicle_cmd', on_command, 10)
        node.create_subscription(PathStatus, '/path_status', on_status, 10)
        client = node.create_client(MowerTrackControl, '/mowerTrackControl')
        deadline = time.monotonic()+45
        while time.monotonic() < deadline and (state['clouds'] < 3 or not state['poses']):
            rclpy.spin_once(node, timeout_sec=.1)
            assert process.poll() is None, 'simulation exited; inspect gazebo.log'
        assert state['clouds'] >= 3 and state['poses'], 'missing lidar/pose; inspect gazebo.log'
        assert client.wait_for_service(timeout_sec=5), 'control service missing'
        request = MowerTrackControl.Request()
        request.track_state, request.line_id = 'start', 1
        future = client.call_async(request)
        deadline = time.monotonic()+5
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert future.done() and future.result().success, 'task start failed'
        deadline, last_report = time.monotonic()+100, time.monotonic()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            assert process.poll() is None, 'simulation exited'
            if time.monotonic()-last_report > 10:
                print('pose:', state['poses'][-1], 'yaw:', state.get('yaw'), flush=True)
                last_report = time.monotonic()
            if state['completed'] or state.get('status') == 5:
                break
        poses = state.pop('poses')
        state['last_pose'] = poses[-1]
        state['max_deviation'] = max(abs(p[1]) for p in poses)
        state['min_box_clearance'] = min(math.hypot(
            max(abs(p[0]-1.8)-.25, 0), max(abs(p[1])-.3, 0)) for p in poses)
        (output/'result.json').write_text(json.dumps(state, indent=2))
        print(json.dumps(state), flush=True)
        assert state['frame'] == 'fairy_optical' and state['filtered'] and state['routes']
        assert state['last_pose'][0] > 3.9 and state['max_deviation'] > .6, state
        assert state['min_box_clearance'] > .35, state
        assert state['completed'] and abs(state['last_pose'][1]) < .2, state
        assert abs(state['yaw']) < .174532925, state
    finally:
        # A second Ctrl+C must not interrupt cleanup and orphan the launch group.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if client is not None and rclpy.ok():
                request = MowerTrackControl.Request()
                request.track_state = 'stop'
                client.call_async(request)
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()


if __name__ == '__main__':
    main()
