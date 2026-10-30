#!/usr/bin/env python3
"""Observe a running FAIRY mapping pipeline; optionally validate its save service."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('fairy_mapping_check')
    report = {}
    stamps = {}
    arrivals = {}
    positions = []
    def observe(key, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        stats = report.setdefault(key, {'count': 0, 'backward_stamps': 0})
        if key in stamps and stamp <= stamps[key]:
            stats['backward_stamps'] += 1
        stamps[key] = stamp
        arrivals.setdefault(key, []).append(time.monotonic())
        stats['count'] += 1
        stats['frame'] = msg.header.frame_id
        if isinstance(msg, PointCloud2):
            stats['points'] = msg.width*msg.height
            stats['fields'] = [field.name for field in msg.fields]
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            stats['finite'] = stats.get('finite', True) and all(math.isfinite(x) for x in values)
            positions.append([p.x, p.y, p.z])
    specs = [('points', PointCloud2, '/fairy/points'), ('imu', Imu, '/fairy/imu'),
             ('odometry', Odometry, '/slam3d/odometry'),
             ('registered', PointCloud2, '/slam3d/registered')]
    subscriptions = [node.create_subscription(typ, topic, lambda msg, key=key: observe(key, msg),
                     qos_profile_sensor_data) for key, typ, topic in specs]
    subscriptions.append(node.create_subscription(PointCloud2, '/slam3d/map',
        lambda msg: observe('map', msg), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    try:
        end = time.monotonic()+args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        for key, values in arrivals.items():
            report[key]['hz'] = (len(values)-1)/(values[-1]-values[0]) if len(values)>1 else 0.0
        if positions:
            report['displacement_m'] = math.dist(positions[0], positions[-1])
            report['last_position'] = positions[-1]
        if args.save:
            client = node.create_client(Trigger, '/slam3d/save_map')
            if client.wait_for_service(timeout_sec=3.0):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
                if future.done() and future.result():
                    result = future.result()
                    report['save'] = {'success': result.success, 'message': result.message}
                else:
                    report['save'] = {'success': False, 'message': 'service timeout'}
            else:
                report['save'] = {'success': False, 'message': 'service unavailable'}
        passed = all(report.get(key, {}).get('count', 0) >= minimum for key, minimum in
                     [('points', 10), ('imu', 100), ('odometry', 5), ('registered', 5), ('map', 1)])
        passed = passed and report.get('odometry', {}).get('finite', False)
        passed = passed and all(stats.get('backward_stamps', 0)==0 for stats in report.values() if isinstance(stats, dict))
        if args.save:
            passed = passed and report.get('save', {}).get('success', False)
        report['passed'] = bool(passed)
        output = json.dumps(report, indent=2)
        print(output)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output+'\n', encoding='utf-8')
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
