#!/usr/bin/env python3
"""Replay a private passive snapshot in isolated DDS, without motor drivers.

The snapshot stays outside the repository. It contains map, scan and static TF.
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 python3 test/smoke_recorded_scan.py SNAPSHOT
"""
import argparse
from concurrent.futures import Future
import json
import os
from pathlib import Path
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Use ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1')

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from smoke_nav2 import Sensors, spin_until
from ugv01_room_explore.navigation_bridge import NavigationBridge
from ugv01_room_explore.stack_supervisor import stop_process


class Replay(Sensors):
    def __init__(self, snapshot):
        super().__init__(publish_map=True)
        set_message_fields(self.grid, snapshot['data']['/map'])
        self.scan = LaserScan()
        set_message_fields(self.scan, snapshot['data']['/ldlidar_node/scan'])
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        for value in snapshot['tf_edges'].values():
            if value['child_frame_id'] in ('base_link', 'ldlidar_base', 'ldlidar_link'):
                msg = TransformStamped()
                set_message_fields(msg, value)
                transforms.append(msg)
        self.static_tf.sendTransform(transforms)
        self.raw = []
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
            lambda m: self.raw.append((m.linear.x, m.angular.z)), 10)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_footprint')]:
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
            msg.transform.rotation.w = 1.0
            transforms.append(msg)
        self.tf.sendTransform(transforms)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.grid.header.stamp = stamp
        self.map_pub.publish(self.grid)
        self.scan.header.stamp = stamp
        self.scan_pub.publish(self.scan)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot')
    parser.add_argument('--params-file')
    parser.add_argument('--expect-stuck', action='store_true')
    args = parser.parse_args()
    text = Path(args.snapshot).read_text(encoding='utf-8-sig')
    snapshot = json.loads(next(line.split('=', 1)[1] for line in text.splitlines() if line.startswith('AUDIT_JSON=')))
    rclpy.init(args=[])
    bridge, replay = NavigationBridge(), Replay(snapshot)
    bridge.do_stack('nav_slam')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(replay)
    command = ['ros2', 'launch', 'ugv01_room_explore', 'navigation.launch.py',
               'mode:=nav_slam', 'start_slam:=false']
    if args.params_file:
        command.append('params_file:=' + args.params_file)
    log_path = Path('/tmp/ugv01-replay-' + ('before' if args.expect_stuck else 'after') + '.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            spin_until(executor, lambda: bridge.read().get('ready'), 25, 'replay Nav2 ready')
            reply = Future()
            bridge.do_goal(1.21, .12, 0., reply=reply)
            spin_until(executor, reply.done, 5, 'replayed goal accepted')
            assert reply.result()['ok'], reply.result()
            until = time.monotonic() + 3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.03)
            forward = max((v for v, w in replay.raw), default=0)
            turn = max((abs(w) for v, w in replay.raw), default=0)
            print('RECORDED_SCAN_COMMANDS', {'raw_count': len(replay.raw),
                'max_forward': forward, 'max_turn': turn, 'expect_stuck': args.expect_stuck}, flush=True)
            if args.expect_stuck:
                assert forward == 0 and turn < .06
            else:
                assert forward > .02 or turn > .2, log_path.read_text()[-3000:]
        finally:
            bridge.do_cancel()
            stop_process(process)
            executor.shutdown()
            replay.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
