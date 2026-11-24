#!/usr/bin/env python3
"""Actual SLAM + Nav2 with synthetic in-place rotation and a paused lidar.

ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 python3 test/smoke_slam_timing.py
No motor driver or real serial port is opened.
"""
import math
import os
from pathlib import Path
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Use ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1')

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from smoke_nav2 import Sensors
from ugv01_room_explore.navigation_bridge import NavigationBridge, yaw_of
from ugv01_room_explore.stack_supervisor import stop_process


class RotatingSensors(Sensors):
    def __init__(self):
        self.started = time.monotonic()
        super().__init__(publish_map=False)
        self.poses = []
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.poses.append, 10)

    def tick(self):
        elapsed = time.monotonic() - self.started
        yaw = min(1.0, max(0.0, elapsed - 4.0) * 0.25)
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation = transform.transform.rotation
        self.odom_pub.publish(odom)
        if 9.0 <= elapsed < 10.6:
            return  # map->odom must stay current, but NAV readiness must expire
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
        scan.angle_min, scan.angle_max = -math.pi, math.pi - math.pi / 180
        scan.angle_increment = math.pi / 180
        scan.scan_time, scan.range_min, scan.range_max = .1, .12, 12.0
        scan.ranges = [2.90 / max(abs(math.cos(-math.pi + i * math.pi / 180 + yaw)),
                                   abs(math.sin(-math.pi + i * math.pi / 180 + yaw)))
                       for i in range(360)]
        self.scan_pub.publish(scan)


def main():
    rclpy.init(args=[])
    bridge, sensors = NavigationBridge(), RotatingSensors()
    bridge.do_stack('nav_slam')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(sensors)
    log_path = Path('/tmp/ugv01-smoke-slam-timing.log')
    seen_ready = saw_expired_scan = False
    tf_ages = []
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'ugv01_room_explore', 'navigation.launch.py', 'mode:=nav_slam'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic() - sensors.started < 14.0:
                executor.spin_once(timeout_sec=.02)
                elapsed = time.monotonic() - sensors.started
                if bridge.read().get('ready'):
                    seen_ready = True
                if 10.2 <= elapsed < 10.5:
                    saw_expired_scan |= bridge.read().get('reason') == 'Нет свежих данных лидара'
                    try:
                        tf = bridge.tf_buffer.lookup_transform('map', 'odom', Time())
                        age = (bridge.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
                        tf_ages.append(age)
                    except TransformException:
                        pass
            assert seen_ready, bridge.read()
            assert len(sensors.poses) >= 5, len(sensors.poses)
            assert max(abs(yaw_of(msg.pose.pose.orientation)) for msg in sensors.poses) > .5
            assert saw_expired_scan, 'Lidar loss was hidden by the restamped map TF'
            assert tf_ages and max(tf_ages) < .15, tf_ages
            assert bridge.read().get('ready'), bridge.read()
            print('SLAM TIMING PASSED', {'in_place_pose_updates': len(sensors.poses),
                'map_tf_age_during_lidar_pause': max(tf_ages),
                'stale_scan_rejected': saw_expired_scan, 'ready_after_resume': True}, flush=True)
        except Exception:
            print(log_path.read_text()[-12000:], flush=True)
            raise
        finally:
            stop_process(process)
            executor.shutdown()
            sensors.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
