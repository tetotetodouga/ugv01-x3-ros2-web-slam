#!/usr/bin/env python3
"""Software-only Nav2 smoke: synthetic sensors, isolated DDS, NO hardware launch.

Run explicitly: ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 python3 test/smoke_nav2.py
This validates plugins, lifecycle, planning, action acceptance and cancellation.
It does not validate physical driving, odometry calibration or real localization.
"""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
from concurrent.futures import Future

if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Use ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 to isolate this smoke test')

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from ugv01_room_explore.navigation_bridge import NavigationBridge
from ugv01_room_explore.stack_supervisor import stop_process


class Sensors(Node):
    def __init__(self, publish_map):
        super().__init__('software_smoke_sensors')
        self.publish_map = publish_map
        self.tf = TransformBroadcaster(self)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.info.width = self.grid.info.height = 60
        self.grid.info.resolution = .1
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -3.0
        self.grid.info.origin.orientation.w = 1.0
        self.grid.data = [100 if x in (0, 59) or y in (0, 59) else 0
                          for y in range(60) for x in range(60)]
        self.map_pub = self.create_publisher(OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if publish_map else None
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/ldlidar_node/scan', 10)
        self.velocities = []
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.velocities.append(m.linear.x), 10)
        self.create_timer(.1, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        edges = [('odom', 'base_footprint')]
        if self.publish_map:
            edges.append(('map', 'odom'))
        for parent, child in edges:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self.tf.sendTransform(transforms)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
        scan.angle_min, scan.angle_max = -math.pi, math.pi - math.pi / 180
        scan.angle_increment = math.pi / 180
        scan.scan_time, scan.range_min, scan.range_max = .1, .12, 12.0
        scan.ranges = [2.90 / max(abs(math.cos(-math.pi + i * math.pi / 180)),
                                   abs(math.sin(-math.pi + i * math.pi / 180))) for i in range(360)]
        self.scan_pub.publish(scan)
        if self.map_pub:
            self.grid.header.stamp = stamp
            self.map_pub.publish(self.grid)


def spin_until(executor, condition, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.05)
        if condition():
            return
    raise AssertionError('Timeout: ' + description)


def check(mode, directory):
    rclpy.init(args=[])
    bridge = NavigationBridge()
    sensors = Sensors(publish_map=mode == 'nav_slam')
    bridge.do_stack(mode)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(sensors)
    command = ['ros2', 'launch', 'ugv01_room_explore', 'navigation.launch.py', 'mode:=' + mode]
    if mode == 'nav_slam':
        command.append('start_slam:=false')
    else:
        image = directory / 'room.pgm'
        image.write_bytes(b'P5\n60 60\n255\n' + bytes(
            0 if x in (0, 59) or y in (0, 59) else 254 for y in range(60) for x in range(60)))
        map_file = directory / 'room.yaml'
        map_file.write_text('image: room.pgm\nresolution: 0.1\norigin: [-3.0, -3.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        command.append('map:=' + str(map_file))
    log_path = Path('/tmp/ugv01-smoke-' + mode + '.log')
    log = log_path.open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        if mode == 'nav_map':
            spin_until(executor, lambda: bridge.active('amcl') and bridge.map_msg is not None,
                       25, 'AMCL active + map')
            bridge.do_initialpose(0., 0., 0.)
        spin_until(executor, lambda: bridge.read().get('ready'), 35,
                   'Nav2 ready: ' + str(bridge.read()))
        print(mode, 'LIFECYCLE_READY', bridge.read(), flush=True)
        planner = ActionClient(sensors, ComputePathToPose, '/compute_path_to_pose')
        spin_until(executor, planner.server_is_ready, 5, 'planner action server')
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = 1.0
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = 'GridBased'
        sent = planner.send_goal_async(goal)
        spin_until(executor, sent.done, 8, 'planner goal accepted')
        assert sent.result().accepted
        planned = sent.result().get_result_async()
        spin_until(executor, planned.done, 8, 'path computed')
        assert len(planned.result().result.path.poses) > 2
        print(mode, 'PATH_OK', len(planned.result().result.path.poses), flush=True)
        response = Future()
        bridge.do_goal(1., 0., 0., reply=response)
        spin_until(executor, response.done, 8, 'navigation goal accepted')
        assert response.result()['ok'], response.result()
        spin_until(executor, lambda: any(v > .01 for v in sensors.velocities), 8, 'isolated velocity pipeline')
        assert bridge.drive_mode == 'NAV'
        bridge.do_cancel()
        assert bridge.drive_mode == 'JOY'
        spin_until(executor, lambda: bridge.goal_state == 'canceled', 3, 'goal canceled')
        print(mode, 'ACTION_AND_CANCEL_OK (isolated domain; no motor driver)', flush=True)
        planner.destroy()
    except Exception:
        print('STATUS AT FAILURE', bridge.read(), flush=True)
        print(log_path.read_text()[-12000:], flush=True)
        raise
    finally:
        bridge.do_cancel()
        stop_process(process)
        log.close()
        executor.shutdown()
        sensors.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='ugv01-nav2-smoke-') as folder:
        for mode in ('nav_slam', 'nav_map'):
            check(mode, Path(folder))
    print('SOFTWARE SMOKE PASSED', flush=True)
