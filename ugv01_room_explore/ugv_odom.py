#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import serial
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from ugv01_room_explore.wheel_odometry import WheelOdometry


class Odom(Node):
    def __init__(self):
        super().__init__('ugv_odom')

        # size
        self.body_length = 0.23154
        self.body_width = 0.13483
        self.total_width = 0.19682
        self.track_width = 0.04450
        self.contact_length = 0.10800

        # turns
        self.track_center = 0.19602

        # settings
        self.port = '/dev/ttyAMA0'
        self.baud = 115200

        self.yaw_sign = 1.0

        self.swap_lr = False
        self.lsign = 1.0
        self.rsign = 1.0

        # odl/odr -> meters
        self.odom_scale = 0.01
        self.estimator = WheelOdometry(self.track_center, self.odom_scale, yaw_sign=self.yaw_sign)
        self.last_feedback_time = None
        self.feedback_timeout = 0.25

        # state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_vl = 0.0
        self.last_vr = 0.0
        self.last_dth_deg = 0.0
        self.last_ds = 0.0

        self.buf = ""

        # cmd_vel passthrough
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = None
        self.zero_sent = False

        # ===== SERIAL =====
        self.ser = serial.Serial(self.port, self.baud, timeout=0.001, exclusive=True)

        # ===== ROS =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 20)

        # esp32
        self.send({"T": 605, "cmd": 0})
        self.send({"T": 143, "cmd": 0})
        self.send({"T": 900, "main": 3, "module": 0})
        self.send({"T": 131, "cmd": 1})
        # 20 Hz feedback leaves UART bandwidth for commands and IMU fields.
        # TF publication is independent of the packet rate.
        self.send({"T": 142, "cmd": 50})

        self.get_logger().info(
            "=== UGV01 WHEEL ODOM STARTED === "
            f"track_center={self.track_center:.5f} m "
            f"track_width={self.track_width:.5f} m "
            f"total_width={self.total_width:.5f} m "
            f"body_length={self.body_length:.5f} m"
        )

        self.create_timer(0.002, self.read_serial)
        self.create_timer(0.05, self.cmd_timer)
        self.create_timer(0.02, self.publish_timer)
        self.create_timer(3.0, self.status)

    def send(self, d):
        try:
            self.ser.write((json.dumps(d, separators=(',', ':')) + '\n').encode())
        except Exception as e:
            self.get_logger().warn(f"send error: {e}")

    def cmd_cb(self, msg):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_wz = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()
        self.zero_sent = False

    def cmd_timer(self):
        now = time.monotonic()

        if self.last_cmd_time is None:
            if not self.zero_sent:
                self.send({"T": 13, "X": 0.0, "Z": 0.0})
                self.zero_sent = True
            return

        age = now - self.last_cmd_time

        if age < 0.3:
            # forward/backward
            self.send({"T": 13, "X": -self.cmd_vx, "Z": self.cmd_wz})
        else:
            if not self.zero_sent:
                self.send({"T": 13, "X": 0.0, "Z": 0.0})
                self.zero_sent = True

    def read_serial(self):
        try:
            raw = self.ser.read(self.ser.in_waiting or 1)
            if not raw:
                return

            self.buf += raw.decode('utf-8', errors='ignore')

            latest_packet = None

            while '\n' in self.buf:
                line, self.buf = self.buf.split('\n', 1)
                line = line.strip()

                if not line.startswith('{'):
                    continue

                try:
                    d = json.loads(line)
                except Exception:
                    continue

                if d.get("T") == 1001:
                    latest_packet = d

            # we take only the freshest package
            if latest_packet is not None:
                self.process(latest_packet)

        except Exception as e:
            self.get_logger().warn(f"read error: {e}")

    def process(self, d):
        # we take a stable linear path from odl/odr
        if "odl" not in d or "odr" not in d:
            return

        try:
            odl = float(d["odl"])
            odr = float(d["odr"])
        except Exception:
            return

        if self.swap_lr:
            odl, odr = odr, odl

        odl *= self.lsign
        odr *= self.rsign

        now = time.monotonic()
        if not self.estimator.update(odl, odr, now):
            return
        self.last_feedback_time = now
        self.x, self.y, self.th = self.estimator.x, self.estimator.y, self.estimator.yaw
        self.last_ds = self.estimator.ds
        self.last_dth_deg = math.degrees(self.estimator.dtheta)
        self.last_vl, self.last_vr = self.estimator.vl, self.estimator.vr

    def publish_timer(self):
        # Hold the most recent measured pose at 50 Hz while telemetry is fresh.
        # Do not restamp a lost encoder stream indefinitely.
        if self.last_feedback_time is None or time.monotonic() - self.last_feedback_time > self.feedback_timeout:
            return
        self.publish(self.get_clock().now(), self.estimator.vx, self.estimator.wz)

    def publish(self, stamp, vx, wz):
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        pose_cov[0] = 0.02
        pose_cov[7] = 0.02
        pose_cov[14] = 9999.0
        pose_cov[21] = 9999.0
        pose_cov[28] = 9999.0
        pose_cov[35] = 0.05

        twist_cov[0] = 0.02
        twist_cov[7] = 0.02
        twist_cov[14] = 9999.0
        twist_cov[21] = 9999.0
        twist_cov[28] = 9999.0
        twist_cov[35] = 0.05

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def status(self):
        feedback_age = (time.monotonic() - self.last_feedback_time
                        if self.last_feedback_time is not None else float('inf'))
        self.get_logger().info(
            f"ODOM x={self.x:.3f} y={self.y:.3f} "
            f"yaw_total={math.degrees(self.th):.1f}° "
            f"dth={self.last_dth_deg:.2f}° "
            f"ds={self.last_ds:.4f} "
            f"vL={self.last_vl:.3f} vR={self.last_vr:.3f} "
            f"track_center={self.track_center:.5f} "
            f"cmd_x={self.cmd_vx:.3f} cmd_z={self.cmd_wz:.3f} "
            f"feedback_age={feedback_age:.3f}s"
        )

    def destroy_node(self):
        try:
            self.send({"T": 13, "X": 0.0, "Z": 0.0})
            self.send({"T": 131, "cmd": 0})
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
