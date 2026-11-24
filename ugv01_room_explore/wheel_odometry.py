"""Cumulative encoder integration, independent of ROS and the serial port."""
from collections import deque
import math


class WheelOdometry:
    def __init__(self, track_center=0.19602, scale=0.01, velocity_window=0.25, yaw_sign=1.0):
        self.track_center, self.scale = track_center, scale
        self.velocity_window = velocity_window
        self.yaw_sign = yaw_sign
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = self.vl = self.vr = 0.0
        self.ds = self.dtheta = 0.0
        self.last_time = self.last_left = self.last_right = None
        self.samples = deque()

    def update(self, left, right, now):
        if not all(math.isfinite(v) for v in (left, right, now)):
            return False
        left, right = left * self.scale, right * self.scale
        if self.last_time is None:
            self.last_time, self.last_left, self.last_right = now, left, right
            self.samples.append((now, left, right))
            return True
        dt = now - self.last_time
        if dt <= 0:
            return False
        dl, dr = left - self.last_left, right - self.last_right
        self.last_time, self.last_left, self.last_right = now, left, right
        # Encoder reset/corruption: rebase counters, without teleporting the pose.
        # Include two centimetres of quantization allowance at short intervals.
        if max(abs(dl), abs(dr)) > 0.8 * dt + 0.02:
            self.samples.clear()
            self.samples.append((now, left, right))
            self.vx = self.wz = self.vl = self.vr = 0.0
            return False
        self.ds = (dl + dr) / 2
        self.dtheta = self.yaw_sign * (dr - dl) / self.track_center
        middle = self.yaw + self.dtheta / 2
        arc = self.ds
        if abs(self.dtheta) > 1e-9:
            arc *= math.sin(self.dtheta / 2) / (self.dtheta / 2)
        self.x += arc * math.cos(middle)
        self.y += arc * math.sin(middle)
        self.yaw += self.dtheta
        # Pose uses every cumulative tick. Average velocity over time so 1 cm
        # feedback does not alternate between zero and one-packet speed spikes.
        self.samples.append((now, left, right))
        while len(self.samples) > 2 and self.samples[1][0] <= now - self.velocity_window:
            self.samples.popleft()
        first_time, first_left, first_right = self.samples[0]
        window = now - first_time
        if window > 0:
            self.vl = (left - first_left) / window
            self.vr = (right - first_right) / window
            self.vx = (self.vl + self.vr) / 2
            self.wz = self.yaw_sign * (self.vr - self.vl) / self.track_center
        return True
