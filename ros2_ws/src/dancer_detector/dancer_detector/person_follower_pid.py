#!/usr/bin/env python3
"""
PID person follower for iRobot Create 3 (or any /cmd_vel robot)

Subscribes:
- /person_center_depth (example_interfaces/Float32MultiArray)  # [u_px, z_m]

Publishes:
- /cmd_vel (geometry_msgs/msg/Twist)

Parameters (declare → runtime-tunable):
- img_width:               int     default 640
- hfov_deg:                float   default 69.4    # camera horizontal FOV (adjust to your lens)
- target_dist_m:           float   default 1.75
- max_lin_vel:             float   default 0.35    # abs(m/s) clamp (applies to forward & reverse)
- max_ang_vel:             float   default 1.8     # abs(rad/s)
- deadband_px:             float   default 4.0     # ignore tiny pixel jitter for yaw
- deadband_m:             float   default 0.03     # ignore tiny depth jitter for forward
- lp_alpha_err:            float   default 0.25    # low-pass on error (0..1, higher=faster)
- timeout_no_target_s:     float   default 0.6     # stop if no target msgs within this
- rate_hz:                 float   default 20.0

PID gains (yaw):
- kp_yaw, ki_yaw, kd_yaw:  floats  defaults  (1.2, 0.0, 0.12)

PID gains (forward):
- kp_fwd, ki_fwd, kd_fwd:  floats  defaults  (0.8, 0.0, 0.05)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from example_interfaces.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_min: float, i_max: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_min = float(i_min)
        self.i_max = float(i_max)
        self.integral = 0.0
        self.prev_err = None
        self.prev_t = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.prev_t = None

    def step(self, err: float, now_sec: float) -> float:
        if self.prev_t is None:
            self.prev_t = now_sec
            self.prev_err = err
            return self.kp * err

        dt = max(1e-4, now_sec - self.prev_t)
        self.prev_t = now_sec

        # Integral with clamp (anti-windup)
        self.integral += err * dt
        if self.integral > self.i_max:
            self.integral = self.i_max
        elif self.integral < self.i_min:
            self.integral = self.i_min

        # Derivative on error
        derr = (err - self.prev_err) / dt
        self.prev_err = err

        return self.kp * err + self.ki * self.integral + self.kd * derr


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower_pid')

        # --- Parameters ---
        p = self.declare_parameter
        self.img_width        = int(p('img_width', 640).value)
        self.hfov_deg         = float(p('hfov_deg', 69.4).value)
        self.target_dist_m    = float(p('target_dist_m', 1.75).value)
        self.max_lin_vel      = float(p('max_lin_vel', 20).value)
        self.max_ang_vel      = float(p('max_ang_vel', 20).value)
        self.deadband_px      = float(p('deadband_px', 0.05).value)
        self.deadband_m       = float(p('deadband_m', 0.03).value)
        self.lp_alpha_err     = float(p('lp_alpha_err', 0.25).value)
        self.timeout_s        = float(p('timeout_no_target_s', 0.6).value)
        self.rate_hz          = float(p('rate_hz', 20.0).value)

        self.kp_yaw = float(p('kp_yaw', 1.2).value)
        self.ki_yaw = float(p('ki_yaw', 0.0).value)
        self.kd_yaw = float(p('kd_yaw', 0.12).value)

        self.kp_fwd = float(p('kp_fwd', 0.8).value)
        self.ki_fwd = float(p('ki_fwd', 0.0).value)
        self.kd_fwd = float(p('kd_fwd', 0.05).value)

        # Derived
        self.cx_px = 0.5 * (self.img_width - 1)
        self.hfov_rad = math.radians(self.hfov_deg)

        # PIDs
        self.pid_yaw = PID(self.kp_yaw, self.ki_yaw, self.kd_yaw, i_min=-1.0, i_max=1.0)
        self.pid_fwd = PID(self.kp_fwd, self.ki_fwd, self.kd_fwd, i_min=-1.0, i_max=1.0)

        # State
        self.last_msg_time: Optional[Time] = None
        self.err_yaw_f = 0.0   # filtered yaw error (rad)
        self.err_fwd_f = 0.0   # filtered forward error (m)
        self.have_target = False

        # I/O
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/person_center_depth',
            self.on_center_depth,
            10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self.loop)

        self.get_logger().info('person_follower_pid ready. Waiting for /person_center_depth...')

    # Convert pixel offset to bearing (rad) using HFOV
    def px_to_bearing(self, u_px: float) -> float:
        # normalize to [-1, 1] across width
        nx = (u_px - self.cx_px) / max(1.0, self.cx_px)
        # exact mapping using tangent model:
        theta = math.atan(nx * math.tan(0.5 * self.hfov_rad))
        return float(theta)

    def on_center_depth(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        u_px, z_m = float(msg.data[0]), float(msg.data[1])

        now = self.get_clock().now()
        self.last_msg_time = now
        self.have_target = True

        # Compute raw errors
        bearing_err = self.px_to_bearing(u_px)            # rad, want 0
        range_err   = (z_m - self.target_dist_m)          # m,  want 0 (negative → too close → reverse)

        # Deadbands
        if abs((u_px - self.cx_px)) <= self.deadband_px:
            bearing_err = 0.0
        if abs(range_err) <= self.deadband_m:
            range_err = 0.0

        # Low-pass filter errors
        a = max(0.0, min(1.0, self.lp_alpha_err))
        self.err_yaw_f = (1.0 - a) * self.err_yaw_f + a * bearing_err
        self.err_fwd_f = (1.0 - a) * self.err_fwd_f + a * range_err

    def loop(self):
        now_ros = self.get_clock().now()
        now_sec = now_ros.seconds_nanoseconds()[0] + now_ros.seconds_nanoseconds()[1] * 1e-9

        # Timeout / safety stop on lost target
        if (self.last_msg_time is None) or \
           ((now_ros - self.last_msg_time) > Duration(seconds=self.timeout_s)) or \
           (not self.have_target):
            self.pid_yaw.reset()
            self.pid_fwd.reset()
            self.publish_stop()
            return

        # PID outputs (note: negative forward error → negative v_cmd = reverse)
        w_cmd = self.pid_yaw.step(self.err_yaw_f, now_sec)   # rad/s
        v_cmd = self.pid_fwd.step(self.err_fwd_f, now_sec)   # m/s

        # Saturation (symmetric limits)
        w_cmd = max(-self.max_ang_vel, min(self.max_ang_vel, w_cmd))
        v_cmd = max(-self.max_lin_vel, min(self.max_lin_vel, v_cmd))

        # Publish
        twist = Twist()
        twist.linear.x  = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(twist)

    def publish_stop(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
