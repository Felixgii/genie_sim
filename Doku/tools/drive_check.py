#!/usr/bin/env python3
"""Short chassis drive check via the servo (/cmd_twist -> /cmd_4ws) (B8).

Drives back-and-forth segments so the robot ends near its start pose, and
reports per segment, in the start-of-segment body frame:
  dx, dy [m], dyaw [deg], sim duration, and the steady-state velocity over
  the last 30 % of the segment in SIM time (comparable across RTFs).
Requires genie_sim_chassis_servo_node running.

usage: drive_check.py [segment ...]   (default: all)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

# name, (vx, vy, wz), drive seconds (wall)
SEGMENTS = {
    "fwd": [("vor  x=+0.1", (0.1, 0.0, 0.0), 3.0), ("zurueck x=-0.1", (-0.1, 0.0, 0.0), 3.0)],
    "spin": [("drehen z=+0.5", (0.0, 0.0, 0.5), 4.0), ("drehen z=-0.5", (0.0, 0.0, -0.5), 4.0)],
    "side": [("seitw. y=+0.1", (0.0, 0.1, 0.0), 10.0), ("seitw. y=-0.1", (0.0, -0.1, 0.0), 10.0)],
}
PAUSE = 2.5  # > twist_timeout (1 s): servo stops sending, engine brakes

rclpy.init()
node = rclpy.create_node("drive_check")
pub = node.create_publisher(Twist, "/cmd_twist", 10)
odom = []  # (sim_t, x, y, yaw)
sim_now = [0.0]


def on_clock(m):
    sim_now[0] = m.clock.sec + m.clock.nanosec * 1e-9


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    odom.append((sim_now[0], p.x, p.y, yaw))


node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", on_odom, 50)


def spin_for(sec, twist=None):
    t_end = time.monotonic() + sec
    next_pub = 0.0
    while time.monotonic() < t_end:
        if twist is not None and time.monotonic() >= next_pub:
            pub.publish(twist)
            next_pub = time.monotonic() + 0.05  # 20 Hz
        rclpy.spin_once(node, timeout_sec=0.01)


def body_delta(a, b, yaw0):
    dx, dy = b[1] - a[1], b[2] - a[2]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * dx + s * dy, -s * dx + c * dy


spin_for(1.0)
names = sys.argv[1:] or list(SEGMENTS)
print(f"{'Segment':16s} {'dx':>7s} {'dy':>7s} {'dyaw':>7s} {'t_sim':>6s} | steady: {'vx':>6s} {'vy':>6s} {'wz':>6s}")
for n in names:
    for label, (vx, vy, wz), dur in SEGMENTS[n]:
        i0 = len(odom)
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
        spin_for(dur, tw)
        i1 = len(odom)
        spin_for(PAUSE)
        seg = odom[i0:i1]
        if len(seg) < 10:
            print(f"{label:16s} too few /odom samples ({len(seg)})")
            continue
        a, b = seg[0], seg[-1]
        dx, dy = body_delta(a, b, a[3])
        dyaw = math.degrees(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        k = seg[int(len(seg) * 0.7)]
        ts = b[0] - k[0]
        sx, sy = body_delta(k, b, k[3])
        swz = math.atan2(math.sin(b[3] - k[3]), math.cos(b[3] - k[3]))
        if ts > 1e-6:
            sx, sy, swz = sx / ts, sy / ts, swz / ts
        print(f"{label:16s} {dx:+7.3f} {dy:+7.3f} {dyaw:+7.1f} {b[0] - a[0]:6.2f} | "
              f"{sx:+6.3f} {sy:+6.3f} {swz:+6.3f}")
        # Rest drift after the pause (should be ~0: robot stopped).
        c = odom[-1]
        rx, ry = body_delta(b, c, b[3])
        print(f"{'  (nach Pause)':16s} {rx:+7.3f} {ry:+7.3f}")
pub.publish(Twist())
node.destroy_node()
rclpy.shutdown()
