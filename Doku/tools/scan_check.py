#!/usr/bin/env python3
"""Laser-scan short check against the B6 test walls (B8).

Needs the temporary B6 test walls in the scene yaml (newton.entries):
inner faces at x = WALL_X (front) and y = WALL_Y (left).  Prints header
info, then r(0 deg) vs. WALL_X - x_odom and r(+90 deg) vs. WALL_Y - y_odom
at standstill and while driving toward the front wall (/cmd_twist, servo).
Assumes yaw ~ 0 (robot at its start pose).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

WALL_X, WALL_Y = 1.75, 1.45

rclpy.init()
node = rclpy.create_node("scan_check")
pub = node.create_publisher(Twist, "/cmd_twist", 10)
st = {"scan": None, "odom": None, "n_scan": 0}


def on_scan(m):
    st["scan"] = m
    st["n_scan"] += 1


def on_odom(m):
    st["odom"] = m


node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", on_odom, 50)


def ray(m, deg):
    i = int(round((math.radians(deg) - m.angle_min) / m.angle_increment)) % len(m.ranges)
    return m.ranges[i]


def spin_for(sec, twist=None):
    t_end = time.monotonic() + sec
    next_pub = 0.0
    while time.monotonic() < t_end:
        if twist is not None and time.monotonic() >= next_pub:
            pub.publish(twist)
            next_pub = time.monotonic() + 0.05  # 20 Hz
        rclpy.spin_once(node, timeout_sec=0.01)


def sample(label):
    m, o = st["scan"], st["odom"]
    x, y = o.pose.pose.position.x, o.pose.pose.position.y
    r0, r90 = ray(m, 0), ray(m, 90)
    print(f"{label:10s} x={x:+.3f} y={y:+.3f} | r(0)={r0:.3f} soll={WALL_X - x:.3f} "
          f"diff={(r0 - (WALL_X - x)) * 1000:+.1f} mm | r(90)={r90:.3f} soll={WALL_Y - y:.3f} "
          f"diff={(r90 - (WALL_Y - y)) * 1000:+.1f} mm")


spin_for(1.0)
n0, t0 = st["n_scan"], time.monotonic()
spin_for(5.0)
m = st["scan"]
fin = sum(1 for r in m.ranges if math.isfinite(r))
print(f"/scan: {(st['n_scan'] - n0) / (time.monotonic() - t0):.2f} Hz, frame={m.header.frame_id}, "
      f"rays={len(m.ranges)}, range_min={m.range_min:.3f}, range_max={m.range_max:.1f}, finite={fin}")
sample("Stand")
tw = Twist()
tw.linear.x = 0.1
# Max ~0.4 m forward: in B6 the robot hit the bottom shelf board after 0.5 m.
for k in range(4):
    spin_for(1.0, tw)
    sample(f"Fahrt {k + 1}s")
pub.publish(Twist())
spin_for(2.5)
sample("Stand 2")
tw.linear.x = -0.1
spin_for(4.0, tw)
pub.publish(Twist())
spin_for(2.5)
sample("zurueck")
node.destroy_node()
rclpy.shutdown()
