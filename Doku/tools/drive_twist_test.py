#!/usr/bin/env python3
"""Fahrtest: /cmd_twist (linear.x) fuer N s mit 20 Hz, dann Stopp.

Gibt /odom-Pose, Radgeschwindigkeiten (/joint_states) und das vom Servo erzeugte
/cmd_4ws aus. Braucht den Servo (Launch mit navigation:=true).

usage (im Container): python3 drive_twist_test.py [vx=0.2] [dauer_s=4]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("drive_twist_test")
last = {}
node.create_subscription(Odometry, "/odom", lambda m: last.__setitem__("odom", m), 10)
node.create_subscription(JointState, "/joint_states", lambda m: last.__setitem__("js", m), qos_profile_sensor_data)
node.create_subscription(JointState, "/cmd_4ws", lambda m: last.__setitem__("c4ws", m), qos_profile_sensor_data)
pub = node.create_publisher(Twist, "/cmd_twist", 10)


def spin(sec, twist=None):
    t = time.monotonic()
    while time.monotonic() - t < sec:
        if twist is not None:
            pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)


def report(tag):
    odom, js = last.get("odom"), last.get("js")
    if odom is None:
        print(f"{tag}: kein /odom")
        return
    p = odom.pose.pose.position
    wheels = {}
    if js is not None and js.velocity:
        wheels = {n: round(v, 3) for n, v in zip(js.name, js.velocity) if "chassis" in n and n.endswith("joint2")}
    print(f"{tag}: odom x={p.x:+.4f} y={p.y:+.4f} vx={odom.twist.twist.linear.x:+.3f} wheel_vel={wheels}")


spin(2)
report("vorher")
cmd = Twist()
cmd.linear.x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
spin(float(sys.argv[2]) if len(sys.argv) > 2 else 4.0, cmd)
report("waehrend")
c4ws = last.get("c4ws")
print("cmd_4ws:", dict(zip(c4ws.name, [round(v, 3) for v in c4ws.velocity])) if c4ws else "nicht empfangen")
spin(2, Twist())
report("nach Stopp")
