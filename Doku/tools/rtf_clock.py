#!/usr/bin/env python3
"""Mittlerer RTF = Delta Sim-Zeit (/clock) / Delta Wanduhr ueber N Sekunden.

usage (im Container): python3 rtf_clock.py <sekunden>
"""
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node("rtf_clock")
samples = []
node.create_subscription(
    Clock, "/clock", lambda m: samples.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 1000
)
t0 = time.monotonic()
while time.monotonic() - t0 < float(sys.argv[1]):
    rclpy.spin_once(node, timeout_sec=0.05)
if len(samples) < 2:
    sys.exit("== keine /clock-Nachrichten empfangen")
(w0, c0), (w1, c1) = samples[0], samples[-1]
print(
    f"== RTF {(c1 - c0) / (w1 - w0):.3f}  (/clock {len(samples) / (w1 - w0):.1f} Hz, "
    f"sim dt pro Nachricht {(c1 - c0) / (len(samples) - 1) * 1000:.2f} ms)"
)
