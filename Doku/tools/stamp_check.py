#!/usr/bin/env python3
"""Stamp check (B10): do all engine stamps sit on the sim-time grid and match /clock?

Records for DURATION seconds (wall) /clock, /odom, /tf (engine + RSP),
/joint_states, /tf_render, /scan and reports per topic:
  * n, rate (wall), clock mode: "sim" if stamps are far from wall time
  * grid: fraction of stamps that are a multiple of dt (1 us tolerance)
  * in_clock: fraction of stamps that equal a /clock value
  * age at receipt: latest /clock received before the message minus its
    stamp (0 = same step; the async scan is typically >= 1 tick)
/clock itself: strictly monotonic? gaps (diff != dt)? publisher count.

tf2 test (node with use_sim_time:=true, own listener thread), every 50 ms:
  * lookup odom <- base_scan at the newest /scan stamp      (costmap style)
  * lookup map  <- gripper_l_base_link at Time() (latest)  (RViz style)
  * lookup map  <- base_scan at now(), no timeout           (strict)
  * lookup map  <- base_scan at now(), 50 ms timeout
and counts failures per exception type.

usage: stamp_check.py [DURATION=20] [DT=0.01]
"""
import sys
import threading
import time
from collections import defaultdict

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
DT_NS = int(round((float(sys.argv[2]) if len(sys.argv) > 2 else 0.01) * 1e9))


def ns(st):
    return st.sec * 1_000_000_000 + st.nanosec


rclpy.init()
node = rclpy.create_node("stamp_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
lock = threading.Lock()
clock_vals = []          # (stamp_ns, wall) every /clock
latest_clock = [None]
rec = defaultdict(list)  # topic -> [(stamp_ns, wall, latest_clock_ns)]


def on_clock(m):
    c = ns(m.clock)
    with lock:
        clock_vals.append((c, time.time()))
        latest_clock[0] = c


def mk(topic):
    def cb(m):
        with lock:
            rec[topic].append((ns(m.header.stamp), time.time(), latest_clock[0]))
    return cb


def on_tf(m):
    with lock:
        for t in m.transforms:
            key = "/tf engine" if t.child_frame_id in ("base_footprint", "base_link") and \
                t.header.frame_id in ("odom", "base_footprint", "map") else "/tf RSP"
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                key = "/tf map->odom"
            rec[key].append((ns(t.header.stamp), time.time(), latest_clock[0]))


scan_stamp = [None]


def on_scan(m):
    mk("/scan")(m)
    scan_stamp[0] = m.header.stamp


node.create_subscription(Clock, "/clock", on_clock, 100)
node.create_subscription(Odometry, "/odom", mk("/odom"), 100)
node.create_subscription(TFMessage, "/tf", on_tf, 1000)
node.create_subscription(JointState, "/joint_states", mk("/joint_states"), qos_profile_sensor_data)
node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
render_seen = []


def on_render(m):
    if m.transforms:
        with lock:
            rec["/tf_render"].append((ns(m.transforms[0].header.stamp), time.time(), latest_clock[0]))


node.create_subscription(TFMessage, "/tf_render", on_render, 50)

tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
tf_lis = tf2_ros.TransformListener(tf_buf, node, spin_thread=True)
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

time.sleep(2.0)  # warm up subscriptions + TF buffer
with lock:
    for v in rec.values():
        v.clear()
    clock_vals.clear()

tf_res = defaultdict(lambda: defaultdict(int))
t_end = time.monotonic() + DURATION
while time.monotonic() < t_end:
    tests = []
    if scan_stamp[0] is not None:
        tests.append(("odom<-base_scan @scan.stamp", "odom", "base_scan", Time.from_msg(scan_stamp[0]), None))
    tests.append(("map<-gripper_l @latest", "map", "gripper_l_base_link", Time(), None))
    now = node.get_clock().now()
    tests.append(("map<-base_scan @now", "map", "base_scan", now, None))
    tests.append(("map<-base_scan @now +50ms", "map", "base_scan", now, Duration(seconds=0.05)))
    for name, tgt, src, t, to in tests:
        try:
            if to is None:
                tf_buf.lookup_transform(tgt, src, t)
            else:
                tf_buf.lookup_transform(tgt, src, t, timeout=to)
            tf_res[name]["ok"] += 1
        except Exception as e:  # noqa: BLE001
            tf_res[name][type(e).__name__] += 1
    time.sleep(0.05)

executor.shutdown()
with lock:
    clocks = list(clock_vals)
    data = {k: list(v) for k, v in rec.items()}

now_wall = time.time()
cset = {c for c, _ in clocks}
print(f"publishers /clock: {sorted({i.node_name for i in node.get_publishers_info_by_topic('/clock')})}")
if clocks:
    cs = [c for c, _ in clocks]
    d = [b - a for a, b in zip(cs, cs[1:])]
    mono = all(x > 0 for x in d)
    gaps = sum(1 for x in d if x != DT_NS)
    span = (clocks[-1][1] - clocks[0][1]) or 1e-9
    sim_span = (cs[-1] - cs[0]) / 1e9
    mode = "sim" if abs(cs[-1] / 1e9 - now_wall) > 1e5 else "wall"
    print(f"/clock: n={len(cs)} rate={len(cs) / span:.1f} Hz (wall)  monotonic={mono}  "
          f"steps!=dt: {gaps} (max diff {max(d) / 1e6 if d else 0:.1f} ms)  "
          f"sim/wall={sim_span / span:.3f}  mode={mode}  last={cs[-1] / 1e9:.3f}")
else:
    print("/clock: NO MESSAGES")

print(f"\n{'Topic':16s} {'n':>6s} {'Hz':>6s} {'mode':>5s} {'grid':>7s} {'in_clock':>9s} "
      f"{'age p50':>8s} {'age max':>8s} {'age<0':>6s}")
for k in ("/odom", "/tf engine", "/tf map->odom", "/tf RSP", "/joint_states", "/tf_render", "/scan"):
    v = data.get(k, [])
    if not v:
        print(f"{k:16s} {'0':>6s}")
        continue
    st = [s for s, _, _ in v]
    walls = [w for _, w, _ in v]
    span = (walls[-1] - walls[0]) or 1e-9
    mode = "sim" if abs(st[-1] / 1e9 - now_wall) > 1e5 else "wall"
    grid = sum(1 for s in st if min(s % DT_NS, DT_NS - s % DT_NS) <= 1000) / len(st)
    inc = sum(1 for s in st if s in cset) / len(st)
    ages = sorted((lc - s) / 1e6 for s, _, lc in v if lc is not None)
    neg = sum(1 for a in ages if a < 0)
    p50 = ages[len(ages) // 2] if ages else float("nan")
    mx = ages[-1] if ages else float("nan")
    if mode == "wall":
        off = sorted((w - s / 1e9) * 1000 for s, w, _ in v)
        p50, mx = off[len(off) // 2], off[-1]
    print(f"{k:16s} {len(v):6d} {len(v) / span:6.1f} {mode:>5s} {grid:7.1%} {inc:9.1%} "
          f"{p50:8.1f} {mx:8.1f} {neg:6d}")
print("(age = latest /clock at receipt - stamp [ms]; in wall mode: receipt wall - stamp [ms])")

print("\ntf2 (use_sim_time=true):")
for name, r in tf_res.items():
    print(f"  {name:28s} " + ", ".join(f"{k}={v}" for k, v in sorted(r.items())))
node.destroy_node()
rclpy.shutdown()
