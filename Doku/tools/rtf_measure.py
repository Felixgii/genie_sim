#!/usr/bin/env python3
"""Passive RTF / step-interval / scan-rate recorder for the Newton engine (B8).

/rtf is published once per physics step (reliable QoS) with value
dt / wall_delta, so dt / rtf is the engine-measured wall interval between
two step starts.  This node only subscribes -- no extra work in the engine.

usage (in the container, login shell):
    python3 rtf_measure.py <seconds> <tag> [warmup_s] [out_dir]
Writes <out_dir>/res_<tag>.json (summary) and raw_<tag>.csv (wall, rtf).
"""
import collections
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

DT = 0.01  # physics_hz = 100
dur = float(sys.argv[1])
tag = sys.argv[2]
warm = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
out_dir = sys.argv[4] if len(sys.argv) > 4 else "/tmp/rtfbench"
os.makedirs(out_dir, exist_ok=True)

rclpy.init()
node = rclpy.create_node("rtf_measure_" + "".join(c if c.isalnum() else "_" for c in tag))
rtf, clk, scan = [], [], []
t_start = t_end = None

reliable = QoSProfile(depth=5000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


def _recording():
    now = time.monotonic()
    return t_start is not None and t_start <= now <= t_end, now


def on_rtf(m):
    ok, now = _recording()
    if ok:
        rtf.append((now, m.data))


def on_clock(m):
    ok, now = _recording()
    if ok:
        clk.append((now, m.clock.sec + m.clock.nanosec * 1e-9))


def on_scan(m):
    ok, now = _recording()
    if ok:
        r = np.asarray(m.ranges, dtype=np.float64)
        fin = np.isfinite(r)
        scan.append((now, int(fin.sum()), float(r[fin].min()) if fin.any() else math.inf))


node.create_subscription(Float32, "/rtf", on_rtf, reliable)
node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)

t_start = time.monotonic() + warm
t_end = t_start + dur
while time.monotonic() < t_end + 0.5:
    rclpy.spin_once(node, timeout_sec=0.05)

res = {"tag": tag, "duration_s": dur, "rtf_publishers": node.count_publishers("/rtf")}
if res["rtf_publishers"] != 1:
    print(f"WARNING: {res['rtf_publishers']} publishers on /rtf -- more than one engine running?", file=sys.stderr)
if len(clk) > 1:
    (w0, s0), (w1, s1) = clk[0], clk[-1]
    res["rtf_mean"] = (s1 - s0) / (w1 - w0)
    res["sim_steps_expected"] = round((s1 - s0) / DT)
v = np.array([x[1] for x in rtf])
tw = np.array([x[0] for x in rtf])
res["rtf_msgs"] = int(len(v))  # == sim_steps_expected (+-1) => no step missing
if len(v):
    iv = DT / np.clip(v, 1e-6, None) * 1000.0  # step interval [ms]
    res.update(
        iv_mean=float(iv.mean()),
        iv_p50=float(np.percentile(iv, 50)),
        iv_p95=float(np.percentile(iv, 95)),
        iv_p99=float(np.percentile(iv, 99)),
        iv_max=float(iv.max()),
        frac_gt15=float((iv > 15).mean()),
        n_gt30=int((iv > 30).sum()),
        n_gt100=int((iv > 100).sum()),
    )
    hist = collections.Counter(int(min(x // 10, 20)) * 10 for x in iv)
    res["iv_hist_10ms"] = {f"{k}-{k + 10}": hist[k] for k in sorted(hist)}
    big = np.where(iv > 100)[0]
    res["gt100_at_s"] = [[round(float(tw[i] - tw[0]), 2), round(float(iv[i]), 1)] for i in big]
    st = np.where(iv > 30)[0]
    if len(st) > 1:
        res["gt30_spacing_steps_top"] = collections.Counter(int(x) for x in np.diff(st)).most_common(5)
    if scan and len(st):
        sw = np.array([s[0] for s in scan])
        d = np.array([np.abs(sw - tw[i]).min() * 1000.0 for i in st])
        res["gt30_within_20ms_of_scan"] = float((d < 20).mean())
if len(scan) > 1:
    sw = np.array([s[0] for s in scan])
    si = np.diff(sw) * 1000.0
    res.update(scan_n=len(scan), scan_hz=float((len(scan) - 1) / (sw[-1] - sw[0])),
               scan_iv_max_ms=float(si.max()), scan_last_finite=scan[-1][1], scan_last_rmin=scan[-1][2])
else:
    res["scan_n"] = len(scan)

with open(os.path.join(out_dir, f"raw_{tag}.csv"), "w") as f:
    f.write("wall,rtf\n")
    for a, b in rtf:
        f.write(f"{a:.6f},{b:.6f}\n")
with open(os.path.join(out_dir, f"res_{tag}.json"), "w") as f:
    json.dump(res, f, indent=1)
print(json.dumps(res, indent=1))
node.destroy_node()
rclpy.shutdown()
