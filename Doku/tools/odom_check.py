#!/usr/bin/env python3
"""Odometry ground-truth check (B9): how well do /odom and TF match the sim pose?

Drives a closed loop via the servo (/cmd_twist -> /cmd_4ws), all in the free
area behind the start pose (x < -0.2; the shelf front is at x = 0.6 and the
grippers reach x ~ 0.58 at the start pose, so never drive forward or spin there):

  1. straight 1 m (backwards)     4. curve vx=0.15 wz=0.3 (90 deg)
  2. spin +90 deg                 5. sideways vy=0.1, 0.5 m
  3. straight 0.3 m (resets       6. back to the start: spin to target,
     steering, see B7 curve note)    straight, spin to heading 0 (stops at
                                     x = -0.25, 5 cm short of the start)

Records, keyed by header stamp:
  /odom                    reference (engine: body_q[base_link], see B9)
  TF map -> base_footprint tf2 lookup at the /odom stamp (map->odom ∘ odom->base_footprint)
  TF world -> base_footprint
  /tf_render base_link     body pose straight from the engine's body list (frame map)
  wheel odometry           integrated here from /joint_states (steer angle +
                           wheel speed, least squares over 4 wheels, sim time)
                           = what a real odometry without ground truth would give

Prints deviation (position mm, yaw deg) of each source vs /odom per segment
end, at the end and max over the run.  Raw samples -> $OUT/odom_check.csv.
Requires genie_sim_chassis_servo_node running.

usage: odom_check.py [--dry]   (--dry: record 5 s without driving)
"""
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import tf2_ros

OUT = os.environ.get("OUT", "/tmp/odomcheck")
# genie_g2.yaml + /tf_render: wheels at (+-0.23, +-0.218) around base_link.
WHEEL_R = 0.07
WHEELS = {  # suffix -> (x, y) in base_link
    "lwheel_front": (0.23, 0.218), "rwheel_front": (0.23, -0.218),
    "lwheel_rear": (-0.23, 0.218), "rwheel_rear": (-0.23, -0.218),
}
X_GUARD = -0.15  # emergency stop if base_footprint x exceeds this (shelf)
PAUSE = 2.0      # zero twist between segments (explicit zeros, no timeout coast)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


rclpy.init()
node = rclpy.create_node("odom_check")
pub = node.create_publisher(Twist, "/cmd_twist", 10)
tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=600))
tf_lis = tf2_ros.TransformListener(tf_buf, node)

sim_now = [0.0]
odom = []      # (stamp_ns, sim_t, x, y, yaw, vx, vy, wz)
render = {}    # stamp_ns -> (x, y, yaw) of base_link
map_odom = []  # (stamp_ns, x, y, yaw) of every map->odom on /tf
wodom = []     # (sim_t, x, y, yaw) integrated wheel odometry
w_state = {"t": None, "pose": None}
seg_marks = []  # (label, odom index at segment end)

# least squares: per wheel [vx_i; vy_i] = [[1,0,-y_i],[0,1,x_i]] [vx vy wz]
_A = []
for _x, _y in WHEELS.values():
    _A += [[1.0, 0.0, -_y], [0.0, 1.0, _x]]
A_PINV = np.linalg.pinv(np.array(_A))
_jidx = {}


def on_clock(m):
    sim_now[0] = m.clock.sec + m.clock.nanosec * 1e-9


def on_odom(m):
    p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
    odom.append((ns(m.header.stamp), sim_now[0], p.x, p.y, yaw_of(q),
                 t.linear.x, t.linear.y, t.angular.z))
    if w_state["pose"] is None:
        w_state["pose"] = [p.x, p.y, yaw_of(q)]


def on_tf(m):
    for t in m.transforms:
        if t.header.frame_id == "map" and t.child_frame_id == "odom":
            r = t.transform
            map_odom.append((ns(t.header.stamp), r.translation.x, r.translation.y,
                             yaw_of(r.rotation), r.translation.z,
                             r.rotation.x, r.rotation.y))


def on_render(m):
    for t in m.transforms:
        if t.child_frame_id.endswith("/base_link"):
            r = t.transform
            render[ns(t.header.stamp)] = (r.translation.x, r.translation.y, yaw_of(r.rotation))
            return


def on_js(m):
    if not _jidx:
        for i, n in enumerate(m.name):
            for suf in WHEELS:
                if suf in n and n.endswith("joint1"):
                    _jidx[(suf, "steer")] = i
                elif suf in n and n.endswith("joint2"):
                    _jidx[(suf, "drive")] = i
        if len(_jidx) != 8:
            node.get_logger().error(f"wheel joints not found: {_jidx}")
    if w_state["pose"] is None or len(_jidx) != 8:
        return
    t = sim_now[0]
    b = []
    for suf in WHEELS:
        d = m.position[_jidx[(suf, "steer")]]
        v = m.velocity[_jidx[(suf, "drive")]] * WHEEL_R
        b += [v * math.cos(d), v * math.sin(d)]
    vx, vy, wz = A_PINV @ np.array(b)
    if w_state["t"] is not None and t > w_state["t"]:
        dt = t - w_state["t"]
        x, y, th = w_state["pose"]
        thm = th + 0.5 * wz * dt
        x += (vx * math.cos(thm) - vy * math.sin(thm)) * dt
        y += (vx * math.sin(thm) + vy * math.cos(thm)) * dt
        w_state["pose"] = [x, y, wrap(th + wz * dt)]
        wodom.append((t, x, y, w_state["pose"][2]))
    w_state["t"] = t


node.create_subscription(Clock, "/clock", on_clock, 50)
node.create_subscription(Odometry, "/odom", on_odom, 200)
node.create_subscription(TFMessage, "/tf", on_tf, 1000)
node.create_subscription(TFMessage, "/tf_render", on_render, 50)
node.create_subscription(JointState, "/joint_states", on_js, qos_profile_sensor_data)
# Spin in a background thread: /tf_render (~70 transforms @ 100 Hz) + /tf keep a
# spin_once loop ~0.3 s behind, which made the segments overshoot by ~10 deg.
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

aborted = [False]


def pose():
    o = odom[-1]
    return o[2], o[3], o[4]


def send(vx=0.0, vy=0.0, wz=0.0):
    tw = Twist()
    tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
    pub.publish(tw)


def run(label, twist, done, timeout):
    """Publish twist at 20 Hz until done(start_pose, pose) or timeout (wall s)."""
    if aborted[0]:
        return
    p0 = pose()
    t_end = time.monotonic() + timeout
    nxt = 0.0
    while time.monotonic() < t_end:
        time.sleep(0.005)
        p = pose()
        if p[0] > X_GUARD:
            aborted[0] = True
            print(f"!! ABORT in '{label}': x={p[0]:.3f} > {X_GUARD} (shelf guard)")
            break
        if done(p0, p):
            break
        if time.monotonic() >= nxt:
            send(*twist(p0, p) if callable(twist) else twist)
            nxt = time.monotonic() + 0.05
    else:
        print(f"   ('{label}' timeout after {timeout:.0f} s)")
    hold(PAUSE)
    seg_marks.append((label, len(odom) - 1))


def hold(sec):
    t_end = time.monotonic() + sec
    nxt = 0.0
    while time.monotonic() < t_end:
        if time.monotonic() >= nxt:
            send()
            nxt = time.monotonic() + 0.05
        time.sleep(0.005)


def dist(p0, p):
    return math.hypot(p[0] - p0[0], p[1] - p0[1])


def turned(p0, p):
    return abs(wrap(p[2] - p0[2]))


def spin_to(label, yaw_target):
    def tw(p0, p):
        return (0.0, 0.0, 0.5 if wrap(yaw_target - p[2]) > 0 else -0.5)
    run(label, tw, lambda p0, p: abs(wrap(yaw_target - p[2])) < math.radians(2.5), 20)


# ---------------------------------------------------------------------------
t0 = time.monotonic()
while (not odom or not render) and time.monotonic() - t0 < 10:
    time.sleep(0.05)
if not odom:
    sys.exit("no /odom")
for topic in ("/odom", "/tf", "/cmd_twist"):
    infos = node.get_publishers_info_by_topic(topic)
    print(f"publishers {topic}: {sorted({i.node_name for i in infos})}")
x0, y0, th0 = pose()
print(f"start pose /odom: x={x0:+.3f} y={y0:+.3f} yaw={math.degrees(th0):+.1f}")
if x0 > -0.19 or abs(th0) > math.radians(5):
    sys.exit("robot not at the expected start pose (x=-0.2, yaw=0) - restart the sim")

hold(1.0)
seg_marks.append(("Start", len(odom) - 1))
if "--dry" in sys.argv:
    hold(5.0)
    seg_marks.append(("Stand 5 s", len(odom) - 1))
else:
    run("1 gerade -1 m", (-0.2, 0.0, 0.0), lambda p0, p: dist(p0, p) >= 0.985, 15)
    run("2 drehen +90", (0.0, 0.0, 0.5), lambda p0, p: turned(p0, p) >= math.radians(87), 15)
    run("3 gerade 0.3 m", (0.15, 0.0, 0.0), lambda p0, p: dist(p0, p) >= 0.29, 10)
    run("4 Kurve 90", (0.15, 0.0, 0.3), lambda p0, p: turned(p0, p) >= math.radians(88), 15)
    run("5 seitw. 0.5 m", (0.0, 0.1, 0.0), lambda p0, p: dist(p0, p) >= 0.49, 20)
    tx, ty = -0.25, 0.0
    px, py, _ = pose()
    spin_to("6a drehen z. Ziel", math.atan2(ty - py, tx - px))
    px, py, _ = pose()
    d_goal = math.hypot(tx - px, ty - py)
    run("6b gerade z. Ziel", (0.2, 0.0, 0.0), lambda p0, p: dist(p0, p) >= d_goal - 0.015, 20)
    spin_to("6c drehen auf 0", 0.0)
hold(0.5)
t_run = time.monotonic() - t0

# ---------------------------------------------------------------------------
# Evaluation.  Every source vs /odom at the same tick.
time.sleep(0.5)
executor.shutdown()  # freeze the recordings; tf_buf lookups need no spinning
keys = sorted(render)
rk = np.array(keys, dtype=np.int64)


def near_render(st):
    i = int(np.searchsorted(rk, st))
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(rk) and (best is None or abs(rk[j] - st) < abs(rk[best] - st)):
            best = j
    if best is None or abs(rk[best] - st) > 5_000_000:  # same tick (< 5 ms)
        return None
    return render[keys[best]]


wt = np.array([w[0] for w in wodom]) if wodom else np.zeros(0)


def near_wheel(sim_t):
    if len(wt) == 0:
        return None
    i = min(int(np.searchsorted(wt, sim_t)), len(wt) - 1)
    return wodom[i][1:]


rows = []
n_tf_fail = 0
for o in odom:
    st, simt, x, y, th = o[:5]
    rec = {"stamp_ns": st, "sim_t": simt, "odom_x": x, "odom_y": y, "odom_yaw": th,
           "tw_vx": o[5], "tw_vy": o[6], "tw_wz": o[7]}
    for fr in ("map", "world"):
        try:
            tr = tf_buf.lookup_transform(fr, "base_footprint", Time(nanoseconds=st))
            t_ = tr.transform
            rec[f"{fr}_x"], rec[f"{fr}_y"], rec[f"{fr}_yaw"] = (
                t_.translation.x, t_.translation.y, yaw_of(t_.rotation))
        except Exception:
            n_tf_fail += 1
    r = near_render(st)
    if r:
        rec["render_x"], rec["render_y"], rec["render_yaw"] = r
    w = near_wheel(simt)
    if w:
        rec["wheel_x"], rec["wheel_y"], rec["wheel_yaw"] = w
    rows.append(rec)

os.makedirs(OUT, exist_ok=True)
cols = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("stamp_ns", "sim_t"), k))
with open(os.path.join(OUT, "odom_check.csv"), "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=cols)
    wr.writeheader()
    wr.writerows(rows)

SRC = [("TF map->base_footprint", "map"), ("TF world->base_footprint", "world"),
       ("/tf_render base_link", "render"), ("Rad-Odometrie (integriert)", "wheel")]


def err(r, s):
    if f"{s}_x" not in r:
        return None
    return (1000 * math.hypot(r[f"{s}_x"] - r["odom_x"], r[f"{s}_y"] - r["odom_y"]),
            math.degrees(abs(wrap(r[f"{s}_yaw"] - r["odom_yaw"]))))


print(f"\nrun {t_run:.0f} s wall, sim {odom[-1][1] - odom[0][1]:.1f} s, "
      f"{len(odom)} /odom, {len(render)} /tf_render, {len(wodom)} wheel samples, "
      f"tf lookup failures {n_tf_fail}")
print("\nSegment-Enden (/odom-Pose; Abweichung der Quellen zu /odom: mm / deg)")
hdr = f"{'Segment':20s} {'x':>7s} {'y':>7s} {'yaw':>7s} |" + "".join(f" {s[1]:>14s}" for s in SRC)
print(hdr)
for label, i in seg_marks:
    r = rows[i]
    line = f"{label:20s} {r['odom_x']:+7.3f} {r['odom_y']:+7.3f} {math.degrees(r['odom_yaw']):+7.1f} |"
    for _, s in SRC:
        e = err(r, s)
        line += f" {e[0]:7.1f}/{e[1]:5.2f}" if e else f" {'-':>14s}"
    print(line)

print("\nZusammenfassung (Abweichung zu /odom)")
print(f"{'Quelle':28s} {'Ende mm':>9s} {'Ende deg':>9s} {'max mm':>9s} {'max deg':>9s} {'n':>6s}")
for name, s in SRC:
    es = [e for e in (err(r, s) for r in rows) if e]
    if not es:
        print(f"{name:28s} keine Daten")
        continue
    end = es[-1]
    print(f"{name:28s} {end[0]:9.2f} {end[1]:9.3f} {max(e[0] for e in es):9.2f} "
          f"{max(e[1] for e in es):9.3f} {len(es):6d}")

if map_odom:
    mo = np.array(map_odom)
    print(f"\nmap->odom auf /tf: {len(mo)} Nachrichten, max |t_xy| = "
          f"{1000 * np.max(np.hypot(mo[:, 1], mo[:, 2])):.3f} mm, max |t_z| = "
          f"{1000 * np.max(np.abs(mo[:, 4])):.3f} mm, max |yaw| = "
          f"{np.degrees(np.max(np.abs(mo[:, 3]))):.4f} deg, max |q_x|,|q_y| = "
          f"{np.max(np.abs(mo[:, 5:7])):.2e}")
else:
    print("\nmap->odom: keine Nachricht auf /tf")
o = np.array([r[:8] for r in odom])
dts = np.diff(o[:, 0]) / 1e6
print(f"/odom: Rate {len(o) / ((o[-1, 0] - o[0, 0]) / 1e9):.1f} Hz (Stempel), dt p50/p99/max = "
      f"{np.percentile(dts, 50):.1f}/{np.percentile(dts, 99):.1f}/{dts.max():.1f} ms")
print(f"/odom twist: max |vx| {np.abs(o[:, 5]).max():.3f}, max |vy| {np.abs(o[:, 6]).max():.3f} m/s, "
      f"max |wz| {np.abs(o[:, 7]).max():.4f} rad/s")
# twist.angular.z vs yaw rate from consecutive poses (stamp dt).
_dts = np.diff(o[:, 0]) / 1e9
_ok = _dts > 1e-6
_rate = np.arctan2(np.sin(np.diff(o[:, 4])), np.cos(np.diff(o[:, 4])))[_ok] / _dts[_ok]
_err = np.abs(o[1:, 7][_ok] - _rate)
print(f"/odom twist.angular.z vs Posen-Gierrate: |Fehler| p50 {np.percentile(_err, 50):.4f}, "
      f"p99 {np.percentile(_err, 99):.4f}, max {_err.max():.4f} rad/s "
      f"(wz bei Drehung p50 {np.percentile(np.abs(o[:, 7][np.abs(o[:, 7]) > 0.1]), 50) if np.any(np.abs(o[:, 7]) > 0.1) else 0:.3f})")
# Stale samples: a /odom pose that steps back against the motion of its
# neighbours (AsyncMirror slot read before its copy landed, see B9).
P = o[:, 2:5]
d = np.diff(P, axis=0)
d[:, 2] = np.arctan2(np.sin(d[:, 2]), np.cos(d[:, 2]))
back_xy = back_yaw = 0
for k in range(1, len(d) - 1):
    a, b, c = d[k - 1], d[k], d[k + 1]
    if np.hypot(*b[:2]) > 2e-4 and np.dot(a[:2], b[:2]) < 0 and np.dot(b[:2], c[:2]) < 0:
        back_xy += 1
    if abs(b[2]) > 5e-4 and a[2] * b[2] < 0 and b[2] * c[2] < 0:
        back_yaw += 1
moving = int(np.sum((np.hypot(d[:, 0], d[:, 1]) > 2e-4) | (np.abs(d[:, 2]) > 5e-4)))
print(f"/odom Ruecksprünge (veraltete Samples): xy {back_xy}, yaw {back_yaw} von {moving} Samples in Bewegung")
print(f"CSV: {os.path.join(OUT, 'odom_check.csv')}")
send()
node.destroy_node()
rclpy.shutdown()
