"""Newton scene plugin: moving test object for the /scan LaserScan (test only).

Activate temporarily in the scene yaml (together with the static test walls):
    newton:
      scene_plugin: /workspace/Doku/scan_test_plugin.py
      entries:
      - {name: scan_wall_front, kind: box, pose: [1.8, 0.0, 0.5, 0, 0, 0, 1], params: {half_extents: [0.05, 1.0, 0.5]}}
      - {name: scan_wall_left,  kind: box, pose: [-0.2, 1.5, 0.5, 0, 0, 0, 1], params: {half_extents: [1.0, 0.05, 0.5]}}

Only on_model_ready / on_post_step: an on_build hook would make the engine
demand a particle solver (setup/model.py), which this rigid scene has not.

Mover: the free bottle body ``iros_beverage_bottle_002`` is driven
kinematically -- its pose is written into state.joint_q / body_q after
every physics step, so contacts and the scan both see it.  Its main hull
(collider hull_001, ~5 cm wide, centre measured at (0.775, -0.01, 1.125)
in the stock scene) is moved to scan height 0.30 m at x = -0.2 (next to
the robot) and swept along y between -2.5 and -0.8 (sine, 6 s sim time).
With the robot at its start pose (x=-0.2, y=0, yaw 0) expect
range(-90 deg) ~= |y_hull| - 0.025.
"""

import math
import time

import numpy as np

MOVER_LABEL = "iros_beverage_bottle_002"
HULL_CENTRE_STOCK = np.array([0.775, -0.01, 1.125])  # from laser_scan debug AABB
TARGET_X, TARGET_Z = -0.2, 0.30
Y_MIN, Y_MAX = -2.5, -0.8
PERIOD_S = 6.0


class ScenePlugin:
    def __init__(self):
        self._body = None
        self._t_log = 0.0
        self._offset = None
        self._rot = None

    def on_model_ready(self, model, ctx):
        labels = list(model.body_label)
        hits = [i for i, lbl in enumerate(labels) if MOVER_LABEL in lbl]
        if not hits:
            others = [lbl for lbl in labels if not lbl.startswith("/genie")]
            ctx.logger.warn(f"[scan_test_plugin] no body matching {MOVER_LABEL!r}; non-robot bodies: {others}")
            return
        self._body = hits[0]
        child = model.joint_child.numpy()
        js = np.nonzero(child == self._body)[0]
        self._joint = int(js[0])
        self._q_start = int(model.joint_q_start.numpy()[self._joint])
        self._qd_start = int(model.joint_qd_start.numpy()[self._joint])
        jtype = int(model.joint_type.numpy()[self._joint])
        ctx.logger.info(
            f"[scan_test_plugin] mover body[{self._body}]={labels[self._body]!r} joint={self._joint} "
            f"type={jtype} q_start={self._q_start}"
        )
        shape_labels = list(getattr(model, "shape_label", []) or [])
        shape_body = model.shape_body.numpy()
        mine = [(i, shape_labels[i], int(shape_body[i])) for i in range(len(shape_labels)) if MOVER_LABEL in shape_labels[i]]
        ctx.logger.info(f"[scan_test_plugin] shapes of {MOVER_LABEL}: {mine}")

    def on_post_step(self, state, sim_time, dt, ctx):
        if self._body is None:
            return
        if self._offset is None:
            q0 = state.body_q.numpy()[self._body]
            self._offset = HULL_CENTRE_STOCK - q0[:3]
            self._rot = q0[3:7].copy()
        now = time.monotonic()
        log_due = now - self._t_log >= 1.0
        if log_due:
            after_step = state.body_q.numpy()[self._body][:3]  # what physics produced
        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * sim_time / PERIOD_S))  # 0..1..0
        y = Y_MIN + (Y_MAX - Y_MIN) * s
        pos = np.array([TARGET_X, y, TARGET_Z]) - self._offset
        pose = np.concatenate([pos, self._rot]).astype(np.float32)
        state.joint_q[self._q_start:self._q_start + 7].assign(pose)
        state.joint_qd[self._qd_start:self._qd_start + 6].zero_()
        state.body_q[self._body:self._body + 1].assign(pose.reshape(1, 7))
        if log_due:
            self._t_log = now
            ctx.logger.info(
                f"[scan_test_plugin] t_sim={sim_time:.2f}s hull y={y:.3f} -> expect range(-90deg)~{abs(y) - 0.025:.3f} m"
                f" | body pos after step={np.round(after_step, 3).tolist()} written={np.round(pos, 3).tolist()}"
            )
