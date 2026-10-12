# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Planar 2D laser scan (``sensor_msgs/LaserScan``) for newton-standalone.

Kit-free / RTX-free: raycasts on the GPU with Newton's own Warp raytracer
(``newton.sensors.SensorTiledCamera``) against the model's shape BVH,
refit from ``state.body_q`` before every scan -- so moving bodies are
seen at their current pose, not a static map.

Enabled by the ``newton.sensors.laser_scan`` block in the scene yaml
(absent / ``enabled: false`` => module never imported, zero cost)::

    newton:
      sensors:
        laser_scan:
          enabled: true
          topic: /scan
          frame_id: base_scan          # new frame, static child of parent_frame
          parent_frame: base_footprint
          height: 0.3                  # [m] above base_footprint (ground)
          num_rays: 360                # evenly spaced over 360 deg, first ray at -pi
          range_min: 0.0               # [m]; 0 = auto (smallest per-ray start offset)
          range_max: 10.0              # [m]
          rate_hz: 10.0                # wall-clock rate (stamps are wall-clock too)
          dead_zone_margin: 0.02       # [m] start rays this far outside the robot

Design notes:

* **Geometry**: the BVH is rebuilt with ``ShapeFlags.COLLIDE_SHAPES`` --
  the scan sees collision geometry, not render meshes.  The model has a
  single shape BVH, so any other ``SensorTiledCamera`` on this model
  would see the same shapes.  Nothing else in the engine uses it.
* **Convex hulls**: Newton 1.5's raytracer never hits ``CONVEX_MESH``
  shapes (see ``_route_convex_meshes_to_mesh_path``); they are routed
  to the triangle-mesh intersection for this sensor only.
* **Pose**: scanner = base_link's world pose ground-projected (x, y,
  yaw), lifted by ``height`` -- the same projection the C++ bridge uses
  for ``odom -> base_footprint``, so ``base_footprint -> base_scan`` is a
  pure static z-offset.  Computed on the GPU from the very ``body_q``
  the rays are cast against.
* **Self-hits**: the robot's own collision shapes are in the BVH.  On
  the first scan each ray is cast from the scanner centre; while the
  closest hit is a robot shape the ray start is pushed past it (plus
  ``dead_zone_margin``).  Rays then start on that per-ray "dead zone"
  outline.  At runtime any remaining robot hit (e.g. an arm swinging
  into the plane) is reported as no-return (+inf).
* **Timing**: gated on wall clock and stamped with the rclpy node's
  wall clock, matching the C++ bridge (``node_->now()``).  One
  device sync per scan (the ``numpy()`` readback).
* No noise model, no beam divergence, no intensities.

``GENIESIM_LASER_SCAN_DEBUG=1`` logs, once, every non-robot shape the
scan can hit and every robot collider near scan height (world AABBs).
"""

from __future__ import annotations

import math
import os
import time
from typing import Any, Optional

import numpy as np
import warp as wp

_DEFAULTS = {
    "topic": "/scan",
    "frame_id": "base_scan",
    "parent_frame": "base_footprint",
    "height": 0.3,
    "num_rays": 360,
    "range_min": 0.0,
    "range_max": 10.0,
    "rate_hz": 10.0,
    "dead_zone_margin": 0.02,
}

# Robot-calibration passes: each pass pushes rays past one more robot shape.
_CALIB_MAX_PASSES = 8


@wp.kernel(enable_backward=False)
def _scanner_pose_kernel(
    body_q: wp.array[wp.transform],
    base_idx: int,
    height: float,
    out: wp.array2d[wp.transformf],
):
    """Ground-projected base_link pose (x, y, yaw) lifted to ``height``."""
    w = wp.tid()
    tf = body_q[base_idx]
    p = wp.transform_get_translation(tf)
    q = wp.transform_get_rotation(tf)  # (x, y, z, w)
    yaw = wp.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
    q_yaw = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), yaw)
    out[0, w] = wp.transformf(wp.vec3f(p[0], p[1], height), wp.quatf(q_yaw))


def _cfg_from_scene(scene_cfg: dict) -> Optional[dict]:
    raw = ((scene_cfg.get("newton") or {}).get("sensors") or {}).get("laser_scan")
    if not raw or not bool(raw.get("enabled", True)):
        return None
    cfg = dict(_DEFAULTS)
    cfg.update({k: v for k, v in raw.items() if k != "enabled"})
    return cfg


class LaserScanSensor:
    """One planar scanner rigidly attached to the robot's base footprint."""

    def __init__(self, model: Any, state: Any, cfg: dict, robot_prefix: str, logger: Any):
        from engine.newton.visualizers.markers import _get_or_create_node  # noqa: PLC0415
        from geometry_msgs.msg import TransformStamped  # noqa: PLC0415
        from newton import ShapeFlags  # noqa: PLC0415
        from newton.sensors import SensorTiledCamera  # noqa: PLC0415
        from rclpy.qos import qos_profile_sensor_data  # noqa: PLC0415
        from sensor_msgs.msg import LaserScan  # noqa: PLC0415
        from tf2_ros import StaticTransformBroadcaster  # noqa: PLC0415

        self._model = model
        self._logger = logger
        self._frame_id = str(cfg["frame_id"])
        self._height = float(cfg["height"])
        self._n = int(cfg["num_rays"])
        self._range_min_cfg = float(cfg["range_min"])
        self._range_max = float(cfg["range_max"])
        self._period = 1.0 / float(cfg["rate_hz"])
        self._margin = float(cfg["dead_zone_margin"])
        self._next_t = 0.0
        self._calibrated = False
        device = model.device

        # --- robot bodies / shapes (label prefix, same rule as markers.py) ---
        labels = list(getattr(model, "body_label", []) or [])
        prefix, exact = f"/{robot_prefix}/", f"/{robot_prefix}"
        robot_body = np.array([lbl.startswith(prefix) or lbl == exact for lbl in labels], dtype=bool)
        base_idx = next((i for i, lbl in enumerate(labels) if robot_body[i] and lbl.endswith("/base_link")), -1)
        if base_idx < 0:
            raise RuntimeError(f"laser_scan: no robot body ending in /base_link under {prefix!r}")
        self._base_idx = base_idx
        shape_body = model.shape_body.numpy()
        self._robot_shape = np.zeros(model.shape_count + 1, dtype=bool)  # +1: index for "no hit"
        self._robot_shape[:-1] = (shape_body >= 0) & robot_body[np.clip(shape_body, 0, None)]

        # --- collision-geometry BVH + raytracer ---
        model.bvh_build_shapes(state, shape_flags=ShapeFlags.COLLIDE_SHAPES)
        self._sensor = SensorTiledCamera(model, load_textures=False)
        n_convex = self._route_convex_meshes_to_mesh_path(model)
        self._depth = self._sensor.utils.create_depth_image_output(self._n, 1)
        self._shape_idx = self._sensor.utils.create_shape_index_image_output(self._n, 1)
        self._cam_tf = wp.zeros((1, model.world_count), dtype=wp.transformf, device=device)

        self._angle_min = -math.pi
        self._angle_inc = 2.0 * math.pi / self._n
        ang = self._angle_min + self._angle_inc * np.arange(self._n)
        self._dirs = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], axis=1).astype(np.float32)
        self._offset = np.zeros(self._n, dtype=np.float32)
        self._rays = wp.zeros((1, 1, self._n, 2), dtype=wp.vec3f, device=device)
        self._upload_rays()

        # --- ROS ---
        node = _get_or_create_node()
        self._node = node
        self._pub = node.create_publisher(LaserScan, str(cfg["topic"]), qos_profile_sensor_data)
        self._msg = LaserScan()
        self._msg.header.frame_id = self._frame_id
        self._msg.angle_min = self._angle_min
        self._msg.angle_max = self._angle_min + self._angle_inc * (self._n - 1)
        self._msg.angle_increment = self._angle_inc
        self._msg.time_increment = 0.0
        self._msg.scan_time = self._period
        self._msg.range_max = self._range_max

        self._tf_static = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = str(cfg["parent_frame"])
        t.child_frame_id = self._frame_id
        t.transform.translation.z = self._height
        t.transform.rotation.w = 1.0
        self._tf_static.sendTransform(t)

        n_enabled = int(model.bvh_shape_count_enabled)
        logger.info(
            f"[newton-standalone] laser_scan: ON (topic={cfg['topic']!r}, frame={self._frame_id!r} "
            f"= {cfg['parent_frame']!r} + z {self._height:.3f} m, rays={self._n}, "
            f"range_max={self._range_max:.1f} m, rate={1.0 / self._period:.1f} Hz wall, "
            f"BVH=COLLIDE_SHAPES {n_enabled}/{model.shape_count} shapes, "
            f"robot shapes={int(self._robot_shape.sum())}, base body={labels[base_idx]!r}, "
            f"convex hulls routed to mesh raycast={n_convex})"
        )

    def _route_convex_meshes_to_mesh_path(self, model: Any) -> int:
        """Make the raytracer hit CONVEX_MESH shapes (Newton 1.5 workaround).

        The render kernel only takes the mesh-intersection path for
        ``GeoType.MESH``; every other type goes to
        ``raycast.ray_intersect_shape``, which handles primitives only, so
        convex-hull colliders (the usual USD collision approximation) are
        never hit.  Their ``shape_source_ptr`` is a ``wp.Mesh`` like MESH,
        so -- exactly as ``RenderContext`` already does for HFIELD -- give
        the renderer its own type array with CONVEX_MESH remapped to MESH.
        ``model.shape_type`` (used by collision) is not touched.
        """
        from newton import GeoType  # noqa: PLC0415

        types = model.shape_type.numpy()
        convex = types == int(GeoType.CONVEX_MESH)
        if not convex.any():
            return 0
        ctx = getattr(self._sensor, "_SensorTiledCamera__render_context", None)
        if ctx is None or not hasattr(ctx, "shape_render_type"):
            self._logger.warn(
                "[newton-standalone] laser_scan: cannot reach the raytracer's shape_render_type; "
                f"{int(convex.sum())} convex-hull shapes will be invisible to the scan"
            )
            return 0
        render_types = ctx.shape_render_type.numpy().copy()
        render_types[convex] = int(GeoType.MESH)
        ctx.shape_render_type = wp.array(render_types, dtype=wp.int32, device=model.device)
        return int(convex.sum())

    def _upload_rays(self) -> None:
        rays = np.empty((1, 1, self._n, 2, 3), dtype=np.float32)
        rays[0, 0, :, 0, :] = self._dirs * self._offset[:, None]  # origin
        rays[0, 0, :, 1, :] = self._dirs  # direction
        self._rays.assign(rays)

    def _cast(self, state: Any) -> tuple[np.ndarray, np.ndarray]:
        self._model.bvh_refit_shapes(state)
        wp.launch(
            _scanner_pose_kernel,
            dim=self._model.world_count,
            inputs=[state.body_q, self._base_idx, self._height, self._cam_tf],
            device=self._model.device,
        )
        self._sensor.update(
            state,
            self._cam_tf,
            self._rays,
            depth_image=self._depth,
            shape_index_image=self._shape_idx,
        )
        # A miss leaves ClearData's values: shape index NO_HIT (0xFFFFFFFF)
        # and depth 0.0 (not negative) -- so classify on the shape index.
        depth = self._depth.numpy()[0, 0, 0]
        shape = self._shape_idx.numpy()[0, 0, 0].astype(np.int64)
        miss = shape >= self._model.shape_count
        shape[miss] = self._model.shape_count
        depth = np.where(miss, -1.0, depth)
        return depth, shape

    def _calibrate(self, state: Any) -> None:
        """Push each ray's start past the robot's own shapes (see module doc)."""
        if os.environ.get("GENIESIM_LASER_SCAN_DEBUG", "0") != "0":
            self._log_bvh_shapes(state)
        for _ in range(_CALIB_MAX_PASSES):
            depth, shape = self._cast(state)
            hit_robot = (depth >= 0.0) & self._robot_shape[shape]
            if not hit_robot.any():
                break
            self._offset[hit_robot] += depth[hit_robot] + self._margin
            self._upload_rays()
        self._range_min = self._range_min_cfg if self._range_min_cfg > 0.0 else float(self._offset.min())
        self._msg.range_min = self._range_min

        def at(deg: float) -> float:
            return float(self._offset[int(round((math.radians(deg) - self._angle_min) / self._angle_inc)) % self._n])

        if self._offset.max() <= 0.0:
            detail = "none -- no robot collider at scan height, rays start at the scanner centre"
        else:
            detail = f"ray start from scanner centre, incl. {self._margin * 100:.0f} cm margin"
        self._logger.info(
            f"[newton-standalone] laser_scan: dead zone ({detail}): front {at(0) * 100:.1f} cm, "
            f"left {at(90) * 100:.1f} cm, rear {at(180) * 100:.1f} cm, right {at(-90) * 100:.1f} cm, "
            f"min {self._offset.min() * 100:.1f} / max {self._offset.max() * 100:.1f} cm; "
            f"range_min={self._range_min:.3f} m"
        )
        self._calibrated = True

    def _log_bvh_shapes(self, state: Any) -> None:
        """Debug: list the non-robot shapes the scan can hit, with world AABBs."""
        m = self._model
        self._model.bvh_refit_shapes(state)
        enabled = m.bvh_shape_enabled.numpy()[: m.bvh_shape_count_enabled]
        lowers, uppers = m.bvh_shapes.lowers.numpy(), m.bvh_shapes.uppers.numpy()
        labels = list(getattr(m, "shape_label", []) or [])
        types = m.shape_type.numpy()
        worlds = m.shape_world.numpy() if getattr(m, "shape_world", None) is not None else None
        lines = []
        for k, s in enumerate(enabled):
            lbl = labels[s] if s < len(labels) else "?"
            if self._robot_shape[s]:
                lo, hi = lowers[k], uppers[k]
                if lo[2] < 2.0 * self._height:
                    at_h = "AT scan height" if lo[2] <= self._height <= hi[2] else "below/above"
                    lines.append(
                        f"    ROBOT shape[{s}] type={int(types[s])} {lbl!r} {at_h}: "
                        f"aabb=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})..({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})"
                    )
                continue
            w = int(worlds[s]) if worlds is not None else "?"
            lo, hi = lowers[k], uppers[k]
            lines.append(
                f"    shape[{s}] type={int(types[s])} world={w} {lbl!r} "
                f"aabb=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})..({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})"
            )
        n_robot = sum(1 for ln in lines if ln.lstrip().startswith("ROBOT"))
        self._logger.info(
            f"[newton-standalone] laser_scan debug: world_count={m.world_count}, "
            f"{len(lines) - n_robot} non-robot shapes in BVH, {n_robot} robot colliders below "
            f"{2.0 * self._height:.2f} m:\n" + "\n".join(lines)
        )

    def maybe_publish(self, state: Any) -> None:
        now = time.monotonic()
        if now < self._next_t:
            return
        # Fixed period; snap forward after long stalls instead of bursting.
        self._next_t = max(self._next_t + self._period, now + 0.5 * self._period)
        if not self._calibrated:
            self._calibrate(state)
        depth, shape = self._cast(state)
        ranges = (self._offset + depth).astype(np.float32)
        no_return = (depth < 0.0) | self._robot_shape[shape] | (ranges > self._range_max)
        ranges[no_return] = np.inf
        self._msg.header.stamp = self._node.get_clock().now().to_msg()
        self._msg.ranges = ranges.tolist()
        self._pub.publish(self._msg)


def maybe_create_laser_scan(engine: Any) -> Optional[LaserScanSensor]:
    """Build the sensor if ``newton.sensors.laser_scan`` is set; never raises."""
    cfg = _cfg_from_scene(engine._scene_cfg)
    if cfg is None:
        return None
    try:
        return LaserScanSensor(
            engine._model, engine._state_0, cfg, str(engine._robot_prefix_str or ""), engine._logger
        )
    except Exception as exc:  # noqa: BLE001
        engine._logger.warn(f"[newton-standalone] laser_scan requested but init failed: {exc!r}; continuing without it")
        return None
