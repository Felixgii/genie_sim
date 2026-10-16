#!/usr/bin/env python3
"""Prueft ein gestagetes robot.usda (Host-venv, braucht pxr): Physics-Variante + Geometrie.

usage: .venv/bin/python Doku/tools/usd_robot_check.py assets/scenes/<scene>/robot.usda

- Physics-Selection auf der Platte und Rigid Bodies / Joints je Variante
  (ohne Selection komponiert keine Physik-Payload).
- Meshes / Vertices in den Instanz-Prototypen (Traverse() steigt nicht in
  Instance-Proxies ab, deshalb ueber GetPrototypes()).
Liest nur, speichert nichts.
"""
import sys

from pxr import Usd, UsdGeom, UsdPhysics

path = sys.argv[1]
stage = Usd.Stage.Open(path)
robot = stage.GetDefaultPrim()
vset = robot.GetVariantSets().GetVariantSet("Physics")
print(f"Physics-Selection auf der Platte: {vset.GetVariantSelection()!r}")

meshes = verts = 0
for proto in stage.GetPrototypes():
    for prim in Usd.PrimRange(proto):
        if prim.IsA(UsdGeom.Mesh):
            meshes += 1
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            verts += len(pts) if pts else 0
print(f"Prototypen={len(stage.GetPrototypes())} Meshes={meshes} Vertices={verts}")

for sel in ["", "physx", "mujoco", "physics", "none"]:
    if sel:
        vset.SetVariantSelection(sel)
    else:
        vset.ClearVariantSelection()
    prims = list(Usd.PrimRange(robot, Usd.TraverseInstanceProxies()))
    bodies = sum(1 for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI))
    joints = sum(1 for p in prims if p.IsA(UsdPhysics.Joint))
    print(f"  Variante {sel or '(keine)':8} rigid_bodies={bodies:3} joints={joints:3}")
