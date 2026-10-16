# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Restore the control-target names the engine is written against (Newton 1.6.1).

Upstream bug: the engine reads and writes ``Control.joint_target_pos`` /
``Control.joint_target_vel`` as DOF-indexed buffers.  The Newton 1.6.1 that the
unpinned ``isaacsim`` install pulls in renamed them to ``joint_target_q`` /
``joint_target_qd`` (the old names raise) and sizes ``joint_target_q`` by
``joint_coord_count`` by default.  ``getattr(control, "joint_target_pos", None)``
then returns ``None`` and every ``/joint_command`` and ``/cmd_4ws`` write is
silently dropped -- arms do not hold pose, the base does not drive.

Fix: select Newton's legacy DOF-indexed target layout (must happen before the
first ``ModelBuilder`` is created) and expose the old names as aliases.
"""

import newton


def _alias(name: str) -> property:
    return property(lambda self: getattr(self, name), lambda self, value: setattr(self, name, value))


def install() -> None:
    newton.use_coord_layout_targets = False
    newton.Control.joint_target_pos = _alias("joint_target_q")
    newton.Control.joint_target_vel = _alias("joint_target_qd")
