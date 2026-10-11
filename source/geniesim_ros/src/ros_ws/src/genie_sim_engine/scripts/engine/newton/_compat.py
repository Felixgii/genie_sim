# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Newton API compatibility shims.

Newton 1.5 renamed ``Control.joint_target_pos`` / ``joint_target_vel`` to
``joint_target_q`` / ``joint_target_qd`` and shadowed the old names with a
``RemovedAttribute`` descriptor that raises on access.  The engine reads
them via ``getattr(control, "joint_target_pos", None)``, which then
silently yields ``None`` and every ``/joint_command`` / ``/cmd_4ws`` write
is dropped.

With ``newton.use_coord_layout_targets = False`` (the default) the new
buffers keep the legacy DOF-indexed layout, so aliasing the old names to
the new ones is semantics-preserving.
"""

from __future__ import annotations

import inspect

_RENAMES = (
    ("joint_target_pos", "joint_target_q"),
    ("joint_target_vel", "joint_target_qd"),
)


def _alias(new: str) -> property:
    return property(
        lambda self: getattr(self, new),
        lambda self, value: setattr(self, new, value),
    )


def install_control_aliases() -> list[str]:
    """Alias removed ``Control`` target attributes to their Newton 1.5 names.

    No-op on Newton versions that still define the old attributes, and when
    coordinate-layout targets are enabled (layouts would differ). Returns
    the aliased attribute names.
    """
    import newton

    if bool(getattr(newton, "use_coord_layout_targets", False)):
        return []
    control_cls = newton.Control
    aliased = []
    for old, new in _RENAMES:
        current = inspect.getattr_static(control_cls, old, None)
        if type(current).__name__ != "RemovedAttribute":
            continue
        setattr(control_cls, old, _alias(new))
        aliased.append(old)
    return aliased
