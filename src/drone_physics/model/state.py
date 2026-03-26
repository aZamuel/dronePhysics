"""State model definitions.

Quaternion convention used everywhere in ``drone_physics``:
    ``quaternion`` is ordered as ``[w, x, y, z]`` and represents the rotation
    from the body frame into the world frame.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(slots=True)
class State:
    """Rigid-body state for the simulated drone.

    Attributes:
        position: Position in the world frame as ``(x, y, z)``.
        velocity: Linear velocity in the world frame as ``(vx, vy, vz)``.
        quaternion: Orientation quaternion as ``[w, x, y, z]`` describing the
            body-to-world rotation.
        angular_velocity: Angular velocity in the body frame as
            ``(wx, wy, wz)``.
    """

    position: Sequence[float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    velocity: Sequence[float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    quaternion: Sequence[float] = field(
        default_factory=lambda: (1.0, 0.0, 0.0, 0.0)
    )
    angular_velocity: Sequence[float] = field(
        default_factory=lambda: (0.0, 0.0, 0.0)
    )
