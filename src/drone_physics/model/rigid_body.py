"""Rigid-body parameter models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


Matrix3x3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


@dataclass(slots=True)
class RigidBody:
    """Static rigid-body properties for the drone.

    Attributes:
        mass: Mass in kilograms.
        inertia_tensor: Body-frame inertia tensor, typically a 3x3 matrix.
    """

    mass: float
    inertia_tensor: Sequence[Sequence[float]]
