"""Quaternion math helpers.

Quaternion convention used in :mod:`drone_physics`:
    quaternions are ordered as ``[w, x, y, z]`` and represent Body → World
    orientation.
"""

from __future__ import annotations

import math
from typing import Sequence

Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    """Return a unit quaternion with the same orientation.

    Args:
        quaternion: Quaternion in ``[w, x, y, z]`` ordering.

    Raises:
        ValueError: If the quaternion has zero magnitude.
    """

    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        msg = "Cannot normalize a zero-magnitude quaternion."
        raise ValueError(msg)
    inv_norm = 1.0 / norm
    return (w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm)


def quaternion_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> Quaternion:
    """Hamilton product ``lhs ⊗ rhs`` for ``[w, x, y, z]`` quaternions."""

    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs

    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_to_rotation_matrix(quaternion: Sequence[float]) -> Matrix3:
    """Build the 3×3 Body → World rotation matrix from a quaternion."""

    w, x, y, z = normalize_quaternion(quaternion)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )
