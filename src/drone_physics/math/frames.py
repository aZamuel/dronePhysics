"""Frame transformation helpers.

All helpers assume quaternion orientation is Body → World in ``[w, x, y, z]``.
"""

from __future__ import annotations

from typing import Sequence

from .quaternion import quaternion_to_rotation_matrix

Vector3 = tuple[float, float, float]


def _mat_vec_mul(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Vector3:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def rotate_body_to_world(vector_body: Sequence[float], quaternion: Sequence[float]) -> Vector3:
    """Rotate a 3D vector from body frame into world frame."""

    rotation_body_to_world = quaternion_to_rotation_matrix(quaternion)
    return _mat_vec_mul(rotation_body_to_world, vector_body)


def rotate_world_to_body(vector_world: Sequence[float], quaternion: Sequence[float]) -> Vector3:
    """Rotate a 3D vector from world frame into body frame."""

    rotation_body_to_world = quaternion_to_rotation_matrix(quaternion)
    rotation_world_to_body = (
        (
            rotation_body_to_world[0][0],
            rotation_body_to_world[1][0],
            rotation_body_to_world[2][0],
        ),
        (
            rotation_body_to_world[0][1],
            rotation_body_to_world[1][1],
            rotation_body_to_world[2][1],
        ),
        (
            rotation_body_to_world[0][2],
            rotation_body_to_world[1][2],
            rotation_body_to_world[2][2],
        ),
    )
    return _mat_vec_mul(rotation_world_to_body, vector_world)
