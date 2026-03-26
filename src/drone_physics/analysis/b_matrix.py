"""Thruster-to-wrench mapping utilities.

This module provides small linear-algebra helpers to build a body-frame
allocation matrix ``B`` from thruster geometry and then compute the resulting
wrench for a control vector ``u``.
"""

from __future__ import annotations

from math import sqrt
from typing import Sequence

from drone_physics.model import Thruster, ThrusterConfiguration

Vector3 = tuple[float, float, float]
Vector6 = tuple[float, float, float, float, float, float]
Matrix = list[list[float]]


def normalize_direction(direction: Sequence[float]) -> Vector3:
    """Return a validated unit 3-vector.

    Args:
        direction: 3D thruster direction (need not be unit-length).

    Raises:
        ValueError: If direction is not 3-dimensional or has zero norm.
    """

    if len(direction) != 3:
        raise ValueError("Thruster direction must be 3-dimensional.")

    dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
    norm = sqrt(dx * dx + dy * dy + dz * dz)
    if norm == 0.0:
        raise ValueError("Thruster direction must be non-zero.")

    return (dx / norm, dy / norm, dz / norm)


def _validate_position(position: Sequence[float]) -> Vector3:
    if len(position) != 3:
        raise ValueError("Thruster position must be 3-dimensional.")
    return (float(position[0]), float(position[1]), float(position[2]))


def compute_force(direction: Sequence[float], command: float) -> Vector3:
    """Compute force ``F_i = d_i * u_i`` for one thruster."""

    dx, dy, dz = normalize_direction(direction)
    ui = float(command)
    return (dx * ui, dy * ui, dz * ui)


def compute_moment(position: Sequence[float], force: Sequence[float]) -> Vector3:
    """Compute moment ``M_i = r_i x F_i`` for one thruster."""

    rx, ry, rz = _validate_position(position)
    if len(force) != 3:
        raise ValueError("Force vector must be 3-dimensional.")
    fx, fy, fz = (float(force[0]), float(force[1]), float(force[2]))

    return (
        ry * fz - rz * fy,
        rz * fx - rx * fz,
        rx * fy - ry * fx,
    )


def _iter_thrusters(thrusters: Sequence[Thruster] | ThrusterConfiguration) -> list[Thruster]:
    return list(thrusters)


def build_b_matrix(thrusters: Sequence[Thruster] | ThrusterConfiguration) -> Matrix:
    """Build a ``6 x n`` allocation matrix from thruster geometry.

    The first 3 rows map control inputs to force, and the last 3 rows map to
    moment.
    """

    thruster_list = _iter_thrusters(thrusters)
    n = len(thruster_list)
    b: Matrix = [[0.0 for _ in range(n)] for _ in range(6)]

    for i, thruster in enumerate(thruster_list):
        direction = normalize_direction(thruster.direction_body)
        position = _validate_position(thruster.position_body)

        fx, fy, fz = direction
        mx, my, mz = compute_moment(position, direction)

        b[0][i] = fx
        b[1][i] = fy
        b[2][i] = fz
        b[3][i] = mx
        b[4][i] = my
        b[5][i] = mz

    return b


def wrench_from_matrix(b_matrix: Matrix, commands: Sequence[float]) -> Vector6:
    """Compute total wrench ``w = B @ u`` from matrix and command vector."""

    if len(b_matrix) != 6:
        raise ValueError("B-matrix must have exactly 6 rows.")

    n = len(commands)
    for row in b_matrix:
        if len(row) != n:
            raise ValueError("All B-matrix rows must match the command length.")

    u = [float(value) for value in commands]
    wrench = []
    for row in b_matrix:
        value = 0.0
        for coeff, cmd in zip(row, u):
            value += coeff * cmd
        wrench.append(value)

    return (
        wrench[0],
        wrench[1],
        wrench[2],
        wrench[3],
        wrench[4],
        wrench[5],
    )


def compute_wrench(
    thrusters: Sequence[Thruster] | ThrusterConfiguration,
    commands: Sequence[float],
) -> Vector6:
    """Build ``B`` and compute ``w = B @ u`` for the provided inputs."""

    b_matrix = build_b_matrix(thrusters)
    return wrench_from_matrix(b_matrix, commands)
