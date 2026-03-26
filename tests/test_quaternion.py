"""Tests for quaternion math and frame conversions."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from drone_physics.math.frames import rotate_body_to_world
from drone_physics.math.quaternion import (
    normalize_quaternion,
    quaternion_to_rotation_matrix,
)


class QuaternionMathTests(unittest.TestCase):
    def test_normalization_preserves_unit_norm(self) -> None:
        q = (2.0, -2.0, 1.0, 3.0)

        q_norm = normalize_quaternion(q)
        norm = math.sqrt(sum(component * component for component in q_norm))

        self.assertAlmostEqual(norm, 1.0, places=12)

    def test_identity_quaternion_produces_identity_rotation(self) -> None:
        identity_q = (1.0, 0.0, 0.0, 0.0)

        rotation = quaternion_to_rotation_matrix(identity_q)

        expected = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        self.assertEqual(rotation, expected)

    def test_z_axis_90_deg_rotation_maps_body_x_to_world_y(self) -> None:
        half_angle = math.pi / 4.0
        q_body_to_world = (math.cos(half_angle), 0.0, 0.0, math.sin(half_angle))

        vector_body = (1.0, 0.0, 0.0)
        vector_world = rotate_body_to_world(vector_body, q_body_to_world)

        self.assertAlmostEqual(vector_world[0], 0.0, places=12)
        self.assertAlmostEqual(vector_world[1], 1.0, places=12)
        self.assertAlmostEqual(vector_world[2], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
