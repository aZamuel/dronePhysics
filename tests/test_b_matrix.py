"""Unit tests for B-matrix construction from thruster geometry."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from drone_physics.analysis.b_matrix import build_b_matrix
from drone_physics.model import Thruster


class BMatrixTests(unittest.TestCase):
    def test_centered_upward_thruster_force_without_moment(self) -> None:
        thrusters = [Thruster(position_body=(0.0, 0.0, 0.0), direction_body=(0.0, 0.0, 1.0))]
        b = build_b_matrix(thrusters)

        self.assertEqual(len(b), 6)
        self.assertEqual([row[0] for row in b], [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    def test_offset_thruster_generates_force_and_moment(self) -> None:
        thrusters = [Thruster(position_body=(1.0, 0.0, 0.0), direction_body=(0.0, 0.0, 1.0))]
        b = build_b_matrix(thrusters)

        # F = [0, 0, 1], M = r x F = [0, -1, 0]
        self.assertEqual([row[0] for row in b], [0.0, 0.0, 1.0, 0.0, -1.0, 0.0])


if __name__ == "__main__":
    unittest.main()
