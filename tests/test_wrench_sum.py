"""Unit tests for wrench summation from the B-matrix."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from drone_physics.analysis.b_matrix import build_b_matrix, compute_wrench, wrench_from_matrix
from drone_physics.model import Thruster


class WrenchSummationTests(unittest.TestCase):
    def test_summed_wrench_matches_manual_computation(self) -> None:
        thrusters = [
            Thruster(position_body=(1.0, 0.0, 0.0), direction_body=(0.0, 0.0, 1.0)),
            Thruster(position_body=(0.0, 1.0, 0.0), direction_body=(0.0, 0.0, 1.0)),
        ]
        commands = [2.0, 3.0]

        b = build_b_matrix(thrusters)
        wrench_via_matrix = wrench_from_matrix(b, commands)
        wrench_direct = compute_wrench(thrusters, commands)

        # Manual computation:
        # F1 = [0, 0, 2], M1 = [0, -2, 0]
        # F2 = [0, 0, 3], M2 = [3, 0, 0]
        # total = [0, 0, 5, 3, -2, 0]
        expected = (0.0, 0.0, 5.0, 3.0, -2.0, 0.0)

        self.assertEqual(wrench_via_matrix, expected)
        self.assertEqual(wrench_direct, expected)

    def test_direction_is_normalized(self) -> None:
        thrusters = [Thruster(position_body=(0.0, 0.0, 0.0), direction_body=(0.0, 0.0, 10.0))]
        commands = [2.0]

        wrench = compute_wrench(thrusters, commands)
        self.assertEqual(wrench, (0.0, 0.0, 2.0, 0.0, 0.0, 0.0))

    def test_zero_direction_is_rejected(self) -> None:
        thrusters = [Thruster(position_body=(0.0, 0.0, 0.0), direction_body=(0.0, 0.0, 0.0))]
        commands = [1.0]

        with self.assertRaises(ValueError):
            compute_wrench(thrusters, commands)


if __name__ == "__main__":
    unittest.main()
