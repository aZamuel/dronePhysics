"""Import smoke tests for the public package API."""

from __future__ import annotations

import importlib
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

MODULES = [
    "drone_physics",
    "drone_physics.model",
    "drone_physics.model.state",
    "drone_physics.model.rigid_body",
    "drone_physics.model.thruster",
    "drone_physics.math",
    "drone_physics.math.quaternion",
    "drone_physics.math.frames",
]


class ImportSmokeTests(unittest.TestCase):
    def test_module_imports_are_side_effect_free(self) -> None:
        for module_name in MODULES:
            with self.subTest(module_name=module_name):
                module = importlib.import_module(module_name)
                self.assertIsNotNone(module)

    def test_top_level_exports_are_available(self) -> None:
        package = importlib.import_module("drone_physics")
        self.assertTrue(hasattr(package, "State"))
        self.assertTrue(hasattr(package, "RigidBody"))
        self.assertTrue(hasattr(package, "Thruster"))
        self.assertTrue(hasattr(package, "ThrusterConfiguration"))


if __name__ == "__main__":
    unittest.main()
