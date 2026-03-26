"""Top-level package for reusable drone physics primitives.

Importing this package is intentionally side-effect free: it only exposes data
models and does not execute any simulation setup or runtime integration code.
"""

from .model import RigidBody, State, Thruster, ThrusterConfiguration

__all__ = ["RigidBody", "State", "Thruster", "ThrusterConfiguration"]
