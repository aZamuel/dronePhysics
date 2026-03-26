"""Core data models for rigid-body drone simulations.

Quaternion convention used throughout this package:
    quaternions are stored as ``[w, x, y, z]`` and represent the rotation from
    the body frame into the world frame.
"""

from .rigid_body import RigidBody
from .state import State
from .thruster import Thruster, ThrusterConfiguration

__all__ = ["RigidBody", "State", "Thruster", "ThrusterConfiguration"]
