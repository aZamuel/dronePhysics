"""Thruster geometry and configuration models."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Sequence


@dataclass(slots=True)
class Thruster:
    """A single thruster defined in the body frame.

    Attributes:
        position_body: Thruster position in the body frame.
        direction_body: Unit thrust direction in the body frame.
        command: Optional latest command or input payload for simple storage.
    """

    position_body: Sequence[float]
    direction_body: Sequence[float]
    command: float | Sequence[float] | None = None


@dataclass(slots=True)
class ThrusterConfiguration:
    """Ordered collection of thrusters.

    The order is preserved to support stable actuator indexing for control input
    vectors and allocation matrices.
    """

    thrusters: list[Thruster] = field(default_factory=list)

    def __iter__(self) -> Iterator[Thruster]:
        return iter(self.thrusters)

    def __len__(self) -> int:
        return len(self.thrusters)

    def __getitem__(self, index: int) -> Thruster:
        return self.thrusters[index]
