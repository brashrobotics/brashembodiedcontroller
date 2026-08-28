"""Pure helpers for guarded quadruped velocity commands."""

from __future__ import annotations

import math
from typing import Sequence


def clamp_velocity(
    linear: Sequence[float],
    angular: Sequence[float],
    max_linear: Sequence[float],
    max_angular: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if not all(len(values) == 3 for values in (linear, angular, max_linear, max_angular)):
        raise ValueError("all velocity vectors must have length three")
    values = [float(value) for vector in (linear, angular) for value in vector]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity command contains a non-finite value")
    limits = [float(value) for vector in (max_linear, max_angular) for value in vector]
    if any(limit < 0.0 or not math.isfinite(limit) for limit in limits):
        raise ValueError("velocity limits must be finite and non-negative")

    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    return (
        tuple(clamp(float(value), float(limit)) for value, limit in zip(linear, max_linear)),
        tuple(clamp(float(value), float(limit)) for value, limit in zip(angular, max_angular)),
    )
