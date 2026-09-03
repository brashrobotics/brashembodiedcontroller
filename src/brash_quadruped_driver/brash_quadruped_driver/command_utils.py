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
        raise ValueError('all velocity vectors must have length three')
    values = [float(value) for vector in (linear, angular) for value in vector]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('velocity command contains a non-finite value')
    limits = [float(value) for vector in (max_linear, max_angular) for value in vector]
    if any(limit < 0.0 or not math.isfinite(limit) for limit in limits):
        raise ValueError('velocity limits must be finite and non-negative')

    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    return (
        tuple(clamp(float(value), float(limit)) for value, limit in zip(linear, max_linear)),
        tuple(clamp(float(value), float(limit)) for value, limit in zip(angular, max_angular)),
    )


def differential_wheel_velocities(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    track_width: float,
    maximum_wheel_speed: float,
) -> tuple[float, float, float, float]:
    """Map base twist to FL, FR, RL, RR wheel angular velocities."""
    values = (linear_x, angular_z, wheel_radius, track_width, maximum_wheel_speed)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('wheel command inputs must be finite')
    if wheel_radius <= 0.0 or track_width <= 0.0 or maximum_wheel_speed <= 0.0:
        raise ValueError('wheel geometry and speed limit must be positive')

    half_track = track_width / 2.0
    left = (linear_x - angular_z * half_track) / wheel_radius
    right = (linear_x + angular_z * half_track) / wheel_radius
    peak = max(abs(left), abs(right))
    if peak > maximum_wheel_speed:
        scale = maximum_wheel_speed / peak
        left *= scale
        right *= scale
    return left, right, left, right


def b2w_gazebo_wheel_velocities(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    track_width: float,
    maximum_wheel_speed: float,
) -> tuple[float, float, float, float]:
    """Map base motion to the vendor Gazebo model's reversed wheel axes."""
    conventional = differential_wheel_velocities(
        linear_x, angular_z, wheel_radius, track_width, maximum_wheel_speed
    )
    return tuple(-speed for speed in conventional)
