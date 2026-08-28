"""Pure conversion helpers shared by the Unitree IMU bridge and its tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


def normalize_quaternion(
    values: Sequence[float], order: str = "wxyz"
) -> tuple[float, float, float, float]:
    """Return a normalized ROS quaternion in x, y, z, w order."""
    if len(values) != 4:
        raise ValueError("a quaternion must contain four values")
    if sorted(order) != ["w", "x", "y", "z"]:
        raise ValueError("orientation_order must be a permutation of 'wxyz'")

    components = {axis: float(values[index]) for index, axis in enumerate(order)}
    ros_order = (
        components["x"],
        components["y"],
        components["z"],
        components["w"],
    )
    if not all(math.isfinite(value) for value in ros_order):
        raise ValueError("quaternion contains a non-finite value")

    norm = math.sqrt(sum(value * value for value in ros_order))
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in ros_order)


def remap_vector(
    values: Sequence[float], axis_order: Sequence[int], axis_sign: Sequence[float]
) -> tuple[float, float, float]:
    """Apply a configurable axis permutation and sign change."""
    if len(values) != 3 or len(axis_order) != 3 or len(axis_sign) != 3:
        raise ValueError("vectors, axis_order, and axis_sign must have length three")
    if sorted(int(index) for index in axis_order) != [0, 1, 2]:
        raise ValueError("axis_order must be a permutation of [0, 1, 2]")
    result = tuple(
        float(values[int(index)]) * float(sign)
        for index, sign in zip(axis_order, axis_sign)
    )
    if not all(math.isfinite(value) for value in result):
        raise ValueError("vector contains a non-finite value")
    return result


def diagonal_covariance(diagonal: Iterable[float]) -> list[float]:
    """Expand three diagonal values into a row-major 3x3 covariance."""
    values = [float(value) for value in diagonal]
    if len(values) != 3 or any(value < 0.0 for value in values):
        raise ValueError("covariance diagonal must contain three non-negative values")
    return [values[0], 0.0, 0.0, 0.0, values[1], 0.0, 0.0, 0.0, values[2]]


@dataclass
class TickClock:
    """Unwrap and align a wrapping controller tick to ROS time."""

    tick_period_sec: float = 1.0e-6
    modulus: int = 2**32
    offset_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.tick_period_sec <= 0.0:
            raise ValueError("tick_period_sec must be positive")
        if self.modulus <= 1:
            raise ValueError("modulus must be greater than one")
        self._last_tick: int | None = None
        self._wraps = 0
        self._alignment_sec: float | None = None

    def reset(self) -> None:
        self._last_tick = None
        self._wraps = 0
        self._alignment_sec = None

    def to_ros_time(self, tick: int, receipt_time_sec: float) -> float:
        tick = int(tick) % self.modulus
        receipt_time_sec = float(receipt_time_sec)
        if not math.isfinite(receipt_time_sec):
            raise ValueError("receipt time must be finite")

        if self._last_tick is not None:
            backwards = self._last_tick - tick
            if backwards > self.modulus // 2:
                self._wraps += 1
            elif backwards > 0:
                # A small backwards jump is a controller reset, not a wrap.
                self.reset()

        unwrapped_tick = tick + self._wraps * self.modulus
        controller_time = unwrapped_tick * self.tick_period_sec
        if self._alignment_sec is None:
            self._alignment_sec = receipt_time_sec - controller_time

        self._last_tick = tick
        return self._alignment_sec + controller_time + self.offset_sec
