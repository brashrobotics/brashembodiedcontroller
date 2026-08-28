import math

import pytest

from brash_quadruped_driver.imu_conversion import (
    TickClock,
    diagonal_covariance,
    normalize_quaternion,
    remap_vector,
)


def test_unitree_wxyz_is_converted_to_ros_xyzw():
    assert normalize_quaternion([1.0, 0.0, 0.0, 0.0], "wxyz") == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_quaternion_is_normalized():
    x, y, z, w = normalize_quaternion([2.0, 0.0, 0.0, 0.0], "wxyz")
    assert math.isclose(x * x + y * y + z * z + w * w, 1.0)


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        normalize_quaternion([0.0, 0.0, 0.0, 0.0])


def test_vector_axis_mapping():
    assert remap_vector([1.0, 2.0, 3.0], [1, 0, 2], [-1.0, 1.0, 1.0]) == (
        -2.0,
        1.0,
        3.0,
    )


def test_covariance_expansion():
    assert diagonal_covariance([1.0, 2.0, 3.0]) == [
        1.0,
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0.0,
        0.0,
        3.0,
    ]


def test_tick_clock_unwraps_uint32_rollover():
    clock = TickClock(tick_period_sec=1.0e-6)
    first = clock.to_ros_time(2**32 - 10, 100.0)
    second = clock.to_ros_time(5, 100.000015)
    assert second > first
    assert math.isclose(second - first, 15.0e-6, rel_tol=0.0, abs_tol=1.0e-9)


def test_tick_clock_resets_on_small_backwards_jump():
    clock = TickClock(tick_period_sec=1.0e-6)
    clock.to_ros_time(1000, 10.0)
    assert math.isclose(clock.to_ros_time(10, 20.0), 20.0)
