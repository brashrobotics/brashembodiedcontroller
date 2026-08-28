import pytest

from brash_quadruped_driver.command_utils import clamp_velocity


def test_velocity_is_clamped_per_axis():
    linear, angular = clamp_velocity(
        (2.0, -2.0, 0.1),
        (0.1, -0.1, 3.0),
        (0.7, 0.4, 0.0),
        (0.0, 0.0, 0.7),
    )
    assert linear == (0.7, -0.4, 0.0)
    assert angular == (0.0, -0.0, 0.7)


def test_non_finite_velocity_is_rejected():
    with pytest.raises(ValueError):
        clamp_velocity(
            (float("nan"), 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (1.0, 1.0, 1.0),
        )
