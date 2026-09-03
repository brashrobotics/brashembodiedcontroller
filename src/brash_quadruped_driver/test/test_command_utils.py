from brash_quadruped_driver.command_utils import (
    b2w_gazebo_wheel_velocities,
    clamp_velocity,
    differential_wheel_velocities,
)
import pytest


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
            (float('nan'), 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (1.0, 1.0, 1.0),
        )


def test_differential_wheel_velocities_maps_forward_and_yaw():
    assert differential_wheel_velocities(0.6, 0.0, 0.12, 0.4, 8.0) == (
        5.0, 5.0, 5.0, 5.0
    )
    left, right, rear_left, rear_right = differential_wheel_velocities(
        0.0, 0.6, 0.12, 0.4, 8.0
    )
    assert left == rear_left == -1.0
    assert right == rear_right == 1.0


def test_differential_wheel_velocities_scales_without_changing_curvature():
    left, right, rear_left, rear_right = differential_wheel_velocities(
        1.0, 1.0, 0.1, 0.4, 5.0
    )
    assert left == pytest.approx(10.0 / 3.0)
    assert right == 5.0
    assert rear_left == left
    assert rear_right == right


def test_b2w_gazebo_wheel_axes_are_inverted_for_positive_base_motion():
    assert b2w_gazebo_wheel_velocities(0.6, 0.0, 0.12, 0.4, 8.0) == (
        -5.0, -5.0, -5.0, -5.0
    )
    left, right, rear_left, rear_right = b2w_gazebo_wheel_velocities(
        0.0, 0.6, 0.12, 0.4, 8.0
    )
    assert left > 0.0
    assert right < 0.0
    assert rear_left == left
    assert rear_right == right
