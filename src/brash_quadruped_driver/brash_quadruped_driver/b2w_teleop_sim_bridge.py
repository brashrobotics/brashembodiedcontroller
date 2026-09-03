"""Fail-closed bridge from the local teleop edge state to Gazebo B2W wheels."""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .command_utils import b2w_gazebo_wheel_velocities


WHEEL_JOINTS = ('FL_wheel_joint', 'FR_wheel_joint', 'RL_wheel_joint', 'RR_wheel_joint')
STAND_POSITION = (0.0, 0.8, -1.5) * 4


class B2WTeleopSimulationBridge(Node):
    """Poll accepted robot-local commands and publish only to Gazebo controllers."""

    def __init__(self) -> None:
        super().__init__('b2w_teleop_sim_bridge')
        self.declare_parameter('edge_url', 'http://127.0.0.1:4311/')
        self.declare_parameter('robot_id', 'b2-unit-001')
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('request_timeout_sec', 0.1)
        self.declare_parameter('joint_state_timeout_sec', 0.5)
        self.declare_parameter('wheel_radius_meters', 0.12)
        self.declare_parameter('track_width_meters', 0.38346)
        self.declare_parameter('maximum_wheel_speed_radians_per_second', 8.0)
        self.declare_parameter('maximum_linear_x_meters_per_second', 0.7)
        self.declare_parameter('maximum_angular_z_radians_per_second', 0.7)

        self._edge_url = _loopback_url(str(self.get_parameter('edge_url').value))
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._token = os.environ.get('BRASH_TELEOP_EDGE_TOKEN', '').strip()
        if len(self._token) < 16:
            raise ValueError('BRASH_TELEOP_EDGE_TOKEN with at least 16 characters is required')
        poll_rate = _positive(self.get_parameter('poll_rate_hz').value, 'poll_rate_hz')
        self._request_timeout = _positive(
            self.get_parameter('request_timeout_sec').value, 'request_timeout_sec'
        )
        self._joint_timeout = _positive(
            self.get_parameter('joint_state_timeout_sec').value, 'joint_state_timeout_sec'
        )
        self._wheel_radius = _positive(
            self.get_parameter('wheel_radius_meters').value, 'wheel_radius_meters'
        )
        self._track_width = _positive(
            self.get_parameter('track_width_meters').value, 'track_width_meters'
        )
        self._max_wheel_speed = _positive(
            self.get_parameter('maximum_wheel_speed_radians_per_second').value,
            'maximum_wheel_speed_radians_per_second',
        )
        self._max_linear_x = _positive(
            self.get_parameter('maximum_linear_x_meters_per_second').value,
            'maximum_linear_x_meters_per_second',
        )
        self._max_angular_z = _positive(
            self.get_parameter('maximum_angular_z_radians_per_second').value,
            'maximum_angular_z_radians_per_second',
        )

        self._wheel_publisher = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10
        )
        self._leg_publisher = self.create_publisher(
            Float64MultiArray, '/joint_position_controller/commands', 10
        )
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 20)
        self._last_joint_state_at: float | None = None
        self._wheel_speeds: dict[str, float] = {}
        self._last_interlock_update_at = 0.0
        self._last_error: str | None = None
        self._lateral_rejection_reported = False
        self.create_timer(1.0 / poll_rate, self._tick)
        self.get_logger().info(
            'B2W teleop simulation bridge started; output is restricted '
            'to Gazebo controller topics'
        )

    def _on_joint_state(self, message: JointState) -> None:
        velocities = {
            name: float(message.velocity[index])
            for index, name in enumerate(message.name)
            if index < len(message.velocity) and name in WHEEL_JOINTS
        }
        if all(name in velocities and math.isfinite(velocities[name]) for name in WHEEL_JOINTS):
            self._wheel_speeds = velocities
            self._last_joint_state_at = time.monotonic()

    def _tick(self) -> None:
        self._publish_stand_position()
        try:
            body = self._request_json('GET', 'edge/v1/state')
            state = _object(body.get('state'), 'state')
            self._synchronize_edge(state)
            self._publish_edge_output(state)
            self._last_error = None
        except (ValueError, KeyError, HTTPError, URLError, TimeoutError, OSError) as error:
            self._publish_stop()
            summary = f'{type(error).__name__}: {error}'
            if summary != self._last_error:
                self.get_logger().warning(
                    f'Teleop edge unavailable or invalid; commanding zero: {summary}'
                )
                self._last_error = summary

    def _synchronize_edge(self, state: dict[str, Any]) -> None:
        now = time.monotonic()
        controller_ready = (
            self._wheel_publisher.get_subscription_count() > 0
            and self._leg_publisher.get_subscription_count() > 0
        )
        joint_fresh = (
            self._last_joint_state_at is not None
            and now - self._last_joint_state_at <= self._joint_timeout
        )
        if now - self._last_interlock_update_at >= 0.25:
            body = self._request_json(
                'POST',
                'edge/v1/safety/interlocks',
                {
                    'interlocks': {
                        'vendorSafetyOk': True,
                        'controllerHealthy': controller_ready,
                        'robotStateFresh': joint_fresh,
                    }
                },
            )
            state.update(_object(body.get('state'), 'state'))
            self._last_interlock_update_at = now

        ownership = _object(state.get('ownership'), 'state.ownership')
        if ownership.get('status') != 'HANDOFF' or not controller_ready or not joint_fresh:
            return
        if any(abs(self._wheel_speeds[name]) > 0.05 for name in WHEEL_JOINTS):
            return
        body = self._request_json(
            'POST',
            'edge/v1/ownership/stopped-ack',
            {
                'expectedRevision': _integer(ownership.get('revision'), 'ownership.revision'),
                'evidence': {
                    'observedAtUnixMs': int(time.time() * 1000),
                    'controllerStateFresh': True,
                    'baseStopped': True,
                    'manipulatorHolding': True,
                },
            },
        )
        state.update(_object(body.get('state'), 'state'))

    def _publish_edge_output(self, state: dict[str, Any]) -> None:
        if state.get('robotId') != self._robot_id:
            raise ValueError('edge state belongs to another robot')
        ownership = _object(state.get('ownership'), 'state.ownership')
        owner = ownership.get('owner')
        safety = _object(state.get('baseSafety'), 'state.baseSafety')
        if (
            ownership.get('status') != 'OWNED'
            or not isinstance(owner, dict)
            or owner.get('actuatorGroup') != 'BASE'
            or safety.get('status') != 'ENABLED'
        ):
            self._publish_stop()
            return
        output = _object(safety.get('output'), 'state.baseSafety.output')
        linear_x = _finite(output.get('linearXMetersPerSecond'), 'output.linearX')
        linear_y = _finite(output.get('linearYMetersPerSecond'), 'output.linearY')
        angular_z = _finite(output.get('angularZRadiansPerSecond'), 'output.angularZ')
        if abs(linear_y) > 1e-3:
            self._publish_stop()
            if not self._lateral_rejection_reported:
                self.get_logger().warning('B2W wheel simulation rejected a lateral base command')
                self._lateral_rejection_reported = True
            return
        self._lateral_rejection_reported = False
        if (
            abs(linear_x) > self._max_linear_x + 1e-6
            or abs(angular_z) > self._max_angular_z + 1e-6
        ):
            raise ValueError('edge output exceeded the simulation bridge limit')
        wheel_speeds = b2w_gazebo_wheel_velocities(
            linear_x,
            angular_z,
            self._wheel_radius,
            self._track_width,
            self._max_wheel_speed,
        )
        # The vendor B2W URDF wheel axes are opposite the conventional mobile
        # base sign: positive joint velocity drives toward base -X. Invert all
        # four joints so positive teleop X and yaw match REP-103 base motion.
        self._wheel_publisher.publish(Float64MultiArray(data=list(wheel_speeds)))

    def _publish_stop(self) -> None:
        self._wheel_publisher.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))

    def _publish_stand_position(self) -> None:
        self._leg_publisher.publish(Float64MultiArray(data=list(STAND_POSITION)))

    def _request_json(
        self, method: str, path: str, payload: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        data = None if payload is None else json.dumps(payload, separators=(',', ':')).encode()
        request = Request(
            urljoin(self._edge_url, path),
            method=method,
            data=data,
            headers={
                'authorization': f'Bearer {self._token}',
                'accept': 'application/json',
                **({'content-type': 'application/json'} if data is not None else {}),
            },
        )
        with urlopen(request, timeout=self._request_timeout) as response:
            raw = response.read(256 * 1024 + 1)
            if len(raw) > 256 * 1024:
                raise ValueError('edge response exceeded 256 KiB')
        decoded = json.loads(raw)
        return _object(decoded, 'response')

    def destroy_node(self):
        # ROS signal handling can invalidate the context before launch calls
        # destroy_node(). Publishing after that point raises RCLError and makes
        # an otherwise clean simulator shutdown look like a bridge failure.
        if self.context.ok():
            self._publish_stop()
        return super().destroy_node()


def _loopback_url(value: str) -> str:
    parsed = urlparse(value)
    if (
        parsed.scheme != 'http'
        or parsed.hostname not in ('127.0.0.1', 'localhost', '::1')
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError('edge_url must be a plain HTTP loopback URL')
    return value.rstrip('/') + '/'


def _positive(value: object, field: str) -> float:
    converted = _finite(value, field)
    if converted <= 0.0:
        raise ValueError(f'{field} must be positive')
    return converted


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} must be numeric')
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f'{field} must be finite')
    return converted


def _integer(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f'{field} must be a positive integer')
    return value


def _object(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{field} must be an object')
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = B2WTeleopSimulationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
