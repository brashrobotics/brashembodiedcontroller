"""Deadman and limit gate for commands sent to the B2W hardware driver."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .command_utils import clamp_velocity


class VelocityGuard(Node):
    def __init__(self) -> None:
        super().__init__("velocity_guard")
        self.declare_parameter("input_topic", "command/cmd_vel")
        self.declare_parameter("enable_topic", "safety/enable")
        self.declare_parameter("output_topic", "hardware/cmd_vel_guarded")
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("enable_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_linear", [0.7, 0.4, 0.0])
        self.declare_parameter("max_angular", [0.0, 0.0, 0.7])

        self._command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._enable_timeout = float(self.get_parameter("enable_timeout_sec").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if self._command_timeout <= 0.0 or self._enable_timeout <= 0.0 or publish_rate <= 0.0:
            raise ValueError("guard timeouts and publish_rate_hz must be positive")
        self._max_linear = list(self.get_parameter("max_linear").value)
        self._max_angular = list(self.get_parameter("max_angular").value)

        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._on_command,
            10,
        )
        self._enable_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self._on_enable,
            10,
        )
        self._last_command: Twist | None = None
        self._last_command_time: float | None = None
        self._last_enable_time: float | None = None
        self._enabled = False
        self._was_active = False
        self.create_timer(1.0 / publish_rate, self._publish_guarded_command)

    def _on_command(self, message: Twist) -> None:
        try:
            linear, angular = clamp_velocity(
                (message.linear.x, message.linear.y, message.linear.z),
                (message.angular.x, message.angular.y, message.angular.z),
                self._max_linear,
                self._max_angular,
            )
        except ValueError as error:
            self.get_logger().error(f"Rejected unsafe velocity command: {error}")
            self._last_command = None
            return

        guarded = Twist()
        guarded.linear.x, guarded.linear.y, guarded.linear.z = linear
        guarded.angular.x, guarded.angular.y, guarded.angular.z = angular
        self._last_command = guarded
        self._last_command_time = time.monotonic()

    def _on_enable(self, message: Bool) -> None:
        self._enabled = bool(message.data)
        self._last_enable_time = time.monotonic()
        if not self._enabled:
            self._publisher.publish(Twist())

    def _publish_guarded_command(self) -> None:
        now = time.monotonic()
        enable_fresh = (
            self._enabled
            and self._last_enable_time is not None
            and now - self._last_enable_time <= self._enable_timeout
        )
        command_fresh = (
            self._last_command is not None
            and self._last_command_time is not None
            and now - self._last_command_time <= self._command_timeout
        )
        active = enable_fresh and command_fresh
        self._publisher.publish(self._last_command if active else Twist())
        if self._was_active and not active:
            self.get_logger().warning("Velocity guard stopped the B2W command stream")
        self._was_active = active

    def destroy_node(self):
        self._publisher.publish(Twist())
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
