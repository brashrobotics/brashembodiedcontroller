"""Convert the B2 LowState IMU into standard ROS sensor messages."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, Temperature
from unitree_go.msg import LowState

from .imu_conversion import (
    TickClock,
    diagonal_covariance,
    normalize_quaternion,
    remap_vector,
)


class UnitreeImuBridge(Node):
    def __init__(self) -> None:
        super().__init__("unitree_imu_bridge")

        self.declare_parameter("input_topic", "lf/lowstate")
        self.declare_parameter("output_topic", "sensor/imu/data")
        self.declare_parameter("temperature_topic", "sensor/imu/temperature")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("timestamp_mode", "receipt")
        self.declare_parameter("tick_period_sec", 1.0e-6)
        self.declare_parameter("timestamp_offset_sec", 0.0)
        self.declare_parameter("orientation_order", "wxyz")
        self.declare_parameter("axis_order", [0, 1, 2])
        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("orientation_covariance_diagonal", [0.01, 0.01, 0.04])
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.0025, 0.0025, 0.0025])
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.04, 0.04, 0.04])
        self.declare_parameter("source_timeout_sec", 1.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._timestamp_mode = str(self.get_parameter("timestamp_mode").value)
        if self._timestamp_mode not in {"receipt", "tick_aligned"}:
            raise ValueError("timestamp_mode must be 'receipt' or 'tick_aligned'")

        self._orientation_order = str(self.get_parameter("orientation_order").value)
        self._axis_order = list(self.get_parameter("axis_order").value)
        self._axis_sign = list(self.get_parameter("axis_sign").value)
        self._orientation_covariance = diagonal_covariance(
            self.get_parameter("orientation_covariance_diagonal").value
        )
        self._angular_covariance = diagonal_covariance(
            self.get_parameter("angular_velocity_covariance_diagonal").value
        )
        self._acceleration_covariance = diagonal_covariance(
            self.get_parameter("linear_acceleration_covariance_diagonal").value
        )
        self._source_timeout_sec = float(self.get_parameter("source_timeout_sec").value)
        self._tick_clock = TickClock(
            tick_period_sec=float(self.get_parameter("tick_period_sec").value),
            offset_sec=float(self.get_parameter("timestamp_offset_sec").value),
        )

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._imu_publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), sensor_qos
        )
        self._temperature_publisher = self.create_publisher(
            Temperature,
            str(self.get_parameter("temperature_topic").value),
            sensor_qos,
        )
        self._subscription = self.create_subscription(
            LowState,
            str(self.get_parameter("input_topic").value),
            self._on_low_state,
            source_qos,
        )

        self._last_message_monotonic: float | None = None
        self._received_once = False
        self.create_timer(0.5, self._check_source)

    def _message_stamp(self, message: LowState, receipt_time_sec: float):
        stamp_sec = receipt_time_sec
        if self._timestamp_mode == "tick_aligned":
            stamp_sec = self._tick_clock.to_ros_time(message.tick, receipt_time_sec)
        return Time(nanoseconds=int(stamp_sec * 1.0e9)).to_msg()

    def _on_low_state(self, message: LowState) -> None:
        receipt_time_sec = self.get_clock().now().nanoseconds / 1.0e9
        raw = message.imu_state
        try:
            orientation = normalize_quaternion(raw.quaternion, self._orientation_order)
            angular_velocity = remap_vector(
                raw.gyroscope, self._axis_order, self._axis_sign
            )
            linear_acceleration = remap_vector(
                raw.accelerometer, self._axis_order, self._axis_sign
            )
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Dropping invalid Unitree IMU sample: {error}",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self._message_stamp(message, receipt_time_sec)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame_id
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = orientation
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular_velocity
        (
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ) = linear_acceleration
        imu.orientation_covariance = self._orientation_covariance
        imu.angular_velocity_covariance = self._angular_covariance
        imu.linear_acceleration_covariance = self._acceleration_covariance
        self._imu_publisher.publish(imu)

        temperature = Temperature()
        temperature.header = imu.header
        temperature.temperature = float(raw.temperature)
        temperature.variance = 0.0
        self._temperature_publisher.publish(temperature)

        self._last_message_monotonic = time.monotonic()
        if not self._received_once:
            self._received_once = True
            self.get_logger().info(
                f"Publishing B2 IMU on {self.get_parameter('output_topic').value} "
                f"using {self._timestamp_mode} timestamps"
            )

    def _check_source(self) -> None:
        if self._last_message_monotonic is None:
            self.get_logger().warning(
                "No Unitree LowState message received",
                throttle_duration_sec=5.0,
            )
            return
        age = time.monotonic() - self._last_message_monotonic
        if age > self._source_timeout_sec:
            self.get_logger().error(
                f"Unitree LowState is stale ({age:.2f} s)",
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitreeImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
