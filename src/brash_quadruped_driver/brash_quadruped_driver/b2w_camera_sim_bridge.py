"""Upload low-rate Gazebo RGB frames to the loopback teleoperation gateway."""

from __future__ import annotations

import base64
import json
import os
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urljoin, urlparse
from urllib.request import Request, urlopen

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .image_utils import encode_png


class B2WCameraSimulationBridge(Node):
    """Keep media transport isolated from the motion/safety bridge process."""

    def __init__(self) -> None:
        super().__init__('b2w_camera_sim_bridge')
        self.declare_parameter('media_url', 'http://127.0.0.1:4312/')
        self.declare_parameter('camera_topic', '/camera/b2w/front/image_raw')
        self.declare_parameter('camera_id', 'b2w-sim-front')
        self.declare_parameter('robot_id', 'b2-unit-001')
        self.declare_parameter('upload_rate_hz', 5.0)
        self.declare_parameter('request_timeout_sec', 0.35)

        self._media_url = _loopback_url(str(self.get_parameter('media_url').value))
        self._camera_topic = str(self.get_parameter('camera_topic').value).strip()
        self._camera_id = str(self.get_parameter('camera_id').value).strip()
        self._robot_id = str(self.get_parameter('robot_id').value).strip()
        if not self._camera_topic or not self._camera_id or not self._robot_id:
            raise ValueError('camera topic, camera ID, and robot ID are required')
        upload_rate = _positive(self.get_parameter('upload_rate_hz').value, 'upload_rate_hz')
        self._request_timeout = _positive(
            self.get_parameter('request_timeout_sec').value, 'request_timeout_sec'
        )
        self._token = (
            os.environ.get('BRASH_TELEOP_MEDIA_INGEST_TOKEN', '').strip()
            or os.environ.get('BRASH_TELEOP_ROBOT_CONTROL_TOKEN', '').strip()
        )
        self._latest: Image | None = None
        self._latest_key = 0
        self._uploaded_key = 0
        # Preserve ordering across ROS bridge restarts while the media gateway
        # keeps its bounded in-memory ring alive.
        self._sequence = int(time.time() * 1000)
        self._last_error: str | None = None
        self.create_subscription(Image, self._camera_topic, self._on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / upload_rate, self._tick)
        if len(self._token) < 16:
            self.get_logger().warning(
                'Camera upload disabled: export BRASH_TELEOP_MEDIA_INGEST_TOKEN '
                '(or BRASH_TELEOP_ROBOT_CONTROL_TOKEN) with at least 16 characters'
            )
        else:
            self.get_logger().info(
                f'B2W Gazebo camera bridge started: {self._camera_topic} -> '
                f'{self._media_url}media/v1/cameras/{self._camera_id}/frames'
            )

    def _on_image(self, message: Image) -> None:
        self._latest = message
        self._latest_key += 1

    def _tick(self) -> None:
        if len(self._token) < 16 or self._latest is None or self._latest_key == self._uploaded_key:
            return
        message = self._latest
        key = self._latest_key
        sequence = self._sequence
        self._sequence += 1
        try:
            if bool(message.is_bigendian):
                raise ValueError('big-endian camera images are not supported')
            png = encode_png(
                int(message.width),
                int(message.height),
                str(message.encoding),
                int(message.step),
                bytes(message.data),
            )
            now = int(time.time() * 1000)
            result = self._request_json(
                f'media/v1/cameras/{quote(self._camera_id, safe="")}/frames',
                {
                    'frameId': f'gazebo-{sequence}',
                    'robotId': self._robot_id,
                    'sequence': sequence,
                    'sourceTimestampUnixMs': now,
                    'mediaType': 'image/png',
                    'bytesBase64': base64.b64encode(png).decode('ascii'),
                },
            )
            accepted = _object(result.get('result'), 'response.result').get('accepted')
            if accepted is not True:
                raise ValueError('media gateway did not accept the camera frame')
            self._uploaded_key = key
            if self._last_error is not None:
                self.get_logger().info('B2W Gazebo camera upload recovered')
            self._last_error = None
        except (ValueError, KeyError, HTTPError, URLError, TimeoutError, OSError) as error:
            summary = f'{type(error).__name__}: {error}'
            if summary != self._last_error:
                self.get_logger().warning(f'Camera frame not uploaded: {summary}')
                self._last_error = summary

    def _request_json(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        request = Request(
            urljoin(self._media_url, path),
            method='POST',
            data=json.dumps(payload, separators=(',', ':')).encode(),
            headers={
                'authorization': f'Bearer {self._token}',
                'accept': 'application/json',
                'content-type': 'application/json',
            },
        )
        with urlopen(request, timeout=self._request_timeout) as response:
            raw = response.read(64 * 1024 + 1)
            if len(raw) > 64 * 1024:
                raise ValueError('media response exceeded 64 KiB')
        return _object(json.loads(raw), 'response')


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
        raise ValueError('media_url must be a plain HTTP loopback URL')
    return value.rstrip('/') + '/'


def _positive(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} must be numeric')
    converted = float(value)
    if converted <= 0.0:
        raise ValueError(f'{field} must be positive')
    return converted


def _object(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{field} must be an object')
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = B2WCameraSimulationBridge()
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
