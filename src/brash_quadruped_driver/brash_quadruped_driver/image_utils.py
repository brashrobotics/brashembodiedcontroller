"""Dependency-free conversion of ROS raw images to bounded browser-safe PNG."""

from __future__ import annotations

import struct
import zlib


SUPPORTED_ENCODINGS = {'rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'}


def encode_png(
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: bytes,
) -> bytes:
    """Encode a tightly bounded ROS Image payload without OpenCV/Pillow."""
    if not isinstance(width, int) or not isinstance(height, int) or width < 1 or height < 1:
        raise ValueError('image dimensions must be positive integers')
    if width > 1920 or height > 1080 or width * height > 1920 * 1080:
        raise ValueError('image dimensions exceed the simulation camera limit')
    normalized = encoding.lower()
    if normalized not in SUPPORTED_ENCODINGS:
        raise ValueError(f'unsupported image encoding: {encoding}')
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}[normalized]
    source_row_bytes = width * channels
    if not isinstance(step, int) or step < source_row_bytes:
        raise ValueError('image row step is smaller than the encoded row')
    if len(data) != step * height:
        raise ValueError('image payload length does not match height and step')

    rows = bytearray()
    for row_index in range(height):
        source = data[row_index * step:row_index * step + source_row_bytes]
        rows.append(0)  # PNG filter: None
        if normalized in ('rgb8', 'mono8'):
            rows.extend(source)
        elif normalized == 'bgr8':
            for index in range(0, len(source), 3):
                rows.extend((source[index + 2], source[index + 1], source[index]))
        elif normalized == 'rgba8':
            for index in range(0, len(source), 4):
                rows.extend(source[index:index + 3])
        else:  # bgra8
            for index in range(0, len(source), 4):
                rows.extend((source[index + 2], source[index + 1], source[index]))

    color_type = 0 if normalized == 'mono8' else 2
    header = struct.pack('>IIBBBBB', width, height, 8, color_type, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + _chunk(b'IHDR', header) + _chunk(
        b'IDAT', zlib.compress(bytes(rows), level=1)
    ) + _chunk(b'IEND', b'')


def _chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind)
    checksum = zlib.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', checksum)
