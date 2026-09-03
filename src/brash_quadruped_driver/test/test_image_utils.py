import struct
import zlib

import pytest

from brash_quadruped_driver.image_utils import encode_png


def test_encode_png_converts_padded_bgr_rows_to_rgb():
    encoded = encode_png(
        width=2,
        height=1,
        encoding='bgr8',
        step=8,
        data=bytes([3, 2, 1, 6, 5, 4, 99, 99]),
    )
    assert encoded.startswith(b'\x89PNG\r\n\x1a\n')
    assert _image_data(encoded) == bytes([0, 1, 2, 3, 4, 5, 6])


def test_encode_png_drops_alpha_and_validates_payload():
    encoded = encode_png(1, 1, 'rgba8', 4, bytes([10, 20, 30, 40]))
    assert _image_data(encoded) == bytes([0, 10, 20, 30])
    with pytest.raises(ValueError, match='payload length'):
        encode_png(1, 2, 'rgb8', 3, bytes([1, 2, 3]))
    with pytest.raises(ValueError, match='unsupported'):
        encode_png(1, 1, '32FC1', 4, bytes(4))


def _image_data(png: bytes) -> bytes:
    offset = 8
    compressed = bytearray()
    while offset < len(png):
        length = struct.unpack('>I', png[offset:offset + 4])[0]
        kind = png[offset + 4:offset + 8]
        payload = png[offset + 8:offset + 8 + length]
        if kind == b'IDAT':
            compressed.extend(payload)
        offset += 12 + length
    return zlib.decompress(compressed)
