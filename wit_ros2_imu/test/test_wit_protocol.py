# Copyright 2026 alianlbj23
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the pure-Python WitMotion protocol module."""

import math
import pathlib
import random
import struct

import pytest
from wit_ros2_imu import wit_protocol as wp

FIXTURE = pathlib.Path(__file__).parent / 'fixtures' / 'cmp10a_stationary_9600.bin'


def make_frame(ptype, payload):
    """Build a valid 11-byte frame from a TYPE byte and 8 payload bytes."""
    assert len(payload) == 8
    body = bytes([0x55, ptype]) + bytes(payload)
    return body + bytes([sum(body) & 0xFF])


def int16_payload(a, b, c, d):
    """Pack four int16 values little-endian."""
    return struct.pack('<hhhh', a, b, c, d)


ACC_FRAME = make_frame(0x51, int16_payload(0, 0, 2048, 2534))      # 1 g on z, 25.34 C
GYRO_FRAME = make_frame(0x52, int16_payload(16384, -16384, 0, 1000))  # +-1000 deg/s
ANGLE_FRAME = make_frame(0x53, int16_payload(8192, -8192, 16384, 0))  # 45, -45, 90 deg
MAG_FRAME = make_frame(0x54, int16_payload(100, -200, 300, 2500))


# ---------------------------------------------------------------- decode_frame

def test_decode_accel_scaling_is_applied_once():
    acc = wp.decode_frame(ACC_FRAME)
    assert isinstance(acc, wp.Accel)
    assert acc.x == pytest.approx(0.0)
    assert acc.y == pytest.approx(0.0)
    assert acc.z == pytest.approx(wp.GRAVITY)          # 2048/32768*16 g = 1 g
    assert acc.temperature == pytest.approx(25.34)


def test_decode_gyro_in_rad_per_s():
    gyro = wp.decode_frame(GYRO_FRAME)
    assert isinstance(gyro, wp.Gyro)
    assert gyro.x == pytest.approx(math.radians(1000.0))
    assert gyro.y == pytest.approx(math.radians(-1000.0))
    assert gyro.z == pytest.approx(0.0)


def test_decode_angle_in_rad():
    ang = wp.decode_frame(ANGLE_FRAME)
    assert isinstance(ang, wp.Angle)
    assert ang.roll == pytest.approx(math.radians(45.0))
    assert ang.pitch == pytest.approx(math.radians(-45.0))
    assert ang.yaw == pytest.approx(math.radians(90.0))


def test_decode_mag_raw():
    mag = wp.decode_frame(MAG_FRAME)
    assert isinstance(mag, wp.Mag)
    assert (mag.x, mag.y, mag.z) == (100, -200, 300)
    assert mag.temperature == pytest.approx(25.0)


def test_decode_pressure_0x56():
    frame = make_frame(0x56, struct.pack('<ii', 101325, -1234))
    p = wp.decode_frame(frame)
    assert isinstance(p, wp.Pressure)
    assert p.pressure_pa == pytest.approx(101325.0)
    assert p.height_m == pytest.approx(-12.34)


def test_decode_quaternion_0x59():
    frame = make_frame(0x59, int16_payload(32767, 0, -16384, 16384))
    q = wp.decode_frame(frame)
    assert isinstance(q, wp.Quaternion)
    assert q.w == pytest.approx(32767 / 32768)
    assert q.x == pytest.approx(0.0)
    assert q.y == pytest.approx(-0.5)
    assert q.z == pytest.approx(0.5)


def test_decode_register_values_0x5f():
    frame = make_frame(0x5F, int16_payload(-1, 2, 300, -32768))
    regs = wp.decode_frame(frame)
    assert isinstance(regs, wp.RegisterValues)
    assert regs.values == (-1, 2, 300, -32768)


def test_decode_unknown_type_returns_none():
    assert wp.decode_frame(make_frame(0x50, bytes(8))) is None
    assert wp.decode_frame(make_frame(0x99, bytes(8))) is None


def test_decode_bad_checksum_raises():
    bad = bytearray(ACC_FRAME)
    bad[10] ^= 0xFF
    with pytest.raises(ValueError):
        wp.decode_frame(bytes(bad))


@pytest.mark.parametrize('frame', [b'', ACC_FRAME[:10], ACC_FRAME + b'\x00'])
def test_decode_wrong_length_raises(frame):
    with pytest.raises(ValueError):
        wp.decode_frame(frame)


def test_decode_bad_header_raises():
    body = bytes([0x54, 0x51]) + bytes(8)
    with pytest.raises(ValueError):
        wp.decode_frame(body + bytes([sum(body) & 0xFF]))


def test_decode_rejects_non_bytes():
    with pytest.raises(TypeError):
        wp.decode_frame([0x55] * 11)


# ---------------------------------------------------------------- FrameParser

STREAM = ACC_FRAME + GYRO_FRAME + ANGLE_FRAME + MAG_FRAME


def kinds(packets):
    """Return the list of packet class names."""
    return [type(p).__name__ for p in packets]


def test_parser_whole_stream():
    parser = wp.FrameParser()
    assert kinds(parser.feed(STREAM)) == ['Accel', 'Gyro', 'Angle', 'Mag']
    assert parser.checksum_errors == 0


def test_parser_byte_by_byte():
    parser = wp.FrameParser()
    out = []
    for b in STREAM:
        out.extend(parser.feed(bytes([b])))
    assert kinds(out) == ['Accel', 'Gyro', 'Angle', 'Mag']
    assert parser.checksum_errors == 0


def test_parser_random_chunking():
    rng = random.Random(1234)
    data = STREAM * 20
    for _ in range(20):
        parser = wp.FrameParser()
        out = []
        i = 0
        while i < len(data):
            n = rng.randint(1, 30)
            out.extend(parser.feed(data[i:i + n]))
            i += n
        assert len(out) == 80
        assert parser.checksum_errors == 0


def test_parser_skips_leading_noise_and_truncated_frame():
    # Truncated tail of a previous frame (no 0x55 header) plus random junk.
    noise = bytes([0x00, 0xB6, 0xFF, 0xD4, 0x7D, 0x08, 0x47, 0x0B, 0x12, 0xAB])
    parser = wp.FrameParser()
    assert kinds(parser.feed(noise + STREAM)) == ['Accel', 'Gyro', 'Angle', 'Mag']
    assert parser.checksum_errors == 0


def test_parser_stray_0x55_with_invalid_type_not_counted():
    parser = wp.FrameParser()
    out = parser.feed(b'\x55\x00\x55\xAA' + ACC_FRAME)
    assert kinds(out) == ['Accel']
    assert parser.checksum_errors == 0


def test_parser_counts_checksum_error_and_resyncs():
    bad = bytearray(GYRO_FRAME)
    bad[10] ^= 0x01
    parser = wp.FrameParser()
    out = parser.feed(ACC_FRAME + bytes(bad) + ANGLE_FRAME)
    assert kinds(out) == ['Accel', 'Angle']
    assert parser.checksum_errors == 1


def test_parser_resyncs_after_dropped_byte():
    # A frame missing one byte swallows the next header; parser must recover.
    parser = wp.FrameParser()
    out = parser.feed(ACC_FRAME[:7] + ACC_FRAME[8:] + GYRO_FRAME + ANGLE_FRAME)
    assert kinds(out)[-2:] == ['Gyro', 'Angle']
    assert parser.checksum_errors >= 1


def test_parser_skips_unknown_type_with_valid_checksum():
    parser = wp.FrameParser()
    out = parser.feed(make_frame(0x50, bytes(8)) + ACC_FRAME)
    assert kinds(out) == ['Accel']
    assert parser.checksum_errors == 0


def test_parser_incomplete_frame_waits_for_more():
    parser = wp.FrameParser()
    assert parser.feed(ACC_FRAME[:6]) == []
    assert kinds(parser.feed(ACC_FRAME[6:])) == ['Accel']


def test_parser_empty_feed():
    parser = wp.FrameParser()
    assert parser.feed(b'') == []


def test_parser_rejects_non_bytes():
    parser = wp.FrameParser()
    with pytest.raises(TypeError):
        parser.feed('UQ')


def test_parser_accepts_bytearray_and_memoryview():
    parser = wp.FrameParser()
    out = parser.feed(bytearray(ACC_FRAME)) + parser.feed(memoryview(GYRO_FRAME))
    assert kinds(out) == ['Accel', 'Gyro']


def test_parser_fixture_real_device():
    """
    Real CP2102 9600-baud capture of a stationary CMP10A.

    The capture starts in the middle of a frame: its first 8 bytes are the tail
    of a 0x53 frame with no 0x55 header.  Checksum errors are only counted when
    a 0x55 header is followed by a plausible TYPE byte (0x50..0x5F) and the sum
    is wrong, so that truncated tail must not be counted.  The capture may also
    end mid-frame; the incomplete tail just stays buffered.
    """
    data = FIXTURE.read_bytes()
    parser = wp.FrameParser()
    packets = parser.feed(data)
    assert parser.checksum_errors == 0
    names = set(kinds(packets))
    assert {'Accel', 'Gyro', 'Angle', 'Mag'} <= names
    accels = [p for p in packets if isinstance(p, wp.Accel)]
    assert len(accels) >= 15
    for a in accels:
        norm = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
        assert 9.0 <= norm <= 10.6
        assert 0.0 < a.temperature < 60.0
    gyros = [p for p in packets if isinstance(p, wp.Gyro)]
    for g in gyros:
        assert max(abs(g.x), abs(g.y), abs(g.z)) < math.radians(5.0)


def test_parser_fixture_random_chunks_same_result():
    data = FIXTURE.read_bytes()
    reference = wp.FrameParser().feed(data)
    rng = random.Random(7)
    parser = wp.FrameParser()
    out = []
    i = 0
    while i < len(data):
        n = rng.randint(1, 17)
        out.extend(parser.feed(data[i:i + n]))
        i += n
    assert out == reference
    assert parser.checksum_errors == 0


# ---------------------------------------------------------------- commands

def test_cmd_unlock():
    assert wp.cmd_unlock() == bytes.fromhex('FFAA6988B5')


def test_cmd_save():
    assert wp.cmd_save() == bytes.fromhex('FFAA000000')


def test_cmd_factory_reset():
    assert wp.cmd_factory_reset() == bytes.fromhex('FFAA000100')


@pytest.mark.parametrize('reg', [0x05, 0x0B, 0x00, 0xFF])
def test_cmd_read(reg):
    assert wp.cmd_read(reg) == bytes([0xFF, 0xAA, 0x27, reg, 0x00])


@pytest.mark.parametrize('reg', [-1, 256, 1.5, '5', None, True])
def test_cmd_read_rejects_bad_register(reg):
    with pytest.raises((ValueError, TypeError)):
        wp.cmd_read(reg)


@pytest.mark.parametrize('mode,expected', [
    (wp.CalMode.NORMAL, 'FFAA010000'),
    (wp.CalMode.ACCEL, 'FFAA010100'),
    (wp.CalMode.HEIGHT_ZERO, 'FFAA010300'),
    (wp.CalMode.HEADING_ZERO, 'FFAA010400'),
    (wp.CalMode.MAG_SPHERE, 'FFAA010700'),
    (wp.CalMode.ANGLE_REF, 'FFAA010800'),
    (wp.CalMode.MAG_DUAL_PLANE, 'FFAA010900'),
])
def test_cmd_calsw(mode, expected):
    assert wp.cmd_calsw(mode) == bytes.fromhex(expected)


@pytest.mark.parametrize('mode', [2, 5, 99, 'ACCEL'])
def test_cmd_calsw_rejects_unknown_mode(mode):
    with pytest.raises((ValueError, TypeError)):
        wp.cmd_calsw(mode)


def test_calmode_values():
    assert int(wp.CalMode.NORMAL) == 0
    assert int(wp.CalMode.ACCEL) == 1
    assert int(wp.CalMode.HEIGHT_ZERO) == 3
    assert int(wp.CalMode.HEADING_ZERO) == 4
    assert int(wp.CalMode.MAG_SPHERE) == 7
    assert int(wp.CalMode.ANGLE_REF) == 8
    assert int(wp.CalMode.MAG_DUAL_PLANE) == 9


@pytest.mark.parametrize('reg,value,expected', [
    (0x03, 0x06, 'FFAA030600'),
    (0x02, 0x021E, 'FFAA021E02'),
    (0x05, -1, 'FFAA05FFFF'),
    (0x05, -32768, 'FFAA050080'),
    (0x05, 65535, 'FFAA05FFFF'),
    (0x05, 32767, 'FFAA05FF7F'),
])
def test_cmd_write(reg, value, expected):
    assert wp.cmd_write(reg, value) == bytes.fromhex(expected)


@pytest.mark.parametrize('value', [-32769, 65536, 10 ** 6])
def test_cmd_write_value_out_of_range(value):
    with pytest.raises(ValueError):
        wp.cmd_write(0x03, value)


@pytest.mark.parametrize('reg', [-1, 256])
def test_cmd_write_register_out_of_range(reg):
    with pytest.raises(ValueError):
        wp.cmd_write(reg, 0)


@pytest.mark.parametrize('reg,value', [(0x03, 1.0), ('3', 1), (0x03, None), (True, 1)])
def test_cmd_write_rejects_non_int(reg, value):
    with pytest.raises(TypeError):
        wp.cmd_write(reg, value)


# ---------------------------------------------------------------- quaternion

def test_euler_to_quaternion_identity():
    q = wp.euler_to_quaternion(0.0, 0.0, 0.0)
    assert (q.w, q.x, q.y, q.z) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_euler_to_quaternion_yaw_90():
    q = wp.euler_to_quaternion(0.0, 0.0, math.pi / 2)
    s = math.sqrt(0.5)
    assert (q.w, q.x, q.y, q.z) == pytest.approx((s, 0.0, 0.0, s))


def test_euler_to_quaternion_roll_90():
    q = wp.euler_to_quaternion(math.pi / 2, 0.0, 0.0)
    s = math.sqrt(0.5)
    assert (q.w, q.x, q.y, q.z) == pytest.approx((s, s, 0.0, 0.0))


def test_euler_to_quaternion_unit_norm_and_zyx_convention():
    roll, pitch, yaw = 0.3, -0.4, 1.2
    q = wp.euler_to_quaternion(roll, pitch, yaw)
    assert q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2 == pytest.approx(1.0)
    # Recover ZYX Euler angles from the quaternion.
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    p = math.asin(2 * (q.w * q.y - q.z * q.x))
    y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    assert (r, p, y) == pytest.approx((roll, pitch, yaw))
