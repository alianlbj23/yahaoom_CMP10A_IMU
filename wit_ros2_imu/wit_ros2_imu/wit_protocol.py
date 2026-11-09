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

"""
WitMotion standard serial protocol (Yahboom CMP10A) in pure Python.

Frames are ``55 TYPE D0..D7 SUM`` where ``SUM`` is the low byte of the sum of
the first ten bytes and the payload is little-endian.  This module has no ROS
dependency so it can be unit tested and reused by the calibration CLI.
"""

from dataclasses import dataclass
import enum
import math
import struct
from typing import List, Optional, Tuple, Union

GRAVITY = 9.80665

FRAME_LEN = 11
HEADER = 0x55

TYPE_TIME = 0x50
TYPE_ACCEL = 0x51
TYPE_GYRO = 0x52
TYPE_ANGLE = 0x53
TYPE_MAG = 0x54
TYPE_PRESSURE = 0x56
TYPE_QUATERNION = 0x59
TYPE_REGISTER = 0x5F

# TYPE bytes defined by the WitMotion standard protocol.  A 0x55 followed by a
# byte in this range is treated as a candidate frame header by FrameParser.
TYPE_RANGE = range(0x50, 0x60)

_ACC_SCALE = 16.0 / 32768.0 * GRAVITY                  # -> m/s^2
_GYRO_SCALE = 2000.0 / 32768.0 * math.pi / 180.0       # -> rad/s
_ANGLE_SCALE = 180.0 / 32768.0 * math.pi / 180.0       # -> rad
_QUAT_SCALE = 1.0 / 32768.0

_INT16 = struct.Struct('<hhhh')
_INT32 = struct.Struct('<ii')


@dataclass
class Accel:
    """Linear acceleration in m/s^2 and chip temperature in deg C."""

    x: float
    y: float
    z: float
    temperature: float


@dataclass
class Gyro:
    """Angular velocity in rad/s."""

    x: float
    y: float
    z: float


@dataclass
class Angle:
    """Euler angles in rad (roll about x, pitch about y, yaw about z)."""

    roll: float
    pitch: float
    yaw: float


@dataclass
class Mag:
    """
    Magnetic field in raw LSB and chip temperature in deg C.

    Decoded values are integers; after software hard-iron correction
    (``SoftwareCalibration.apply_mag``) they may be floats.
    """

    x: int
    y: int
    z: int
    temperature: float


@dataclass
class Pressure:
    """Barometric pressure in Pa and altitude in m."""

    pressure_pa: float
    height_m: float


@dataclass
class Quaternion:
    """Orientation quaternion (``w`` is the scalar part)."""

    w: float
    x: float
    y: float
    z: float


@dataclass
class RegisterValues:
    """Four consecutive int16 register values from a read reply (0x5F)."""

    values: Tuple[int, int, int, int]


Packet = Union[Accel, Gyro, Angle, Mag, Pressure, Quaternion, RegisterValues]


class CalMode(enum.IntEnum):
    """Values for the CALSW (0x01) register."""

    NORMAL = 0x00
    ACCEL = 0x01
    HEIGHT_ZERO = 0x03
    HEADING_ZERO = 0x04
    MAG_SPHERE = 0x07
    ANGLE_REF = 0x08
    MAG_DUAL_PLANE = 0x09


REG_SAVE = 0x00
REG_CALSW = 0x01
REG_RSW = 0x02
REG_RRATE = 0x03
REG_READ = 0x27
REG_UNLOCK = 0x69


def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def _require_bytes(data, name: str) -> bytes:
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError(f'{name} must be bytes-like, got {type(data).__name__}')
    return bytes(data)


def decode_frame(frame: bytes) -> Optional[Packet]:
    """
    Decode one 11-byte frame.

    Return ``None`` for a well-formed frame with an unsupported TYPE.
    Raise ``ValueError`` on wrong length, wrong header or checksum mismatch,
    and ``TypeError`` if ``frame`` is not bytes-like.
    """
    frame = _require_bytes(frame, 'frame')
    if len(frame) != FRAME_LEN:
        raise ValueError(f'frame must be {FRAME_LEN} bytes, got {len(frame)}')
    if frame[0] != HEADER:
        raise ValueError(f'bad frame header 0x{frame[0]:02X}')
    if _checksum(frame[:10]) != frame[10]:
        raise ValueError(
            f'checksum mismatch for TYPE 0x{frame[1]:02X}: '
            f'expected 0x{_checksum(frame[:10]):02X}, got 0x{frame[10]:02X}')

    ptype = frame[1]
    payload = frame[2:10]
    if ptype == TYPE_PRESSURE:
        p, h = _INT32.unpack(payload)
        return Pressure(pressure_pa=float(p), height_m=h / 100.0)

    a, b, c, d = _INT16.unpack(payload)
    if ptype == TYPE_ACCEL:
        return Accel(a * _ACC_SCALE, b * _ACC_SCALE, c * _ACC_SCALE, d / 100.0)
    if ptype == TYPE_GYRO:
        return Gyro(a * _GYRO_SCALE, b * _GYRO_SCALE, c * _GYRO_SCALE)
    if ptype == TYPE_ANGLE:
        return Angle(a * _ANGLE_SCALE, b * _ANGLE_SCALE, c * _ANGLE_SCALE)
    if ptype == TYPE_MAG:
        return Mag(a, b, c, d / 100.0)
    if ptype == TYPE_QUATERNION:
        return Quaternion(a * _QUAT_SCALE, b * _QUAT_SCALE, c * _QUAT_SCALE, d * _QUAT_SCALE)
    if ptype == TYPE_REGISTER:
        return RegisterValues((a, b, c, d))
    return None


class FrameParser:
    """
    Incremental frame parser for an arbitrarily chunked byte stream.

    Bytes before a header are discarded.  A candidate frame is a 0x55 byte
    followed by a TYPE in 0x50..0x5F; if its checksum is wrong,
    ``checksum_errors`` is incremented and the parser resynchronises one byte
    later.  Bytes that never form a candidate header (for example the
    truncated tail of a frame at the start of a capture) are skipped without
    being counted.  Frames with a valid checksum but unsupported TYPE are
    silently dropped.
    """

    def __init__(self):
        """Create an empty parser."""
        self._buf = bytearray()
        self.checksum_errors = 0

    def reset(self) -> None:
        """Drop buffered bytes (the error counter is kept)."""
        self._buf.clear()

    def feed(self, data: bytes) -> List[Packet]:
        """Append ``data`` and return every complete packet decoded from it."""
        data = _require_bytes(data, 'data')
        buf = self._buf
        buf += data
        packets: List[Packet] = []
        pos = 0
        n = len(buf)
        while True:
            pos = buf.find(HEADER, pos)
            if pos < 0:
                pos = n
                break
            if n - pos < 2:
                break
            if buf[pos + 1] not in TYPE_RANGE:
                pos += 1
                continue
            if n - pos < FRAME_LEN:
                break
            frame = bytes(buf[pos:pos + FRAME_LEN])
            if _checksum(frame[:10]) != frame[10]:
                self.checksum_errors += 1
                pos += 1
                continue
            packet = decode_frame(frame)
            if packet is not None:
                packets.append(packet)
            pos += FRAME_LEN
        del buf[:pos]
        return packets


def _check_int(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f'{name} must be int, got {type(value).__name__}')
    return value


def _check_reg(reg) -> int:
    reg = _check_int(reg, 'reg')
    if not 0 <= reg <= 0xFF:
        raise ValueError(f'register 0x{reg:X} out of range 0x00..0xFF')
    return reg


def cmd_write(reg: int, value: int) -> bytes:
    """
    Build a write-register command ``FF AA reg lo hi``.

    ``value`` may be signed (-32768..-1, sent as two's complement) or
    unsigned (0..65535).
    """
    reg = _check_reg(reg)
    value = _check_int(value, 'value')
    if not -32768 <= value <= 65535:
        raise ValueError(f'value {value} out of range -32768..65535')
    value &= 0xFFFF
    return bytes([0xFF, 0xAA, reg, value & 0xFF, value >> 8])


def cmd_unlock() -> bytes:
    """Return the unlock command (valid for about 10 s)."""
    return cmd_write(REG_UNLOCK, 0xB588)


def cmd_save() -> bytes:
    """Return the save-configuration-to-flash command."""
    return cmd_write(REG_SAVE, 0x0000)


def cmd_factory_reset() -> bytes:
    """Return the restore-factory-defaults command."""
    return cmd_write(REG_SAVE, 0x0001)


def cmd_read(reg: int) -> bytes:
    """Return the read-register command ``FF AA 27 reg 00`` (reply TYPE 0x5F)."""
    return cmd_write(REG_READ, _check_reg(reg))


def cmd_calsw(mode: CalMode) -> bytes:
    """Return the CALSW command for ``mode``."""
    if isinstance(mode, bool) or not isinstance(mode, int):
        raise TypeError(f'mode must be CalMode, got {type(mode).__name__}')
    return cmd_write(REG_CALSW, int(CalMode(mode)))


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert ZYX (yaw-pitch-roll) Euler angles in rad to a quaternion."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )
