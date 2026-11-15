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
Software calibration model (bias / hard-iron offsets) stored as YAML.

YAML format::

    gyro_bias: [x, y, z]     # rad/s
    accel_bias: [x, y, z]    # m/s^2
    mag_offset: [x, y, z]    # raw LSB
"""

from dataclasses import dataclass
import math
import os
import pathlib
import tempfile
from typing import List, Optional, Sequence, Tuple, Union

import yaml

from .wit_protocol import Accel, GRAVITY, Gyro, Mag

Vector3 = Tuple[float, float, float]
PathLike = Union[str, os.PathLike]

MIN_SAMPLES = 10
FIELDS = ('gyro_bias', 'accel_bias', 'mag_offset')


@dataclass
class SoftwareCalibration:
    """Offsets subtracted from raw sensor readings."""

    gyro_bias: Vector3 = (0.0, 0.0, 0.0)     # rad/s
    accel_bias: Vector3 = (0.0, 0.0, 0.0)    # m/s^2
    mag_offset: Vector3 = (0.0, 0.0, 0.0)    # raw LSB

    def apply_gyro(self, g: Gyro) -> Gyro:
        """Return ``g`` minus the gyro bias."""
        bx, by, bz = self.gyro_bias
        return Gyro(g.x - bx, g.y - by, g.z - bz)

    def apply_accel(self, a: Accel) -> Accel:
        """Return ``a`` minus the accelerometer bias."""
        bx, by, bz = self.accel_bias
        return Accel(a.x - bx, a.y - by, a.z - bz, a.temperature)

    def apply_mag(self, m: Mag) -> Mag:
        """Return ``m`` minus the hard-iron offset (components may become float)."""
        ox, oy, oz = self.mag_offset
        return Mag(m.x - ox, m.y - oy, m.z - oz, m.temperature)


def _vector(value, name: str) -> Vector3:
    """Validate a 3-element finite numeric sequence and return it as floats."""
    if not isinstance(value, (list, tuple)):
        raise ValueError(f'{name} must be a list of 3 numbers, got {type(value).__name__}')
    if len(value) != 3:
        raise ValueError(f'{name} must have 3 elements, got {len(value)}')
    out = []
    for i, v in enumerate(value):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f'{name}[{i}] must be a number, got {v!r}')
        if not math.isfinite(v):
            raise ValueError(f'{name}[{i}] must be finite, got {v!r}')
        out.append(float(v))
    return (out[0], out[1], out[2])


def _validated(cal: SoftwareCalibration) -> SoftwareCalibration:
    return SoftwareCalibration(**{f: _vector(getattr(cal, f), f) for f in FIELDS})


def load_calibration(path: PathLike) -> SoftwareCalibration:
    """
    Load a calibration YAML file.

    A missing file yields the default (all-zero) calibration; keys missing
    from the file default to zero.  Malformed content raises ``ValueError``.
    """
    path = pathlib.Path(path)
    if not path.exists():
        return SoftwareCalibration()
    text = path.read_text(encoding='utf-8')
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f'{path}: invalid YAML: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(f'{path}: top level must be a mapping, got {type(data).__name__}')
    unknown = sorted(str(k) for k in data if k not in FIELDS)
    if unknown:
        raise ValueError(f'{path}: unknown key(s) {unknown}; expected {list(FIELDS)}')
    try:
        values = {k: _vector(data[k], k) for k in FIELDS if k in data}
    except ValueError as exc:
        raise ValueError(f'{path}: {exc}') from exc
    return SoftwareCalibration(**values)


CALIBRATION_FILE_MODE = 0o644   # offsets only, no secrets


def save_calibration(path: PathLike, cal: SoftwareCalibration) -> None:
    """Validate ``cal`` and write it atomically to ``path`` as YAML."""
    cal = _validated(cal)
    path = pathlib.Path(path)
    data = {f: list(getattr(cal, f)) for f in FIELDS}
    text = yaml.safe_dump(data, default_flow_style=None, sort_keys=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix='.' + path.name, suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
        # mkstemp creates the file 0600; the tool runs as root in the
        # container, so the host user could not read it through the mount.
        os.chmod(tmp, CALIBRATION_FILE_MODE)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def _mean(values: Sequence[float]) -> float:
    return math.fsum(values) / len(values)


def compute_static_bias(accels: Sequence[Accel],
                        gyros: Sequence[Gyro]) -> Tuple[Vector3, Vector3]:
    """
    Compute ``(gyro_bias, accel_bias)`` from samples taken at rest.

    The sensor is assumed level, so the expected acceleration is (0, 0, +g)
    and the expected angular velocity is zero.  Fewer than ``MIN_SAMPLES``
    samples of either kind raise ``ValueError``.
    """
    if len(accels) < MIN_SAMPLES:
        raise ValueError(f'need at least {MIN_SAMPLES} accel samples, got {len(accels)}')
    if len(gyros) < MIN_SAMPLES:
        raise ValueError(f'need at least {MIN_SAMPLES} gyro samples, got {len(gyros)}')
    gyro_bias = (_mean([g.x for g in gyros]),
                 _mean([g.y for g in gyros]),
                 _mean([g.z for g in gyros]))
    accel_bias = (_mean([a.x for a in accels]),
                  _mean([a.y for a in accels]),
                  _mean([a.z for a in accels]) - GRAVITY)
    return gyro_bias, accel_bias


def compute_mag_offset(mags: Sequence[Mag]) -> Vector3:
    """
    Compute the hard-iron offset ``(max + min) / 2`` per axis.

    Fewer than ``MIN_SAMPLES`` samples raise ``ValueError``.
    """
    if len(mags) < MIN_SAMPLES:
        raise ValueError(f'need at least {MIN_SAMPLES} mag samples, got {len(mags)}')
    xs = [m.x for m in mags]
    ys = [m.y for m in mags]
    zs = [m.z for m in mags]
    return ((max(xs) + min(xs)) / 2.0,
            (max(ys) + min(ys)) / 2.0,
            (max(zs) + min(zs)) / 2.0)


MAG_COVERAGE_TARGET = 0.85   # fraction of the full +-|B| swing that counts as "axis done"
AXIS_NAMES = ('X', 'Y', 'Z')


PLANE_SECTORS = 12            # 30 deg sectors around the centre of each plane
PLANE_SECTORS_NEEDED = 11     # a plane counts as one full turn at 330 deg
PLANE_MIN_RADIUS = 0.3        # ignore points closer than this share of the plane's radius


class PlaneCoverage:
    """
    Per-plane progress of a magnetometer calibration, like the official MagCal charts.

    The WitMotion PC software shows the XY, XZ and YZ projections of the
    magnetometer readings and asks for one to three turns in each plane until
    a regular ellipse is drawn.  Here a plane counts as done once its
    projection has gone round the (running) centre: PLANE_SECTORS_NEEDED of
    PLANE_SECTORS sectors visited.  A plane name 'XZ|YZ' accepts either.
    """

    def __init__(self, planes):
        """``planes``: required plane names such as ('XY', 'XZ', 'YZ') or ('XZ|YZ',)."""
        self.planes = tuple(planes)
        self._points: List[Vector3] = []
        self.samples = 0

    def add(self, mag: Mag) -> None:
        """Record one magnetometer sample; non-finite samples are ignored."""
        values = (mag.x, mag.y, mag.z)
        if all(math.isfinite(v) for v in values):
            self._points.append(tuple(float(v) for v in values))
            self.samples += 1

    def _single(self, plane: str) -> float:
        if not self._points:
            return 0.0
        i, j = 'XYZ'.index(plane[0]), 'XYZ'.index(plane[1])
        ci = (max(p[i] for p in self._points) + min(p[i] for p in self._points)) / 2.0
        cj = (max(p[j] for p in self._points) + min(p[j] for p in self._points)) / 2.0
        vecs = [(p[i] - ci, p[j] - cj) for p in self._points]
        radius = max(math.hypot(a, b) for a, b in vecs)
        if radius <= 0:
            return 0.0
        filled = {int((math.atan2(b, a) + math.pi) / (2 * math.pi) * PLANE_SECTORS) % PLANE_SECTORS
                  for a, b in vecs if math.hypot(a, b) >= PLANE_MIN_RADIUS * radius}
        return min(1.0, len(filled) / PLANE_SECTORS_NEEDED)

    def progress(self, plane: str) -> float:
        """Return the progress of one required plane in [0, 1]."""
        return max(self._single(p) for p in plane.split('|'))

    def missing(self) -> List[str]:
        """Names of the required planes that are not yet done."""
        return [p for p in self.planes if self.progress(p) < 1.0]

    def complete(self) -> bool:
        """Return True once every required plane is done."""
        return not self.missing()


class MagCoverage:
    """
    Per-axis progress of a magnetometer calibration, from the magnetometer itself.

    While the IMU is rotated, each axis reading should swing across the whole
    field: from about ``-|B|`` to ``+|B|`` around the (unknown) hard-iron
    centre.  Progress per axis is ``(max - min) / (2 * radius)``, scaled so
    that reaching ``target`` of the full swing counts as 100 %.

    The gyro cannot be used for this: the CMP10A freezes its accel / gyro /
    angle output while in magnetometer-calibration mode (see the test fixture
    ``cmp10a_magcal_mode_rotating_9600.bin``).
    """

    def __init__(self, field_radius: Optional[float] = None,
                 target: float = MAG_COVERAGE_TARGET):
        """``field_radius``: |B| measured beforehand (raw LSB), or None if unknown."""
        if field_radius is not None and (not math.isfinite(field_radius) or field_radius <= 0):
            raise ValueError('field_radius must be a positive finite number, '
                             f'got {field_radius!r}')
        if not math.isfinite(target) or not 0 < target <= 1:
            raise ValueError(f'target must be in (0, 1], got {target!r}')
        self._field_radius = field_radius or 0.0
        self._target = target
        self._min = [math.inf, math.inf, math.inf]
        self._max = [-math.inf, -math.inf, -math.inf]
        self.samples = 0

    def add(self, mag: Mag) -> None:
        """Record one magnetometer sample; non-finite samples are ignored."""
        values = (mag.x, mag.y, mag.z)
        if not all(math.isfinite(v) for v in values):
            return
        for i, v in enumerate(values):
            self._min[i] = min(self._min[i], v)
            self._max[i] = max(self._max[i], v)
        self.samples += 1

    def _spans(self) -> Vector3:
        if not self.samples:
            return (0.0, 0.0, 0.0)
        x, y, z = (hi - lo for hi, lo in zip(self._max, self._min))
        return (x, y, z)

    @property
    def radius(self) -> float:
        """Field radius used for scaling: the reference, or half the largest span if bigger."""
        return max(self._field_radius, max(self._spans()) / 2.0)

    def axis_progress(self) -> Vector3:
        """Per-axis progress in [0, 1]."""
        radius = self.radius
        if radius <= 0:
            return (0.0, 0.0, 0.0)
        x, y, z = (min(1.0, span / (2.0 * radius) / self._target) for span in self._spans())
        return (x, y, z)

    def progress(self) -> float:
        """Overall progress in [0, 1] (mean of the three axes)."""
        return sum(self.axis_progress()) / 3.0

    def missing_axes(self) -> List[str]:
        """Names of the axes that are not yet covered, e.g. ``['Y', 'Z']``."""
        return [name for name, p in zip(AXIS_NAMES, self.axis_progress()) if p < 1.0 - 1e-9]

    def complete(self) -> bool:
        """Return True once every axis is covered."""
        return not self.missing_axes()
