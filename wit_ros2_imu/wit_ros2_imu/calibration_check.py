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
Reference-based calibration check for the CMP10A (pure Python, no ROS).

Every test compares raw sensor output with a reference that does not come
from the IMU itself, following common inertial-sensor test practice:

* accelerometer: six-position (tumble) test against gravity
  (cf. IEEE Std 1293-2018);
* gyroscope: static bias against zero rotation, and one full turn against a
  physical reference line (bias / scale factor as defined in
  IEEE Std 2700-2017);
* magnetometer: constant field magnitude in every direction, and the dip
  angle against the local inclination from the World Magnetic Model.

The pass thresholds below are this project's own choices; the reasoning for
each is written down in docs/CALIBRATION_CHECK.md.  The standards define how
to measure, not what counts as good enough.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple

from .wit_protocol import GRAVITY

Vector3 = Tuple[float, float, float]

# Pass thresholds (see docs/CALIBRATION_CHECK.md for the reasoning).
ACCEL_MAG_TOL = 0.1          # m/s^2; |a| on every face vs. GRAVITY  (~0.6 deg of tilt)
ACCEL_OFFSET_TOL = 0.1       # m/s^2; (up + down) / 2 on each axis
GYRO_BIAS_TOL_DPS = 0.02     # deg/s; static mean on each axis      (1.2 deg/min)
TURN_TARGET_DEG = 360.0
TURN_TOL_DEG = 3.0           # deg on one full turn                  (0.8 % scale)
MAG_SPREAD_TOL = 0.05        # fraction of the mean |B|              (~3 deg heading)
MAG_SPREAD_SHARE = 0.95      # share of samples that must be inside the band
DIP_TOL_DEG = 3.0            # deg vs. the World Magnetic Model inclination

FACE_MIN_FRACTION = 0.9      # the expected axis must read >= 0.9 g to count as "on the face"
MAX_GAP_S = 0.5              # longer gaps in gyro data are not integrated

AXES = ('X', 'Y', 'Z')
FACES = (('Z', +1), ('Z', -1), ('X', +1), ('X', -1), ('Y', +1), ('Y', -1))


def axis_index(axis: str) -> int:
    """Return 0, 1 or 2 for 'X', 'Y' or 'Z'."""
    return AXES.index(axis)


def face_label(axis: str, sign: int) -> str:
    """Return e.g. 'Z 軸朝上' for ('Z', +1)."""
    return f'{axis} 軸朝{"上" if sign > 0 else "下"}'


def face_name(accel: Sequence[float]) -> str:
    """Return the label of the face the IMU is currently resting on."""
    i = max(range(3), key=lambda k: abs(accel[k]))
    return face_label(AXES[i], 1 if accel[i] >= 0 else -1)


def facing(accel: Sequence[float], axis: str, sign: int,
           min_fraction: float = FACE_MIN_FRACTION) -> bool:
    """Return True if ``axis`` points up (sign +1) or down (-1) within ~25 deg."""
    return sign * accel[axis_index(axis)] >= min_fraction * GRAVITY


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def _mean3(samples: Sequence[Sequence[float]]) -> Vector3:
    if not samples:
        raise ValueError('no samples')
    n = float(len(samples))
    return (sum(s[0] for s in samples) / n, sum(s[1] for s in samples) / n,
            sum(s[2] for s in samples) / n)


@dataclass
class FaceResult:
    """Mean accelerometer reading on one face."""

    axis: str
    sign: int
    mean: Vector3
    magnitude: float

    @property
    def ok(self) -> bool:
        """Return True if |a| on this face is within ACCEL_MAG_TOL of gravity."""
        return abs(self.magnitude - GRAVITY) <= ACCEL_MAG_TOL


@dataclass
class AccelResult:
    """Outcome of the six-face test."""

    faces: List[FaceResult]
    offsets: Vector3

    @property
    def magnitudes_ok(self) -> bool:
        """Return True if every face reads 1 g."""
        return all(f.ok for f in self.faces)

    @property
    def offsets_ok(self) -> bool:
        """Return True if every axis is centred on zero."""
        return all(abs(o) <= ACCEL_OFFSET_TOL for o in self.offsets)

    @property
    def ok(self) -> bool:
        """Return True if the accelerometer passes."""
        return self.magnitudes_ok and self.offsets_ok


def evaluate_face(axis: str, sign: int, samples: Sequence[Sequence[float]]) -> FaceResult:
    """Average the accelerometer samples (m/s^2) taken on one face."""
    mean = _mean3(samples)
    return FaceResult(axis, sign, mean, _norm(mean))


def evaluate_six_faces(faces: Dict[Tuple[str, int], FaceResult]) -> AccelResult:
    """
    Combine the six faces.

    The offset of an axis is the mean of its readings with that axis up and
    down: +g and -g cancel and only the offset remains.
    """
    missing = [k for k in FACES if k not in faces]
    if missing:
        raise ValueError(f'missing faces: {missing}')
    offsets = tuple((faces[(a, +1)].mean[axis_index(a)] + faces[(a, -1)].mean[axis_index(a)])
                    / 2.0 for a in AXES)
    return AccelResult([faces[k] for k in FACES], offsets)


def gyro_bias_dps(samples: Sequence[Sequence[float]]) -> Vector3:
    """Return the mean of gyro samples (rad/s) in deg/s."""
    return tuple(math.degrees(c) for c in _mean3(samples))


def gyro_bias_ok(bias_dps: Sequence[float]) -> bool:
    """Return True if every axis of the static mean is within GYRO_BIAS_TOL_DPS."""
    return all(abs(b) <= GYRO_BIAS_TOL_DPS for b in bias_dps)


class TurnIntegrator:
    """Integrate the Z angular rate (rad/s) over time (s) with the trapezoid rule."""

    def __init__(self):
        """Start at zero."""
        self.angle_deg = 0.0
        self._last = None

    def add(self, t: float, rate_rad_s: float) -> float:
        """Add one sample taken at time ``t``; return the angle so far in degrees."""
        if self._last is not None:
            t0, r0 = self._last
            dt = t - t0
            if 0.0 < dt <= MAX_GAP_S:
                self.angle_deg += math.degrees((r0 + rate_rad_s) / 2.0 * dt)
        self._last = (t, rate_rad_s)
        return self.angle_deg


def turn_ok(angle_deg: float) -> bool:
    """Return True if the integrated angle is one full turn within TURN_TOL_DEG."""
    return abs(abs(angle_deg) - TURN_TARGET_DEG) <= TURN_TOL_DEG


def magnitude_spread(mags: Sequence[Sequence[float]]) -> float:
    """
    Return the band, as a fraction of the mean |B|, that holds MAG_SPREAD_SHARE of samples.

    With a well calibrated magnetometer |B| is the same in every direction.
    A share below 100 % keeps a few spikes (a phone passing by) from deciding.
    """
    norms = [_norm(m) for m in mags]
    if not norms:
        raise ValueError('no samples')
    mean = sum(norms) / len(norms)
    if mean <= 0:
        raise ValueError('magnetometer reads zero')
    deviations = sorted(abs(n - mean) for n in norms)
    k = max(0, math.ceil(MAG_SPREAD_SHARE * len(deviations)) - 1)
    return deviations[k] / mean


def magnitude_ok(spread: float) -> bool:
    """Return True if the magnitude band is within MAG_SPREAD_TOL."""
    return spread <= MAG_SPREAD_TOL


def dip_angle_deg(accel: Sequence[float], mag: Sequence[float]) -> float:
    """
    Return the angle of the field below the horizontal plane, in degrees.

    At rest the accelerometer points up, so "down" is -accel.  The result is
    independent of how the IMU is held, which makes it comparable with the
    inclination (I) given by the World Magnetic Model.
    """
    na = _norm(accel)
    nm = _norm(mag)
    if na <= 0 or nm <= 0:
        raise ValueError('zero vector')
    down_component = -sum(a * m for a, m in zip(accel, mag)) / na
    return math.degrees(math.asin(max(-1.0, min(1.0, down_component / nm))))


def mean_dip(pairs: Sequence[Tuple[Sequence[float], Sequence[float]]]) -> float:
    """Return the mean dip angle over (accel, mag) pairs."""
    if not pairs:
        raise ValueError('no samples')
    return sum(dip_angle_deg(a, m) for a, m in pairs) / len(pairs)


def dip_ok(measured_deg: float, expected_deg: float) -> bool:
    """Return True if the measured dip is within DIP_TOL_DEG of the expected one."""
    return abs(measured_deg - expected_deg) <= DIP_TOL_DEG


def parse_dip(text: str) -> float:
    """Parse a local inclination in degrees (-90..90); raise ValueError otherwise."""
    value = float(text.strip())
    if not math.isfinite(value) or not -90.0 <= value <= 90.0:
        raise ValueError(f'inclination must be between -90 and 90 degrees, got {text!r}')
    return value


# ----------------------------------------- environment and mounting position
#
# Checked before a magnetometer calibration.  The Earth's field is the same
# everywhere in a room and does not change over seconds, so a field that moves
# while the IMU is still, changes when the IMU is slid without turning, or
# changes when the motors run, comes from the surroundings.

ENV_NOISE_TOL = 0.01         # std of |B| at rest / mean |B|          (sensor noise ~0.5 %)
ENV_CHANGE_TOL = 0.05        # field change / |B| when slid or with motors running (~3 deg)
SLIDE_MAX_ROTATION_DEG = 3.0  # more turning than this makes the slide test invalid
TILT_NOTE_DEG = 5.0          # mounted tilt above this is reported
VIBRATION_TOL = 1.0          # m/s^2, std of |a| with the motors running (~0.1 g)


def field_noise(mags: Sequence[Sequence[float]]) -> float:
    """Return the standard deviation of |B| divided by its mean."""
    norms = [_norm(m) for m in mags]
    if len(norms) < 2:
        raise ValueError('not enough samples')
    mean = sum(norms) / len(norms)
    if mean <= 0:
        raise ValueError('magnetometer reads zero')
    var = sum((n - mean) ** 2 for n in norms) / (len(norms) - 1)
    return math.sqrt(var) / mean


def noise_ok(noise: float) -> bool:
    """Return True if the field at rest is steady."""
    return noise <= ENV_NOISE_TOL


def field_change(reference: Sequence[Sequence[float]],
                 mags: Sequence[Sequence[float]]) -> float:
    """
    Return how far the field moved away from the reference, as a fraction of it.

    The vector difference is used, so a change of direction counts as well as
    a change of strength.  MAG_SPREAD_SHARE of the samples decide, so one spike
    does not.
    """
    ref = _mean3(reference)
    ref_norm = _norm(ref)
    if ref_norm <= 0 or not mags:
        raise ValueError('not enough samples')
    deviations = sorted(_norm([a - b for a, b in zip(m, ref)]) for m in mags)
    k = max(0, math.ceil(MAG_SPREAD_SHARE * len(deviations)) - 1)
    return deviations[k] / ref_norm


def change_ok(change: float) -> bool:
    """Return True if the field stayed within ENV_CHANGE_TOL."""
    return change <= ENV_CHANGE_TOL


class RotationMeter:
    """Add up how much the IMU turned, about any axis, from gyro samples (rad/s)."""

    def __init__(self):
        """Start at zero."""
        self.total_deg = 0.0
        self._last = None

    def add(self, t: float, rate_xyz: Sequence[float]) -> float:
        """Add one sample taken at time ``t``; return the total in degrees."""
        rate = _norm(rate_xyz)
        if self._last is not None:
            t0, r0 = self._last
            dt = t - t0
            if 0.0 < dt <= MAX_GAP_S:
                self.total_deg += math.degrees((r0 + rate) / 2.0 * dt)
        self._last = (t, rate)
        return self.total_deg


def slide_valid(rotation_deg: float) -> bool:
    """Return True if the IMU was slid without turning noticeably."""
    return rotation_deg <= SLIDE_MAX_ROTATION_DEG


def tilt_deg(accel: Sequence[float]) -> float:
    """Return the angle between the IMU's +Z axis and straight up, in degrees."""
    n = _norm(accel)
    if n <= 0:
        raise ValueError('zero vector')
    return math.degrees(math.acos(max(-1.0, min(1.0, accel[2] / n))))


def vibration(accels: Sequence[Sequence[float]]) -> float:
    """Return the standard deviation of |a| in m/s^2."""
    norms = [_norm(a) for a in accels]
    if not norms:
        raise ValueError('no samples')
    mean = sum(norms) / len(norms)
    return math.sqrt(sum((n - mean) ** 2 for n in norms) / len(norms))


def vibration_ok(value: float) -> bool:
    """Return True if the vibration with the motors running is acceptable."""
    return value <= VIBRATION_TOL
