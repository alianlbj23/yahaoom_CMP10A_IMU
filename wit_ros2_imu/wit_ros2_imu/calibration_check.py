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
Environment and mounting check before a magnetometer calibration (pure Python, no ROS).

The Earth's field is the same everywhere in a room and does not change over
seconds.  So a field that moves while the IMU is still, changes when the IMU
is slid without turning, or changes when the motors run, comes from the
surroundings.  These checks only compare readings with each other: they need
no calibration and no unit conversion.

The thresholds below are this project's own choices; the reasoning for each
is written down in docs/CALIBRATION_CHECK.md.
"""

import math
from typing import Sequence, Tuple

Vector3 = Tuple[float, float, float]

MAG_SPREAD_SHARE = 0.95      # share of samples that decides a field change (spikes ignored)
MAX_GAP_S = 0.5              # longer gaps in gyro data are not integrated


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def _mean3(samples: Sequence[Sequence[float]]) -> Vector3:
    if not samples:
        raise ValueError('no samples')
    n = float(len(samples))
    return (sum(s[0] for s in samples) / n, sum(s[1] for s in samples) / n,
            sum(s[2] for s in samples) / n)


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


def _quat_mul(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


class NetRotation:
    """
    Follow the net rotation from gyro samples (rad/s), as one angle.

    Unlike RotationMeter, a wobble that comes back adds nothing: only how far
    the IMU ends up turned from where it started counts.
    """

    def __init__(self):
        """Start unturned."""
        self._q = (1.0, 0.0, 0.0, 0.0)
        self._last = None

    def add(self, t: float, rate_xyz: Sequence[float]) -> float:
        """Add one sample taken at time ``t``; return the net angle in degrees."""
        rate = tuple(float(c) for c in rate_xyz)
        if self._last is not None:
            t0, r0 = self._last
            dt = t - t0
            if 0.0 < dt <= MAX_GAP_S:
                mid = [(a + b) / 2.0 for a, b in zip(r0, rate)]
                speed = _norm(mid)
                if speed > 0.0:
                    half = speed * dt / 2.0
                    s = math.sin(half) / speed
                    self._q = _quat_mul(self._q, (math.cos(half), mid[0] * s,
                                                  mid[1] * s, mid[2] * s))
        self._last = (t, rate)
        return self.angle_deg

    @property
    def angle_deg(self) -> float:
        """Net angle turned so far, in degrees."""
        n = _norm(self._q)
        return math.degrees(2.0 * math.acos(min(1.0, abs(self._q[0]) / n)))


def rest_change(start: Sequence[Sequence[float]], end: Sequence[Sequence[float]]) -> float:
    """
    Return how far the mean field at rest moved between two places, as a fraction.

    Only readings taken at rest are compared, so hand wobble and sensor noise
    while the IMU is being pushed do not count.
    """
    ref = _mean3(start)
    ref_norm = _norm(ref)
    if ref_norm <= 0:
        raise ValueError('magnetometer reads zero')
    moved = _mean3(end)
    return _norm([a - b for a, b in zip(moved, ref)]) / ref_norm


def rotation_allowance(net_deg: float) -> float:
    """
    Return the most a turn of ``net_deg`` can move the field reading, as a fraction.

    Turning by an angle moves a fixed vector by at most 2 sin(angle / 2) of its
    length (when the turn axis is at right angles to the field).
    """
    return 2.0 * math.sin(math.radians(abs(net_deg)) / 2.0)


def slide_change_tol(net_deg: float) -> float:
    """Return the slide limit: ENV_CHANGE_TOL plus what the turn alone can explain."""
    return ENV_CHANGE_TOL + rotation_allowance(net_deg)


def slide_change_ok(change: float, net_deg: float) -> bool:
    """Return True if the field change after a slide is within slide_change_tol."""
    return change <= slide_change_tol(net_deg)


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
