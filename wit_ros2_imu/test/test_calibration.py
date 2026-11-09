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

"""Unit tests for the software calibration model."""

import pathlib
import struct

import pytest
from wit_ros2_imu import calibration as cal
from wit_ros2_imu.wit_protocol import Accel, GRAVITY, Gyro, Mag
import yaml


# ---------------------------------------------------------------- apply_*

def test_default_calibration_is_zero():
    c = cal.SoftwareCalibration()
    assert tuple(c.gyro_bias) == (0.0, 0.0, 0.0)
    assert tuple(c.accel_bias) == (0.0, 0.0, 0.0)
    assert tuple(c.mag_offset) == (0.0, 0.0, 0.0)


def test_apply_gyro_subtracts_bias():
    c = cal.SoftwareCalibration(gyro_bias=(0.1, -0.2, 0.3))
    g = c.apply_gyro(Gyro(1.0, 1.0, 1.0))
    assert isinstance(g, Gyro)
    assert (g.x, g.y, g.z) == pytest.approx((0.9, 1.2, 0.7))


def test_apply_accel_subtracts_bias_keeps_temperature():
    c = cal.SoftwareCalibration(accel_bias=(0.5, -0.5, 0.25))
    a = c.apply_accel(Accel(1.0, 2.0, 10.0, 24.5))
    assert isinstance(a, Accel)
    assert (a.x, a.y, a.z) == pytest.approx((0.5, 2.5, 9.75))
    assert a.temperature == pytest.approx(24.5)


def test_apply_mag_subtracts_offset_keeps_temperature():
    c = cal.SoftwareCalibration(mag_offset=(10.0, -20.0, 5.5))
    m = c.apply_mag(Mag(100, 100, 100, 30.0))
    assert isinstance(m, Mag)
    assert (m.x, m.y, m.z) == pytest.approx((90.0, 120.0, 94.5))
    assert m.temperature == pytest.approx(30.0)


def test_apply_does_not_mutate_input():
    c = cal.SoftwareCalibration(gyro_bias=(1.0, 1.0, 1.0))
    g = Gyro(2.0, 2.0, 2.0)
    c.apply_gyro(g)
    assert (g.x, g.y, g.z) == (2.0, 2.0, 2.0)


# ---------------------------------------------------------------- YAML I/O

def test_load_missing_file_returns_default(tmp_path):
    c = cal.load_calibration(tmp_path / 'nope.yaml')
    assert c == cal.SoftwareCalibration()


def test_save_then_load_roundtrip(tmp_path):
    path = tmp_path / 'sub' / 'imu_calibration.yaml'
    original = cal.SoftwareCalibration(
        gyro_bias=(0.001, -0.002, 0.003),
        accel_bias=(0.1, 0.2, -0.3),
        mag_offset=(12.5, -40.0, 7.0),
    )
    cal.save_calibration(path, original)
    loaded = cal.load_calibration(str(path))
    assert loaded.gyro_bias == pytest.approx(original.gyro_bias)
    assert loaded.accel_bias == pytest.approx(original.accel_bias)
    assert loaded.mag_offset == pytest.approx(original.mag_offset)


def test_saved_file_matches_spec_format(tmp_path):
    path = tmp_path / 'c.yaml'
    cal.save_calibration(path, cal.SoftwareCalibration(gyro_bias=(1, 2, 3)))
    data = yaml.safe_load(path.read_text())
    assert set(data) == {'gyro_bias', 'accel_bias', 'mag_offset'}
    assert data['gyro_bias'] == [1.0, 2.0, 3.0]
    assert all(isinstance(v, float) for v in data['accel_bias'])


def test_load_partial_file_uses_defaults_for_missing_keys(tmp_path):
    path = tmp_path / 'c.yaml'
    path.write_text('gyro_bias: [0.1, 0.2, 0.3]\n')
    c = cal.load_calibration(path)
    assert c.gyro_bias == pytest.approx((0.1, 0.2, 0.3))
    assert tuple(c.accel_bias) == (0.0, 0.0, 0.0)
    assert tuple(c.mag_offset) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('content', [
    'gyro_bias: [0.1, 0.2\n',                         # YAML syntax error
    '- just\n- a list\n',                             # not a mapping
    '42\n',                                           # scalar
    '',                                               # empty document
    'gyro_bias: [0.1, 0.2]\n',                        # wrong length
    'accel_bias: [0.1, 0.2, 0.3, 0.4]\n',             # wrong length
    'mag_offset: [a, b, c]\n',                        # not numeric
    'gyro_bias: [true, false, true]\n',               # bool is not a number
    'gyro_bias: 0.5\n',                               # not a list
    'gyro_bias: [.nan, 0, 0]\n',                      # not finite
    'accel_bias: [.inf, 0, 0]\n',                     # not finite
    'gyro_bais: [0, 0, 0]\n',                         # unknown (typo) key
    '!!python/object/apply:os.system ["echo hi"]\n',  # unsafe tag
])
def test_load_invalid_file_raises_value_error(tmp_path, content):
    path = tmp_path / 'c.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        cal.load_calibration(path)


def test_save_rejects_invalid_calibration(tmp_path):
    with pytest.raises(ValueError):
        cal.save_calibration(tmp_path / 'c.yaml',
                             cal.SoftwareCalibration(gyro_bias=(0.0, 0.0)))
    assert not (tmp_path / 'c.yaml').exists()


# ---------------------------------------------------------------- compute_*

def _accels(n, x=0.0, y=0.0, z=GRAVITY):
    return [Accel(x, y, z, 25.0) for _ in range(n)]


def _gyros(n, x=0.0, y=0.0, z=0.0):
    return [Gyro(x, y, z) for _ in range(n)]


def test_compute_static_bias_level():
    accels = _accels(10, 0.1, -0.2, GRAVITY + 0.3)
    gyros = _gyros(10, 0.01, -0.02, 0.03)
    gyro_bias, accel_bias = cal.compute_static_bias(accels, gyros)
    assert gyro_bias == pytest.approx((0.01, -0.02, 0.03))
    assert accel_bias == pytest.approx((0.1, -0.2, 0.3))


def test_compute_static_bias_averages():
    accels = _accels(10, 0.0) + _accels(10, 0.2)
    gyros = _gyros(10, 0.0, 0.0, 0.1) + _gyros(10, 0.0, 0.0, 0.3)
    gyro_bias, accel_bias = cal.compute_static_bias(accels, gyros)
    assert gyro_bias == pytest.approx((0.0, 0.0, 0.2))
    assert accel_bias == pytest.approx((0.1, 0.0, 0.0))


def test_compute_static_bias_result_cancels_offset():
    accels = _accels(12, 0.3, 0.1, GRAVITY - 0.2)
    gyros = _gyros(12, 0.05, 0.05, -0.05)
    gyro_bias, accel_bias = cal.compute_static_bias(accels, gyros)
    c = cal.SoftwareCalibration(gyro_bias=gyro_bias, accel_bias=accel_bias)
    a = c.apply_accel(accels[0])
    g = c.apply_gyro(gyros[0])
    assert (a.x, a.y, a.z) == pytest.approx((0.0, 0.0, GRAVITY))
    assert (g.x, g.y, g.z) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


@pytest.mark.parametrize('n_acc,n_gyro', [(9, 10), (10, 9), (0, 0)])
def test_compute_static_bias_too_few_samples(n_acc, n_gyro):
    with pytest.raises(ValueError):
        cal.compute_static_bias(_accels(n_acc), _gyros(n_gyro))


def test_compute_mag_offset_hard_iron():
    mags = [Mag(x, -x, 2 * x, 25.0) for x in range(-100, 121, 20)]  # 12 samples
    off = cal.compute_mag_offset(mags)
    # x: (120 + -100)/2 = 10; y: (100 + -120)/2 = -10; z: (240 + -200)/2 = 20
    assert off == pytest.approx((10.0, -10.0, 20.0))


def test_compute_mag_offset_too_few_samples():
    with pytest.raises(ValueError):
        cal.compute_mag_offset([Mag(1, 2, 3, 25.0)] * 9)
    with pytest.raises(ValueError):
        cal.compute_mag_offset([])


# ---------------------------------------------------------------- magnetometer coverage

FIXTURES = pathlib.Path(__file__).parent / 'fixtures'


def _mag(x, y, z):
    return Mag(x, y, z, 25.0)


def _sweep(cov, axis, radius, center=(0.0, 0.0, 0.0), steps=21):
    """Sweep one axis from -radius to +radius around ``center``."""
    for i in range(steps):
        v = [center[0], center[1], center[2]]
        v[axis] += -radius + 2.0 * radius * i / (steps - 1)
        cov.add(_mag(*v))


def test_mag_coverage_starts_empty():
    cov = cal.MagCoverage(field_radius=1000.0)
    assert cov.samples == 0
    assert cov.axis_progress() == (0.0, 0.0, 0.0)
    assert cov.progress() == 0.0
    assert cov.missing_axes() == ['X', 'Y', 'Z']
    assert not cov.complete()


def test_mag_coverage_is_per_axis():
    cov = cal.MagCoverage(field_radius=1000.0)
    _sweep(cov, 0, 1000.0)
    x, y, z = cov.axis_progress()
    assert x == pytest.approx(1.0)
    assert y == 0.0 and z == 0.0
    assert cov.missing_axes() == ['Y', 'Z']
    assert not cov.complete()


def test_mag_coverage_target_fraction_counts_as_full_axis():
    cov = cal.MagCoverage(field_radius=1000.0, target=0.8)
    _sweep(cov, 1, 800.0)                       # span 1600 = 0.8 * 2R -> axis done
    assert cov.axis_progress()[1] == pytest.approx(1.0)
    cov2 = cal.MagCoverage(field_radius=1000.0, target=0.8)
    _sweep(cov2, 1, 400.0)                      # half of the required span
    assert cov2.axis_progress()[1] == pytest.approx(0.5)


def test_mag_coverage_ignores_hard_iron_offset():
    cov = cal.MagCoverage(field_radius=1000.0)
    center = (3000.0, -2500.0, 800.0)
    for axis in range(3):
        _sweep(cov, axis, 1000.0, center)
    assert cov.complete()
    assert cov.progress() == pytest.approx(1.0)
    assert cov.missing_axes() == []


def test_mag_coverage_radius_grows_with_observed_span():
    # A too-small reference radius must not make a partial sweep look complete.
    cov = cal.MagCoverage(field_radius=100.0)
    _sweep(cov, 0, 1000.0)
    _sweep(cov, 1, 300.0)
    assert cov.radius == pytest.approx(1000.0)
    assert cov.axis_progress()[1] < 0.5


def test_mag_coverage_without_reference_radius_uses_largest_span():
    cov = cal.MagCoverage()
    _sweep(cov, 2, 500.0)
    assert cov.radius == pytest.approx(500.0)
    assert cov.missing_axes() == ['X', 'Y']


@pytest.mark.parametrize('radius', [0.0, -5.0, float('nan'), float('inf')])
def test_mag_coverage_rejects_invalid_radius(radius):
    with pytest.raises(ValueError):
        cal.MagCoverage(field_radius=radius)


@pytest.mark.parametrize('target', [0.0, 1.5, float('nan')])
def test_mag_coverage_rejects_invalid_target(target):
    with pytest.raises(ValueError):
        cal.MagCoverage(target=target)


def test_mag_coverage_ignores_non_finite_samples():
    cov = cal.MagCoverage(field_radius=1000.0)
    cov.add(_mag(float('nan'), 0.0, 0.0))
    cov.add(_mag(0.0, float('inf'), 0.0))
    assert cov.samples == 0


def _frames(path):
    data = path.read_bytes()
    out = {}
    i = 0
    while i + 11 <= len(data):
        if data[i] == 0x55 and sum(data[i:i + 10]) & 0xFF == data[i + 10]:
            out.setdefault(data[i + 1], []).append(struct.unpack('<hhhh', data[i + 2:i + 10]))
            i += 11
        else:
            i += 1
    return out


def test_real_mag_calibration_mode_freezes_gyro_but_not_mag():
    """
    Regression evidence (CMP10A, 2026-11-09).

    Recorded for 25 s in CALSW=0x07 while the IMU was rotated by hand: the
    accel / gyro / angle packets stay constant, only 0x54 changes.  Progress
    during hardware magnetometer calibration therefore cannot use the gyro.
    """
    frames = _frames(FIXTURES / 'cmp10a_magcal_mode_rotating_9600.bin')
    for ptype in (0x51, 0x52, 0x53):
        assert len(set(frames[ptype])) == 1, hex(ptype)
    mags = frames[0x54]
    for axis in range(3):
        assert max(m[axis] for m in mags) - min(m[axis] for m in mags) > 5000


def test_mag_coverage_on_real_mag_calibration_recording_completes():
    frames = _frames(FIXTURES / 'cmp10a_magcal_mode_rotating_9600.bin')
    cov = cal.MagCoverage(field_radius=4070.0)   # |B| measured in normal mode
    for x, y, z, _ in frames[0x54]:
        cov.add(_mag(x, y, z))
    assert cov.complete(), cov.axis_progress()
