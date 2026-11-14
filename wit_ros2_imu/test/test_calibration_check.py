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


"""Tests for the calibration check analysis (reference-based pass / fail)."""

import math

import pytest
from wit_ros2_imu import calibration_check as cc
from wit_ros2_imu.wit_protocol import GRAVITY

G = GRAVITY


def _unit(axis, sign):
    v = [0.0, 0.0, 0.0]
    v[cc.axis_index(axis)] = sign
    return v


# ------------------------------------------------------------- six faces

def test_face_detection_uses_the_axis_that_reads_plus_g():
    assert cc.facing((0.1, -0.2, 9.8), 'Z', +1)
    assert not cc.facing((0.1, -0.2, 9.8), 'Z', -1)
    assert cc.facing((-9.7, 0.3, 0.5), 'X', -1)
    assert not cc.facing((6.0, 0.0, 7.0), 'X', +1)        # 40 deg tilted: not on the face
    assert cc.face_name((0.0, -9.8, 0.0)) == 'Y 軸朝下'


def test_six_faces_pass_with_a_perfect_accelerometer():
    faces = {(a, s): [tuple(G * c for c in _unit(a, s))] * 50 for a, s in cc.FACES}
    result = cc.evaluate_six_faces({k: cc.evaluate_face(k[0], k[1], v) for k, v in faces.items()})
    assert result.ok
    assert all(abs(o) < 1e-9 for o in result.offsets)


def test_six_faces_find_a_constant_offset():
    """An offset of +0.3 on X shows up as the mean of the X-up and X-down readings."""
    bias = (0.3, 0.0, 0.0)
    faces = {}
    for a, s in cc.FACES:
        v = [G * c + b for c, b in zip(_unit(a, s), bias)]
        faces[(a, s)] = cc.evaluate_face(a, s, [tuple(v)] * 50)
    result = cc.evaluate_six_faces(faces)
    assert result.offsets[0] == pytest.approx(0.3)
    assert not result.offsets_ok
    assert not result.ok


def test_six_faces_find_a_scale_error_through_the_magnitude():
    faces = {(a, s): cc.evaluate_face(a, s, [tuple(1.02 * G * c for c in _unit(a, s))] * 50)
             for a, s in cc.FACES}
    result = cc.evaluate_six_faces(faces)
    assert not result.magnitudes_ok          # 2 % scale error = about 0.2 m/s^2
    assert result.offsets_ok


def test_six_faces_need_all_faces():
    with pytest.raises(ValueError):
        cc.evaluate_six_faces({})


# ------------------------------------------------------------------ gyro

def test_gyro_bias_is_reported_in_degrees_per_second():
    samples = [(math.radians(0.01), 0.0, math.radians(-0.05))] * 100
    bias = cc.gyro_bias_dps(samples)
    assert bias == pytest.approx((0.01, 0.0, -0.05))
    assert not cc.gyro_bias_ok(bias)
    assert cc.gyro_bias_ok((0.01, -0.015, 0.0))


def test_gyro_bias_needs_samples():
    with pytest.raises(ValueError):
        cc.gyro_bias_dps([])


def test_turn_integrator_adds_up_a_full_turn():
    integ = cc.TurnIntegrator()
    rate = math.radians(36.0)                # 36 deg/s for 10 s
    for i in range(101):
        integ.add(i * 0.1, rate)
    assert integ.angle_deg == pytest.approx(360.0)
    assert cc.turn_ok(integ.angle_deg)
    assert cc.turn_ok(-358.0)
    assert not cc.turn_ok(350.0)


def test_turn_integrator_ignores_gaps():
    """A pause in the data (e.g. a dropped USB packet) must not be integrated."""
    integ = cc.TurnIntegrator()
    integ.add(0.0, math.radians(10.0))
    integ.add(5.0, math.radians(10.0))
    assert integ.angle_deg == 0.0


# ------------------------------------------------------------- magnetometer

def _sphere(radius, centre=(0.0, 0.0, 0.0), n=400):
    pts = []
    for i in range(n):
        z = 1 - 2 * (i + 0.5) / n
        r = math.sqrt(1 - z * z)
        phi = i * math.pi * (3 - math.sqrt(5))
        pts.append(tuple(c + radius * v for c, v in
                         zip(centre, (r * math.cos(phi), r * math.sin(phi), z))))
    return pts


def test_magnitude_spread_is_small_on_a_sphere():
    spread = cc.magnitude_spread(_sphere(4000.0))
    assert spread == pytest.approx(0.0, abs=1e-9)
    assert cc.magnitude_ok(spread)


def test_magnitude_spread_detects_a_hard_iron_offset():
    spread = cc.magnitude_spread(_sphere(4000.0, centre=(600.0, 0.0, 0.0)))
    assert spread > 0.10
    assert not cc.magnitude_ok(spread)


def test_magnitude_spread_ignores_a_few_spikes():
    pts = _sphere(4000.0) + [(9000.0, 0.0, 0.0)] * 5      # about 1 % outliers
    assert cc.magnitude_ok(cc.magnitude_spread(pts))


def test_dip_angle_of_an_inclined_field():
    """Field 36 deg below the horizon, IMU level with Z up."""
    dip = math.radians(36.0)
    mag = (4000 * math.cos(dip), 0.0, -4000 * math.sin(dip))   # Z up, field points down
    accel = (0.0, 0.0, G)
    assert cc.dip_angle_deg(accel, mag) == pytest.approx(36.0)
    assert cc.mean_dip([(accel, mag)] * 10) == pytest.approx(36.0)
    assert cc.dip_ok(36.0, 34.0)
    assert not cc.dip_ok(36.0, 40.0)


def test_dip_angle_does_not_depend_on_how_the_imu_is_held():
    dip = math.radians(36.0)
    field = (4000 * math.cos(dip), 0.0, -4000 * math.sin(dip))
    up = (0.0, 0.0, G)
    # Rotate both vectors 90 deg about X: the angle between them is unchanged.
    rot = (lambda v: (v[0], -v[2], v[1]))
    assert cc.dip_angle_deg(rot(up), rot(field)) == pytest.approx(36.0)


@pytest.mark.parametrize('text,value', [('36', 36.0), (' 35.5 ', 35.5), ('-20', -20.0)])
def test_parse_dip_accepts_degrees(text, value):
    assert cc.parse_dip(text) == value


@pytest.mark.parametrize('text', ['abc', '91', '-90.5', 'nan', 'inf', '1e5'])
def test_parse_dip_rejects_bad_values(text):
    with pytest.raises(ValueError):
        cc.parse_dip(text)
