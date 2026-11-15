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

# ------------------------------------------------------------------ gyro

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


# ---------------------------------------- environment and mounting position

def test_field_noise_is_small_when_the_field_is_steady():
    mags = [(4000.0 + (i % 3) * 10.0, 0.0, 0.0) for i in range(50)]   # +-10 LSB jitter
    assert cc.field_noise(mags) < 0.005
    assert cc.noise_ok(cc.field_noise(mags))


def test_field_noise_sees_a_changing_field():
    mags = [(4000.0 + (300.0 if i % 2 else 0.0), 0.0, 0.0) for i in range(50)]
    assert not cc.noise_ok(cc.field_noise(mags))


def test_field_change_compares_against_the_reference_position():
    ref = [(4000.0, 0.0, 0.0)] * 5
    same = [(4010.0, 5.0, -5.0)] * 40
    near_iron = [(4000.0, 0.0, 0.0)] * 20 + [(4000.0, 700.0, 0.0)] * 20
    assert cc.change_ok(cc.field_change(ref, same))
    assert not cc.change_ok(cc.field_change(ref, near_iron))


def test_field_change_needs_data():
    with pytest.raises(ValueError):
        cc.field_change([], [(1.0, 0.0, 0.0)])


def test_rest_change_compares_the_two_resting_places():
    start = [(4000.0, 0.0, 0.0)] * 10
    assert cc.rest_change(start, [(4000.0, 0.0, 0.0)] * 10) == pytest.approx(0.0)
    assert cc.rest_change(start, [(4000.0, 400.0, 0.0)] * 10) == pytest.approx(0.1)


def test_rest_change_needs_both_places():
    with pytest.raises(ValueError):
        cc.rest_change([], [(1.0, 0.0, 0.0)])
    with pytest.raises(ValueError):
        cc.rest_change([(1.0, 0.0, 0.0)], [])
    with pytest.raises(ValueError):
        cc.rest_change([(0.0, 0.0, 0.0)], [(1.0, 0.0, 0.0)])


def test_net_rotation_follows_a_turn():
    net = cc.NetRotation()
    for i in range(11):                       # 2.5 deg/s about Z for 1 s
        net.add(i * 0.1, (0.0, 0.0, math.radians(2.5)))
    assert net.angle_deg == pytest.approx(2.5)


def test_net_rotation_ignores_a_wobble_that_comes_back():
    net, meter = cc.NetRotation(), cc.RotationMeter()
    rates = [math.radians(20.0)] * 10 + [math.radians(-20.0)] * 10
    for i, rate in enumerate(rates):
        net.add(i * 0.1, (rate, 0.0, 0.0))
        meter.add(i * 0.1, (rate, 0.0, 0.0))
    assert net.angle_deg == pytest.approx(0.0, abs=1e-6)
    assert meter.total_deg == pytest.approx(38.0)     # 19 intervals of 2 deg, any direction


def test_net_rotation_skips_gaps():
    net = cc.NetRotation()
    net.add(0.0, (0.0, 0.0, 1.0))
    net.add(cc.MAX_GAP_S + 1.0, (0.0, 0.0, 1.0))
    assert net.angle_deg == 0.0


def test_turning_explains_part_of_the_slide_change():
    """Regression: turning 3 deg alone moves the reading 5.2 % and failed the 5 % limit."""
    assert cc.rotation_allowance(3.0) == pytest.approx(2 * math.sin(math.radians(1.5)))
    assert cc.rotation_allowance(0.0) == 0.0
    expected = cc.ENV_CHANGE_TOL + cc.rotation_allowance(3.0)
    assert cc.slide_change_tol(3.0) == pytest.approx(expected)
    assert cc.slide_change_ok(0.052, 3.0)
    assert not cc.slide_change_ok(0.052, 0.0)
    assert not cc.slide_change_ok(0.12, 2.5)          # turning cannot hide iron


def test_rotation_meter_adds_up_turning_on_any_axis():
    meter = cc.RotationMeter()
    for i in range(11):                       # 10 deg/s about Y for 1 s
        meter.add(i * 0.1, (0.0, math.radians(10.0), 0.0))
    assert meter.total_deg == pytest.approx(10.0)
    assert not cc.slide_valid(meter.total_deg)
    assert cc.slide_valid(1.0)


def test_tilt_from_z_up():
    assert cc.tilt_deg((0.0, 0.0, G)) == pytest.approx(0.0)
    assert cc.tilt_deg((G * math.sin(math.radians(8)), 0.0, G * math.cos(math.radians(8)))) \
        == pytest.approx(8.0)
    assert cc.tilt_deg((0.0, 0.0, -G)) == pytest.approx(180.0)


def test_vibration_is_the_spread_of_the_acceleration():
    calm = [(0.0, 0.0, G)] * 20
    shaking = [(0.0, 0.0, G + (2.0 if i % 2 else -2.0)) for i in range(20)]
    assert cc.vibration(calm) == pytest.approx(0.0)
    assert cc.vibration(shaking) == pytest.approx(2.0)
    assert not cc.vibration_ok(cc.vibration(shaking))
