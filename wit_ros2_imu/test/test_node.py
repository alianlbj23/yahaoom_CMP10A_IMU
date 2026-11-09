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

"""Tests for the wit_ros2_imu driver node and its ROS-free helpers."""

import math
from pathlib import Path
import socket
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField, Temperature
import serial
from wit_ros2_imu.calibration import SoftwareCalibration
from wit_ros2_imu.wit_protocol import Accel, Angle, euler_to_quaternion, GRAVITY, Gyro, Quaternion
from wit_ros2_imu.wit_ros2_imu import (
    build_imu_msg,
    ImuSample,
    PacketProcessor,
    SerialReader,
    WitImuNode,
)

FIXTURE = Path(__file__).parent / 'fixtures' / 'cmp10a_stationary_9600.bin'


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

class FixtureServer:
    """TCP server that streams the recorded IMU bytes (pyserial socket:// URL)."""

    def __init__(self, data, chunk=44, period=0.01):
        self._data = data
        self._chunk = chunk
        self._period = period
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('127.0.0.1', 0))
        self._sock.listen(1)
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(1.0)
                try:
                    while not self._stop.is_set():
                        for i in range(0, len(self._data), self._chunk):
                            conn.sendall(self._data[i:i + self._chunk])
                            time.sleep(self._period)
                except OSError:
                    continue

    def close(self):
        self._stop.set()
        self._thread.join(2.0)
        self._sock.close()


class FakePort:
    """Minimal pyserial stand-in: returns queued chunks, raises queued exceptions."""

    def __init__(self, items):
        self._items = list(items)
        self.closed = False

    @property
    def in_waiting(self):
        if self._items and isinstance(self._items[0], bytes):
            return len(self._items[0])
        return 0

    def read(self, size):
        if not self._items:
            time.sleep(0.01)
            return b''
        item = self._items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def spin_until(executor, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        executor.spin_once(timeout_sec=0.05)
    return predicate()


@pytest.fixture
def ros_context():
    context = Context()
    rclpy.init(context=context)
    yield context
    rclpy.try_shutdown(context=context)


@pytest.fixture
def namespace():
    return '/test_' + uuid.uuid4().hex[:8]


# --------------------------------------------------------------------------
# PacketProcessor (no ROS)
# --------------------------------------------------------------------------

def test_processor_emits_sample_on_angle_after_accel_and_gyro():
    proc = PacketProcessor()
    assert proc.process(Angle(0.0, 0.0, 0.0)) is None  # no accel/gyro yet
    accel = Accel(0.0, 0.0, GRAVITY, 25.0)
    assert proc.process(accel) == accel  # temperature output
    assert proc.process(Gyro(0.01, 0.02, 0.03)) is None
    sample = proc.process(Angle(0.1, -0.2, 0.3))
    assert isinstance(sample, ImuSample)
    expected = euler_to_quaternion(0.1, -0.2, 0.3)
    assert sample.orientation == expected
    assert sample.accel.z == pytest.approx(GRAVITY)
    assert sample.gyro.z == pytest.approx(0.03)


def test_processor_prefers_device_quaternion_then_falls_back():
    now = [100.0]
    proc = PacketProcessor(clock=lambda: now[0], quaternion_timeout=1.0)
    proc.process(Accel(0.0, 0.0, GRAVITY, 25.0))
    proc.process(Gyro(0.0, 0.0, 0.0))
    q = Quaternion(1.0, 0.0, 0.0, 0.0)
    sample = proc.process(q)
    assert isinstance(sample, ImuSample)
    assert sample.orientation == q
    assert proc.process(Angle(0.5, 0.0, 0.0)) is None  # suppressed: quaternion active
    now[0] += 2.0  # quaternion stream stopped
    assert isinstance(proc.process(Angle(0.5, 0.0, 0.0)), ImuSample)


def test_processor_applies_software_calibration():
    cal = SoftwareCalibration(gyro_bias=(0.1, -0.2, 0.3), accel_bias=(0.5, 0.0, -0.5),
                              mag_offset=(10, 20, 30))
    proc = PacketProcessor(cal)
    raw_accel = Accel(1.0, 2.0, 3.0, 20.0)
    raw_gyro = Gyro(1.0, 1.0, 1.0)
    proc.process(raw_accel)
    proc.process(raw_gyro)
    sample = proc.process(Angle(0.0, 0.0, 0.0))
    assert sample.accel == cal.apply_accel(raw_accel)
    assert sample.gyro == cal.apply_gyro(raw_gyro)
    assert sample.gyro != raw_gyro


def test_build_imu_msg_has_covariances_and_frame():
    sample = ImuSample(Quaternion(1.0, 0.0, 0.0, 0.0), Accel(0.0, 0.0, GRAVITY, 20.0),
                       Gyro(0.0, 0.0, 0.0))
    msg = build_imu_msg(sample, Time().to_msg(), 'imu_link')
    assert msg.header.frame_id == 'imu_link'
    assert msg.orientation.w == pytest.approx(1.0)
    for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                msg.linear_acceleration_covariance):
        assert cov[0] > 0 and cov[4] > 0 and cov[8] > 0
        assert cov[1] == 0.0


# --------------------------------------------------------------------------
# SerialReader (no ROS)
# --------------------------------------------------------------------------

def test_reader_reconnects_after_read_error():
    ports = [FakePort([b'ab', serial.SerialException('device gone')]), FakePort([b'cd'])]
    received = bytearray()
    errors = []
    reader = SerialReader(opener=lambda: ports.pop(0) if ports else FakePort([]),
                          on_data=received.extend, on_error=errors.append,
                          reconnect_interval=0.05)
    reader.start()
    try:
        assert wait_until(lambda: bytes(received) == b'abcd')
    finally:
        reader.stop()
    assert not reader.is_alive()
    assert reader.connections >= 2
    assert reader.read_failures == 1
    assert any('device gone' in e for e in errors)


def test_reader_retries_when_port_cannot_be_opened():
    attempts = []

    def opener():
        attempts.append(1)
        if len(attempts) < 3:
            raise serial.SerialException('no such device')
        return FakePort([b'\x55'])

    received = bytearray()
    errors = []
    reader = SerialReader(opener=opener, on_data=received.extend, on_error=errors.append,
                          reconnect_interval=0.05)
    reader.start()
    try:
        assert wait_until(lambda: received == b'\x55')
    finally:
        reader.stop()
    assert reader.connect_failures == 2
    assert len([e for e in errors if 'Cannot open serial port' in e]) == 2


def test_reader_rejects_bad_interval():
    with pytest.raises(ValueError):
        SerialReader(opener=FakePort, on_data=print, on_error=print, reconnect_interval=0)


# --------------------------------------------------------------------------
# WitImuNode (ROS)
# --------------------------------------------------------------------------

def test_node_publishes_imu_from_recorded_stream(ros_context, namespace):
    server = FixtureServer(FIXTURE.read_bytes())
    node = WitImuNode(context=ros_context, namespace=namespace, parameter_overrides=[
        Parameter('port', value=f'socket://127.0.0.1:{server.port}'),
        Parameter('calibration_file', value=''),
        Parameter('reconnect_interval', value=0.2),
    ])
    listener = rclpy.create_node('listener', context=ros_context, namespace=namespace)
    imu_msgs, mag_msgs, temp_msgs = [], [], []
    listener.create_subscription(Imu, 'imu/data', imu_msgs.append, 10)
    listener.create_subscription(MagneticField, 'imu/mag', mag_msgs.append, 10)
    listener.create_subscription(Temperature, 'imu/temperature', temp_msgs.append, 10)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(listener)
    try:
        assert spin_until(executor, lambda: len(imu_msgs) >= 5 and mag_msgs and temp_msgs,
                          timeout=15.0), 'no IMU data received from recorded stream'
    finally:
        executor.shutdown()
        node.destroy_node()
        listener.destroy_node()
        server.close()

    assert not node.reader.is_alive()
    msg = imu_msgs[-1]
    assert msg.header.frame_id == 'imu_link'
    a = msg.linear_acceleration
    norm = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
    assert norm == pytest.approx(GRAVITY, abs=0.3)  # stationary -> ~1 g
    assert a.z == pytest.approx(GRAVITY, abs=0.3)  # recorded lying flat
    q = msg.orientation
    assert math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2) == pytest.approx(1.0, abs=1e-3)
    w = msg.angular_velocity
    assert max(abs(w.x), abs(w.y), abs(w.z)) < 0.1  # rad/s, stationary
    assert 0.0 < temp_msgs[-1].temperature < 80.0


def test_node_survives_unopenable_port(ros_context, namespace, capfd):
    node = WitImuNode(context=ros_context, namespace=namespace, parameter_overrides=[
        Parameter('port', value='/dev/does_not_exist_wit_imu'),
        Parameter('calibration_file', value=''),
        Parameter('reconnect_interval', value=0.1),
    ])
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        spin_until(executor, lambda: node.reader.connect_failures >= 3, timeout=5.0)
        assert node.reader.connect_failures >= 3
        assert node.reader.is_alive()
        assert not node.reader.connected
        assert ros_context.ok()
    finally:
        executor.shutdown()
        node.destroy_node()
    assert not node.reader.is_alive()
    assert 'Cannot open serial port' in node.reader.last_error
    assert 'Cannot open serial port' in capfd.readouterr().err


def test_node_rejects_invalid_parameters(ros_context, namespace):
    with pytest.raises(ValueError):
        WitImuNode(context=ros_context, namespace=namespace, parameter_overrides=[
            Parameter('baud', value=-1),
        ])
