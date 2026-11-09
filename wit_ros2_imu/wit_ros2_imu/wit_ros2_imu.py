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
ROS 2 driver node for the Yahboom CMP10A (WitMotion protocol) 10-axis IMU.

The serial / protocol logic (``SerialReader``, ``PacketProcessor``) does not
depend on rclpy so it can be unit tested without a ROS graph; ``WitImuNode``
only wires those pieces to ROS publishers.
"""

from dataclasses import dataclass
import math
import os
import signal
import sys
import threading
import time
import traceback

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Temperature
import serial
from tf2_ros import TransformBroadcaster
from wit_ros2_imu.calibration import load_calibration, SoftwareCalibration
from wit_ros2_imu.wit_protocol import (
    Accel,
    Angle,
    euler_to_quaternion,
    FrameParser,
    Gyro,
    Mag,
    Pressure,
    Quaternion,
)

SERIAL_TIMEOUT_S = 0.5
# If no 0x59 quaternion packet arrives for this long, fall back to 0x53 Euler.
QUATERNION_TIMEOUT_S = 1.0

# Diagonal covariances (row-major 3x3). Rough figures for a consumer-grade
# MEMS IMU (WitMotion datasheet: roll/pitch ~0.05 deg static, yaw drifts
# without a good mag calibration); conservative so filters do not over-trust.
_ROLL_PITCH_STD_RAD = math.radians(0.5)
_YAW_STD_RAD = math.radians(2.0)
_GYRO_STD_RAD_S = math.radians(0.1)
_ACCEL_STD_M_S2 = 0.01 * 9.80665
ORIENTATION_COVARIANCE = (
    _ROLL_PITCH_STD_RAD ** 2, 0.0, 0.0,
    0.0, _ROLL_PITCH_STD_RAD ** 2, 0.0,
    0.0, 0.0, _YAW_STD_RAD ** 2,
)
ANGULAR_VELOCITY_COVARIANCE = (
    _GYRO_STD_RAD_S ** 2, 0.0, 0.0,
    0.0, _GYRO_STD_RAD_S ** 2, 0.0,
    0.0, 0.0, _GYRO_STD_RAD_S ** 2,
)
LINEAR_ACCELERATION_COVARIANCE = (
    _ACCEL_STD_M_S2 ** 2, 0.0, 0.0,
    0.0, _ACCEL_STD_M_S2 ** 2, 0.0,
    0.0, 0.0, _ACCEL_STD_M_S2 ** 2,
)


@dataclass
class ImuSample:
    """One complete IMU sample, ready to be published as sensor_msgs/Imu."""

    orientation: Quaternion
    accel: Accel
    gyro: Gyro


class PacketProcessor:
    """
    Apply software calibration and decide what each packet should publish.

    ``process`` returns one of: ``ImuSample`` (publish Imu + TF), ``Accel``
    (publish temperature), ``Mag``, ``Pressure`` or ``None`` (nothing to do).
    An Imu sample is emitted for every 0x53 Euler packet, or for every 0x59
    quaternion packet while the device is sending them (device quaternion
    preferred).
    """

    def __init__(self, calibration=None, clock=time.monotonic,
                 quaternion_timeout=QUATERNION_TIMEOUT_S):
        self._cal = calibration if calibration is not None else SoftwareCalibration()
        self._clock = clock
        self._quaternion_timeout = quaternion_timeout
        self._last_quaternion_time = None
        self.accel = None
        self.gyro = None

    def _quaternion_active(self):
        if self._last_quaternion_time is None:
            return False
        return self._clock() - self._last_quaternion_time <= self._quaternion_timeout

    def _sample(self, orientation):
        if self.accel is None or self.gyro is None:
            return None
        return ImuSample(orientation=orientation, accel=self.accel, gyro=self.gyro)

    def process(self, packet):
        """Process one decoded packet; return what to publish (or None)."""
        if isinstance(packet, Accel):
            self.accel = self._cal.apply_accel(packet)
            return self.accel
        if isinstance(packet, Gyro):
            self.gyro = self._cal.apply_gyro(packet)
            return None
        if isinstance(packet, Quaternion):
            self._last_quaternion_time = self._clock()
            return self._sample(packet)
        if isinstance(packet, Angle):
            if self._quaternion_active():
                return None
            return self._sample(
                euler_to_quaternion(packet.roll, packet.pitch, packet.yaw))
        if isinstance(packet, Mag):
            return self._cal.apply_mag(packet)
        if isinstance(packet, Pressure):
            return packet
        return None


class SerialReader:
    """
    Background thread that keeps a serial port open and streams its bytes.

    ``opener()`` must return an open pyserial-like object (``read``,
    ``in_waiting``, ``close``). On open/read failure ``on_error(msg)`` is
    called, the port is closed and re-opened after ``reconnect_interval``
    seconds. The thread never exits on I/O errors; only ``stop()`` ends it.
    """

    def __init__(self, opener, on_data, on_error, on_connect=None,
                 reconnect_interval=2.0):
        if reconnect_interval <= 0:
            raise ValueError('reconnect_interval must be > 0')
        self._opener = opener
        self._on_data = on_data
        self._on_error = on_error
        self._on_connect = on_connect
        self._reconnect_interval = reconnect_interval
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name='wit_serial_reader',
                                        daemon=True)
        self._port = None
        self.connect_failures = 0
        self.read_failures = 0
        self.connections = 0
        self.last_error = None

    @property
    def connected(self):
        """Return True while a port is open."""
        return self._port is not None

    def is_alive(self):
        """Return True while the reader thread is running."""
        return self._thread.is_alive()

    def start(self):
        """Start the reader thread."""
        self._thread.start()

    def stop(self, timeout=3.0):
        """Ask the thread to exit and wait for it."""
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout)

    def _report(self, message):
        self.last_error = message
        self._on_error(message)

    def _close(self):
        port, self._port = self._port, None
        if port is None:
            return
        try:
            port.close()
        except (serial.SerialException, OSError) as exc:
            self._on_error(f'Error while closing serial port: {exc}')

    def _run(self):
        try:
            while not self._stop_event.is_set():
                if self._port is None:
                    try:
                        self._port = self._opener()
                    except (serial.SerialException, OSError, ValueError) as exc:
                        self.connect_failures += 1
                        self._report(f'Cannot open serial port: {exc}; retrying in '
                                     f'{self._reconnect_interval:.1f} s')
                        self._stop_event.wait(self._reconnect_interval)
                        continue
                    self.connections += 1
                    if self._on_connect is not None:
                        self._on_connect()
                try:
                    data = self._port.read(max(1, self._port.in_waiting))
                except (serial.SerialException, OSError) as exc:
                    self.read_failures += 1
                    self._report(f'Serial read failed (device disconnected?): {exc}; '
                                 f'reconnecting in {self._reconnect_interval:.1f} s')
                    self._close()
                    self._stop_event.wait(self._reconnect_interval)
                    continue
                if not data:
                    continue
                try:
                    self._on_data(data)
                except Exception:  # noqa: B902 - keep the reader alive, but report
                    self._report('Error while handling serial data:\n'
                                 + traceback.format_exc())
        finally:
            self._close()


def _fill_quaternion(target, q):
    target.w = float(q.w)
    target.x = float(q.x)
    target.y = float(q.y)
    target.z = float(q.z)


def build_imu_msg(sample, stamp, frame_id):
    """Convert an ``ImuSample`` into a ``sensor_msgs/Imu`` message."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    _fill_quaternion(msg.orientation, sample.orientation)
    msg.orientation_covariance = list(ORIENTATION_COVARIANCE)
    msg.angular_velocity.x = float(sample.gyro.x)
    msg.angular_velocity.y = float(sample.gyro.y)
    msg.angular_velocity.z = float(sample.gyro.z)
    msg.angular_velocity_covariance = list(ANGULAR_VELOCITY_COVARIANCE)
    msg.linear_acceleration.x = float(sample.accel.x)
    msg.linear_acceleration.y = float(sample.accel.y)
    msg.linear_acceleration.z = float(sample.accel.z)
    msg.linear_acceleration_covariance = list(LINEAR_ACCELERATION_COVARIANCE)
    return msg


def build_transform(sample, stamp, parent_frame, child_frame):
    """Build the ``parent_frame -> child_frame`` orientation-only transform."""
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    _fill_quaternion(tf.transform.rotation, sample.orientation)
    return tf


class WitImuNode(Node):
    """Publish WitMotion IMU data on imu/data, imu/mag, imu/temperature, imu/pressure."""

    def __init__(self, **kwargs):
        super().__init__('wit_ros2_imu', **kwargs)
        self._port_url = self._str_param('port', '/dev/ttyUSB0')
        self._baud = self.declare_parameter('baud', 9600).value
        self._frame_id = self._str_param('frame_id', 'imu_link')
        self._parent_frame = self._str_param('parent_frame', 'world')
        publish_tf = self.declare_parameter('publish_tf', True).value
        calibration_file = self.declare_parameter(
            'calibration_file', '/config/imu_calibration.yaml').value
        reconnect_interval = self.declare_parameter('reconnect_interval', 2.0).value

        if not isinstance(self._baud, int) or isinstance(self._baud, bool) \
                or self._baud <= 0:
            raise ValueError(f'parameter baud must be a positive integer, got {self._baud!r}')
        if not isinstance(publish_tf, bool):
            raise ValueError(f'parameter publish_tf must be a bool, got {publish_tf!r}')
        if not isinstance(calibration_file, str):
            raise ValueError('parameter calibration_file must be a string')
        if not isinstance(reconnect_interval, (int, float)) \
                or isinstance(reconnect_interval, bool) or reconnect_interval <= 0:
            raise ValueError('parameter reconnect_interval must be a number > 0, '
                             f'got {reconnect_interval!r}')

        self._processor = PacketProcessor(self._load_calibration(calibration_file))
        self._parser = FrameParser()

        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self._temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self._pressure_pub = self.create_publisher(FluidPressure, 'imu/pressure', 10)
        self._tf_broadcaster = TransformBroadcaster(self) if publish_tf else None

        self._reader = SerialReader(
            opener=self._open_port,
            on_data=self._on_data,
            on_error=self.get_logger().error,
            on_connect=self._on_connect,
            reconnect_interval=float(reconnect_interval),
        )
        self.get_logger().info(
            f'Opening {self._port_url} @ {self._baud} baud '
            f'(frame {self._parent_frame} -> {self._frame_id}, publish_tf={publish_tf})')
        self._reader.start()

    @property
    def reader(self):
        """Return the SerialReader (connection state and error counters)."""
        return self._reader

    def _str_param(self, name, default):
        value = self.declare_parameter(name, default).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'parameter {name} must be a non-empty string, got {value!r}')
        return value

    def _load_calibration(self, path):
        if not path:
            self.get_logger().info('No calibration_file set; publishing uncalibrated data')
            return SoftwareCalibration()
        if not os.path.exists(path):
            self.get_logger().warning(
                f'Calibration file {path} not found; publishing uncalibrated data')
            return SoftwareCalibration()
        try:
            cal = load_calibration(path)
        except (ValueError, OSError) as exc:
            self.get_logger().error(
                f'Invalid calibration file {path}: {exc}; publishing UNCALIBRATED data')
            return SoftwareCalibration()
        self.get_logger().info(f'Software calibration from {path}: {cal}')
        return cal

    def _open_port(self):
        return serial.serial_for_url(self._port_url, baudrate=self._baud,
                                     timeout=SERIAL_TIMEOUT_S)

    def _on_connect(self):
        self._parser = FrameParser()
        self.get_logger().info(f'Serial port {self._port_url} opened')

    def _on_data(self, data):
        for packet in self._parser.feed(data):
            self._publish(self._processor.process(packet))

    def _publish(self, item):
        if item is None:
            return
        stamp = self.get_clock().now().to_msg()
        if isinstance(item, ImuSample):
            self._imu_pub.publish(build_imu_msg(item, stamp, self._frame_id))
            if self._tf_broadcaster is not None:
                self._tf_broadcaster.sendTransform(
                    build_transform(item, stamp, self._parent_frame, self._frame_id))
        elif isinstance(item, Accel):
            msg = Temperature()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.temperature = float(item.temperature)
            self._temp_pub.publish(msg)
        elif isinstance(item, Mag):
            # The CMP10A reports raw magnetometer LSB (unscaled), not Tesla.
            msg = MagneticField()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.magnetic_field.x = float(item.x)
            msg.magnetic_field.y = float(item.y)
            msg.magnetic_field.z = float(item.z)
            self._mag_pub.publish(msg)
        elif isinstance(item, Pressure):
            msg = FluidPressure()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.fluid_pressure = float(item.pressure_pa)
            self._pressure_pub.publish(msg)

    def destroy_node(self):
        """Stop the serial thread before tearing down publishers."""
        self._reader.stop()
        return super().destroy_node()


def main(args=None):
    """Entry point for the ``wit_ros2_imu`` executable."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = WitImuNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, ParameterException) as exc:
        get_logger('wit_ros2_imu').fatal(f'Invalid configuration: {exc}')
        exit_code = 1
    finally:
        # Ctrl-C in a terminal reaches this process twice (directly and forwarded
        # by ros2 launch); a second SIGINT must not abort the cleanup below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
