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

"""Tests for host/serial_tcp_bridge.py (macOS serial to TCP bridge)."""

import logging
import socket
import threading
import time

import pytest

import serial

import serial_tcp_bridge as bridge_mod

FRAME = bytes([0x55, 0x51, 1, 2, 3, 4, 5, 6, 7, 8, 0x00])
TIMEOUT = 5.0


class FakeSerial:
    """Minimal pyserial-like object with scripted reads."""

    def __init__(self, fail_read_after=None):
        """Create an empty port that fails after N reads if requested."""
        self.incoming = bytearray()
        self.written = bytearray()
        self.lock = threading.Lock()
        self.closed = False
        self.reads = 0
        self.fail_read_after = fail_read_after

    @property
    def in_waiting(self):
        """Return the number of buffered bytes."""
        with self.lock:
            return len(self.incoming)

    def feed(self, data):
        """Queue bytes as if they came from the IMU."""
        with self.lock:
            self.incoming.extend(data)

    def read(self, size=1):
        """Return queued bytes, waiting briefly like a real port."""
        self.reads += 1
        if (self.fail_read_after is not None
                and self.reads > self.fail_read_after):
            raise serial.SerialException('device disconnected')
        with self.lock:
            data = bytes(self.incoming[:size])
            del self.incoming[:size]
        if not data:
            time.sleep(0.01)
        return data

    def write(self, data):
        """Record written bytes."""
        self.written.extend(data)
        return len(data)

    def reset_input_buffer(self):
        """Drop queued input."""
        with self.lock:
            self.incoming.clear()

    def close(self):
        """Mark the port closed."""
        self.closed = True


class Factory:
    """serial_for_url replacement returning scripted results."""

    def __init__(self, results):
        """Store the results returned by successive calls."""
        self.results = list(results)
        self.calls = []

    def __call__(self, url, **kwargs):
        """Return the next scripted serial object or raise."""
        self.calls.append((url, kwargs))
        result = self.results.pop(0) if len(self.results) > 1 \
            else self.results[0]
        if isinstance(result, Exception):
            raise result
        return result


@pytest.fixture
def run_bridge():
    """Start a bridge in a background thread and stop it afterwards."""
    started = []

    def start(**kwargs):
        kwargs.setdefault('port', 0)
        kwargs.setdefault('retry_interval', 0.05)
        bridge = bridge_mod.SerialTcpBridge(**kwargs)
        bridge.start_listening()
        thread = threading.Thread(target=bridge.serve_forever, daemon=True)
        thread.start()
        started.append((bridge, thread))
        return bridge

    yield start
    for bridge, thread in started:
        bridge.stop()
        thread.join(TIMEOUT)
        assert not thread.is_alive(), 'bridge did not stop'
        bridge.close()


def connect(bridge):
    """Open a client connection to the bridge."""
    sock = socket.create_connection(bridge.address, timeout=TIMEOUT)
    sock.settimeout(TIMEOUT)
    return sock


def recv_exactly(sock, size):
    """Read ``size`` bytes from ``sock``."""
    data = b''
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            break
        data += chunk
    return data


def free_port():
    """Return a currently unused TCP port on 127.0.0.1."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_for(predicate, timeout=TIMEOUT):
    """Poll ``predicate`` until it is true or the timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_binds_loopback_only(run_bridge):
    """Listen on 127.0.0.1 only."""
    bridge = run_bridge(serial_url='loop://')
    assert bridge.address[0] == '127.0.0.1'


def test_loop_url_round_trip(run_bridge):
    """Forward client bytes to serial and serial bytes back (loop://)."""
    bridge = run_bridge(serial_url='loop://')
    with connect(bridge) as sock:
        sock.sendall(FRAME)
        assert recv_exactly(sock, len(FRAME)) == FRAME


def test_bidirectional_with_fake_serial(run_bridge):
    """Check each direction independently."""
    fake = FakeSerial()
    bridge = run_bridge(serial_url='/dev/fake',
                        serial_factory=Factory([fake]))
    with connect(bridge) as sock:
        assert wait_for(lambda: bridge.client_connected)
        fake.feed(FRAME)
        assert recv_exactly(sock, len(FRAME)) == FRAME
        sock.sendall(b'\xff\xaa\x27\x34\x00')
        assert wait_for(lambda: bytes(fake.written) == b'\xff\xaa\x27\x34'
                        b'\x00')


def test_serial_opened_with_timeouts(run_bridge):
    """Open the serial port with read and write timeouts."""
    fake = FakeSerial()
    factory = Factory([fake])
    run_bridge(serial_url='/dev/fake', baudrate=115200,
               serial_factory=factory)
    assert wait_for(lambda: factory.calls)
    url, kwargs = factory.calls[0]
    assert url == '/dev/fake'
    assert kwargs['baudrate'] == 115200
    assert kwargs['timeout'] > 0
    assert kwargs['write_timeout'] > 0


def test_client_reconnect(run_bridge):
    """Serve the next client after the first one disconnects."""
    bridge = run_bridge(serial_url='loop://')
    with connect(bridge) as sock:
        sock.sendall(b'first')
        assert recv_exactly(sock, 5) == b'first'
    assert wait_for(lambda: not bridge.client_connected)
    with connect(bridge) as sock:
        sock.sendall(b'second')
        assert recv_exactly(sock, 6) == b'second'


def test_retries_when_serial_open_fails(run_bridge, caplog):
    """Log and retry when the serial device is missing."""
    fake = FakeSerial()
    factory = Factory([serial.SerialException('no such device'),
                       serial.SerialException('no such device'), fake])
    caplog.set_level(logging.WARNING)
    bridge = run_bridge(serial_url='/dev/fake', serial_factory=factory)
    with connect(bridge) as sock:
        assert wait_for(lambda: len(factory.calls) >= 3)
        assert wait_for(lambda: bridge.client_connected)
        fake.feed(b'ok')
        assert recv_exactly(sock, 2) == b'ok'
    assert 'no such device' in caplog.text


def test_serial_disconnect_drops_client_and_reopens(run_bridge, caplog):
    """Close the client and reopen the port after a serial error."""
    broken = FakeSerial(fail_read_after=3)
    healthy = FakeSerial()
    factory = Factory([broken, healthy])
    caplog.set_level(logging.WARNING)
    bridge = run_bridge(serial_url='/dev/fake', serial_factory=factory)
    with connect(bridge) as sock:
        assert sock.recv(16) == b''
    assert broken.closed
    assert wait_for(lambda: len(factory.calls) >= 2)
    assert 'device disconnected' in caplog.text
    with connect(bridge) as sock:
        assert wait_for(lambda: bridge.client_connected)
        healthy.feed(b'back')
        assert recv_exactly(sock, 4) == b'back'


def test_stop_returns_promptly(run_bridge):
    """Stop serve_forever quickly even while waiting for clients."""
    bridge = run_bridge(serial_url='loop://')
    started = time.monotonic()
    bridge.stop()
    assert wait_for(lambda: bridge.stopped, timeout=2.0)
    assert time.monotonic() - started < 2.0


@pytest.mark.parametrize('argv', [
    ['--serial', 'loop://', '--port', '70000'],
    ['--serial', 'loop://', '--port', '0'],
    ['--serial', 'loop://', '--baud', '0'],
    ['--serial', 'loop://', '--baud', 'fast'],
    ['--serial', 'loop://', '--retry-interval', '-1'],
    ['--serial', ''],
    [],
])
def test_parse_args_rejects_invalid(argv):
    """Validate ports, baud rate and the serial argument."""
    with pytest.raises(SystemExit):
        bridge_mod.parse_args(argv)


def test_parse_args_defaults():
    """Use 9600 baud and TCP port 5760 by default."""
    args = bridge_mod.parse_args(['--serial', '/dev/cu.usbserial-0001'])
    assert args.serial == '/dev/cu.usbserial-0001'
    assert args.baud == 9600
    assert args.port == 5760


def test_main_handles_ctrl_c(monkeypatch):
    """Exit with 0 and release the socket on Ctrl+C."""
    closed = []

    def interrupted(self):
        raise KeyboardInterrupt

    monkeypatch.setattr(bridge_mod.SerialTcpBridge, 'serve_forever',
                        interrupted)
    monkeypatch.setattr(bridge_mod.SerialTcpBridge, 'close',
                        lambda self: closed.append(True))
    port = free_port()
    assert bridge_mod.main(['--serial', 'loop://', '--port', str(port)]) == 0
    assert closed == [True]


def test_main_port_in_use_returns_error(caplog):
    """Fail with a non-zero exit code if the TCP port is taken."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as busy:
        busy.bind(('127.0.0.1', 0))
        busy.listen(1)
        port = busy.getsockname()[1]
        caplog.set_level(logging.ERROR)
        code = bridge_mod.main(['--serial', 'loop://', '--port', str(port)])
    assert code == 1
    assert str(port) in caplog.text
