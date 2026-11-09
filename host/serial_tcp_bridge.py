#!/usr/bin/env python3
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

"""Bidirectional serial <-> TCP bridge for the Yahboom CMP10A IMU.

Docker Desktop for macOS cannot pass USB serial devices into containers,
so ``imu.py`` runs this bridge on the host.  The container then opens
``socket://host.docker.internal:5760`` through pyserial.

* Listens on 127.0.0.1 only and serves one client at a time; when the
  client disconnects the bridge waits for the next one.
* Serial errors are logged, the port is closed and reopened after
  ``--retry-interval`` seconds; the current client is dropped so the
  driver in the container reconnects cleanly.
* Ctrl+C or SIGTERM stops the bridge cleanly.

Requires ``pyserial==3.5`` (``python3 -m pip install pyserial==3.5``).
"""

import argparse
import logging
import select
import signal
import socket
import sys
import threading

try:
    import serial
except ImportError:  # reported by main() with an installation hint
    serial = None

HOST = '127.0.0.1'
DEFAULT_PORT = 5760
DEFAULT_BAUD = 9600
DEFAULT_RETRY_INTERVAL = 2.0
ACCEPT_TIMEOUT = 0.2
SERIAL_READ_TIMEOUT = 0.05
SERIAL_WRITE_TIMEOUT = 1.0
CLIENT_TIMEOUT = 5.0
BUFFER_SIZE = 4096
MAX_BAUD = 4000000
MAX_RETRY_INTERVAL = 60.0
PYSERIAL_HINT = 'python3 -m pip install pyserial==3.5'

LOG = logging.getLogger('serial_tcp_bridge')


class SerialTcpBridge:
    """Forward bytes between one serial port and one TCP client."""

    def __init__(self, serial_url, baudrate=DEFAULT_BAUD, port=DEFAULT_PORT,
                 retry_interval=DEFAULT_RETRY_INTERVAL, serial_factory=None,
                 client_timeout=CLIENT_TIMEOUT):
        """Configure the bridge; call start_listening() to bind."""
        self.serial_url = serial_url
        self.baudrate = baudrate
        self.port = port
        self.retry_interval = retry_interval
        self.client_timeout = client_timeout
        self._factory = serial_factory
        self._stop_event = threading.Event()
        self._server = None
        self._serial = None
        self.client_connected = False
        self.stopped = False

    @property
    def address(self):
        """Return the (host, port) the bridge listens on."""
        if self._server is None:
            raise RuntimeError('bridge is not listening')
        return self._server.getsockname()[:2]

    def start_listening(self):
        """Bind the TCP server socket on 127.0.0.1; raise OSError on error."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((HOST, self.port))
            sock.listen(1)
            sock.settimeout(ACCEPT_TIMEOUT)
        except OSError:
            sock.close()
            raise
        self._server = sock
        LOG.info('listening on %s:%d', *self.address)
        return self.address

    def stop(self):
        """Ask serve_forever() to return (thread and signal safe)."""
        self._stop_event.set()

    def close(self):
        """Release the serial port and the server socket."""
        self._close_serial()
        if self._server is not None:
            self._server.close()
            self._server = None

    def serve_forever(self):
        """Run until stop() is called."""
        if self._server is None:
            self.start_listening()
        try:
            while not self._stop_event.is_set():
                ser = self._ensure_serial()
                if ser is None:
                    continue
                client = self._accept()
                if client is None:
                    continue
                self._serve_client(client, ser)
        finally:
            self.stopped = True

    # ---------------------------------------------------------- internals
    def _ensure_serial(self):
        if self._serial is not None:
            return self._serial
        factory = self._factory or serial.serial_for_url
        try:
            self._serial = factory(self.serial_url, baudrate=self.baudrate,
                                   timeout=SERIAL_READ_TIMEOUT,
                                   write_timeout=SERIAL_WRITE_TIMEOUT)
        except (OSError, ValueError) as exc:
            # serial.SerialException is a subclass of OSError (IOError).
            LOG.error('cannot open serial port %s: %s (retry in %.1f s)',
                      self.serial_url, exc, self.retry_interval)
            self._stop_event.wait(self.retry_interval)
            return None
        LOG.info('serial port %s opened at %d baud', self.serial_url,
                 self.baudrate)
        return self._serial

    def _close_serial(self):
        if self._serial is None:
            return
        try:
            self._serial.close()
        except OSError as exc:
            LOG.warning('error while closing serial port: %s', exc)
        self._serial = None

    def _serial_failed(self, exc):
        LOG.error('serial port %s failed: %s (reopening in %.1f s)',
                  self.serial_url, exc, self.retry_interval)
        self._close_serial()
        self._stop_event.wait(self.retry_interval)

    def _accept(self):
        try:
            client, peer = self._server.accept()
        except socket.timeout:
            return None
        except OSError as exc:
            if self._stop_event.is_set():
                return None
            LOG.error('accept failed: %s', exc)
            self._stop_event.wait(self.retry_interval)
            return None
        client.settimeout(self.client_timeout)
        LOG.info('client %s:%d connected', peer[0], peer[1])
        return client

    def _serve_client(self, client, ser):
        self.client_connected = True
        try:
            try:
                ser.reset_input_buffer()
            except (OSError, ValueError) as exc:
                self._serial_failed(exc)
                return
            while not self._stop_event.is_set():
                if not self._pump_once(client, ser):
                    return
        finally:
            client.close()
            self.client_connected = False

    def _pump_once(self, client, ser):
        """Move one chunk in each direction; return False to drop client."""
        try:
            data = ser.read(ser.in_waiting or 1)
        except (OSError, ValueError) as exc:
            self._serial_failed(exc)
            return False
        if data:
            try:
                client.sendall(data)
            except OSError as exc:
                LOG.warning('client send failed: %s', exc)
                return False
        try:
            readable, _, _ = select.select([client], [], [], 0)
            chunk = client.recv(BUFFER_SIZE) if readable else None
        except OSError as exc:
            LOG.warning('client receive failed: %s', exc)
            return False
        if chunk is None:
            return True
        if not chunk:
            LOG.info('client disconnected; waiting for the next one')
            return False
        try:
            ser.write(chunk)
        except (OSError, ValueError) as exc:
            self._serial_failed(exc)
            return False
        return True


# -------------------------------------------------------------- command line
def _tcp_port(text):
    value = _int(text)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError('port must be 1..65535')
    return value


def _baud(text):
    value = _int(text)
    if not 1 <= value <= MAX_BAUD:
        raise argparse.ArgumentTypeError('baud must be 1..%d' % MAX_BAUD)
    return value


def _int(text):
    try:
        return int(text)
    except ValueError:
        raise argparse.ArgumentTypeError('not an integer: %r' % text)


def _retry_interval(text):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError('not a number: %r' % text)
    if not 0 < value <= MAX_RETRY_INTERVAL:
        raise argparse.ArgumentTypeError(
            'retry interval must be in (0, %g] seconds' % MAX_RETRY_INTERVAL)
    return value


def _serial_name(text):
    if not text.strip():
        raise argparse.ArgumentTypeError('serial port must not be empty')
    return text


def parse_args(argv=None):
    """Parse and validate the command line."""
    parser = argparse.ArgumentParser(
        description='Bridge a serial port to TCP on 127.0.0.1 '
                    '(one client at a time).')
    parser.add_argument('--serial', required=True, type=_serial_name,
                        help='serial device or pyserial URL, '
                             'e.g. /dev/cu.usbserial-0001')
    parser.add_argument('--baud', type=_baud, default=DEFAULT_BAUD,
                        help='baud rate (default %(default)s)')
    parser.add_argument('--port', type=_tcp_port, default=DEFAULT_PORT,
                        help='TCP port on 127.0.0.1 (default %(default)s)')
    parser.add_argument('--retry-interval', type=_retry_interval,
                        default=DEFAULT_RETRY_INTERVAL,
                        help='seconds between serial reconnect attempts '
                             '(default %(default)s)')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the bridge until Ctrl+C / SIGTERM; return the exit code."""
    args = parse_args(argv)
    if serial is None:
        print('pyserial is not installed; run: %s' % PYSERIAL_HINT,
              file=sys.stderr)
        return 1
    logging.basicConfig(level=logging.INFO, stream=sys.stderr,
                        format='%(asctime)s [bridge] %(levelname)s '
                               '%(message)s')
    bridge = SerialTcpBridge(args.serial, baudrate=args.baud, port=args.port,
                             retry_interval=args.retry_interval)
    try:
        bridge.start_listening()
    except OSError as exc:
        LOG.error('cannot listen on %s:%d: %s (port already in use?)',
                  HOST, args.port, exc)
        return 1

    previous = signal.signal(signal.SIGTERM, lambda signum, frame:
                             bridge.stop())
    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        LOG.info('Ctrl+C received, stopping')
    finally:
        signal.signal(signal.SIGTERM, previous)
        bridge.close()
    LOG.info('bridge stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
