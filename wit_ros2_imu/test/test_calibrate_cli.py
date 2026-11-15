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

"""Tests for the interactive calibration CLI using a fake serial port."""

import math
import struct

import pytest
import serial
from wit_ros2_imu import calibrate_cli as cli
from wit_ros2_imu import calibration as cal
from wit_ros2_imu import calibration_check as cc
from wit_ros2_imu import wit_protocol as wp

UNLOCK = wp.cmd_unlock()
SAVE = wp.cmd_save()
NORMAL = wp.cmd_calsw(wp.CalMode.NORMAL)


def make_frame(ptype, payload):
    """Build a valid 11-byte frame."""
    body = bytes([0x55, ptype]) + bytes(payload)
    return body + bytes([sum(body) & 0xFF])


def i16(a, b, c, d):
    """Pack four int16 little-endian values."""
    return struct.pack('<hhhh', a, b, c, d)


class FakeClock:
    """Deterministic replacement for time.monotonic / time.sleep."""

    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.now += seconds


class FakeSerial:
    """Minimal pyserial stand-in that records writes."""

    def __init__(self, clock, stream=b'', responses=None, chunk=23,
                 interrupt_after=None, fail_write=False):
        self.clock = clock
        self.stream = bytes(stream)
        self.responses = responses or {}
        self.chunk = chunk
        self.interrupt_after = interrupt_after
        self.fail_write = fail_write
        self.writes = []
        self.write_times = []
        self.reads = 0
        self.closed = False
        self._rx = bytearray()

    def write(self, data):
        if self.fail_write:
            raise serial.SerialException('device disconnected')
        data = bytes(data)
        self.writes.append(data)
        self.write_times.append(self.clock.now)
        self._rx += self.responses.get(data, b'')
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._rx.clear()

    def read(self, size=1):
        self.reads += 1
        if self.interrupt_after is not None and self.reads > self.interrupt_after:
            raise KeyboardInterrupt
        if not self._rx and self.stream:
            self._rx += self.stream
        if not self._rx:
            self.clock.sleep(0.5)       # emulate the 0.5 s read timeout
            return b''
        self.clock.sleep(0.02)
        n = min(size, self.chunk, len(self._rx))
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def close(self):
        self.closed = True


def scripted(answers, clock=None, advance=0.0):
    """Return an input() replacement that replays answers then raises EOFError."""
    it = iter(answers)
    prompts = []

    def _input(prompt=''):
        prompts.append(prompt)
        if clock is not None:
            clock.sleep(advance)
        try:
            return next(it)
        except StopIteration:
            raise EOFError from None
    _input.prompts = prompts
    return _input


class Output:
    """Collect output_fn calls."""

    def __init__(self):
        self.lines = []

    def __call__(self, *args, **kwargs):
        self.lines.append(' '.join(str(a) for a in args))

    @property
    def text(self):
        return '\n'.join(self.lines)


def run(ser, answers, clock, tmp_path, advance=0.0):
    out = Output()
    inp = scripted(answers, clock, advance)
    rc = cli.run_menu(ser, inp, out, tmp_path / 'imu_calibration.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    return rc, out, inp


@pytest.fixture
def clock():
    return FakeClock()


# ---------------------------------------------------------------- menu basics

def test_exit_immediately(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    assert '11' in out.text   # menu was shown


def test_eof_on_input_exits_cleanly(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, [], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []


@pytest.mark.parametrize('choice', ['abc', '14', '-1', '', '1 2'])
def test_invalid_choice_reports_and_continues(clock, tmp_path, choice):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, [choice, '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    assert '無效' in out.text


# ---------------------------------------------------------------- hardware calibration

def test_mag_calibration_long_rotation_reunlocks_before_normal(clock, tmp_path):
    # Unlock only lasts ~10 s; after a 60 s rotation the CLI re-unlocks first.
    ser = FakeSerial(clock, stream=_sphere_stream())
    rc, _, _ = run(ser, ['3', 'y', '', 'y', '0'], clock, tmp_path, advance=60.0)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.MAG_SPHERE),
                          UNLOCK, NORMAL, UNLOCK, SAVE]


def test_hardware_write_failure_is_reported(clock, tmp_path):
    ser = FakeSerial(clock, fail_write=True)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert 'device disconnected' in out.text
    assert '失敗' in out.text


def test_ctrl_c_during_mag_calibration_restores_normal_without_save(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream())
    out = Output()
    answers = iter(['3', 'y'])

    def inp(prompt=''):
        try:
            return next(answers)
        except StopIteration:
            raise KeyboardInterrupt from None

    calls = {'n': 0}

    def inp_then_exit(prompt=''):
        calls['n'] += 1
        if calls['n'] <= 3:
            return inp(prompt)
        return '0'

    rc = cli.run_menu(ser, inp_then_exit, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    assert rc == 0
    assert SAVE not in ser.writes
    assert ser.writes[-1] == NORMAL
    assert '中止' in out.text


# ---------------------------------------------------------------- factory reset

def test_factory_reset_requires_double_confirmation(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, ['11', 'y', 'n', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []

    ser = FakeSerial(clock)
    run(ser, ['11', 'n', '0'], clock, tmp_path)
    assert ser.writes == []


def test_factory_reset_confirmed(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, ['11', 'y', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_factory_reset()]


# ---------------------------------------------------------------- read offsets

def test_read_offset_registers(clock, tmp_path):
    responses = {
        wp.cmd_read(0x05): make_frame(0x5F, i16(1, 2, 3, 4)),
        wp.cmd_read(0x09): make_frame(0x5F, i16(5, 6, 7, 8)),
        wp.cmd_read(0x0D): make_frame(0x5F, i16(-9, 10, 11, 12)),
    }
    ser = FakeSerial(clock, responses=responses)
    rc, out, _ = run(ser, ['8', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [wp.cmd_read(0x05), wp.cmd_read(0x09), wp.cmd_read(0x0D)]
    text = out.text
    for name, value in [('AXOFFSET', 1), ('AYOFFSET', 2), ('AZOFFSET', 3),
                        ('GXOFFSET', 4), ('GYOFFSET', 5), ('GZOFFSET', 6),
                        ('HXOFFSET', 7), ('HYOFFSET', 8), ('HZOFFSET', -9)]:
        assert f'{name}' in text
        assert any(name in line and str(value) in line for line in out.lines)
    assert UNLOCK not in ser.writes and SAVE not in ser.writes


def test_read_offset_registers_timeout(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['8', '0'], clock, tmp_path)
    assert rc == 0
    assert '逾時' in out.text


# ---------------------------------------------------------------- live view

def test_live_view_shows_values_and_ctrl_c_returns_to_menu(clock, tmp_path):
    stream = (make_frame(0x51, i16(0, 0, 2048, 2500))
              + make_frame(0x52, i16(0, 0, 0, 0))
              + make_frame(0x53, i16(8192, -8192, 16384, 0))
              + make_frame(0x54, i16(111, -222, 333, 2500)))
    ser = FakeSerial(clock, stream=stream, interrupt_after=10)
    rc, out, _ = run(ser, ['1', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    text = out.text
    assert 'roll' in text and 'pitch' in text and 'yaw' in text
    assert '45.00' in text and '-45.00' in text and '90.00' in text
    assert '9.81' in text
    assert '-222' in text
    assert 'Ctrl+C' in text


def test_live_view_serial_error_returns_to_menu(clock, tmp_path):
    class Broken(FakeSerial):

        def read(self, size=1):
            raise serial.SerialException('read failed')

    ser = Broken(clock)
    rc, out, _ = run(ser, ['1', '0'], clock, tmp_path)
    assert rc == 0
    assert 'read failed' in out.text


# ---------------------------------------------------------------- main()

def test_main_open_failure_returns_nonzero(monkeypatch, capsys):
    def boom(*args, **kwargs):
        raise serial.SerialException('could not open port /dev/ttyFAKE')

    monkeypatch.setattr(cli.serial, 'serial_for_url', boom)
    rc = cli.main(['--port', '/dev/ttyFAKE'])
    assert rc != 0
    err = capsys.readouterr().err
    assert '/dev/ttyFAKE' in err


def test_main_opens_port_with_timeout_and_runs_menu(monkeypatch, tmp_path):
    opened = {}
    fake = FakeSerial(FakeClock())

    def fake_open(url, **kwargs):
        opened['url'] = url
        opened.update(kwargs)
        return fake

    captured = {}

    def fake_menu(ser, input_fn, output_fn, config_path, **kwargs):
        captured['ser'] = ser
        captured['config'] = config_path
        return 0

    monkeypatch.setattr(cli.serial, 'serial_for_url', fake_open)
    monkeypatch.setattr(cli, 'run_menu', fake_menu)
    cfg = tmp_path / 'x.yaml'
    rc = cli.main(['--port', 'socket://host.docker.internal:5760',
                   '--baud', '115200', '--config', str(cfg)])
    assert rc == 0
    assert opened == {'url': 'socket://host.docker.internal:5760',
                      'baudrate': 115200, 'timeout': 0.5}
    assert captured['ser'] is fake
    assert str(captured['config']) == str(cfg)
    assert fake.closed


def test_main_defaults(monkeypatch):
    opened = {}

    def fake_open(url, **kwargs):
        opened['url'] = url
        opened.update(kwargs)
        return FakeSerial(FakeClock())

    monkeypatch.setattr(cli.serial, 'serial_for_url', fake_open)
    monkeypatch.setattr(cli, 'run_menu', lambda *a, **k: 0)
    assert cli.main([]) == 0
    assert opened == {'url': '/dev/ttyUSB0', 'baudrate': 9600, 'timeout': 0.5}


@pytest.mark.parametrize('baud', ['0', '-9600', 'fast'])
def test_main_rejects_bad_baud(baud):
    with pytest.raises(SystemExit) as exc:
        cli.main(['--baud', baud])
    assert exc.value.code != 0


# ---------------------------------------------------------------- placement notes

MENU_HINTS = {
    '2': '水平桌面',
    '3': '50 cm',
    '4': '先水平、再垂直',
    '5': '靜止放置',
    '6': '水平地面',
    '7': '0 m',
    '9': '任何地方',
    '10': '寫入 IMU 設定',
    '12': '最後一步',
}


def _menu_line(choice):
    prefix = f'{int(choice):2d}. '
    lines = [ln for ln in cli.MENU.splitlines() if ln.startswith(prefix)]
    assert len(lines) == 1, f'menu line for {choice!r} not found'
    return lines[0]


@pytest.mark.parametrize('choice,hint', sorted(MENU_HINTS.items()))
def test_menu_line_shows_placement_hint(choice, hint):
    assert hint in _menu_line(choice)


def test_menu_lines_fit_flake8_width():
    assert all(len(ln) <= 99 for ln in cli.MENU.splitlines())


def _logged_run(ser, answers, clock, tmp_path):
    """Run the menu with input prompts and output interleaved in one log."""
    out = Output()
    inner = scripted(answers, clock)

    def _input(prompt=''):
        out.lines.append(f'<PROMPT>{prompt}')
        return inner(prompt)
    rc = cli.run_menu(ser, _input, out, tmp_path / 'imu_calibration.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    return rc, out


@pytest.mark.parametrize('choice', ['2', '3', '4', '5', '6', '7'])
def test_hardware_notes_shown_before_confirmation(clock, tmp_path, choice):
    ser = FakeSerial(clock)
    rc, out = _logged_run(ser, [choice, 'n', '0'], clock, tmp_path)
    assert rc == 0
    assert all(_is_read(w) for w in ser.writes)          # Z zero reads the algorithm first
    notes_idx = out.lines.index('注意事項:')
    confirm_idx = next(i for i, ln in enumerate(out.lines)
                       if ln.startswith('<PROMPT>') and 'y/N' in ln)
    assert notes_idx < confirm_idx
    for note in cli.NOTES[choice]:
        assert note in out.lines[notes_idx:confirm_idx]


@pytest.mark.parametrize('choice', ['1', '8', '11'])
def test_no_placement_notes_for_non_calibration_items(clock, tmp_path, choice):
    ser = FakeSerial(clock, interrupt_after=3)    # stops the live view loop
    rc, out = _logged_run(ser, [choice, 'n', '0'], clock, tmp_path)
    assert '注意事項:' not in out.lines


def test_menu_hints_are_aligned_in_terminal_columns():
    import unicodedata

    def width(s):
        return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)
    widths = {width(ln.rsplit(' → ', 1)[0]) for ln in cli.MENU.splitlines() if ' → ' in ln}
    assert len(widths) == 1, widths


# ---------------------------------------------------------------- progress display

def test_progress_bar_formatting():
    assert cli.progress_bar(0.0, width=10) == '[----------]   0%'
    assert cli.progress_bar(0.5, width=10) == '[#####-----]  50%'
    assert cli.progress_bar(1.0, width=10) == '[##########] 100%'


@pytest.mark.parametrize('value,expected', [(1.7, 1.0), (-1.0, 0.0), (float('nan'), 0.0)])
def test_progress_bar_clamps_out_of_range(value, expected):
    assert cli.progress_bar(value, width=10) == cli.progress_bar(expected, width=10)


def test_accel_calibration_shows_countdown_progress(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert '剩' in out.text
    assert '100%' in out.text


def _gyro_frame(x_dps, y_dps, z_dps):
    raw = [int(v / 2000.0 * 32768) for v in (x_dps, y_dps, z_dps)]
    return make_frame(0x52, i16(raw[0], raw[1], raw[2], 0))


def _mag_frame(x, y, z):
    return make_frame(0x54, i16(int(x), int(y), int(z), 2500))


FROZEN_GYRO = _gyro_frame(-1.7, -2.6, 4.0)   # CMP10A freezes gyro output in mag-cal mode


def _sphere_stream(radius=1000, center=(60, -40, 20), planes=('XY', 'XZ', 'YZ')):
    """
    Return mag frames turning once in each plane, with the gyro frozen like the real IMU.

    A level accelerometer frame goes with every point, as the dual-plane
    calibration checks the tilt before it starts.
    """
    frames = b''
    for plane in planes:
        i, j = 'XYZ'.index(plane[0]), 'XYZ'.index(plane[1])
        for d in range(0, 360, 15):
            v = list(center)
            v[i] += radius * math.cos(math.radians(d))
            v[j] += radius * math.sin(math.radians(d))
            frames += _mag_frame(*v) + FROZEN_GYRO + _level_accel()
    return frames


def _level_accel(tilt_deg=0.0):
    t = math.radians(tilt_deg)
    raw = [int(round(v / (16.0 * wp.GRAVITY) * 32768))
           for v in (wp.GRAVITY * math.sin(t), 0.0, wp.GRAVITY * math.cos(t))]
    return make_frame(0x51, i16(raw[0], raw[1], raw[2], 2500))


def _wait_for(out, text, timeout=5.0):
    """Block (real time) until ``text`` appears in the output, like a user watching."""
    import time as _time
    deadline = _time.monotonic() + timeout
    while text not in out.text:
        if _time.monotonic() > deadline:
            raise AssertionError(f'{text!r} never shown; output:\n{out.text[-2000:]}')
        _time.sleep(0.01)


def _run_mag_waiting_for(ser, clock, tmp_path, choice, wait_text, partial_answer='0'):
    out = Output()
    answers = [choice, 'y', None, partial_answer, '0']
    calls = {'n': 0}

    def inp(prompt=''):
        answer = answers[calls['n']]
        calls['n'] += 1
        if answer is None:
            _wait_for(out, wait_text)
            return ''
        return answer
    rc = cli.run_menu(ser, inp, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    return rc, out


def _display_width(text):
    import unicodedata
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text)


def _status_lines(out):
    return [ln.replace(cli.CLEAR_EOL, '').lstrip('\r') for ln in out.lines if ln.startswith('\r')]


def test_mag_calibration_without_data_does_not_enter_calibration_or_save(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['3', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert '未收到' in out.text
    assert ser.writes == []


def test_mag_read_failure_restores_normal_without_save(clock, tmp_path):
    class BrokenDuringCalibration(FakeSerial):

        def read(self, size=1):
            if wp.cmd_calsw(wp.CalMode.MAG_SPHERE) in self.writes:
                raise serial.SerialException('reader disconnected')
            return super().read(size)

    ser = BrokenDuringCalibration(clock, stream=_sphere_stream())
    rc, out, _ = run(ser, ['3', 'y', '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes[-1] == NORMAL
    assert SAVE not in ser.writes
    assert 'reader disconnected' in out.text


def test_mag_data_stops_after_start_restores_normal_without_save(clock, tmp_path):
    class StopsDuringCalibration(FakeSerial):

        def read(self, size=1):
            if wp.cmd_calsw(wp.CalMode.MAG_SPHERE) in self.writes:
                self.stream = b''
                self._rx.clear()
            return super().read(size)

    ser = StopsDuringCalibration(clock, stream=_sphere_stream())
    rc, out, _ = run(ser, ['3', 'y', '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes[-1] == NORMAL
    assert SAVE not in ser.writes
    assert '未收到' in out.text


@pytest.mark.parametrize('choice,fields', [
    ('2', ('accel_bias',)),
    ('3', ('mag_offset',)),
    ('4', ('mag_offset',)),
    ('11', cal.FIELDS),
])
def test_hardware_change_clears_only_obsolete_software_offsets(clock, tmp_path, choice, fields):
    """A new hardware baseline must not receive the previous residual correction."""
    path = tmp_path / 'c.yaml'
    previous = cal.SoftwareCalibration(gyro_bias=(0.01, 0.02, 0.03),
                                       accel_bias=(0.1, 0.2, 0.3), mag_offset=(1, 2, 3))
    cal.save_calibration(path, previous)

    class CheckOrder(FakeSerial):

        def write(self, data):
            if data in (SAVE, wp.cmd_factory_reset()):
                current = cal.load_calibration(path)
                assert all(getattr(current, field) == (0, 0, 0) for field in fields)
            return super().write(data)

    ser = CheckOrder(clock, stream=_sphere_stream())
    if choice in ('3', '4'):
        _run_mag_waiting_for(ser, clock, tmp_path, choice, '[完成] 按 Enter')
    else:
        answers = [choice, 'y', 'y', '0'] if choice == '11' else [choice, 'y', '0']
        cli.run_menu(ser, scripted(answers), Output(), path,
                     sleep_fn=clock.sleep, time_fn=clock.time)
    current = cal.load_calibration(path)
    for field in cal.FIELDS:
        expected = (0, 0, 0) if field in fields else getattr(previous, field)
        assert getattr(current, field) == expected


def test_hardware_calibration_does_not_overwrite_a_corrupt_yaml(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    path.write_text('accel_bias: [oops\n')
    before = path.read_bytes()
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    assert path.read_bytes() == before
    assert '格式錯誤' in out.text


@pytest.mark.parametrize('choice', ['2', '11'])
def test_offset_file_failure_prevents_persisting_hardware_change(
        clock, tmp_path, monkeypatch, choice):
    path = tmp_path / 'imu_calibration.yaml'
    cal.save_calibration(path, cal.SoftwareCalibration(accel_bias=(0.1, 0.2, 0.3)))
    before = path.read_bytes()

    def cannot_save(*args):
        raise PermissionError('read-only calibration directory')

    monkeypatch.setattr(cli, 'save_calibration', cannot_save)
    ser = FakeSerial(clock)
    answers = [choice, 'y', 'y', '0'] if choice == '11' else [choice, 'y', '0']
    rc, out, _ = run(ser, answers, clock, tmp_path)
    assert rc == 0
    assert SAVE not in ser.writes
    assert wp.cmd_factory_reset() not in ser.writes
    assert path.read_bytes() == before
    assert '未送出儲存或重設指令' in out.text and '重新上電' in out.text
    assert cli.DONE_MARK not in _menu_row(_menu_blocks(out)[-1], int(choice))


@pytest.mark.parametrize('choice,fields,command', [
    ('2', ('accel_bias',), SAVE),
    ('11', cal.FIELDS, wp.cmd_factory_reset()),
])
def test_hardware_persist_failure_keeps_software_offsets_zero(
        clock, tmp_path, choice, fields, command):
    """Failure after clearing YAML may lose a residual, never restore a stale one."""
    path = tmp_path / 'imu_calibration.yaml'
    previous = cal.SoftwareCalibration(gyro_bias=(0.01, 0.02, 0.03),
                                       accel_bias=(0.1, 0.2, 0.3), mag_offset=(1, 2, 3))
    cal.save_calibration(path, previous)

    class CannotPersist(FakeSerial):

        def write(self, data):
            if data == command:
                raise serial.SerialException('persist failed')
            return super().write(data)

    ser = CannotPersist(clock)
    answers = [choice, 'y', 'y', '0'] if choice == '11' else [choice, 'y', '0']
    rc, out, _ = run(ser, answers, clock, tmp_path)
    assert rc == 0
    current = cal.load_calibration(path)
    for field in cal.FIELDS:
        expected = (0, 0, 0) if field in fields else getattr(previous, field)
        assert getattr(current, field) == expected
    assert 'persist failed' in out.text and '重新上電' in out.text


def test_mag_calibration_serial_error_in_reader_is_reported(clock, tmp_path):
    class Broken(FakeSerial):

        def read(self, size=1):
            raise serial.SerialException('read failed')

    ser = Broken(clock)
    rc, out, _ = run(ser, ['3', 'y', '', '0'], clock, tmp_path)
    assert rc == 0
    assert 'read failed' in out.text


def test_progress_bar_never_shows_100_percent_before_done():
    assert cli.progress_bar(0.9997, width=10).endswith(' 99%')


# ---------------------------------------------------------------- attitude view (12)

def _angle_frame(roll_deg, pitch_deg, yaw_deg):
    raw = [int(round(v / 180.0 * 32768)) for v in (roll_deg, pitch_deg, yaw_deg)]
    return make_frame(0x53, i16(raw[0], raw[1], raw[2], 0))


def _pressure_frame(pa, height_cm):
    return make_frame(0x56, struct.pack('<ii', pa, height_cm))


@pytest.mark.parametrize('value,expected', [
    (0.0, 0.0), (190.0, -170.0), (-190.0, 170.0), (360.0, 0.0), (180.0, -180.0), (90.0, 90.0),
])
def test_wrap_degrees(value, expected):
    assert cli.wrap_degrees(value) == pytest.approx(expected)


def test_attitude_line_absolute_without_height():
    line = cli.format_attitude((10.0, -20.5, 45.25), None)
    assert 'roll(X)  +10.00°' in line
    assert 'pitch(Y)  -20.50°' in line
    assert 'yaw(Z)  +45.25°' in line
    assert '高度 (未輸出)' in line


def test_attitude_line_with_height():
    line = cli.format_attitude((0.0, 0.0, 0.0), 12.34)
    assert '高度  +12.34 m' in line


def test_attitude_line_relative_to_reference_wraps_yaw():
    ref = ((1.0, 2.0, 170.0), 10.0)
    line = cli.format_attitude((1.5, 1.0, -170.0), 11.25, ref)
    assert 'Δroll(X)   +0.50°' in line
    assert 'Δpitch(Y)   -1.00°' in line
    assert 'Δyaw(Z)  +20.00°' in line          # 170 -> -170 is +20, not -340
    assert 'Δ高度   +1.25 m' in line


@pytest.mark.parametrize('ref', [None, ((0.0, 0.0, 0.0), 0.0), ((0.0, 0.0, 0.0), None)])
def test_attitude_line_fits_80_columns(ref):
    line = cli.format_attitude((-179.99, -89.99, -179.99), -999.99, ref)
    assert _display_width(line) <= 79, line


def _attitude_stream(yaw=0.0, height_cm=1234):
    return (_angle_frame(1.0, -2.0, yaw) + _pressure_frame(101325, height_cm))


def _run_with_watcher(ser, clock, tmp_path, script):
    """script: list of answers; ('wait', text) entries block until text is shown."""
    out = Output()
    items = list(script)

    def inp(prompt=''):
        while items and isinstance(items[0], tuple):
            _wait_for(out, items.pop(0)[1])
        if not items:
            raise EOFError
        return items.pop(0)
    rc = cli.run_menu(ser, inp, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    return rc, out


def test_attitude_view_shows_live_angles_and_height_then_quits(clock, tmp_path):
    ser = FakeSerial(clock, stream=_attitude_stream())
    rc, out = _run_with_watcher(ser, clock, tmp_path,
                                ['12', 'f', ('wait', '高度  +12.34 m'), 'q', '0'])
    assert rc == 0
    assert ser.writes == []                    # read-only
    assert 'roll(X)   +1.00°' in out.text
    assert 'pitch(Y)   -2.00°' in out.text


def test_attitude_view_enter_sets_reference_and_r_clears_it(clock, tmp_path):
    ser = FakeSerial(clock, stream=_attitude_stream(yaw=30.0))
    rc, out = _run_with_watcher(ser, clock, tmp_path, [
        '12', 'f', ('wait', 'yaw(Z)  +30.00°'), '', ('wait', 'Δyaw(Z)   +0.00°'), 'r',
        'q', '0'])
    assert rc == 0
    assert '參考點已設定' in out.text
    assert '已清除參考點' in out.text
    assert ser.writes == []


def test_attitude_view_reference_needs_data(clock, tmp_path):
    ser = FakeSerial(clock)                      # no data at all
    rc, out = _run_with_watcher(ser, clock, tmp_path, ['12', 'f', '', 'q', '0'])
    assert rc == 0
    assert '尚未收到角度資料' in out.text


def test_option_12_offers_the_calibration_check(clock, tmp_path):
    """The entry prompt offers the full check, single checks and the free view."""
    ser = FakeSerial(clock)
    rc, out, inp = run(ser, ['12', 'q', '0'], clock, tmp_path)
    prompt = ' '.join(inp.prompts)
    assert '完整檢測' in prompt and '1 加速度計' in prompt and 'f = 姿態檢視' in prompt


def test_attitude_view_ctrl_c_returns_to_menu(clock, tmp_path):
    ser = FakeSerial(clock, stream=_attitude_stream())
    calls = {'n': 0}
    out = Output()

    def inp(prompt=''):
        calls['n'] += 1
        if calls['n'] == 1:
            return '12'
        if calls['n'] == 2:
            raise KeyboardInterrupt
        return '0'
    rc = cli.run_menu(ser, inp, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time)
    assert rc == 0
    assert ser.writes == []


def test_attitude_view_serial_error_is_reported(clock, tmp_path):
    class Broken(FakeSerial):

        def read(self, size=1):
            raise serial.SerialException('read failed')

    ser = Broken(clock)
    rc, out, _ = run(ser, ['12', 'f', 'q', '0'], clock, tmp_path)
    assert rc == 0
    assert 'read failed' in out.text


# ---------------------------------------------------------------- pressure output toggle (13)

def _rsw_reply(rsw):
    return make_frame(0x5F, i16(rsw, 0x06, 0x02, 0))


def test_enable_pressure_output_sets_rsw_bit_and_saves(clock, tmp_path):
    read_rsw = wp.cmd_read(0x02)
    ser = FakeSerial(clock, responses={read_rsw: _rsw_reply(0x001E)})
    rc, out, _ = run(ser, ['13', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [read_rsw, UNLOCK, wp.cmd_write(0x02, 0x005E), SAVE]
    assert '已開啟' in out.text


def test_disable_pressure_output_clears_rsw_bit(clock, tmp_path):
    read_rsw = wp.cmd_read(0x02)
    ser = FakeSerial(clock, responses={read_rsw: _rsw_reply(0x005E)})
    rc, out, _ = run(ser, ['13', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [read_rsw, UNLOCK, wp.cmd_write(0x02, 0x001E), SAVE]
    assert '已關閉' in out.text


@pytest.mark.parametrize('answer', ['n', '', 'whatever'])
def test_pressure_output_toggle_declined_only_reads(clock, tmp_path, answer):
    read_rsw = wp.cmd_read(0x02)
    ser = FakeSerial(clock, responses={read_rsw: _rsw_reply(0x001E)})
    rc, out, _ = run(ser, ['13', answer, '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [read_rsw]
    assert '未開啟' in out.text                  # current state is shown


def test_pressure_output_toggle_read_timeout_writes_nothing(clock, tmp_path):
    ser = FakeSerial(clock)                      # never answers the register read
    rc, out, _ = run(ser, ['13', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [wp.cmd_read(0x02)]
    assert '逾時' in out.text


def test_height_zero_notes_mention_pressure_output_disabled():
    text = '\n'.join(cli.NOTES['7'])
    assert '0x56' in text
    assert '/imu/pressure' in text


def test_height_zero_notes_point_to_pressure_toggle():
    assert any('13' in line for line in cli.NOTES['7'])


def test_use_color_only_on_a_tty_without_no_color():
    class Tty:

        def isatty(self):
            return True

    class Pipe:

        def isatty(self):
            return False

    assert cli.use_color(Tty(), {}) is True
    assert cli.use_color(Pipe(), {}) is False
    assert cli.use_color(Tty(), {'NO_COLOR': '1'}) is False


# --------------------------------------------- in-place updates on a terminal

def test_free_view_keeps_one_line_when_reference_is_set_on_a_terminal(clock, tmp_path):
    """On a terminal Enter must not leave a 'reference set' line behind."""
    ser = FakeSerial(clock, stream=_attitude_stream(yaw=30.0))
    out = Output()
    items = ['12', 'f', ('wait', 'yaw(Z)  +30.00°'), '', ('wait', 'Δyaw(Z)   +0.00°'), 'q', '0']

    def inp(prompt=''):
        while items and isinstance(items[0], tuple):
            _wait_for(out, items.pop(0)[1])
        if not items:
            raise EOFError
        return items.pop(0)
    rc = cli.run_menu(ser, inp, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    assert rc == 0
    assert '參考點已設定' not in out.text
    assert cli.CURSOR_UP_CLEAR in out.text          # the Enter newline is taken back


# ------------------------------------ menu marks steps done in this session

def _menu_blocks(out):
    """Return every printed menu, in order."""
    return [ln for ln in out.lines if 'CMP10A IMU 校正工具' in ln]


def _menu_row(block, number):
    """Return the raw menu row for ``number`` (colour codes are ignored when matching)."""
    green = cli.ANSI_COLOURS[cli.PASS]
    return next(ln for ln in block.splitlines()
                if ln.replace(green, '').strip().startswith(f'{number}.'))


def test_menu_starts_with_nothing_marked_done(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    first = _menu_blocks(out)[0]
    assert cli.ANSI_COLOURS[cli.PASS] not in first
    assert cli.DONE_MARK not in first


def test_menu_marks_a_successful_calibration_green_for_this_session(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['2', 'y', '0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    before, after = _menu_blocks(out)[0], _menu_blocks(out)[-1]
    assert cli.DONE_MARK not in _menu_row(before, 2)
    row = _menu_row(after, 2)
    assert row.startswith(cli.ANSI_COLOURS[cli.PASS]) and cli.DONE_MARK in row
    assert cli.DONE_MARK not in _menu_row(after, 3)          # other steps stay plain


def test_menu_does_not_mark_a_cancelled_calibration(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['2', 'n', '0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    assert cli.DONE_MARK not in _menu_blocks(out)[-1]


def test_menu_marks_are_not_kept_for_the_next_run(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['2', 'y', '0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    assert cli.DONE_MARK in _menu_blocks(out)[-1]
    out2 = Output()
    cli.run_menu(FakeSerial(clock), scripted(['0']), out2, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    assert cli.DONE_MARK not in out2.text


def test_factory_reset_clears_the_done_marks(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['2', 'y', '11', 'y', 'y', '0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time, color=True)
    assert cli.DONE_MARK not in _menu_blocks(out)[-1]


def test_menu_marks_done_without_colour_codes_off_a_terminal(clock, tmp_path):
    ser = FakeSerial(clock)
    out = Output()
    cli.run_menu(ser, scripted(['2', 'y', '0']), out, tmp_path / 'c.yaml',
                 sleep_fn=clock.sleep, time_fn=clock.time)
    last = _menu_blocks(out)[-1]
    assert cli.DONE_MARK in _menu_row(last, 2)
    assert '\033[' not in last


# ------------------------------------------- calibration check (option 12)

G0 = wp.GRAVITY


def _acc_frame(x, y, z):
    raw = [int(round(v / (16.0 * G0) * 32768)) for v in (x, y, z)]
    return make_frame(0x51, i16(raw[0], raw[1], raw[2], 2500))


STILL = _gyro_frame(0.0, 0.0, 0.0)


def _face_stream(axis, sign, bias=(0.0, 0.0, 0.0)):
    v = [0.0, 0.0, 0.0]
    v[cc.axis_index(axis)] = sign * G0
    return _acc_frame(*(a + b for a, b in zip(v, bias))) + STILL


def _mag_check_stream(dip_deg=36.0, radius=4000.0, offset=(0.0, 0.0, 0.0), n=150):
    """
    Return data for an IMU turned so that every face has been up and down.

    Gravity directions cover the whole sphere; each magnetometer reading is
    the field at ``dip_deg`` below the horizontal for that attitude.
    """
    dip = math.radians(dip_deg)
    out = b''
    for i in range(n):
        z = 1 - 2 * (i + 0.5) / n
        r = math.sqrt(1 - z * z)
        phi = i * math.pi * (3 - math.sqrt(5))
        down = (r * math.cos(phi), r * math.sin(phi), z)     # "down" in the body frame
        q = (-down[1], down[0], 0.0) if abs(down[2]) < 0.9 else (0.0, -down[2], down[1])
        qn = math.sqrt(sum(c * c for c in q))
        q = tuple(c / qn for c in q)                          # a horizontal direction
        u = tuple(math.sin(dip) * d + math.cos(dip) * h for d, h in zip(down, q))
        out += _acc_frame(*(-G0 * d for d in down)) + STILL
        out += _mag_frame(*(round(radius * c + o) for c, o in zip(u, offset)))
    return out


class _Check:
    """
    Run option 12 on a fake terminal.

    ``steps`` has one (stream, key, delay_s) entry per prompt of the check, in
    order.  The stream is what the IMU sends from then on; the key is typed
    ``delay_s`` simulated seconds later (None = never, the step must end by
    itself).
    """

    def __init__(self, clock, tmp_path, steps, entry='', max_reads=60000,
                 config=None, menu=None):
        self.clock = clock
        self.ser = FakeSerial(clock)
        self.steps = list(steps)
        self.key = None
        self.due = 0.0
        self.menu = list(menu) if menu is not None else ['12', entry]
        self.lines = []
        self.prompts = []
        self.reads = 0
        real_read = self.ser.read

        self.timeline = []

        def guarded_read(size=1):
            self.reads += 1
            if self.reads > max_reads:
                raise AssertionError('check did not finish:\n' + self.text[-1500:])
            while self.timeline and self.clock.now >= self.timeline[0][0]:
                self.ser.stream = self.timeline.pop(0)[1]
                self.ser._rx.clear()
            return real_read(size)
        self.ser.read = guarded_read
        self.rc = cli.run_menu(self.ser, self._input, self._output,
                               config or tmp_path / 'c.yaml', sleep_fn=clock.sleep,
                               time_fn=clock.time, color=True,
                               key_ready=lambda: self.key is not None
                               and self.clock.now >= self.due)

    def _next_step(self):
        stream, key, delay = self.steps.pop(0)
        if isinstance(stream, list):              # [(seconds after the prompt, bytes), ...]
            self.timeline = [(self.clock.now + t, b) for t, b in stream]
        elif stream is not None:
            self.timeline = []
            self.ser.stream = stream
        self.key, self.due = key, self.clock.now + delay

    def _output(self, *args, **kwargs):
        line = ' '.join(str(a) for a in args)
        self.lines.append(line)
        if 's 略過 / q 結束' in line and self.steps:
            self._next_step()

    def _input(self, prompt=''):
        self.prompts.append(prompt)
        if '磁傾角' in prompt and self.steps:
            self._next_step()
        if self.key is not None:
            key, self.key = self.key, None
            if key == '^C':
                raise KeyboardInterrupt
            return key
        return self.menu.pop(0) if self.menu else '0'

    @property
    def text(self):
        return '\n'.join(self.lines)


def _good_steps(dip_answer='36'):
    faces = [(_face_stream(a, s), None, 0) for a, s in cc.FACES]
    return faces + [
        (STILL + _acc_frame(0, 0, G0), None, 0),                     # gyro at rest
        (STILL + _acc_frame(0, 0, G0), '', 0),                       # align with the line
        (_gyro_frame(0, 0, 36.0) + _acc_frame(0, 0, G0), '', 10.05),  # one turn in 10 s
        (None, dip_answer, 0),                                       # local inclination
        (_mag_check_stream(), None, 0),                              # magnetometer
    ]


@pytest.fixture
def short_still(monkeypatch):
    """Shorten the 5 minute static gyro test for the unit tests."""
    monkeypatch.setattr(cli, 'GYRO_STILL_S', 3.0)


def test_check_passes_a_good_imu_and_marks_option_12(clock, tmp_path, short_still):
    run = _Check(clock, tmp_path, _good_steps())
    assert run.rc == 0
    for item in ('1. 加速度計', '2. 陀螺儀 靜止', '3. 陀螺儀 轉一圈', '4. 磁力計 合成大小',
                 '5. 磁力計 磁傾角'):
        assert f'[合格] {item}' in run.text, item
    assert '5/5 項全部合格' in run.text
    last_menu = [ln for ln in run.lines if 'CMP10A IMU 校正工具' in ln][-1]
    assert cli.DONE_MARK in _menu_row(last_menu, 12)
    assert run.ser.writes == []                                      # read-only


def test_check_finds_an_accelerometer_offset(clock, tmp_path, short_still):
    steps = [(_face_stream(a, s, bias=(0.3, 0.0, 0.0)), None, 0) for a, s in cc.FACES]
    run = _Check(clock, tmp_path, steps + [(None, 'q', 0)])
    assert '[不合格] 1. 加速度計 六面測試' in run.text
    assert '偏移 X +0.30' in run.text


def test_check_finds_a_gyro_bias(clock, tmp_path, short_still):
    steps = [(_gyro_frame(0, 0, 0.13) + _acc_frame(0, 0, G0), None, 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='2')
    assert '[不合格] 2. 陀螺儀 靜止測試' in run.text
    assert 'Z +0.12' in run.text


def test_check_applies_the_software_calibration(clock, tmp_path, short_still):
    """With the bias saved by option 9, the same gyro output passes."""
    raw_z = _gyro_frame(0, 0, 0.13)
    z_rad = wp.decode_frame(raw_z).z
    cfg = tmp_path / 'c.yaml'
    cal.save_calibration(cfg, cal.SoftwareCalibration(gyro_bias=(0.0, 0.0, z_rad)))
    steps = [(raw_z + _acc_frame(0, 0, G0), None, 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='2', config=cfg)
    assert '已套用軟體校正' in run.text
    assert '[合格] 2. 陀螺儀 靜止測試' in run.text


def test_check_gyro_still_restarts_when_moved(clock, tmp_path, short_still):
    moving = STILL + _acc_frame(0, 0, G0) + STILL + _acc_frame(2.0, 0, G0)
    run = _Check(clock, tmp_path, [(moving, 's', 6.0), (None, 'q', 0)], entry='2')
    assert '重新開始' in run.text
    assert '[略過] 2. 陀螺儀 靜止測試' in run.text


def test_check_full_turn_fails_when_the_gyro_reads_short(clock, tmp_path):
    steps = [(STILL, '', 0), (_gyro_frame(0, 0, 30.0), '', 10.05), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='3')
    assert '[不合格] 3. 陀螺儀 轉一圈測試' in run.text
    assert '已轉' in run.text                         # live angle while turning


def test_check_magnetometer_detects_hard_iron(clock, tmp_path):
    """
    A hard-iron offset is caught by the magnitude test.

    Averaged over every direction the offset cancels in the mean dip angle,
    so the dip test cannot see it; that is why both tests are needed.
    """
    steps = [(None, '36', 0), (_mag_check_stream(offset=(900.0, 0.0, 0.0)), None, 0),
             (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='4')
    assert '[不合格] 4. 磁力計 合成大小' in run.text


def test_check_magnetometer_compares_with_the_entered_inclination(clock, tmp_path):
    steps = [(None, '30', 0), (_mag_check_stream(dip_deg=36.0), None, 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='4')
    assert '[合格] 4. 磁力計 合成大小' in run.text
    assert '[不合格] 5. 磁力計 磁傾角:量到 +36.0°,當地 +30.0°' in run.text


def test_check_dip_is_skipped_without_an_inclination(clock, tmp_path):
    steps = [(None, '', 0), (_mag_check_stream(), None, 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='4')
    assert '[略過] 5. 磁力計 磁傾角:未輸入當地磁傾角' in run.text
    assert 'ngdc.noaa.gov' in run.text


def test_check_rejects_an_invalid_inclination(clock, tmp_path):
    steps = [(None, '95', 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='4')
    assert '-90 到 90' in run.text


def test_check_shows_which_face_is_up(clock, tmp_path):
    steps = [(_face_stream('X', +1), 's', 3.0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='1')
    assert '目前:X 軸朝上,請改成 Z 軸朝上' in run.text
    assert '[略過] 1. 加速度計 六面測試' in run.text


def test_check_single_item_does_not_mark_option_12(clock, tmp_path):
    steps = [(_face_stream(a, s), None, 0) for a, s in cc.FACES] + [(None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='1')
    assert '[合格] 1. 加速度計 六面測試' in run.text
    assert '本次已合格 1/5 項' in run.text
    last_menu = [ln for ln in run.lines if 'CMP10A IMU 校正工具' in ln][-1]
    assert cli.DONE_MARK not in _menu_row(last_menu, 12)


def test_check_needs_a_terminal(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['12', '', '0'], clock, tmp_path)
    assert '需要在終端機中執行' in out.text
    assert ser.writes == []


# ------------------------ environment / mounting check before a mag calibration

def _env_stream(mag=(4000, 0, 0), accel=(0.0, 0.0, G0), gyro=STILL):
    return gyro + _acc_frame(*accel) + _mag_frame(*mag)


STEADY = _env_stream()
NOISY = _env_stream() + _env_stream(mag=(4300, 0, 0))            # field jumps 7 %
NEAR_IRON_END = _env_stream(mag=(4000, 700, 0))                  # field differs at the new place
MOTORS = (_env_stream(mag=(4000, 600, 0), accel=(0.0, 0.0, G0 + 2.0))
          + _env_stream(mag=(4000, -600, 0), accel=(0.0, 0.0, G0 - 2.0)))


def _slide(start=STEADY, end=STEADY, moving=None):
    """IMU data during the slide: at rest, being pushed, at rest in the new place."""
    moving = moving or _env_stream(accel=(0.6, 0.0, G0))
    return [(0.0, start), (2.5, moving), (3.5, end)]


def _env_steps(still=STEADY, end=STEADY):
    return [(still, None, 0), (_slide(end=end), None, 0)]


MAG_SPHERE_CMD = wp.cmd_calsw(wp.CalMode.MAG_SPHERE)


def test_mag_calibration_checks_the_environment_first(clock, tmp_path):
    """A good place passes in green and the calibration goes on as before."""
    run = _Check(clock, tmp_path, _env_steps(), menu=['3', 'n', 'y', ''])
    assert '[合格] 1. 磁場穩定' in run.text
    assert '[合格] 2. 附近有無鐵件' in run.text
    assert '\033[32m[合格] 環境與安裝位置合適' in run.text
    assert not any('仍要在這裡繼續校正' in p for p in run.prompts)
    assert MAG_SPHERE_CMD in run.ser.writes


def test_unsuitable_place_is_explained_and_can_be_cancelled(clock, tmp_path):
    run = _Check(clock, tmp_path, _env_steps(still=NOISY), menu=['3', 'n', 'n'])
    assert '[不合格] 1. 磁場穩定' in run.text
    assert '附近有馬達在轉' in run.text
    assert '已取消校正,未送出任何指令' in run.text
    assert run.ser.writes == []


def test_unsuitable_place_asks_a_second_time(clock, tmp_path):
    """Saying yes once is not enough: the second question can still cancel."""
    run = _Check(clock, tmp_path, _env_steps(end=NEAR_IRON_END), menu=['3', 'n', 'y', 'n'])
    assert '[不合格] 2. 附近有無鐵件' in run.text
    assert any('仍要在這裡繼續校正' in p for p in run.prompts)
    assert any('再確認一次' in p for p in run.prompts)
    assert '已取消校正' in run.text
    assert run.ser.writes == []


def test_user_who_insists_twice_gets_the_calibration(clock, tmp_path):
    run = _Check(clock, tmp_path, _env_steps(end=NEAR_IRON_END),
                 menu=['3', 'n', 'y', 'y', 'y', ''])
    assert '依你的選擇,繼續校正' in run.text
    assert MAG_SPHERE_CMD in run.ser.writes


def test_slide_that_turns_starts_over_by_itself(clock, tmp_path):
    """Turning voids the attempt; it restarts on its own and then passes."""
    turning = _env_stream(gyro=_gyro_frame(0, 0, 10.0), accel=(0.6, 0.0, G0))
    slide = [(0.0, STEADY), (2.5, turning), (3.5, STEADY),
             (6.0, _env_stream(accel=(0.6, 0.0, G0))), (7.0, STEADY)]
    run = _Check(clock, tmp_path, [(STEADY, None, 0), (slide, None, 0)], menu=['3', 'n', 'n'])
    assert '不算;停住後自動重來' in run.text
    assert '[合格] 2. 附近有無鐵件' in run.text


def test_slide_finishes_without_any_key(clock, tmp_path):
    run = _Check(clock, tmp_path, _env_steps(), menu=['3', 'n', 'n'])
    assert '準備好了:不要轉動,開始平移' in run.text
    assert '[合格] 2. 附近有無鐵件' in run.text


def test_slide_ignores_the_gyro_bias_of_an_uncalibrated_imu(clock, tmp_path):
    """A 1 deg/s bias would add up past 3 deg; the bias seen at rest is removed."""
    biased = _gyro_frame(0, 0, 1.0)
    rest = _env_stream(gyro=biased)
    slide = [(0.0, rest), (2.5, _env_stream(gyro=biased, accel=(0.6, 0.0, G0))),
             (5.0, rest)]
    run = _Check(clock, tmp_path, [(rest, None, 0), (slide, None, 0)], menu=['3', 'n', 'n'])
    assert '[合格] 2. 附近有無鐵件' in run.text


def test_mounting_position_near_motors_is_reported(clock, tmp_path):
    # The motors are found by themselves: no key is pressed.
    steps = _env_steps() + [(STEADY, None, 0), (MOTORS, None, 0)]
    run = _Check(clock, tmp_path, steps, menu=['3', 'y', 'n'])
    assert '偵測到馬達轉動' in run.text
    assert '[合格] 3. 安裝方向' in run.text
    assert '[不合格] 4. 馬達干擾' in run.text
    assert '[不合格] 5. 振動' in run.text
    assert '離馬達或電源線太近' in run.text
    assert run.ser.writes == []


def test_tilted_mounting_is_a_note_not_a_failure(clock, tmp_path):
    tilted = _env_stream(accel=(G0 * math.sin(math.radians(10)), 0.0,
                                G0 * math.cos(math.radians(10))))
    moving = _env_stream(accel=(G0 * math.sin(math.radians(10)) + 0.6, 0.0,
                                G0 * math.cos(math.radians(10))))
    steps = [(tilted, None, 0), (_slide(tilted, tilted, moving), None, 0),
             (STEADY, None, 0), (STEADY, '', 12.0)]       # well mounted: Enter starts the run
    run = _Check(clock, tmp_path, steps, menu=['3', 'y', 'y', ''])
    assert '[注意] 3. 安裝方向:傾斜 10.0°' in run.text
    assert '環境與安裝位置合適' in run.text
    assert MAG_SPHERE_CMD in run.ser.writes


def test_option_12_can_check_the_place_without_calibrating(clock, tmp_path):
    run = _Check(clock, tmp_path, _env_steps(end=NEAR_IRON_END), menu=['12', '5', 'n'])
    assert '[不合格] 2. 附近有無鐵件' in run.text
    assert not any('仍要在這裡繼續校正' in p for p in run.prompts)
    assert run.ser.writes == []


def test_quitting_the_check_cancels_the_calibration(clock, tmp_path):
    run = _Check(clock, tmp_path, [(STEADY, 'q', 0.5)], menu=['3'])
    assert '已取消校正' in run.text
    assert run.ser.writes == []


def test_environment_check_is_skipped_off_a_terminal(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['3', 'n', '0'], clock, tmp_path)
    assert '環境與安裝位置檢查需要在終端機中執行' in out.text


def test_dual_plane_does_not_ask_whether_the_imu_is_mounted(clock, tmp_path):
    """Option 4 exists for robots too big to flip, so the IMU is already mounted."""
    steps = _env_steps() + [(STEADY, None, 0), (STEADY, '', 12.0)]
    run = _Check(clock, tmp_path, steps, menu=['4', 'n'])
    assert not any('裝在車上' in p for p in run.prompts)
    assert '[合格] 3. 安裝方向' in run.text                 # mounting checks still run
    assert '4. 馬達干擾' in run.text


def test_sphere_fit_still_asks_whether_the_imu_is_mounted(clock, tmp_path):
    run = _Check(clock, tmp_path, _env_steps(), menu=['3', 'n', 'n'])
    assert any('裝在車上' in p for p in run.prompts)


def test_placement_notes_are_shown_again_after_the_check(clock, tmp_path):
    """The check clears the screen, so the notes for the option come back afterwards."""
    run = _Check(clock, tmp_path, _env_steps(), menu=['3', 'n', 'n'])
    last_clear = max(i for i, ln in enumerate(run.lines) if cli.CLEAR_SCREEN in ln)
    after = '\n'.join(run.lines[last_clear:])
    assert '注意事項' in after and '車上的鐵件' in after


def test_motor_check_reminds_to_stop_the_motors(clock, tmp_path):
    steps = _env_steps() + [(STEADY, None, 0), (STEADY, '', 12.0)]
    run = _Check(clock, tmp_path, steps, menu=['3', 'y', 'n'])
    assert '先讓馬達停止' in run.text


# ------------------------------------------------- fixes from the code review

def _wobble_stream(offset=(900.0, 0.0, 0.0), n=40):
    """
    Return data for an IMU wobbled by a few degrees around Z-up.

    All three magnetometer axes move by a similar small amount, which is the
    case where coverage measured against the data itself looks complete.
    """
    dip = math.radians(36.0)
    base = (4000 * math.cos(dip) + offset[0], offset[1], -4000 * math.sin(dip) + offset[2])
    out = b''
    for i in range(n):
        a = 2 * math.pi * i / n
        t = math.radians(4.0 * math.sin(a))
        acc = (G0 * math.sin(t), 0.0, G0 * math.cos(t))
        mag = (base[0] + 150 * math.cos(a), base[1] + 150 * math.sin(a),
               base[2] + 150 * math.sin(a + 1.0))
        out += _acc_frame(*acc) + STILL + _mag_frame(*(round(c) for c in mag))
    return out


def test_check_magnetometer_needs_real_turning_not_a_wobble(clock, tmp_path):
    """Coverage comes from gravity directions, so a small wobble never completes."""
    steps = [(None, '36', 0), (_wobble_stream(), '', 8.0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, entry='4')
    assert '[合格] 4.' not in run.text
    assert '[略過] 4. 磁力計 合成大小:方向涵蓋不足' in run.text


def test_a_later_failure_removes_an_earlier_pass(clock, tmp_path, short_still):
    steps = _good_steps() + [(None, '', 0),
                             (_mag_check_stream(offset=(900.0, 0.0, 0.0)), None, 0),
                             (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, menu=['12', '', '12', '4'])
    assert '5/5 項全部合格' in run.text                        # first run
    last = run.text.rsplit('== 檢測結果 ==', 1)[1]
    assert '[不合格] 4. 磁力計 合成大小' in last
    assert '全部合格' not in last
    last_menu = [ln for ln in run.lines if 'CMP10A IMU 校正工具' in ln][-1]
    assert cli.DONE_MARK not in _menu_row(last_menu, 12)


def test_skipping_a_repeated_check_revokes_its_previous_pass(clock, tmp_path, short_still):
    """A skipped retest must not silently reuse its old green result."""
    steps = _good_steps() + [(None, '', 0), (STEADY, 's', 0.1), (None, 'q', 0)]
    result = _Check(clock, tmp_path, steps, menu=['12', '', '12', '4'])
    assert '5/5 項全部合格' in result.text
    last = result.text.rsplit('== 檢測結果 ==', 1)[-1]
    assert '5/5 項全部合格' not in last
    last_menu = [line for line in result.lines if 'CMP10A IMU 校正工具' in line][-1]
    assert cli.DONE_MARK not in _menu_row(last_menu, 12)


@pytest.mark.parametrize('choice,stream,answers', [
    ('2', b'', ['y']),
])
def test_calibration_changes_invalidate_previous_verification(clock, tmp_path,
                                                              choice, stream, answers):
    menu = cli.CalibrationMenu(FakeSerial(clock, stream=stream), scripted(answers), Output(),
                               tmp_path / 'c.yaml', sleep_fn=clock.sleep, time_fn=clock.time)
    menu._check_passed.update(cli.CHECK_ITEMS)
    menu._done.add('12')
    menu._actions[choice]()
    assert not menu._check_passed
    assert '12' not in menu._done


def test_skipped_environment_step_is_not_reported_as_suitable(clock, tmp_path):
    result = _Check(clock, tmp_path, [(STEADY, None, 0), (STEADY, 's', 0.1)],
                    menu=['3', 'n', 'n'])
    assert '環境與安裝位置合適' not in result.text
    assert '檢查未完成' in result.text


def test_interrupted_full_retest_cannot_reuse_unvisited_results(clock, tmp_path, short_still):
    steps = _good_steps() + [(STEADY, 'q', 0.1)]
    result = _Check(clock, tmp_path, steps, menu=['12', '', '12', ''])
    assert '5/5 項全部合格' in result.text
    last_menu = [line for line in result.lines if 'CMP10A IMU 校正工具' in line][-1]
    assert cli.DONE_MARK not in _menu_row(last_menu, 12)


def test_motor_step_does_not_accept_enter_before_waiting(clock, tmp_path):
    """Enter right away would measure the motors stopped; it is refused at first."""
    steps = _env_steps() + [(STEADY, None, 0), (STEADY, '', 0.5), (STEADY, '', 12.0)]
    run = _Check(clock, tmp_path, steps, menu=['3', 'y', 'n'])
    assert '還在偵測馬達' in run.text
    assert '4. 馬達干擾' in run.text


def test_check_gyro_still_restarts_when_turned_about_z(clock, tmp_path, short_still):
    rest = STILL + _acc_frame(0, 0, G0)
    turning = _gyro_frame(0, 0, 20.0) + _acc_frame(0, 0, G0)   # accel unchanged
    run = _Check(clock, tmp_path, [([(0.0, rest), (2.0, turning), (2.5, rest)], 's', 3.2),
                                   (None, 'q', 0)], entry='2')
    assert '重新開始' in run.text


def test_check_gyro_still_ignores_enter_while_measuring(clock, tmp_path, short_still):
    rest = STILL + _acc_frame(0, 0, G0)
    run = _Check(clock, tmp_path, [(rest, '', 2.5), (rest, None, 0)], entry='2')
    assert '量測中,不用按 Enter' in run.text
    assert '[合格] 2. 陀螺儀 靜止測試' in run.text


def test_inclination_is_asked_every_time_with_the_last_value_offered(clock, tmp_path):
    steps = [(None, '30', 0), (_mag_check_stream(), None, 0),
             (None, '36', 0), (_mag_check_stream(), None, 0), (None, 'q', 0)]
    run = _Check(clock, tmp_path, steps, menu=['12', '4', '12', '4'])
    dip_prompts = [p for p in run.prompts if '磁傾角' in p]
    assert len(dip_prompts) == 2
    assert '上次 30.0' in dip_prompts[1]
    assert '[合格] 5. 磁力計 磁傾角:量到 +36.0°,當地 +36.0°' in run.text


def test_all_zero_magnetometer_does_not_crash_the_tool(clock, tmp_path):
    dead = _env_stream(mag=(0, 0, 0))
    run = _Check(clock, tmp_path, [(dead, None, 0), (None, 's', 0)], menu=['3', 'n', 'n'])
    assert run.rc == 0
    assert '資料異常' in run.text


def test_ctrl_c_during_the_check_still_shows_the_results_so_far(clock, tmp_path, short_still):
    steps = [(_face_stream(a, s), None, 0) for a, s in cc.FACES]
    steps += [(STILL + _acc_frame(0, 0, G0), '^C', 0.5)]
    run = _Check(clock, tmp_path, steps)
    assert run.rc == 0
    assert '== 檢測結果 ==' in run.text
    tail = run.text.rsplit('== 檢測結果 ==', 1)[-1]
    assert '[合格] 1. 加速度計 六面測試' in tail


def test_placement_tilt_uses_the_software_calibration(clock, tmp_path):
    """A saved accel offset is removed before the tilt is judged."""
    cfg = tmp_path / 'c.yaml'
    cal.save_calibration(cfg, cal.SoftwareCalibration(accel_bias=(1.7, 0.0, 0.0)))
    leaning = _env_stream(accel=(1.7, 0.0, G0))           # raw reading ~10 deg, true 0
    moving = _env_stream(accel=(2.3, 0.0, G0))
    steps = [(leaning, None, 0), (_slide(leaning, leaning, moving), None, 0),
             (leaning, None, 0), (leaning, '', 12.0)]
    run = _Check(clock, tmp_path, steps, menu=['3', 'y', 'n'], config=cfg)
    assert '[合格] 3. 安裝方向' in run.text


# ------------------------------------------------ official calibration steps

READ_AXIS6 = wp.cmd_read(cli.REG_AXIS6)
READ_GYROCAL = wp.cmd_read(cli.REG_GYROCALITHR)


def _reply(*values):
    vals = list(values) + [0] * (4 - len(values))
    return make_frame(0x5F, i16(*vals[:4]))


def _is_read(cmd):
    return cmd[:3] == bytes([0xFF, 0xAA, wp.REG_READ])


def test_accel_calibration_holds_still_for_the_official_time(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.ACCEL), NORMAL, SAVE]
    t_cal, t_normal = ser.write_times[1], ser.write_times[2]
    assert t_normal - t_cal >= cli.ACCEL_CAL_WAIT == 2.0
    assert '水平' in out.text


def test_accel_calibration_shows_the_readings_to_expect(clock, tmp_path):
    """The manual says to check about 0, 0, 1 g and 0 deg afterwards."""
    stream = _level_accel() + _angle_frame(0.1, -0.2, 30.0)
    ser = FakeSerial(clock, stream=stream)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert '校正後讀數' in out.text
    assert 'Z +1.000 g' in out.text
    assert 'roll +0.10°' in out.text
    assert '0、0、1 g' in out.text


def test_heading_zero_is_refused_in_nine_axis_mode(clock, tmp_path):
    ser = FakeSerial(clock, responses={READ_AXIS6: _reply(0)})
    rc, out, _ = run(ser, ['5', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [READ_AXIS6]
    assert '九軸' in out.text and '不能歸零' in out.text
    assert '選項 10' in out.text


def test_heading_zero_runs_in_six_axis_mode(clock, tmp_path):
    ser = FakeSerial(clock, responses={READ_AXIS6: _reply(1)})
    rc, out, _ = run(ser, ['5', 'y', '0'], clock, tmp_path)
    assert ser.writes == [READ_AXIS6, UNLOCK, wp.cmd_calsw(wp.CalMode.HEADING_ZERO),
                          NORMAL, SAVE]
    assert '100%' in out.text


def test_heading_zero_asks_when_the_mode_is_unknown(clock, tmp_path):
    ser = FakeSerial(clock)                                  # no register reply
    rc, out, inp = run(ser, ['5', 'n', '0'], clock, tmp_path)
    assert '讀不到演算法設定' in out.text
    assert all(_is_read(w) for w in ser.writes)


def test_angle_reference_shows_the_official_warning_first(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, inp = run(ser, ['6', 'n', '0'], clock, tmp_path)
    assert any('官方警告' in p for p in inp.prompts)
    assert ser.writes == []
    ser = FakeSerial(clock)
    rc, out, inp = run(ser, ['6', 'y', 'y', '0'], clock, tmp_path)
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.ANGLE_REF), NORMAL, SAVE]


def test_height_zero_sends_the_command(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['7', 'y', '0'], clock, tmp_path)
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.HEIGHT_ZERO), NORMAL, SAVE]
    assert '100%' in out.text


@pytest.mark.parametrize('choice,answers', [
    ('2', ['n']), ('3', ['n']), ('4', ['n']), ('6', ['n']), ('7', ['n']), ('5', ['n']),
])
def test_declining_a_hardware_step_writes_nothing(clock, tmp_path, choice, answers):
    """Only register reads may be sent when the user says no."""
    ser = FakeSerial(clock, stream=_sphere_stream())
    rc, _, _ = run(ser, [choice] + answers + ['0'], clock, tmp_path)
    assert rc == 0
    assert all(_is_read(w) for w in ser.writes)


def test_mag_calibration_waits_for_enter(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream())
    rc, out, inp = run(ser, ['3', 'y', '', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.MAG_SPHERE), NORMAL, SAVE]
    assert '平面' in out.text and '南北' in out.text
    assert any('Enter' in p for p in inp.prompts)


@pytest.mark.parametrize('choice,mode', [
    ('3', wp.CalMode.MAG_SPHERE),
    ('4', wp.CalMode.MAG_DUAL_PLANE),
])
def test_mag_calibration_progress_uses_magnetometer_with_frozen_gyro(clock, tmp_path,
                                                                     choice, mode):
    # Regression: the real CMP10A freezes gyro output in mag-cal mode, so progress
    # must come from the magnetometer.
    ser = FakeSerial(clock, stream=_sphere_stream())
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, choice, '[完成] 按 Enter')
    assert rc == 0
    assert ser.writes[:2] == [UNLOCK, wp.cmd_calsw(mode)]
    assert ser.writes[-2:] == [NORMAL, SAVE]
    assert '尚未轉完' not in out.text


def test_sphere_calibration_shows_each_plane_and_what_is_missing(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream(planes=('XY',)))
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '3', '缺 XZ YZ')
    assert rc == 0
    last = [ln for ln in _status_lines(out) if '缺' in ln][-1]
    assert 'XY [######] 100%' in last and 'XZ [' in last and 'YZ [' in last
    assert '尚未轉完' in out.text                              # Enter pressed too early
    assert ser.writes[-1:] == [NORMAL]
    assert SAVE not in ser.writes


def test_dual_plane_needs_a_level_turn_then_an_upright_turn(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream(planes=('XY',)))
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '4', '缺 垂直')
    lines = _status_lines(out)
    assert any('第 1 段' in ln for ln in lines)
    assert any('第 2 段' in ln and '缺 垂直' in ln for ln in lines)
    assert SAVE not in ser.writes


def test_dual_plane_will_not_start_when_tilted_over_15_degrees(clock, tmp_path):
    tilted = _sphere_stream().replace(_level_accel(), _level_accel(30.0))
    ser = FakeSerial(clock, stream=tilted)
    rc, out, inp = run(ser, ['4', 'y', 'q', '0'], clock, tmp_path)
    assert rc == 0
    assert any('不可超過 15°' in p for p in inp.prompts)
    assert ser.writes == []                                  # nothing sent


def test_mag_calibration_explains_which_rotation_fixes_each_plane(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['3', 'n', '0'], clock, tmp_path)
    assert rc == 0
    assert 'XY 不夠' in out.text and '繞 Z 軸轉' in out.text


@pytest.mark.parametrize('choice,planes', [('3', ('XY',)), ('3', ('XY', 'XZ', 'YZ')),
                                           ('4', ('XY',)), ('4', ('XY', 'XZ'))])
def test_mag_status_line_fits_80_columns(clock, tmp_path, choice, planes):
    ser = FakeSerial(clock, stream=_sphere_stream(planes=planes))
    wait = '[完成] 按 Enter' if len(planes) > 1 else '缺'
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, choice, wait)
    widths = [_display_width(ln) for ln in _status_lines(out)]
    assert widths and max(widths) <= 79, max(widths)


def test_partial_mag_save_requires_confirmation_and_is_not_marked_done(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream(planes=('XY',)))
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '3', '缺 XZ YZ', partial_answer='y')
    assert rc == 0
    assert ser.writes[-1] == SAVE
    assert cli.DONE_MARK not in _menu_row(_menu_blocks(out)[-1], 3)


def test_gyro_auto_calibration_status_is_read_only(clock, tmp_path):
    ser = FakeSerial(clock, responses={READ_GYROCAL: _reply(50, 0, 1000, 0)})
    rc, out, _ = run(ser, ['9', '0'], clock, tmp_path)
    assert ser.writes == [READ_GYROCAL]
    assert '0.050°/s' in out.text and '1000 ms' in out.text
    assert '預設' in out.text and '等速旋轉' in out.text


def test_gyro_auto_calibration_status_without_reply(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['9', '0'], clock, tmp_path)
    assert '逾時' in out.text
    assert ser.writes == [READ_GYROCAL]


class _AlgorithmSerial(FakeSerial):
    """Remember the AXIS6 value written, like the module does."""

    def __init__(self, clock, axis6):
        super().__init__(clock)
        self.axis6 = axis6

    def write(self, data):
        data = bytes(data)
        if data[:3] == bytes([0xFF, 0xAA, cli.REG_AXIS6]):
            self.axis6 = data[3]
        self.responses = {READ_AXIS6: _reply(self.axis6)}
        return super().write(data)


def test_switch_to_six_axis_writes_and_reads_back(clock, tmp_path):
    ser = _AlgorithmSerial(clock, axis6=0)
    rc, out, _ = run(ser, ['10', 'y', '0'], clock, tmp_path)
    assert ser.writes == [READ_AXIS6, UNLOCK, wp.cmd_write(cli.REG_AXIS6, 1), SAVE, READ_AXIS6]
    assert '已切換為六軸' in out.text
    assert 'Z 軸歸零(5)' in out.text
    assert cli.DONE_MARK in _menu_row(_menu_blocks(out)[-1], 10)


def test_switch_algorithm_declined_only_reads(clock, tmp_path):
    ser = _AlgorithmSerial(clock, axis6=1)
    rc, out, _ = run(ser, ['10', 'n', '0'], clock, tmp_path)
    assert '目前:六軸' in out.text
    assert ser.writes == [READ_AXIS6]


def test_switch_algorithm_reports_an_unconfirmed_change(clock, tmp_path):
    ser = FakeSerial(clock, responses={READ_AXIS6: _reply(0)})   # never changes
    rc, out, _ = run(ser, ['10', 'y', '0'], clock, tmp_path)
    assert '狀態未知' in out.text
    assert cli.DONE_MARK not in _menu_row(_menu_blocks(out)[-1], 10)


def test_notes_follow_the_official_manual():
    assert set(cli.NOTES) == {'2', '3', '4', '5', '6', '7', '10'}
    assert any('50 cm' in n for n in cli.NOTES['3'])
    assert any('六軸' in n for n in cli.NOTES['5'])
    assert any('官方警告' in n for n in cli.NOTES['6'])
    assert any('選項 13' in n for n in cli.NOTES['7'])
