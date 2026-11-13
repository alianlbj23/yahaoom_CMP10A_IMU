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
from wit_ros2_imu import wit_protocol as wp
import yaml

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

@pytest.mark.parametrize('choice', ['2', '3', '4', '5', '6', '7'])
@pytest.mark.parametrize('answer', ['n', 'N', '', 'no', 'whatever'])
def test_hardware_calibration_declined_sends_nothing(clock, tmp_path, choice, answer):
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, [choice, answer, '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []


def test_accel_calibration_sequence_and_wait(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.ACCEL), NORMAL, SAVE]
    t_cal, t_normal = ser.write_times[1], ser.write_times[2]
    assert t_normal - t_cal >= 5.0
    assert '水平' in out.text


@pytest.mark.parametrize('choice,mode', [
    ('3', wp.CalMode.MAG_SPHERE),
    ('4', wp.CalMode.MAG_DUAL_PLANE),
])
def test_mag_calibration_waits_for_enter(clock, tmp_path, choice, mode):
    ser = FakeSerial(clock)
    rc, out, inp = run(ser, [choice, 'y', '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(mode), NORMAL, SAVE]
    assert '三軸' in out.text
    assert any('Enter' in p for p in inp.prompts)


def test_mag_calibration_long_rotation_reunlocks_before_normal(clock, tmp_path):
    # Unlock only lasts ~10 s; after a 60 s rotation the CLI re-unlocks first.
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, ['3', 'y', '', '0'], clock, tmp_path, advance=60.0)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(wp.CalMode.MAG_SPHERE),
                          UNLOCK, NORMAL, SAVE]


@pytest.mark.parametrize('choice,mode', [
    ('5', wp.CalMode.HEADING_ZERO),
    ('6', wp.CalMode.ANGLE_REF),
    ('7', wp.CalMode.HEIGHT_ZERO),
])
def test_simple_hardware_commands(clock, tmp_path, choice, mode):
    ser = FakeSerial(clock)
    rc, _, _ = run(ser, [choice, 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == [UNLOCK, wp.cmd_calsw(mode), NORMAL, SAVE]


def test_hardware_write_failure_is_reported(clock, tmp_path):
    ser = FakeSerial(clock, fail_write=True)
    rc, out, _ = run(ser, ['2', 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert 'device disconnected' in out.text
    assert '失敗' in out.text


def test_ctrl_c_during_mag_calibration_restores_normal_without_save(clock, tmp_path):
    ser = FakeSerial(clock)
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


# ---------------------------------------------------------------- software calibration

def _static_stream():
    acc = make_frame(0x51, i16(20, -10, 2048 + 41, 2500))
    gyro = make_frame(0x52, i16(3, -3, 6, 0))
    angle = make_frame(0x53, i16(0, 0, 0, 0))
    mag = make_frame(0x54, i16(0, 0, 0, 2500))
    return acc + gyro + angle + mag


def test_software_static_calibration_writes_yaml(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    cal.save_calibration(path, cal.SoftwareCalibration(mag_offset=(1.0, 2.0, 3.0)))
    ser = FakeSerial(clock, stream=_static_stream())
    rc, out, _ = run(ser, ['9', '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    data = yaml.safe_load(path.read_text())
    g_scale = 2000.0 / 32768.0 * 3.141592653589793 / 180.0
    a_scale = 16.0 / 32768.0 * wp.GRAVITY
    assert data['gyro_bias'] == pytest.approx([3 * g_scale, -3 * g_scale, 6 * g_scale])
    assert data['accel_bias'] == pytest.approx([20 * a_scale, -10 * a_scale, 41 * a_scale])
    assert data['mag_offset'] == pytest.approx([1.0, 2.0, 3.0])   # preserved
    assert str(path) in out.text


def test_software_static_calibration_no_data(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['9', '', '0'], clock, tmp_path)
    assert rc == 0
    assert not path.exists()
    assert '失敗' in out.text


def test_software_static_calibration_corrupt_existing_yaml(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    path.write_text('gyro_bias: [oops\n')
    ser = FakeSerial(clock, stream=_static_stream())
    rc, out, _ = run(ser, ['9', '', '0'], clock, tmp_path)
    assert rc == 0
    assert '格式錯誤' in out.text
    data = yaml.safe_load(path.read_text())
    assert data['mag_offset'] == [0.0, 0.0, 0.0]


def _mag_stream():
    frames = b''
    for i in range(12):
        x = -100 + 20 * i
        frames += make_frame(0x54, i16(x + 50, -x - 30, 2 * x, 2500))
    return frames


def test_software_mag_calibration_writes_yaml(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    cal.save_calibration(path, cal.SoftwareCalibration(gyro_bias=(0.1, 0.2, 0.3)))
    ser = FakeSerial(clock, stream=_mag_stream())
    rc, out, _ = run(ser, ['10', '', '0'], clock, tmp_path)
    assert rc == 0
    assert ser.writes == []
    data = yaml.safe_load(path.read_text())
    # x: [-50, 170] -> 60; y: [-150, 70] -> -40; z: [-200, 240] -> 20
    assert data['mag_offset'] == pytest.approx([60.0, -40.0, 20.0])
    assert data['gyro_bias'] == pytest.approx([0.1, 0.2, 0.3])   # preserved
    assert '轉動' in out.text


def test_software_mag_calibration_no_data(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['10', '', '0'], clock, tmp_path)
    assert rc == 0
    assert not (tmp_path / 'imu_calibration.yaml').exists()
    assert '失敗' in out.text


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
    '3': '最終安裝位置',
    '4': '最終安裝位置',
    '5': '正前方',
    '6': '水平地面',
    '7': '0 m',
    '9': '暖機',
    '10': '最終安裝位置',
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


def test_notes_cover_every_calibration_choice():
    assert set(cli.NOTES) == {'2', '3', '4', '5', '6', '7', '9', '10'}
    assert all(cli.NOTES[k] for k in cli.NOTES)


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
    assert ser.writes == []
    notes_idx = out.lines.index('注意事項:')
    confirm_idx = next(i for i, ln in enumerate(out.lines)
                       if ln.startswith('<PROMPT>') and 'y/N' in ln)
    assert notes_idx < confirm_idx
    for note in cli.NOTES[choice]:
        assert note in out.lines[notes_idx:confirm_idx]


@pytest.mark.parametrize('choice', ['9', '10'])
def test_software_notes_shown_before_start(clock, tmp_path, choice):
    ser = FakeSerial(clock)
    rc, out = _logged_run(ser, [choice], clock, tmp_path)   # EOF at "press Enter"
    assert rc == 0
    assert ser.writes == []
    notes_idx = out.lines.index('注意事項:')
    first_prompt_after = next(i for i, ln in enumerate(out.lines)
                              if i > notes_idx and ln.startswith('<PROMPT>'))
    for note in cli.NOTES[choice]:
        assert note in out.lines[notes_idx:first_prompt_after]


@pytest.mark.parametrize('choice', ['1', '8', '11'])
def test_no_placement_notes_for_non_calibration_items(clock, tmp_path, choice):
    ser = FakeSerial(clock, interrupt_after=3)    # stops the live view loop
    rc, out = _logged_run(ser, [choice, 'n', '0'], clock, tmp_path)
    assert '注意事項:' not in out.lines


def test_height_zero_notes_mention_pressure_output_disabled():
    text = '\n'.join(cli.NOTES['7'])
    assert '0x56' in text
    assert '/imu/pressure' in text


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


@pytest.mark.parametrize('choice', ['5', '6', '7'])
def test_short_hardware_wait_shows_progress(clock, tmp_path, choice):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, [choice, 'y', '0'], clock, tmp_path)
    assert rc == 0
    assert '100%' in out.text


def _gyro_frame(x_dps, y_dps, z_dps):
    raw = [int(v / 2000.0 * 32768) for v in (x_dps, y_dps, z_dps)]
    return make_frame(0x52, i16(raw[0], raw[1], raw[2], 0))


def _mag_frame(x, y, z):
    return make_frame(0x54, i16(int(x), int(y), int(z), 2500))


FROZEN_GYRO = _gyro_frame(-1.7, -2.6, 4.0)   # CMP10A freezes gyro output in mag-cal mode


def _sphere_stream(radius=1000, center=(60, -40, 20), axes=(0, 1, 2)):
    """Mag frames sweeping +-radius on the given axes; gyro stays frozen like the real IMU."""
    frames = b''
    for axis in axes:
        for k in range(-5, 6):
            v = list(center)
            v[axis] += radius * k / 5
            frames += _mag_frame(*v) + FROZEN_GYRO
    return frames


def _wait_for(out, text, timeout=5.0):
    """Block (real time) until ``text`` appears in the output, like a user watching."""
    import time as _time
    deadline = _time.monotonic() + timeout
    while text not in out.text:
        if _time.monotonic() > deadline:
            raise AssertionError(f'{text!r} never shown; output:\n{out.text[-2000:]}')
        _time.sleep(0.01)


def _run_mag_waiting_for(ser, clock, tmp_path, choice, wait_text):
    out = Output()
    answers = [choice, 'y', None, '0']        # None = press Enter after wait_text appears
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
    assert '尚未' not in out.text


def test_mag_calibration_shows_each_axis_and_what_is_missing(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream(axes=(0,)))     # only X ever changes
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '3', '缺 Y Z')
    assert rc == 0
    last = [ln for ln in _status_lines(out) if '缺' in ln][-1]
    assert 'X [' in last and 'Y [' in last and 'Z [' in last
    assert 'X [########] 100%' in last
    assert '尚未' in out.text                                  # Enter pressed too early
    assert ser.writes[-2:] == [NORMAL, SAVE]


def test_mag_calibration_explains_which_rotation_fixes_each_axis(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out, _ = run(ser, ['3', 'n', '0'], clock, tmp_path)
    assert rc == 0
    assert 'X 不夠' in out.text and '繞 Y 或 Z' in out.text


def test_mag_status_line_fits_80_columns(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream())
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '3', '[完成] 按 Enter')
    widths = [_display_width(ln) for ln in _status_lines(out)]
    assert widths and max(widths) <= 79, max(widths)


def test_mag_calibration_without_data_shows_elapsed_time_hint(clock, tmp_path):
    ser = FakeSerial(clock)                                  # IMU sends nothing
    rc, out = _run_mag_waiting_for(ser, clock, tmp_path, '3', '未收到')
    assert rc == 0
    assert '秒' in out.text
    assert ser.writes[-2:] == [NORMAL, SAVE]


def test_mag_calibration_serial_error_in_reader_is_reported(clock, tmp_path):
    class Broken(FakeSerial):

        def read(self, size=1):
            raise serial.SerialException('read failed')

    ser = Broken(clock)
    rc, out, _ = run(ser, ['3', 'y', '', '0'], clock, tmp_path)
    assert rc == 0
    assert 'read failed' in out.text


def test_software_static_calibration_shows_progress(clock, tmp_path):
    ser = FakeSerial(clock, stream=_static_stream())
    rc, out, _ = run(ser, ['9', '', '0'], clock, tmp_path)
    assert rc == 0
    assert '100%' in out.text


def test_software_mag_calibration_ends_when_every_axis_is_covered(clock, tmp_path):
    path = tmp_path / 'imu_calibration.yaml'
    ser = FakeSerial(clock, stream=_sphere_stream(radius=1000, center=(60, -40, 20)))
    rc, out, _ = run(ser, ['10', '', '0'], clock, tmp_path)
    assert rc == 0
    assert clock.now < cli.SOFT_MAG_TIMEOUT
    assert '[完成]' in out.text
    assert yaml.safe_load(path.read_text())['mag_offset'] == pytest.approx([60.0, -40.0, 20.0])
    assert '不足' not in out.text


def test_software_mag_calibration_warns_with_missing_axes(clock, tmp_path):
    ser = FakeSerial(clock, stream=_sphere_stream(axes=(0, 1)))  # Z never covered
    rc, out, _ = run(ser, ['10', '', '0'], clock, tmp_path)
    assert rc == 0
    assert clock.now >= cli.SOFT_MAG_TIMEOUT
    assert '不足' in out.text and 'Z' in out.text
    assert (tmp_path / 'imu_calibration.yaml').exists()


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


def test_attitude_view_explains_how_to_verify(clock, tmp_path):
    ser = FakeSerial(clock)
    rc, out = _run_with_watcher(ser, clock, tmp_path, ['12', 'f', 'q', '0'])
    assert '90°' in out.text and '轉一圈' in out.text


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


def test_height_zero_notes_point_to_pressure_toggle():
    assert any('13' in line for line in cli.NOTES['7'])


def _coverage_after_dual_plane(target):
    """Coverage after a level turn and an upright turn at 37 deg inclination."""
    field = 5000.0
    horizontal = field * math.cos(math.radians(37.0))
    vertical = field * math.sin(math.radians(37.0))
    cov = cal.MagCoverage(field, target=target)
    for deg in range(0, 360, 5):
        a = math.radians(deg)
        # Level, turning about Z: X and Y sweep the horizontal field.
        cov.add(wp.Mag(horizontal * math.cos(a), horizontal * math.sin(a), -vertical, 25.0))
        # Upright (X up), turning about X: Y and Z sweep the horizontal field.
        cov.add(wp.Mag(vertical, horizontal * math.cos(a), horizontal * math.sin(a), 25.0))
    return cov


def test_dual_plane_target_is_reachable_with_the_documented_motion():
    """
    The two documented turns must be able to finish the dual-plane progress.

    The geomagnetic field is inclined (about 37 deg in Taiwan), so an axis
    that only moves in the horizontal plane sees about 80 % of the full swing.
    """
    sphere = cli.mag_coverage_target(wp.CalMode.MAG_SPHERE)
    dual = cli.mag_coverage_target(wp.CalMode.MAG_DUAL_PLANE)
    assert dual < sphere
    assert not _coverage_after_dual_plane(sphere).complete()
    assert _coverage_after_dual_plane(dual).complete()


def test_dual_plane_instructions_explain_when_to_stop():
    """The on-screen text says the bars may stop short and Enter is fine."""
    text = '\n'.join(cli.DUAL_PLANE_STEPS)
    assert 'Enter' in text
    assert '鐵件' in text or '干擾' in text


# ------------------------------------------------- guided verification (12, Enter)

def _guided(clock, tmp_path, steps, color=False):
    """Run the menu with (stream or None, answer) steps; a stream is set before answering."""
    ser = FakeSerial(clock)
    out = Output()
    items = list(steps)

    def inp(prompt=''):
        out(prompt)
        if not items:
            raise EOFError
        stream, answer = items.pop(0)
        if stream is not None:
            ser.stream = stream
        return answer
    rc = cli.run_menu(ser, inp, out, tmp_path / 'c.yaml',
                      sleep_fn=clock.sleep, time_fn=clock.time, color=color)
    return rc, out, ser


def _level(yaw=0.0, roll=0.1, pitch=-0.2):
    return _angle_frame(roll, pitch, yaw)


GOOD_RUN = [
    (None, '12'), (None, ''),          # open the menu entry, start guided verification
    (_level(), ''),                    # 1 level
    (_level(10.0), ''),                # 2 start direction
    (_level(100.5), ''), (_level(-169.0), ''), (_level(-80.0), ''), (_level(11.0), ''),
    (None, '0'),
]


def test_guided_verification_all_pass_without_height(clock, tmp_path):
    rc, out, ser = _guided(clock, tmp_path, GOOD_RUN)
    assert rc == 0
    assert ser.writes == []                                   # read-only
    assert '[合格] 1. 水平' in out.text
    assert '[合格] 2. 每轉 90°' in out.text
    assert '[合格] 3. 轉一圈回原位' in out.text
    assert '[略過] 4. 高度' in out.text and '13' in out.text
    assert '不合格' not in out.text
    # Each of the four turns is reported with its measured change.
    assert out.text.count('第 ') >= 4 and '+90.50°' in out.text


def test_guided_verification_reports_failures(clock, tmp_path):
    steps = [
        (None, '12'), (None, ''),
        (_level(roll=1.5), ''),                               # tilted: level fails
        (_level(0.0), ''),
        (_level(80.0), ''), (_level(-180.0), ''), (_level(-95.0), ''), (_level(8.0), ''),
        (None, '0'),
    ]
    rc, out, _ = _guided(clock, tmp_path, steps)
    assert '[不合格] 1. 水平' in out.text
    assert '[不合格] 2. 每轉 90°' in out.text                  # 80, 100, 85, 103
    assert '[不合格] 3. 轉一圈回原位' in out.text               # ends 8 deg off
    assert '全部合格' not in out.text


def test_guided_verification_height_step(clock, tmp_path):
    def at(height_cm, yaw=0.0):
        return _level(yaw) + _pressure_frame(101325, height_cm)
    steps = [
        (None, '12'), (None, ''),
        (at(0), ''),
        (None, 's'),                                          # skip the 90 deg turns
        (None, 's'),                                          # skip the full turn
        (at(0), ''), (at(95), ''),                            # height: +0.95 m
        (None, '0'),
    ]
    rc, out, _ = _guided(clock, tmp_path, steps)
    assert '[略過] 2. 每轉 90°' in out.text
    assert '[略過] 3. 轉一圈回原位' in out.text
    assert '[合格] 4. 高度' in out.text and '+0.95 m' in out.text


def test_guided_verification_can_quit_early(clock, tmp_path):
    steps = [(None, '12'), (None, ''), (_level(), ''), (None, 'q'), (None, '0')]
    rc, out, _ = _guided(clock, tmp_path, steps)
    assert rc == 0
    assert '[合格] 1. 水平' in out.text
    assert '已結束驗證' in out.text


def test_guided_verification_without_data_does_not_pass(clock, tmp_path):
    steps = [(None, '12'), (None, ''), (None, ''), (None, 'q'), (None, '0')]
    rc, out, _ = _guided(clock, tmp_path, steps)
    assert '[合格]' not in out.text
    assert '未收到角度資料' in out.text


def test_guided_verification_colours_pass_green_and_fail_red(clock, tmp_path):
    rc, out, _ = _guided(clock, tmp_path, GOOD_RUN, color=True)
    assert '\033[32m[合格] 1. 水平' in out.text
    steps = [(None, '12'), (None, ''), (_level(roll=2.0), ''), (None, 'q'), (None, '0')]
    rc, out, _ = _guided(clock, tmp_path, steps, color=True)
    assert '\033[31m[不合格] 1. 水平' in out.text


def test_guided_verification_has_no_colour_codes_by_default(clock, tmp_path):
    rc, out, _ = _guided(clock, tmp_path, GOOD_RUN)
    assert '\033[' not in out.text


def test_guided_verification_serial_error_is_reported(clock, tmp_path):
    class Broken(FakeSerial):

        def read(self, size=1):
            raise serial.SerialException('read failed')

    ser = Broken(clock)
    rc, out, _ = run(ser, ['12', '', '', '0'], clock, tmp_path)
    assert rc == 0
    assert 'read failed' in out.text
    assert '[合格]' not in out.text


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


def test_guided_verification_redraws_the_screen_on_a_terminal(clock, tmp_path):
    """On a terminal each step clears the screen so old lines do not pile up."""
    rc, out, _ = _guided(clock, tmp_path, GOOD_RUN, color=True)
    assert out.text.count(cli.CLEAR_SCREEN) >= 6
    # After the last redraw only the final table remains: each result once.
    tail = out.text.rsplit(cli.CLEAR_SCREEN, 1)[1]
    assert tail.count('1. 水平') == 1
    assert tail.count('2. 每轉 90°') == 1
    assert '全部合格' not in tail or tail.count('全部合格') == 1


def test_guided_verification_does_not_clear_the_screen_without_a_terminal(clock, tmp_path):
    rc, out, _ = _guided(clock, tmp_path, GOOD_RUN)
    assert cli.CLEAR_SCREEN not in out.text


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


def test_passed_guided_verification_marks_option_12(clock, tmp_path):
    rc, out, _ = _guided(clock, tmp_path, GOOD_RUN, color=True)
    assert cli.DONE_MARK in _menu_row(_menu_blocks(out)[-1], 12)


def test_failed_guided_verification_does_not_mark_option_12(clock, tmp_path):
    steps = [(None, '12'), (None, ''), (_level(roll=2.0), ''), (None, 'q'), (None, '0')]
    rc, out, _ = _guided(clock, tmp_path, steps, color=True)
    assert cli.DONE_MARK not in _menu_blocks(out)[-1]
