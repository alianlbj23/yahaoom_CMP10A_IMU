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

"""Tests for the host-side interactive launcher ``imu.py``."""

import os
import subprocess

import imu

import pytest

REPO = '/repo'


# ---------------------------------------------------------------- helpers
def make_input(answers):
    """Return an input() replacement that replays ``answers``."""
    remaining = list(answers)

    def fake_input(prompt=''):
        if not remaining:
            raise EOFError
        return remaining.pop(0)

    return fake_input


class FakeDocker:
    """Record docker invocations made through subprocess.run."""

    def __init__(self, daemon_ok=True, image=True, running=False,
                 run_rc=0, missing=False):
        """Configure the simulated docker state."""
        self.calls = []
        self.daemon_ok = daemon_ok
        self.image = image
        self.running = running
        self.run_rc = run_rc
        self.pull_rc = 0
        self.missing = missing

    def __call__(self, args, **kwargs):
        """Emulate subprocess.run for the docker CLI."""
        self.calls.append((list(args), kwargs))
        assert isinstance(args, list)
        assert not kwargs.get('shell', False)
        if self.missing:
            raise FileNotFoundError(2, 'No such file', 'docker')
        sub = args[1]
        if sub == 'run':
            return subprocess.CompletedProcess(args, self.run_rc)
        assert kwargs.get('timeout'), 'non-interactive docker needs timeout'
        if sub == 'info':
            rc = 0 if self.daemon_ok else 1
            return subprocess.CompletedProcess(args, rc, '27.0.0\n', 'err')
        if sub == 'image':
            return subprocess.CompletedProcess(
                args, 0 if self.image else 1, '', '')
        if sub == 'ps':
            out = imu.CONTAINER_NAME + '\n' if self.running else ''
            return subprocess.CompletedProcess(args, 0, out, '')
        if sub == 'stop':
            self.running = False
            return subprocess.CompletedProcess(args, 0, '', '')
        if sub == 'pull':
            return subprocess.CompletedProcess(args, self.pull_rc)
        raise AssertionError('unexpected docker call: %r' % (args,))

    def subcommands(self):
        """Return the docker subcommands in call order."""
        return [c[0][1] for c in self.calls]

    def run_call(self):
        """Return the recorded ``docker run`` invocation."""
        runs = [c for c in self.calls if c[0][1] == 'run']
        assert len(runs) == 1
        return runs[0]


class FakePopen:
    """Stand-in for the background serial bridge process."""

    instances = []

    def __init__(self, args, **kwargs):
        """Record the command line."""
        self.args = args
        self.kwargs = kwargs
        self.terminated = False
        FakePopen.instances.append(self)

    def poll(self):
        """Report the bridge as still running until terminated."""
        return 0 if self.terminated else None

    def terminate(self):
        """Mark the process as terminated."""
        self.terminated = True

    def wait(self, timeout=None):
        """Return immediately."""
        return 0

    def kill(self):
        """Mark the process as killed."""
        self.terminated = True


def linux_port():
    """Return the serial port seen on the development host."""
    return [imu.SerialPort(
        '/dev/ttyUSB0',
        ('/dev/serial/by-id/usb-Silicon_Labs_CP2102-if00-port0',))]


def make_launcher(platform='linux', answers=(), env=None, ports=None,
                  access=True, pyserial=True, tmp_path=None):
    """Build a Launcher with all host interactions faked."""
    repo = str(tmp_path) if tmp_path is not None else REPO
    return imu.Launcher(
        platform=platform,
        repo_dir=repo,
        env={'DISPLAY': ':0'} if env is None else env,
        input_func=make_input(answers),
        port_lister=lambda plat: list(linux_port() if ports is None
                                      else ports),
        access_func=lambda path, mode: access,
        isatty=lambda: True,
        pyserial_available=lambda: pyserial,
    )


@pytest.fixture
def docker(monkeypatch):
    """Patch subprocess.run inside imu with a FakeDocker."""
    fake = FakeDocker()
    monkeypatch.setattr(imu.subprocess, 'run', fake)
    monkeypatch.setattr(imu.time, 'sleep', lambda s: None)
    FakePopen.instances = []
    monkeypatch.setattr(imu.subprocess, 'Popen', FakePopen)
    return fake


# ------------------------------------------------------ platform detection
def test_detect_platform_mac():
    """Detect macOS from sys.platform."""
    assert imu.detect_platform('darwin', '/nonexistent') == 'mac'


def test_detect_platform_wsl(tmp_path):
    """Detect WSL from /proc/version containing 'microsoft'."""
    proc = tmp_path / 'version'
    proc.write_text('Linux version 6.18.33.2-microsoft-standard-WSL2 (gcc)')
    assert imu.detect_platform('linux', str(proc)) == 'wsl'


def test_detect_platform_wsl1_uppercase(tmp_path):
    """Detect WSL1 which reports 'Microsoft' capitalised."""
    proc = tmp_path / 'version'
    proc.write_text('Linux version 4.4.0-19041-Microsoft')
    assert imu.detect_platform('linux', str(proc)) == 'wsl'


def test_detect_platform_plain_linux(tmp_path):
    """Detect native Linux."""
    proc = tmp_path / 'version'
    proc.write_text('Linux version 6.8.0-45-generic (buildd@lcy02)')
    assert imu.detect_platform('linux', str(proc)) == 'linux'


def test_detect_platform_linux_without_proc_version(tmp_path):
    """Fall back to linux when /proc/version is unreadable."""
    assert imu.detect_platform('linux', str(tmp_path / 'missing')) == 'linux'


def test_detect_platform_unsupported():
    """Reject native Windows and other platforms."""
    with pytest.raises(imu.LauncherError):
        imu.detect_platform('win32', '/nonexistent')


# ------------------------------------------------------ serial port listing
def test_list_serial_ports_linux_resolves_and_dedupes():
    """Resolve by-id symlinks and merge them with the device."""
    by_id = '/dev/serial/by-id/usb-Silicon_Labs_CP2102-if00-port0'
    tree = {
        '/dev/serial/by-id/*': [by_id],
        '/dev/ttyUSB*': ['/dev/ttyUSB0'],
        '/dev/ttyACM*': ['/dev/ttyACM0'],
    }
    links = {by_id: '/dev/ttyUSB0'}
    ports = imu.list_serial_ports(
        'linux', glob_func=lambda pattern: tree.get(pattern, []),
        realpath_func=lambda p: links.get(p, p))
    assert [p.device for p in ports] == ['/dev/ttyUSB0', '/dev/ttyACM0']
    assert set(ports[0].aliases) == {by_id}
    assert ports[1].aliases == ()
    assert 'by-id' in ports[0].label


def test_list_serial_ports_mac():
    """List the macOS call-out devices of common USB-UART chips."""
    tree = {
        '/dev/cu.usbserial*': ['/dev/cu.usbserial-0001'],
        '/dev/cu.SLAB*': ['/dev/cu.SLAB_USBtoUART'],
        '/dev/cu.wchusbserial*': [],
    }
    ports = imu.list_serial_ports(
        'mac', glob_func=lambda pattern: tree.get(pattern, []),
        realpath_func=lambda p: p)
    assert [p.device for p in ports] == [
        '/dev/cu.usbserial-0001', '/dev/cu.SLAB_USBtoUART']


def test_list_serial_ports_none():
    """Return an empty list when nothing is plugged in."""
    assert imu.list_serial_ports('wsl', glob_func=lambda p: []) == []


# --------------------------------------------------------- docker run args
def test_build_run_args_linux_foxglove():
    """Build the exact docker run command line for Linux + Foxglove."""
    args = imu.build_run_args('linux', '/dev/ttyUSB0', 'foxglove', False,
                              REPO, {'DISPLAY': ':0'})
    assert args == [
        'docker', 'run', '--rm', '-it', '--name', 'yahboom-imu',
        '-p', '127.0.0.1:9090:9090', '-p', '127.0.0.1:8765:8765',
        '-v', '/repo/config:/config',
        '-v', '/repo/wit_ros2_imu:/ws/src/wit_ros2_imu:ro',
        '--device', '/dev/ttyUSB0:/dev/ttyUSB0',
        'ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy',
        'ros2', 'launch', 'wit_ros2_imu', 'imu.launch.py',
        'port:=/dev/ttyUSB0', 'rviz:=false',
    ]


def test_build_run_args_linux_rviz_has_x11():
    """Pass DISPLAY and the X11 socket for RViz on Linux."""
    args = imu.build_run_args('linux', '/dev/ttyACM1', 'rviz', False,
                              REPO, {'DISPLAY': ':1'})
    assert _has_pair(args, '-e', 'DISPLAY')
    assert _has_pair(args, '-v', '/tmp/.X11-unix:/tmp/.X11-unix')
    assert _has_pair(args, '--device', '/dev/ttyACM1:/dev/ttyACM1')
    assert args[-2:] == ['port:=/dev/ttyACM1', 'rviz:=true']


def test_build_run_args_linux_rviz_without_display_has_no_x11():
    """Do not pass X11 options when DISPLAY is not set."""
    args = imu.build_run_args('linux', '/dev/ttyUSB0', 'rviz', False,
                              REPO, {})
    assert '-e' not in args
    assert '/tmp/.X11-unix:/tmp/.X11-unix' not in args


def test_build_run_args_wsl_rviz_uses_wslg():
    """Use the WSLg sockets and only forward variables that exist."""
    env = {'DISPLAY': ':0', 'WAYLAND_DISPLAY': 'wayland-0'}
    args = imu.build_run_args('wsl', '/dev/ttyUSB0', 'rviz', False, REPO,
                              env)
    assert _has_pair(args, '-v', '/tmp/.X11-unix:/tmp/.X11-unix')
    assert _has_pair(args, '-v', '/mnt/wslg:/mnt/wslg')
    assert _has_pair(args, '-e', 'DISPLAY')
    assert _has_pair(args, '-e', 'WAYLAND_DISPLAY')
    assert _has_pair(args, '-e', 'XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir')
    assert not _has_pair(args, '-e', 'PULSE_SERVER')
    assert _has_pair(args, '--device', '/dev/ttyUSB0:/dev/ttyUSB0')


def test_build_run_args_wsl_forwards_pulse_server_when_set():
    """Forward PULSE_SERVER when WSLg exports it."""
    env = {'DISPLAY': ':0', 'PULSE_SERVER': 'unix:/mnt/wslg/PulseServer'}
    args = imu.build_run_args('wsl', '/dev/ttyUSB0', 'rviz', False, REPO,
                              env)
    assert _has_pair(args, '-e', 'PULSE_SERVER')
    assert not _has_pair(args, '-e', 'WAYLAND_DISPLAY')


def test_build_run_args_mac_rviz_uses_tcp_bridge_and_xquartz():
    """Use the TCP bridge URL instead of --device on macOS."""
    args = imu.build_run_args('mac', '/dev/cu.usbserial-0001', 'rviz',
                              False, '/Users/me/imu', {})
    assert '--device' not in args
    assert _has_pair(args, '-e', 'DISPLAY=host.docker.internal:0')
    assert _has_pair(args, '-v', '/Users/me/imu/config:/config')
    assert args[-2:] == ['port:=socket://host.docker.internal:5760',
                         'rviz:=true']


def test_build_run_args_lan_exposure_drops_loopback_binding():
    """Publish on all interfaces when the user opts into LAN access."""
    args = imu.build_run_args('linux', '/dev/ttyUSB0', 'foxglove', True,
                              REPO, {})
    assert _has_pair(args, '-p', '9090:9090')
    assert _has_pair(args, '-p', '8765:8765')
    assert not any(a.startswith('127.0.0.1') for a in args)


def test_build_run_args_calibrate():
    """Run the calibration CLI with the mounted YAML path."""
    args = imu.build_run_args('wsl', '/dev/ttyUSB0', 'calibrate', False,
                              REPO, {'DISPLAY': ':0'})
    image_at = args.index('ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy')
    assert args[image_at + 1:] == [
        'ros2', 'run', 'wit_ros2_imu', 'imu_calibrate',
        '--port', '/dev/ttyUSB0', '--baud', '9600',
        '--config', '/config/imu_calibration.yaml',
    ]
    assert '/mnt/wslg:/mnt/wslg' not in args


def test_build_run_args_mac_calibrate_uses_socket_url():
    """Point the calibration CLI at the TCP bridge on macOS."""
    args = imu.build_run_args('mac', '/dev/cu.SLAB_USBtoUART', 'calibrate',
                              False, REPO, {})
    assert _has_pair(args, '--port', 'socket://host.docker.internal:5760')


def test_build_run_args_shell_without_port():
    """Allow entering a shell without a serial device."""
    args = imu.build_run_args('linux', None, 'shell', False, REPO, {})
    assert '--device' not in args
    assert args[-2:] == ['ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy', 'bash']


@pytest.mark.parametrize('platform,mode,port', [
    ('linux', 'teleport', '/dev/ttyUSB0'),
    ('windows', 'foxglove', '/dev/ttyUSB0'),
    ('linux', 'foxglove', None),
    ('linux', 'calibrate', ''),
    ('linux', 'foxglove', 'ttyUSB0'),
])
def test_build_run_args_rejects_invalid_input(platform, mode, port):
    """Reject unknown platforms/modes and missing or bogus ports."""
    with pytest.raises(imu.LauncherError):
        imu.build_run_args(platform, port, mode, False, REPO, {})


def _has_pair(args, flag, value):
    return any(args[i] == flag and args[i + 1] == value
               for i in range(len(args) - 1))


# ----------------------------------------------------- permission handling
def test_check_port_access_denied_explains_fix():
    """Point to usb_udev_namer / dialout when the port is not accessible."""
    with pytest.raises(imu.LauncherError) as excinfo:
        imu.check_port_access('/dev/ttyUSB0',
                              access_func=lambda p, m: False)
    message = str(excinfo.value)
    assert 'usb_udev_namer' in message
    assert '/dev/usb_imu' in message
    assert 'dialout' in message
    assert 'imu_usb.rules' not in message


def test_check_port_access_ok():
    """Accept a readable and writable port."""
    seen = []
    imu.check_port_access(
        '/dev/ttyUSB0',
        access_func=lambda p, m: seen.append((p, m)) or True)
    assert seen == [('/dev/ttyUSB0', os.R_OK | os.W_OK)]


# ---------------------------------------------------------- menu handling
def test_menu_rejects_invalid_choices_then_exits(docker, capsys):
    """Keep asking on out-of-range or non-numeric input."""
    launcher = make_launcher(answers=['9', 'abc', '-1', '', '\u00b2', '0'])
    assert launcher.run_menu() == 0
    out = capsys.readouterr().out
    assert out.count('無效') == 5
    assert docker.calls == []


def test_menu_eof_returns_error(docker):
    """Treat end of input before choosing 0 as abnormal termination."""
    launcher = make_launcher(answers=[])
    assert launcher.run_menu() == 1


def test_menu_reports_failure_exit_code(docker, capsys):
    """Return non-zero when an action failed during the session."""
    launcher = make_launcher(answers=['2', '0'], ports=[])
    assert launcher.run_menu() == 1
    assert '錯誤' in capsys.readouterr().err


def test_ask_choice_validates_range():
    """Only accept numbers inside the menu range."""
    launcher = make_launcher(answers=['3', '7', '2'])
    assert launcher.ask_choice('選擇', 1, 2) == 2


# ------------------------------------------------------- start: error paths
def test_start_wsl_without_ports_prints_usbipd_steps(docker, capsys):
    """Guide WSL users to wsl_usb_auto when no device is attached."""
    launcher = make_launcher(platform='wsl', ports=[])
    with pytest.raises(imu.LauncherError):
        launcher.start('foxglove')
    out = capsys.readouterr().out
    assert 'winget install usbipd' in out
    assert 'wsl_usb_auto' in out
    assert 'usb_udev_namer' in out
    assert 'run' not in docker.subcommands()


def test_start_linux_without_ports_has_no_usbipd(docker, capsys):
    """Do not mention usbipd on native Linux."""
    launcher = make_launcher(platform='linux', ports=[])
    with pytest.raises(imu.LauncherError):
        launcher.start('rviz')
    assert 'usbipd' not in capsys.readouterr().out


def test_start_without_permission_aborts(docker):
    """Abort before docker run when the port is not accessible."""
    launcher = make_launcher(answers=['1'], access=False)
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.start('foxglove')
    assert 'dialout' in str(excinfo.value)
    assert 'run' not in docker.subcommands()


def test_start_requires_built_image(docker):
    """Ask the user to download the image first."""
    docker.image = False
    launcher = make_launcher(answers=['1', 'n'])
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.start('foxglove')
    assert '下載' in str(excinfo.value)


def test_start_requires_docker_daemon(docker):
    """Fail clearly when docker info fails."""
    docker.daemon_ok = False
    launcher = make_launcher(answers=['1', 'n'])
    with pytest.raises(imu.LauncherError):
        launcher.start('foxglove')
    assert docker.subcommands() == ['info']


def test_start_docker_missing(docker):
    """Fail clearly when the docker CLI is not installed."""
    docker.missing = True
    launcher = make_launcher(answers=['1', 'n'])
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.start('foxglove')
    assert 'docker' in str(excinfo.value)


def test_start_rviz_without_display_suggests_foxglove(docker, capsys):
    """Warn about missing DISPLAY and let the user back out."""
    launcher = make_launcher(answers=['1', 'n'], env={})
    assert launcher.start('rviz') is False
    assert 'Foxglove' in capsys.readouterr().out
    assert 'run' not in docker.subcommands()


def test_start_requires_tty(docker):
    """Refuse docker run -it without an interactive terminal."""
    launcher = make_launcher(answers=['1', 'n'])
    launcher.isatty = lambda: False
    with pytest.raises(imu.LauncherError):
        launcher.start('foxglove')
    assert 'run' not in docker.subcommands()


def test_start_reports_docker_run_failure(docker, tmp_path):
    """Raise when docker itself fails (exit code 125, e.g. port in use)."""
    docker.run_rc = 125
    launcher = make_launcher(answers=['1', 'n'], tmp_path=tmp_path)
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.start('foxglove')
    assert '125' in str(excinfo.value)


def test_start_keeps_running_container_when_user_declines(docker):
    """Do not touch a running container unless the user agrees."""
    docker.running = True
    launcher = make_launcher(answers=['1', 'n', 'n'])
    assert launcher.start('foxglove') is False
    assert 'stop' not in docker.subcommands()
    assert 'run' not in docker.subcommands()


# ------------------------------------------------------- start: happy paths
def test_start_linux_foxglove_runs_expected_command(
        docker, capsys, tmp_path):
    """Run docker with the pure-function arguments and print endpoints."""
    launcher = make_launcher(answers=['1', 'n'], tmp_path=tmp_path)
    assert launcher.start('foxglove') is True
    args, kwargs = docker.run_call()
    assert args == imu.build_run_args('linux', '/dev/ttyUSB0', 'foxglove',
                                      False, str(tmp_path),
                                      {'DISPLAY': ':0'})
    assert (tmp_path / 'config').is_dir()
    assert kwargs.get('check') is False
    out = capsys.readouterr().out
    assert 'ws://localhost:8765' in out
    assert 'ws://localhost:9090' in out
    assert '/imu/data' in out


def test_start_lan_answer_yes_publishes_on_all_interfaces(docker, tmp_path):
    """Translate the LAN question into 0.0.0.0 port bindings."""
    launcher = make_launcher(answers=['1', 'y'], tmp_path=tmp_path)
    launcher.start('foxglove')
    args, _ = docker.run_call()
    assert _has_pair(args, '-p', '9090:9090')


def test_start_stops_running_container_when_confirmed(docker, tmp_path):
    """Stop the old container before starting a new one."""
    docker.running = True
    launcher = make_launcher(answers=['1', 'n', 'y'], tmp_path=tmp_path)
    assert launcher.start('foxglove') is True
    subs = docker.subcommands()
    assert subs.index('stop') < subs.index('run')


def test_start_calibrate_skips_lan_question(docker, tmp_path):
    """Calibration needs no bridges, so only the port is asked."""
    launcher = make_launcher(answers=['1'], tmp_path=tmp_path)
    assert launcher.start('calibrate') is True
    args, _ = docker.run_call()
    assert 'imu_calibrate' in args


def test_start_shell_without_ports_continues(docker, tmp_path):
    """Allow a container shell even without a serial device."""
    launcher = make_launcher(answers=['n'], ports=[], tmp_path=tmp_path)
    assert launcher.start('shell') is True
    args, _ = docker.run_call()
    assert '--device' not in args
    assert args[-1] == 'bash'


def test_start_mac_requires_pyserial(docker, capsys):
    """Tell mac users how to install pyserial instead of installing it."""
    ports = [imu.SerialPort('/dev/cu.usbserial-0001', ())]
    launcher = make_launcher(platform='mac', answers=['1', 'n'],
                             ports=ports, pyserial=False)
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.start('foxglove')
    assert 'python3 -m pip install pyserial==3.5' in str(excinfo.value)
    assert FakePopen.instances == []
    assert 'run' not in docker.subcommands()


def test_start_mac_runs_bridge_around_container(docker, tmp_path):
    """Start the TCP bridge before docker run and stop it afterwards."""
    ports = [imu.SerialPort('/dev/cu.usbserial-0001', ())]
    launcher = make_launcher(platform='mac', answers=['1', 'n'],
                             ports=ports, tmp_path=tmp_path)
    assert launcher.start('foxglove') is True
    assert len(FakePopen.instances) == 1
    bridge = FakePopen.instances[0]
    assert bridge.args[0] == imu.sys.executable
    assert bridge.args[1] == os.path.join(str(tmp_path), 'host',
                                          'serial_tcp_bridge.py')
    assert '/dev/cu.usbserial-0001' in bridge.args
    assert '5760' in bridge.args
    assert bridge.terminated
    args, _ = docker.run_call()
    assert 'port:=socket://host.docker.internal:5760' in args


def test_start_mac_stops_bridge_on_ctrl_c(docker, monkeypatch, tmp_path):
    """Terminate the bridge and the container after Ctrl+C."""
    ports = [imu.SerialPort('/dev/cu.usbserial-0001', ())]
    launcher = make_launcher(platform='mac', answers=['1', 'n'],
                             ports=ports, tmp_path=tmp_path)

    def interrupted_run(args, **kwargs):
        if args[1] == 'run':
            docker.calls.append((list(args), kwargs))
            docker.running = True
            raise KeyboardInterrupt
        return docker(args, **kwargs)

    monkeypatch.setattr(imu.subprocess, 'run', interrupted_run)
    assert launcher.start('foxglove') is True
    assert FakePopen.instances[0].terminated
    assert docker.subcommands()[-1] == 'stop'


# ----------------------------------------------------- other menu actions
def test_image_is_prebuilt_on_ghcr():
    """The launcher uses the image built by GitHub Actions, not a local one."""
    assert imu.IMAGE == 'ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy'


def test_pull_image_runs_docker_pull(docker, capsys):
    """Download the prebuilt image with a long timeout; never build locally."""
    launcher = make_launcher()
    launcher.pull_image()
    args, kwargs = [c for c in docker.calls if c[0][1] == 'pull'][0]
    assert args == ['docker', 'pull', imu.IMAGE]
    assert kwargs['timeout'] >= 3600
    assert 'build' not in docker.subcommands()


def test_pull_image_failure_is_reported(docker):
    """A failed pull raises a clear error naming the image."""
    docker.pull_rc = 1
    launcher = make_launcher()
    with pytest.raises(imu.LauncherError) as excinfo:
        launcher.pull_image()
    assert imu.IMAGE in str(excinfo.value)


def test_pull_image_requires_docker_daemon(docker):
    """Do not try to pull when the docker daemon is down."""
    docker.daemon_ok = False
    with pytest.raises(imu.LauncherError):
        make_launcher().pull_image()
    assert 'pull' not in docker.subcommands()


def test_stop_container_when_not_running(docker, capsys):
    """Report that nothing is running."""
    make_launcher().stop_container()
    assert 'stop' not in docker.subcommands()


def test_stop_container_when_running(docker):
    """Stop the running container."""
    docker.running = True
    make_launcher().stop_container()
    assert 'stop' in docker.subcommands()


def test_check_environment_reports(docker, capsys):
    """Print platform, docker, image, port and display status."""
    docker.image = False
    problems = make_launcher(platform='wsl').check_environment()
    out = capsys.readouterr().out
    assert 'wsl' in out
    assert '/dev/ttyUSB0' in out
    assert 'DISPLAY' in out
    assert any('映像' in p for p in problems)


def test_check_environment_without_docker(docker, capsys):
    """Report a missing docker CLI as a problem instead of crashing."""
    docker.missing = True
    problems = make_launcher(ports=[]).check_environment()
    assert any('docker' in p for p in problems)
    assert any('序列埠' in p for p in problems)


def test_list_serial_ports_prefers_usb_imu_name():
    """A /dev/usb_imu symlink is listed first and used as the selected path."""
    tree = {
        '/dev/usb_imu': ['/dev/usb_imu'],
        '/dev/ttyUSB*': ['/dev/ttyUSB0', '/dev/ttyUSB1'],
    }
    links = {'/dev/usb_imu': '/dev/ttyUSB1'}
    ports = imu.list_serial_ports(
        'linux', glob_func=lambda pattern: tree.get(pattern, []),
        realpath_func=lambda path: links.get(path, path))
    assert [p.device for p in ports] == ['/dev/ttyUSB1', '/dev/ttyUSB0']
    assert ports[0].selected == '/dev/usb_imu'
    assert ports[0].label == '/dev/usb_imu'
    assert ports[1].selected == '/dev/ttyUSB0'


def test_serial_port_without_usb_imu_selects_device():
    """Without the fixed name the resolved device is used, as before."""
    port = imu.SerialPort('/dev/ttyUSB0', ('/dev/serial/by-id/x',))
    assert port.selected == '/dev/ttyUSB0'


def test_build_run_args_maps_device_to_fixed_name():
    """Map the resolved host device to the fixed name in the container."""
    args = imu.build_run_args(
        'linux', '/dev/usb_imu', 'foxglove', False, '/repo', {},
        realpath_func=lambda path: '/dev/ttyUSB1')
    assert '/dev/ttyUSB1:/dev/usb_imu' in args
    assert 'port:=/dev/usb_imu' in args


def test_start_passes_usb_imu_name_into_container(docker, tmp_path,
                                                  monkeypatch, capsys):
    """Choosing the usb_imu port makes the driver use /dev/usb_imu."""
    monkeypatch.setattr(imu.os.path, 'realpath', lambda path: '/dev/ttyUSB0'
                        if path == '/dev/usb_imu' else path)
    ports = [imu.SerialPort('/dev/ttyUSB1'),
             imu.SerialPort('/dev/ttyUSB0', ('/dev/usb_imu',))]
    # Only the LAN question is asked: the fixed name is used without a menu.
    launcher = make_launcher(answers=['n'], ports=ports, tmp_path=tmp_path)
    launcher.start('foxglove')
    out = capsys.readouterr().out
    assert '使用 /dev/usb_imu' in out
    assert '可用序列埠' not in out
    assert '/dev/ttyUSB1' not in out
    args = docker.run_call()[0]
    assert '/dev/ttyUSB0:/dev/usb_imu' in args
    assert 'port:=/dev/usb_imu' in args


def test_no_port_message_points_to_naming_tool():
    """The Linux hint names the fixed device name and the naming tool."""
    message = imu.no_port_message('linux')
    assert '/dev/usb_imu' in message
    assert 'usb_udev_namer' in message


@pytest.mark.parametrize('mode', ['foxglove', 'rviz', 'calibrate', 'shell'])
def test_build_run_args_mounts_the_source_read_only(mode):
    """
    The image only provides the environment; the code comes from the repo.

    Every mode mounts wit_ros2_imu/ so that source edits take effect on the
    next start without rebuilding the image.
    """
    args = imu.build_run_args('linux', '/dev/ttyUSB0', mode, False,
                              '/repo', {})
    assert _has_pair(args, '-v', '/repo/wit_ros2_imu:/ws/src/wit_ros2_imu:ro')
