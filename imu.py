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

"""Interactive host-side launcher for the Yahboom CMP10A IMU Docker image.

Usage: ``python3 imu.py`` and pick a number from the menu.

Standard library only (Python >= 3.8); supports Linux, WSL2 and macOS.
All docker commands are executed with ``subprocess.run([...])`` (list form,
never ``shell=True``).
"""

from __future__ import annotations

import glob
import importlib
import os
import platform as platform_mod
import socket
import subprocess
import sys
import tempfile
import time
from typing import Callable, Dict, List, Mapping, NamedTuple, Optional
from typing import Sequence, Tuple

# Built and published by .github/workflows/docker-image.yml; never built
# locally by this launcher.
IMAGE = 'ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy'
CONTAINER_NAME = 'yahboom-imu'
ROSBRIDGE_PORT = 9090
FOXGLOVE_PORT = 8765
BRIDGE_PORT = 5760
BAUD = 9600
CONTAINER_CONFIG = '/config'
CALIBRATION_FILE = CONTAINER_CONFIG + '/imu_calibration.yaml'
MAC_SERIAL_URL = 'socket://host.docker.internal:%d' % BRIDGE_PORT

PLATFORMS = ('linux', 'wsl', 'mac')
MODES = ('foxglove', 'rviz', 'calibrate', 'shell')
GUI_MODES = ('rviz', 'shell')
PUBLISH_MODES = ('foxglove', 'rviz', 'shell')

PULL_TIMEOUT = 3600           # docker pull downloads 2-3 GB
DOCKER_TIMEOUT = 30           # docker info / inspect / ps
STOP_TIMEOUT = 60             # docker stop (10 s grace + margin)
SESSION_TIMEOUT = 24 * 3600   # foreground session; normally ended by Ctrl+C
BRIDGE_STARTUP_WAIT = 1.0
BRIDGE_STOP_TIMEOUT = 5

# Fixed device name created with usb_udev_namer (see tutorial unit 0).
PREFERRED_PORT = '/dev/usb_imu'
NAMER_URL = 'https://github.com/alianlbj23/usb_udev_namer'
WSL_ATTACH_URL = 'https://github.com/alianlbj23/wsl_usb_auto'
LINUX_PORT_GLOBS = (PREFERRED_PORT, '/dev/serial/by-id/*', '/dev/ttyUSB*',
                    '/dev/ttyACM*')
MAC_PORT_GLOBS = ('/dev/cu.usbserial*', '/dev/cu.SLAB*',
                  '/dev/cu.wchusbserial*')

PYSERIAL_HINT = 'python3 -m pip install pyserial==3.5'

MENU = (
    ('1', '下載 Docker 映像 (%s)' % IMAGE),
    ('2', '啟動 IMU + Foxglove bridges (rosbridge 9090 / foxglove 8765)'),
    ('3', '啟動 IMU + RViz2 (+ bridges)'),
    ('4', 'IMU 校正工具'),
    ('5', '進入容器 shell'),
    ('6', '停止容器'),
    ('7', '環境檢查'),
    ('0', '離開'),
)


class LauncherError(Exception):
    """A user-facing error; the message explains how to fix it."""


class SerialPort(NamedTuple):
    """A host serial device plus the symlinks that point to it."""

    device: str
    aliases: Tuple[str, ...] = ()

    @property
    def selected(self) -> str:
        """Return the path to use: the fixed name if present."""
        if PREFERRED_PORT in self.aliases:
            return PREFERRED_PORT
        return self.device

    @property
    def label(self) -> str:
        """Return a human readable description."""
        if self.selected != self.device:
            return self.selected
        head = self.device
        others = self.aliases
        if not others:
            return head
        names = []
        for alias in others:
            if alias.startswith('/dev/serial/by-id/'):
                names.append('by-id: ' + os.path.basename(alias))
            else:
                names.append(alias)
        return '%s (%s)' % (head, ', '.join(names))


# ---------------------------------------------------------------- platform
def detect_platform(sys_platform: Optional[str] = None,
                    proc_version_path: str = '/proc/version') -> str:
    """Return 'linux', 'wsl' or 'mac'; raise LauncherError otherwise."""
    name = sys.platform if sys_platform is None else sys_platform
    if name == 'darwin':
        return 'mac'
    if name.startswith('linux'):
        try:
            with open(proc_version_path, encoding='utf-8',
                      errors='replace') as handle:
                version = handle.read()
        except OSError:
            return 'linux'
        return 'wsl' if 'microsoft' in version.lower() else 'linux'
    raise LauncherError(
        '不支援的平台 %r。Windows 請在 WSL2 內執行 python3 imu.py。' % name)


# ------------------------------------------------------------ serial ports
def list_serial_ports(platform: str,
                      glob_func: Callable[[str], List[str]] = glob.glob,
                      realpath_func: Callable[[str], str] = os.path.realpath
                      ) -> List[SerialPort]:
    """List candidate IMU serial ports, resolving symlinks to devices."""
    patterns = MAC_PORT_GLOBS if platform == 'mac' else LINUX_PORT_GLOBS
    order: List[str] = []
    aliases: Dict[str, List[str]] = {}
    for pattern in patterns:
        for path in sorted(glob_func(pattern)):
            device = realpath_func(path)
            if device not in aliases:
                order.append(device)
                aliases[device] = []
            if path != device and path not in aliases[device]:
                aliases[device].append(path)
    return [SerialPort(dev, tuple(aliases[dev])) for dev in order]


def permission_hint(device: str) -> str:
    """Explain how to get read/write access to a serial device."""
    return '\n'.join((
        '目前使用者沒有 %s 的讀寫權限。擇一處理:' % device,
        '  (a) 用 usb_udev_namer 把 IMU 命名為 %s (同時開放讀寫權限):'
        % PREFERRED_PORT,
        '        %s' % NAMER_URL,
        '  (b) 加入 dialout 群組後重新登入 (WSL: 在 PowerShell 執行 '
        'wsl --shutdown 再開):',
        '        sudo usermod -aG dialout "$USER"',
    ))


def check_port_access(device: str,
                      access_func: Callable[[str, int], bool] = os.access
                      ) -> None:
    """Raise LauncherError if ``device`` is not readable and writable."""
    if not access_func(device, os.R_OK | os.W_OK):
        raise LauncherError(permission_hint(device))


def usbipd_instructions() -> str:
    """Return the steps to attach a USB device to WSL2 and name it."""
    return '\n'.join((
        'WSL2 看不到 USB 序列埠。請依序處理:',
        '  1. Windows PowerShell 安裝 usbipd-win (只需一次,裝完重開 WSL):',
        '       winget install usbipd',
        '  2. 在 WSL 內用 wsl_usb_auto 把 IMU 綁定進 WSL (之後插上自動掛載):',
        '       %s' % WSL_ATTACH_URL,
        '  3. 用 usb_udev_namer 把 IMU 命名為 %s:' % PREFERRED_PORT,
        '       %s' % NAMER_URL,
        '完成後執行 ls -l %s 確認,再重新選擇。' % PREFERRED_PORT,
    ))


def no_port_message(platform: str) -> str:
    """Return the error shown when no serial port is found."""
    if platform == 'mac':
        return ('找不到序列埠 (/dev/cu.usbserial* / /dev/cu.SLAB* / '
                '/dev/cu.wchusbserial*)。請確認 IMU 已插上,'
                '必要時安裝 CP210x 驅動。')
    return ('找不到序列埠 (%s、/dev/serial/by-id/*、/dev/ttyUSB*、'
            '/dev/ttyACM*)。請確認 IMU 已插上。\n'
            '固定裝置名稱 %s 可用 usb_udev_namer 建立: %s'
            % (PREFERRED_PORT, PREFERRED_PORT, NAMER_URL))


# ---------------------------------------------------------- docker run args
def container_port(platform: str, device: str) -> str:
    """Return the serial port name the driver uses inside the container."""
    return MAC_SERIAL_URL if platform == 'mac' else device


def container_command(mode: str, port: Optional[str]) -> List[str]:
    """Return the command executed inside the container."""
    if mode == 'shell':
        return ['bash']
    if not port:
        raise LauncherError('模式 %s 需要序列埠' % mode)
    if mode in ('foxglove', 'rviz'):
        return ['ros2', 'launch', 'wit_ros2_imu', 'imu.launch.py',
                'port:=%s' % port,
                'rviz:=%s' % ('true' if mode == 'rviz' else 'false')]
    if mode == 'calibrate':
        return ['ros2', 'run', 'wit_ros2_imu', 'imu_calibrate',
                '--port', port, '--baud', str(BAUD),
                '--config', CALIBRATION_FILE]
    raise LauncherError('未知的模式 %r' % mode)


def publish_args(expose_lan: bool) -> List[str]:
    """Return -p options; loopback only unless LAN access is requested."""
    prefix = '' if expose_lan else '127.0.0.1:'
    args: List[str] = []
    for port in (ROSBRIDGE_PORT, FOXGLOVE_PORT):
        args += ['-p', '%s%d:%d' % (prefix, port, port)]
    return args


def display_available(platform: str, env: Mapping[str, str]) -> bool:
    """Return True if a GUI display is configured for ``platform``."""
    if platform == 'wsl':
        return bool(env.get('DISPLAY') or env.get('WAYLAND_DISPLAY'))
    return bool(env.get('DISPLAY'))


def gui_args(platform: str, env: Mapping[str, str]) -> List[str]:
    """Return docker options that give the container a display."""
    if platform == 'mac':
        return ['-e', 'DISPLAY=host.docker.internal:0']
    if platform == 'wsl':
        args = ['-v', '/tmp/.X11-unix:/tmp/.X11-unix',
                '-v', '/mnt/wslg:/mnt/wslg']
        for name in ('DISPLAY', 'WAYLAND_DISPLAY'):
            if env.get(name):
                args += ['-e', name]
        args += ['-e', 'XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir']
        if env.get('PULSE_SERVER'):
            args += ['-e', 'PULSE_SERVER']
        return args
    if env.get('DISPLAY'):
        return ['-e', 'DISPLAY', '-v', '/tmp/.X11-unix:/tmp/.X11-unix']
    return []


def build_run_args(platform: str, port: Optional[str], mode: str,
                   expose_lan: bool, repo_dir: str,
                   env: Mapping[str, str],
                   realpath_func: Callable[[str], str] = os.path.realpath
                   ) -> List[str]:
    """Return the full ``docker run`` argument list.

    ``port`` is the host serial path (e.g. ``/dev/usb_imu``, ``/dev/ttyUSB0``
    or ``/dev/cu.usbserial-0001``); it may be None only in shell mode.  On
    Linux / WSL the resolved host device is mapped into the container under
    the same name as ``port``, so a fixed name stays visible to the driver.
    """
    if platform not in PLATFORMS:
        raise LauncherError('未知的平台 %r' % platform)
    if mode not in MODES:
        raise LauncherError('未知的模式 %r' % mode)
    if port is not None and not port.startswith('/dev/'):
        raise LauncherError('序列埠必須是 /dev/ 底下的裝置: %r' % port)
    if not port and mode != 'shell':
        raise LauncherError('模式 %s 需要序列埠' % mode)

    args = ['docker', 'run', '--rm', '-it', '--name', CONTAINER_NAME]
    if mode in PUBLISH_MODES:
        args += publish_args(expose_lan)
    args += ['-v', '%s:%s' % (os.path.join(repo_dir, 'config'),
                              CONTAINER_CONFIG)]
    inner_port = None
    if port:
        inner_port = container_port(platform, port)
        if platform != 'mac':
            args += ['--device', '%s:%s' % (realpath_func(port), port)]
    if mode in GUI_MODES:
        args += gui_args(platform, env)
    args.append(IMAGE)
    args += container_command(mode, inner_port)
    return args


def bridge_command(repo_dir: str, device: str) -> List[str]:
    """Return the command that starts the macOS serial->TCP bridge."""
    return [sys.executable,
            os.path.join(repo_dir, 'host', 'serial_tcp_bridge.py'),
            '--serial', device, '--baud', str(BAUD),
            '--port', str(BRIDGE_PORT)]


# ----------------------------------------------------------- docker helpers
def docker(args: Sequence[str], timeout: Optional[float] = DOCKER_TIMEOUT,
           capture: bool = True) -> subprocess.CompletedProcess:
    """Run ``docker <args>`` (list form) and translate common failures."""
    cmd = ['docker'] + list(args)
    try:
        return subprocess.run(cmd, check=False, timeout=timeout,
                              capture_output=capture,
                              text=True if capture else None)
    except FileNotFoundError:
        raise LauncherError('找不到 docker 指令,請先安裝 Docker '
                            '(Linux: Docker Engine;WSL2/macOS: Docker '
                            'Desktop 或 WSL 內的 Docker Engine)。')
    except subprocess.TimeoutExpired:
        raise LauncherError('docker %s 超過 %s 秒沒有回應'
                            % (' '.join(args[:2]), timeout))


def docker_daemon_status() -> Tuple[bool, str]:
    """Return (ok, detail) for ``docker info``."""
    result = docker(['info', '--format', '{{.ServerVersion}}'])
    if result.returncode == 0:
        return True, 'server %s' % result.stdout.strip()
    detail = (result.stderr or result.stdout or '').strip().splitlines()
    return False, detail[-1] if detail else 'exit %d' % result.returncode


def image_exists() -> bool:
    """Return True if the IMU image has been built."""
    return docker(['image', 'inspect', IMAGE]).returncode == 0


def container_running() -> bool:
    """Return True if the yahboom-imu container is running."""
    result = docker(['ps', '--filter', 'name=^/%s$' % CONTAINER_NAME,
                     '--format', '{{.Names}}'])
    if result.returncode != 0:
        raise LauncherError('docker ps 失敗: %s' % result.stderr.strip())
    return CONTAINER_NAME in result.stdout.split()


def stop_running_container() -> None:
    """Stop the yahboom-imu container."""
    result = docker(['stop', CONTAINER_NAME], timeout=STOP_TIMEOUT)
    if result.returncode != 0:
        raise LauncherError('docker stop 失敗: %s' % result.stderr.strip())


def pyserial_installed() -> bool:
    """Return True if pyserial can be imported by this interpreter."""
    try:
        importlib.import_module('serial')
    except ImportError:
        return False
    return True


def tcp_port_free(port: int) -> bool:
    """Return True if 127.0.0.1:<port> can be bound."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(('127.0.0.1', port))
        except OSError:
            return False
    return True


def connection_info(mode: str, expose_lan: bool) -> str:
    """Return the Foxglove / RViz connection instructions."""
    lines = [
        '',
        '=== 連線資訊 ===',
        'Foxglove (app.foxglove.dev 或桌面版) → "Open connection":',
        '  • Foxglove WebSocket: ws://localhost:%d  (建議)' % FOXGLOVE_PORT,
        '  • Rosbridge:          ws://localhost:%d' % ROSBRIDGE_PORT,
        '建議面板:',
        '  • 3D:看 TF world → imu_link 的姿態 (Fixed frame = world)',
        '  • Plot:/imu/data.linear_acceleration.x/.y/.z、'
        '/imu/data.angular_velocity.z',
        '  • Raw Messages:/imu/data、/imu/mag、/imu/temperature',
    ]
    if expose_lan:
        lines.append('已開放區網:其他電腦請改用 ws://<本機 IP>:%d / :%d'
                     % (FOXGLOVE_PORT, ROSBRIDGE_PORT))
    if mode == 'rviz':
        lines.append('RViz2 視窗會自動開啟 (Fixed Frame = world)。')
    lines.append('結束:在此終端機按 Ctrl+C。')
    return '\n'.join(lines)


# ------------------------------------------------------------- interactive
class Launcher:
    """Interactive menu; all host interactions are injectable for tests."""

    def __init__(self, platform: str, repo_dir: str,
                 env: Mapping[str, str],
                 input_func: Callable[[str], str] = input,
                 port_lister: Callable[[str], List[SerialPort]]
                 = list_serial_ports,
                 access_func: Callable[[str, int], bool] = os.access,
                 isatty: Optional[Callable[[], bool]] = None,
                 pyserial_available: Callable[[], bool] = pyserial_installed
                 ) -> None:
        """Store the platform, repo path and host interaction hooks."""
        if platform not in PLATFORMS:
            raise LauncherError('未知的平台 %r' % platform)
        self.platform = platform
        self.repo_dir = repo_dir
        self.env = env
        self.input_func = input_func
        self.port_lister = port_lister
        self.access_func = access_func
        self.isatty = isatty or sys.stdin.isatty
        self.pyserial_available = pyserial_available

    # ------------------------------------------------------------ prompts
    def ask_choice(self, prompt: str, minimum: int, maximum: int) -> int:
        """Ask for an integer in [minimum, maximum] until one is given."""
        while True:
            text = self.input_func('%s [%d-%d]: '
                                   % (prompt, minimum, maximum)).strip()
            if text.isdecimal() and minimum <= int(text) <= maximum:
                return int(text)
            print('無效的選擇 %r,請輸入 %d 到 %d 的數字。'
                  % (text, minimum, maximum))

    def ask_yes_no(self, prompt: str) -> bool:
        """Ask a y/N question; default is No."""
        while True:
            text = self.input_func('%s (y/N): ' % prompt).strip().lower()
            if text in ('y', 'yes'):
                return True
            if text in ('', 'n', 'no'):
                return False
            print('請輸入 y 或 n。')

    # --------------------------------------------------------------- menu
    def run_menu(self) -> int:
        """Run the menu loop and return the process exit code."""
        actions = {
            1: self.pull_image,
            2: lambda: self.start('foxglove'),
            3: lambda: self.start('rviz'),
            4: lambda: self.start('calibrate'),
            5: lambda: self.start('shell'),
            6: self.stop_container,
            7: self.check_environment,
        }
        failed = False
        try:
            while True:
                print('\n=== Yahboom CMP10A IMU (%s) ===' % self.platform)
                for key, text in MENU:
                    print('  %s) %s' % (key, text))
                choice = self.ask_choice('請選擇', 0, len(actions))
                if choice == 0:
                    return 1 if failed else 0
                try:
                    actions[choice]()
                except LauncherError as exc:
                    failed = True
                    print('\n[錯誤] %s' % exc, file=sys.stderr)
        except EOFError:
            print('\n輸入結束,離開。', file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            print('\n已中斷。', file=sys.stderr)
            return 130

    # ------------------------------------------------------------ actions
    def pull_image(self) -> None:
        """Download the prebuilt image published by GitHub Actions."""
        self._require_daemon()
        print('下載 %s (約 2-3 GB,視網路速度需數分鐘)...' % IMAGE)
        result = docker(['pull', IMAGE], timeout=PULL_TIMEOUT, capture=False)
        if result.returncode != 0:
            raise LauncherError(
                'docker pull %s 失敗 (exit %d)。請確認網路連線,'
                '以及該映像已由 GitHub Actions 建置並設為公開。'
                % (IMAGE, result.returncode))
        print('映像 %s 下載完成。' % IMAGE)

    def stop_container(self) -> None:
        """Stop the running container, if any."""
        if not container_running():
            print('容器 %s 沒有在執行。' % CONTAINER_NAME)
            return
        stop_running_container()
        print('已停止容器 %s。' % CONTAINER_NAME)

    def start(self, mode: str) -> bool:
        """Start the container in ``mode``; return False if cancelled."""
        if mode not in MODES:
            raise LauncherError('未知的模式 %r' % mode)
        if not self.isatty():
            raise LauncherError('docker run -it 需要互動式終端機 (TTY),'
                                '請直接在終端機執行 python3 imu.py。')
        self._require_daemon()
        if not image_exists():
            raise LauncherError('映像 %s 尚未下載,請先選 1 下載映像。'
                                % IMAGE)
        port = self._select_port(mode)
        if port is not None:
            self._check_port(port)
        if mode in GUI_MODES and not self._check_display(mode):
            return False
        expose_lan = False
        if mode in PUBLISH_MODES:
            expose_lan = self.ask_yes_no(
                '是否開放區網存取 (埠綁 0.0.0.0;rosbridge 沒有認證)?')
        if container_running():
            if not self.ask_yes_no('容器 %s 正在執行,要先停止它嗎?'
                                   % CONTAINER_NAME):
                print('已取消。')
                return False
            stop_running_container()

        config_dir = os.path.join(self.repo_dir, 'config')
        os.makedirs(config_dir, exist_ok=True)
        args = build_run_args(self.platform, port, mode, expose_lan,
                              self.repo_dir, self.env)
        bridge = None
        if self.platform == 'mac' and port is not None:
            bridge = self._start_bridge(port)
        try:
            if mode in ('foxglove', 'rviz'):
                print(connection_info(mode, expose_lan))
            print('\n執行: %s\n' % ' '.join(args))
            self._run_foreground(args)
        finally:
            if bridge is not None:
                self._stop_bridge(bridge)
        return True

    def check_environment(self) -> List[str]:
        """Print an environment report and return the problems found."""
        problems: List[str] = []
        print('\n=== 環境檢查 ===')
        print('平台: %s (%s %s), Python %s' % (
            self.platform, platform_mod.system(), platform_mod.release(),
            platform_mod.python_version()))
        daemon_ok = False
        try:
            daemon_ok, detail = docker_daemon_status()
        except LauncherError as exc:
            detail = str(exc)
        self._report(daemon_ok, 'docker daemon', detail, problems)
        if daemon_ok:
            built = image_exists()
            self._report(built, '映像 %s' % IMAGE,
                         '已下載' if built else '尚未下載 (選 1)', problems)
            running = container_running()
            state = '執行中' if running else '未執行'
            print('[--] 容器 %s: %s' % (CONTAINER_NAME, state))
        ports = self.port_lister(self.platform)
        if not ports:
            self._report(False, '序列埠', no_port_message(self.platform),
                         problems)
            if self.platform == 'wsl':
                print(usbipd_instructions())
        for item in ports:
            if self.platform == 'mac':
                ok, detail = True, '可用'
            else:
                ok = bool(self.access_func(item.device, os.R_OK | os.W_OK))
                detail = '可讀寫' if ok else '沒有讀寫權限 (usb_udev_namer / dialout)'
            self._report(ok, '序列埠 %s' % item.label, detail, problems)
        self._report_display(problems)
        if self.platform == 'mac':
            ok = self.pyserial_available()
            self._report(ok, 'pyserial (主機)',
                         '已安裝' if ok else '未安裝:' + PYSERIAL_HINT,
                         problems)
        summary = '發現 %d 個問題' % len(problems) if problems else '全部正常'
        print('結果: %s' % summary)
        return problems

    # ---------------------------------------------------------- internals
    @staticmethod
    def _report(ok: bool, name: str, detail: str,
                problems: List[str]) -> None:
        print('[%s] %s: %s' % ('OK' if ok else '!!', name, detail))
        if not ok:
            problems.append('%s: %s' % (name, detail))

    def _report_display(self, problems: List[str]) -> None:
        names = ('DISPLAY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR')
        values = ', '.join('%s=%s' % (n, self.env.get(n, '(未設定)'))
                           for n in names)
        if self.platform == 'mac':
            print('[--] GUI: XQuartz (DISPLAY=host.docker.internal:0); '
                  '主機 %s' % values)
            return
        ok = display_available(self.platform, self.env)
        if not ok:
            values += ' — RViz2 不可用,可改用 Foxglove'
        self._report(ok, 'GUI', values, problems)

    def _require_daemon(self) -> None:
        ok, detail = docker_daemon_status()
        if not ok:
            raise LauncherError('Docker daemon 無法使用: %s\n'
                                '請先啟動 Docker (Docker Desktop 或 '
                                'sudo service docker start),並確認使用者'
                                '在 docker 群組。' % detail)

    def _select_port(self, mode: str) -> Optional[str]:
        ports = self.port_lister(self.platform)
        if not ports:
            if self.platform == 'wsl':
                print(usbipd_instructions())
            if mode == 'shell':
                print('[注意] 沒有序列埠,容器將不帶 IMU 裝置啟動。')
                return None
            raise LauncherError(no_port_message(self.platform))
        if any(item.selected == PREFERRED_PORT for item in ports):
            print('\n使用 %s' % PREFERRED_PORT)
            return PREFERRED_PORT
        print('\n可用序列埠:')
        for index, item in enumerate(ports, start=1):
            print('  %d) %s' % (index, item.label))
        choice = self.ask_choice('選擇序列埠', 1, len(ports))
        device = ports[choice - 1].selected
        if not device.startswith('/dev/'):
            raise LauncherError('不合法的序列埠路徑: %r' % device)
        return device

    def _check_port(self, device: str) -> None:
        if self.platform == 'mac':
            if not self.pyserial_available():
                raise LauncherError(
                    'macOS 需要主機端 pyserial 來執行序列埠橋接,請執行:\n'
                    '  %s\n然後重新啟動。' % PYSERIAL_HINT)
            return
        check_port_access(device, self.access_func)

    def _check_display(self, mode: str) -> bool:
        if self.platform == 'mac':
            print('[提示] macOS 需要 XQuartz:Preferences > Security 勾選 '
                  '"Allow connections from network clients",重新啟動 '
                  'XQuartz 後在主機執行 xhost +localhost。')
            return True
        if not display_available(self.platform, self.env):
            print('[警告] 沒有 DISPLAY,RViz2 無法顯示視窗。'
                  '建議改用選項 2 (Foxglove)。')
            if mode == 'rviz':
                return self.ask_yes_no('仍要繼續嗎?')
            return True
        if self.platform == 'linux':
            print('[提示] 若 RViz2 無法開啟視窗,可能需要在主機執行 '
                  'xhost +local:docker (本工具不會自動執行)。')
        return True

    def _start_bridge(self, device: str) -> subprocess.Popen:
        if not tcp_port_free(BRIDGE_PORT):
            raise LauncherError('127.0.0.1:%d 已被佔用,請關閉佔用的程式 '
                                '(lsof -i :%d)。' % (BRIDGE_PORT,
                                                    BRIDGE_PORT))
        log_path = os.path.join(tempfile.gettempdir(),
                                'yahboom-imu-bridge.log')
        print('啟動序列埠橋接 %s → 127.0.0.1:%d (log: %s)'
              % (device, BRIDGE_PORT, log_path))
        with open(log_path, 'ab') as log_file:
            proc = subprocess.Popen(bridge_command(self.repo_dir, device),
                                    stdin=subprocess.DEVNULL,
                                    stdout=log_file, stderr=log_file)
        time.sleep(BRIDGE_STARTUP_WAIT)
        if proc.poll() is not None:
            raise LauncherError('序列埠橋接啟動失敗 (exit %s),請看 %s'
                                % (proc.returncode, log_path))
        return proc

    @staticmethod
    def _stop_bridge(proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=BRIDGE_STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            print('[警告] 橋接未在 %d 秒內結束,強制終止。'
                  % BRIDGE_STOP_TIMEOUT, file=sys.stderr)
            proc.kill()
            proc.wait(timeout=BRIDGE_STOP_TIMEOUT)
        print('序列埠橋接已停止。')

    def _run_foreground(self, args: List[str]) -> None:
        interrupted = False
        try:
            result = subprocess.run(args, check=False,
                                    timeout=SESSION_TIMEOUT)
        except KeyboardInterrupt:
            interrupted = True
            result = None
        except subprocess.TimeoutExpired:
            interrupted = True
            result = None
            print('\n[警告] 執行超過 %d 小時,停止容器。'
                  % (SESSION_TIMEOUT // 3600), file=sys.stderr)
        except FileNotFoundError:
            raise LauncherError('找不到 docker 指令')
        if interrupted:
            print('\n已中斷,清理容器...')
            if container_running():
                stop_running_container()
            return
        code = result.returncode
        if code in (125, 126, 127):
            raise LauncherError(
                'docker run 失敗 (exit %d)。常見原因:9090/8765 埠被佔用 '
                '(先選 6 停止容器或關閉佔用程式)、裝置不存在、映像損壞。'
                % code)
        print('容器已結束 (exit %d)。' % code)


def main() -> int:
    """Entry point: detect the platform and run the menu."""
    try:
        platform = detect_platform()
    except LauncherError as exc:
        print('[錯誤] %s' % exc, file=sys.stderr)
        return 1
    repo_dir = os.path.dirname(os.path.abspath(__file__))
    return Launcher(platform, repo_dir, os.environ).run_menu()


if __name__ == '__main__':
    sys.exit(main())
