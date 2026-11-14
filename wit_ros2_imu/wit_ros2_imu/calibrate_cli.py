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
Interactive calibration menu for the Yahboom CMP10A IMU (no ROS needed).

The serial port, ``input`` and ``print`` replacements, sleep and clock are
injected into :func:`run_menu` so the whole menu can be tested with a fake
serial object.
"""

import argparse
import dataclasses
import logging
import math
import os
import re
import select
import statistics
import sys
import threading
import time
from typing import Callable, Dict, Iterator, List, Optional, Tuple

import serial

from . import calibration_check as cc
from . import wit_protocol as wp
from .calibration import (compute_mag_offset, compute_static_bias, load_calibration,
                          MAG_COVERAGE_TARGET, MagCoverage, MIN_SAMPLES, save_calibration,
                          SoftwareCalibration)

logger = logging.getLogger(__name__)

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 9600
DEFAULT_CONFIG = '/config/imu_calibration.yaml'
SERIAL_TIMEOUT = 0.5

READ_CHUNK = 64
CMD_GAP = 0.1               # s between consecutive commands
UNLOCK_WINDOW = 10.0        # s the device stays unlocked
UNLOCK_MARGIN = 2.0         # re-unlock if less than this is left
ACCEL_CAL_WAIT = 5.0        # s to hold still during accelerometer calibration
SHORT_CAL_WAIT = 1.0        # s for heading / angle-reference / height zeroing
REPLY_TIMEOUT = 1.5         # s to wait for a 0x5F register reply
STATIC_SAMPLES = 50         # accel + gyro samples for software static calibration
STATIC_TIMEOUT = 20.0
SOFT_MAG_TIMEOUT = 90.0     # s; software hard-iron sampling ends earlier once all axes are covered
SOFT_MAG_COVERAGE_TARGET = 0.95   # auto-stop needs both extremes for (max+min)/2
# Dual-plane turns keep one axis in the horizontal plane, where it only sees
# the horizontal part of the inclined geomagnetic field: cos(inclination) of
# the full swing (about 0.80 in Taiwan).  0.70 stays reachable up to 45 deg.
DUAL_PLANE_COVERAGE_TARGET = 0.70
DUAL_PLANE_STEPS = [
    '開始後請先將 IMU(連同機器人)水平繞 Z 軸緩慢轉一圈,再立起繞 X 或 Y 軸轉一圈;',
    '若不確定,繞三軸緩慢轉動即可。',
    '雙平面的轉法掃不滿每一軸的全部範圍,進度條已依此放寬;',
    '某一軸仍停在九成左右時可直接按 Enter 結束,再用選項 12 驗證。',
    '進度長時間不動,通常是附近有鐵件或電器干擾,請移開後重做。',
]
FIELD_MEASURE_TIME = 1.5    # s of normal-mode data used to measure |B| before a mag calibration
LARGE_ACCEL_BIAS = 1.0      # m/s^2; above this the IMU was probably not level

# Progress display
STATUS_INTERVAL = 0.5       # s between progress-line refreshes
COUNTDOWN_STEP = 0.5        # s per countdown tick
NO_DATA_HINT_AFTER = 3.0    # s without gyro data before falling back to an elapsed-time hint
RECOMMENDED_MAG_SECONDS = 60.0   # rotation time suggested when no data is visible
READER_YIELD = 0.005        # s; real-time pause per read so the Enter prompt stays responsive
READER_JOIN_TIMEOUT = 2.0   # s; > SERIAL_TIMEOUT so a blocked read can finish
PROGRESS_WIDTH = 20
ATTITUDE_INTERVAL = 0.2     # s between attitude-view refreshes
PRESS_BIT = 1 << 6          # RSW bit enabling the 0x56 pressure / height packet
AXIS_BAR_WIDTH = 8          # per-axis bars; keeps the status line within 79 columns
CLEAR_EOL = '\x1b[K'
CLEAR_SCREEN = '\x1b[2J\x1b[H'
# Take back the line break the terminal echoed for Enter, so a view keeps one line.
CURSOR_UP_CLEAR = '\x1b[A\r\x1b[2K'

# Offset registers shown by "read offsets" (read-only).
OFFSET_REGISTERS = [
    (0x05, 'AXOFFSET'), (0x06, 'AYOFFSET'), (0x07, 'AZOFFSET'),
    (0x08, 'GXOFFSET'), (0x09, 'GYOFFSET'), (0x0A, 'GZOFFSET'),
    (0x0B, 'HXOFFSET'), (0x0C, 'HYOFFSET'), (0x0D, 'HZOFFSET'),
]

MENU = """
========== CMP10A IMU 校正工具 ==========
 1. 即時資料檢視                          → 任何地方
 2. 加速度計水平校正(硬體)                → 水平桌面、靜止,高度不拘
 3. 磁力計校正 球面擬合(硬體)             → 最終安裝位置、實際使用環境
 4. 磁力計校正 雙平面(硬體)               → 最終安裝位置(機器人無法三軸翻轉時)
 5. Z 軸航向歸零(硬體)                    → 最終位置,對準正前方
 6. 設定角度參考(硬體)                    → 裝好後,機器人放水平地面
 7. 高度歸零(硬體,氣壓計)                 → 在要當作 0 m 的高度
 8. 讀取 IMU 內部偏移暫存器               → 任何地方(唯讀)
 9. 軟體靜態偏移校正(gyro/accel → YAML)   → 暖機 5 分鐘後、水平靜止
10. 軟體磁力計 hard-iron 校正(→ YAML)     → 最終安裝位置,三軸各轉滿一圈
11. 恢復原廠設定                          → 任何地方
12. 校正檢測 / 姿態檢視                   → 任何地方(唯讀),判定校正是否合格
13. 氣壓高度輸出 開啟/關閉                → 寫入 IMU 設定
 0. 離開
=========================================="""

ATTITUDE_HELP = [
    '操作:Enter = 設定參考點(之後顯示相對變化 Δ)  r = 清除參考點  q = 離開(Ctrl+C 也可以)',
]

# Appended to a menu row once that step has succeeded in this run of the tool.
# The marks live in memory only: a new run starts with none, so a green row
# always means "done since this tool was started", never an older calibration.
DONE_MARK = '✓ 本次已完成'
_MENU_ROW = re.compile(r'^\s*(\d+)\.')


# Calibration check (option 12).  Pass thresholds live in calibration_check.py;
# the values here only control how the data is collected.
STILL_GYRO_DPS = 2.0          # |gyro| below this counts as "not moving"
STILL_HOLD_S = 1.0            # hold still this long before a measurement starts
FACE_SAMPLE_S = 5.0           # accelerometer averaging time per face
GYRO_STILL_S = 300.0          # static gyro test duration (5 minutes)
MOVE_ACCEL = 0.5              # m/s^2 change of any accel axis = the IMU was moved
FAST_TURN_DPS = 90.0          # warn above this rate during the full-turn test
DIP_MAX_RATE_DPS = 30.0       # accel is only gravity when turning slower than this
DIP_ACCEL_TOL = 0.5           # m/s^2; | |a| - g | above this = accelerating
MAG_MIN_SAMPLES = 50
DIP_MIN_PAIRS = 20
NOAA_CALCULATOR = 'https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml'
CHECK_ITEMS = ('accel', 'gyro_still', 'gyro_turn', 'mag_norm', 'mag_dip')
CHECK_TITLE = '== 校正檢測 =='
# Environment / mounting check before a magnetometer calibration.
ENV_STILL_S = 5.0
SLIDE_DISTANCE_CM = 30
MOTOR_BASE_S = 3.0
MOTOR_RUN_S = 5.0
PRECHECK_INTRO = [
    '校正前先用 IMU 自己的讀數檢查環境與安裝位置(約 1 分鐘):',
    '  地磁在一個房間內處處相同、不隨時間改變;讀數若會變動或跟著位置改變,就是周圍有干擾。',
    '標準與理由見 docs/CALIBRATION_CHECK.md。',
]
PASS, FAIL, SKIP, NOTE = '合格', '不合格', '略過', '注意'
ANSI_COLOURS = {PASS: '\033[32m', FAIL: '\033[31m', SKIP: '\033[33m', NOTE: '\033[33m'}
ANSI_RESET = '\033[0m'
CHECK_INTRO = [
    '每一項都拿「IMU 以外的已知參考」比對原始讀數,不使用模組自己算的角度:',
    '  重力 1 g、靜止時角速度為 0、桌上的參考線、NOAA 地磁模型的當地磁傾角。',
    '合格門檻與理由見 docs/CALIBRATION_CHECK.md。q = 結束檢測,s = 略過這一項。',
]
CHECK_FAIL_HINTS = [
    '不合格時:',
    '  - 加速度計:在水平桌面重做選項 2。',
    '  - 陀螺儀靜止:通電 5 分鐘後,靜止重做選項 9。',
    '  - 陀螺儀轉一圈:轉慢一點、確實對準參考線再測一次;仍不合格是陀螺儀刻度誤差,'
    '本模組沒有可校正刻度的指令。',
    '  - 磁力計:清空周圍 20 cm 的電器與鐵件,重做選項 3 或 4。',
]


class _VerifyQuit(Exception):
    """Raised inside the guided verification when the user enters q."""


# Shown before every magnetometer calibration that displays per-axis progress.
MAG_PROGRESS_HELP = [
    '下方會分別顯示 X、Y、Z 三軸的進度(依磁力計讀數涵蓋的範圍計算),並列出還缺哪一軸:',
    '  X 不夠 → 繞 Y 或 Z 軸轉;Y 不夠 → 繞 X 或 Z 軸轉;Z 不夠 → 繞 X 或 Y 軸轉。',
    '三軸都到 100% 會顯示 [完成],再按 Enter 結束;多轉幾圈也沒關係。',
]

# Placement / precaution notes shown after a calibration item is chosen,
# before anything is sent to the IMU.  Keys are menu choices.
NOTES: Dict[str, List[str]] = {
    '2': [
        '- 放在「真正水平」的桌面上,最好用水平儀確認;高度沒有影響,水平最重要。',
        '- IMU Z 軸朝上,校正期間完全靜止、不要碰桌子。',
        '- 可以先單獨校正 IMU 模組,再裝到機器人上。',
    ],
    '3': [
        '- 在最終安裝位置、實際使用的環境做:IMU 要連同機器人一起轉,',
        '  機器人身上會一起移動的鐵件才會被補償。',
        '- 遠離環境中的鐵件與磁鐵 20 cm 以上(鐵桌、鐵櫃、含鋼筋地板、馬達、喇叭)。',
        '- 如果 IMU 實際會貼近地板或桌面運作,就在同樣的距離下校正,不要換地方校正再搬回去。',
        '- 機器人太大、無法三軸翻轉時,改用 4 雙平面校正。',
    ],
    '4': [
        '- 在最終安裝位置、實際使用的環境做,適合無法三軸翻轉的機器人。',
        '- 遠離環境中的鐵件與磁鐵 20 cm 以上(鐵桌、鐵櫃、含鋼筋地板、馬達、喇叭)。',
        '- 如果 IMU 實際會貼近地板或桌面運作,就在同樣的距離下校正。',
    ],
    '5': [
        '- 在最終位置做,先把機器人擺成你要當作「正前方」的方向。',
        '- 需要時可以每次使用前重做。',
    ],
    '6': [
        '- IMU 裝到機器人上之後,把機器人放在水平地面再執行。',
        '- 執行後,目前的姿態會被當作 roll = pitch = 0,用來修正 IMU 裝歪。',
    ],
    '7': [
        '- 在你要當作 0 m 的高度執行,例如地面或出發點。',
        '- 氣壓會隨天氣變化,每次使用前最好重做。',
        '- CMP10A 出廠預設不輸出氣壓封包 (0x56):請先用選項 13 開啟,',
        '  /imu/pressure 與 12 姿態檢視才看得到高度。',
    ],
    '9': [
        '- IMU 通電暖機約 5 分鐘後再做:陀螺儀偏移會隨溫度改變。',
        '- 水平靜止、Z 軸朝上;這項假設靜置時 accel = (0, 0, +g),IMU 裝歪時不要用。',
        '- 建議先做 2 加速度計硬體校正,這項用來修剩下的誤差。',
    ],
    '10': [
        '- 和 3 一樣,在最終安裝位置、實際使用環境做,遠離環境中的鐵件 20 cm 以上。',
        '- 取樣期間 X、Y、Z 三軸都要各轉滿一圈,結果才準。',
        '- 建議先做 3 或 4 磁力計硬體校正,這項用來修剩下的誤差。',
    ],
}


class CalibrationMenu:
    """State and actions of the interactive menu."""

    def __init__(self, ser, input_fn: Callable[[str], str], output_fn: Callable[..., None],
                 config_path, sleep_fn: Callable[[float], None] = time.sleep,
                 time_fn: Callable[[], float] = time.monotonic, color: bool = False,
                 key_ready: Optional[Callable[[], bool]] = None):
        """
        Store the injected serial port, I/O functions and clock.

        ``key_ready`` reports whether a typed line is waiting; when given, the
        guided verification shows live values and can advance by itself.
        """
        self._color = bool(color)
        self._key_ready = key_ready
        self.ser = ser
        self._input = input_fn
        self._out = output_fn
        self.config_path = config_path
        self._sleep = sleep_fn
        self._time = time_fn
        self._unlock_time: Optional[float] = None
        self._verify_lines: List[str] = []
        self._verify_title = CHECK_TITLE
        self._check_cal = SoftwareCalibration()
        self._check_passed: set = set()   # check items passed in this run
        self._expected_dip: Optional[float] = None
        self._done: set = set()          # menu choices that succeeded in this run
        self._step_ok = False            # set by an action when it succeeds
        self._mag_covered = False
        self._actions = {
            '1': self.live_view,
            '2': self.hw_accel,
            '3': lambda: self.hw_mag(wp.CalMode.MAG_SPHERE, '磁力計校正 球面擬合'),
            '4': lambda: self.hw_mag(wp.CalMode.MAG_DUAL_PLANE, '磁力計校正 雙平面'),
            '5': self.hw_heading_zero,
            '6': self.hw_angle_ref,
            '7': self.hw_height_zero,
            '8': self.read_offsets,
            '9': self.sw_static,
            '10': self.sw_mag,
            '11': self.factory_reset,
            '12': self.attitude_view,
            '13': self.toggle_pressure_output,
        }

    # ------------------------------------------------------------ helpers

    def _ask(self, prompt: str) -> str:
        answer = self._input(prompt)
        if not isinstance(answer, str):
            raise TypeError(f'input_fn must return str, got {type(answer).__name__}')
        return answer.strip()

    def _confirm(self, question: str) -> bool:
        return self._ask(f'{question} [y/N]: ').lower() in ('y', 'yes')

    def _send(self, cmd: bytes) -> None:
        logger.debug('TX %s', cmd.hex(' '))
        self.ser.write(cmd)
        self.ser.flush()
        self._sleep(CMD_GAP)

    def _unlock(self) -> None:
        self._send(wp.cmd_unlock())
        self._unlock_time = self._time()

    def _ensure_unlocked(self) -> None:
        if (self._unlock_time is None
                or self._time() - self._unlock_time > UNLOCK_WINDOW - UNLOCK_MARGIN):
            self._unlock()

    def _read_stream(self, parser: wp.FrameParser) -> List[wp.Packet]:
        data = self.ser.read(READ_CHUNK)
        if not data:
            return []
        return parser.feed(data)

    def _packets_for(self, duration: float) -> Iterator[wp.Packet]:
        """Yield decoded packets for ``duration`` seconds of fresh data."""
        parser = wp.FrameParser()
        self.ser.reset_input_buffer()
        end = self._time() + duration
        while self._time() < end:
            yield from self._read_stream(parser)
        if parser.checksum_errors:
            self._out(f'(取樣期間 checksum 錯誤 {parser.checksum_errors} 次)')

    def _load_existing(self) -> SoftwareCalibration:
        try:
            return load_calibration(self.config_path)
        except ValueError as exc:
            logger.warning('invalid calibration file: %s', exc)
            self._out(f'現有校正檔格式錯誤,其餘欄位將以預設值覆寫:{exc}')
            return SoftwareCalibration()

    def _save(self, cal: SoftwareCalibration) -> None:
        save_calibration(self.config_path, cal)
        self._out(f'已寫入 {self.config_path}')
        self._out(f'  gyro_bias  [rad/s]  = {_fmt3(cal.gyro_bias, 6)}')
        self._out(f'  accel_bias [m/s^2]  = {_fmt3(cal.accel_bias, 4)}')
        self._out(f'  mag_offset [LSB]    = {_fmt3(cal.mag_offset, 1)}')
        self._out('重新啟動 IMU 節點後生效。')

    def _status(self, text: str) -> None:
        """Redraw the single in-place progress line."""
        self._out('\r' + text + CLEAR_EOL, end='', flush=True)

    def _end_status(self) -> None:
        """Finish the progress line so later output starts on a new line."""
        self._out('')

    def _countdown(self, seconds: float, label: str) -> None:
        """Sleep ``seconds`` while showing a progress bar and the time left."""
        steps = max(1, math.ceil(seconds / COUNTDOWN_STEP))
        step = seconds / steps
        for i in range(steps):
            left = seconds - i * step
            self._status(f'{label} {progress_bar(i / steps)}  剩 {math.ceil(left):.0f} 秒')
            self._sleep(step)
        self._status(f'{label} {progress_bar(1.0)}  完成')
        self._end_status()

    @staticmethod
    def _axis_bars(cov: MagCoverage) -> str:
        return '  '.join(f'{name} {progress_bar(p, AXIS_BAR_WIDTH)}'
                         for name, p in zip('XYZ', cov.axis_progress()))

    def _coverage_line(self, cov: MagCoverage, elapsed: float) -> str:
        if not cov.samples:
            if elapsed < NO_DATA_HINT_AFTER:
                return f'等待 IMU 磁力計資料... {elapsed:3.0f}秒'
            return (f'未收到資料,依時間判斷 {progress_bar(elapsed / RECOMMENDED_MAG_SECONDS, 8)} '
                    f'{elapsed:3.0f}/{RECOMMENDED_MAG_SECONDS:.0f}秒 時間到再按 Enter')
        tail = '[完成] 按 Enter' if cov.complete() else '缺 ' + ' '.join(cov.missing_axes())
        return f'{self._axis_bars(cov)}  {elapsed:3.0f}秒  {tail}'

    def _measure_field_radius(self) -> Optional[float]:
        """Median |B| (raw LSB) over a short normal-mode window, or None without data."""
        norms = []
        for p in self._packets_for(FIELD_MEASURE_TIME):
            if isinstance(p, wp.Mag):
                n = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
                if math.isfinite(n) and n > 0:
                    norms.append(n)
        return statistics.median(norms) if norms else None

    def _wait_mag_coverage(self, field_radius: Optional[float],
                           target: float = MAG_COVERAGE_TARGET) -> None:
        """
        Wait for Enter while a reader thread shows per-axis magnetometer coverage.

        Progress comes from the magnetometer because the CMP10A freezes its
        gyro / accel / angle output in magnetometer-calibration mode.
        KeyboardInterrupt / EOFError from the prompt propagate to the caller
        after the reader thread has been stopped.
        """
        cov = MagCoverage(field_radius, target=target)
        stop = threading.Event()
        errors: List[BaseException] = []

        def reader() -> None:
            parser = wp.FrameParser()
            start = self._time()
            next_status = start
            try:
                while True:     # read at least once, even if Enter comes immediately
                    for p in self._read_stream(parser):
                        if isinstance(p, wp.Mag):
                            cov.add(p)
                    now = self._time()
                    if now >= next_status:
                        self._status(self._coverage_line(cov, now - start))
                        next_status = now + STATUS_INTERVAL
                    if stop.wait(READER_YIELD):
                        break
            except (serial.SerialException, OSError) as exc:
                logger.error('reading IMU data during calibration failed: %s', exc)
                errors.append(exc)

        thread = threading.Thread(target=reader, name='imu-mag-progress', daemon=True)
        thread.start()
        try:
            self._ask('(按 Enter 結束校正)')
        finally:
            stop.set()
            thread.join(READER_JOIN_TIMEOUT)
            self._end_status()
        self._mag_covered = bool(cov.samples) and cov.complete() and not errors
        if errors:
            self._out(f'讀取 IMU 資料失敗:{errors[0]}(無法顯示進度,仍會結束校正)')
        elif cov.samples and not cov.complete():
            self._out(f'提醒:{"、".join(cov.missing_axes())} 軸尚未涵蓋完整,'
                      '校正結果可能不準,建議之後重做。')

    def _hardware(self, mode: wp.CalMode, title: str, instructions: List[str],
                  wait_s: float = 0.0, mag_progress: bool = False) -> None:
        """Run unlock -> CALSW mode -> wait -> CALSW NORMAL -> save."""
        self._out(f'== {title}(硬體) ==')
        for line in instructions:
            self._out(line)
        if not self._confirm('此操作會寫入 IMU flash,確定執行?'):
            self._out('已取消,未送出任何指令。')
            return
        field_radius = None
        if mag_progress:
            self._out('量測目前磁場強度(約 2 秒)...')
            field_radius = self._measure_field_radius()
        self._unlock()
        self._send(wp.cmd_calsw(mode))
        try:
            if mag_progress:
                self._wait_mag_coverage(field_radius, mag_coverage_target(mode))
            elif wait_s > 0:
                self._countdown(wait_s, '校正中,請保持不動')
        except (KeyboardInterrupt, EOFError):
            self._out('\n已中止:送出 CALSW NORMAL 恢復正常模式(未儲存)。')
            self._ensure_unlocked()
            self._send(wp.cmd_calsw(wp.CalMode.NORMAL))
            return
        self._ensure_unlocked()
        self._send(wp.cmd_calsw(wp.CalMode.NORMAL))
        self._send(wp.cmd_save())
        self._out('完成,已儲存到 IMU flash。')
        # A magnetometer calibration only counts when every axis was covered.
        self._step_ok = self._mag_covered if mag_progress else True

    # ------------------------------------------------------------ actions

    def live_view(self) -> None:
        """Print roll/pitch/yaw, accel, gyro and mag until Ctrl+C."""
        self._out('即時資料檢視(按 Ctrl+C 返回選單)')
        parser = wp.FrameParser()
        acc = gyro = mag = None
        try:
            while True:
                for p in self._read_stream(parser):
                    if isinstance(p, wp.Accel):
                        acc = p
                    elif isinstance(p, wp.Gyro):
                        gyro = p
                    elif isinstance(p, wp.Mag):
                        mag = p
                    elif isinstance(p, wp.Angle):
                        self._out(_format_live(p, acc, gyro, mag))
        except KeyboardInterrupt:
            self._out(f'\n返回選單(checksum 錯誤 {parser.checksum_errors} 次)')

    def hw_accel(self) -> None:
        """Hardware accelerometer level calibration."""
        self._hardware(wp.CalMode.ACCEL, '加速度計水平校正',
                       ['請將 IMU 水平靜置於桌面,校正期間不要觸碰。'],
                       wait_s=ACCEL_CAL_WAIT)

    def hw_mag(self, mode: wp.CalMode, title: str) -> None:
        """Hardware magnetometer calibration (sphere fit or dual plane)."""
        if not self._mag_precheck():
            return
        if mode == wp.CalMode.MAG_DUAL_PLANE:
            steps = list(DUAL_PLANE_STEPS)
        else:
            steps = ['開始後請將 IMU 繞三軸緩慢轉動(X、Y、Z 各轉數圈),遠離金屬與磁鐵。']
        steps += MAG_PROGRESS_HELP
        self._hardware(mode, title, steps, mag_progress=True)

    def hw_heading_zero(self) -> None:
        """Hardware Z-axis heading reset."""
        self._hardware(wp.CalMode.HEADING_ZERO, 'Z 軸航向歸零',
                       ['將以目前方向作為 yaw = 0。'], wait_s=SHORT_CAL_WAIT)

    def hw_angle_ref(self) -> None:
        """Hardware angle reference."""
        self._hardware(wp.CalMode.ANGLE_REF, '設定角度參考',
                       ['請先將 IMU 擺在要作為參考 (roll = pitch = 0) 的姿態。'],
                       wait_s=SHORT_CAL_WAIT)

    def hw_height_zero(self) -> None:
        """Hardware barometric height zeroing."""
        self._hardware(wp.CalMode.HEIGHT_ZERO, '高度歸零',
                       ['將以目前氣壓高度作為 0 m。'], wait_s=SHORT_CAL_WAIT)

    def _read_registers(self, start: int) -> Optional[wp.RegisterValues]:
        parser = wp.FrameParser()
        self.ser.reset_input_buffer()
        self._send(wp.cmd_read(start))
        deadline = self._time() + REPLY_TIMEOUT
        while self._time() < deadline:
            for p in self._read_stream(parser):
                if isinstance(p, wp.RegisterValues):
                    return p
        return None

    def read_offsets(self) -> None:
        """Read and display the offset registers (read-only)."""
        values = {}
        for start in (0x05, 0x09, 0x0D):
            regs = self._read_registers(start)
            if regs is None:
                self._out(f'讀取暫存器 0x{start:02X} 逾時(未收到 0x5F 回覆),'
                          '請確認連線與鮑率。')
                return
            for i, v in enumerate(regs.values):
                values[start + i] = v
        self._out('== IMU 內部偏移暫存器(唯讀)==')
        for addr, name in OFFSET_REGISTERS:
            self._out(f'  {name} (0x{addr:02X}) = {values[addr]:6d}')

    def sw_static(self) -> None:
        """Software gyro/accel static bias calibration written to YAML."""
        self._out('== 軟體靜態偏移校正 ==')
        self._out('請將 IMU 水平靜置(Z 軸朝上),取樣期間不要觸碰。')
        self._ask('準備好後按 Enter 開始...')
        accels: List[wp.Accel] = []
        gyros: List[wp.Gyro] = []
        self._out(f'取樣中(各 {STATIC_SAMPLES} 筆),請保持不動...')
        shown = -1
        for p in self._packets_for(STATIC_TIMEOUT):
            if isinstance(p, wp.Accel) and len(accels) < STATIC_SAMPLES:
                accels.append(p)
            elif isinstance(p, wp.Gyro) and len(gyros) < STATIC_SAMPLES:
                gyros.append(p)
            done = min(len(accels), len(gyros))
            if done != shown:
                self._status(f'取樣 {progress_bar(done / STATIC_SAMPLES)}  '
                             f'{done}/{STATIC_SAMPLES} 筆')
                shown = done
            if done >= STATIC_SAMPLES:
                break
        self._end_status()
        try:
            gyro_bias, accel_bias = compute_static_bias(accels, gyros)
        except ValueError as exc:
            logger.error('static calibration failed: %s', exc)
            self._out(f'校正失敗:{exc}(請確認序列埠與鮑率)')
            return
        level = math.sqrt(sum(b * b for b in accel_bias)) <= LARGE_ACCEL_BIAS
        if not level:
            self._out('警告:加速度偏移量偏大,IMU 可能沒有水平放置。')
        cal = dataclasses.replace(self._load_existing(),
                                  gyro_bias=gyro_bias, accel_bias=accel_bias)
        self._save(cal)
        self._step_ok = level

    def sw_mag(self) -> None:
        """Software magnetometer hard-iron calibration written to YAML."""
        if not self._mag_precheck():
            return
        self._out('== 軟體磁力計 hard-iron 校正 ==')
        self._out('按 Enter 後開始取樣,期間請將 IMU 繞三軸緩慢轉動(遠離金屬與磁鐵)。')
        for line in MAG_PROGRESS_HELP[:2]:
            self._out(line)
        self._out(f'三軸都到 100% 會自動結束(最長 {SOFT_MAG_TIMEOUT:.0f} 秒)。')
        self._ask('準備好後按 Enter 開始...')
        field_radius = self._measure_field_radius()
        cov = MagCoverage(field_radius, target=SOFT_MAG_COVERAGE_TARGET)
        mags: List[wp.Mag] = []
        start = self._time()
        next_status = start
        for p in self._packets_for(SOFT_MAG_TIMEOUT):
            now = self._time()
            if isinstance(p, wp.Mag):
                mags.append(p)
                cov.add(p)
            done = cov.complete() and len(mags) >= MIN_SAMPLES
            if done or now >= next_status:
                tail = '[完成]' if done else '缺 ' + ' '.join(cov.missing_axes())
                self._status(f'{self._axis_bars(cov)}  {now - start:3.0f}秒  '
                             f'{len(mags)}筆  {tail}')
                next_status = now + STATUS_INTERVAL
            if done:
                break
        self._end_status()
        if mags and not cov.complete():
            self._out(f'警告:{"、".join(cov.missing_axes())} 軸涵蓋不足,'
                      'hard-iron 偏移可能不準,建議重做。')
        try:
            offset = compute_mag_offset(mags)
        except ValueError as exc:
            logger.error('mag calibration failed: %s', exc)
            self._out(f'校正失敗:{exc}(請確認序列埠與鮑率)')
            return
        cal = dataclasses.replace(self._load_existing(), mag_offset=offset)
        self._save(cal)
        self._step_ok = cov.complete()

    def attitude_view(self) -> None:
        """Run the calibration check, or open the free attitude view."""
        self._out('== 校正檢測 / 姿態檢視(唯讀)==')
        choices = {'': ('accel', 'gyro_still', 'gyro_turn', 'mag'), '1': ('accel',),
                   '2': ('gyro_still',), '3': ('gyro_turn',), '4': ('mag',)}
        cmd = self._ask('Enter = 完整檢測(約 10 分鐘)  1 加速度計  2 陀螺儀靜止(5 分鐘)  '
                        '3 陀螺儀轉一圈  4 磁力計  5 環境 / 安裝位置  f = 姿態檢視  '
                        'q = 返回:').lower()
        if cmd == 'q':
            return
        if cmd == '5':
            self._mag_precheck(report_only=True)
            return
        if cmd == 'f':
            self._attitude_free_view()
            return
        if cmd not in choices:
            self._out(f'無效的選擇 {cmd!r}。')
            return
        self.calibration_check(choices[cmd])

    # ---------------------------------------------------- calibration check

    def _mark(self, status: str, text: str) -> str:
        line = f'[{status}] {text}'
        if self._color:
            return f'{ANSI_COLOURS[status]}{line}{ANSI_RESET}'
        return line

    def _verify_redraw(self) -> None:
        """On a terminal, clear the screen and show only what is still relevant."""
        if not self._color:
            return
        self._out(CLEAR_SCREEN + self._verify_title)
        for line in self._verify_lines:
            self._out(line)

    def _verify_note(self, line: str) -> None:
        """Print a line and keep it for the next redraw."""
        self._verify_lines.append(line)
        self._out(line)

    def _record(self, results: List[Tuple[str, str]], status: str, text: str) -> None:
        results.append((status, text))
        self._verify_note(self._mark(status, text))

    def _corrected(self, p):
        """Apply the software calibration, as the ROS driver does."""
        cal = self._check_cal
        if isinstance(p, wp.Accel):
            return cal.apply_accel(p)
        if isinstance(p, wp.Gyro):
            return cal.apply_gyro(p)
        if isinstance(p, wp.Mag):
            return cal.apply_mag(p)
        return p

    def _watch(self, prompt: str, status, on_packet=None, done=None) -> Optional[str]:
        """
        Show ``prompt`` and keep one live line updated from the IMU.

        ``on_packet(packet, now)`` sees every (software-corrected) packet,
        ``status(now)`` returns the live line and ``done(now)`` ends the wait.
        Return None when ``done`` ended it, else the typed line; raise
        _VerifyQuit on q.
        """
        self._verify_redraw()
        self._out(prompt)
        parser = wp.FrameParser()
        self.ser.reset_input_buffer()
        next_status = self._time()
        try:
            while True:
                for p in self._read_stream(parser):
                    p = self._corrected(p)
                    if on_packet is not None:
                        on_packet(p, self._time())
                now = self._time()
                if done is not None and done(now):
                    return None
                if now >= next_status:
                    self._status(status(now))
                    next_status = now + ATTITUDE_INTERVAL
                if self._key_ready():
                    cmd = self._ask('').lower()
                    if cmd == 'q':
                        raise _VerifyQuit
                    return cmd
        finally:
            self._end_status()

    @staticmethod
    def _still(gyro) -> bool:
        if gyro is None:
            return False
        rate = math.degrees(math.sqrt(gyro.x ** 2 + gyro.y ** 2 + gyro.z ** 2))
        return rate < STILL_GYRO_DPS

    # -- 1. accelerometer: six faces against gravity

    def _check_accel_face(self, axis: str, sign: int, index: int) -> Optional[cc.FaceResult]:
        label = cc.face_label(axis, sign)
        st = {'gyro': None, 'accel': None, 'hold': None, 'start': None, 'samples': []}

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
                return
            if not isinstance(p, wp.Accel):
                return
            st['accel'] = p
            if not (cc.facing((p.x, p.y, p.z), axis, sign) and self._still(st['gyro'])):
                st['hold'] = st['start'] = None
                st['samples'] = []
                return
            if st['hold'] is None:
                st['hold'] = now
            if st['start'] is None and now - st['hold'] >= STILL_HOLD_S:
                st['start'] = now
            if st['start'] is not None:
                st['samples'].append((p.x, p.y, p.z))

        def done(now):
            return (st['start'] is not None and now - st['start'] >= FACE_SAMPLE_S
                    and len(st['samples']) >= 3)

        def status(now):
            a = st['accel']
            if a is None:
                return '等待加速度資料...'
            vec = (a.x, a.y, a.z)
            vals = (f'accel=({a.x:+6.2f},{a.y:+6.2f},{a.z:+6.2f})  '
                    f'|a| {math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2):5.2f}')
            if not cc.facing(vec, axis, sign):
                return f'目前:{cc.face_name(vec)},請改成 {label}  {vals}'
            if st['start'] is not None:
                bar = progress_bar((now - st['start']) / FACE_SAMPLE_S, 10)
                return f'量測中,保持不動 {bar}  {vals}'
            return f'方向正確,保持不動...  {vals}'

        prompt = (f'加速度計 {index}/6:讓 {label}(靠著方形盒子擺正),'
                  f'擺好不動會自動量 {FACE_SAMPLE_S:.0f} 秒(s 略過 / q 結束):')
        while True:
            cmd = self._watch(prompt, status, on_packet, done)
            if cmd is None:
                return cc.evaluate_face(axis, sign, st['samples'])
            if cmd == 's':
                return None
            self._out('這一步會自動量測:擺好、不要動即可。')

    def _check_accel(self, results) -> None:
        name = '1. 加速度計 六面測試'
        faces = {}
        for index, (axis, sign) in enumerate(cc.FACES, start=1):
            face = self._check_accel_face(axis, sign, index)
            if face is None:
                self._record(results, SKIP, name)
                return
            faces[(axis, sign)] = face
            self._verify_note('  ' + self._mark(
                PASS if face.ok else FAIL,
                f'{cc.face_label(axis, sign)}:|a| {face.magnitude:.3f} m/s²'))
        acc = cc.evaluate_six_faces(faces)
        ox, oy, oz = acc.offsets
        self._record(results, PASS if acc.ok else FAIL,
                     f'{name}:各面 |a| 與 {wp.GRAVITY:.2f} 差 ±{cc.ACCEL_MAG_TOL} 內 '
                     f'{"是" if acc.magnitudes_ok else "否"};偏移 X {ox:+.3f} Y {oy:+.3f} '
                     f'Z {oz:+.3f} m/s²(標準 ±{cc.ACCEL_OFFSET_TOL})')
        if acc.ok:
            self._check_passed.add('accel')

    # -- 2. gyroscope at rest against zero rotation

    def _check_gyro_still(self, results) -> None:
        name = '2. 陀螺儀 靜止測試'
        st = {'gyro': None, 'ref': None, 'hold': None, 'start': None, 'samples': [],
              'restarts': 0, 'accel': None}

        def start(now):
            st['start'] = now
            st['ref'] = st['accel']
            st['samples'] = []

        def moved(p) -> bool:
            ref = st['ref']
            return ref is not None and any(
                abs(a - b) > MOVE_ACCEL for a, b in zip((p.x, p.y, p.z), (ref.x, ref.y, ref.z)))

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
                if st['start'] is not None:
                    st['samples'].append((p.x, p.y, p.z))
            elif isinstance(p, wp.Accel):
                st['accel'] = p
                if st['start'] is not None:
                    if moved(p):
                        st['start'] = st['hold'] = None
                        st['restarts'] += 1
                elif not self._still(st['gyro']):
                    st['hold'] = None
                elif st['hold'] is None:
                    st['hold'] = now
                elif now - st['hold'] >= STILL_HOLD_S:
                    start(now)

        def done(now):
            return (st['start'] is not None and now - st['start'] >= GYRO_STILL_S
                    and len(st['samples']) >= 10)

        def status(now):
            if st['start'] is None:
                note = '(偵測到移動,已重新開始)' if st['restarts'] else ''
                return f'等待靜止...{note}'
            left = max(0.0, GYRO_STILL_S - (now - st['start']))
            bar = progress_bar((now - st['start']) / GYRO_STILL_S, 10)
            mean = cc.gyro_bias_dps(st['samples']) if st['samples'] else (0.0, 0.0, 0.0)
            return (f'量測中 {bar} 剩 {int(left) // 60}:{int(left) % 60:02d}  '
                    f'平均 X {mean[0]:+.3f} Y {mean[1]:+.3f} Z {mean[2]:+.3f} °/s')

        prompt = (f'陀螺儀靜止:把 IMU 放在桌上不要碰,靜止後自動量 {GYRO_STILL_S / 60:.0f} 分鐘'
                  '(Enter 立即開始 / s 略過 / q 結束):')
        while True:
            cmd = self._watch(prompt, status, on_packet, done)
            if cmd is None:
                break
            if cmd == 's':
                self._record(results, SKIP, name)
                return
            if cmd == '' and st['accel'] is not None:
                start(self._time())
        bias = cc.gyro_bias_dps(st['samples'])
        ok = cc.gyro_bias_ok(bias)
        self._record(results, PASS if ok else FAIL,
                     f'{name}:平均 X {bias[0]:+.3f} Y {bias[1]:+.3f} Z {bias[2]:+.3f} °/s'
                     f'(標準 ±{cc.GYRO_BIAS_TOL_DPS};Z 軸換算每分鐘漂 '
                     f'{abs(bias[2]) * 60:.1f}°)')
        if ok:
            self._check_passed.add('gyro_still')

    # -- 3. gyroscope: one full turn against a reference line

    def _check_gyro_turn(self, results) -> None:
        name = '3. 陀螺儀 轉一圈測試'
        cmd = self._watch('陀螺儀轉一圈:IMU 平放,對齊桌上的參考線(桌邊或一條膠帶),'
                          '準備好按 Enter(s 略過 / q 結束):',
                          lambda now: '對齊參考線後按 Enter')
        if cmd == 's':
            self._record(results, SKIP, name)
            return
        integ = cc.TurnIntegrator()
        st = {'rate': 0.0}

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['rate'] = math.degrees(p.z)
                integ.add(now, p.z)

        def status(now):
            fast = '  太快了,請放慢' if abs(st['rate']) > FAST_TURN_DPS else ''
            return (f'已轉 {integ.angle_deg:+7.1f}°(目標 ±{cc.TURN_TARGET_DEG:.0f}°)'
                    f'  轉速 {st["rate"]:+6.1f}°/s{fast}')

        while True:
            cmd = self._watch('  水平慢慢轉一整圈(10 秒以上),對回同一條參考線後按 Enter。'
                              '請看參考線對齊,不要看數字(s 略過 / q 結束):',
                              status, on_packet)
            if cmd == 's':
                self._record(results, SKIP, name)
                return
            if cmd == '':
                break
        angle = integ.angle_deg
        ok = cc.turn_ok(angle)
        self._record(results, PASS if ok else FAIL,
                     f'{name}:陀螺儀累加 {angle:+.1f}°(標準 ±{cc.TURN_TARGET_DEG:.0f}° '
                     f'±{cc.TURN_TOL_DEG:.0f}°)')
        if ok:
            self._check_passed.add('gyro_turn')

    # -- 4. magnetometer: constant |B| and the local dip angle

    def _ask_dip(self) -> Optional[float]:
        if self._expected_dip is not None:
            return self._expected_dip
        self._verify_redraw()
        self._out(f'磁傾角:到 NOAA 地磁計算器輸入所在位置,查 Inclination:{NOAA_CALCULATOR}')
        self._out('  (台灣約 30–37°,依地點不同)')
        while True:
            text = self._ask('輸入當地磁傾角(度);直接 Enter 略過磁傾角判定:')
            if text.lower() == 'q':
                raise _VerifyQuit
            if not text:
                return None
            try:
                self._expected_dip = cc.parse_dip(text)
                return self._expected_dip
            except ValueError:
                self._out('請輸入 -90 到 90 之間的數字,例如 35.8。')

    def _check_mag(self, results) -> None:
        name_norm = '4. 磁力計 合成大小'
        name_dip = '5. 磁力計 磁傾角'
        expected = self._ask_dip()
        cov = MagCoverage(None, target=MAG_COVERAGE_TARGET)
        st = {'gyro': None, 'accel': None, 'mags': [], 'pairs': []}

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
            elif isinstance(p, wp.Accel):
                st['accel'] = p
            elif isinstance(p, wp.Mag):
                vec = (p.x, p.y, p.z)
                st['mags'].append(vec)
                cov.add(p)
                a, g = st['accel'], st['gyro']
                if a is not None and g is not None:
                    rate = math.degrees(math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2))
                    norm_a = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
                    if rate < DIP_MAX_RATE_DPS and abs(norm_a - wp.GRAVITY) < DIP_ACCEL_TOL:
                        st['pairs'].append(((a.x, a.y, a.z), vec))

        def done(now):
            return (cov.complete() and len(st['mags']) >= MAG_MIN_SAMPLES
                    and len(st['pairs']) >= DIP_MIN_PAIRS)

        def status(now):
            tail = ('缺 ' + ' '.join(cov.missing_axes())) if not cov.complete() else '轉慢一點'
            return f'{self._axis_bars(cov)}  {len(st["mags"])} 筆  {tail}'

        cmd = self._watch('磁力計:往各個方向慢慢轉(每一面都朝上、朝下過),三條進度條都滿會自動結束'
                          '(Enter 提早結束 / s 略過 / q 結束):', status, on_packet, done)
        if cmd == 's':
            self._record(results, SKIP, name_norm)
            self._record(results, SKIP, name_dip)
            return
        if not cov.complete() or len(st['mags']) < MAG_MIN_SAMPLES:
            self._record(results, SKIP, f'{name_norm}:方向涵蓋不足,無法判定')
        else:
            spread = cc.magnitude_spread(st['mags'])
            ok = cc.magnitude_ok(spread)
            self._record(results, PASS if ok else FAIL,
                         f'{name_norm}:{cc.MAG_SPREAD_SHARE:.0%} 的讀數在平均值 '
                         f'±{spread:.1%} 內(標準 ±{cc.MAG_SPREAD_TOL:.0%})')
            if ok:
                self._check_passed.add('mag_norm')
        if expected is None:
            self._record(results, SKIP, f'{name_dip}:未輸入當地磁傾角')
        elif len(st['pairs']) < DIP_MIN_PAIRS:
            self._record(results, SKIP, f'{name_dip}:慢速轉動的資料不足,無法判定')
        else:
            dip = cc.mean_dip(st['pairs'])
            ok = cc.dip_ok(dip, expected)
            self._record(results, PASS if ok else FAIL,
                         f'{name_dip}:量到 {dip:+.1f}°,當地 {expected:+.1f}°'
                         f'(標準 ±{cc.DIP_TOL_DEG:.0f}°)')
            if ok:
                self._check_passed.add('mag_dip')

    # -------------------------------------- environment / mounting precheck

    def _record_still(self, prompt: str, seconds: float, need_still: bool = True):
        """
        Record ``seconds`` of magnetometer and accelerometer data.

        With ``need_still`` the recording starts once the IMU has been still for
        STILL_HOLD_S and restarts if it moves.  Return a dict with 'mags' and
        'accels', or None when skipped.
        """
        st = {'gyro': None, 'hold': None, 'start': None, 'mags': [], 'accels': []}

        def reset():
            st['hold'] = st['start'] = None
            st['mags'] = []
            st['accels'] = []

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
                if need_still and not self._still(p):
                    reset()
                return
            if st['start'] is None:
                if need_still and not self._still(st['gyro']):
                    return
                if st['hold'] is None:
                    st['hold'] = now
                if not need_still or now - st['hold'] >= STILL_HOLD_S:
                    st['start'] = now
            if st['start'] is None:
                return
            if isinstance(p, wp.Mag):
                st['mags'].append((p.x, p.y, p.z))
            elif isinstance(p, wp.Accel):
                st['accels'].append((p.x, p.y, p.z))

        def done(now):
            return (st['start'] is not None and now - st['start'] >= seconds
                    and len(st['mags']) >= 3 and len(st['accels']) >= 3)

        def status(now):
            if st['start'] is None:
                return '等待靜止...'
            bar = progress_bar((now - st['start']) / seconds, 10)
            return f'記錄中 {bar}  {len(st["mags"])} 筆'

        while True:
            cmd = self._watch(prompt, status, on_packet, done)
            if cmd is None:
                return st
            if cmd == 's':
                return None
            self._out('這一步會自動記錄,不用按 Enter。')

    def _precheck_still(self, results, hints) -> Optional[Tuple[float, float, float]]:
        """Return the mean acceleration at rest (for the mounting tilt), or None."""
        name = '1. 磁場穩定'
        data = self._record_still(
            f'環境 1/2 靜止:IMU 放在要校正的位置,放著不動,自動記錄 {ENV_STILL_S:.0f} 秒'
            '(s 略過 / q 結束):', ENV_STILL_S)
        if data is None:
            self._record(results, SKIP, name)
            return None
        noise = cc.field_noise(data['mags'])
        ok = cc.noise_ok(noise)
        self._record(results, PASS if ok else FAIL,
                     f'{name}:靜止時磁場變動 {noise:.1%}(標準 {cc.ENV_NOISE_TOL:.0%} 內)')
        if not ok:
            hints.append('磁場在變動:附近有馬達在轉、電流在變,或有人拿著金屬、手機經過。'
                         '關掉附近的電器、請旁人離開再試。')
        n = float(len(data['accels']))
        return tuple(sum(a[i] for a in data['accels']) / n for i in range(3))

    def _precheck_slide(self, results, hints) -> None:
        name = '2. 附近有無鐵件'
        while True:
            cmd = self._watch('環境 2/2 平移:IMU(或整台車)擺在起點,按 Enter 開始'
                              '(s 略過 / q 結束):', lambda now: '準備好後按 Enter')
            if cmd == 's':
                self._record(results, SKIP, name)
                return
            if cmd != '':
                continue
            meter = cc.RotationMeter()
            st = {'ref': [], 'mags': []}

            def on_packet(p, now):
                if isinstance(p, wp.Gyro):
                    meter.add(now, (p.x, p.y, p.z))
                elif isinstance(p, wp.Mag):
                    vec = (p.x, p.y, p.z)
                    if len(st['ref']) < 3:
                        st['ref'].append(vec)
                    st['mags'].append(vec)

            def status(now):
                change = (f'{cc.field_change(st["ref"], st["mags"]):.1%}'
                          if len(st['ref']) >= 3 else '--')
                warn = '  轉動了,請保持方向不變' if not cc.slide_valid(meter.total_deg) else ''
                return f'磁場變化 {change}  已轉動 {meter.total_deg:.1f}°{warn}'

            cmd = self._watch(f'  不要轉動,沿桌面或地面平移約 {SLIDE_DISTANCE_CM} cm,停住後按 Enter'
                              '(s 略過 / q 結束):', status, on_packet)
            if cmd == 's':
                self._record(results, SKIP, name)
                return
            if len(st['ref']) < 3 or len(st['mags']) < 5:
                self._verify_note('  資料不足,請再做一次(平移時間久一點)。')
                continue
            if not cc.slide_valid(meter.total_deg):
                self._verify_note(f'  平移時轉了 {meter.total_deg:.1f}°'
                                  f'(要在 {cc.SLIDE_MAX_ROTATION_DEG:.0f}° 內),這次不算,請再做一次。')
                continue
            change = cc.field_change(st['ref'], st['mags'])
            ok = cc.change_ok(change)
            self._record(results, PASS if ok else FAIL,
                         f'{name}:平移時磁場變化 {change:.1%}(標準 {cc.ENV_CHANGE_TOL:.0%} 內)')
            if not ok:
                hints.append('平移時磁場跟著位置改變:附近有鐵件(鐵桌、鐵櫃、筆電、含鋼筋的地板)。'
                             '換到木桌或空曠處,或把 IMU 墊高遠離地面。')
            return

    def _precheck_mounting(self, results, hints, accel_mean) -> None:
        if accel_mean is not None:
            tilt = cc.tilt_deg(accel_mean)
            name = '3. 安裝方向'
            if tilt <= cc.TILT_NOTE_DEG:
                self._record(results, PASS, f'{name}:元件面朝上,傾斜 {tilt:.1f}°')
            elif tilt < 45.0:
                self._record(results, NOTE, f'{name}:傾斜 {tilt:.1f}°。小角度可用選項 6 修正,'
                             '或在 ROS 的 TF 設定安裝角度')
            else:
                self._record(results, NOTE, f'{name}:元件面沒有朝上(偏 {tilt:.0f}°),'
                             'ROS 的 TF 要設定安裝角度')
        names = ('4. 馬達干擾', '5. 振動')
        base = self._record_still(
            f'安裝 1/2:車子通電,把車架高讓輪子懸空,馬達先不要轉。自動記錄 {MOTOR_BASE_S:.0f} 秒'
            '(s 略過 / q 結束):', MOTOR_BASE_S)
        cmd = None if base is None else self._watch(
            '安裝 2/2:讓馬達以平常的速度轉起來,轉穩後按 Enter(s 略過 / q 結束):',
            lambda now: '馬達轉穩後按 Enter')
        running = None if base is None or cmd == 's' else self._record_still(
            f'  量測中,馬達保持轉動,自動記錄 {MOTOR_RUN_S:.0f} 秒(s 略過 / q 結束):',
            MOTOR_RUN_S, need_still=False)
        if running is None:
            for name in names:
                self._record(results, SKIP, name)
            return
        change = cc.field_change(base['mags'], running['mags'])
        ok = cc.change_ok(change)
        self._record(results, PASS if ok else FAIL,
                     f'{names[0]}:馬達轉動時磁場變化 {change:.1%}(標準 {cc.ENV_CHANGE_TOL:.0%} 內)')
        if not ok:
            hints.append('馬達運轉時磁場改變:IMU 離馬達或電源線太近。'
                         '移遠一點(至少 10 cm,越遠越好)或墊高。這會讓 yaw 不準。')
        vib = cc.vibration(running['accels'])
        ok = cc.vibration_ok(vib)
        self._record(results, PASS if ok else FAIL,
                     f'{names[1]}:馬達轉動時 {vib:.2f} m/s²(標準 {cc.VIBRATION_TOL} 內)')
        if not ok:
            hints.append('振動大:IMU 沒有鎖緊,或太靠近馬達。鎖緊固定、靠近車體中心,'
                         '必要時加一層薄的減震墊。')

    def _mag_precheck(self, report_only: bool = False) -> bool:
        """
        Check the surroundings and the mounting position from the IMU's own readings.

        Return True to go on with the magnetometer calibration.  When something
        is unsuitable the user is told why and asked twice before going on.
        """
        if self._key_ready is None:
            self._out('(環境與安裝位置檢查需要在終端機中執行,已略過)')
            return True
        self._verify_title = '== 校正前檢查:環境與安裝位置 =='
        self._verify_lines = []
        for line in PRECHECK_INTRO:
            self._verify_note(line)
        results: List[Tuple[str, str]] = []
        hints: List[str] = []
        try:
            accel_mean = self._precheck_still(results, hints)
            self._precheck_slide(results, hints)
            if self._confirm('IMU 已經裝在車上的最終位置了嗎?'):
                self._precheck_mounting(results, hints, accel_mean)
            else:
                self._verify_note('IMU 還沒裝上車:裝上去之後,要在最終位置重新檢查與校正。')
        except _VerifyQuit:
            self._out('已結束檢查。')
            if report_only:
                return True
            self._out('已取消校正,未送出任何指令。')
            return False
        except (serial.SerialException, OSError) as exc:
            logger.error('reading IMU data in the environment check failed: %s', exc)
            self._out(f'讀取 IMU 資料失敗:{exc}')
            return report_only
        finally:
            self._verify_title = CHECK_TITLE
        self._out((CLEAR_SCREEN if self._color else '') + '== 環境與安裝位置 檢查結果 ==')
        for status, text in results:
            self._out(self._mark(status, text))
        if not any(status == FAIL for status, _ in results):
            if any(status == PASS for status, _ in results):
                self._out(self._mark(PASS, '環境與安裝位置合適'))
            return True
        self._out('不適合的原因與改善方式:')
        for hint in hints:
            self._out(f'  - {hint}')
        if report_only:
            return True
        if not self._confirm('目前的環境或安裝位置不適合。仍要在這裡繼續校正嗎?'):
            self._out('已取消校正,未送出任何指令。')
            return False
        if not self._confirm('再確認一次:在不適合的環境或位置校正,結果會不準,'
                             '裝上車後也可能不穩定。確定繼續?'):
            self._out('已取消校正,未送出任何指令。')
            return False
        self._out('依你的選擇,繼續校正。')
        return True

    def calibration_check(self, items=('accel', 'gyro_still', 'gyro_turn', 'mag')) -> None:
        """Run the reference-based checks; read-only."""
        if self._key_ready is None:
            self._out('校正檢測需要在終端機中執行(python3 imu.py → 4 IMU 校正工具)。')
            return
        self._verify_lines = []
        self._verify_title = CHECK_TITLE
        self._check_cal = self._load_existing()
        for line in CHECK_INTRO:
            self._verify_note(line)
        if os.path.exists(str(self.config_path)):
            self._verify_note(f'已套用軟體校正:{self.config_path}')
        else:
            self._verify_note('沒有軟體校正檔,以模組輸出判定。')
        results: List[Tuple[str, str]] = []
        steps = {'accel': self._check_accel, 'gyro_still': self._check_gyro_still,
                 'gyro_turn': self._check_gyro_turn, 'mag': self._check_mag}
        try:
            for item in items:
                steps[item](results)
        except _VerifyQuit:
            self._out('已結束檢測。')
        except (serial.SerialException, OSError) as exc:
            logger.error('reading IMU data in the calibration check failed: %s', exc)
            self._out(f'讀取 IMU 資料失敗:{exc}')
        finally:
            self._check_cal = SoftwareCalibration()
        if not results:
            return
        self._out((CLEAR_SCREEN if self._color else '') + '== 檢測結果 ==')
        for status, text in results:
            self._out(self._mark(status, text))
        if FAIL in [status for status, _ in results]:
            for line in CHECK_FAIL_HINTS:
                self._out(line)
        passed = len(self._check_passed)
        if passed == len(CHECK_ITEMS):
            self._out(self._mark(PASS, f'{passed}/{len(CHECK_ITEMS)} 項全部合格:這顆 IMU 校正良好'))
        else:
            self._out(f'本次已合格 {passed}/{len(CHECK_ITEMS)} 項;五項都合格才算校正良好。')
        self._step_ok = passed == len(CHECK_ITEMS)

    def _attitude_free_view(self) -> None:
        """Show live roll / pitch / yaw / height, optionally relative to a reference."""
        for line in ATTITUDE_HELP:
            self._out(line)
        lock = threading.Lock()
        state = {'angles': None, 'height': None, 'ref': None}
        stop = threading.Event()
        errors: List[BaseException] = []

        def reader() -> None:
            parser = wp.FrameParser()
            next_status = self._time()
            try:
                while True:     # read at least once, even if 'q' comes immediately
                    for p in self._read_stream(parser):
                        with lock:
                            if isinstance(p, wp.Angle):
                                state['angles'] = (math.degrees(p.roll), math.degrees(p.pitch),
                                                   math.degrees(p.yaw))
                            elif isinstance(p, wp.Pressure):
                                state['height'] = p.height_m
                    now = self._time()
                    if now >= next_status:
                        with lock:
                            angles, height, ref = state['angles'], state['height'], state['ref']
                        self._status('等待角度資料...' if angles is None
                                     else format_attitude(angles, height, ref))
                        next_status = now + ATTITUDE_INTERVAL
                    if stop.wait(READER_YIELD):
                        break
            except (serial.SerialException, OSError) as exc:
                logger.error('reading IMU data in attitude view failed: %s', exc)
                errors.append(exc)

        thread = threading.Thread(target=reader, name='imu-attitude-view', daemon=True)
        thread.start()
        try:
            while not errors:
                cmd = self._ask('').lower()
                if cmd == 'q':
                    break
                if self._color:
                    # Keep a single updating line: undo the echoed line break.
                    self._out(CURSOR_UP_CLEAR, end='', flush=True)
                if cmd == 'r':
                    with lock:
                        state['ref'] = None
                    if not self._color:
                        self._out('已清除參考點,顯示目前值。')
                elif cmd == '':
                    with lock:
                        angles, height = state['angles'], state['height']
                        if angles is not None:
                            state['ref'] = (angles, height)
                    if angles is None:
                        self._out('尚未收到角度資料,無法設定參考點。')
                    elif not self._color:
                        self._out('參考點已設定:' + format_attitude(angles, height))
                else:
                    self._out(f'無效的指令 {cmd!r}:Enter 設定參考點、r 清除、q 離開。')
        finally:
            stop.set()
            thread.join(READER_JOIN_TIMEOUT)
            self._end_status()
        if errors:
            self._out(f'讀取 IMU 資料失敗:{errors[0]}')

    def toggle_pressure_output(self) -> None:
        """Turn the 0x56 pressure / height output on or off (RSW bit 6, saved to flash)."""
        self._out('== 氣壓高度輸出 (0x56) ==')
        regs = self._read_registers(wp.REG_RSW)
        if regs is None:
            self._out('讀取輸出設定 (RSW) 逾時,未做任何變更;請確認連線與鮑率。')
            return
        rsw = regs.values[0] & 0xFFFF
        enabled = bool(rsw & PRESS_BIT)
        self._out(f'目前 RSW = 0x{rsw:04X},氣壓高度輸出:{"已開啟" if enabled else "未開啟"}')
        new = rsw & ~PRESS_BIT if enabled else rsw | PRESS_BIT
        action = '關閉' if enabled else '開啟'
        if not enabled:
            self._out('開啟後每秒多 10 個封包;9600 baud 下約使用 57% 頻寬,仍足夠。')
        if not self._confirm(f'要{action}氣壓高度輸出嗎?(會寫入 IMU flash)'):
            self._out('已取消,未變更設定。')
            return
        self._unlock()
        self._send(wp.cmd_write(wp.REG_RSW, new))
        self._send(wp.cmd_save())
        self._out(f'已{action}氣壓高度輸出(RSW = 0x{new:04X})。'
                  '重新啟動 IMU 節點後 /imu/pressure 才會更新。')

    def factory_reset(self) -> None:
        """Restore factory defaults after two confirmations."""
        self._out('== 恢復原廠設定 ==')
        self._out('警告:所有設定(含硬體校正值、輸出內容、速率、鮑率)都會恢復預設。')
        if not self._confirm('確定要恢復原廠設定?'):
            self._out('已取消,未送出任何指令。')
            return
        if not self._confirm('再次確認:此動作無法復原,真的要執行?'):
            self._out('已取消,未送出任何指令。')
            return
        self._unlock()
        self._send(wp.cmd_factory_reset())
        self._out('已送出恢復原廠設定指令;建議重新上電,並以預設 9600 baud 連線。')
        self._done.clear()               # every earlier calibration is gone
        self._check_passed.clear()

    # ------------------------------------------------------------ main loop

    def _show_notes(self, choice: str) -> None:
        """Print placement precautions for a calibration item, if any."""
        notes = NOTES.get(choice)
        if not notes:
            return
        self._out('注意事項:')
        for line in notes:
            self._out(line)

    def run(self) -> int:
        """Run the menu loop and return a process exit code."""
        while True:
            self._out(render_menu(self._done, self._color))
            try:
                choice = self._ask('請選擇: ')
            except EOFError:
                self._out('\n輸入結束,離開。')
                return 0
            except KeyboardInterrupt:
                self._out('\n已中斷,離開。')
                return 130
            if choice == '0':
                return 0
            action = self._actions.get(choice)
            if action is None:
                self._out(f'無效的選項:{choice!r},請輸入 0-13。')
                continue
            self._show_notes(choice)
            self._step_ok = False
            try:
                action()
                if self._step_ok:
                    self._done.add(choice)
            except EOFError:
                self._out('\n輸入結束,離開。')
                return 0
            except KeyboardInterrupt:
                self._out('\n已中止,返回選單。')
            except (serial.SerialException, OSError) as exc:
                logger.error('action %s failed: %s', choice, exc)
                self._out(f'操作失敗:{exc}')


def wrap_degrees(value: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (value + 180.0) % 360.0 - 180.0


def format_attitude(angles, height: Optional[float], ref=None) -> str:
    """
    Format roll / pitch / yaw (degrees) and height (m) as one status line.

    With ``ref = ((roll, pitch, yaw), height)`` the line shows the change
    relative to the reference instead (angles wrapped to +-180 degrees).
    """
    r, p, y = angles
    if ref is None:
        h = '高度 (未輸出)' if height is None else f'高度 {height:+7.2f} m'
        return f'roll(X) {r:+7.2f}°  pitch(Y) {p:+7.2f}°  yaw(Z) {y:+7.2f}°  {h}'
    (r0, p0, y0), h0 = ref
    dr, dp, dy = (wrap_degrees(a - b) for a, b in ((r, r0), (p, p0), (y, y0)))
    dh = 'Δ高度 (未輸出)' if height is None or h0 is None else f'Δ高度 {height - h0:+7.2f} m'
    return f'Δroll(X) {dr:+7.2f}°  Δpitch(Y) {dp:+7.2f}°  Δyaw(Z) {dy:+7.2f}°  {dh}'


def render_menu(done=(), color: bool = False) -> str:
    """Return the menu with the rows in ``done`` marked (green on a terminal)."""
    lines = []
    for line in MENU.split('\n'):
        match = _MENU_ROW.match(line)
        if match and match.group(1) in done:
            line = f'{line}  {DONE_MARK}'
            if color:
                line = f'{ANSI_COLOURS[PASS]}{line}{ANSI_RESET}'
        lines.append(line)
    return '\n'.join(lines)


def mag_coverage_target(mode: wp.CalMode) -> float:
    """Return the per-axis swing fraction that counts as done for ``mode``."""
    if mode == wp.CalMode.MAG_DUAL_PLANE:
        return DUAL_PLANE_COVERAGE_TARGET
    return MAG_COVERAGE_TARGET


def progress_bar(fraction: float, width: int = PROGRESS_WIDTH) -> str:
    """Return an ASCII progress bar such as ``[#####-----]  50%`` (input clamped to 0..1)."""
    if not math.isfinite(fraction):
        fraction = 0.0
    fraction = min(1.0, max(0.0, fraction))
    filled = int(fraction * width + 1e-9)
    percent = int(fraction * 100 + 1e-9)     # floor: 99.97 % must not read as 100 %
    return f'[{"#" * filled}{"-" * (width - filled)}] {percent:3d}%'


def _fmt3(v, digits: int) -> str:
    return '[' + ', '.join(f'{x:.{digits}f}' for x in v) + ']'


def _format_live(angle: wp.Angle, acc: Optional[wp.Accel], gyro: Optional[wp.Gyro],
                 mag: Optional[wp.Mag]) -> str:
    parts = ['roll={:8.2f} pitch={:8.2f} yaw={:8.2f} deg'.format(
        math.degrees(angle.roll), math.degrees(angle.pitch), math.degrees(angle.yaw))]
    if acc is not None:
        parts.append(f'accel=({acc.x:6.2f},{acc.y:6.2f},{acc.z:6.2f}) m/s^2')
    if gyro is not None:
        parts.append('gyro=({:7.2f},{:7.2f},{:7.2f}) deg/s'.format(
            math.degrees(gyro.x), math.degrees(gyro.y), math.degrees(gyro.z)))
    if mag is not None:
        parts.append(f'mag=({mag.x},{mag.y},{mag.z}) LSB')
    return ' | '.join(parts)


def run_menu(ser, input_fn: Callable[[str], str] = input,
             output_fn: Callable[..., None] = print, config_path=DEFAULT_CONFIG, *,
             sleep_fn: Callable[[float], None] = time.sleep,
             time_fn: Callable[[], float] = time.monotonic, color: bool = False,
             key_ready: Optional[Callable[[], bool]] = None) -> int:
    """Run the interactive menu on an already opened serial port."""
    return CalibrationMenu(ser, input_fn, output_fn, config_path,
                           sleep_fn=sleep_fn, time_fn=time_fn, color=color,
                           key_ready=key_ready).run()


def stdin_ready() -> bool:
    """Return True when a typed line is waiting on standard input."""
    try:
        return bool(select.select([sys.stdin], [], [], 0)[0])
    except (OSError, ValueError):
        return False


def use_color(stream=None, environ=None) -> bool:
    """Return True when ANSI colours suit ``stream`` (a TTY and NO_COLOR unset)."""
    stream = sys.stdout if stream is None else stream
    environ = os.environ if environ is None else environ
    if environ.get('NO_COLOR'):
        return False
    isatty = getattr(stream, 'isatty', None)
    return bool(isatty and isatty())


def _baud(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f'baud must be an integer, got {text!r}') from None
    if not 1 <= value <= 4_000_000:
        raise argparse.ArgumentTypeError(f'baud out of range: {value}')
    return value


def _non_empty(text: str) -> str:
    if not text.strip():
        raise argparse.ArgumentTypeError('must not be empty')
    return text


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line argument parser."""
    parser = argparse.ArgumentParser(
        prog='imu_calibrate', description='Yahboom CMP10A IMU 互動校正工具')
    parser.add_argument('--port', type=_non_empty, default=DEFAULT_PORT,
                        help='序列埠或 pyserial URL,例如 socket://host.docker.internal:5760 '
                             f'(預設 {DEFAULT_PORT})')
    parser.add_argument('--baud', type=_baud, default=DEFAULT_BAUD,
                        help=f'鮑率(預設 {DEFAULT_BAUD})')
    parser.add_argument('--config', type=_non_empty, default=DEFAULT_CONFIG,
                        help=f'軟體校正 YAML 路徑(預設 {DEFAULT_CONFIG})')
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Console-script entry point (``imu_calibrate``)."""
    args = build_parser().parse_args(argv)
    try:
        ser = serial.serial_for_url(args.port, baudrate=args.baud, timeout=SERIAL_TIMEOUT)
    except (serial.SerialException, ValueError, OSError) as exc:
        logger.debug('open failed', exc_info=True)
        print(f'錯誤:無法開啟序列埠 {args.port} ({args.baud} baud):{exc}', file=sys.stderr)
        print('請確認裝置已連接、路徑正確且有存取權限(容器需以 --device 傳入)。',
              file=sys.stderr)
        return 1
    try:
        terminal = use_color()
        return run_menu(ser, input, print, args.config, color=terminal,
                        key_ready=stdin_ready if terminal else None)
    finally:
        ser.close()


if __name__ == '__main__':
    sys.exit(main())
