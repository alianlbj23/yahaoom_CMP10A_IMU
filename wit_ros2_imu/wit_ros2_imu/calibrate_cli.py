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
from .calibration import (load_calibration, PlaneCoverage,
                          save_calibration, SoftwareCalibration)

logger = logging.getLogger(__name__)

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 9600
DEFAULT_CONFIG = '/config/imu_calibration.yaml'
SERIAL_TIMEOUT = 0.5

READ_CHUNK = 64
CMD_GAP = 0.1               # s between consecutive commands
UNLOCK_WINDOW = 10.0        # s the device stays unlocked
UNLOCK_MARGIN = 2.0         # re-unlock if less than this is left
# The steps below follow the WitMotion PC software and its manual
# (software instructions v23-0529, section 3.3 "Sensor calibration").
ACCEL_CAL_WAIT = 2.0        # s; official prompt: "keep the sensor still for 2 seconds"
SHORT_CAL_WAIT = 1.0        # s for heading / angle-reference / height zeroing
REPLY_TIMEOUT = 1.5         # s to wait for a 0x5F register reply
LEVEL_MAX_TILT_DEG = 15.0   # official dual-plane rule: tilt must not exceed 15 deg
READBACK_TIME = 1.0         # s of data shown after the accelerometer calibration
REG_AXIS6 = 0x24            # algorithm: 0 = 9-axis (absolute heading), 1 = 6-axis
REG_GYROCALITHR = 0x61      # gyro auto-calibration threshold; 0x63 GYROCALTIME follows
SPHERE_STEPS = [
    '官方步驟:',
    '  1. 先用手機在水平面轉幾圈,找出南北方向。',
    '  2. 開始後,沿南北方向的垂直面,讓 XZ、YZ、XY 三個平面各轉 1–3 圈。',
    '  3. 三個平面的進度都滿,出現 [完成] 後按 Enter,寫入參數。',
]
DUAL_PLANE_STEPS = [
    '官方步驟:',
    '  1. 先把 IMU 水平放好(傾斜不可超過 15°)。',
    '  2. 開始後,保持水平,慢慢轉一圈。',
    '  3. 再把 IMU 垂直立起來,慢慢轉一圈。',
    '  4. 出現 [完成] 後按 Enter,寫入參數。',
]
# Stages of the magnetometer progress display: (instruction, required planes).
MAG_STAGES = {
    wp.CalMode.MAG_SPHERE: [('三個平面各轉 1–3 圈', ('XY', 'XZ', 'YZ'))],
    wp.CalMode.MAG_DUAL_PLANE: [('第 1 段:保持水平,慢慢轉一圈', ('XY',)),
                                ('第 2 段:垂直立起來,慢慢轉一圈', ('XZ|YZ',))],
}
PLANE_LABEL = {'XZ|YZ': '垂直'}
PLANE_BAR_WIDTH = 6          # three plane bars must fit in 79 columns
FIELD_MEASURE_TIME = 1.5    # s of normal-mode data used to measure |B| before a mag calibration

# Progress display
STATUS_INTERVAL = 0.5       # s between progress-line refreshes
COUNTDOWN_STEP = 0.5        # s per countdown tick
NO_DATA_HINT_AFTER = 3.0    # s without gyro data before falling back to an elapsed-time hint
READER_YIELD = 0.005        # s; real-time pause per read so the Enter prompt stays responsive
READER_JOIN_TIMEOUT = 2.0   # s; > SERIAL_TIMEOUT so a blocked read can finish
PROGRESS_WIDTH = 20
ATTITUDE_INTERVAL = 0.2     # s between live-line refreshes in the environment check
PRESS_BIT = 1 << 6          # RSW bit enabling the 0x56 pressure / height packet
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
 2. 加速度計水平校正(硬體)                → 裝在車上,車停真正水平的地面
 3. 磁力計校正 橢球(硬體)                 → 離電腦 50 cm,拿在空中轉
 4. 磁力計校正 雙平面(硬體)               → 先水平、再垂直,各轉一圈
 5. Z 軸角度歸零(硬體,僅六軸)             → 靜止放置
 6. 設定角度參考(硬體)                    → 裝好後,機器人放水平地面
 7. 高度歸零(硬體,氣壓計)                 → 在要當作 0 m 的高度
 8. 讀取 IMU 內部偏移暫存器               → 任何地方(唯讀)
 9. 陀螺儀自動校正 狀態(唯讀)             → 任何地方
10. 演算法 六軸 / 九軸 切換               → 寫入 IMU 設定
11. 恢復原廠設定                          → 任何地方
12. 氣壓高度輸出 開啟/關閉                → 寫入 IMU 設定
 0. 離開
=========================================="""

# Appended to a menu row once that step has succeeded in this run of the tool.
# The marks live in memory only: a new run starts with none, so a green row
# always means "done since this tool was started", never an older calibration.
DONE_MARK = '✓ 本次已完成'
_MENU_ROW = re.compile(r'^\s*(\d+)\.')

# The menu follows the algorithm read from the module (AXIS6: 0 nine-axis, 1
# six-axis).  Rows the current algorithm cannot use are hidden; the numbers
# stay the same so the tutorial still matches.  Unknown (None) shows them all.
MODE_NAMES = {0: '九軸', 1: '六軸'}
HIDDEN_BY_MODE = {
    1: ('3', '4'),      # six-axis: the yaw does not use the magnetometer
    0: ('5',),          # nine-axis: official, Z-axis zero only works in six-axis
}
HIDDEN_NOTE = {
    1: '(3、4 磁力計校正:六軸用不到,已隱藏)',
    0: '(5 Z 軸歸零:九軸用不到,已隱藏)',
    None: '(讀不到演算法設定,全部列出)',
}
HIDDEN_REASON = {
    1: '目前是六軸:yaw 不用磁力計,不需要磁力計校正(3、4)。要用磁力計,先選 10 切換到九軸。',
    0: '目前是九軸:官方說明 Z 軸歸零只在六軸有效。要歸零,先選 10 切換到六軸。',
}


# Environment check before a magnetometer calibration.  Pass thresholds live
# in calibration_check.py; the values here only control how data is collected.
STILL_GYRO_DPS = 2.0          # |gyro| below this counts as "not moving"
STILL_HOLD_S = 1.0            # hold still this long before a measurement starts
ENV_STILL_S = 5.0
SLIDE_DISTANCE_CM = 30
MOTOR_BASE_S = 3.0
MOTOR_RUN_S = 5.0
SLIDE_MOVE_ACCEL = 0.15       # m/s^2 away from the resting reading = the IMU is being slid
SLIDE_RECENT = 5              # latest mag samples shown while the IMU is still moving
MOTOR_DETECT_VIB = 0.1        # m/s^2 std of |a| (and 3x the stopped value) = motors running
MOTOR_DETECT_CHANGE = 0.02    # field change vs. stopped (2x the rest noise) = motors running
MOTOR_SETTLE_S = 1.0          # wait after the motors are detected before recording
MOTOR_WINDOW = 10             # samples used to detect the motors
MOTOR_ENTER_AFTER_S = 10.0    # Enter starts the run by hand only after this long undetected
PRECHECK_INTRO = [
    '校正前先用 IMU 自己的讀數檢查環境與安裝位置(約 1 分鐘):',
    '  地磁在一個房間內處處相同、不隨時間改變;讀數若會變動或跟著位置改變,就是周圍有干擾。',
    '標準與理由見 docs/CALIBRATION_CHECK.md。',
]
PASS, FAIL, SKIP, NOTE = '合格', '不合格', '略過', '注意'
ANSI_COLOURS = {PASS: '\033[32m', FAIL: '\033[31m', SKIP: '\033[33m', NOTE: '\033[33m'}
ANSI_RESET = '\033[0m'


class _VerifyQuit(Exception):
    """Raised inside the environment check when the user enters q."""


# Shown before every magnetometer calibration that displays per-axis progress.
MAG_PROGRESS_HELP = [
    '下方顯示每個平面的進度(和官方軟體的 XY、XZ、YZ 三張圖一樣:畫出一圈才算):',
    '  XY 不夠 → 繞 Z 軸轉;XZ 不夠 → 繞 Y 軸轉;YZ 不夠 → 繞 X 軸轉。',
    '進度只表示轉得夠不夠,不保證校正品質。',
]

# Placement / precaution notes shown after a calibration item is chosen,
# before anything is sent to the IMU.  Keys are menu choices.
NOTES: Dict[str, List[str]] = {
    '2': [
        '- 官方:模組水平靜止放置,約 2 秒後三軸加速度應約為 0、0、1 g,X、Y 角度約 0°。',
        '- IMU 裝在車上的最終位置,車子停在真正水平的地面(用水平儀確認);兩個條件都要。',
        '- 校正期間不要碰車子,馬達要停。',
    ],
    '3': [
        '- 官方:離電腦 50 cm 以上,拿在空中轉;不要直接在桌面上轉(桌下常有鐵樑)。',
        '- 官方:常見干擾有電腦喇叭、桌下的鐵架、地板裡的鋼筋網。',
        '- IMU 已裝在車上時,連車一起轉,車上的鐵件才會一起被補償。',
    ],
    '4': [
        '- 官方:離電腦 50 cm 以上,拿在空中轉;不要直接在桌面上轉(桌下常有鐵樑)。',
        '- 適合車子太大、無法每個方向都翻轉時:只需要水平轉一圈、垂直轉一圈。',
    ],
    '5': [
        '- 官方:Z 軸歸零只在六軸演算法有效;九軸的 Z 軸是絕對角度(以磁北為準),不能歸零。',
        '- 官方:模組靜止放置後執行;六軸模式下,模組每次上電 Z 軸也會自動歸零。',
    ],
    '6': [
        '- 官方:把目前的姿態設為水平面。',
        '- 官方警告:角度參考和加速度計校正同時使用,可能造成加速度、角度異常。',
    ],
    '7': [
        '- 官方:以目前氣壓作為 0 m 高度;在你要當作 0 m 的位置執行。',
        '- CMP10A 出廠預設不輸出氣壓(0x56):請先用選項 12 開啟,',
        '  才會有 /imu/pressure 與高度。',
    ],
    '10': [
        '- 官方:九軸的航向(Z 軸)由磁力計決定,不會漂移,但怕磁場干擾;',
        '  六軸的航向由陀螺儀累加,不受磁場影響,但會慢慢漂移。',
        '- 官方:切換到六軸後,要重做加速度計校正(2)與 Z 軸歸零(5)。',
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
        self._verify_title = ''
        self._check_cal = SoftwareCalibration()
        self._done: set = set()          # menu choices that succeeded in this run
        self._axis6: Optional[int] = None  # algorithm read from the module; None = unknown
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
            '9': self.gyro_autocal_status,
            '10': self.switch_algorithm,
            '11': self.factory_reset,
            '12': self.toggle_pressure_output,
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
            self._out(f'現有校正檔格式錯誤,本次檢視未套用軟體校正:{exc}')
            return SoftwareCalibration()

    def _load_for_update(self) -> Optional[SoftwareCalibration]:
        """Refuse to replace an unreadable calibration with default offsets."""
        try:
            return load_calibration(self.config_path)
        except ValueError as exc:
            self._out(f'校正檔格式錯誤,未寫入:{exc}')
            self._out(f'請先備份並修正 {self.config_path},再重做校正。')
            return None

    def _save(self, cal: SoftwareCalibration) -> None:
        save_calibration(self.config_path, cal)
        self._out(f'已寫入 {self.config_path}')
        self._out(f'  gyro_bias  [rad/s]  = {_fmt3(cal.gyro_bias, 6)}')
        self._out(f'  accel_bias [m/s^2]  = {_fmt3(cal.accel_bias, 4)}')
        self._out(f'  mag_offset [LSB]    = {_fmt3(cal.mag_offset, 1)}')
        self._out('重新啟動 IMU 節點後生效。')

    def _reset_software_offsets(self, fields) -> bool:
        """Clear residual offsets before persisting a new hardware baseline."""
        if not fields or not os.path.exists(self.config_path):
            return True
        try:
            cal = load_calibration(self.config_path)
            save_calibration(self.config_path, dataclasses.replace(
                cal, **{field: (0.0, 0.0, 0.0) for field in fields}))
        except (ValueError, OSError) as exc:
            self._out(f'無法清除過時的軟體偏移:{exc}')
            self._out('未送出儲存或重設指令;若已開始校正,請重新上電以重新載入 flash。')
            self._out(f'請先備份並修正 {self.config_path} 的 {", ".join(fields)},'
                      '不要載入舊值啟動 driver。')
            return False
        self._out(f'已將 {self.config_path} 的 {", ".join(fields)} 歸零,保留其他欄位。')
        return True

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
    def _plane_line(stage_text: Optional[str], cov: PlaneCoverage, last: bool,
                    elapsed: float) -> str:
        bars = '  '.join(
            f'{PLANE_LABEL.get(p, p)} {progress_bar(cov.progress(p), PLANE_BAR_WIDTH)}'
            for p in cov.planes)
        if cov.complete():
            tail = '[完成] 按 Enter' if last else '這段完成'
        else:
            tail = '缺 ' + ' '.join(PLANE_LABEL.get(p, p) for p in cov.missing())
        head = f'{stage_text}  ' if stage_text else ''
        return f'{head}{bars}  {elapsed:3.0f}秒  {tail}'

    def _measure_field_radius(self) -> Optional[float]:
        """Median |B| (raw LSB) over a short normal-mode window, or None without data."""
        norms = []
        for p in self._packets_for(FIELD_MEASURE_TIME):
            if isinstance(p, wp.Mag):
                n = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
                if math.isfinite(n) and n > 0:
                    norms.append(n)
        return statistics.median(norms) if norms else None

    def _wait_mag_planes(self, stages) -> None:
        """
        Wait for Enter while a reader thread shows the per-plane progress, stage by stage.

        Progress comes from the magnetometer because the CMP10A freezes its
        gyro / accel / angle output in magnetometer-calibration mode.  Each
        stage collects fresh samples; the next stage starts when one is done.
        """
        covs = [PlaneCoverage(planes) for _, planes in stages]
        state = {'stage': 0}
        self._mag_covered = False
        self.ser.reset_input_buffer()
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
                            covs[state['stage']].add(p)
                    if covs[state['stage']].complete() and state['stage'] < len(covs) - 1:
                        state['stage'] += 1
                    now = self._time()
                    if now >= next_status:
                        i = state['stage']
                        if not covs[0].samples and now - start >= NO_DATA_HINT_AFTER:
                            line = (f'未收到磁力計資料 {now - start:3.0f}秒,無法判定;'
                                    'Enter 結束,不儲存')
                        else:
                            text = stages[i][0] if len(stages) > 1 else None
                            line = self._plane_line(text, covs[i], i == len(covs) - 1,
                                                    now - start)
                        self._status(line)
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
        if thread.is_alive():
            raise serial.SerialException('無法停止磁力計取樣,不儲存')
        self._mag_covered = all(c.complete() for c in covs) and not errors
        if errors:
            raise errors[0]
        if not covs[0].samples:
            raise serial.SerialException('未收到磁力計資料,不儲存')
        if not self._mag_covered:
            missing = [f'{text}({"、".join(c.missing())})'
                       for (text, _), c in zip(stages, covs) if not c.complete()]
            self._out(f'提醒:{";".join(missing)} 尚未轉完,品質尚未驗證。')

    def _wait_level(self) -> bool:
        """Official dual-plane rule: start only with the IMU level within 15 deg."""
        while True:
            accels = [(p.x, p.y, p.z) for p in self._packets_for(READBACK_TIME)
                      if isinstance(p, wp.Accel)]
            if not accels:
                self._out('未收到加速度資料,無法確認是否水平;已取消,未送出任何指令。')
                return False
            tilt = cc.tilt_deg(tuple(sum(a[i] for a in accels) / len(accels) for i in range(3)))
            if tilt <= LEVEL_MAX_TILT_DEG:
                self._out(f'IMU 目前水平(傾斜 {tilt:.1f}°),可以開始。')
                return True
            answer = self._ask(f'IMU 傾斜 {tilt:.1f}°,官方規定不可超過 {LEVEL_MAX_TILT_DEG:.0f}°。'
                               '擺正後按 Enter 重新檢查,q 取消:').lower()
            if answer == 'q':
                self._out('已取消,未送出任何指令。')
                return False

    def _hardware(self, mode: wp.CalMode, title: str, instructions: List[str],
                  wait_s: float = 0.0, mag_progress: bool = False) -> None:
        """Run unlock -> CALSW mode -> wait -> CALSW NORMAL -> save."""
        self._out(f'== {title}(硬體) ==')
        for line in instructions:
            self._out(line)
        fields = ('accel_bias',) if mode == wp.CalMode.ACCEL else (
            ('mag_offset',) if mag_progress else ())
        if fields:
            if self._load_for_update() is None:
                return
            self._out(f'儲存硬體校正前,會先將 YAML 的 {", ".join(fields)} 歸零,'
                      '避免重複扣除舊偏移。')
        if not self._confirm('此操作會寫入 IMU flash,確定執行?'):
            self._out('已取消,未送出任何指令。')
            return
        field_radius = None
        if mag_progress:
            self._out('量測目前磁場強度(約 2 秒)...')
            field_radius = self._measure_field_radius()
            if field_radius is None:
                self._out('未收到有效磁力計資料,已取消,未送出校正或儲存指令。')
                return
        if mode == wp.CalMode.MAG_DUAL_PLANE and not self._wait_level():
            return
        self._unlock()
        choices = {wp.CalMode.ACCEL: ('2',), wp.CalMode.MAG_SPHERE: ('3', '4'),
                   wp.CalMode.MAG_DUAL_PLANE: ('3', '4'), wp.CalMode.HEADING_ZERO: ('5',),
                   wp.CalMode.ANGLE_REF: ('6',), wp.CalMode.HEIGHT_ZERO: ('7',)}
        self._done.difference_update(choices.get(mode, ()))
        try:
            self._send(wp.cmd_calsw(mode))
            if mag_progress:
                self._wait_mag_planes(MAG_STAGES[mode])
            elif wait_s > 0:
                self._countdown(wait_s, '校正中,請保持不動')
        except (KeyboardInterrupt, EOFError):
            self._out('\n已中止,未送出儲存指令;模組記憶體可能已變更,請重新上電。')
            return
        except (serial.SerialException, OSError):
            self._out('讀取或指令失敗,未送出儲存指令;請重新上電並確認連線。')
            raise
        finally:
            self._ensure_unlocked()
            self._send(wp.cmd_calsw(wp.CalMode.NORMAL))
        if mag_progress and not self._mag_covered:
            self._out('涵蓋不足,不會標示完成;若不儲存,請重新上電以重新載入 flash 設定。')
            if not self._confirm('仍要將未驗證的結果儲存到 IMU flash?'):
                self._out('已恢復正常模式,未儲存。')
                return
        if not self._reset_software_offsets(fields):
            return
        self._ensure_unlocked()
        try:
            self._send(wp.cmd_save())
        except (serial.SerialException, OSError):
            self._out('儲存指令失敗,硬體狀態未知;舊軟體偏移已停用,請重新上電。')
            raise
        self._out('已送出儲存指令。')
        if mag_progress and not self._mag_covered:
            return
        self._step_ok = True

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
        """Hardware accelerometer level calibration (official: still for 2 s, then check)."""
        self._hardware(wp.CalMode.ACCEL, '加速度計水平校正',
                       ['官方步驟:模組水平靜止放置,點一下校正,約 2 秒後完成。'],
                       wait_s=ACCEL_CAL_WAIT)
        if self._step_ok:
            self._show_accel_readback()

    def _show_accel_readback(self) -> None:
        """Show the readings the manual says to expect: about 0, 0, 1 g and 0 deg."""
        accels, angles = [], []
        for p in self._packets_for(READBACK_TIME):
            if isinstance(p, wp.Accel):
                accels.append((p.x, p.y, p.z))
            elif isinstance(p, wp.Angle):
                angles.append((math.degrees(p.roll), math.degrees(p.pitch)))
        if not accels or not angles:
            self._out('未收到資料,無法顯示校正後讀數;請用選項 1 確認。')
            return
        g = [sum(a[i] for a in accels) / len(accels) / wp.GRAVITY for i in range(3)]
        roll = sum(a[0] for a in angles) / len(angles)
        pitch = sum(a[1] for a in angles) / len(angles)
        self._out(f'校正後讀數:加速度 X {g[0]:+.3f}  Y {g[1]:+.3f}  Z {g[2]:+.3f} g,'
                  f'roll {roll:+.2f}°  pitch {pitch:+.2f}°')
        self._out('官方:應約為 0、0、1 g,角度約 0°。')

    def hw_mag(self, mode: wp.CalMode, title: str) -> None:
        """Hardware magnetometer calibration (sphere fit or dual plane)."""
        # Dual plane is for robots too big to flip: the IMU is already mounted.
        is_dual_plane = mode == wp.CalMode.MAG_DUAL_PLANE
        if not self._mag_precheck(mounted=True if is_dual_plane else None,
                                  notes_choice='4' if is_dual_plane else '3'):
            return
        if mode == wp.CalMode.MAG_DUAL_PLANE:
            steps = list(DUAL_PLANE_STEPS)
        else:
            steps = list(SPHERE_STEPS)
        steps += MAG_PROGRESS_HELP
        self._hardware(mode, title, steps, mag_progress=True)

    def _read_axis6(self) -> Optional[int]:
        regs = self._read_registers(REG_AXIS6)
        return None if regs is None else regs.values[0] & 1

    def hw_heading_zero(self) -> None:
        """Hardware Z-axis zero; official: only valid with the six-axis algorithm."""
        axis6 = self._read_axis6()
        if axis6 == 0:
            self._out('目前是九軸演算法。官方:九軸的 Z 軸是絕對角度(以磁北為準),不能歸零。')
            self._out('要歸零,先用選項 10 切換到六軸。未送出任何指令。')
            return
        if axis6 is None:
            self._out('讀不到演算法設定。官方:Z 軸歸零只在六軸演算法有效。')
            if not self._confirm('仍要執行嗎?'):
                self._out('已取消,未送出任何指令。')
                return
        self._hardware(wp.CalMode.HEADING_ZERO, 'Z 軸角度歸零',
                       ['官方步驟:模組靜止放置,以目前方向作為 Z 軸 0°。'],
                       wait_s=SHORT_CAL_WAIT)

    def hw_angle_ref(self) -> None:
        """Hardware angle reference (official warning: conflicts with accel calibration)."""
        if not self._confirm('官方警告:角度參考和加速度計校正同時使用,可能造成加速度、'
                             '角度異常。仍要執行嗎?'):
            self._out('已取消,未送出任何指令。')
            return
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

    def gyro_autocal_status(self) -> None:
        """Show the gyro auto-calibration registers and the official explanation (read-only)."""
        self._out('== 陀螺儀自動校正(唯讀)==')
        regs = self._read_registers(REG_GYROCALITHR)
        if regs is None:
            self._out('讀取暫存器 0x61 逾時,未收到回覆;請確認連線與鮑率。')
        else:
            thr = regs.values[0] & 0xFFFF
            hold = regs.values[2] & 0xFFFF
            self._out(f'  靜止門檻 GYROCALITHR (0x61) = {thr / 1000:.3f}°/s')
            self._out(f'  判定時間 GYROCALTIME (0x63) = {hold} ms')
        self._out('官方說明(手冊 3.3.7):')
        self._out('  - 陀螺儀預設會自動校正:角速度變化小於門檻時當作靜止,輸出 0。')
        self._out('  - 因此「等速旋轉」也可能被當成靜止而輸出 0,這種用途官方建議關閉。')
        self._out('  - 一般使用不需要關閉。')
        self._out('這顆模組的開關指令尚未確認(不同型號的同一個暫存器意義不同),')
        self._out('本工具不提供開關;需要時請用官方 Windows 軟體。')

    def switch_algorithm(self) -> None:
        """Switch between the six-axis and nine-axis algorithm (AXIS6, 0x24)."""
        self._out('== 演算法 六軸 / 九軸 ==')
        axis6 = self._read_axis6()
        self._axis6 = axis6
        if axis6 is None:
            self._out('讀取演算法設定 (0x24) 逾時,未做任何變更;請確認連線與鮑率。')
            return
        names = MODE_NAMES
        new = 1 - axis6
        self._out(f'目前:{names[axis6]}')
        if not self._confirm(f'要切換成{names[new]}嗎?(會寫入 IMU flash)'):
            self._out('已取消,未變更設定。')
            return
        self._unlock()
        self._send(wp.cmd_write(REG_AXIS6, new))
        self._ensure_unlocked()
        self._send(wp.cmd_save())
        self._axis6 = self._read_axis6()
        if self._axis6 == new:
            self._out(f'已切換為{names[new]}並送出儲存指令。')
            self._step_ok = True
        else:
            self._out('已送出切換與儲存指令,但讀回的設定不符,狀態未知;請重新上電後再看一次。')
        self._out('官方:切換後要重做加速度計校正(2)' +
                  (',並做 Z 軸歸零(5)。' if new == 1 else '與磁力計校正(3 或 4)。'))

    # ------------------------------------ live screen of the environment check

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

    # -------------------------------------- environment / mounting precheck

    def _record_still(self, prompt: str, seconds: float):
        """
        Record ``seconds`` of magnetometer and accelerometer data at rest.

        The recording starts once the IMU has been still for STILL_HOLD_S and
        restarts if it moves.  Return a dict with 'mags' and 'accels', or None
        when skipped.
        """
        st = {'gyro': None, 'hold': None, 'start': None, 'mags': [], 'accels': []}

        def reset():
            st['hold'] = st['start'] = None
            st['mags'] = []
            st['accels'] = []

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
                if not self._still(p):
                    reset()
                return
            if st['start'] is None:
                if not self._still(st['gyro']):
                    return
                if st['hold'] is None:
                    st['hold'] = now
                if now - st['hold'] >= STILL_HOLD_S:
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

    def _precheck_field_at_rest(self, results, hints) -> Optional[Tuple[float, float, float]]:
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
        """
        Slide the IMU without turning it; every phase is detected automatically.

        Still for STILL_HOLD_S -> reference taken; acceleration away from the
        reference -> moving; still again for STILL_HOLD_S -> done.  Turning
        more than SLIDE_MAX_ROTATION_DEG voids the attempt, which then starts
        over by itself once the IMU is still again.
        """
        name = '2. 附近有無鐵件'
        st = {'gyro': None, 'retries': 0, 'last_turn': 0.0}

        def restart():
            st.update(phase='wait', hold=None, ref_acc=[], ref=[], ref_gyro=[], mags=[],
                      end=[], moved=False, rest=None, meter=None, net=None,
                      bias=(0.0, 0.0, 0.0))
        restart()

        def on_packet(p, now):
            if isinstance(p, wp.Gyro):
                st['gyro'] = p
                vec = (p.x, p.y, p.z)
                if st['phase'] == 'wait' and st['hold'] is not None:
                    st['ref_gyro'].append(vec)
                if st['meter'] is not None:
                    rate = [a - b for a, b in zip(vec, st['bias'])]
                    st['net'].add(now, rate)
                    turned = st['meter'].add(now, rate)
                    if not cc.slide_valid(turned):
                        st['retries'] += 1
                        st['last_turn'] = turned
                        restart()
                return
            if not isinstance(p, (wp.Accel, wp.Mag)):
                return
            vec = (p.x, p.y, p.z)
            still = self._still(st['gyro'])
            if st['phase'] == 'wait':
                if not still:
                    restart()
                    return
                if st['hold'] is None:
                    st['hold'] = now
                (st['ref_acc'] if isinstance(p, wp.Accel) else st['ref']).append(vec)
                if (now - st['hold'] >= STILL_HOLD_S and len(st['ref']) >= 3
                        and len(st['ref_acc']) >= 3):
                    st['phase'] = 'slide'
                    st['mags'] = list(st['ref'])
                    if st['ref_gyro']:
                        n = float(len(st['ref_gyro']))
                        st['bias'] = tuple(sum(g[i] for g in st['ref_gyro']) / n
                                           for i in range(3))
                    st['meter'] = cc.RotationMeter()
                    st['net'] = cc.NetRotation()
                return
            if isinstance(p, wp.Mag):
                st['mags'].append(vec)
                if st['rest'] is not None:
                    st['end'].append(vec)             # at rest in the new place
                return
            n = float(len(st['ref_acc']))
            ref = [sum(a[i] for a in st['ref_acc']) / n for i in range(3)]
            away = math.sqrt(sum((a - b) ** 2 for a, b in zip(vec, ref))) > SLIDE_MOVE_ACCEL
            if away or not still:
                st['moved'] = st['moved'] or away
                st['rest'] = None
                st['end'] = []
            elif st['moved'] and st['rest'] is None:
                st['rest'] = now

        def done(now):
            return (st['phase'] == 'slide' and st['moved'] and st['rest'] is not None
                    and now - st['rest'] >= STILL_HOLD_S and len(st['end']) >= 3)

        def status(now):
            if st['phase'] == 'wait':
                again = (f'  上次轉了 {st["last_turn"]:.0f}°,不算;停住後自動重來'
                         if st['retries'] else '')
                return f'放著不動,準備中...{again}'
            change = cc.rest_change(st['ref'], st['end'] or st['mags'][-SLIDE_RECENT:])
            if not st['moved']:
                return f'準備好了:不要轉動,開始平移  磁場變化 {change:.1%}'
            tol = cc.slide_change_tol(st['net'].angle_deg)
            state = '停住了,確認中...' if st['rest'] is not None else '平移中...'
            return (f'{state}  磁場變化 {change:.1%}(標準 {tol:.1%} 內)  '
                    f'已轉動 {st["meter"].total_deg:.1f}°/{cc.SLIDE_MAX_ROTATION_DEG:.0f}°')

        prompt = (f'環境 2/2 平移:IMU(或整台車)放在起點不動,等畫面顯示「準備好了」,'
                  f'不要轉動,沿桌面或地面平移約 {SLIDE_DISTANCE_CM} cm 再停住。全程自動'
                  '(s 略過 / q 結束):')
        while True:
            cmd = self._watch(prompt, status, on_packet, done)
            if cmd == 's':
                self._record(results, SKIP, name)
                return
            if cmd is None or (st['phase'] == 'slide' and st['moved']):
                break
            self._out('還沒偵測到平移;準備好了之後,不要轉動地移動 IMU。')
        # Only the two resting places count; a stop that never settled has no end data.
        change = cc.rest_change(st['ref'], st['end'])
        net = st['net'].angle_deg
        ok = cc.slide_change_ok(change, net)
        self._record(results, PASS if ok else FAIL,
                     f'{name}:平移前後變化 {change:.1%}'
                     f'(標準 {cc.slide_change_tol(net):.1%} 內,含淨轉動 {net:.1f}°)')
        if not ok:
            hints.append('平移時磁場跟著位置改變:附近有鐵件(鐵桌、鐵櫃、筆電、含鋼筋的地板)。'
                         '換到木桌或空曠處,或把 IMU 墊高遠離地面。')

    def _record_motors(self, base):
        """
        Record MOTOR_RUN_S of data once the motors are detected running.

        Running = more vibration or a field change compared with ``base``
        (motors stopped).  A mounting good enough to show neither cannot be
        detected, so Enter starts the recording by hand.  Return the data or
        None when skipped.
        """
        base_vib = cc.vibration(base['accels'])
        st = {'acc_win': [], 'mag_win': [], 'detected': None, 'start': None,
              'mags': [], 'accels': [], 'since': self._time()}

        def on_packet(p, now):
            if not isinstance(p, (wp.Accel, wp.Mag)):
                return
            vec = (p.x, p.y, p.z)
            if isinstance(p, wp.Accel):
                st['acc_win'] = (st['acc_win'] + [vec])[-MOTOR_WINDOW:]
                if st['start'] is not None:
                    st['accels'].append(vec)
            else:
                st['mag_win'] = (st['mag_win'] + [vec])[-MOTOR_WINDOW:]
                if st['start'] is not None:
                    st['mags'].append(vec)
            if st['start'] is not None:
                return
            if st['detected'] is None:
                shaking = (len(st['acc_win']) >= 5 and cc.vibration(st['acc_win'])
                           > max(MOTOR_DETECT_VIB, 3.0 * base_vib))
                field_changed = (len(st['mag_win']) >= 3
                                 and cc.field_change(base['mags'], st['mag_win'])
                                 > MOTOR_DETECT_CHANGE)
                if shaking or field_changed:
                    st['detected'] = now
            elif now - st['detected'] >= MOTOR_SETTLE_S:
                st['start'] = now

        def done(now):
            return (st['start'] is not None and now - st['start'] >= MOTOR_RUN_S
                    and len(st['mags']) >= 3 and len(st['accels']) >= 3)

        def status(now):
            if st['start'] is not None:
                bar = progress_bar((now - st['start']) / MOTOR_RUN_S, 10)
                return f'量測中,馬達保持轉動 {bar}'
            if st['detected'] is not None:
                return '偵測到馬達轉動,等它轉穩...'
            if now - st['since'] < MOTOR_ENTER_AFTER_S:
                return '等待馬達轉動...'
            return '沒偵測到馬達轉動。馬達確定在轉的話,代表影響很小,按 Enter 開始量'

        prompt = (f'安裝 2/2:讓馬達以平常的速度轉起來,偵測到會自動量 {MOTOR_RUN_S:.0f} 秒'
                  '(s 略過 / q 結束):')
        while True:
            cmd = self._watch(prompt, status, on_packet, done)
            if cmd is None:
                return st
            if cmd == 's':
                return None
            if cmd != '' or st['start'] is not None:
                continue
            wait = MOTOR_ENTER_AFTER_S - (self._time() - st['since'])
            if st['detected'] is None and wait > 0:
                self._out(f'還在偵測馬達,{wait:.0f} 秒內偵測不到才能按 Enter 手動開始。')
            else:
                st['start'] = self._time()

    def _precheck_mounting(self, results, hints, accel_mean) -> None:
        if accel_mean is not None:
            tilt = cc.tilt_deg(accel_mean)
            name = '3. 安裝方向'
            if tilt <= cc.TILT_NOTE_DEG:
                self._record(results, PASS, f'{name}:元件面朝上,傾斜 {tilt:.1f}°')
            elif tilt < 45.0:
                self._record(results, NOTE, f'{name}:傾斜 {tilt:.1f}°。小角度在車停水平地面做選項 2 '
                             '時會一起扣掉,或在 ROS 的 TF 設定安裝角度')
            else:
                self._record(results, NOTE, f'{name}:元件面沒有朝上(偏 {tilt:.0f}°),'
                             'ROS 的 TF 要設定安裝角度')
        names = ('4. 馬達干擾', '5. 振動')
        base = self._record_still(
            f'安裝 1/2:車子通電,把車架高讓輪子懸空,馬達先不要轉。自動記錄 {MOTOR_BASE_S:.0f} 秒'
            '(s 略過 / q 結束):', MOTOR_BASE_S)
        running = None if base is None else self._record_motors(base)
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

    def _mag_precheck(self, mounted: Optional[bool] = None,
                      notes_choice: Optional[str] = None) -> bool:
        """
        Check the surroundings and the mounting position from the IMU's own readings.

        Return True to go on with the magnetometer calibration.  When something
        is unsuitable the user is told why and asked twice before going on.
        ``mounted`` None asks whether the IMU is on the robot; True skips the
        question (dual-plane calibration is only used on a mounted IMU).
        ``notes_choice`` is the menu choice whose placement notes are shown
        again afterwards, since the check clears the screen.
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
        self._check_cal = self._load_existing()

        def guarded(step, label, *args):
            try:
                return step(*args)
            except ValueError as exc:
                logger.error('environment check %s: bad data: %s', label, exc)
                self._record(results, SKIP, f'{label}:資料異常,無法判定({exc})')
                return None

        try:
            accel_mean = guarded(self._precheck_field_at_rest, '1. 磁場穩定', results, hints)
            guarded(self._precheck_slide, '2. 附近有無鐵件', results, hints)
            if mounted is None:
                mounted = self._confirm('IMU 已經裝在車上的最終位置了嗎?')
            if mounted:
                guarded(self._precheck_mounting, '3–5. 安裝位置', results, hints, accel_mean)
            else:
                self._verify_note('IMU 還沒裝上車:裝上去之後,要在最終位置重新檢查與校正。')
        except _VerifyQuit:
            self._out('已結束檢查。')
            self._out('已取消校正,未送出任何指令。')
            return False
        except (serial.SerialException, OSError) as exc:
            logger.error('reading IMU data in the environment check failed: %s', exc)
            self._out(f'讀取 IMU 資料失敗:{exc}')
            return False
        finally:
            self._check_cal = SoftwareCalibration()
        self._out((CLEAR_SCREEN if self._color else '') + '== 環境與安裝位置 檢查結果 ==')
        for status, text in results:
            self._out(self._mark(status, text))
        if any(text.startswith('4. 馬達干擾:') for _, text in results):
            self._out('馬達測完了:開始校正前,先讓馬達停止。')
        if notes_choice is not None:
            self._show_notes(notes_choice)
        if not any(status == FAIL for status, _ in results):
            if any(status == SKIP for status, _ in results):
                self._out(self._mark(NOTE, '檢查未完成:略過或異常的項目無法判定。'))
            elif any(status == PASS for status, _ in results):
                self._out(self._mark(PASS, '環境與安裝位置合適'))
            return True
        self._out('不適合的原因與改善方式:')
        for hint in hints:
            self._out(f'  - {hint}')
        if not self._confirm('目前的環境或安裝位置不適合。仍要在這裡繼續校正嗎?'):
            self._out('已取消校正,未送出任何指令。')
            return False
        if not self._confirm('再確認一次:在不適合的環境或位置校正,結果會不準,'
                             '裝上車後也可能不穩定。確定繼續?'):
            self._out('已取消校正,未送出任何指令。')
            return False
        self._out('依你的選擇,繼續校正。')
        return True

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
            self._out('開啟後每秒共 50 個封包(約 550 bytes),約佔 9600 baud(約 960 bytes/s)的 57%,'
                      '仍足夠。')
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
        if self._load_for_update() is None:
            return
        self._out('同時將 YAML 的 gyro_bias、accel_bias、mag_offset 全部歸零。')
        if not self._confirm('確定要恢復原廠設定?'):
            self._out('已取消,未送出任何指令。')
            return
        if not self._confirm('再次確認:此動作無法復原,真的要執行?'):
            self._out('已取消,未送出任何指令。')
            return
        if not self._reset_software_offsets(('gyro_bias', 'accel_bias', 'mag_offset')):
            return
        try:
            self._unlock()
            self._send(wp.cmd_factory_reset())
        except (serial.SerialException, OSError):
            self._out('重設指令失敗,硬體可能仍保留舊設定;請重新上電。')
            raise
        self._out('已送出恢復原廠設定指令;建議重新上電,並以預設 9600 baud 連線。')
        self._done.clear()               # every earlier calibration is gone
        self._refresh_mode()             # factory settings may use another algorithm

    # ------------------------------------------------------------ main loop

    def _show_notes(self, choice: str) -> None:
        """Print placement precautions for a calibration item, if any."""
        notes = NOTES.get(choice)
        if not notes:
            return
        self._out('注意事項:')
        for line in notes:
            self._out(line)

    def _refresh_mode(self) -> None:
        """Read which algorithm the module runs, so the menu shows only what it needs."""
        try:
            self._axis6 = self._read_axis6()
        except (serial.SerialException, OSError) as exc:
            logger.error('reading the algorithm setting failed: %s', exc)
            self._axis6 = None

    def run(self) -> int:
        """Run the menu loop and return a process exit code."""
        try:
            self._refresh_mode()
        except KeyboardInterrupt:
            self._out('\n已中斷,離開。')
            return 130
        while True:
            self._out(render_menu(self._done, self._color, self._axis6))
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
                self._out(f'無效的選項:{choice!r},請輸入 0-12。')
                continue
            if choice in HIDDEN_BY_MODE.get(self._axis6, ()):
                self._out(HIDDEN_REASON[self._axis6])
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


def render_menu(done=(), color: bool = False, axis6: Optional[int] = None) -> str:
    """
    Return the menu for the algorithm ``axis6`` with the rows in ``done`` marked.

    Rows the algorithm cannot use are left out (see HIDDEN_BY_MODE); done rows
    are green on a terminal.
    """
    hidden = HIDDEN_BY_MODE.get(axis6, ())
    lines = []
    for line in MENU.split('\n'):
        if 'CMP10A IMU 校正工具' in line:
            line = f'===== CMP10A IMU 校正工具(目前:{MODE_NAMES.get(axis6, "模式未知")})====='
        if line.startswith(' 0. '):
            lines.append(HIDDEN_NOTE.get(axis6, HIDDEN_NOTE[None]))
        match = _MENU_ROW.match(line)
        if match and match.group(1) in hidden:
            continue
        if match and match.group(1) in done:
            line = f'{line}  {DONE_MARK}'
            if color:
                line = f'{ANSI_COLOURS[PASS]}{line}{ANSI_RESET}'
        lines.append(line)
    return '\n'.join(lines)


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
