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
import statistics
import sys
import threading
import time
from typing import Callable, Dict, Iterator, List, Optional

import serial

from . import wit_protocol as wp
from .calibration import (compute_mag_offset, compute_static_bias, load_calibration,
                          MagCoverage, MIN_SAMPLES, save_calibration, SoftwareCalibration)

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
12. 姿態檢視 / 校正驗證                   → 任何地方(唯讀),驗證校正結果
13. 氣壓高度輸出 開啟/關閉                → 寫入 IMU 設定
 0. 離開
=========================================="""

ATTITUDE_HELP = [
    '操作:Enter = 設定參考點(之後顯示相對變化 Δ)  r = 清除參考點  q = 離開(Ctrl+C 也可以)',
    '驗證方式(合格標準為經驗值):',
    '  1. 放在水平面(用水平儀確認):roll、pitch 都在 ±0.5° 內。',
    '  2. 靠著方形盒子的邊,每次轉 90° 前按 Enter:Δyaw 每次約 ±90°,誤差 ±3° 內。',
    '  3. 對齊桌上的參考線按 Enter,轉一圈再對回:Δyaw 回到 0,誤差 ±3° 內。',
    '  4. 按 Enter 後抬高 1 m:Δ高度 約 +1 m(氣壓計有數十公分雜訊;需先用 13 開啟高度輸出)。',
]

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
                 time_fn: Callable[[], float] = time.monotonic):
        """Store the injected serial port, I/O functions and clock."""
        self.ser = ser
        self._input = input_fn
        self._out = output_fn
        self.config_path = config_path
        self._sleep = sleep_fn
        self._time = time_fn
        self._unlock_time: Optional[float] = None
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

    def _wait_mag_coverage(self, field_radius: Optional[float]) -> None:
        """
        Wait for Enter while a reader thread shows per-axis magnetometer coverage.

        Progress comes from the magnetometer because the CMP10A freezes its
        gyro / accel / angle output in magnetometer-calibration mode.
        KeyboardInterrupt / EOFError from the prompt propagate to the caller
        after the reader thread has been stopped.
        """
        cov = MagCoverage(field_radius)
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
                self._wait_mag_coverage(field_radius)
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
        if mode == wp.CalMode.MAG_DUAL_PLANE:
            steps = ['開始後請先將 IMU 水平繞 Z 軸緩慢轉一圈,再立起繞 X 或 Y 軸轉一圈;',
                     '若不確定,繞三軸緩慢轉動即可。']
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
        if math.sqrt(sum(b * b for b in accel_bias)) > LARGE_ACCEL_BIAS:
            self._out('警告:加速度偏移量偏大,IMU 可能沒有水平放置。')
        cal = dataclasses.replace(self._load_existing(),
                                  gyro_bias=gyro_bias, accel_bias=accel_bias)
        self._save(cal)

    def sw_mag(self) -> None:
        """Software magnetometer hard-iron calibration written to YAML."""
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

    def attitude_view(self) -> None:
        """Show live roll / pitch / yaw / height, optionally relative to a reference."""
        self._out('== 姿態檢視 / 校正驗證(唯讀)==')
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
                if cmd == 'r':
                    with lock:
                        state['ref'] = None
                    self._out('已清除參考點,顯示目前值。')
                elif cmd == '':
                    with lock:
                        angles, height = state['angles'], state['height']
                        if angles is not None:
                            state['ref'] = (angles, height)
                    if angles is None:
                        self._out('尚未收到角度資料,無法設定參考點。')
                    else:
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
            self._out(MENU)
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
            try:
                action()
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
             time_fn: Callable[[], float] = time.monotonic) -> int:
    """Run the interactive menu on an already opened serial port."""
    return CalibrationMenu(ser, input_fn, output_fn, config_path,
                           sleep_fn=sleep_fn, time_fn=time_fn).run()


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
        return run_menu(ser, input, print, args.config)
    finally:
        ser.close()


if __name__ == '__main__':
    sys.exit(main())
