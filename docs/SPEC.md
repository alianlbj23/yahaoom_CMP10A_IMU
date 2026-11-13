# SPEC: Yahboom CMP10A IMU — Docker 可視化與校正

## 目標
在 Linux / WSL2 / macOS 上,以單一 Docker 映像(ROS 2 Jazzy)讀取 Yahboom 10 軸 IMU (CMP10A),
提供 RViz2 與 Foxglove(經 rosbridge 或 foxglove_bridge)可視化,並提供硬體 + 軟體校正工具。
使用者透過主機端 Python 互動選單腳本 `imu.py` 進入/操作 docker。

## 已驗證的硬體事實(2026-11-09 實機)
- USB-UART:CP2102 (VID 10c4 / PID ea60),Linux 顯示為 `/dev/ttyUSB*`
- 預設 9600 baud,輸出 0x51/0x52/0x53/0x54 四種封包,約 10 Hz
- 讀暫存器 `FF AA 27 <addr> 00` 不需 unlock,回覆 `55 5F` + 4 個 int16 LE + SUM
- 實機錄製資料:`wit_ros2_imu/test/fixtures/cmp10a_stationary_9600.bin`(靜置,約 2.2 秒)

## WitMotion 標準協定(來源:wit-motion.gitbook.io wit-standard-communication-protocol)
封包:`55 TYPE D0..D7 SUM`,SUM = 前 10 bytes 和 & 0xFF。資料為 int16 little-endian。
| TYPE | 內容 | 換算 |
|---|---|---|
| 0x51 | ax ay az temp | /32768*16 g(×9.80665 → m/s²);temp/100 °C |
| 0x52 | wx wy wz volt | /32768*2000 °/s(→ rad/s) |
| 0x53 | roll pitch yaw ver | /32768*180 ° |
| 0x54 | hx hy hz temp | raw LSB;temp/100 °C |
| 0x56 | P(int32 LE) H(int32 LE) | P: Pa;H: cm |
| 0x59 | q0 q1 q2 q3 | /32768(q0 = w) |
| 0x5F | 4 個暫存器值 | int16 LE |

指令(寫暫存器 `FF AA <reg> <lo> <hi>`):
- unlock `FF AA 69 88 B5`(10 秒內要完成後續指令);save `FF AA 00 00 00`;factory reset `FF AA 00 01 00`
- CALSW (0x01):0x00 正常、0x01 加速度計校正、0x03 高度歸零、0x04 航向角(Z)歸零、
  0x07 磁力計校正(球面擬合)、0x08 設定角度參考、0x09 磁力計校正(雙平面)
- RSW (0x02) bit: ACC=1, GYRO=2, ANGLE=3, MAG=4, PRESS=6, QUATER=9
- RRATE (0x03):0x06=10Hz, 0x07=20Hz, 0x08=50Hz, 0x09=100Hz
- 偏移暫存器(唯讀檢視):AXOFFSET 0x05 … GZOFFSET 0x0A, HXOFFSET 0x0B … HZOFFSET 0x0D

## 目錄結構(契約)
```
imu.py                          # 主機端互動選單 (stdlib only, Python >=3.8)
host/serial_tcp_bridge.py       # 主機端 serial→TCP 橋接 (macOS 用;需 pyserial==3.5)
tests/                          # 主機端腳本測試 (pytest)
docker/Dockerfile               # FROM ros:jazzy-ros-base
docker/entrypoint.sh
config/                         # 掛載到容器 /config(軟體校正 YAML)
wit_ros2_imu/                   # ROS 2 ament_python 套件
  wit_ros2_imu/wit_protocol.py  # 純 Python(不 import rclpy)
  wit_ros2_imu/calibration.py   # 純 Python 軟體校正模型
  wit_ros2_imu/calibrate_cli.py # 互動校正選單(直接開序列埠,不需 ROS)
  wit_ros2_imu/wit_ros2_imu.py  # ROS 2 driver node
  launch/imu.launch.py
  rviz/imu.rviz
  test/
```

## 介面契約

### `wit_protocol.py`
```python
GRAVITY = 9.80665
@dataclass Accel(x, y, z: float  # m/s²
                 temperature: float)
@dataclass Gyro(x, y, z: float)   # rad/s
@dataclass Angle(roll, pitch, yaw: float)  # rad
@dataclass Mag(x, y, z: int, temperature: float)  # raw LSB
@dataclass Pressure(pressure_pa: float, height_m: float)
@dataclass Quaternion(w, x, y, z: float)
@dataclass RegisterValues(values: tuple[int, int, int, int])
class FrameParser:
    def feed(self, data: bytes) -> list[Packet]  # 處理任意切割/雜訊/錯 checksum;回傳解出的封包
    checksum_errors: int
def decode_frame(frame: bytes) -> Packet | None   # 11 bytes;未知 TYPE 回 None;checksum 錯 raise ValueError
def cmd_write(reg: int, value: int) -> bytes       # value 範圍 -32768..65535,超出 raise ValueError
def cmd_unlock() -> bytes; cmd_save() -> bytes; cmd_factory_reset() -> bytes
def cmd_read(reg: int) -> bytes                    # FF AA 27 reg 00
def cmd_calsw(mode: CalMode) -> bytes
class CalMode(IntEnum): NORMAL=0, ACCEL=1, HEIGHT_ZERO=3, HEADING_ZERO=4, MAG_SPHERE=7, ANGLE_REF=8, MAG_DUAL_PLANE=9
def euler_to_quaternion(roll, pitch, yaw) -> Quaternion
```

### `calibration.py`
```python
@dataclass SoftwareCalibration(gyro_bias=(0,0,0) rad/s, accel_bias=(0,0,0) m/s², mag_offset=(0,0,0) LSB)
    def apply_gyro(g: Gyro) -> Gyro; apply_accel(a: Accel) -> Accel; apply_mag(m: Mag) -> Mag
def load_calibration(path) -> SoftwareCalibration   # 檔案不存在 → 預設值;格式錯 → raise ValueError
def save_calibration(path, cal) -> None             # YAML (yaml.safe_load / safe_dump)
def compute_static_bias(accels: list[Accel], gyros: list[Gyro]) -> (gyro_bias, accel_bias)
    # 假設靜置水平:accel 期望 (0,0,+g);樣本 < 10 raise ValueError
def compute_mag_offset(mags: list[Mag]) -> tuple   # (max+min)/2 hard-iron;樣本 < 10 raise ValueError
```
YAML 格式:
```yaml
gyro_bias: [x, y, z]     # rad/s
accel_bias: [x, y, z]    # m/s^2
mag_offset: [x, y, z]    # raw LSB
```

### `calibrate_cli.py`(console script `imu_calibrate`)
參數:`--port`(可為 pyserial URL,例如 `socket://host.docker.internal:5760`)、`--baud`、`--config`(YAML 路徑)。
互動選單(stdlib `input()`):
1. 即時資料檢視 2. 加速度計水平校正(硬體) 3. 磁力計校正 球面擬合(硬體) 4. 磁力計校正 雙平面(硬體)
5. Z 軸航向歸零(硬體) 6. 設定角度參考(硬體) 7. 高度歸零(硬體,氣壓計) 8. 讀取 IMU 內部偏移暫存器
9. 軟體靜態偏移校正(gyro/accel → YAML) 10. 軟體磁力計 hard-iron 校正(→ YAML) 11. 恢復原廠設定 12. 姿態檢視 / 校正驗證(唯讀;Enter 進入逐項引導驗證並以顏色標示合格與否,f 為可設參考點看 Δ 的自由檢視) 13. 氣壓高度輸出 開啟/關閉(RSW bit 6) 0. 離開
- 任何寫入 flash 前都要求使用者確認 (y/N)
- 硬體指令流程:unlock → CALSW → (等待/使用者操作) → CALSW NORMAL → save
- 序列埠開啟設 timeout;錯誤訊息清楚並以非 0 結束

### `wit_ros2_imu.py`(ROS 2 node,executable `wit_ros2_imu`)
參數:`port`(預設 `/dev/ttyUSB0`,可為 pyserial URL)、`baud`(9600)、`frame_id`(`imu_link`)、
`parent_frame`(`world`)、`publish_tf`(true)、`calibration_file`(`/config/imu_calibration.yaml`)、
`reconnect_interval`(2.0 s)。
發佈:`imu/data` (sensor_msgs/Imu,含 orientation)、`imu/mag` (sensor_msgs/MagneticField)、
`imu/temperature` (sensor_msgs/Temperature)、`imu/pressure` (sensor_msgs/FluidPressure,若有 0x56)、
TF `parent_frame → frame_id`(orientation)。
- 每收到 0x53(或 0x59)封包發佈一次 Imu;0x59 存在時優先用裝置四元數
- 斷線時 log error 並依 reconnect_interval 重連,不 exit
- 使用 `serial.serial_for_url(port, baudrate, timeout=0.5)`

### launch `imu.launch.py`
args:`port`, `baud`, `rviz`(false), `rosbridge`(true), `foxglove`(true), `calibration_file`。
rosbridge_websocket port 9090、foxglove_bridge port 8765,皆 address 0.0.0.0(容器內;主機端由 `-p` 控制綁定)。

### Docker
- 映像名 `ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy`(由 GitHub Actions 建置並發佈,amd64 + arm64;`imu.py` 只下載、不在本機建置);套件:ros-jazzy-rviz2、ros-jazzy-rviz-imu-plugin、ros-jazzy-rosbridge-suite、
  ros-jazzy-foxglove-bridge、ros-jazzy-tf2-ros、python3-serial、python3-yaml、python3-numpy
- 容器內 colcon build `wit_ros2_imu`;entrypoint source `/opt/ros/jazzy` 與 workspace
- `LIBGL_ALWAYS_SOFTWARE=1`(WSLg / XQuartz 相容)

### `imu.py`(主機端選單,stdlib only)
平台偵測:linux / wsl(`/proc/version` 含 microsoft)/ mac(`sys.platform == 'darwin'`)。
選單:1 下載映像(docker pull)2 啟動 IMU + Foxglove bridges 3 啟動 IMU + RViz2(+ bridges)4 IMU 校正工具
5 進入容器 shell 6 停止容器 7 環境檢查 0 離開。
- 序列埠:linux/wsl 列出 `/dev/serial/by-id/*`、`/dev/ttyUSB*`、`/dev/ttyACM*` 讓使用者選,`--device` 傳入容器並以
  容器內 `/dev/ttyUSB0`…原路徑使用;有 `/dev/usb_imu`(usb_udev_namer 建立)時排第一,並以該名稱傳入容器;
  WSL 找不到裝置時印出 wsl_usb_auto 與 usb_udev_namer 指引
- mac:列出 `/dev/cu.usbserial*`、`/dev/cu.SLAB*`、`/dev/cu.wchusbserial*`;背景啟動 `host/serial_tcp_bridge.py`
  (綁 127.0.0.1:5760),容器 port 參數 `socket://host.docker.internal:5760`;缺 pyserial 時提示 `pip install pyserial==3.5`
- GUI:linux → DISPLAY + `/tmp/.X11-unix`;wsl → WSLg(`/tmp/.X11-unix`、`/mnt/wslg`、WAYLAND_DISPLAY、XDG_RUNTIME_DIR);
  mac → XQuartz `DISPLAY=host.docker.internal:0`(提示需開啟 network clients 與 `xhost +localhost`)
- 埠對應預設綁 `127.0.0.1`(9090、8765);啟動時詢問是否開放區網(綁 0.0.0.0)
- docker 指令用 `subprocess.run([...])` list 形式,不用 shell=True;`./config` 掛到 `/config`
- 容器名 `yahboom-imu`

## 驗收
- `pytest` 主機端測試、容器內 `colcon test`(flake8、pep257、copyright、單元測試)全通過
- 實機:容器內 `ros2 topic hz /imu/data` ≈ 10 Hz;rosbridge 9090 / foxglove 8765 可連線
- 授權:Apache-2.0,所有新 .py 檔加 Apache 授權標頭
