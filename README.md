# yahaoom_CMP10A_IMU

Yahboom 10 軸 IMU(CMP10A,WitMotion 協定)的 ROS 2 Jazzy Docker 環境:
讀取 IMU、以 **Foxglove** 或 **RViz2** 可視化,並提供**硬體 + 軟體校正**工具。
主機端只需要 Docker 與 Python 3.8 以上,所有操作都從互動選單 `python3 imu.py` 進行,不需要記指令參數。

支援平台:**Linux**、**Windows 11 / WSL2**。

---

## 目錄

1. [架構概覽](#架構概覽)
2. [前置需求(依平台)](#前置需求依平台)
3. [使用流程:python3 imu.py](#使用流程python3-imupy)
4. [Foxglove 連線](#foxglove-連線)
5. [RViz2](#rviz2)
6. [校正流程](#校正流程)
7. [ROS topic 列表](#ros-topic-列表)
8. [疑難排解](#疑難排解)
9. [開發與測試](#開發與測試)

---

## 架構概覽

```
                         主機 (Linux / WSL2)
 ┌──────────┐  USB   ┌──────────────────────────────────────────────────────┐
 │ CMP10A   │──────▶│ /dev/usb_imu  (-> /dev/ttyUSB0)                       │
 │ (CP2102) │        │        │                                              │
 └──────────┘        │        │ --device                                     │
                     │        ▼                                              │
                     │  ┌─────────── docker: yahboom-imu (ghcr.io 預建映像) ─┐│
                     │  │ wit_ros2_imu driver → /imu/data, /imu/mag, TF ...  ││
                     │  │ rosbridge :9090   foxglove_bridge :8765   RViz2    ││
                     │  └────────────────────────────────────────────────────┘│
                     │      ▲ -p 127.0.0.1:9090 / 127.0.0.1:8765              │
                     │  Foxglove (瀏覽器 / 桌面 App)                           │
                     └──────────────────────────────────────────────────────┘
```

- 映像:`ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy`,由 GitHub Actions 預先建置(amd64、arm64),**只提供執行環境**。
  程式碼 `wit_ros2_imu/` 在啟動時由 `imu.py` 掛載進容器,修改程式不需要重建映像
- 容器名:`yahboom-imu`(`--rm`,結束即刪除)
- `./config` 掛載到容器 `/config`,軟體校正結果存在 `config/imu_calibration.yaml`

IMU 預設 9600 baud,約 10 Hz 輸出加速度、角速度、角度、磁力計封包。

---

## 前置需求(依平台)

### 共通

| 項目 | 說明 |
|---|---|
| Docker | 可執行 `docker info`(使用者需能不加 sudo 使用 docker) |
| Python | 3.8 以上,`imu.py` 只用標準函式庫 |
| 磁碟 | 映像約 2–3 GB,第一次使用需下載 |
| Foxglove(選用) | [app.foxglove.dev](https://app.foxglove.dev) 或 Foxglove 桌面版 |

### Linux(Ubuntu 22.04 / 24.04 等)

1. 安裝 Docker Engine,並把自己加入 docker 群組:
   `sudo usermod -aG docker "$USER"`(重新登入生效)。
2. 固定 IMU 的裝置名稱:用 [usb_udev_namer](https://github.com/alianlbj23/usb_udev_namer) 把 IMU 命名為 `usb_imu`,
   之後一律使用 `/dev/usb_imu`。命名時會一併開放讀寫權限,不需另外加入 dialout 群組。
   ```bash
   git clone https://github.com/alianlbj23/usb_udev_namer.git
   python3 usb_udev_namer/usb_udev_namer.py     # 選 1 Name a device,名稱輸入 usb_imu
   ls -l /dev/usb_imu                           # 應顯示 /dev/usb_imu -> ttyUSB0
   ```
3. RViz2(選用):需要 X11 桌面(有 `DISPLAY`)。若視窗打不開,可能需要在主機執行
   `xhost +local:docker`(`imu.py` 只提示,不會自動執行)。

### Windows 11 / WSL2

1. 安裝 WSL2(Ubuntu)與 Docker(Docker Desktop 的 WSL 整合,或在 WSL 內安裝 Docker Engine)。
2. 把 IMU 綁定進 WSL:在 Windows PowerShell 安裝 usbipd-win(`winget install usbipd`,裝完重開 WSL),
   再於 WSL 內用 [wsl_usb_auto](https://github.com/alianlbj23/wsl_usb_auto) 新增自動掛載規則,之後 IMU 插上就自動進入 WSL。
   ```bash
   git clone https://github.com/alianlbj23/wsl_usb_auto.git
   python3 wsl_usb_auto/wsl_usb_auto.py         # 選 1 新增自動掛載裝置
   ```
3. 固定 IMU 的裝置名稱:同 Linux 第 2 點,用 `usb_udev_namer` 命名為 `usb_imu`。
4. RViz2 使用 **WSLg**(Windows 11 內建),不需另裝 X server。`imu.py` 會自動帶入
   `/tmp/.X11-unix`、`/mnt/wslg`、`DISPLAY`、`WAYLAND_DISPLAY`、
   `XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir`、`PULSE_SERVER`(變數存在才帶)。

---

## 使用流程:python3 imu.py

```bash
git clone <this repo> && cd yahaoom_CMP10A_IMU
python3 imu.py
```

```
=== Yahboom CMP10A IMU (wsl) ===
  1) 下載 Docker 映像 (ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy)
  2) 啟動 IMU + Foxglove bridges (rosbridge 9090 / foxglove 8765)
  3) 啟動 IMU + RViz2 (+ bridges)
  4) IMU 校正工具
  5) 進入容器 shell
  6) 停止容器
  7) 環境檢查
  8) 設定 (topic 名稱 / frame 名稱 / 網路 / ROS_DOMAIN_ID)
  0) 離開
請選擇 [0-8]:
```

| 選項 | 做什麼 |
|---|---|
| 1 下載映像 | `docker pull ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy`(約 2–3 GB,上限 1 小時)。映像由 GitHub Actions 預先建置 |
| 2 Foxglove | 容器內執行 `ros2 launch wit_ros2_imu imu.launch.py port:=<埠> rviz:=false`,啟動 driver + rosbridge(9090)+ foxglove_bridge(8765) |
| 3 RViz2 | 同上但 `rviz:=true`,另開 RViz2 視窗;沒有 `DISPLAY` 時會警告並建議改用 Foxglove |
| 4 校正工具 | `ros2 run wit_ros2_imu imu_calibrate --port <埠> --baud 9600 --config /config/imu_calibration.yaml`(見[校正流程](#校正流程)) |
| 5 容器 shell | `bash`,ROS 環境已 source;有序列埠時一併傳入 |
| 6 停止容器 | `docker stop yahboom-imu` |
| 8 設定 | 修改 IMU topic 名稱、frame 名稱、是否發佈 `world → frame` 的 TF、網路模式(bridge / host)、ROS_DOMAIN_ID;存於 `config/launcher.json` |
| 7 環境檢查 | 平台、docker daemon、映像是否已下載、容器狀態、序列埠與權限、DISPLAY |

建議第一次使用:**7 環境檢查 → 1 下載映像 → 2 啟動 Foxglove**。

啟動流程(選項 2–5)會依序:

1. 檢查 docker daemon 與映像。
2. 列出序列埠讓你選:
   - Linux / WSL:`/dev/usb_imu`、`/dev/serial/by-id/*`、`/dev/ttyUSB*`、`/dev/ttyACM*`。
     已用 [usb_udev_namer](https://github.com/alianlbj23/usb_udev_namer) 建立固定名稱時,不顯示清單,
     直接使用 `/dev/usb_imu`,以 `--device /dev/ttyUSB0:/dev/usb_imu` 傳入容器,driver 使用 `/dev/usb_imu`。
     沒有固定名稱時,其他 symlink 會解析成實際裝置(例如 `/dev/ttyUSB0`),以
     `--device /dev/ttyUSB0:/dev/ttyUSB0` 傳入容器,容器內用同一路徑。
     若目前使用者沒有讀寫權限會中止並說明處理方式(用 `usb_udev_namer` 命名,或加入 dialout 群組)。
   - WSL 找不到裝置時會印出 `wsl_usb_auto` 與 `usb_udev_namer` 的步驟。
3. 詢問**是否開放區網**(預設否):
   - 否:`-p 127.0.0.1:9090:9090 -p 127.0.0.1:8765:8765`,只有本機能連。
   - 是:`-p 9090:9090 -p 8765:8765`,區網其他電腦可連 `ws://<本機 IP>:8765`。
     **rosbridge / foxglove_bridge 沒有認證**,只在信任的網路開放。
4. 若同名容器 `yahboom-imu` 正在執行,詢問是否先停止。
5. 印出連線資訊,以前景執行容器;**按 Ctrl+C 結束**(會一併清理容器)。

注意:啟動需要互動式終端機(`docker run -it`),請直接在終端機執行,不要用管線餵輸入。
環境檢查可以用管線測試:`printf '7\n0\n' | python3 imu.py`。

結束碼:正常離開為 0;本次有任何操作失敗、或輸入提早結束(EOF)為 1;Ctrl+C 為 130。

---

## 設定(選項 8)

這個 repo 只負責打開 IMU。預設只送出 IMU 資料,frame 為 `imu_link`,**不發佈任何 TF**,不會干擾機器人其他部分的座標系。

| 項目 | 預設 | 說明 |
|---|---|---|
| IMU topic 名稱 | `/imu/data` | `sensor_msgs/Imu` 的 topic。`/imu/mag`、`/imu/temperature`、`/imu/pressure` 不受影響 |
| frame 名稱 | `imu_link` | 寫入每筆訊息的 `header.frame_id` |
| 發佈 `world → frame` 的 TF | 否 | 只在桌上用 3D 面板或 RViz2 看姿態時開啟;與機器人的 TF 樹一起使用時必須關閉 |
| 網路模式 | `bridge` | `bridge`:只把 9090 / 8765 開給 Foxglove。`host`:加上 `--network host --ipc host`,其他容器與主機上的 ROS 2 節點可直接收到 topic |
| ROS_DOMAIN_ID | `0` | 0–232,須與要互通的節點相同 |

設定存於 `config/launcher.json`(不進版控),每次啟動時讀取;檔案內容不合法時會指出檔案路徑並拒絕啟動。

`host` 模式下不再詢問是否開放區網:埠直接開在主機的所有網路介面,且沒有認證,只在信任的網路使用。
Docker Desktop 的 host 網路指的是它的虛擬機,不是 Windows 本身;在 WSL 內安裝的 Docker Engine 則沒有這個差別。

## Foxglove 連線

1. 用選項 **2**(或 3)啟動。
2. 開啟 Foxglove → **Open connection**,擇一:
   - **Foxglove WebSocket**:`ws://localhost:8765`(建議,效能較好)
   - **Rosbridge**:`ws://localhost:9090`
3. 建議面板:
   | 面板 | 設定 |
   |---|---|
   | **3D** | 需先在選項 8 開啟「發佈 world → frame 的 TF」。Display frame / Fixed frame 設 `world`,看 TF `world → imu_link` 隨 IMU 轉動 |
   | **Plot** | `/imu/data.linear_acceleration.x`、`.y`、`.z`(靜置時 z ≈ 9.8 m/s²);`/imu/data.angular_velocity.z` |
   | **Raw Messages** | `/imu/data`、`/imu/mag`、`/imu/temperature` |
   | **Topic Graph / Diagnostics** | 確認 `/imu/data` 約 10 Hz |

瀏覽器版 app.foxglove.dev 連 `ws://localhost` 不需 TLS;若瀏覽器阻擋,改用 Foxglove 桌面版。

---

## RViz2

選項 **3** 會在容器內啟動 RViz2(Fixed Frame = `world`),可看到 TF 與 `rviz_imu_plugin` 的 IMU 姿態。
要看到姿態,需先在選項 8 開啟「發佈 world → frame 的 TF」(預設關閉)。

- Linux:需要 X11 `DISPLAY`;若出現 `could not connect to display`,執行 `xhost +local:docker` 後重試。
- WSL2:使用 WSLg,通常不需額外設定;`echo $DISPLAY` 應為 `:0`。

---

## 校正流程

選項 **4** 進入容器內的互動校正選單(直接開序列埠,不需 ROS 節點;**請先確認沒有其他容器佔用序列埠**,
`imu.py` 會詢問是否停止正在執行的 `yahboom-imu`)。

```
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
```

某一項在**本次執行中**成功完成後,選單上那一行會以綠色顯示並標上 `✓ 本次已完成`(2–7、9、10,以及逐項驗證全數合格的 12)。
這個標記只存在記憶體中,重新啟動工具後全部消失,因此綠色一定代表「這次啟動後做過」,不會把以前的校正誤認為剛完成。
取消、中斷、磁力計未涵蓋完整、軟體靜態偏移偵測到未放平時不會標記;選項 11 恢復原廠會清除所有標記。

選單每一項後面的 `→` 是簡短的放置提示。選了 2、3、4、5、6、7、9、10 之後,工具會先印出完整的「注意事項」,
硬體校正要等你看完、回答 `y` 才會送出指令。

### 每項校正要在哪裡做

| 項目 | 在哪裡做 | 重點 |
|---|---|---|
| 1 即時資料檢視 | 任何地方 | 只看資料,不寫入 |
| 2 加速度計水平校正 | **真正水平的桌面** | 高度沒差,**水平最重要**,最好用水平儀確認。IMU Z 軸朝上、完全靜止。可以先單獨校正模組,再裝上去 |
| 3 / 4 磁力計校正 | **最終安裝位置與實際使用環境** | 要連同機器人一起轉,機器人身上的鐵件才會一起被補償。機器人太大、無法三軸翻轉時改用 4 雙平面。遠離環境中的鐵件(鐵桌、含鋼筋地板)20 cm 以上 |
| 5 Z 軸航向歸零 | 最終位置 | 把機器人擺成要當作「正前方」的方向再執行,需要時每次使用前重做 |
| 6 設定角度參考 | **裝好後、機器人放在水平地面上** | 用來修正 IMU 裝歪:執行後,目前姿態就算 roll = pitch = 0 |
| 7 高度歸零 | **要當作 0 m 的高度** | 唯一跟「高度」直接有關的一項,例如在地面或出發點執行。氣壓會隨天氣變化,每次使用前最好重做 |
| 8 讀取偏移暫存器 | 任何地方 | 唯讀 |
| 9 軟體靜態偏移 | 水平、靜止,**IMU 通電暖機約 5 分鐘後** | 陀螺儀偏移會隨溫度改變,要在工作溫度下做。這項假設 Z 軸朝上,IMU 裝歪時不要用 |
| 10 軟體 hard-iron | 同 3,最終安裝位置 | X、Y、Z 三軸都要各轉滿一圈,結果才準 |
| 11 恢復原廠設定 | 任何地方 | — |
| 12 姿態檢視 / 校正驗證 | 任何地方 | 唯讀;用來驗證校正結果,見[校正結果驗證](#校正結果驗證選項-12) |
| 13 氣壓高度輸出 開啟/關閉 | 任何地方 | 寫入 IMU 設定;開啟後才看得到高度 |

**關於「放置高度」**:對磁力計來說,重點不是高度本身,而是**離鐵件多遠**。
如果 IMU 實際會貼近含鋼筋的地板或鐵桌運作,就在同樣的距離下校正,不要拿到別處校正完再搬回來。
只有 7 高度歸零是真的要在目標高度做。

> ⚠ **高度資料要先開啟**:CMP10A 出廠預設不輸出氣壓計封包(0x56)。請先用選項 **13** 開啟,
> `/imu/pressure` 與選項 12 姿態檢視才看得到氣壓 / 高度。開啟後每秒多 10 個封包,9600 baud 下約使用 57% 頻寬。
> 未歸零前的高度是以標準大氣壓換算(實測約 -31 m),用選項 7 高度歸零後才以目前位置為 0 m。

**建議順序**

1. (需要時)11 恢復原廠設定
2. 2 加速度計:IMU 單獨放在水平桌面上
3. 把 IMU 裝到機器人上
4. 3 或 4 磁力計:在實際使用的地點
5. 6 角度參考:機器人放在水平地面
6. 5 航向歸零:機器人對準正前方
7. 7 高度歸零:在參考高度
8. 9 / 10 軟體校正:暖機後做,放在對應的硬體校正之後,用來修剩下的誤差
9. 12 姿態檢視:照下方「校正結果驗證」逐項確認

(要看高度的話,在第 7 步之前先用 13 開啟氣壓高度輸出。)

### IMU 擺放位置與磁場干擾

CMP10A 內建磁力計,用來修正**航向角 yaw**。周圍的鐵磁性物體會扭曲地磁場,讓 yaw 偏移或亂跳;
roll / pitch 主要由加速度計與陀螺儀決定,受影響較小。

**避免放在這些東西旁邊**

- 鐵盒、鍍鋅鋼板機殼、鐵桌 / 鐵櫃、鋼製螺絲與支架
- 馬達、喇叭、磁鐵、電池,以及大電流的電源線

鋁、銅、塑膠、木頭不帶磁性,可以當外殼或底座。

**校正與安裝建議**

1. 磁力計校正(選單 3、4、10)時,IMU 要距離鐵件、磁鐵 **20 cm 以上**。這是 WitMotion 手冊的建議。
2. 如果 IMU 一定要裝在金屬旁,而且金屬跟 IMU **固定在一起、會一起動**(例如機器人車架),
   請在**裝好後的最終位置**重做一次磁力計校正。校正可以補償大部分這種固定干擾。
3. 如果金屬固定在環境裡、**不跟 IMU 一起動**(例如 IMU 在鐵桌上移動、靠近鐵櫃),校正補償不了,
   只能拉開距離。

**怎麼確認有沒有受干擾**

- 選單 **1 即時資料檢視**:IMU 放著不動,拿鐵盒靠近再移開。yaw 或 mag 數值明顯跳動,就代表有干擾。
- 用 Foxglove 的 Plot 面板看,步驟見下方「用 Foxglove 檢查磁力計」。

#### 用 Foxglove 檢查磁力計

**設定 Plot 面板**

1. 新增 **Plot** 面板,在左側 **Series** 的 **Y-value path** 輸入 `/imu/mag.magnetic_field.x`。
2. 按 **Series** 右邊的「+」新增兩條 series,分別輸入 `/imu/mag.magnetic_field.y`、`/imu/mag.magnetic_field.z`。
3. 想確認 IMU 有沒有被轉動,可以再加一條 `/imu/data.angular_velocity.z`。轉動的瞬間會出現尖峰。

`/imu/mag` 輸出的是原始數值(raw LSB),不是 Tesla,所以數字落在數千是正常的。
本專案實測的 CMP10A,靜置時三軸合成大小約 4000~4100,僅供參考,不同環境會有差異。

**怎麼判讀**

磁力計量的是地球磁場。地磁場在空間中的方向固定,IMU 一轉,磁場分配到 IMU 各軸的比例就會改變,
所以**轉動時三條線一起起伏是正常的**,yaw 就是靠這個算出來的。轉動時應該**不變**的是三軸合成大小
√(x² + y² + z²)。

| 現象 | 判斷 |
|---|---|
| IMU 不動,三條線平穩,抖動約 ±20~30 | 正常雜訊 |
| 轉動時三條線起伏,但合成大小幾乎不變 | 正常 |
| IMU 不動,線卻在跳 | 附近有干擾,例如有人拿鐵件或手機靠近、馬達在轉 |
| 轉動時合成大小明顯忽大忽小(例如 3500~4500 之間跳) | 周圍有鐵件,或磁力計需要重新校正(校正選單 3) |
| 某一軸一直卡在同一個值、完全不動 | 感測器或資料異常 |

實例:轉動前 (x, y, z) ≈ (2400, -2830, -1680),轉動後 ≈ (2150, -2830, -1950),
各軸都變了,但合成大小約 4070 → 4050,只差約 0.5%,代表只是 IMU 方向改變,沒有受到干擾。

**XY 圖檢查校正品質**

1. 面板設定 **General → X-axis value type** 改成 `Path (accumulated)`。
2. X 軸的 path 填 `/imu/mag.magnetic_field.x`,Series 只留 `/imu/mag.magnetic_field.y`。
3. IMU 水平放著,原地慢慢轉一圈。

| 畫出來的形狀 | 判斷 |
|---|---|
| 接近正圓,圓心靠近原點 | 校正良好 |
| 圓心偏離原點 | 磁力計還沒校正,或附近有跟著 IMU 一起動的鐵件 |
| 變成橢圓,或某個角度突然凹進去 | 周圍有鐵件干擾 |

### 校正結果驗證(選項 12)

「工具顯示完成」或選項 8 讀到偏移值有變,只代表有寫進 IMU,**不代表校正得好**。
IMU 不管有沒有校好都會顯示一個看起來合理的數字,所以要跟**已知的參考**比對才算數。

選項 12 只讀不寫,進入後有兩種模式:

- **Enter:逐項驗證(建議)**。工具依序帶你做水平、每轉 90°(四次)、轉一圈回原位、高度四個步驟:
  照畫面擺好後按 Enter,工具量測並判定,`[合格]` 以綠色、`[不合格]` 以紅色、`[略過]` 以黃色顯示,最後列出總表。
  每一步可按 `s` 略過、`q` 結束。四次 90° 轉完即回到起點,第 3 步直接用它判定,不必再轉。
  沒有氣壓資料時高度一步自動略過。輸出不是終端機或設了 `NO_COLOR` 時不上色。
- **f:自由檢視**。即時顯示 roll(繞 X)、pitch(繞 Y)、yaw(繞 Z)與高度:

```
roll(X)   +0.01°  pitch(Y)   -0.02°  yaw(Z) +111.71°  高度  -31.12 m
```

  - **Enter**:把目前姿態記成參考點,之後改顯示相對變化 Δ
  - **r**:清除參考點;**q** 或 Ctrl+C:離開
  - Δyaw 會自動處理 ±180° 換圈

各項檢查與合格標準:

| 檢查 | 怎麼做 | 合格(經驗值) | 檢查的是 |
|---|---|---|---|
| 水平 | 放在水平面(用水平儀確認) | roll、pitch 都在 ±0.5° 內 | 2 加速度計、6 角度參考 |
| 傾斜 90° | 按 Enter,靠著方形盒子立起來 | Δpitch 或 Δroll ≈ ±90°,誤差約 ±1° | 2 加速度計 |
| yaw 每轉 90° | 靠著方形盒子的邊,每次轉 90° 前按 Enter | Δyaw 每次約 ±90°,誤差 ±3° 內 | 3 / 4 / 10 磁力計 |
| 轉一圈回原位 | 對齊桌上的參考線按 Enter,轉一圈再對回 | Δyaw 回到 0,誤差 ±3° 內 | 3 / 4 / 10 磁力計 |
| 漂移 | 靜置按 Enter,等 5 分鐘 | Δyaw 幾乎不變 | 9 陀螺儀偏移 |
| 高度 | 按 Enter 後抬高 1 m | Δ高度 約 +1 m(有數十公分雜訊) | 7 高度歸零、13 氣壓輸出 |

**方向(yaw)最重要的是「每轉 90°」和「轉一圈回原位」**:磁力計沒校好時,
通常會出現有的 90° 只變 80°、有的變 100°,或轉一圈回來差了好幾度。不合格就回去重做對應的校正。

> 合格標準是依這顆 CMP10A 實測數值抓的經驗值,不是 WitMotion 官方規格。

### 硬體校正(寫入 IMU flash)

硬體校正會透過 WitMotion 指令改變 IMU 內部參數,流程為
**unlock → CALSW(校正模式)→ 等待/使用者操作 → CALSW NORMAL → save(寫入 flash)**。

> ⚠ **注意**
> - 結果會**永久寫入 IMU flash**,斷電後仍保留,也會影響其他使用這顆 IMU 的程式(例如 Windows 上位機)。
> - 每次寫入前都會詢問確認 `(y/N)`,預設為否。
> - 校正期間不要拔線或關閉終端機;中途失敗可重做,或用 **11 恢復原廠設定**。
> - 磁力計校正請遠離金屬桌腳、電腦機殼、磁鐵與馬達,詳見[IMU 擺放位置與磁場干擾](#imu-擺放位置與磁場干擾)。

| 項目 | 用途 | 操作步驟 |
|---|---|---|
| 1 即時資料檢視 | 確認連線與數值;不寫入 | 觀察加速度、角速度、角度、磁力計;Ctrl+C 回選單 |
| 2 加速度計水平校正 | 消除加速度零偏,讓水平時 roll/pitch ≈ 0 | IMU **水平靜置**於桌面(Z 軸朝上)→ 確認 → 不要碰觸,等倒數進度條跑到 100%(約 5 秒)→ 自動儲存 |
| 3 磁力計 球面擬合 | 校正磁力計 hard/soft-iron,改善航向(yaw) | 確認後**繞 X、Y、Z 三軸各緩慢轉數圈** → 進度條出現 `[完成]` 後按 Enter → 儲存 |
| 4 磁力計 雙平面 | 空間受限時的磁力計校正 | 確認後先**水平繞 Z 軸轉一圈,再立起繞 X 或 Y 軸轉一圈** → 進度條出現 `[完成]` 後按 Enter → 儲存 |
| 5 Z 軸航向歸零 | 以目前方向為 yaw = 0 | 擺好方向 → 確認 → 等進度條跑完(約 1 秒) |
| 6 設定角度參考 | 以目前姿態為 roll = pitch = 0(安裝不水平時用) | 擺成要當參考的姿態 → 確認 → 等進度條跑完(約 1 秒) |
| 7 高度歸零 | 以目前氣壓高度為 0 m(需有氣壓計的型號) | 確認 → 等進度條跑完(約 1 秒) |
| 8 讀取偏移暫存器 | 唯讀檢視 AXOFFSET…GZOFFSET、HXOFFSET…HZOFFSET | 直接讀取,不寫入 |
| 11 恢復原廠設定 | 清除所有硬體校正與設定 | 需要兩次確認;執行後建議重做 2、3 |

建議順序與每項要在哪裡做,見[每項校正要在哪裡做](#每項校正要在哪裡做)。

### 進度顯示:怎麼知道何時該結束

校正期間畫面最下面會有一行持續更新的進度條:

| 項目 | 進度條顯示 | 何時結束 |
|---|---|---|
| 2 加速度計 | `校正中,請保持不動 [#####---------------]  25%  剩 4 秒` | 自動結束,跑到 100% 後儲存 |
| 3 / 4 磁力計(硬體) | `X [########] 100%  Y [######--]  75%  Z [##------]  25%   12秒  缺 Y Z` | 三軸都 100% 後出現 `[完成] 按 Enter` → 按 Enter |
| 5 / 6 / 7 | 約 1 秒的倒數進度條 | 自動結束 |
| 9 軟體靜態偏移 | `取樣 [##########----------]  50%  25/50 筆` | 自動結束(50 筆約 5 秒) |
| 10 軟體 hard-iron | 同 3,另顯示已收集筆數 | 三軸都 100% 自動結束,最長 90 秒 |

**磁力計的三軸進度怎麼看**

- 每一軸的進度,是看磁力計在那一軸的讀數有沒有涵蓋到地磁場的正、反兩個方向。
- `缺 Y Z` 表示還沒涵蓋完整的軸。補法:
  - **X 不夠** → 繞 **Y 或 Z** 軸轉
  - **Y 不夠** → 繞 **X 或 Z** 軸轉
  - **Z 不夠** → 繞 **X 或 Y** 軸轉
- 多轉幾圈沒關係,`[完成]` 只是代表「已經夠了」。
- 選項 4 雙平面的轉法掃不滿每一軸的全部範圍(地磁是斜的,只在水平面上轉的那一軸約只掃到八成),
  所以它的滿分門檻比選項 3 寬。某一軸仍停在九成左右時可直接按 Enter,再用選項 12 驗證。
- 進度未滿就按 Enter 也會結束,但會提醒哪幾軸尚未涵蓋完整,建議之後重做。
- 開始校正前,工具會先花約 2 秒量測目前的磁場強度,作為 100% 的基準。
- 選項 10 是自動結束,門檻比 3 / 4 嚴格(要涵蓋約 95%),因為 hard-iron 偏移是用各軸最大、最小值的中點算的,兩端都要碰到才準。

> **為什麼不用陀螺儀算轉了幾圈?** 實測 CMP10A 進入磁力計校正模式後,加速度、陀螺儀、角度的輸出會**凍結不變**,
> 只有磁力計持續更新,所以進度只能從磁力計計算。
> (實機錄製資料:`wit_ros2_imu/test/fixtures/cmp10a_magcal_mode_rotating_9600.bin`)

如果校正時完全沒收到 IMU 資料,進度條會改成依時間判斷:`未收到資料,依時間判斷 [###-----] 25/60秒`,
建議轉滿 60 秒再按 Enter。

### 軟體校正(寫入 YAML,不動 IMU)

軟體校正只把偏移量寫到主機的 `config/imu_calibration.yaml`(容器內 `/config/imu_calibration.yaml`),
ROS driver 啟動時載入,以「量測值 − 偏移」修正輸出。刪除該檔即恢復未校正狀態。

| 項目 | 操作步驟 | 寫入欄位 |
|---|---|---|
| 9 軟體靜態偏移 | IMU **水平靜置、Z 軸朝上**,取樣期間不要碰觸;假設靜置時 accel = (0, 0, +g)、gyro = 0 | `gyro_bias`、`accel_bias` |
| 10 軟體 hard-iron | **繞三軸緩慢轉動**,X、Y、Z 三軸進度都到 100% 會自動結束(最長 90 秒);以各軸 (max+min)/2 作為偏移 | `mag_offset` |

YAML 格式(範例見 `config/imu_calibration.example.yaml`):

```yaml
gyro_bias: [x, y, z]     # rad/s
accel_bias: [x, y, z]    # m/s^2
mag_offset: [x, y, z]    # raw LSB
```

軟體校正完成後,重新用選項 2 或 3 啟動 driver 才會套用。
`config/*.yaml` 已列在 `.gitignore`(只有 `*.example.yaml` 會進版控)。

---

## ROS topic 列表

| Topic / TF | 型別 | 說明 |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | orientation(四元數)、angular_velocity(rad/s)、linear_acceleration(m/s²);每個角度封包發佈一次,約 10 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 磁力計(raw LSB) |
| `/imu/temperature` | `sensor_msgs/Temperature` | 晶片溫度(°C) |
| `/imu/pressure` | `sensor_msgs/FluidPressure` | 氣壓(Pa);需先用校正選單 13 開啟氣壓輸出 |
| TF `world → imu_link` | `tf2_msgs/TFMessage` | IMU 姿態。預設不發佈,在選項 8 開啟 |

在容器 shell(選項 5)內檢查:

```bash
ros2 topic list
ros2 topic hz /imu/data      # 約 10 Hz
ros2 topic echo /imu/data --once
```

driver 參數:`port`(預設 `/dev/ttyUSB0`,可為 pyserial URL)、`baud`(9600)、`frame_id`(`imu_link`)、
`parent_frame`(`world`)、`publish_tf`(true)、`calibration_file`(`/config/imu_calibration.yaml`)、
`reconnect_interval`(2.0 秒;序列埠斷線會自動重連)。

---

## 疑難排解

### 序列埠權限 `Permission denied` / `imu.py` 提示沒有讀寫權限

用 [usb_udev_namer](https://github.com/alianlbj23/usb_udev_namer) 把 IMU 命名為 `usb_imu`,規則會一併開放讀寫權限。
不使用固定名稱時,改為加入 dialout 群組:

```bash
sudo usermod -aG dialout "$USER"   # 然後重新登入(WSL:wsl --shutdown)
```

### WSL 看不到 `/dev/ttyUSB*` 或 `/dev/usb_imu`

- 執行 [wsl_usb_auto](https://github.com/alianlbj23/wsl_usb_auto),選「查看所有規則」確認 IMU 的規則存在且監聽已啟動。
- `/dev/ttyUSB0` 存在但 `/dev/usb_imu` 不存在:在 `usb_udev_namer` 選 `4 Re-apply the rules`;WSL 未啟動 udev 時先執行 `sudo service udev restart`。
- 若被 Windows 程式(例如 WitMotion 上位機)開著,先關閉它。
- WSL 核心太舊時更新:PowerShell 執行 `wsl --update`。

### 埠被佔用(9090 / 8765)

`docker run` 結束碼 125、訊息含 `address already in use`:

```bash
docker ps                          # 是否有舊容器 → imu.py 選 6 停止
sudo lsof -i :8765 -i :9090        # Linux
ss -ltnp | grep -E '8765|9090'     # Linux / WSL
```

### 序列埠被佔用 / 沒有資料

- 同一時間只能有一個程式開啟序列埠:校正工具與 driver 不能同時跑,其他終端機的 `screen`、`minicom` 也要關掉。
- 預設 baud 為 9600;若曾用上位機改過 baud,請改回或在容器 shell 內以 `baud:=<值>` 啟動。

### docker daemon 無法使用

- Linux / WSL(Docker Engine):`sudo service docker start`,確認使用者在 docker 群組。
- Docker Desktop:確認已啟動;WSL 需在 Settings → Resources → WSL integration 啟用該發行版。

### 映像尚未下載

選單選 **1** 下載預建映像。下載失敗時確認網路連線,以及 GitHub 上該映像(Packages)已設為公開。

---

## 開發與測試

```bash
# 主機端腳本測試(需 pytest 與 pyserial==3.5)
python3 -m pytest tests -q
python3 -m flake8 imu.py host tests

# ROS 套件測試(容器內)
docker run --rm ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy bash -c \
  'cd /ws && colcon test --packages-select wit_ros2_imu && colcon test-result --verbose'
```

### 映像只提供環境,程式碼用掛載的

映像 `ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy` 只裝好 ROS 2 與相依套件。
`imu.py` 啟動容器時,會把 repo 的 `wit_ros2_imu/` 以唯讀方式掛載到容器的 `/ws/src/wit_ros2_imu`;
映像內的套件以 `colcon build --symlink-install` 安裝,實際執行的就是掛載進來的程式碼。

| 改了什麼 | 要做什麼 |
|---|---|
| `wit_ros2_imu/` 裡的程式(driver、校正工具、launch) | 重新啟動 `imu.py` 的選項即可,不用重建映像 |
| `imu.py`、教材、README | 不用做任何事 |
| `docker/Dockerfile`、`package.xml`(相依套件)、`setup.py`(執行檔進入點、安裝的檔案) | push 到 `main`,GitHub Actions 自動重建;完成後選 1 重新下載 |

使用者要更新工具,`git pull` 即可。

映像由 `.github/workflows/docker-image.yml` 建置,產出 `linux/amd64` 與 `linux/arm64`,標籤為 `jazzy` 與 `sha-<commit>`;
也可在 GitHub 的 Actions 頁面手動觸發(Run workflow)。

目錄結構:

```
imu.py                      # 主機端互動選單(stdlib only,Python >= 3.8)
tests/                      # 主機端腳本測試(pytest)
docker/                     # Dockerfile、entrypoint.sh
config/                     # 掛載到容器 /config(軟體校正 YAML)
wit_ros2_imu/               # ROS 2 ament_python 套件(driver、協定、校正 CLI、launch、rviz)
docs/SPEC.md                # 規格
```

授權:Apache-2.0。
