# IMU 入門:從原理到裝上自走車(CMP10A 實作課)

本系列教材以 Yahboom CMP10A(10 軸 IMU)為實驗平台,
目標是讓大學生學會把 IMU 用在自走車上:理解慣性量測單元 (Inertial Measurement Unit, IMU) 量到什麼、
決定擺放位置、完成有驗證的校正,並把它接進 ROS 2 的 `robot_localization`、`slam_toolbox`、Nav2。

校正與可視化使用本 repo 提供的工具:`python3 imu.py` 互動選單、Docker 內的 ROS 2 driver 與校正 CLI、Foxglove。

## 學習目標

上完課,學生應能:

1. 說明加速度計、陀螺儀、磁力計、氣壓計各自量到什麼,以及 6 軸、9 軸、10 軸配置的差別
2. 說明陀螺儀累加出的角度為何會漂移 (drift),以及磁力計對 yaw 的作用與限制
3. 依據磁場干擾與座標系慣例,決定 IMU 在自走車上的擺放位置與方向
4. 區分 bias、noise、hard-iron、soft-iron 四種誤差,並選對校正項目
5. 用已知的參考(水平面、直角、轉一圈回原位)驗證校正結果
6. 在自走車的 ROS 2 系統中設定 IMU:driver 參數、TF、`robot_localization` 的 EKF

## 內容

| 順序 | 單元 | 形式 | 教材 |
|---|---|---|---|
| 1 | 單元 0:固定 USB 裝置名稱 | 講解 + 實作 | [00-usb-device-naming.md](00-usb-device-naming.md) |
| 2 | 單元 1:IMU 是什麼、6 / 9 / 10 軸配置 | 講解 + 手機觀察 | [01-what-is-imu.md](01-what-is-imu.md) |
| 3 | 單元 2:CMP10A 模組的特性 | 講解 | [02-cmp10a.md](02-cmp10a.md) |
| 4 | 實作 1:讀出第一筆資料 | 實作 | [03-lab1-first-reading.md](03-lab1-first-reading.md) |
| 5 | 單元 3:擺放與磁場干擾 | 講解 + 干擾實驗 | [04-mounting-and-interference.md](04-mounting-and-interference.md) |
| 6 | 單元 4 + 實作 2:校正 | 實作 | [05-calibration.md](05-calibration.md) |
| 7 | 實作 3:驗證與可視化 | 實作 | [06-lab3-verification-and-visualization.md](06-lab3-verification-and-visualization.md) |
| 8 | 單元 5:把 IMU 接進自走車的 ROS 2 系統 | 講解 + 指令操作 | [08-ros2-integration.md](08-ros2-integration.md) |
| 9 | 總結 | 講解 | [07-summary.md](07-summary.md) |

課程以實際運用為主,不推導數學。卡爾曼濾波只說明用途,重點放在擺放、校正、驗證與 ROS 2 設定。

課程一開始先固定 IMU 的裝置名稱:Windows + WSL2 的使用者以 [wsl_usb_auto](https://github.com/alianlbj23/wsl_usb_auto)
把 IMU 綁定進 WSL,接著所有人以 [usb_udev_namer](https://github.com/alianlbj23/usb_udev_namer) 把 IMU 命名為 `usb_imu`。
後續實作都使用 `/dev/usb_imu`。

授課者請另讀 [instructor-notes.md](instructor-notes.md):器材清單、課前必做(Docker 映像要在課前下載好)、每段的常見卡關點。

## 每組器材

- CMP10A IMU 模組 1 個、USB 線 1 條
- 筆電 1 台:Linux、Windows 11 (WSL2) 或 macOS,**課前已下載 Docker 映像**(見 [instructor-notes.md](instructor-notes.md))
- 水平儀(手機 app 即可)、一個方形盒子或書本(用來量 90°)
- 一塊小鐵片或鐵盒(干擾實驗用)、尺

## 事前準備(學生)

1. 依 repo 根目錄 [README.md](../README.md) 的「前置需求」完成 Docker 安裝;WSL2 使用者另在 Windows 安裝 usbipd-win(`winget install usbipd`)
2. 執行 `python3 imu.py` 選 `7 環境檢查`,確認 docker daemon、映像、序列埠都是綠的
3. 在手機安裝 [phyphox](https://phyphox.org/)(開放原始碼的物理實驗 app,單元 1 會用到)
4. 讀完 [01-what-is-imu.md](01-what-is-imu.md),對加速度計與陀螺儀先有概念

## 教材使用慣例

- 術語第一次出現時附英文,之後只用其中一種
- `程式碼區塊` 內的指令可直接複製執行;`<...>` 表示要自行替換

## 本教材與 repo 其他文件的關係

| 文件 | 用途 |
|---|---|
| [README.md](../README.md) | 工具的完整操作手冊,疑難排解查這裡 |
| [docs/SPEC.md](../docs/SPEC.md) | 工具的規格與實機驗證紀錄 |
