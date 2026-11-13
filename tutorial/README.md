# CMP10A IMU 教學

把 IMU 裝進自走車要會的事,共 7 頁,每頁一件事。

## 這個裝置能做什麼

- 量**轉多快**:車子轉彎的角速度
- 量**有多斜**:車身的傾斜(roll、pitch)
- 量**朝哪邊**:車頭方向(yaw),以磁北為準
- 量**高度變化**:氣壓計(預設關閉)

自走車主要用第一項:輪子打滑時,IMU 仍然知道車子轉了多少。

## 它不能做什麼

- 不能單獨算出位置(誤差累積太快)
- 靠近馬達、鐵件時,「朝哪邊」會不準

## 照順序做

| # | 做什麼 | 頁面 |
|---|---|---|
| 0 | 把裝置名稱固定成 `/dev/usb_imu` | [00-usb-device-naming.md](00-usb-device-naming.md) |
| 1 | 認識 IMU | [01-what-is-imu.md](01-what-is-imu.md) |
| 2 | 讀出資料 | [02-read-data.md](02-read-data.md) |
| 3 | 決定裝在哪 | [03-mounting.md](03-mounting.md) |
| 4 | 校正 | [04-calibration.md](04-calibration.md) |
| 5 | 確認校正成功 | [05-verification.md](05-verification.md) |
| 6 | 接進 ROS 2 | [06-ros2.md](06-ros2.md) |

## 需要準備

- CMP10A 模組、USB 線
- 筆電(Linux 或 Windows + WSL2),已裝 Docker
- 先執行 `python3 imu.py` 選 `1` 下載映像(約 2 GB)
- 工具有更新時,在 repo 資料夾執行 `git pull` 即可,不用重新下載映像
- 水平儀(手機 app)、方形盒子、一塊小鐵片

授課者另見 [instructor-notes.md](instructor-notes.md)。
