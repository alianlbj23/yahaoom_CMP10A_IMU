# 總結

## 校正與驗證的邏輯

> 感測器有 bias → 用已知輸入量出 bias → 已知輸入決定校正要在哪裡做 → 用另一組已知輸入驗證。

三種感測器各自的已知輸入:

| 感測器 | 已知輸入 | 校正的條件 |
|---|---|---|
| 加速度計 | 水平靜置時 z 軸為 1 g | 水平桌面、靜止 |
| 陀螺儀 | 靜止時角速度為 0 | 靜止、已通電暖機 |
| 磁力計 | 轉動時三軸合成大小不變 | 最終安裝位置、遠離鐵件、三軸轉滿 |

## 從模組到自走車的流程

| 步驟 | 內容 | 教材 |
|---|---|---|
| 1 | 固定裝置名稱為 `/dev/usb_imu` | [00-usb-device-naming.md](00-usb-device-naming.md) |
| 2 | 了解各感測器量到什麼、各種配置的差別 | [01-what-is-imu.md](01-what-is-imu.md)、[02-cmp10a.md](02-cmp10a.md) |
| 3 | 讀出資料,確認靜置時的讀數合理 | [03-lab1-first-reading.md](03-lab1-first-reading.md) |
| 4 | 決定安裝位置與方向,避開磁場干擾 | [04-mounting-and-interference.md](04-mounting-and-interference.md) |
| 5 | 校正:加速度計、磁力計、零點、陀螺儀偏移 | [05-calibration.md](05-calibration.md) |
| 6 | 以已知的參考驗證校正結果 | [06-lab3-verification-and-visualization.md](06-lab3-verification-and-visualization.md) |
| 7 | 接進 ROS 2:TF、`robot_localization`、`slam_toolbox` | [08-ros2-integration.md](08-ros2-integration.md) |

## 裝上自走車後的重點

- 模組裝到最終位置後,磁力計須重新校正並重新驗證
- 差速車主要使用 IMU 的 Z 軸角速度,最需要顧好的是陀螺儀偏移
- TF 樹的每一段轉換只能有一個發佈者
- 輪速與 IMU 經 EKF 合成 odom;LiDAR 由 `slam_toolbox` 使用,修正 odom 累積的誤差

## 延伸閱讀

- O. J. Woodman, *An introduction to inertial navigation*, UCAM-CL-TR-696, 2007 — 誤差模型與 Allan variance
- S. O. H. Madgwick, *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*, 2010 — 開源 AHRS
- NXP AN4246, *Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference*
- [REP-103](https://www.ros.org/reps/rep-0103.html)、[REP-145](https://www.ros.org/reps/rep-0145.html)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)、[Nav2](https://github.com/ros-navigation/navigation2)、[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- 本 repo:[README.md](../README.md)(完整操作)、[docs/SPEC.md](../docs/SPEC.md)(規格)
