# 0. 固定裝置名稱

**目標:IMU 永遠叫 `/dev/usb_imu`。**

不固定的話,它有時是 `ttyUSB0`、有時是 `ttyUSB1`,取決於插入順序。

## Windows + WSL2:先把 IMU 送進 WSL

Linux 原生系統跳過這一步。

前置:Windows PowerShell 執行 `winget install usbipd`,然後重開 WSL。

```bash
git clone https://github.com/alianlbj23/wsl_usb_auto.git
python3 wsl_usb_auto/wsl_usb_auto.py
```

1. 插上 IMU
2. 選 `1 新增自動掛載裝置`,選 `CP210x`(`10c4:ea60`)
3. Windows 跳出確認視窗,按「是」

之後 IMU 插上就自動進 WSL。

## 取名

```bash
git clone https://github.com/alianlbj23/usb_udev_namer.git
python3 usb_udev_namer/usb_udev_namer.py
```

1. 選 `1 Name a device`
2. 選 IMU 那一列
3. 名稱輸入 `usb_imu`
4. 其餘按 Enter,最後輸入 `y` 與密碼

## 確認

```bash
ls -l /dev/usb_imu
```

看到 `/dev/usb_imu -> ttyUSB0` 就完成。拔掉再插回,名稱應該還在。

## 出問題時

| 現象 | 處理 |
|---|---|
| WSL 看不到裝置 | `wsl_usb_auto` 選 `3` 看規則是否存在 |
| 取名後名稱沒出現 | `usb_udev_namer` 選 `4`;WSL 先執行 `sudo service udev restart` |
| 同時接了 ESP32 之類的同款晶片 | 取名時**兩個都插著**,工具會自動分辨 |
