# 單元 0:固定 USB 裝置名稱

目標:把 IMU 的裝置名稱固定為 `/dev/usb_imu`。後續所有實作與自走車上的設定都使用這個名稱。

## 為什麼要固定名稱

Linux 依插入順序為 USB 序列裝置編號:第一個插上的是 `/dev/ttyUSB0`,第二個是 `/dev/ttyUSB1`。
自走車上通常同時接著 IMU、馬達控制板、LiDAR 等多個 USB 序列裝置,重開機或重新插拔後編號可能互換,
寫死 `/dev/ttyUSB0` 的設定檔就會連到錯的裝置。

解決方式是由 udev(Linux 管理裝置的服務)在裝置插上時,依裝置本身的特徵建立一個固定名稱的連結:

```
/dev/usb_imu -> ttyUSB0
```

之後不論 IMU 被編成 `ttyUSB0` 或 `ttyUSB3`,`/dev/usb_imu` 都指向它。

本單元使用兩個工具,皆為互動選單,不需要記指令:

| 工具 | 用途 | 誰需要 |
|---|---|---|
| [wsl_usb_auto](https://github.com/alianlbj23/wsl_usb_auto) | 把 USB 裝置從 Windows 綁定進 WSL,之後插上就自動進入 WSL | 只有 Windows + WSL2 的使用者 |
| [usb_udev_namer](https://github.com/alianlbj23/usb_udev_namer) | 把裝置命名為 `/dev/usb_imu` | Linux 與 WSL2 的使用者 |

macOS 沒有 udev,不適用本單元;macOS 的裝置名稱為 `/dev/cu.usbserial-*`,本身已固定,直接使用即可。

## 步驟 1(僅 WSL2):用 wsl_usb_auto 把 IMU 綁定進 WSL

Windows 不會自動把 USB 裝置交給 WSL。原本需要在系統管理員 PowerShell 手動執行 `usbipd bind` 與 `usbipd attach`,
而且每次重新插拔都要重做。`wsl_usb_auto` 把這件事設定成規則,設定一次後,裝置插上就自動進入 WSL。

前置條件:Windows 已安裝 usbipd-win(PowerShell 執行 `winget install usbipd`,安裝後重開 WSL)。

在 WSL 內:

```bash
git clone https://github.com/alianlbj23/wsl_usb_auto.git
cd wsl_usb_auto
python3 wsl_usb_auto.py
```

```
========== WSL USB 自動掛載 ==========
目前規則:0 條
快捷指令:未設定
  1. 新增自動掛載裝置
  2. 刪除規則
  3. 查看所有規則
  4. 查看目前插著的 USB 裝置
  ...
  0. 離開
```

1. 插上 IMU
2. 選 `1 新增自動掛載裝置`,從清單選擇 IMU(名稱含 `CP210x USB to UART`,VID:PID 為 `10c4:ea60`)
3. Windows 跳出系統管理員確認視窗,按「是」
4. 選 `3 查看所有規則`,確認 IMU 的狀態為已掛載

確認裝置已進入 WSL:

```bash
ls /dev/ttyUSB*
```

應看到 `/dev/ttyUSB0`。

## 步驟 2:用 usb_udev_namer 把 IMU 命名為 usb_imu

Linux 原生系統從這一步開始;WSL2 在完成步驟 1 後進行。

```bash
git clone https://github.com/alianlbj23/usb_udev_namer.git
cd usb_udev_namer
python3 usb_udev_namer.py
```

```
========== USB device fixed names (udev) ==========
Names: 0
Shortcut: not set
  1. Name a device
  2. Show all names (with health check)
  3. Delete a name
  4. Re-apply the rules (use when a name is missing after re-plugging)
  5. Show the USB serial devices currently plugged in
  6. Set a shortcut command (then type usbname to open this menu)
  7. Remove the shortcut command
  8. Remove everything (the rules file and all settings)
  0. Quit
Choose [0-8]:
```

選 `1 Name a device`:

```
USB serial devices currently plugged in:
  1. /dev/ttyUSB0  10c4:ea60  Silicon Labs CP2102 USB to UART Bridge Controller  serial 0001  port 1-3.2
Choose a device [1-1] (0 to go back): 1
Enter a name (it will appear under /dev/, e.g. usb_imu; press Enter to cancel): usb_imu

Identified by: VID:PID 10c4:ea60 + serial 0001. Any USB port works.
Identify by physical USB port instead? (you must rename after moving to another port) [y/N]
This will write /etc/udev/rules.d/99-usb-udev-namer.rules (requires sudo). Continue? [y/N] y
[sudo] password for user:
Applying...
lrwxrwxrwx 1 root root 7 Nov 11 10:00 /dev/usb_imu -> ttyUSB0
```

操作順序:

1. 選擇 IMU 所在的那一列
2. 名稱輸入 `usb_imu`(全班統一使用這個名稱)
3. 「Identify by physical USB port instead?」直接按 Enter(維持預設的 N)
4. 「Continue?」輸入 `y`,再輸入 sudo 密碼
5. 最後一行出現 `/dev/usb_imu -> ttyUSB0` 即完成

## 步驟 3:驗證名稱已固定

```bash
ls -l /dev/usb_imu
```

應顯示 `/dev/usb_imu -> ttyUSB0`(箭頭後的編號依各電腦而異)。

接著拔掉 IMU 再插回去,重新執行同一個指令,`/dev/usb_imu` 應仍然存在。這項測試通過才算完成。

之後的實作中,`python3 imu.py` 偵測到 `/dev/usb_imu` 就直接使用,不會再要求選擇序列埠;容器內同樣以 `/dev/usb_imu` 存取。
自走車上的 launch 檔則直接填 `/dev/usb_imu`(見 [08-ros2-integration.md](08-ros2-integration.md))。

## 同時接上兩個相同晶片的裝置

電腦能用來區分兩個 USB 裝置的資訊只有三項:VID:PID(晶片型號)、序號、插在哪個 USB 孔。

CP2102 是很常見的 USB 轉序列埠晶片,許多 ESP32 開發板也使用它,VID:PID 同樣是 `10c4:ea60`,
而且廉價模組的序號常常都是 `0001`。此時 IMU 與 ESP32 在前兩項上完全相同,只能靠 USB 孔區分。

`usb_udev_namer` 會自動判斷:

| 情況 | 辨識方式 | 使用上的差別 |
|---|---|---|
| 「VID:PID + 序號」在目前插著的裝置中唯一 | VID:PID + 序號 | 插哪個 USB 孔都可以 |
| 沒有序號,或與另一個插著的裝置完全相同 | USB 孔 + VID:PID | 固定插同一個孔;換孔須重新命名 |

因此**有兩個以上同型號的裝置時,命名時要全部插著**,工具才看得到它們相同,並改用 USB 孔辨識。

命名時只插了一個、之後才接上另一個相同的裝置時,選單 `2 Show all names` 的健康檢查會顯示該名稱對應到 2 個裝置。
處理方式:選 `3` 刪除該名稱,把兩個裝置都插上,再重新命名。

WSL2 的限制:經 usbipd 進入 WSL 的裝置,其 USB 孔編號由掛載順序決定,與 Windows 上實際插的孔無關,
無法用來穩定區分兩個完全相同的裝置。WSL2 上遇到這種情況時,一次只接一個,或換用序號不同的模組。

## 常見問題

| 現象 | 處理 |
|---|---|
| WSL 內看不到 `/dev/ttyUSB*` | 步驟 1 未完成;在 `wsl_usb_auto` 選 `3` 檢查規則與掛載狀態 |
| `usb_udev_namer` 的清單是空的 | 裝置未插好;WSL2 則是尚未掛載進 WSL |
| 命名後 `/dev/usb_imu` 沒有出現 | 選 `4 Re-apply the rules`;仍沒有時選 `2` 看健康檢查 |
| WSL 上名稱始終不出現 | WSL 未啟動 udev 服務。執行 `sudo service udev restart` 後再選 `4`;長期解法是在 `/etc/wsl.conf` 加入 `[boot]` 與 `systemd=true`,並在 Windows 執行 `wsl --shutdown` |
| 換了 USB 孔後名稱消失 | 該名稱以 USB 孔辨識;插回原本的孔,或重新命名 |
| sudo 驗證失敗 | 密碼錯誤或帳號沒有 sudo 權限;此時工具不會做任何變更 |

## 本單元重點

- `ttyUSB` 的編號依插入順序而定,不可寫死在設定中
- WSL2 先用 `wsl_usb_auto` 把裝置綁定進 WSL,再用 `usb_udev_namer` 命名
- IMU 統一命名為 `usb_imu`,路徑為 `/dev/usb_imu`
- 以「拔掉再插回,名稱仍在」驗證
- 同型號裝置命名時要全部插著
