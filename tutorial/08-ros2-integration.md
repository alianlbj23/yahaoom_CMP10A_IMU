# 單元 5:把 IMU 接進自走車的 ROS 2 系統

前面的單元處理的是 IMU 本身:原理、擺放、校正、驗證。本單元處理下一步:
IMU 已裝上自走車並校正完成,如何讓 `robot_localization`、`slam_toolbox`、Nav2 用到它。

本單元假設的車輛組成:差速驅動、輪式編碼器、2D LiDAR,軟體為 ROS 2 Jazzy、
`robot_localization`、`slam_toolbox`、Nav2。

> 第 4 節的設定檔取自 [urdf_odom_pkg](https://github.com/alianlbj23/urdf_odom_pkg),該套件的預設值針對模擬車。
> 本單元的內容尚未以實體 CMP10A 在實車上驗證。
> topic 與 frame 名稱須依實際車輛調整。

## 1. IMU 在自走車上提供什麼

`/imu/data`(`sensor_msgs/Imu`)有三組資料,在差速車上的用途不同:

| 欄位 | 內容 | 在自走車上 |
|---|---|---|
| `angular_velocity.z` | 繞 Z 軸的角速度 | **主要用途**。輪子打滑時里程計的轉向會錯,陀螺儀不受影響 |
| `orientation` | 模組融合後的姿態(四元數) | roll、pitch 可用;yaw 依賴磁力計,馬達附近不可靠,預設不使用 |
| `linear_acceleration` | 三軸加速度 | 平地上的差速車通常不融合;雜訊大,累加成速度或位置的誤差增長很快 |

因此上車後最需要顧好的校正是**陀螺儀偏移**(校正選單 9),其次才是磁力計。

各套件的分工:

| 套件 | 輸入 | 輸出 |
|---|---|---|
| 輪速節點 | 編碼器(輪子的轉速) | 車子的前進速度,短距離準確,打滑時出錯 |
| `wit_ros2_imu` | CMP10A | `/imu/data`:角速度,不受打滑影響,長時間會漂 |
| `robot_localization`(EKF) | 輪速 + `/imu/data` | `/odometry/filtered` 與 TF `odom → base_link`:平滑、連續的位姿 |
| `slam_toolbox` | LiDAR + TF `odom → base_link` | 地圖與 TF `map → odom`:以環境特徵修正長時間累積的誤差 |
| Nav2 | 以上全部 | 路徑規劃與速度指令 |

### 名詞:EKF 與 robot_localization

這兩個名稱在本單元反覆出現。兩者不是同一個東西,而是「方法」與「實作該方法的套件」的關係。

| 名稱 | 是什麼 |
|---|---|
| EKF (Extended Kalman Filter,擴展卡爾曼濾波器) | 一種演算法:依各感測器的可信程度合併資料,估計車子的位置、朝向與速度 |
| `robot_localization` | ROS 2 的套件,內含已寫好的 EKF。使用者只需撰寫設定檔,不需自行實作演算法 |

EKF 是卡爾曼濾波器的一種,用於處理車輛轉彎這類非直線的運動。使用時不需區分兩者,
也不需了解其數學;作用與 IMU 模組內部的融合相同,差別在於這一層的設定由使用者負責(第 4、5 節)。

使用 `robot_localization` 時會看到下列幾個名稱,指的是同一件事的不同層面:

| 名稱 | 意思 |
|---|---|
| `robot_localization` | 套件名稱,安裝與啟動時使用 |
| `ekf_node` | 套件中執行 EKF 的程式 |
| `ekf_filter_node` | 該程式啟動後的節點名稱,須與設定檔第一行相同 |
| `ekf.yaml` | 設定檔:指定要融合哪些感測器、各取哪些量 |
| `/odometry/filtered` | 節點輸出的融合結果 |

本單元後續提到「EKF」時,指的是 `robot_localization` 套件的 `ekf_node` 節點。
`robot_localization` 另提供採用不同演算法的 `ukf_node`,用途相同;一般使用 `ekf_node` 即可。

第 4 節以實際的套件 [urdf_odom_pkg](https://github.com/alianlbj23/urdf_odom_pkg) 為例,說明輪速、IMU、LiDAR 如何組合。

## 2. frame 與 TF:誰發佈哪一段

ROS 的慣例([REP-105](https://www.ros.org/reps/rep-0105.html))規定移動機器人的座標系為一棵樹:

```
map ──▶ odom ──▶ base_link ──▶ imu_link
                           └─▶ laser
```

圖中每個名稱是一個 frame(座標系),箭頭是一段轉換 (transform),表示「子 frame 相對於父 frame 的位置與方向」。
frame 不是 ROS 節點;它只是一個名稱,附在每筆感測器資料的 `header.frame_id` 上,說明這筆資料是在哪個座標系下量到的。

### 每個 frame 的意思

| frame | 代表什麼 | 原點在哪裡 |
|---|---|---|
| `map` | 地圖座標系,固定在環境上 | 建圖時的起點 |
| `odom` | 里程計座標系,同樣固定在環境上,但位置是由車子自己的感測器推算的 | 車子啟動時所在的位置 |
| `base_link` | 車體座標系,跟著車子移動 | 車體上的一個固定點,通常是兩驅動輪的中點 |
| `imu_link` | IMU 的座標系,固定在車體上 | IMU 模組所在位置 |
| `laser` | LiDAR 的座標系,固定在車體上 | LiDAR 的旋轉中心 |

### 為什麼是這樣的相依關係

TF 規定每個 frame 只能有一個父 frame,整體是一棵樹。有了這棵樹,任意兩個 frame 之間的關係都能沿著樹查出來。
例如 LiDAR 量到前方 2 m 有障礙物,這筆資料屬於 `laser`;沿著 `laser → base_link → odom → map` 一路換算,
就得到障礙物在地圖上的位置。樹中任何一段缺少,這條換算路徑就斷掉,下游節點會報錯。

各段的相依理由:

- **`imu_link`、`laser` 掛在 `base_link` 下**:感測器鎖在車體上,和車體的相對位置固定。
  感測器資料必須先換算到車體座標系,才能和其他感測器的資料合併。IMU 的角速度要先知道 IMU 在車上朝哪個方向,才知道車子往哪邊轉。
- **`base_link` 掛在 `odom` 下**:這一段描述車子從啟動到現在移動了多少,由輪式里程計與 IMU 推算。
  它連續、平滑、更新快,適合用來控制車子;缺點是誤差會累積,跑久了會偏離真實位置。
- **`odom` 掛在 `map` 下**:這一段是修正量。`slam_toolbox` 用 LiDAR 比對地圖,算出車子在地圖上的真實位置,
  再與里程計推算的位置相減,差值就是 `map → odom`。里程計沒有誤差時,這一段固定不動;誤差越大,這一段偏移越多。

車子在地圖上的位置為何不直接用 `map → base_link` 一段表示:LiDAR 比對的結果準確但更新慢,而且修正時位置會瞬間跳動;
里程計與 IMU 的結果更新快而連續,但會漂。拆成兩段後,控制器使用連續的 `odom → base_link`,
導航規劃使用加上修正的 `map → base_link`,兩種需求都能滿足。同時因為每個 frame 只能有一個父 frame,
`base_link` 的父節點已經是 `odom`,SLAM 不能再直接發佈 `map → base_link`,只能發佈 `map → odom`。

### 這些 frame 是怎麼產生的

frame 不需要事先宣告或建立。只要有節點發佈一段轉換,轉換中提到的父 frame 與子 frame 就出現在 TF 樹中;
沒有任何節點發佈的 frame 不存在。

轉換分為兩種,發佈到不同的 topic:

| 種類 | topic | 特性 | 本例中的轉換 |
|---|---|---|---|
| 靜態轉換 | `/tf_static` | 數值不變,發佈一次即可 | `base_link → imu_link`、`base_link → laser` |
| 動態轉換 | `/tf` | 數值隨時間變化,須持續發佈 | `odom → base_link`、`map → odom` |

各段由下列節點產生:

| 轉換 | 產生的節點 | 依據 |
|---|---|---|
| `base_link → imu_link`、`base_link → laser` | `robot_state_publisher` | 讀取車輛的 URDF,把其中每個固定關節發佈成靜態轉換;沒有 URDF 時改用 `static_transform_publisher`,數值由指令參數給定 |
| `odom → base_link` | `robot_localization` 的 EKF | 融合輪速與 `/imu/data` 後算出車子的位姿,持續發佈(第 4 節的實例為 50 Hz) |
| `map → odom` | `slam_toolbox`(或 AMCL) | LiDAR 掃描與地圖比對的結果,扣掉 `odom → base_link` 後發佈 |

因此啟動順序上沒有限制,但全部節點都啟動後,樹才完整。少了 EKF,`odom` 與 `base_link` 之間就斷開;
少了 URDF 或 static transform,`imu_link` 不存在,EKF 無法使用 IMU 的資料。

**每一段轉換只能有一個發佈者。** 兩個節點發佈同一段,TF 會在兩個值之間跳動,導航無法使用。
上車時有兩處必須關掉:

1. **IMU driver 的 TF**。本 repo 的 driver 預設發佈 `world → imu_link`,用途是桌上測試時在 RViz2 與 Foxglove 顯示姿態。
   上車後 `imu_link` 的父節點是 `base_link`,須把參數 `publish_tf` 設為 `false`。
2. **其他會發佈 `odom → base_link` 的節點**。這一段由 EKF 發佈,不可再有第二個來源。
   以 `urdf_odom_pkg` 為例,`urdf_odom.launch.py`(單純以輪速推算、不經 EKF)也會發佈這一段,
   不可與 `localization.launch.py` 或 `cus_slam.launch.py` 同時執行。

### 描述 IMU 的安裝位置

車輛已有 URDF 時,在 URDF 加一個固定關節:

```xml
<link name="imu_link"/>
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <!-- xyz:IMU 相對 base_link 的位置(公尺);rpy:安裝角度(弧度) -->
  <origin xyz="0.05 0.0 0.10" rpy="0 0 0"/>
</joint>
```

沒有 URDF 時,用 static transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.05 --y 0.0 --z 0.10 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id imu_link
```

模組的 X 箭頭對準車頭、元件面朝上時,`rpy` 為 `0 0 0`。
模組在水平面上轉了 90° 安裝(X 箭頭朝車子左側)時,`yaw` 填 `1.5708`。
這個角度填錯,EKF 會把轉彎的方向或大小算錯。

## 3. 在車上啟動 driver

本 repo 的 `imu.launch.py` 是桌上測試用的,會一併啟動 rosbridge 與 foxglove_bridge,且沒有開放 `publish_tf` 參數。
上車時把 `wit_ros2_imu` 套件放進車輛的 workspace,並在車輛自己的 launch 檔中直接啟動節點。

```bash
# 車上電腦
cd ~/ros2_ws/src
cp -r <本 repo>/wit_ros2_imu .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wit_ros2_imu
source install/setup.bash
```

在車輛的 launch 檔中加入 `wit_ros2_imu` 套件的 `wit_ros2_imu` 節點,並依下表設定參數。

driver 的參數:

| 參數 | 預設值 | 上車時的設定 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 固定名稱,例如 `/dev/usb_imu`;車上通常同時接多個 USB 序列裝置,`ttyUSB` 的編號會隨插入順序改變 |
| `baud` | `9600` | 與模組設定一致 |
| `frame_id` | `imu_link` | 與 URDF 的 link 名稱一致 |
| `publish_tf` | `true` | `false`;上車後關閉,TF 由 URDF 與 EKF 負責 |
| `parent_frame` | `world` | `publish_tf` 為 `false` 時無作用 |
| `calibration_file` | `/config/imu_calibration.yaml` | 軟體校正 YAML 在車上的實際路徑;檔案不存在時不套用校正 |
| `reconnect_interval` | `2.0` | 序列埠斷線後重連的間隔(秒),一般不需更改 |

軟體校正的 YAML 是在哪台電腦上產生的,就存在哪台電腦。在筆電上做的軟體校正要複製到車上,
或直接在車上重做;硬體校正存在模組內,不需搬移。

## 4. 實例:urdf_odom_pkg

[urdf_odom_pkg](https://github.com/alianlbj23/urdf_odom_pkg) 是一個完整的範例套件:由輪子轉速算出車速,
以 `robot_localization` 與 IMU 融合,再由 `slam_toolbox` 用 LiDAR 建立 2D 地圖。
它的預設值是針對 Isaac Sim 中的模擬車,接上實體的 CMP10A 時要改的地方列在本節最後。

### 套件的組成

| 部分 | 做什麼 | 輸出 |
|---|---|---|
| `wheel_twist_node` | 讀 `/joint_states` 中的輪子轉速,依輪半徑與輪距換算成車體速度 | `/wheel_twist` |
| `ekf_node`(`robot_localization`) | 融合 `/wheel_twist` 與 IMU | `/odometry/filtered`、TF `odom → base_link` |
| static transform | 描述 LiDAR 與 IMU 在車上的安裝位置 | TF `base_link → laser_frame`、`base_link → <IMU frame>` |
| `slam_toolbox` | 以 LiDAR 掃描與上述 TF 建圖 | `/map`、TF `map → odom` |

對照第 2 節的 TF 樹,每一段轉換各有一個發佈者,整棵樹完整。

### 三種感測器各自進入哪一層

| 感測器 | 進入哪裡 | 用途 |
|---|---|---|
| 輪子(編碼器) | EKF | 提供前進速度 |
| IMU | EKF | 提供轉向的角速度 |
| LiDAR | `slam_toolbox` | 比對環境,修正 EKF 累積的誤差(`map → odom`) |

LiDAR **沒有**進入 EKF。EKF 只融合輪速與 IMU,輸出連續的 `odom → base_link`;LiDAR 在上一層由 `slam_toolbox` 使用。
套件另有一個以 LiDAR 掃描比對估算里程計的實驗性 launch(`lidar_odom_slam.launch.py`),其結果目前未接入 EKF。

### 啟動方式

```bash
# 建圖:輪速 + IMU 融合 + slam_toolbox(建議的進入點)
ros2 launch urdf_odom_pkg cus_slam.launch.py use_sim_time:=false

# 只要 odom -> base_link 與 /odometry/filtered,不建圖
ros2 launch urdf_odom_pkg localization.launch.py use_sim_time:=false

# 儲存目前的地圖
ros2 launch urdf_odom_pkg save_map.launch.py map_name:=my_map
```

實體車須加上 `use_sim_time:=false`;預設的 `true` 是給模擬器用的。
輪子的關節名稱、輪半徑、輪距以 launch 參數傳入,名稱與預設值見該套件的 README。

### EKF 設定檔

套件的 `config/ekf.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 15 個值的順序:
    # [x, y, z, roll, pitch, yaw,
    #  vx, vy, vz, vroll, vpitch, vyaw,
    #  ax, ay, az]

    # 輪速:只融合前進速度 vx
    twist0: /wheel_twist
    twist0_config: [false, false, false,
                    false, false, false,
                    true,  false, false,
                    false, false, false,
                    false, false, false]
    twist0_queue_size: 10
    twist0_nodelay: true

    # IMU:只融合 Z 軸角速度 vyaw
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_queue_size: 20
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: false

    print_diagnostics: true
    debug: false
```

各項設定的作用:

| 設定 | 值 | 作用 |
|---|---|---|
| `frequency` | `50.0` | EKF 每秒輸出 50 次位姿 |
| `sensor_timeout` | `0.2` | 超過 0.2 秒沒有任何感測器資料時,EKF 仍依模型繼續推算 |
| `two_d_mode` | `true` | 車子只在平面移動,不估計高度、roll、pitch |
| `publish_tf` | `true` | 由 EKF 發佈 `odom → base_link` |
| `world_frame` | `odom` | 這個 EKF 負責 `odom → base_link`;`map → odom` 留給 `slam_toolbox` |
| `twist0` | `/wheel_twist` | 第一個輸入:輪速 |
| `twist0_config` | 只有第 7 個值為 `true` | 只採用輪速的前進速度 `vx` |
| `imu0` | `/imu` | 第二個輸入:IMU 的 topic |
| `imu0_config` | 只有第 12 個值為 `true` | 只採用 IMU 的 Z 軸角速度 `vyaw` |
| `imu0_relative`、`imu0_differential` | `false` | 與姿態角有關的選項;只融合角速度時不起作用 |
| `imu0_remove_gravitational_acceleration` | `false` | 是否扣除重力;未融合加速度時不起作用 |
| `*_queue_size` | `10`、`20` | 每個輸入暫存的訊息筆數 |
| `*_nodelay` | `true` | 關閉網路層的延遲合併,資料到達後立即處理 |
| `print_diagnostics` | `true` | 在 `/diagnostics` 輸出 EKF 的狀態,供除錯使用 |

`*_config` 的 15 個布林值順序固定,`true` 表示把該量交給 EKF。
這份設定的分工很明確:**前進速度只信輪子,轉向只信 IMU**。
輪子打滑時算出的轉向不可靠,所以轉向完全交給陀螺儀;陀螺儀量不到前進速度,所以前進速度交給輪子。
也因此陀螺儀偏移(校正選單 9)直接決定這份設定下的轉向精度。

### 接上實體 CMP10A 時要修改的地方

套件的預設值對應模擬器中的 IMU,換成本課的 CMP10A 時須修改三處:

| 項目 | 套件的預設 | 改為 | 改哪裡 |
|---|---|---|---|
| IMU 的 topic | `/imu` | `/imu/data` | `config/ekf.yaml` 的 `imu0`。只改 launch 參數 `imu_topic` 不會生效 |
| IMU 的 frame | `sim_imu` | `imu_link` | `cus_slam.launch.py` 中 `base_link → sim_imu` 的 static transform;位置與角度一併改為實際的安裝值 |
| 時間來源 | `use_sim_time` 為 `true` | `false` | 啟動時加上 `use_sim_time:=false` |

另外,`wit_ros2_imu` driver 的 `publish_tf` 須設為 `false`(第 3 節),並確認 `/imu/data` 的 `header.frame_id` 是 `imu_link`。

以上三處是依該套件的設定檔與 README 整理的,尚未以實體 CMP10A 搭配該套件實際執行過。

### 何時可以融合 IMU 的 yaw

模組能遠離馬達與鐵件(例如裝在桿子上),且上車後的磁力計驗證合格時,可以把 `imu0_config` 第 6 個值(yaw)改為 `true`,
並加上 `imu0_relative: true`,使 yaw 以啟動時的方向為 0。
開啟前須先確認馬達運轉時 yaw 不受影響;無法確認時維持 `false`。

### 下游套件要對應修改的地方

| 套件 | 設定 | 值 |
|---|---|---|
| `slam_toolbox` | `odom_frame`、`base_frame`、`scan_topic` | `odom`、`base_link`、`/scan`。`urdf_odom_pkg` 的 `config/slam_config.yaml` 已如此設定 |
| Nav2 | `bt_navigator`、`velocity_smoother` 等的 `odom_topic` | `/odometry/filtered` |
| Nav2 | 各 costmap 與 controller 的 `robot_base_frame` | `base_link` |

## 5. covariance:告訴 EKF 該相信誰

`sensor_msgs/Imu` 的每組資料都附帶 covariance,表示該組資料的不確定程度。
EKF 以這個數字決定權重:**數字越大,越不採信**。

本 repo 的 driver 把 covariance 寫成固定值,位置在 `wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py`:

| 常數 | 目前的值 | 意義 |
|---|---|---|
| `_GYRO_STD_RAD_S` | 0.1 °/s | 角速度的不確定度 |
| `_ROLL_PITCH_STD_RAD` | 0.5° | roll、pitch 的不確定度 |
| `_YAW_STD_RAD` | 2.0° | yaw 的不確定度 |
| `_ACCEL_STD_M_S2` | 0.01 g | 加速度的不確定度 |

調整方向:

| 現象 | 調整 |
|---|---|
| 轉彎時融合結果的轉向反應遲鈍 | 調小 IMU 的角速度 covariance |
| 靜止時融合出的朝向緩慢轉動 | 陀螺儀偏移未校正;先重做校正選單 9,而非調 covariance |
| 融合結果抖動 | 調大雜訊較大那個來源的 covariance |

輪速的 covariance 由輪速節點(`wheel_twist_node`)提供。若其值全為 0,EKF 無法正確加權,須補上合理的值。

covariance 只能調整權重,不能補償 bias。校正沒做好時,調 covariance 無法解決漂移。

## 6. 檢查與除錯

啟動全部節點後,依序檢查:

```bash
# 1. IMU 有資料,頻率正確(本模組預設約 10 Hz)
ros2 topic hz /imu/data

# 2. 靜止時 angular_velocity 接近 0;frame_id 是 imu_link
ros2 topic echo /imu/data --once

# 3. EKF 有輸出
ros2 topic hz /odometry/filtered

# 4. TF 樹:產生 frames 的 PDF,確認是一條鏈,每段只有一個發佈者
ros2 run tf2_tools view_frames

# 5. 個別轉換
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link

# 6. EKF 的診斷訊息
ros2 topic echo /diagnostics
```

常見問題:

| 現象 | 原因 | 處理 |
|---|---|---|
| RViz2 中車子在兩個位置之間跳動 | `odom → base_link` 有兩個發佈者 | 只保留 EKF 發佈這一段;不要同時執行 `urdf_odom.launch.py` |
| TF 樹出現獨立的 `world → imu_link` | driver 的 `publish_tf` 未關 | 設為 `false` |
| EKF 警告找不到 `imu_link` 到 `base_link` 的轉換 | 缺少 `base_link → imu_link`,或 `frame_id` 名稱不一致 | 補上 URDF 關節或 static transform;核對名稱 |
| 車子右轉,融合結果顯示左轉 | IMU 安裝方向與 TF 描述不符(例如模組倒裝) | 修正 `base_link → imu_link` 的 `rpy` |
| `/odometry/filtered` 沒有資料 | `twist0`、`imu0` 的 topic 名稱錯誤,或 `use_sim_time` 與實際情況不符 | `ros2 topic list` 核對名稱 |
| driver 啟動後不斷重連 | `port` 錯誤、裝置名稱未固定、或另一個程式佔用序列埠 | 確認 `/dev/usb_imu` 存在;關閉校正工具 |
| 多台電腦之間 TF 逾時 | 各電腦時鐘不同步 | 以 chrony 或 NTP 同步時間 |

### 輸出頻率的限制

CMP10A 預設 9600 baud、約 10 Hz。對低速自走車的 EKF 而言可以運作,但偏低;一般建議 50 Hz 以上。
提高輸出頻率須同時提高鮑率,目前本 repo 的校正工具沒有這項設定,可用廠商的 Windows 上位機
(repo 內 `IMU standard PC software` 資料夾)調整,調整後 driver 的 `baud` 參數要跟著改。

## 本單元重點

- 差速車主要使用 IMU 的 Z 軸角速度;yaw 預設不融合
- TF 鏈為 `map → odom → base_link → imu_link`,每段只能有一個發佈者
- 上車時關閉 driver 的 `publish_tf`;`odom → base_link` 只由 EKF 發佈
- 以 URDF 或 static transform 描述 IMU 的安裝位置與方向
- 以 `urdf_odom_pkg` 為例:EKF 的輪速只取 `vx`,IMU 只取 `vyaw`;LiDAR 由 `slam_toolbox` 使用,不進 EKF
- covariance 調的是權重,不能取代校正

## 參考資料

- [urdf_odom_pkg](https://github.com/alianlbj23/urdf_odom_pkg):本單元第 4 節的實例
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)、
  [官方 ekf.yaml 範本](https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/params/ekf.yaml)
- [Nav2](https://github.com/ros-navigation/navigation2);其官方文件 Setup Guides 中的 Setting Up Odometry 一節說明里程計與 robot_localization 的搭配
- [REP-105 Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)、
  [REP-103](https://www.ros.org/reps/rep-0103.html)、[REP-145](https://www.ros.org/reps/rep-0145.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
