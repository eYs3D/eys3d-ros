# eYs3D 立體深度相機 ROS 2 驅動程式

[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy%20%7C%20Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/rolling/Releases.html)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` 是 eYs3D 立體深度相機的官方 ROS 2 驅動程式，發布
彩色影像、深度影像與點雲，遵循 REP-103 frame tree。支援 ROS 2
Foxy、Humble、Jazzy。

### 支援的相機

| 型號 | Product code | USB | 狀態 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量產 |
| **R77** | YX8072 | USB 2.0 | 量產 |
| **G62** | YX8081 | USB 2.0 | 量產 |

---

## 安裝

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
```

將此 package 放入 workspace 的 `src/` 後建置：

```bash
cd ~/ros2_ws
colcon build --packages-select eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

per-model launch 預設會開啟 RViz；安裝一次
`sudo apt install ros-$ROS_DISTRO-rviz2`，或加 `rviz:=false` 略過視覺化。

---

## 快速開始

每個型號都有對應的 launch 捷徑，自動套用該機型的預設 video mode：

```bash
ros2 launch eys3d_camera G100P.launch.py     # G100+:L'+D 1280x720 interleave，SDK 每串流 30 fps
ros2 launch eys3d_camera R77.launch.py       # R77: L'+D 1280x920 color + 640x460 depth @ 30 fps
ros2 launch eys3d_camera G62.launch.py       # G62: L'+D 640x480 @ 25 fps
```

要切換 mode 用 `mode_id:=<n>`；各機型完整 mode 列表位於
`launch/video_modes/<MODEL>.yaml`。

### 發布的 Topic

以 `camera_name=G100P_1` 為例：

| Topic | 型別 | 說明 |
|---|---|---|
| `/G100P_1/left_color` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 與 MJPEG 來源皆 inline 解碼為 RGB888；灰階感測模組以 R=G=B 輸出）。是否經過校正取決於當前的 video mode。|
| `/G100P_1/right_color` | `sensor_msgs/Image` | 右眼彩色影像；僅在 video mode 帶有 L\|R 並排輸出時發布（`split_lr: true`）|
| `/G100P_1/depth_image` | `sensor_msgs/Image` (`16UC1`，mm，REP-118) | 深度（毫米） |
| `/G100P_1/pointcloud` | `sensor_msgs/PointCloud2` | XYZ float32，已轉換為 ROS 基底軸（公尺）；當 `colored_pointcloud:=true` 時改為 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相機內參，每張 Image 一張（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康狀態，1 Hz |

### 訂閱端 QoS

影像、深度與 PointCloud2 topic 採用 `SensorDataQoS`
（BestEffort + KeepLast 5 + Volatile）。訂閱端必須宣告相同 QoS：

```cpp
sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(), callback);
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, topic, callback, qos_profile=qos_profile_sensor_data)
```

新版 ros2cli 的 `ros2 topic hz` / `echo` 會自動 negotiate QoS，因此
**不需要額外 flag**。

### Interleave 模式（僅 G100+）

雙目相機沒有獨立的第三目用於 RGB 輸出。
為了同時取得「不含 IR 紅外點的 color」與「高品質的 depth」，
eYs3D 提出 Interleave 模式來達到此需求：

- **偶數 frame —— IR 關閉 → `/left_color`**。
  沒有 IR 紅外點干擾，輸出乾淨的 RGB 影像。
- **奇數 frame —— IR 開啟 → `/depth_image`**。
  IR 紅外點為立體匹配提供結構特徵，得到較高品質的深度。

驅動在 SDK 串流源頭做奇偶 frame 過濾，因此每條串流的 FPS 會減半。

#### 時戳影響

以 G100+ 預設 `mode_id=1` 為例：

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color   30 fps
                                 └  /G100P_1/depth_image  30 fps
```

color 與 depth **來自相鄰兩張 sensor frame**，stamp 相差
**1 個 sensor frame ≈ 15.6 ms（1000 / 60）**。
同 stream 內 `Image` 與其 `CameraInfo` stamp 一致。

下游用 `message_filters::ApproximateTime` 配對，`slop` 設一個
sensor frame 週期（例如 20 ms）即可穩定配對。

#### 影響的 mode IDs

於 `launch/video_modes/G100P.yaml` 中，**`mode_id` 1、3、5、7–21** 為 interleave。
其他 mode 為 color 與 depth 同時擷取，stamp 一致。

---

## 設定

### 切換 video mode

```bash
ros2 launch eys3d_camera G100P.launch.py mode_id:=7    # G100+:L'+D 640x480 interleave（SDK 30 fps）
ros2 launch eys3d_camera R77.launch.py   mode_id:=4    # R77:D-only 640x460 @ 30 fps
ros2 launch eys3d_camera G62.launch.py   mode_id:=3    # G62:L'+D 320x240 @ 30 fps
```

每台相機完整的 video mode 列表位於 `launch/video_modes/<MODEL>.yaml`。
節點啟動時會在 RCLCPP INFO log 印出完整列表；per-model launch 預設
`log:=sdk` 會壓掉這層 log，要看請改用 `log:=all`（或直接打開 YAML）。

### Launch 參數

| 參數 | 預設值 | 說明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace 與 frame-id 前綴 |
| `mode_id` | `1` | video mode 列表中的索引 |
| `dev_serial_number` | `""` | 以序號子字串綁定 |
| `usb_port` | `""` | 以 USB 拓樸路徑綁定，例如 `2-3:1.0` |
| `depth_minimum_mm` | `-1` | 同時套用到 `/depth_image` 與 `/pointcloud` 的距離下限，低於此值的像素歸零。`-1` = 該型號預設（G100+ 250、R77 200、G62 100）|
| `depth_maximum_mm` | `-1` | 同時套用到 `/depth_image` 與 `/pointcloud` 的距離上限，高於此值的像素歸零。`-1` = 該型號預設（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 從最近一張左目彩色幀取色，輸出 XYZRGB PointCloud2；depth-only mode 自動回退為 XYZ |
| `spatial_filter` | `false` | 啟用視差域邊緣感知 IIR 空間濾波器（布林開關）|
| `temporal_filter` | `false` | 啟用時間濾波器，alpha 混合 + persistence（布林開關，執行中可調）|
| `hole_filling` | `0` | Z 域補洞模式。`0` = 關閉、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（為整數模式，非布林）|
| `filter_profile` | `default` | 濾波器調校設定檔名稱；解析至 `cfg/filter_profiles/<name>.yaml`，啟動時載入濾波器調校值 |
| `ir_intensity` | `-1` | `-1` = 該型號預設值（G100+ / R77 = 3、G62 = 60）；`0` = 關閉；正整數覆寫至 FW 範圍（G100+ 0-9、R77 0-6、G62 0-96）|
| `log` | `sdk`（per-model）/ `all`（generic） | 終端輸出層級。`all` = RCLCPP + SDK 完整輸出；`sdk` = 抑制 RCLCPP INFO/DEBUG，保留 WARN/ERROR 與 SDK printf；`close` = 全部導向 per-process log 檔（終端靜默）|
| `rviz` | `true` | 是否自動開啟 RViz |

彩色串流是否切分為 `left_color` 與 `right_color` 由選定的 video mode 決定（`launch/video_modes/<MODEL>.yaml` 的 `split_lr` 旗標），並非由 launch 參數控制。

### 影像參數控制

所有 eYs3D 模組出廠預設 `enable_auto_exposure` 開啟、
`enable_auto_white_balance` 開啟、`power_line_frequency` 設為
60 Hz；驅動程式僅承襲韌體開機時的設定，**只有當操作者明確覆寫時
才寫回相機**，所以在 ROS 之外調整過的設定能跨重啟保留。IR 是例外，
因為相機開機後投射器預設為關閉，因此一定會套用。

執行中即可調整，毋需重啟：

```bash
ros2 param set /G100P_1/eys3d_camera ir_intensity         5
ros2 param set /G100P_1/eys3d_camera enable_auto_exposure      false   # 手動曝光
ros2 param set /G100P_1/eys3d_camera exposure_time_step        -8
ros2 param set /G100P_1/eys3d_camera enable_auto_white_balance false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

### 後處理濾波器

內建三組可選濾波器，預設全關，且彼此獨立運作 —— 任意組合皆可。

| 濾波器 | 切換方式 |
|---|---|
| `spatial_filter` | 僅 launch |
| `temporal_filter` | launch + 執行中 |
| `hole_filling` | 僅 launch（`0` = 關閉；`2` = 建議起始模式）|

調校值（`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`）放在 `cfg/filter_profiles/<name>.yaml`。要自訂
profile，複製 `default.yaml` 並修改欄位即可；缺漏欄位會回退到
節點編譯時的預設值。

```bash
# 以預設 profile 啟用 spatial + temporal
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true temporal_filter:=true

# 加上 farthest_from_around 補洞（建議起始模式）
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true hole_filling:=2

# 切換到自訂的調校 profile
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true filter_profile:=indoor
```

`temporal_filter` 的參數可在執行中重新調整：

```bash
ros2 param set /G100P_1/eys3d_camera temporal_filter             true
ros2 param set /G100P_1/eys3d_camera temporal_filter_alpha       0.4
ros2 param set /G100P_1/eys3d_camera temporal_filter_persistence 3
```

### 執行時串流控制

兩個 service 可以暫停或停止相機輸出，而不需要關掉 ROS node。
兩者都會同時控制 color 與 depth。

```bash
# 停止發布 frame,但相機仍在 USB 上 stream。Driver CPU 降到接近 0,
# resume 後下一幀馬上出現。適合短暫的中斷情境。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# 完全釋放 USB pipe。ROS node 仍然活著，topic 仍然註冊。
# resume 約 300 ms 重新打開相機。適合需要把 USB 頻寬讓給其他
# 裝置的情境。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"
```

Standby 生效期間，自動重連 watchdog(下一節)會把無訊號視為刻意行為，
不會誤判為斷線。目前的控制狀態會在 `/diagnostics` 的 `stream_state`
欄位露出(`Active`、`Paused`、`Standby`)。

### 熱插拔自動復原

驅動內建 1 Hz watchdog。當 color **與** depth 連續 **3 秒**沒有任何
幀進來（啟動時放寬到 10 秒），watchdog 會關閉裝置進入重連迴圈，
每 2 秒嘗試一次。相機重新接回後，原本的 topic 會自動恢復發布 —
**不需要重跑 `ros2 launch`**。

```
[ERROR] watchdog: no frames for 3 s — declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

慢速模式（例如 R77 7 fps）由 10 秒啟動寬限期吸收；一旦觀察到任一幀，
就改套用 3 秒的穩態判定門檻。

### 多相機

連接多台相機時，驅動程式會依 launch 的 `model` 挑選 PID 符合的裝置
（G100+ = `0x0181`、R77 = `0x0180`、G62 = `0x0183`）。若指定的 `model`
找不到對應 PID 的相機，或 `usb_port` / `dev_serial_number` 解析到不同
機型，驅動會直接拒絕開啟並回報 PID 不符。

兩台同型號要區分時，launch **必須**以序號或 USB 拓樸路徑明確綁定 ——
自動 model-PID 比對無法分辨相同型號的兩台相機。兩種綁定方式都能
跨重啟與插拔順序保持穩定:

```bash
ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=front dev_serial_number:=8036259M200025

ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=rear  usb_port:=2-3:1.0
```

`launch/examples/` 已內建三個多相機範例，每個都已預留 `usb_port` 欄位，
請依實際接線改寫後再啟動:

```bash
ros2 launch eys3d_camera examples/dual_G62.launch.py
ros2 launch eys3d_camera examples/dual_G100P.launch.py
ros2 launch eys3d_camera examples/G100P_plus_R77.launch.py
```

內建的多相機範例已預先選好較輕的 `mode_id`,確保兩台相機能在 USB
頻寬限制內穩定一起啟動。若手動把 `mode_id` 調高、結果有相機起不來,
請退回較輕的 mode,或把兩台相機接到不同的 USB port 上。

若提示都對不上候選相機，節點會在 log 列出所有裝置的
`(PID, serial_number, /dev/videoN, usb_port)` 後結束 —— 從中複製正確
的值到 launch 即可。

---

## Frame ID

驅動程式啟動時一次性廣播靜態 TF 樹，根節點為 `<camera_name>_link`
（ROS 基底軸：X 前、Y 左、Z 上）。每條串流都有對應的感測器 frame 與
REP-103 `_optical_frame`。PointCloud2 使用 `<camera_name>_points_frame`，
已轉至 ROS 基底軸，無須再旋轉。

整合到機器人 URDF 時，將 `<camera_name>_link` 掛在既有 frame 之下即可。
最小範例（請依平台調整 `parent_link`、joint 名稱與掛載姿態）:

```xml
<joint name="g100p_mount" type="fixed">
  <parent link="parent_link"/>
  <child  link="G100P_1_link"/>
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</joint>
<link name="G100P_1_link"/>
```

驅動程式以 `TRANSIENT_LOCAL` durability 將相機 TF 樹一次性發布到
`/tf_static`,任何後加入的訂閱者都會立即收到快取的 transform。

---

## Composable Node

驅動程式以 `rclcpp` component 形式註冊，可載入至
`ComposableNodeContainer` 與下游 component 同處一個 process 並交換
訊息。設定 `use_intra_process_comms:=true` 後，影像、深度與點雲訊息
以指標方式交給同容器訂閱者，而非經 DDS 傳遞。

```bash
ros2 launch eys3d_camera examples/G100P_composable.launch.py \
    use_intra_process_comms:=true
```

`launch/examples/G100P_composable.launch.py` 是起始點；於
`composable_node_descriptions` 中追加自己的 `ComposableNode`，並在每
個項目加上 `extra_arguments=[{'use_intra_process_comms': True}]`
即可共用容器。此啟動檔支援 `params_file` 參數，可在單一 YAML 中
帶入所有驅動程式設定。

`/tf_static` 一律以 `TRANSIENT_LOCAL` durability 發布，無論節點層級
intra-process 設定為何皆可正常運作。

---

## 診斷訊息 `/diagnostics`

每秒一筆 `DiagnosticArray`。每一筆內含 5 個 `DiagnosticStatus`（每
task 一個),名稱為 `"<hardware_id>: <task>"`(`hardware_id` 為相機序號,
模組未回報時則為 `eys3d_camera`)。整體健康狀態由 `device` task 的
`level` + `message` 概括:

| `level` | `message` | 意義 |
|---|---|---|
| `OK` | `streaming OK` | 每條啟用的串流都達到預期 fps 的 50 % 以上 |
| `WARN` | `one stream below 50% of expected fps` | 某條串流速度過低 |
| `ERROR` | `no frames flowing on enabled streams` | 啟用的串流都低於門檻（或皆無資料） |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 斷線；watchdog 會在裝置回插時自動重連 |

各 task 的 `values` 鍵值對：

**`device`** — 連線與識別：

| Key | 說明 |
|---|---|
| `connection_state` | `streaming` 或 `disconnected` |
| `device_present` | `true` / `false`，對應 V4L2 節點是否存在 |
| `reconnect_attempts` | 自啟動以來累積重連次數 |
| `usb_port` | open 時解析到的 sysfs 介面路徑（如 `2-3:1.0`）|
| `serial_number` | SDK 回報的模組序號 |
| `actual_fps` | `APC_OpenDevice2` 回傳的 fps（interleave mode 為每串流半值）|
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` service 控制的執行時狀態 |

**`color`** 與 **`depth`** — 每條串流吞吐：

| Key | 說明 |
|---|---|
| `input_fps` | 過去 1 秒從 SDK 收到的幀數。與訂閱者狀態無關 — 用於判斷相機 / USB 健康度 |
| `publish_fps` | 過去 1 秒實際發布到 topic 的幀數。沒訂閱者時為 0；有訂閱者且 driver 跟得上時 ≈ `input_fps`；持續低於代表 driver 落後 |
| `input_total` | 自 open 起累積從 SDK 收到的幀數 |
| `publish_total` | 自 open 起累積發布的幀數 |
| `input_dropped` | 累積 SDK 端掉幀數（由 serial-number 不連續偵測）|
| `decode_avg_ms` | （僅 `color`）過去 1 秒 color 解碼平均耗時。當下 1 秒內有解碼過 frame 才會出現 |
| `decode_max_ms` | （僅 `color`）至今觀察到的最長 color 解碼耗時。曾經解碼過 frame 才會出現 |

**`pointcloud`** — 點雲投影 + 後處理計數：

| Key | 說明 |
|---|---|
| `compute_status` | `active`（有訂閱者拉點雲）、`idle (no /pointcloud subscriber)`、`idle (never run ; no subscriber since start)` 或 `(disconnected — see device task)` |
| `publish_fps` | 過去 1 秒點雲發布幀數。沒訂閱者時為 0 |
| `compute_avg_ms` | 過去 1 秒點雲計算平均耗時（`active` 時出現）|
| `compute_max_ms` | 至今觀察到的最長點雲計算耗時 |
| `publish_total` | 累積發布的點雲數 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後處理階段的累積套用次數 |

**`thermal`**：

| Key | 說明 |
|---|---|
| `temperature_c` | 晶片溫度（°C）；不支援的機型顯示 `n/a (not supported on this model)` |

`/diagnostics` 只在有訂閱者連線時才會組裝並發布訊息；hot-path 的
atomic counter 持續累計，所以隨時掛上監控都能看到自 open 起的累積值。

判讀方式：

| `input_fps` | `publish_fps` | 意義 |
|---|---|---|
| ≈ 預期 | 0 | 沒有訂閱者，driver idle |
| ≈ 預期 | ≈ input | driver 跟得上 |
| ≈ 預期 | < input | 有訂閱者，但發布路徑落後（解碼太慢 / DDS congestion / 訂閱端 QoS 不對）|
| 0 | 0 | 相機 / USB 沒在送資料 — 看 `device.connection_state` |

訂閱 `/diagnostics`（或執行 `rqt_robot_monitor`）即可即時查看。

內附監控工具：自動訂閱所有 stream，並把 SDK / Pub / Rx 三條 rate
跟解碼 / 計算耗時並列顯示：

```bash
ros2 run eys3d_camera perf_monitor            # 自動偵測 namespace
ros2 run eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

---

## 疑難排解

| 問題 | 解法 |
|---|---|
| 自寫的訂閱者收不到影像或點雲訊息 | driver 採用 `SensorDataQoS`（BestEffort + KeepLast 5 + Volatile）發布。訂閱端必須宣告相容的 QoS — C++ 用 `rclcpp::SensorDataQoS()`、Python 用 `qos_profile_sensor_data`。詳見[訂閱端 QoS](#訂閱端-qos) 段落 |
| RViz Image 面板空白 | 將 Fixed Frame 設為 `<camera_name>_link`，並確認 topic 名稱對應到目前的 `camera_name` namespace |
| 相機開啟失敗（device busy） | 其他程序佔用 `/dev/videoN`；用 `lsof /dev/video*` 找出來源，若是殘留的 driver 用 `pkill -9 -f camera_node` 結束 |
| `No device matches binding hints` | log 會列出所有偵測到的 eYs3D 模組，將正確的 `dev_serial_number` 或 `usb_port` 填到 launch 參數 |
| `cv_bridge` 回報 encoding 不符 | 所有 color topic 皆以 `rgb8` 發布（不論底層是 MJPEG 或 YUYV），訂閱端必須宣告 `rgb8`，不要用 `bgr8` 或 `mono8` |

### 訂閱端收到的 fps 比 driver 公布的少

`perf_monitor` 或 `ros2 topic hz` 顯示 Rx < Pub，但 `/diagnostics`
顯示 `dropped = 0`，這是 DDS 傳輸層在掉，不是 driver。

各個支援的 video mode 之單張影像都比 kernel UDP socket 與 IP
fragment reassembly 在一次 burst 內所能吸收的量大(`rgb8` 1280×720
≈ 2.7 MB；典型 point cloud 最多 ~11 MB)。同一個底層原因(kernel 端
buffer 撐爆)會在兩種情境中以不同表象出現：

- **效能較高的 x86 主機**:driver 把每張 frame 在微秒級的 UDP
  fragment burst 內塞給 DDS，接收端 socket / reassembly queue 還沒
  消化就溢位。
- **運算資源受限的 ARM 主機**:receiver 端讀同一個 kernel buffer
  的速度太慢，buffer 提早滿，結果一樣。

package 附了一份 opt-in 腳本，把 FastRTPS 切到 32 MB 的 shared-
memory segment，可消除任何**同機**訂閱者的 UDP fragmentation 問題。
**跨機**訂閱者不適用 — 跨機需依
[ROS 2 DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
調高 `net.core.rmem_max`。預設不自動啟用，因為 SHM segment 會在
`/dev/shm` 內為每個 ROS 2 participant 預留約 32 MB。

啟用方式：把下面這行加到 `~/.bashrc`，位置放在你的
`source <workspace>/install/setup.bash` **之後**，並將 `~/ros2_ws`
換成你實際的 workspace 路徑：

```bash
source ~/ros2_ws/install/eys3d_camera/share/eys3d_camera/config/enable_fastrtps_shm.bash
```

