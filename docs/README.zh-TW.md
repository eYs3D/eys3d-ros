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
sudo apt install ros-$ROS_DISTRO-diagnostic-updater \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz2
```

per-model launch 預設 `urdf:=true` 與 `rviz:=true`，缺少
`robot_state_publisher`、`xacro`、`rviz2` 會無法啟動；加上
`urdf:=false rviz:=false` 則不需要。

請將 `eys3d_camera` 與 `eys3d_camera_interfaces`（自我校正 action 的定義）
兩個 package 一併放入 workspace 的 `src/` 後建置；`--packages-up-to` 會先
建置 interfaces：

```bash
cd ~/ros2_ws
colcon build --packages-up-to eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


### 裝置權限

驅動以一般使用者身分開啟相機。若開啟裝置時出現權限錯誤，安裝隨附的
udev 規則，讓 eSPDI SDK 能存取該 USB 裝置：

```bash
sudo cp install/eys3d_camera/share/eys3d_camera/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

然後重新插拔相機。此規則授予對 eYs3D 裝置（USB vendor `3438`）的
存取權限；或將使用者加入 `video` 群組後重新登入。

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
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 與 MJPEG 來源皆 inline 解碼為 RGB888；灰階感測模組以 R=G=B 輸出）。是否經過校正取決於當前的 video mode。|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右眼彩色影像；僅在 video mode 帶有 L\|R 並排輸出時發布（`split_lr: true`）|
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`，mm，REP-118) | 深度（毫米） |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32，已轉換為 ROS 基底軸（公尺）；當 `colored_pointcloud:=true` 時改為 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相機內參，每張 Image 一張（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康狀態，1 Hz |

### CameraInfo 與畸變係數

每一份 `camera_info` 描述的都是它自己 topic 上發布的那張影像。

深度在所有模式下都是已校正的;彩色則在目錄名稱帶撇號的模式下已校正
(`L'+D`、`L'+R'+D`)。那些 topic 上的 `k` 是 `p` 的左 3×3、`d` 為零
(`plumb_bob`)、`r` 為單位矩陣 —— 相機已經移除畸變,沒有東西需要還原。

`L+R` 與 `L+R+D` 模式在彩色 topic 上發布的是原始感測器影像。那裡的
`k` 與 `d` 是發布解析度下的原廠鏡頭模型、`r` 是校正旋轉,`image_proc`
可以正常校正它們。請讀 `distortion_model` 而不要假設係數個數:driver 會回報
`rational_polynomial` 八個係數或 `plumb_bob` 五個,依該台模組內
儲存的校正資料而定。

兩種情況下 `p` 都是投影矩陣,也是姿態估計(AprilTag、PnP、SLAM)應該
取用的內參來源。立體對的右相機 `p[3]` 是 `-fx × 基線`,單位為公尺。

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

- **偶數 frame —— IR 關閉 → `/left_color/image_raw`**。
  沒有 IR 紅外點干擾，輸出乾淨的 RGB 影像。
- **奇數 frame —— IR 開啟 → `/depth/image_raw`**。
  IR 紅外點為立體匹配提供結構特徵，得到較高品質的深度。

驅動在 SDK 串流源頭做奇偶 frame 過濾，因此每條串流的 FPS 會減半。

#### 時戳影響

以 G100+ 在 USB 3.0 下的預設 `mode_id=1` 為例：

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color/image_raw  30 fps
                                 └  /G100P_1/depth/image_raw       30 fps
```

color 與 depth **來自相鄰兩張 sensor frame**，stamp 相差
**1 個 sensor frame ≈ 15.6 ms（1000 / 60）**。
同 stream 內 `Image` 與其 `CameraInfo` stamp 一致。

下游用 `message_filters::ApproximateTime` 配對，`slop` 設一個
sensor frame 週期（例如 20 ms）即可穩定配對。

#### 影響的 mode IDs

於 `launch/video_modes/G100P.yaml` 中，**`mode_id` 1、3、5、7–21、56、57** 為 interleave。
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
| `mode_id` | `-1` | video mode 列表中的索引;`-1` = 自動(依協商到的 USB 速度取該 signature 的預設) |
| `dev_serial_number` | `""` | 以序號子字串綁定 |
| `usb_port` | `""` | 以 USB 拓樸路徑綁定，例如 `2-3:1.0` |
| `depth_near_mm` | `-1` | 同時套用到 `/depth/image_raw` 與 `/depth/points` 的距離下限，低於此值的像素歸零。`-1` = 該型號預設（G100+ 250、R77 200、G62 100）|
| `depth_far_mm` | `-1` | 同時套用到 `/depth/image_raw` 與 `/depth/points` 的距離上限，高於此值的像素歸零。`-1` = 該型號預設（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 從最近一張左目彩色幀取色，輸出 XYZRGB PointCloud2；depth-only mode 自動回退為 XYZ |
| `spatial_filter` | `false` | 啟用視差域邊緣感知 IIR 空間濾波器（布林開關）|
| `temporal_filter` | `false` | 啟用時間濾波器，alpha 混合 + persistence（布林開關，執行中可調）|
| `hole_filling` | `0` | Z 域補洞模式。`0` = 關閉、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（為整數模式，非布林）|
| `filter_profile` | `default` | 濾波器調校設定檔名稱；解析至 `cfg/filter_profiles/<name>.yaml`，啟動時載入濾波器調校值 |
| `ir_value` | `-1` | `-1` = 依模式決定：mode 含深度或機型為黑白（G62 / R77）時採型號預設值（G100+ / R77 = 3、G62 = 60），彩色感光元件跑純彩色 mode 時關閉；`0` = 關閉；正整數覆寫至 FW 範圍（G100+ 0-6、R77 0-6、G62 0-96）|
| `log` | `sdk`（per-model）/ `all`（generic） | 終端輸出層級。`all` = RCLCPP + SDK 完整輸出；`sdk` = 抑制 RCLCPP INFO/DEBUG，保留 WARN/ERROR 與 SDK printf；`close` = 全部導向 per-process log 檔（終端靜默）|
| `urdf` | `true` | 透過 namespace 隔離的 `robot_state_publisher` 在 `<camera_name>/robot_description` 發布相機模型（mesh + 鎖固孔 frame）。若整機 bringup 已含相機描述，設為 `false` |
| `rviz` | `true` | 是否自動開啟 RViz |
| `selfcal_enable` | `false` | 啟用選用的自我校正;`true` 會啟用 `selfcal/run` action 與 `selfcal/commit` service（詳見自我校正段落）。僅 launch 時可設 |

彩色串流是否切分為 `left_color` 與 `right_color` 由選定的 video mode 決定（`launch/video_modes/<MODEL>.yaml` 的 `split_lr` 旗標），並非由 launch 參數控制。

### 影像參數控制

所有 eYs3D 模組出廠預設 `auto_exposure` 開啟、
`auto_white_balance` 開啟、`power_line_frequency` 設為
60 Hz；驅動程式僅承襲韌體開機時的設定，**只有當操作者明確覆寫時
才寫回相機**，所以在 ROS 之外調整過的設定能跨重啟保留。IR 是例外，
因為相機開機後投射器預設為關閉，因此一定會套用。

執行中即可調整，毋需重啟：

```bash
ros2 param set /G100P_1/eys3d_camera ir_value             5
ros2 param set /G100P_1/eys3d_camera auto_exposure        false   # 手動曝光
ros2 param set /G100P_1/eys3d_camera exposure_time_step   -8
ros2 param set /G100P_1/eys3d_camera auto_white_balance   false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

`exposure_time_step` 接受 **[-13, 3]** 範圍內的帶號整數（log2 曝光暫存器），且僅在 `auto_exposure` 為 `false` 時生效。

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

三個 service 可以控制相機輸出,而不需要關掉 ROS node。`pause` 與
`standby` 接收 `std_srvs/srv/SetBool`,同時控制 color 與 depth;
`hw_reset` 接收 `std_srvs/srv/Empty`,透過 USB 對相機做硬體重置。

```bash
# 停止發布 frame,但相機仍在 USB 上 stream。Driver CPU 降到接近 0,
# resume 後下一幀馬上出現。適合短暫的中斷情境。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# 完全釋放 USB pipe。ROS node 仍然活著，topic 仍然註冊。
# resume 會重新打開相機，依機種需要數秒。適合需要把 USB 頻寬讓給其他
# 裝置的情境。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"

# 透過 USB 重置相機(重新列舉裝置)。node 會停止串流、發出重置,
# 之後 watchdog 自動重連,frame 通常在約 12 秒後恢復。用於在不重啟
# node 的情況下復原卡死的相機。
ros2 service call /G100P_1/hw_reset std_srvs/srv/Empty
```

Standby 生效期間，自動重連 watchdog(下一節)會把無訊號視為刻意行為，
不會誤判為斷線。目前的控制狀態會在 `/diagnostics` 的 `stream_state`
欄位露出(`Active`、`Paused`、`Standby`)。

### 熱插拔自動復原

驅動內建 1 Hz watchdog，每個串流各自監看：某個串流送出過影格之後，
若連續 **3 秒**沒有新影格，就關閉裝置進入重連迴圈，每 2 秒嘗試一次 ——
因此 depth 在韌體裡卡住、color 仍持續送的情況也能復原。第一個影格
進來之前門檻放寬為 10 秒，足以涵蓋 R77 7 fps 這類慢速模式。相機重新
接回後，原本的 topic 會自動恢復發布，**不需要重跑 `ros2 launch`**。

```
[ERROR] watchdog: depth stream silent for 3 s; declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```
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
ros2 launch eys3d_camera dual_G62.launch.py
ros2 launch eys3d_camera dual_G100P.launch.py
ros2 launch eys3d_camera G100P_plus_R77.launch.py
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

每個型號皆隨附含 3D mesh 的 URDF/xacro 描述檔
（`urdf/eys3d_<MODEL>.urdf.xacro` + `meshes/<MODEL>.dae`）。`<name>_link`
位於深度起始點 —— 兩顆感光元件的橫向中點、光軸高度上，距相機前殼面
內縮 Z'（G100+ 6.75 mm、R77 4.8 mm、G62 3.1 mm）。在機器人既有 frame
之下以實際掛載姿態實例化 macro：

```xml
<xacro:include filename="$(find eys3d_camera)/urdf/eys3d_G100P.urdf.xacro"/>
<xacro:eys3d_G100P name="G100P_1" parent="parent_link">
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</xacro:eys3d_G100P>
```

單獨預覽（`name` 預設為 `<MODEL>_1`，與驅動程式及隨附 rviz 佈局一致）：

```bash
ros2 launch eys3d_camera display_model.launch.py model:=G100P
```

並排預覽三款模型（不需硬體）:

```bash
ros2 launch eys3d_camera three_models.launch.py
```

每份描述檔同時帶有機殼鎖固孔 frame（`<name>_tripod_frame` 與
`<name>_back/bottom_screw*_frame`，位置取自原廠 CAD），供機構
整合時核對虛擬與實體的一致性。

驅動程式以 `TRANSIENT_LOCAL` durability 將相機 TF 樹一次性發布到
`/tf_static`,任何後加入的訂閱者都會立即收到快取的 transform。

---

## Composable Node

驅動程式以 `rclcpp` component 形式註冊，可載入至
`ComposableNodeContainer` 與下游 component 同處一個 process 並交換
訊息。設定 `use_intra_process_comms:=true` 後，影像、深度與點雲訊息
以指標方式交給同容器訂閱者，而非經 DDS 傳遞。

```bash
ros2 launch eys3d_camera G100P_composable.launch.py \
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
| `OK` | `streaming` | 每條設定中的串流都有在送 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` 生效中 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` 生效中 |
| `ERROR` | `no frames flowing on any configured stream` | 每條設定中的串流都低於預期速率的一半 |
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

兩者的摘要為 `streaming`、`input rate below 50% of expected`（WARN）、
`not configured (D-only mode)` 或 `standby`。

**`pointcloud`** — 點雲投影 + 後處理計數：

| Key | 說明 |
|---|---|
| `compute_status` | `active`（有訂閱者拉點雲）、`idle (no /depth/points subscriber)`、`idle (never run ; no subscriber since start)` 或 `(disconnected — see device task)` |
| `publish_fps` | 過去 1 秒點雲發布幀數。沒訂閱者時為 0 |
| `compute_avg_ms` | 過去 1 秒點雲計算平均耗時（`active` 時出現）|
| `compute_max_ms` | 至今觀察到的最長點雲計算耗時 |
| `publish_total` | 累積發布的點雲數 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後處理階段的累積套用次數 |

**`thermal`**：

| Key | 說明 |
|---|---|
| `temperature_c` | 晶片溫度（°C）。僅在機型具備感測器且讀取成功時發布；否則不會出現此欄位，原因寫在該任務的 summary |

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

## 自我校正

選用的串流中自我校正會重新對齊立體視覺配對,在校正已漂移的模組上恢復深度
填充率。預設即編入(CMake `EYS3D_WITH_SELFCAL`),並於 launch 時以
`selfcal_enable:=true` 啟用。

```bash
ros2 launch eys3d_camera G100P.launch.py selfcal_enable:=true
```

執行時相機需**正在串流深度模式,並對準工作距離下一般、有紋理的場景** ——
校正器以深度覆蓋率為量測依據,因此需要有效深度。一個 `selfcal/run` action
即執行完整一次 session,並自動套用內建的調校參數。

```bash
# 執行一次 session(阻塞直到收斂,約 20-30 秒,接著一段短暫複檢;--feedback
# 會串流 phase / progress)。auto_commit_shift_px < 0 會讓結果生效但永不寫 flash;
# >= 0 則在此次「驗證為更好」且 cy 位移達到這麼多像素時自動燒錄(詳見下表)。
ros2 action send_goal /G100P_1/selfcal/run \
  eys3d_camera_interfaces/action/SelfCal \
  "{auto_commit_shift_px: 0.25}" --feedback

# 將保留的結果寫入 flash(僅在未自動燒錄時需要)。
# 檢查回應的 `success` 欄位 —— 若沒有可保留的結果會是 false。
ros2 service call /G100P_1/selfcal/commit std_srvs/srv/Trigger
```

搜尋結束後,此次會依校正器的 outcome **加上一次即時 A/B 複檢**自行收尾 ——
複檢在同一場景量測「新對齊 vs 跑之前對齊」的深度填充率,因此判定反映真實的
前後差異,而非場景相依的臆測:

- **驗證為更好** —— 複檢確認的 `SUCCESS`,已生效(`applied: true`)。若有設
  `auto_commit_shift_px` 且 `cy_shift_px` 達標,此次即**燒錄**進 flash
  (`committed: true`);否則**保留生效**但為暫存 —— 呼叫 `selfcal/commit`
  才能在斷電後留存。
- **已是最佳**(`NO_CHANGE`)—— 不改動、不保留、不寫入。這是正常且健康的結果。
- **更差 / 無法驗證 / 失敗** —— 複檢判定更差或無法確認(例如跑的過程中相機
  移動了),或 `INSUFFICIENT_INPUT` / `TIMEOUT` / `FAILED` —— 相機**還原**回
  跑之前的對齊(`reverted: true`)。

`cy_shift_px` 是實測的垂直位移,直接讀自硬體,也是自動燒錄門檻採用的值。步階
固定在 **0.25 px**,位移上限為 **5.0 px**,所以它永遠是 `0.25、0.50、… 5.00`
其中之一(或 `0`)。`auto_commit_shift_px` 是門檻 —— 可帶任何值,但只有步階
邊界會改變行為,介於步階之間的值會向上取整(`0.3` 等同 `0.5`):

| `auto_commit_shift_px` | 效果 |
| --- | --- |
| `-1`(預設)| 永不自動燒 —— 檢視後手動燒錄。未帶此參數時採用的預設值 |
| `0.25` | 任何真實移動(≥ 1 步)就燒 —— **建議值** |
| `0.50`、`0.75`、… 直到 `5.00` | 要求更大的位移才燒 |
| `> 5.0` | 永不觸發(位移不可能超過上限)|

### Action 回饋

此次執行進行中,若 goal 以 `--feedback` 送出,便會持續串流:

| 欄位 | 意義 |
| --- | --- |
| `phase` | `INITIAL_SEARCH` / `REFINEMENT` / `RECHECK` / `COMPLETED` |
| `progress` | 搜尋進度,`0.0`–`1.0` |
| `processed_frames` | 至今餵給校正器的深度幀數 |
| `valid_ratio_latest` | 最新一幀的填充率(有結果後才有值)|

### Action 結果

無法啟動、無法得出結論、或無法回退已否決的變更時，goal 會 abort；刻意的
revert 則是 succeed。兩種情況的細節都在 `outcome`。

此次執行收尾時回傳一次 —— 記錄這次 session 量到了什麼、又在相機上留下了什麼:

| 欄位 | 意義 |
| --- | --- |
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`(場景中有效深度不足)/ `TIMEOUT`(未在時限內收斂)/ `FAILED` |
| `cy_shift_px` | 實測垂直 cy 位移(px)—— 自動燒錄門檻依據 |
| `recheck_verdict` | A/B 複檢:`improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `recheck_ratio_after` | 複檢時「跑前 / 收斂後」對齊的填充率 |
| `applied` | 修正已生效於暫存器 |
| `reverted` | 已還原回跑前校正 |
| `committed` | 已寫入使用者校正區 |
| `message` | 結果的人類可讀摘要 |
| `correction_level`、`valid_ratio_first` / `_latest` / `_delta` | 僅供診斷(永不作為燒錄門檻)|

整個 session 全程串流:深度持續發布、任何後處理濾鏡持續運作 —— 此次執行從不
重啟串流。搜尋期間深度品質會隨校正器探測各對齊而明顯波動,收斂後便穩定下來。
session **無法中斷**(cancel 一律拒絕,控制設定項與 `pause` / `standby` /
`hw_reset` 也會拒絕)—— 它很短,且結果更差時會自動還原,所以沒有需要中斷的
情形。

`commit` 是唯一寫入 flash 的步驟;原廠校正保留作為備份,永不被覆寫。**保留但
未燒錄**的結果只存在於相機暫存器,斷電重開或呼叫 `~/hw_reset` 就會清回已儲存
的校正。這讓「在串流上校正、永不寫 flash」是預設就安全的流程 —— 每當機器人
停靠且靜止時執行即可。

有兩點值得注意:

- **連續執行會疊加。** 在 commit 或斷電重開之前再跑一次 `selfcal/run`,是接
  著上一次保留的結果繼續、而不是從 flash 重來。想要乾淨的基準,請先 `commit`
  或斷電重開。
- **不符資格 / 已有 session 在跑 → 是拒絕,不是失敗。** 不支援自我校正的模組,
  以及已有 session 在跑時再送出的第二個 `selfcal/run`,都會直接被拒絕 ——
  `ros2 action send_goal` 回報「Goal was rejected」(沒有 `Result`),原因要看
  node log。

**每個 process 同時只能有一次 session。** 校正器將相機 handle 綁在 process 全域,
因此共用同一 process 的相機 —— 例如同一個 `ComposableNodeContainer` 內的多個
driver —— 只能一次校正一台;在第一次結束之前,第二個 `selfcal/run` 會被拒絕並在
node log 留下訊息。以獨立 process 啟動的相機則不受影響。除此之外,intra-process
通訊與此無關:校正器會自行複製一份深度 frame,因此執行期間
`use_intra_process_comms` 的傳遞方式不變。

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
顯示 `input_dropped = 0`，這是 DDS 傳輸層在掉，不是 driver。

各個支援的 video mode 之單張影像都比 kernel UDP socket 與 IP
fragment reassembly 在一次 burst 內所能吸收的量大(`rgb8` 1280×720
≈ 2.7 MB；典型 point cloud 最多 ~11 MB)，因此 kernel 端 buffer 會
溢位 —— 不論是 x86 主機送得太快、還是 ARM 主機讀得太慢。

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

