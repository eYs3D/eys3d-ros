# eYs3D 立体深度相机 ROS 2 驱动程序

[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy%20%7C%20Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/rolling/Releases.html)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` 是 eYs3D 立体深度相机的官方 ROS 2 驱动程序，
发布彩色影像、深度影像与点云，遵循 REP-103 frame tree。支持
ROS 2 Foxy、Humble、Jazzy。

### 支持的相机

| 型号 | Product code | USB | 状态 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量产 |
| **R77** | YX8072 | USB 2.0 | 量产 |
| **G62** | YX8081 | USB 2.0 | 量产 |

---

## 安装

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
```

将此 package 放入 workspace 的 `src/` 后构建:

```bash
cd ~/ros2_ws
colcon build --packages-select eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

per-model launch 默认会打开 RViz；安装一次
`sudo apt install ros-$ROS_DISTRO-rviz2`，或加 `rviz:=false` 跳过可视化。

---

## 快速开始

每个型号都有对应的 launch 快捷方式，自动应用该机型的默认 video mode:

```bash
ros2 launch eys3d_camera G100P.launch.py     # G100+:L'+D 1280x720 interleave，SDK 每串流 30 fps
ros2 launch eys3d_camera R77.launch.py       # R77: L'+D 1280x920 color + 640x460 depth @ 30 fps
ros2 launch eys3d_camera G62.launch.py       # G62: L'+D 640x480 @ 25 fps
```

要切换 mode 用 `mode_id:=<n>`；各机型完整 mode 列表位于
`launch/video_modes/<MODEL>.yaml`。

### 发布的 Topic

以 `camera_name=G100P_1` 为例：

| Topic | 类型 | 说明 |
|---|---|---|
| `/G100P_1/left_color` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 与 MJPEG 来源皆 inline 解码为 RGB888；灰阶传感模块以 R=G=B 输出）。是否经过校正取决于当前的 video mode。|
| `/G100P_1/right_color` | `sensor_msgs/Image` | 右眼彩色影像；仅在 video mode 带有 L\|R 并排输出时发布（`split_lr: true`）|
| `/G100P_1/depth_image` | `sensor_msgs/Image` (`16UC1`,mm,REP-118) | 深度（毫米） |
| `/G100P_1/pointcloud` | `sensor_msgs/PointCloud2` | XYZ float32，已转换为 ROS 基底轴（米）；当 `colored_pointcloud:=true` 时改为 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，每张 Image 一张（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康状态，1 Hz |

### 订阅端 QoS

影像、深度与 PointCloud2 topic 采用 `SensorDataQoS`
(BestEffort + KeepLast 5 + Volatile)。订阅端必须声明相同 QoS:

```cpp
sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(), callback);
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, topic, callback, qos_profile=qos_profile_sensor_data)
```

新版 ros2cli 的 `ros2 topic hz` / `echo` 会自动 negotiate QoS，因此
**不需要额外 flag**。

### Interleave 模式（仅 G100+）

双目相机没有独立的第三目用于 RGB 输出。
为了同时取得「不含 IR 红外点的 color」与「高品质的 depth」，
eYs3D 提出 Interleave 模式来达到此需求：

- **偶数 frame —— IR 关闭 → `/left_color`**。
  没有 IR 红外点干扰，输出干净的 RGB 图像。
- **奇数 frame —— IR 开启 → `/depth_image`**。
  IR 红外点为立体匹配提供结构特征，得到较高品质的深度。

驱动在 SDK 串流源头做奇偶 frame 过滤，因此每条串流的 FPS 会减半。

#### 时戳影响

以 G100+ 默认 `mode_id=1` 为例：

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color   30 fps
                                 └  /G100P_1/depth_image  30 fps
```

color 与 depth **来自相邻两张 sensor frame**，stamp 相差
**1 个 sensor frame ≈ 15.6 ms（1000 / 60）**。
同 stream 内 `Image` 与其 `CameraInfo` stamp 一致。

下游用 `message_filters::ApproximateTime` 配对，`slop` 设一个
sensor frame 周期（例如 20 ms）即可稳定配对。

#### 影响的 mode IDs

于 `launch/video_modes/G100P.yaml` 中，**`mode_id` 1、3、5、7–21** 为 interleave。
其他 mode 为 color 与 depth 同时采集，stamp 一致。

---

## 配置

### 切换 video mode

```bash
ros2 launch eys3d_camera G100P.launch.py mode_id:=7    # G100+:L'+D 640x480 interleave（SDK 30 fps）
ros2 launch eys3d_camera R77.launch.py   mode_id:=4    # R77:D-only 640x460 @ 30 fps
ros2 launch eys3d_camera G62.launch.py   mode_id:=3    # G62:L'+D 320x240 @ 30 fps
```

每台相机完整的 video mode 列表位于 `launch/video_modes/<MODEL>.yaml`。
节点启动时会在 RCLCPP INFO log 中打印完整列表；per-model launch 默认
`log:=sdk` 会抑制这层 log，需要查看请改用 `log:=all`(或直接打开 YAML)。

### Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace 与 frame-id 前缀 |
| `mode_id` | `1` | video mode 列表中的索引 |
| `dev_serial_number` | `""` | 通过序列号子串绑定 |
| `usb_port` | `""` | 通过 USB 拓扑路径绑定，例如 `2-3:1.0` |
| `depth_minimum_mm` | `-1` | 同时套用到 `/depth_image` 与 `/pointcloud` 的距离下限，低于此值的像素归零。`-1` = 该型号默认（G100+ 250、R77 200、G62 100）|
| `depth_maximum_mm` | `-1` | 同时套用到 `/depth_image` 与 `/pointcloud` 的距离上限，高于此值的像素归零。`-1` = 该型号默认（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 从最近一张左目彩色帧取色，输出 XYZRGB PointCloud2；depth-only mode 自动回退为 XYZ |
| `spatial_filter` | `false` | 启用视差域边缘感知 IIR 空间滤波器（布尔开关）|
| `temporal_filter` | `false` | 启用时间滤波器，alpha 混合 + persistence（布尔开关，运行中可调）|
| `hole_filling` | `0` | Z 域补洞模式。`0` = 关闭、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（整数模式，非布尔）|
| `filter_profile` | `default` | 滤波器调校配置档名称；解析至 `cfg/filter_profiles/<name>.yaml`，启动时载入滤波器调校值 |
| `ir_intensity` | `-1` | `-1` = 该型号默认值（G100+ / R77 = 3、G62 = 60）；`0` = 关闭；正整数覆盖至 FW 范围（G100+ 0-9、R77 0-6、G62 0-96）|
| `log` | `sdk`（per-model）/ `all`（generic） | 终端输出层级。`all` = RCLCPP + SDK 完整输出；`sdk` = 抑制 RCLCPP INFO/DEBUG，保留 WARN/ERROR 与 SDK printf；`close` = 全部重定向至 per-process log 文件（终端静默）|
| `rviz` | `true` | 是否自动开启 RViz |

彩色串流是否切分为 `left_color` 与 `right_color` 由选定的 video mode 决定（`launch/video_modes/<MODEL>.yaml` 的 `split_lr` 旗标），并非由 launch 参数控制。

### 影像参数控制

所有 eYs3D 模组出厂预设 `enable_auto_exposure` 开启、
`enable_auto_white_balance` 开启、`power_line_frequency` 设为
60 Hz；驱动程序仅承袭固件开机时的设置，**只有当操作者明确覆盖时
才写回相机**，所以在 ROS 之外调整过的设置可跨重启保留。IR 是例外，
因为相机开机后投射器默认关闭，所以一定会应用。

运行中即可调整，无需重启：

```bash
ros2 param set /G100P_1/eys3d_camera ir_intensity         5
ros2 param set /G100P_1/eys3d_camera enable_auto_exposure      false   # 手动曝光
ros2 param set /G100P_1/eys3d_camera exposure_time_step        -8
ros2 param set /G100P_1/eys3d_camera enable_auto_white_balance false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

### 后处理滤波器

内置三组可选滤波器，默认全关，且彼此独立运作 —— 任意组合均可。

| 滤波器 | 切换方式 |
|---|---|
| `spatial_filter` | 仅 launch |
| `temporal_filter` | launch + 运行中 |
| `hole_filling` | 仅 launch(`0` = 关闭；`2` = 建议起始模式)|

调校值（`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`）放在 `cfg/filter_profiles/<name>.yaml`。要自订
profile，复制 `default.yaml` 并修改字段即可；缺漏字段会回退到
节点编译时的默认值。

```bash
# 以默认 profile 启用 spatial + temporal
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true temporal_filter:=true

# 加上 farthest_from_around 补洞（建议起始模式）
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true hole_filling:=2

# 切换到自订的调校 profile
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true filter_profile:=indoor
```

`temporal_filter` 的参数可在运行中重新调整：

```bash
ros2 param set /G100P_1/eys3d_camera temporal_filter             true
ros2 param set /G100P_1/eys3d_camera temporal_filter_alpha       0.4
ros2 param set /G100P_1/eys3d_camera temporal_filter_persistence 3
```

### 运行时串流控制

两个 service 可以暂停或停止相机输出，而不需要关掉 ROS node。
两者都会同时控制 color 与 depth。

```bash
# 停止发布 frame,但相机仍在 USB 上 stream。Driver CPU 降到接近 0,
# resume 后下一帧立即出现。适合短暂中断的情境。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# 完全释放 USB pipe。ROS node 仍然存活，topic 仍然注册。
# resume 约 300 ms 重新打开相机。适合需要把 USB 带宽让给其他
# 设备的情境。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"
```

Standby 生效期间，自动重连 watchdog(下一节)会把无信号视为刻意行为，
不会误判为断线。当前的控制状态会在 `/diagnostics` 的 `stream_state`
字段露出(`Active`、`Paused`、`Standby`)。

### 热插拔自动恢复

驱动内置 1 Hz watchdog。当 color **与** depth 连续 **3 秒**没有任何
帧进入（启动时放宽到 10 秒）,watchdog 会关闭设备进入重连循环，每 2 秒尝试一次。相机重新接回后，原本的 topic 会自动恢复发布 ——
**无需重跑 `ros2 launch`**。

```
[ERROR] watchdog: no frames for 3 s — declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

慢速模式（例如 R77 7 fps）由 10 秒启动宽限期吸收；一旦观察到任一帧，就改套用 3 秒的稳态判定阈值。

### 多相机

连接多台相机时，驱动程序会按 launch 的 `model` 挑选 PID 匹配的设备（G100+ = `0x0181`、R77 = `0x0180`、G62 = `0x0183`）。如果指定的 `model`
找不到对应 PID 的相机，或 `usb_port` / `dev_serial_number` 解析到不同
型号，驱动会直接拒绝打开并报告 PID 不匹配。

两台同型号需要区分时，launch **必须**用序列号或 USB 拓扑路径明确
绑定 —— 自动 model-PID 比对无法分辨相同型号的两台相机。两种绑定
都能跨重启与插拔顺序保持稳定:

```bash
ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=front dev_serial_number:=8036259M200025

ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=rear  usb_port:=2-3:1.0
```

`launch/examples/` 已内建三个多相机示例，每个都预留 `usb_port` 字段，
请按实际接线改写后再启动:

```bash
ros2 launch eys3d_camera examples/dual_G62.launch.py
ros2 launch eys3d_camera examples/dual_G100P.launch.py
ros2 launch eys3d_camera examples/G100P_plus_R77.launch.py
```

内建的多相机示例已预先选好较轻的 `mode_id`,保证两台相机能在 USB
带宽限制内稳定一起启动。如果手动把 `mode_id` 调高、结果有相机起
不来,请退回较轻的 mode,或把两台相机接到不同的 USB port 上。

如果提示都无法匹配候选相机，节点会在 log 列出所有设备的
`(PID, serial_number, /dev/videoN, usb_port)` 后退出 —— 复制正确
的值到 launch 即可。

---

## Frame ID

驱动程序启动时一次性广播静态 TF 树，根节点为 `<camera_name>_link`
（ROS 基底轴：X 前、Y 左、Z 上）。每条串流都有对应的传感器 frame
与 REP-103 `_optical_frame`。PointCloud2 使用 `<camera_name>_points_frame`，
已转至 ROS 基底轴，无需再旋转。

集成到机器人 URDF 时，将 `<camera_name>_link` 挂在已有 frame 下即可。
最小示例（请按平台调整 `parent_link`、joint 名称与安装姿态）:

```xml
<joint name="g100p_mount" type="fixed">
  <parent link="parent_link"/>
  <child  link="G100P_1_link"/>
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</joint>
<link name="G100P_1_link"/>
```

驱动程序以 `TRANSIENT_LOCAL` durability 将相机 TF 树一次性发布到
`/tf_static`,任何后加入的订阅者都会立即收到缓存的 transform。

---

## Composable Node

驱动程序以 `rclcpp` component 形式注册，可加载至
`ComposableNodeContainer` 与下游 component 同处一个 process 并交换
消息。设置 `use_intra_process_comms:=true` 后，图像、深度与点云消息
以指针方式交给同容器订阅者，而非经 DDS 传递。

```bash
ros2 launch eys3d_camera examples/G100P_composable.launch.py \
    use_intra_process_comms:=true
```

`launch/examples/G100P_composable.launch.py` 是起点；在
`composable_node_descriptions` 中追加自己的 `ComposableNode`，并在
每个条目加上 `extra_arguments=[{'use_intra_process_comms': True}]`
即可共享容器。该 launch 文件支持 `params_file` 参数，可在单一 YAML
中携带所有驱动程序设置。

`/tf_static` 始终以 `TRANSIENT_LOCAL` durability 发布，无论节点层级
intra-process 设置如何均可正常工作。

---

## 诊断信息 `/diagnostics`

每秒一条 `DiagnosticArray`。每一条内含 5 个 `DiagnosticStatus`（每
task 一个),名称为 `"<hardware_id>: <task>"`(`hardware_id` 为相机序列号,
模块未上报时则为 `eys3d_camera`)。整体健康状态由 `device` task 的
`level` + `message` 概括:

| `level` | `message` | 含义 |
|---|---|---|
| `OK` | `streaming OK` | 每条启用的串流都达到预期 fps 的 50 % 以上 |
| `WARN` | `one stream below 50% of expected fps` | 某条串流速度过低 |
| `ERROR` | `no frames flowing on enabled streams` | 启用的串流都低于阈值（或皆无数据） |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 断线；watchdog 会在设备回插时自动重连 |

各 task 的 `values` 键值对：

**`device`** — 连接与识别：

| Key | 说明 |
|---|---|
| `connection_state` | `streaming` 或 `disconnected` |
| `device_present` | `true` / `false`，对应 V4L2 节点是否存在 |
| `reconnect_attempts` | 自启动以来累计重连次数 |
| `usb_port` | open 时解析到的 sysfs 接口路径（如 `2-3:1.0`）|
| `serial_number` | SDK 上报的模块序列号 |
| `actual_fps` | `APC_OpenDevice2` 返回的 fps(interleave mode 为每串流半值)|
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` service 控制的运行时状态 |

**`color`** 与 **`depth`** — 每条串流吞吐：

| Key | 说明 |
|---|---|
| `input_fps` | 过去 1 秒从 SDK 收到的帧数。与订阅者状态无关 — 用于判断相机 / USB 健康度 |
| `publish_fps` | 过去 1 秒实际发布到 topic 的帧数。无订阅者时为 0；有订阅者且 driver 跟得上时 ≈ `input_fps`；持续低于代表 driver 落后 |
| `input_total` | 自 open 起累计从 SDK 收到的帧数 |
| `publish_total` | 自 open 起累计发布的帧数 |
| `input_dropped` | 累计 SDK 端掉帧数（由 serial-number 跳跃检测）|
| `decode_avg_ms` | （仅 `color`）过去 1 秒 color 解码平均耗时。当下 1 秒内有解码过 frame 才会出现 |
| `decode_max_ms` | （仅 `color`）至今观察到的最长 color 解码耗时。曾经解码过 frame 才会出现 |

**`pointcloud`** — 点云投影 + 后处理计数：

| Key | 说明 |
|---|---|
| `compute_status` | `active`（有订阅者拉点云）、`idle (no /pointcloud subscriber)`、`idle (never run ; no subscriber since start)` 或 `(disconnected — see device task)` |
| `publish_fps` | 过去 1 秒点云发布帧数。无订阅者时为 0 |
| `compute_avg_ms` | 过去 1 秒点云计算平均耗时（`active` 时出现）|
| `compute_max_ms` | 至今观察到的最长点云计算耗时 |
| `publish_total` | 累计发布的点云数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各后处理阶段的累计调用次数 |

**`thermal`**：

| Key | 说明 |
|---|---|
| `temperature_c` | 芯片温度（°C）；不支持的机型显示 `n/a (not supported on this model)` |

`/diagnostics` 仅在有订阅者连接时才组装并发布消息；hot-path 的
atomic counter 持续累计，所以随时连接监控都能看到自 open 起的累计值。

判读方法：

| `input_fps` | `publish_fps` | 含义 |
|---|---|---|
| ≈ 预期 | 0 | 无订阅者，driver idle |
| ≈ 预期 | ≈ input | driver 跟得上 |
| ≈ 预期 | < input | 有订阅者，但发布路径落后（解码太慢 / DDS congestion / 订阅端 QoS 不对）|
| 0 | 0 | 相机 / USB 没在发送数据 — 查看 `device.connection_state` |

订阅 `/diagnostics`（或运行 `rqt_robot_monitor`）即可实时查看。

内置监控工具：自动订阅所有 stream，并把 SDK / Pub / Rx 三条 rate
与解码 / 计算耗时并列显示：

```bash
ros2 run eys3d_camera perf_monitor            # 自动检测 namespace
ros2 run eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

---

## 故障排除

| 问题 | 解法 |
|---|---|
| 自写的订阅者收不到影像或点云消息 | driver 采用 `SensorDataQoS`(BestEffort + KeepLast 5 + Volatile)发布。订阅端必须声明兼容的 QoS;C++ 用 `rclcpp::SensorDataQoS()`、Python 用 `qos_profile_sensor_data`。详见[订阅端 QoS](#订阅端-qos)章节 |
| RViz Image 面板空白 | 将 Fixed Frame 设为 `<camera_name>_link`，并确认 topic 名称对应当前的 `camera_name` namespace |
| 相机打开失败（device busy） | 其他进程占用 `/dev/videoN`；用 `lsof /dev/video*` 找出来源，若是残留 driver 用 `pkill -9 -f camera_node` 结束 |
| `No device matches binding hints` | log 会列出所有检测到的 eYs3D 模块，把正确的 `dev_serial_number` 或 `usb_port` 填到 launch 参数 |
| `cv_bridge` 报 encoding 不匹配 | 所有 color topic 均以 `rgb8` 发布（不论底层 MJPEG 或 YUYV），订阅端必须声明 `rgb8`，勿用 `bgr8` 或 `mono8` |

### 订阅端收到的 fps 比 driver 公布的少

`perf_monitor` 或 `ros2 topic hz` 显示 Rx < Pub，但 `/diagnostics`
显示 `dropped = 0`，这是 DDS 传输层在丢，不是 driver。

每个支持的 video mode 之单张影像都比 kernel UDP socket 与 IP
fragment reassembly 在一次 burst 内所能吸收的量大(`rgb8` 1280×720
≈ 2.7 MB；典型 point cloud 最多 ~11 MB)。同一个底层原因(kernel
端 buffer 撑爆)会在两种情境中以不同表象出现：

- **性能较高的 x86 主机**:driver 把每张 frame 在微秒级的 UDP
  fragment burst 内塞给 DDS，接收端 socket / reassembly queue 还没
  消化就溢位。
- **运算资源受限的 ARM 主机**:receiver 端读同一个 kernel buffer
  的速度太慢，buffer 提早满，结果一样。

package 附带一份 opt-in 脚本，把 FastRTPS 切到 32 MB 的 shared-
memory segment，可消除任何**同机**订阅者的 UDP fragmentation 问题。
**跨机**订阅者不适用 — 跨机需依
[ROS 2 DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
调高 `net.core.rmem_max`。默认不自动启用，因为 SHM segment 会在
`/dev/shm` 内为每个 ROS 2 participant 预留约 32 MB。

启用方式：把下面这行加到 `~/.bashrc`，位置放在你的
`source <workspace>/install/setup.bash` **之后**，并将 `~/ros2_ws`
替换为你实际的 workspace 路径：

```bash
source ~/ros2_ws/install/eys3d_camera/share/eys3d_camera/config/enable_fastrtps_shm.bash
```

