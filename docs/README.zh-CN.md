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
sudo apt install ros-$ROS_DISTRO-diagnostic-updater \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz2
```

per-model launch 默认 `urdf:=true` 与 `rviz:=true`，缺少
`robot_state_publisher`、`xacro`、`rviz2` 会无法启动；加上
`urdf:=false rviz:=false` 则不需要。

将 `eys3d_camera` 与 `eys3d_camera_interfaces`（自我标定 action 定义）两个 package 一起放入 workspace 的 `src/` 后构建；`--packages-up-to` 会先构建 interfaces:

```bash
cd ~/ros2_ws
colcon build --packages-up-to eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


### 设备权限

驱动以普通用户身份打开相机。若打开设备时出现权限错误，安装随附的
udev 规则，让 eSPDI SDK 能访问该 USB 设备:

```bash
sudo cp install/eys3d_camera/share/eys3d_camera/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

然后重新插拔相机。此规则授予对 eYs3D 设备（USB vendor `3438`）的
访问权限;或者将用户加入 `video` 组后重新登录。

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
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左眼彩色影像，固定 `rgb8`（YUYV 与 MJPEG 来源皆 inline 解码为 RGB888；灰阶传感模块以 R=G=B 输出）。是否经过校正取决于当前的 video mode。|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右眼彩色影像；仅在 video mode 带有 L\|R 并排输出时发布（`split_lr: true`）|
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`,mm,REP-118) | 深度（毫米） |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32，已转换为 ROS 基底轴（米）；当 `colored_pointcloud:=true` 时改为 XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，每张 Image 一张（`header.stamp` 相同） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康状态，1 Hz |

### CameraInfo 与畸变系数

每一份 `camera_info` 描述的都是它自己 topic 上发布的那张图像。

深度在所有模式下都是已校正的;彩色则在目录名称带撇号的模式下已校正
(`L'+D`、`L'+R'+D`)。那些 topic 上的 `k` 是 `p` 的左 3×3、`d` 为零
(`plumb_bob`)、`r` 为单位矩阵 —— 相机已经移除畸变,没有东西需要还原。

`L+R` 与 `L+R+D` 模式在彩色 topic 上发布的是原始传感器图像。那里的
`k` 与 `d` 是发布分辨率下的原厂镜头模型、`r` 是校正旋转,`image_proc`
可以正常校正它们。请读 `distortion_model` 而不要假设系数个数:driver 会报告
`rational_polynomial` 八个系数或 `plumb_bob` 五个,依该台模组内
存储的标定数据而定。

两种情况下 `p` 都是投影矩阵,也是位姿估计(AprilTag、PnP、SLAM)应该
取用的内参来源。立体对的右相机 `p[3]` 是 `-fx × 基线`,单位为米。

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

- **偶数 frame —— IR 关闭 → `/left_color/image_raw`**。
  没有 IR 红外点干扰，输出干净的 RGB 图像。
- **奇数 frame —— IR 开启 → `/depth/image_raw`**。
  IR 红外点为立体匹配提供结构特征，得到较高品质的深度。

驱动在 SDK 串流源头做奇偶 frame 过滤，因此每条串流的 FPS 会减半。

#### 时戳影响

以 G100+ 在 USB 3.0 下的默认 `mode_id=1` 为例：

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color/image_raw  30 fps
                                 └  /G100P_1/depth/image_raw       30 fps
```

color 与 depth **来自相邻两张 sensor frame**，stamp 相差
**1 个 sensor frame ≈ 15.6 ms（1000 / 60）**。
同 stream 内 `Image` 与其 `CameraInfo` stamp 一致。

下游用 `message_filters::ApproximateTime` 配对，`slop` 设一个
sensor frame 周期（例如 20 ms）即可稳定配对。

#### 影响的 mode IDs

于 `launch/video_modes/G100P.yaml` 中，**`mode_id` 1、3、5、7–21、56、57** 为 interleave。
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
| `mode_id` | `-1` | video mode 列表中的索引;`-1` = 自动(按协商到的 USB 速度取该 signature 的默认) |
| `dev_serial_number` | `""` | 通过序列号子串绑定 |
| `usb_port` | `""` | 通过 USB 拓扑路径绑定，例如 `2-3:1.0` |
| `depth_near_mm` | `-1` | 同时套用到 `/depth/image_raw` 与 `/depth/points` 的距离下限，低于此值的像素归零。`-1` = 该型号默认（G100+ 250、R77 200、G62 100）|
| `depth_far_mm` | `-1` | 同时套用到 `/depth/image_raw` 与 `/depth/points` 的距离上限，高于此值的像素归零。`-1` = 该型号默认（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 从最近一张左目彩色帧取色，输出 XYZRGB PointCloud2；depth-only mode 自动回退为 XYZ |
| `spatial_filter` | `false` | 启用视差域边缘感知 IIR 空间滤波器（布尔开关）|
| `temporal_filter` | `false` | 启用时间滤波器，alpha 混合 + persistence（布尔开关，运行中可调）|
| `hole_filling` | `0` | Z 域补洞模式。`0` = 关闭、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（整数模式，非布尔）|
| `filter_profile` | `default` | 滤波器调校配置档名称；解析至 `cfg/filter_profiles/<name>.yaml`，启动时载入滤波器调校值 |
| `ir_value` | `-1` | `-1` = 依模式决定：mode 含深度或机型为黑白（G62 / R77）时采用型号默认值（G100+ / R77 = 3、G62 = 60），彩色传感器跑纯彩色 mode 时关闭；`0` = 关闭；正整数覆盖至 FW 范围（G100+ 0-6、R77 0-6、G62 0-96）|
| `log` | `sdk`（per-model）/ `all`（generic） | 终端输出层级。`all` = RCLCPP + SDK 完整输出；`sdk` = 抑制 RCLCPP INFO/DEBUG，保留 WARN/ERROR 与 SDK printf；`close` = 全部重定向至 per-process log 文件（终端静默）|
| `urdf` | `true` | 通过 namespace 隔离的 `robot_state_publisher` 在 `<camera_name>/robot_description` 发布相机模型（mesh + 锁固孔 frame）。若整机 bringup 已含相机描述，设为 `false` |
| `rviz` | `true` | 是否自动开启 RViz |
| `selfcal_enable` | `false` | 启用可选的自我标定；`true` 会开启 `selfcal/run` action 与 `selfcal/commit` service（见「自我标定」章节）。仅 launch 时可设 |

彩色串流是否切分为 `left_color` 与 `right_color` 由选定的 video mode 决定（`launch/video_modes/<MODEL>.yaml` 的 `split_lr` 旗标），并非由 launch 参数控制。

### 影像参数控制

所有 eYs3D 模组出厂预设 `auto_exposure` 开启、
`auto_white_balance` 开启、`power_line_frequency` 设为
60 Hz；驱动程序仅承袭固件开机时的设置，**只有当操作者明确覆盖时
才写回相机**，所以在 ROS 之外调整过的设置可跨重启保留。IR 是例外，
因为相机开机后投射器默认关闭，所以一定会应用。

运行中即可调整，无需重启：

```bash
ros2 param set /G100P_1/eys3d_camera ir_value             5
ros2 param set /G100P_1/eys3d_camera auto_exposure        false   # 手动曝光
ros2 param set /G100P_1/eys3d_camera exposure_time_step   -8
ros2 param set /G100P_1/eys3d_camera auto_white_balance   false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

`exposure_time_step` 接受 **[-13, 3]** 范围内的带符号整数（log2 曝光寄存器），仅在 `auto_exposure` 为 `false` 时生效。

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

三个 service 可以控制相机输出,而不需要关掉 ROS node。`pause` 与
`standby` 接收 `std_srvs/srv/SetBool`,同时控制 color 与 depth;
`hw_reset` 接收 `std_srvs/srv/Empty`,通过 USB 对相机做硬件复位。

```bash
# 停止发布 frame,但相机仍在 USB 上 stream。Driver CPU 降到接近 0,
# resume 后下一帧立即出现。适合短暂中断的情境。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# 完全释放 USB pipe。ROS node 仍然存活，topic 仍然注册。
# resume 会重新打开相机，依机种需要数秒。适合需要把 USB 带宽让给其他
# 设备的情境。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"

# 通过 USB 复位相机(重新枚举设备)。node 会停止串流、发出复位,
# 之后 watchdog 自动重连,frame 通常在约 12 秒后恢复。用于在不重启
# node 的情况下恢复卡死的相机。
ros2 service call /G100P_1/hw_reset std_srvs/srv/Empty
```

Standby 生效期间，自动重连 watchdog(下一节)会把无信号视为刻意行为，
不会误判为断线。当前的控制状态会在 `/diagnostics` 的 `stream_state`
字段露出(`Active`、`Paused`、`Standby`)。

### 热插拔自动恢复

驱动内置 1 Hz watchdog，每个流各自监看：某个流送出过帧之后，若连续
**3 秒**没有新帧，就关闭设备进入重连循环，每 2 秒尝试一次 —— 因此
depth 在固件里卡住、color 仍持续送的情况也能恢复。第一帧进入之前
门槛放宽为 10 秒，足以覆盖 R77 7 fps 这类慢速模式。相机重新接回后，
原本的 topic 会自动恢复发布，**无需重跑 `ros2 launch`**。

```
[ERROR] watchdog: depth stream silent for 3 s; declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

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
ros2 launch eys3d_camera dual_G62.launch.py
ros2 launch eys3d_camera dual_G100P.launch.py
ros2 launch eys3d_camera G100P_plus_R77.launch.py
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

每个型号均随附含 3D mesh 的 URDF/xacro 描述文件
（`urdf/eys3d_<MODEL>.urdf.xacro` + `meshes/<MODEL>.dae`）。`<name>_link`
位于深度起始点 —— 两颗传感器的横向中点、光轴高度上，距相机前壳面
内缩 Z'（G100+ 6.75 mm、R77 4.8 mm、G62 3.1 mm）。在机器人已有 frame
下以实际安装姿态实例化 macro：

```xml
<xacro:include filename="$(find eys3d_camera)/urdf/eys3d_G100P.urdf.xacro"/>
<xacro:eys3d_G100P name="G100P_1" parent="parent_link">
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</xacro:eys3d_G100P>
```

单独预览（`name` 默认为 `<MODEL>_1`，与驱动程序及随附 rviz 布局一致）：

```bash
ros2 launch eys3d_camera display_model.launch.py model:=G100P
```

并排预览三款模型（无需硬件）:

```bash
ros2 launch eys3d_camera three_models.launch.py
```

每份描述文件同时带有机壳锁固孔 frame（`<name>_tripod_frame` 与
`<name>_back/bottom_screw*_frame`，位置取自原厂 CAD），供机构
集成时核对虚拟与实体的一致性。

驱动程序以 `TRANSIENT_LOCAL` durability 将相机 TF 树一次性发布到
`/tf_static`,任何后加入的订阅者都会立即收到缓存的 transform。

---

## Composable Node

驱动程序以 `rclcpp` component 形式注册，可加载至
`ComposableNodeContainer` 与下游 component 同处一个 process 并交换
消息。设置 `use_intra_process_comms:=true` 后，图像、深度与点云消息
以指针方式交给同容器订阅者，而非经 DDS 传递。

```bash
ros2 launch eys3d_camera G100P_composable.launch.py \
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
| `OK` | `streaming` | 每条配置中的流都在发送 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` 生效中 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` 生效中 |
| `ERROR` | `no frames flowing on any configured stream` | 每条配置中的流都低于预期速率的一半 |
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

两者的摘要为 `streaming`、`input rate below 50% of expected`（WARN）、
`not configured (D-only mode)` 或 `standby`。

**`pointcloud`** — 点云投影 + 后处理计数：

| Key | 说明 |
|---|---|
| `compute_status` | `active`（有订阅者拉点云）、`idle (no /depth/points subscriber)`、`idle (never run ; no subscriber since start)` 或 `(disconnected — see device task)` |
| `publish_fps` | 过去 1 秒点云发布帧数。无订阅者时为 0 |
| `compute_avg_ms` | 过去 1 秒点云计算平均耗时（`active` 时出现）|
| `compute_max_ms` | 至今观察到的最长点云计算耗时 |
| `publish_total` | 累计发布的点云数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各后处理阶段的累计调用次数 |

**`thermal`**：

| Key | 说明 |
|---|---|
| `temperature_c` | 芯片温度（°C）。仅在机型具备传感器且读取成功时发布；否则不会出现该字段，原因写在该任务的 summary |

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

## 自我标定

可选的流内自我标定会重新对齐立体视觉配对,在标定已漂移的模组上恢复深度
填充率。默认即编入(CMake `EYS3D_WITH_SELFCAL`),并于 launch 时以
`selfcal_enable:=true` 启用。

```bash
ros2 launch eys3d_camera G100P.launch.py selfcal_enable:=true
```

执行时相机需**正在流式传输深度模式,并对准工作距离下一般、有纹理的场景** ——
标定器以深度覆盖率为测量依据,因此需要有效深度。一个 `selfcal/run` action
即执行完整一次 session,并自动套用内建的调校参数。

```bash
# 执行一次 session(阻塞直到收敛,约 20-30 秒,接着一段短暂复检;--feedback
# 会流式传输 phase / progress)。auto_commit_shift_px < 0 会让结果生效但永不写
# flash;>= 0 则在验证为更好的一次执行中,当 cy 位移达到这么多像素时自动烧录
# (见下表)。
ros2 action send_goal /G100P_1/selfcal/run \
  eys3d_camera_interfaces/action/SelfCal \
  "{auto_commit_shift_px: 0.25}" --feedback

# 将保留的结果写入 flash(仅在未自动烧录时需要)。
# 检查响应的 `success` 字段 —— 若没有可保留的结果会是 false。
ros2 service call /G100P_1/selfcal/commit std_srvs/srv/Trigger
```

搜索结束后,此次会依标定器的 outcome **加上一次实时 A/B 复检**自行收尾 ——
复检在同一场景测量“新对齐 vs 跑之前对齐”的深度填充率,因此判定反映真实的
前后差异,而非场景相关的臆测:

- **验证为更好** —— 复检确认的 `SUCCESS`,已生效(`applied: true`)。若有设
  `auto_commit_shift_px` 且 `cy_shift_px` 达标,此次即**烧录**进 flash
  (`committed: true`);否则**保留生效**但为暂存 —— 调用 `selfcal/commit`
  才能在断电后留存。
- **已是最佳**(`NO_CHANGE`)—— 不改动、不保留、不写入。这是正常且健康的结果。
- **更差 / 无法验证 / 失败** —— 复检判定更差或无法确认(例如跑的过程中相机
  移动了),或 `INSUFFICIENT_INPUT` / `TIMEOUT` / `FAILED` —— 相机**还原**回
  跑之前的对齐(`reverted: true`)。

`cy_shift_px` 是实测的垂直位移,直接读自硬件,也是自动烧录闸门采用的值。步长
固定在 **0.25 px**,校正量上限为 **5.0 px**,所以它永远是 `0.25、0.50、… 5.00`
其中之一(或 `0`)。`auto_commit_shift_px` 是阈值 —— 可带任何值,但只有这些
步长边界会改变行为,介于步长之间的值会向上取整(`0.3` 等同 `0.5`):

| `auto_commit_shift_px` | 效果 |
| --- | --- |
| `-1`(默认)| 永不自动烧 —— 查看后手动烧录。未传该参数时采用的默认值 |
| `0.25` | 任何真实移动(≥ 1 步)就烧 —— **建议值** |
| `0.50`、`0.75`、… 直到 `5.00` | 要求更大的位移才烧 |
| `> 5.0` | 永不触发(位移不可能超过上限)|

### Action 反馈

执行进行中持续流式传输,前提是 goal 送出时带上 `--feedback`:

| 字段 | 含义 |
| --- | --- |
| `phase` | `INITIAL_SEARCH` / `REFINEMENT` / `RECHECK` / `COMPLETED` |
| `progress` | 搜索进度,`0.0`–`1.0` |
| `processed_frames` | 至今喂给标定器的深度帧数 |
| `valid_ratio_latest` | 最新一帧的填充率(有结果后才有值)|

### Action 结果

无法启动、无法得出结论、或无法回退已否决的变更时，goal 会 abort；刻意的
revert 则是 succeed。两种情况的细节都在 `outcome`。

此次执行收尾时返回一次 —— 记录这次 session 测量到什么、又在相机上留下什么:

| 字段 | 含义 |
| --- | --- |
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`(场景中有效深度不足)/ `TIMEOUT`(未在时限内收敛)/ `FAILED` |
| `cy_shift_px` | 实测垂直 cy 位移(px)—— 自动烧录阈值依据 |
| `recheck_verdict` | A/B 复检:`improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `recheck_ratio_after` | 复检时「跑前 / 收敛后」对齐的填充率 |
| `applied` | 修正已生效于寄存器 |
| `reverted` | 已还原回跑前标定 |
| `committed` | 已写入用户标定区 |
| `message` | 结果的人类可读摘要 |
| `correction_level`、`valid_ratio_first` / `_latest` / `_delta` | 仅供诊断(永不作为烧录阈值)|

整个 session 全程流式:深度持续发布、任何后处理滤镜持续运作 —— 此次执行从不
重启流。搜索期间深度质量会随标定器探测各对齐而明显波动,收敛后便稳定下来。
session **无法中断**(cancel 一律拒绝,控制设置项与 `pause` / `standby` /
`hw_reset` 也会拒绝)—— 它很短,且结果更差时会自动还原,所以没有需要中断的
情形。

`commit` 是唯一写入 flash 的步骤;原厂标定保留作为备份,永不被覆写。**保留但
未烧录**的结果只存在于相机寄存器,断电重开或调用 `~/hw_reset` 就会清回已存储
的标定。这让“在流上标定、永不写 flash”是默认就安全的流程 —— 每当机器人
停靠且静止时执行即可。

有两点值得注意:

- **连续执行会叠加。** 在 commit 或断电重开之前再执行一次 `selfcal/run`,是
  接着上一次保留的结果继续、而不是从 flash 重来。想要一个干净的基准,请先
  `commit` 或断电重开。
- **不符合条件 / 已有 session 在跑 → 是拒绝,不是失败。** 不支持自我标定的
  模组,以及已有 session 在跑时再发送的第二个 `selfcal/run`,都会直接被拒绝
  —— `ros2 action send_goal` 报告「Goal was rejected」(没有 `Result`),原因
  需要看 node log。

**每个 process 只能有一次 session。** 标定器以 process 全局变量绑定相机
handle,因此共用同一个 process 的相机 —— 例如同一个
`ComposableNodeContainer` 内的多个驱动程序 —— 一次只能标定一台;在第一次执行
结束之前,第二个 `selfcal/run` 会被拒绝并在 node log 留下消息。以独立 process
启动的相机则不受影响。除此之外与 intra-process 通信互不相干:标定器会自行取得
一份深度帧的副本,因此执行期间 `use_intra_process_comms` 的传递方式不变。

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
显示 `input_dropped = 0`，这是 DDS 传输层在丢，不是 driver。

每个支持的 video mode 之单张影像都比 kernel UDP socket 与 IP
fragment reassembly 在一次 burst 内所能吸收的量大(`rgb8` 1280×720
≈ 2.7 MB；典型 point cloud 最多 ~11 MB)，因此 kernel 端 buffer 会
溢位 —— 不论是 x86 主机送得太快、还是 ARM 主机读得太慢。

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

