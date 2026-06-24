# ROS 2 Driver for eYs3D Stereo Depth Cameras

[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy%20%7C%20Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/rolling/Releases.html)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**Language:** [English](README.md) · [日本語](docs/README.ja.md) · [繁體中文](docs/README.zh-TW.md) · [简体中文](docs/README.zh-CN.md)

`eys3d_camera` is the official ROS 2 driver for eYs3D stereo depth
cameras. It publishes color, depth, and point cloud topics with a
standard REP-103 frame tree. Supports ROS 2 Foxy, Humble, and Jazzy.

### Supported Devices

| Module | Product code | USB | Status |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | Production |
| **R77** | YX8072 | USB 2.0 | Production |
| **G62** | YX8081 | USB 2.0 | Production |

---

## Installation

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
```

Place this package under your workspace `src/` and build:

```bash
cd ~/ros2_ws
colcon build --packages-select eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The per-model launches open RViz by default; install it once with
`sudo apt install ros-$ROS_DISTRO-rviz2`, or pass `rviz:=false` to skip
the visualiser.

---

## Quick Start

Each model has a launch shortcut that opens its default mode:

```bash
ros2 launch eys3d_camera G100P.launch.py     # G100+: L'+D 1280x720 interleave, SDK 30 fps per stream
ros2 launch eys3d_camera R77.launch.py       # R77:   L'+D 1280x920 color + 640x460 depth @ 30 fps
ros2 launch eys3d_camera G62.launch.py       # G62:   L'+D 640x480 @ 25 fps
```

Override the mode with `mode_id:=<n>`; the full per-model catalogue is
in `launch/video_modes/<MODEL>.yaml`.

### Published Topics

With `camera_name=G100P_1` (matches the per-model defaults):

| Topic | Type | Description |
|---|---|---|
| `/G100P_1/left_color` | `sensor_msgs/Image` | Left color image, always `rgb8` (YUYV and MJPEG sources are decoded inline; grayscale-source modules deliver R=G=B). Rectified or raw is determined by the active video mode. |
| `/G100P_1/right_color` | `sensor_msgs/Image` | Right color image; published only in wide-color modes that carry L\|R on one endpoint (`split_lr: true` in the video-mode YAML) |
| `/G100P_1/depth_image` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | Depth in millimetres |
| `/G100P_1/pointcloud` | `sensor_msgs/PointCloud2` | XYZ float32 in ROS base axes (metres); XYZRGB when `colored_pointcloud:=true`. The bundled RViz layout colours the cloud with FlatColor; switch the PointCloud display's `Color Transformer` to `RGB8` to render the colour channel |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, one per matching Image frame (same `header.stamp`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health metrics, 1 Hz |

### Subscriber QoS

Image, depth, and PointCloud2 topics use `SensorDataQoS`
(BestEffort + KeepLast 5 + Volatile). Subscribers must declare the
same QoS:

```cpp
sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(), callback);
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, topic, callback, qos_profile=qos_profile_sensor_data)
```

`ros2 topic hz` and `ros2 topic echo` in recent ros2cli releases
auto-negotiate QoS, so no flag is required.

### Interleave Mode (G100+ only)

The stereo camera has no dedicated third sensor for RGB output.
To deliver both "color without IR dots" and "high-quality depth"
at the same time, eYs3D introduces an interleave mode:

- **Even frame — IR off → `/left_color`**.
  Without IR dots in the scene, the output is a clean RGB image.
- **Odd frame — IR on → `/depth_image`**.
  IR dots provide structured features for the stereo matcher,
  yielding higher-quality depth.

The driver separates even and odd frames at the SDK stream source,
so the per-stream FPS is halved.

#### Timestamp consequence

Using `mode_id=1` (the G100+ default) as an example:

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color   30 fps
                                 └  /G100P_1/depth_image  30 fps
```

`color` and `depth` come from **two adjacent sensor frames**, so
their stamps differ by **one sensor-frame period ≈ 15.6 ms
(1000 / 60)**. Within a stream, `Image` and its `CameraInfo` share
the same stamp.

Pair color and depth downstream with
`message_filters::ApproximateTime`; setting `slop` to one
sensor-frame period (e.g. 20 ms) yields a stable match.

#### Affected mode IDs

In `launch/video_modes/G100P.yaml`, `mode_id` **1, 3, 5, 7–21** are
interleave. The other modes capture color and depth simultaneously
with matching stamps.

---

## Configuration

### Switch Video Mode

```bash
ros2 launch eys3d_camera G100P.launch.py mode_id:=7    # L'+D 640x480 interleave (SDK 30 fps)
ros2 launch eys3d_camera R77.launch.py   mode_id:=4    # D-only 640x460 @ 30 fps
ros2 launch eys3d_camera G62.launch.py   mode_id:=3    # L'+D 320x240 @ 30 fps
```

Full mode catalogue per camera lives at `launch/video_modes/<MODEL>.yaml`.
The node prints the full table on startup at the RCLCPP INFO level, which
is suppressed by the per-model launches' default `log:=sdk`. Pass
`log:=all` (or open the YAML directly) to view the catalogue.

### Launch Parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace + frame-id prefix |
| `mode_id` | `1` | Mode index in the per-model catalogue |
| `dev_serial_number` | `""` | Bind by serial-number substring |
| `usb_port` | `""` | Bind by USB topology, e.g. `2-3:1.0` |
| `depth_minimum_mm` | `-1` | Near cutoff applied to `/depth_image` and `/pointcloud`; pixels closer than this are zeroed. `-1` = per-model default (G100+ 250, R77 200, G62 100) |
| `depth_maximum_mm` | `-1` | Far cutoff applied to `/depth_image` and `/pointcloud`; pixels beyond this are zeroed. `-1` = per-model default (G100+ 1900, R77/G62 1500) |
| `colored_pointcloud` | `false` | Publish XYZRGB PointCloud2 sampled from the latest left-color frame. Falls back to XYZ on depth-only modes |
| `spatial_filter` | `false` | Enable the disparity-domain edge-aware IIR spatial filter (boolean toggle) |
| `temporal_filter` | `false` | Enable the temporal filter, alpha-blend + persistence (boolean toggle, runtime-tunable) |
| `hole_filling` | `0` | Z-domain hole fill mode. `0` = off, `1` = fill_from_left, `2` = farthest_from_around, `3` = nearest_from_around (integer, not boolean) |
| `filter_profile` | `default` | Tuning profile name; resolves to `cfg/filter_profiles/<name>.yaml`. Seeds filter tuning values at startup |
| `ir_intensity` | `-1` | `-1` = per-model default (G100+ / R77 = 3, G62 = 60); `0` = off; positive integer in the FW range (G100+ 0-9, R77 0-6, G62 0-96) to override |
| `log` | `sdk` (per-model) / `all` (generic) | Terminal output level. `all` = full RCLCPP + SDK output; `sdk` = suppress RCLCPP INFO/DEBUG but keep WARN/ERROR plus SDK printf; `close` = redirect everything to a per-process log file (terminal silent) |
| `rviz` | `true` | Auto-open RViz |

Whether the color stream is split into `left_color` and `right_color`
is determined by the selected video mode (the `split_lr` flag in
`launch/video_modes/<MODEL>.yaml`), not a launch parameter.

### Image Controls

All eYs3D modules ship with `enable_auto_exposure` on,
`enable_auto_white_balance` on, and `power_line_frequency` set to 60
Hz; the driver inherits whatever the firmware boots with and only
writes a value back when the operator explicitly overrides it, so
settings tuned outside ROS persist across restarts. IR is the
exception: it is always applied because the camera boots with the
projector off.

Apply runtime overrides — no restart needed:

```bash
ros2 param set /G100P_1/eys3d_camera ir_intensity         5
ros2 param set /G100P_1/eys3d_camera enable_auto_exposure      false   # manual exposure
ros2 param set /G100P_1/eys3d_camera exposure_time_step        -8
ros2 param set /G100P_1/eys3d_camera enable_auto_white_balance false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz, 2 = 60 Hz
```

### Post-Processing Filters

Three optional filters are bundled. All are off by default and run
independently — any combination is supported.

| Filter | Toggle |
|---|---|
| `spatial_filter` | Launch only |
| `temporal_filter` | Launch + runtime |
| `hole_filling` | Launch only (`0` = off; `2` = recommended) |

Tuning values (`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`) live in `cfg/filter_profiles/<name>.yaml`. The shipped
`default.yaml` is a single conservative tuning that runs across all
three modules. Authoring a profile is the recommended way to adapt
the filter behaviour to a specific working envelope (working
distance range, scene texture, motion characteristics) — copy
`default.yaml`, edit fields, and load via `filter_profile:=<name>`.
Absent fields fall back to the node's compiled defaults.

```bash
# Enable spatial + temporal with the default tuning profile
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true temporal_filter:=true

# Add farthest-from-around hole filling (recommended starting mode)
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true hole_filling:=2

# Swap in a custom tuning profile
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true filter_profile:=indoor
```

Temporal filter parameters can be retuned at runtime:

```bash
ros2 param set /G100P_1/eys3d_camera temporal_filter             true
ros2 param set /G100P_1/eys3d_camera temporal_filter_alpha       0.4
ros2 param set /G100P_1/eys3d_camera temporal_filter_persistence 3
```

### Runtime Stream Control

Two services pause or stop the camera output without unloading the
ROS node. Both control the color and depth streams together.

```bash
# Stop publishing frames but keep the camera streaming on USB.
# Driver CPU drops to near zero and resume appears on the next frame.
# Use for short interruptions.
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# Release the USB pipe entirely. The ROS node stays alive and its
# topics stay advertised. Resume reopens the camera in roughly 300 ms.
# Use to free USB bandwidth for another device.
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"
```

While Standby is in effect, the auto-recovery watchdog (next section)
treats the silence as intentional rather than a disconnect.
The current control state is published on `/diagnostics` under the
`stream_state` key (`Active`, `Paused`, or `Standby`).

### Hot-plug Auto-recovery

The node runs a 1 Hz watchdog. If no color **and** no depth frames
arrive for **3 consecutive seconds** (10 s at startup), the watchdog
closes the device and enters a reconnect loop, polling the SDK every
2 s. When the camera comes back the streams resume on the same
topics — no `ros2 launch` restart required.

```
[ERROR] watchdog: no frames for 3 s — declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

Slow modes (e.g. R77 7 fps) are accommodated by the 10 s startup
grace; once any frame has been observed, the 3 s steady-state
threshold takes effect.

### Multiple Cameras

When several cameras are attached the driver picks the device whose USB
PID matches the launch `model` (G100+ = `0x0181`, R77 = `0x0180`,
G62 = `0x0183`). A launch targeting a model whose camera is not
attached, or a `usb_port` / `dev_serial_number` that resolves to the
wrong model, refuses to open and exits with a PID-mismatch error.

For two cameras of the same model, the launch **must** bind each
instance explicitly by serial number or USB port — automatic
model-PID matching cannot distinguish two cameras of the same model.
Both bindings survive reboots and plug order:

```bash
ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=front dev_serial_number:=8036259M200025

ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=rear  usb_port:=2-3:1.0
```

Three multi-camera example launches are bundled under `launch/examples/`.
Each one ships with `usb_port` placeholders — edit them to match your
wiring before launching:

```bash
ros2 launch eys3d_camera examples/dual_G62.launch.py
ros2 launch eys3d_camera examples/dual_G100P.launch.py
ros2 launch eys3d_camera examples/G100P_plus_R77.launch.py
```

The bundled multi-camera examples pre-select lighter `mode_id`
values so two cameras come up together reliably within USB
bandwidth limits. If you raise either `mode_id` and the launch
fails to bring up a camera, step the mode back down or split the
cameras across separate USB ports.

If no candidate matches the bind hints, the node logs every attached
camera's `(PID, serial_number, /dev/videoN, usb_port)` and exits — copy
the right entry into your launch.

---

## Frame ID

The driver broadcasts a static TF tree once at startup, rooted at
`<camera_name>_link` (ROS base axes: X forward, Y left, Z up). Each
stream has both a sensor frame and a REP-103 `_optical_frame`.
PointCloud2 carries `<camera_name>_points_frame` and is already in ROS
base axes — no rotation needed.

Mount `<camera_name>_link` under your robot's existing frame in URDF
to integrate. Minimal skeleton (replace `parent_link`, the joint
name, and the mount pose with values that match your platform):

```xml
<joint name="g100p_mount" type="fixed">
  <parent link="parent_link"/>
  <child  link="G100P_1_link"/>
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</joint>
<link name="G100P_1_link"/>
```

The driver publishes the camera TF tree once on `/tf_static` with
`TRANSIENT_LOCAL` durability, so any subscriber that joins later
receives the cached transforms immediately.

---

## Composable Node

The driver is registered as an `rclcpp` component so it can be
loaded into a `ComposableNodeContainer` and exchange messages with
downstream components in the same process. With
`use_intra_process_comms:=true`, image, depth, and point-cloud
messages are delivered to same-container subscribers by pointer
instead of through DDS.

```bash
ros2 launch eys3d_camera examples/G100P_composable.launch.py \
    use_intra_process_comms:=true
```

`launch/examples/G100P_composable.launch.py` is a starting point;
append additional `ComposableNode` entries with
`extra_arguments=[{'use_intra_process_comms': True}]` to share the
container. The launch accepts a `params_file` for full YAML
configuration.

`/tf_static` is always published with `TRANSIENT_LOCAL` durability,
including when the node-level intra-process setting is enabled.

---

## Diagnostics (/diagnostics)

`/diagnostics` carries one `DiagnosticArray` per second. Each round
emits five `DiagnosticStatus` entries — one per task — named
`"<hardware_id>: <task>"` (`hardware_id` is the camera serial number
when reported by the module, otherwise `eys3d_camera`). The `device`
task carries the overall health summary:

| `level` | `message` | Meaning |
|---|---|---|
| `OK` | `streaming OK` | Every enabled stream is delivering at least 50 % of its expected fps |
| `WARN` | `one stream below 50% of expected fps` | One enabled stream is slow |
| `ERROR` | `no frames flowing on enabled streams` | Both enabled streams below threshold (or both empty) |
| `ERROR` | `camera disconnected; Linux device node not present` | USB lost; the watchdog will reconnect when the device returns |

Per-task `values` (KeyValue pairs):

**`device`** — connection + identity:

| Key | Description |
|---|---|
| `connection_state` | `streaming` or `disconnected` |
| `device_present` | `true` / `false` mirror of the V4L2 node presence |
| `reconnect_attempts` | Cumulative reconnect attempts since launch |
| `usb_port` | sysfs interface path resolved at open (`2-3:1.0` style) |
| `serial_number` | Module SN reported by the SDK |
| `actual_fps` | FPS that `APC_OpenDevice2` returned (per-stream after interleave halving) |
| `stream_state` | `Active` / `Paused` / `Standby` — runtime tier reported by `pause` / `standby` services |

**`color`** and **`depth`** — per-stream throughput:

| Key | Description |
|---|---|
| `input_fps` | Frames per second received from the camera SDK over the past second. Independent of subscribers — measures camera / USB health |
| `publish_fps` | Frames per second actually emitted to the topic over the past second. Zero when no subscribers; matches `input_fps` when subscribers exist and the driver keeps up. A persistent gap indicates the driver is behind |
| `input_total` | Cumulative frames received from the SDK since open |
| `publish_total` | Cumulative frames published since open |
| `input_dropped` | Cumulative SDK-side drop counter (detected via serial-number gaps) |
| `decode_avg_ms` | (color only) Mean color decode time during the past second. Reported only when at least one frame was decoded in that period |
| `decode_max_ms` | (color only) Longest color decode time observed so far. Reported only when at least one frame has been decoded |

**`pointcloud`** — reprojection + post-processing counters:

| Key | Description |
|---|---|
| `compute_status` | `active` (a subscriber is pulling clouds), `idle (no /pointcloud subscriber)`, `idle (never run ; no subscriber since start)`, or `(disconnected — see device task)` |
| `publish_fps` | Clouds published per second over the past second. Zero when no subscriber |
| `compute_avg_ms` | Mean compute time per cloud during the past second (present when `active`) |
| `compute_max_ms` | Longest compute time observed so far |
| `publish_total` | Cumulative number of clouds published |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | Cumulative invocation counters for each enabled post-processing stage |

**`thermal`**:

| Key | Description |
|---|---|
| `temperature_c` | On-die thermal sensor (°C). `n/a (not supported on this model)` on modules without one |

The diagnostic message is only built and published when at least one
subscriber is connected to `/diagnostics`; the hot-path atomic counters
keep ticking either way, so attaching a monitor at any moment shows
the cumulative values from open.

How to read it:

| `input_fps` | `publish_fps` | Meaning |
|---|---|---|
| ≈ expected | 0 | No subscribers; driver is idle |
| ≈ expected | ≈ input | Driver keeps up |
| ≈ expected | < input | Subscribers present, publish path is behind (decode too slow / DDS congestion / subscriber QoS) |
| 0 | 0 | Camera / USB not delivering — check `device.connection_state` |

Pipe `/diagnostics` into `rqt_robot_monitor` (or `ros2 topic echo`) for a
live readout.

A bundled monitor subscribes to every stream and prints
SDK / Pub / Rx rates per stream alongside decode and compute timings:

```bash
ros2 run eys3d_camera perf_monitor            # auto-detect namespace
ros2 run eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

---

## Troubleshooting

| Symptom | Resolution |
|---|---|
| Custom subscriber code receives nothing on image or point-cloud topics | The driver publishes with `SensorDataQoS` (BestEffort + KeepLast 5 + Volatile). Subscribers must request a compatible QoS — `rclcpp::SensorDataQoS()` in C++ or `qos_profile_sensor_data` in Python. See the [Subscriber QoS](#subscriber-qos) section. |
| RViz Image panel is blank | Set Fixed Frame to `<camera_name>_link` and confirm the topic name matches the active `camera_name` namespace. |
| Camera open fails with "device busy" | Another process is holding `/dev/videoN`. Identify it with `lsof /dev/video*`; if it is a stale driver instance, terminate with `pkill -9 -f camera_node`. |
| `No device matches binding hints` | The log enumerates every attached eYs3D module. Copy the correct `dev_serial_number` or `usb_port` from the listing into the launch arguments. |
| `cv_bridge` reports an encoding mismatch | All color topics publish `rgb8` regardless of source wire format. Subscribers must request `rgb8`, not `bgr8` or `mono8`. |

### Subscribers receive fewer frames than the driver publishes

If `perf_monitor` or `ros2 topic hz` reports Rx < Pub while
`/diagnostics` shows `dropped = 0`, the loss is on the DDS transport,
not in the driver.

Each image frame in the supported modes is larger than the kernel UDP
socket and IP fragment reassembly budget can absorb in a single burst
(an `rgb8` 1280×720 frame is ~2.7 MB; a typical point cloud is up to
~11 MB). The same symptom shows up in two configurations — both are
the same underlying mechanism (kernel-side buffer overflow), seen from
different ends:

- **Fast x86 hosts** emit each frame as a tight burst of UDP fragments
  that overflows the receive socket / reassembly queue before the
  receiver thread drains it.
- **Resource-constrained ARM hosts** drain the same kernel buffer too
  slowly, so the buffer fills sooner, with the same outcome.

The package ships an opt-in helper that switches FastRTPS to a 32 MB
shared-memory segment, eliminating UDP fragmentation for any same-host
subscriber. It does not help subscribers on a different host; for
those, raise `net.core.rmem_max` as documented in the [ROS 2 DDS
tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html).
The helper is not enabled automatically because the SHM segment
reserves ~32 MB of `/dev/shm` per ROS 2 participant.

To turn it on for your user account, append the following to
`~/.bashrc`, **after** your `source <workspace>/install/setup.bash`
line. Replace `~/ros2_ws` with your actual workspace path:

```bash
source ~/ros2_ws/install/eys3d_camera/share/eys3d_camera/config/enable_fastrtps_shm.bash
```

