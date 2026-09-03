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
sudo apt install ros-$ROS_DISTRO-diagnostic-updater \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz2
```

The per-model launches default to `urdf:=true` and `rviz:=true` and will not
start without `robot_state_publisher`, `xacro` and `rviz2`. Pass
`urdf:=false rviz:=false` to run without them.

Place both packages — `eys3d_camera` and `eys3d_camera_interfaces` (the
self-calibration action definition) — under your workspace `src/` and build.
`--packages-up-to` builds the interfaces first:

```bash
cd ~/ros2_ws
colcon build --packages-up-to eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


### Device Permissions

The driver opens the camera as a normal user. If device open fails with a
permission error, install the bundled udev rule so the eSPDI SDK can reach
the USB device:

```bash
sudo cp install/eys3d_camera/share/eys3d_camera/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then replug the camera. The rule grants access to eYs3D devices (USB vendor
`3438`); alternatively, add your user to the `video` group and re-login.

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
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | Left color image, always `rgb8` (YUYV and MJPEG sources are decoded inline; grayscale-source modules deliver R=G=B). Rectified or raw is determined by the active video mode. |
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | Right color image; published only in wide-color modes that carry L\|R on one endpoint (`split_lr: true` in the video-mode YAML) |
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | Depth in millimetres |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32 in ROS base axes (metres); XYZRGB when `colored_pointcloud:=true`. The bundled RViz layout colours the cloud with FlatColor; switch the PointCloud display's `Color Transformer` to `RGB8` to render the colour channel |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, one per matching Image frame (same `header.stamp`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health metrics, 1 Hz |

### CameraInfo and Distortion Coefficients

Each `camera_info` describes the image published on its own topic.

Depth is rectified in every mode, and color is rectified in every mode whose
catalogue name carries an apostrophe (`L'+D`, `L'+R'+D`). On those topics `k`
is the left 3x3 of `p`, `d` is zero (`plumb_bob`) and `r` is the identity: the
camera has already removed the distortion, so there is nothing left to undo.

The `L+R` and `L+R+D` modes publish the raw sensor image on the color topics.
There `k` and `d` are the factory lens model at the published resolution and
`r` is the rectification rotation, so `image_proc` rectifies them normally.
Read `distortion_model` rather than assuming a length: the driver reports
`rational_polynomial` with eight coefficients or `plumb_bob` with five,
following the calibration stored in that individual module.

`p` is the projection matrix in both cases and is the right source of
intrinsics for pose estimation (AprilTag, PnP, SLAM). For a stereo pair,
`p[3]` of the right camera is `-fx * baseline` in metres.

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

- **Even frame — IR off → `/left_color/image_raw`**.
  Without IR dots in the scene, the output is a clean RGB image.
- **Odd frame — IR on → `/depth/image_raw`**.
  IR dots provide structured features for the stereo matcher,
  yielding higher-quality depth.

The driver separates even and odd frames at the SDK stream source,
so the per-stream FPS is halved.

#### Timestamp consequence

Using `mode_id=1` (the G100+ default on USB 3.0) as an example:

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color/image_raw  30 fps
                                 └  /G100P_1/depth/image_raw       30 fps
```

`color` and `depth` come from **two adjacent sensor frames**, so
their stamps differ by **one sensor-frame period ≈ 15.6 ms
(1000 / 60)**. Within a stream, `Image` and its `CameraInfo` share
the same stamp.

Pair color and depth downstream with
`message_filters::ApproximateTime`; setting `slop` to one
sensor-frame period (e.g. 20 ms) yields a stable match.

#### Affected mode IDs

In `launch/video_modes/G100P.yaml`, `mode_id` **1, 3, 5, 7–21, 56, 57** are
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
| `mode_id` | `-1` | Mode index in the per-model catalogue; `-1` = auto (the per-link signature default for the negotiated USB speed) |
| `dev_serial_number` | `""` | Bind by serial-number substring |
| `usb_port` | `""` | Bind by USB topology, e.g. `2-3:1.0` |
| `depth_near_mm` | `-1` | Near cutoff applied to `/depth/image_raw` and `/depth/points`; pixels closer than this are zeroed. `-1` = per-model default (G100+ 250, R77 200, G62 100) |
| `depth_far_mm` | `-1` | Far cutoff applied to `/depth/image_raw` and `/depth/points`; pixels beyond this are zeroed. `-1` = per-model default (G100+ 1900, R77/G62 1500) |
| `colored_pointcloud` | `false` | Publish XYZRGB PointCloud2 sampled from the latest left-color frame. Falls back to XYZ on depth-only modes |
| `spatial_filter` | `false` | Enable the disparity-domain edge-aware IIR spatial filter (boolean toggle) |
| `temporal_filter` | `false` | Enable the temporal filter, alpha-blend + persistence (boolean toggle, runtime-tunable) |
| `hole_filling` | `0` | Z-domain hole fill mode. `0` = off, `1` = fill_from_left, `2` = farthest_from_around, `3` = nearest_from_around (integer, not boolean) |
| `filter_profile` | `default` | Tuning profile name; resolves to `cfg/filter_profiles/<name>.yaml`. Seeds filter tuning values at startup |
| `ir_value` | `-1` | `-1` = per-mode default: the model default (G100+ / R77 = 3, G62 = 60) when the mode has depth or the module is monochrome (G62 / R77), off for a color-only mode on a color sensor; `0` = off; positive integer in the FW range (G100+ 0-6, R77 0-6, G62 0-96) to override |
| `log` | `sdk` (per-model) / `all` (generic) | Terminal output level. `all` = full RCLCPP + SDK output; `sdk` = suppress RCLCPP INFO/DEBUG but keep WARN/ERROR plus SDK printf; `close` = redirect everything to a per-process log file (terminal silent) |
| `urdf` | `true` | Publish the camera model (mesh + mounting-hole frames) on `<camera_name>/robot_description` via a namespaced `robot_state_publisher`. Set `false` when the robot bringup already carries the camera in its own description |
| `rviz` | `true` | Auto-open RViz |
| `selfcal_enable` | `false` | Arm the optional self-calibration; `true` enables the `selfcal/run` action and `selfcal/commit` service (see [Self-Calibration](#self-calibration)). Launch-time only |

Whether the color stream is split into `left_color` and `right_color`
is determined by the selected video mode (the `split_lr` flag in
`launch/video_modes/<MODEL>.yaml`), not a launch parameter.

### Image Controls

All eYs3D modules ship with `auto_exposure` on,
`auto_white_balance` on, and `power_line_frequency` set to 60
Hz; the driver inherits whatever the firmware boots with and only
writes a value back when the operator explicitly overrides it, so
settings tuned outside ROS persist across restarts. IR is the
exception: it is always applied because the camera boots with the
projector off.

Apply runtime overrides — no restart needed:

```bash
ros2 param set /G100P_1/eys3d_camera ir_value             5
ros2 param set /G100P_1/eys3d_camera auto_exposure        false   # manual exposure
ros2 param set /G100P_1/eys3d_camera exposure_time_step   -8
ros2 param set /G100P_1/eys3d_camera auto_white_balance   false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz, 2 = 60 Hz
```

`exposure_time_step` takes a signed integer in **[-13, 3]** (log2 exposure
register) and applies only while `auto_exposure` is `false`.

### Post-Processing Filters

Three optional filters are bundled. All are off by default and run
independently — any combination is supported.

| Filter | Toggle |
|---|---|
| `spatial_filter` | Launch only |
| `temporal_filter` | Launch + runtime |
| `hole_filling` | Launch only (`0` = off; `2` = recommended) |

Tuning values (`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`) live in `cfg/filter_profiles/<name>.yaml`. To author a
profile, copy `default.yaml` and edit the fields; absent fields fall back
to the node's compiled defaults.

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

Three services control the camera output without unloading the ROS node.
`pause` and `standby` take a `std_srvs/srv/SetBool` and control the color and
depth streams together; `hw_reset` takes a `std_srvs/srv/Empty` and power-cycles
the camera over USB.

```bash
# Stop publishing frames but keep the camera streaming on USB.
# Driver CPU drops to near zero and resume appears on the next frame.
# Use for short interruptions.
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# Release the USB pipe entirely. The ROS node stays alive and its
# topics stay advertised. Resume reopens the camera, which takes a few
# seconds depending on the model.
# Use to free USB bandwidth for another device.
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"

# Reset the camera over USB (re-enumerate the device). The node stops the
# streams, issues the reset, and the watchdog reconnects automatically —
# frames typically resume in ~12 s. Use to recover a wedged camera without
# restarting the node.
ros2 service call /G100P_1/hw_reset std_srvs/srv/Empty
```

While Standby is in effect, the auto-recovery watchdog (next section)
treats the silence as intentional rather than a disconnect.
The current control state is published on `/diagnostics` under the
`stream_state` key (`Active`, `Paused`, or `Standby`).

### Hot-plug Auto-recovery

The node runs a 1 Hz watchdog. Each stream is watched on its own: once a
stream has delivered a frame, **3 consecutive seconds** of silence on that
stream closes the device and enters a reconnect loop, polling the SDK every
2 s. Depth wedging in firmware while color keeps flowing is therefore
recovered. Before the first frame arrives the threshold is 10 s, which covers
slow modes such as R77 at 7 fps. When the camera comes back the streams resume
on the same topics — no `ros2 launch` restart required.

```
[ERROR] watchdog: depth stream silent for 3 s; declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

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
ros2 launch eys3d_camera dual_G62.launch.py
ros2 launch eys3d_camera dual_G100P.launch.py
ros2 launch eys3d_camera G100P_plus_R77.launch.py
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

Each model ships a URDF/xacro description with a 3D mesh
(`urdf/eys3d_<MODEL>.urdf.xacro` + `meshes/<MODEL>.dae`). `<name>_link`
sits at the depth start point — centered between the two imagers, on
the optical axis, Z' behind the camera front face (G100+ 6.75 mm,
R77 4.8 mm, G62 3.1 mm). Instantiate the macro under your robot's
existing frame with the physical mount pose:

```xml
<xacro:include filename="$(find eys3d_camera)/urdf/eys3d_G100P.urdf.xacro"/>
<xacro:eys3d_G100P name="G100P_1" parent="parent_link">
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</xacro:eys3d_G100P>
```

For a standalone preview (`name` defaults to `<MODEL>_1`, matching the
driver and the shipped rviz layouts):

```bash
ros2 launch eys3d_camera display_model.launch.py model:=G100P
```

Preview all three models side by side (no hardware needed):

```bash
ros2 launch eys3d_camera three_models.launch.py
```

Each description also carries the case mounting-hole frames
(`<name>_tripod_frame` and `<name>_back/bottom_screw*_frame` as
applicable, positions from the factory CAD) for aligning the model
with the mechanical design.

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
ros2 launch eys3d_camera G100P_composable.launch.py \
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
| `OK` | `streaming` | Every configured stream is delivering |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` is in effect |
| `OK` | `standby (USB pipe closed by operator)` | `standby` is in effect |
| `ERROR` | `no frames flowing on any configured stream` | Every configured stream is below half its expected rate |
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

Their summary is `streaming`, `input rate below 50% of expected` (WARN),
`not configured (D-only mode)`, or `standby`.

**`pointcloud`** — reprojection + post-processing counters:

| Key | Description |
|---|---|
| `compute_status` | `active` (a subscriber is pulling clouds), `idle (no /depth/points subscriber)`, `idle (never run ; no subscriber since start)`, or `(disconnected — see device task)` |
| `publish_fps` | Clouds published per second over the past second. Zero when no subscriber |
| `compute_avg_ms` | Mean compute time per cloud during the past second (present when `active`) |
| `compute_max_ms` | Longest compute time observed so far |
| `publish_total` | Cumulative number of clouds published |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | Cumulative invocation counters for each enabled post-processing stage |

**`thermal`**:

| Key | Description |
|---|---|
| `temperature_c` | On-die thermal sensor (°C). Present only when the module has the sensor and the read succeeds; otherwise the key is absent and the task summary carries the reason |

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

## Self-Calibration

Optional in-stream self-calibration re-aligns the stereo pair to recover depth
fill rate on a module whose calibration has drifted. It is built in by default
(CMake `EYS3D_WITH_SELFCAL`) and enabled per launch with `selfcal_enable:=true`.

```bash
ros2 launch eys3d_camera G100P.launch.py selfcal_enable:=true
```

Run it with the camera **streaming a depth mode and pointed at a normal,
textured scene at working distance** — the calibrator measures depth coverage,
so it needs valid depth to work from. One `selfcal/run` action runs a whole
session, with the bundled tuning applied automatically.

```bash
# Run one session (blocks until it converges, ~20-30 s, then a brief re-check;
# --feedback streams phase / progress). auto_commit_shift_px < 0 keeps the result
# live but never writes flash; >= 0 auto-commits a verified-better run once the cy
# shift reaches that many pixels (see the table below).
ros2 action send_goal /G100P_1/selfcal/run \
  eys3d_camera_interfaces/action/SelfCal \
  "{auto_commit_shift_px: 0.25}" --feedback

# Persist a kept result to flash (only needed when auto_commit did not fire).
# Check the response's `success` field — it is false if there is nothing kept.
ros2 service call /G100P_1/selfcal/commit std_srvs/srv/Trigger
```

The run resolves itself when the search finishes, on the calibrator's outcome
**plus a live A/B re-check** that measures depth fill-rate at the new vs. the
pre-run alignment on the same scene (so the verdict reflects the real before/after,
not a scene-dependent guess):

- **verified better** — a `SUCCESS` the re-check confirms is live
  (`applied: true`). If `auto_commit_shift_px` is set and `cy_shift_px` reaches
  it, the run **commits** to flash (`committed: true`); otherwise it is **kept
  live** but volatile — call `selfcal/commit` to keep it past a power-cycle.
- **already optimal** (`NO_CHANGE`) — nothing is changed, applied, or written.
  A normal, healthy result.
- **worse / unverifiable / failed** — a result the re-check finds worse or cannot
  confirm (e.g. the camera moved during the run), or an `INSUFFICIENT_INPUT` /
  `TIMEOUT` / `FAILED` outcome — the camera is **rolled back** to its pre-run
  alignment (`reverted: true`).

`cy_shift_px` is the measured vertical shift, read straight from the hardware, and
is the value the auto-commit gate uses. The step is fixed at **0.25 px** and the
correction is capped at **5.0 px**, so it is always one of
`0.25, 0.50, … 5.00` (or `0`).
`auto_commit_shift_px` is a threshold — it accepts any value, but only the step
boundaries change behaviour and a value between steps rounds up (`0.3` acts like
`0.5`):

| `auto_commit_shift_px` | Effect |
| --- | --- |
| `-1` (default) | never auto-commit — review, then commit manually. The value used when the goal omits the field |
| `0.25` | commit on any real move (≥ 1 step) — **recommended** |
| `0.50`, `0.75`, … up to `5.00` | require a larger shift |
| `> 5.0` | never fires (the shift cannot exceed the cap) |

### Action Feedback

Streamed continuously while the run is in progress, when the goal is sent with
`--feedback`:

| Field | Meaning |
| --- | --- |
| `phase` | `INITIAL_SEARCH` / `REFINEMENT` / `RECHECK` / `COMPLETED` |
| `progress` | search progress, `0.0`–`1.0` |
| `processed_frames` | depth frames fed to the calibrator so far |
| `valid_ratio_latest` | fill-rate of the latest frame (populated once a result exists) |

### Action Result

The goal aborts when the session could not run, could not reach an answer, or
could not undo a change it rejected; a deliberate revert succeeds. `outcome`
carries the detail either way.

Delivered once when the run resolves — the record of what the session measured
and what it left on the camera:

| Field | Meaning |
| --- | --- |
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT` (not enough valid depth in the scene) / `TIMEOUT` (did not converge in time) / `FAILED` |
| `cy_shift_px` | measured vertical cy shift in pixels — the auto-commit gate |
| `recheck_verdict` | A/B re-check: `improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `recheck_ratio_after` | fill-rate at the pre-run / converged alignment during the re-check |
| `applied` | a correction is live in the registers |
| `reverted` | rolled back to the pre-run calibration |
| `committed` | written to the user calibration slot |
| `message` | human-readable summary of the outcome |
| `correction_level`, `valid_ratio_first` / `_latest` / `_delta` | diagnostics only (never gate the commit) |

The whole session runs in-stream: depth keeps publishing and any post-processing
filters keep running — the run never restarts the stream. During the search the
depth quality visibly fluctuates as the calibrator probes alignments, then
settles. A session **cannot be interrupted** (cancels are rejected, and the
control setters plus `pause` / `standby` / `hw_reset` decline) — it is short and
auto-reverts if the result is worse, so there is nothing to interrupt.

`commit` is the only step that writes flash; the factory calibration is kept as
a backup and never overwritten. A kept-but-not-committed result lives only in
the camera's registers, so a power-cycle or `~/hw_reset` clears it back to the
stored calibration. This makes "calibrate live, never write flash" a safe
default workflow — run it whenever the robot is docked and still.

A couple of things worth knowing:

- **Repeated runs can stack.** Re-running `selfcal/run` before a commit or
  power-cycle builds on the last kept result, not on flash. Commit or
  power-cycle first for a clean baseline.
- **Not eligible or already running → rejected, not failed.** A module that
  does not support self-calibration, and a second concurrent run, are both
  refused outright — `ros2 action send_goal` reports "Goal was rejected" (no
  `Result`); check the node log for the reason.

**One session per process.** The calibrator binds the camera handle in a process
global, so cameras sharing a process — several drivers in one
`ComposableNodeContainer` — calibrate one at a time; a second `selfcal/run` is
rejected on the node log until the first resolves. Cameras launched as separate
processes are unaffected. Intra-process communication is otherwise orthogonal:
the calibrator takes its own copy of the depth frame, so `use_intra_process_comms`
delivery is unchanged during a run.

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
`/diagnostics` shows `input_dropped = 0`, the loss is on the DDS transport,
not in the driver.

Each image frame in the supported modes is larger than the kernel UDP socket
and IP fragment reassembly budget can absorb in one burst (an `rgb8` 1280×720
frame is ~2.7 MB; a point cloud up to ~11 MB), so the kernel-side buffer
overflows — whether because a fast x86 host emits the fragments faster than
the receiver drains them, or because a constrained ARM host drains them too
slowly.

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

