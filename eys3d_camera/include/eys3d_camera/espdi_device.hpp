#ifndef EYS3D_CAMERA__ESPDI_DEVICE_HPP_
#define EYS3D_CAMERA__ESPDI_DEVICE_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace eys3d_camera {

// Hardware configuration applied to `APC_OpenDevice2` and the SDK fetch
// loops. All fields are populated by CameraNode from the active video-mode
// catalogue and any matching launch overrides.
struct DeviceConfig {
    // Camera model token from the launch parameter (e.g. "G100P", "R77",
    // "G62"). Drives PID validation: the driver refuses to open a device
    // whose USB PID does not match the expected PID for this model, and
    // when multiple cameras are connected the matching PID wins over
    // index-0 fallback.
    std::string model;
    int color_width = 1280;
    int color_height = 720;
    int color_format = 0;        // 0 = YUYV, 1 = MJPEG
    int depth_width = 1280;
    int depth_height = 720;
    int depth_data_type = 18;    // see APC_DEPTH_DATA_* in eSPDI_def.h
    // Index into the per-resolution rectify-log and ZD-table set stored in
    // FW. Follows the color-resolution group from the datasheet (1280x720→0,
    // 640x480→1, 640x360→2, 480x270→3, 424x240→4). Must match the active
    // color resolution so camera_info K/R/P and depth→XYZ projection use
    // the right intrinsics.
    int zd_index   = 0;
    int framerate = 60;          // physical fps; interleave halves per stream
    bool interleave = true;
    // When true *and* the wire format is YUYV, the color fetch thread
    // decodes the wide L|R raster into two half-width rgb8 buffers in
    // one pass (no wide intermediate, no row-by-row memcpy split).
    // Ignored for MJPEG modes — those still decode to a single wide
    // buffer that camera_node slices downstream.
    bool split_color = false;
    // PointCloud Z clip range in mm. -1 on either side uses the per-model
    // default (resolved at open from depth_range_for_pid).
    int depth_minimum_mm = -1;
    int depth_maximum_mm = -1;
    // IR projector intensity applied before APC_OpenDevice2.
    // -1 = per-PID default; ≥0 = explicit value, clamped by FW IR-MAX.
    int ir_intensity = -1;

    // Disparity-domain spatial filter. When enabled, the depth stream
    // opens in 11-bit disparity mode (depth_data_type + 2); the
    // pc_thread runs a four-direction edge-aware IIR on the raw
    // disparity and converts to Z via the firmware ZD table before
    // reprojection. Disabled by default, in which case the 14-bit-mm
    // depth pipeline runs unchanged.
    bool   spatial_filter_enabled   = false;
    double spatial_filter_alpha     = 0.5;   // 0..1
    int    spatial_filter_delta     = 20;    // raw disparity units
    int    spatial_filter_magnitude = 2;     // 1..5
    int    spatial_filter_holes_fill  = 0;     // max consecutive holes bridged per direction; 0 = no bridging

    // Temporal filter, runtime-adjustable through
    // EspdiDevice::set_temporal_filter. Runs on D11 disparity when
    // chained after the spatial IIR (between IIR and ZD lookup), or
    // directly on the FW Z14 mm raster when spatial_filter is off.
    // `delta` is stored raw; the driver interprets it as raw disparity
    // units (shifted to Q4 inside) for the D11 path and as mm for the
    // Z14 path.
    bool   temporal_filter_enabled    = false;
    double temporal_filter_alpha      = 0.4;   // 0..1
    int    temporal_filter_delta      = 20;    // disparity units in D11 path, mm in Z14 path
    int    temporal_filter_persistence = 3;    // 0..8 persistence index

    // Z-domain hole filling applied to the final Z14 mm raster before
    // depth_image publish and reprojection. Operates on the post-ZD-
    // lookup buffer when spatial_filter is on, and on the FW depth
    // directly otherwise. Launch-time only.
    //   0 = off
    //   1 = fill_from_left
    //   2 = farthest_from_around (recommended starting mode)
    //   3 = nearest_from_around
    int    hole_filling = 0;

    // When true the pc_thread emits XYZRGB (point_step=16); color_fetch
    // snapshots its decoded rgb8 buffer for the projector to sample.
    // Falls back to XYZ-only on depth-only modes.
    bool   colored_pointcloud = false;
    // Optional substring match against the camera serial number. When empty,
    // the first detected device is used.
    std::string serial_number;
    // Optional substring match against the USB topology path of the V4L2
    // device (for example, "2-3:1.0" = bus 2, port 3, config 1, interface 0).
    // The path is stable across reboots and plug order, making it the
    // preferred way to pin a camera to a fixed physical port in production
    // deployments. Resolved via /sys/class/video4linux/videoN/device.
    // Ignored when serial_number already matches.
    std::string usb_port;
};

// Frame payload moved (not copied) from the SDK fetch thread into the
// publish path, allowing the buffer to be handed directly to
// sensor_msgs::Image::data without an intermediate copy.
//
// `data_right` is non-empty only for wide YUYV modes when split_color
// is active — in that case the device layer has already split the wide
// raster into two half-width rgb8 buffers, and `data` carries the left
// side while `data_right` carries the right side. `width` and `height`
// describe the per-side image (half the wire width). For every other
// mode `data_right` is empty and `data` holds the full image.
struct FrameBuffer {
    std::vector<uint8_t> data;
    std::vector<uint8_t> data_right;
    int serial_number = 0;
    uint64_t hw_timestamp_us = 0;
    int width = 0;
    int height = 0;
};

using ColorFrameCb = std::function<void(FrameBuffer&&)>;
using DepthFrameCb = std::function<void(FrameBuffer&&)>;
// Point-cloud callback. The point-cloud thread hands the consumer an
// owning byte buffer sized to `valid_points * point_step`, with
// `point_step` set to 12 for the XYZ-only layout (X, Y, Z float32) or
// 16 for the XYZRGB layout (the same three coordinates followed by a
// uint32 RGB packed as 0x00RRGGBB). Buffer ownership is transferred
// via std::move; the consumer typically moves it again into
// sensor_msgs::msg::PointCloud2::data with no further copy.
using PointCloudCb = std::function<void(
    std::vector<uint8_t>&& xyz_bytes,
    uint32_t valid_points,
    uint32_t point_step,
    uint64_t hw_timestamp_us)>;
// Predicate consulted by the point-cloud thread before each reprojection.
// Returning false suppresses computation for the next depth notification.
// CameraNode uses this to skip work when no client is subscribed to the
// point-cloud topic.
using PointCloudGate = std::function<bool()>;

// Predicate consulted by the color and depth fetch threads before decoding
// and dispatching each frame. Returning false suppresses the per-frame
// decode / memcpy / publish-callback work for that stream; the V4L2 DQBUF
// still runs so the driver-side buffer queue never stalls. Used by
// CameraNode to skip work when no client is subscribed to the corresponding
// image topic.
using FrameStreamGate = std::function<bool()>;

class EspdiDevice {
public:
    EspdiDevice();
    ~EspdiDevice();

    EspdiDevice(const EspdiDevice&) = delete;
    EspdiDevice& operator=(const EspdiDevice&) = delete;

    bool open(const DeviceConfig& cfg);
    void close();

    void start(ColorFrameCb on_color, DepthFrameCb on_depth, PointCloudCb on_pc);
    void stop();

    // Set/clear the PC computation gate. Default: gate is unset → always run.
    void set_pc_gate(PointCloudGate gate);

    // Set/clear per-stream gates for color and depth. When a gate
    // returns false, the matching fetch thread still drains the V4L2
    // buffer but skips decode, the latest-frame snapshot, and the
    // publish callback. Default: gate unset → always run.
    void set_color_gate(FrameStreamGate gate);
    void set_depth_gate(FrameStreamGate gate);

    // Parsed from APC_GetRectifyMatLogData (eSPCtrl_RectLogData). K, R, P
    // follow sensor_msgs/CameraInfo row-major convention.
    struct LensCalibration {
        std::array<double, 9>  K{};            // 3x3 raw camera matrix (CamMat*)
        std::array<double, 5>  D{};            // plumb_bob: k1, k2, p1, p2, k3 (CamDist*)
        std::array<double, 9>  R{};            // 3x3 rectification rotation (*RotaMat)
        std::array<double, 12> P{};            // 3x4 projection rectified (NewCamMat*)
    };
    struct Calibration {
        int width = 0;
        int height = 0;
        LensCalibration left;                   // CamMat1 / CamDist1 / LRotaMat / NewCamMat1
        LensCalibration right;                  // CamMat2 / CamDist2 / RRotaMat / NewCamMat2
        double baseline_mm = 0;                 // |TranMat[0]| from rect log
        bool valid = false;
    };
    Calibration calibration() const;

    // Applies per-chip register tuning from
    // <cfg_dir>/<model>_DM_Quality_Register_Setting.cfg in a detached worker.
    // Must be called after the first depth frame has been received so the
    // pipeline is stable; the worker performs its own retries and does not
    // block the calling thread.
    void apply_dm_quality_register_setting_async(const std::string& cfg_dir);

    // Runtime image controls. Safe to call after start(). Each returns false
    // when the device is closed or the SDK call fails; failures are logged
    // through rclcpp at the warn level.
    //
    // set_ir_intensity:
    //   value > 0  → enable projector and set raw level (clamped to ir_max)
    //   value == 0 → disable projector
    //   value < 0  → use the per-PID default
    bool set_ir_intensity(int value);

    // Runtime control for the temporal filter. Safe to call from any
    // thread while the pipeline is streaming; the new settings take
    // effect on the next depth frame. A disabled-to-enabled
    // transition clears the per-pixel persistence history. The filter
    // runs on D11 disparity when chained after the spatial IIR and on
    // the Z14 mm raster otherwise; `delta` is interpreted in the
    // active domain unit.
    //
    // Argument ranges:
    //   alpha          0.0 .. 1.0
    //   delta          raw disparity units when spatial_filter is on,
    //                  mm otherwise (≥ 1)
    //   persistence    0 .. 8 (see TemporalFilterParams in temporal_filter.hpp)
    bool set_temporal_filter(bool enabled, double alpha,
                             int delta, int persistence);

    // On-die thermal sensor reading. supported=false on models without
    // the sensor; read_ok=false on a transient USB read failure.
    struct TemperatureReading {
        bool  supported = false;
        bool  read_ok   = false;
        float celsius   = 0.0f;
    };
    TemperatureReading read_temperature() const;
    bool set_auto_exposure(bool enable);
    // Manual exposure step via UVC CT_EXPOSURE_TIME_ABSOLUTE. The value is a
    // signed log-step (negative = darker, positive = brighter). The setting
    // takes effect only when auto-exposure is set to manual.
    bool set_exposure_time_step(int step);
    bool set_auto_white_balance(bool enable);
    // Power-line anti-flicker. UVC PU_POWER_LINE_FREQUENCY_CTRL values:
    //   0 = disabled, 1 = 50 Hz, 2 = 60 Hz, 3 = auto.
    bool set_power_line_frequency(int mode);

    // Read current FW state without writing. Used by CameraNode to populate
    // ROS param defaults from the camera's boot configuration so the
    // operator's pre-configured values are preserved across node restarts.
    struct RuntimeState {
        int  ir_intensity         = -1;   // -1 = read failed
        bool ir_read_ok           = false;
        bool auto_exposure        = true;
        bool auto_exposure_read_ok = false;
        int  exposure_time_step   = 0;
        bool exposure_read_ok     = false;
        bool auto_white_balance   = true;
        bool awb_read_ok          = false;
        int  power_line_frequency = 0;
        bool plf_read_ok          = false;
    };
    RuntimeState read_runtime_state() const;

    // Runtime stream control. There are two distinct cost / latency tiers
    // here, exposed as separate methods rather than a single SetBool:
    //
    //   Active   — normal streaming. SDK fetch threads running, frames
    //              decoded and published.
    //   Paused   — SDK + USB still active, fetch threads still drain the
    //              USB buffer, but every frame is dropped at the top of
    //              the iteration before decode / filter / publish. Almost
    //              zero CPU; resumes on the next frame (~33 ms @ 30 fps).
    //   Standby  — APC_CloseDevice releases the V4L2 fd. No USB traffic,
    //              fetch threads joined, but the SDK handle (and the
    //              cached calibration / ZD table) is kept so the next
    //              standby(false) restarts in ~200-400 ms without redoing
    //              APC_Init / GetRectifyMatLogData.
    //
    // Both controls toggle the colour + depth pair together — per-stream
    // toggling is not supported because interleave modes (G100+ mode 1)
    // require both halves of the stream to be active and the use cases
    // never call for splitting them.
    enum class StreamState : uint8_t { Active = 0, Paused = 1, Standby = 2 };

    // pause(true)  :: Active -> Paused  (no SDK work; resume next frame)
    // pause(false) :: Paused -> Active
    // Calling pause() while Standby is active records the desired
    // post-standby state; the actual transition happens when the next
    // standby(false) succeeds. pause() shares lifecycle_mtx with
    // standby() so the two cannot race; the lock is sub-microsecond
    // uncontended.
    // Returns true on success or when the requested state already
    // matches; never returns false in the current implementation.
    bool pause(bool on);

    // standby(true)  :: stops fetch threads + APC_CloseDevice.
    // standby(false) :: APC_OpenDevice2 + spawn fetch threads, lands in
    //                   Active (or Paused if pause() was called while
    //                   Standby was in effect).
    // Returns true on success or when the requested state already
    // matches. Returns false only if standby(false)'s reopen fails — in
    // that case the device is left closed and the node should be
    // restarted.
    bool standby(bool on);

    StreamState stream_state() const;

    // Atomic per-stream counters maintained by the fetch + pc threads. Read
    // any time from any thread; consumers (CameraNode's diagnostics timer)
    // compute fps by diffing across a fixed wall window.
    //
    // Two distinct rates are maintained per stream:
    //   - *_input_total   : count of frames received from the SDK
    //                       (always increments while the camera streams).
    //   - *_publish_total : count of frames actually emitted through the
    //                       publish callback (gated by subscriber state).
    //
    // input - publish reveals "subscribers present but driver behind",
    // while publish = 0 with input > 0 simply means nobody is listening
    // and the gate is suppressing the decode + dispatch work.
    struct Stats {
        // Per-stream input rates (SDK → driver).
        uint64_t color_input_total    = 0;
        uint64_t depth_input_total    = 0;
        // Frames the USB / SDK layer lost before reaching the publisher
        // (detected as gaps in the firmware's serial-number sequence).
        uint64_t color_input_dropped  = 0;
        uint64_t depth_input_dropped  = 0;
        // Per-stream publish rates (driver → ROS topic).
        uint64_t color_publish_total  = 0;
        uint64_t depth_publish_total  = 0;
        // Color decode timing (tjDecompress2 for MJPEG, NEON / scalar
        // YUYV→RGB conversion). Aggregated per published frame.
        uint64_t color_decode_sum_us  = 0;
        uint64_t color_decode_max_us  = 0;
        // Point-cloud reprojection timing (computed only when there is a
        // /pointcloud subscriber). pc_publish_total == pc_compute_count.
        uint64_t pc_publish_total     = 0;
        uint64_t pc_compute_sum_us    = 0;   // for avg = sum / count
        uint64_t pc_compute_max_us    = 0;
        // Post-processing filter execution counters. Each ticks once
        // per kernel invocation in pc_thread; stays at zero when the
        // corresponding filter is disabled.
        uint64_t spatial_filter_total = 0;
        uint64_t temporal_filter_total = 0;
        uint64_t hole_fill_total      = 0;
    };
    Stats stats() const;

    // Resolved at open(); empty until the SDK enumerates a device.
    std::string serial_number() const;
    std::string usb_port() const;
    int actual_fps() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Spawn the color / depth / pc threads. Called from start() and from
    // standby(false) after a successful reopen. Always spawns all three;
    // the Paused state is handled by gating at the top of each iteration
    // rather than skipping the spawn.
    void spawn_fetch_threads_();
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__ESPDI_DEVICE_HPP_
