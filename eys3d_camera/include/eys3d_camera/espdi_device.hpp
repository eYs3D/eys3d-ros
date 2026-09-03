#ifndef EYS3D_CAMERA__ESPDI_DEVICE_HPP_
#define EYS3D_CAMERA__ESPDI_DEVICE_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "eys3d_camera/selfcal_manager.hpp"

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
    // USB port type this mode needs (2 = USB2.0, 3 = USB3.0; 0 = unspecified).
    // Opening the device is refused when it differs from the negotiated link.
    int mode_usb = 0;
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
    // Per-model constants from the video-mode catalogue header
    // (launch/video_modes/<MODEL>.yaml). The code layer keeps no
    // per-model tables; adding a camera means adding a YAML file.
    unsigned short expected_pid = 0;  // 0 = skip the open-time PID check
    bool mono = false;                // monochrome sensor pair (color is luma)
    int ir_default = 0;               // projector level for ir_value = -1
    int default_near_mm = 0;          // catalogue working range, used when
    int default_far_mm = 0;           // the matching depth_*_mm is -1

    // Working range in mm applied to the depth raster and the point
    // cloud. -1 on either side falls back to the catalogue default.
    int depth_near_mm = -1;
    int depth_far_mm = -1;
    // IR projector intensity applied before APC_OpenDevice2.
    // -1 = catalogue default; ≥0 = explicit value, clamped by FW IR-MAX.
    int ir_value = -1;

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
    //   2 = farthest_from_around
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
    // device (for example, "2-3:1.0" = bus 2, port 3, config 1, interface 0),
    // resolved via /sys/class/video4linux/videoN/device. Stable across reboots
    // and plug order. When both serial_number and usb_port are set, both must
    // match.
    std::string usb_port;

    // Self-calibration (selfk). Off by default. When true (and the feature is
    // built in), open() binds a SelfCalManager to the device handle so an
    // in-stream calibration session can be started later. selfcal_config_dir
    // holds the JSON tuning profiles loaded by start_selfcal().
    bool        selfcal_enable = false;
    std::string selfcal_config_dir;
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
    int frame_number = 0;
    uint64_t hw_timestamp_us = 0;
    int width = 0;
    int height = 0;
};

using ColorFrameCb = std::function<void(FrameBuffer&&)>;
using DepthFrameCb = std::function<void(FrameBuffer&&)>;
// Point-cloud callback. The buffer is `valid_points * point_step` bytes and
// its ownership transfers to the consumer. point_step is 12 for the XYZ-only
// layout (X, Y, Z float32) or 16 for XYZRGB (those three followed by a uint32
// RGB packed as 0x00RRGGBB).
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

    // Selects the device the given hints would open (serial / usb_port / PID
    // precedence, mirroring open()) and returns its negotiated USB link type
    // (2 = USB2.0, 3 = USB3.0), or nullopt if selection/query fails. Used to
    // resolve the signature default mode before the full open. Does not leave
    // a device open.
    std::optional<int> probe_usb_type(const DeviceConfig& cfg);

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
        std::array<double, 8>  D{};            // rational_polynomial: k1, k2, p1, p2, k3, k4, k5, k6 (CamDist*)
        std::array<double, 9>  R{};            // 3x3 rectification rotation (*RotaMat)
        std::array<double, 12> P{};            // 3x4 projection rectified (NewCamMat*)
    };
    struct Calibration {
        // Height, not width: side-by-side doubles the log's width, never its
        // height. K and D are at in_height, P at out_height.
        int in_height  = 0;                     // InImgHeight
        int out_height = 0;                     // OutImgHeight
        LensCalibration left;                   // CamMat1 / CamDist1 / LRotaMat / NewCamMat1
        LensCalibration right;                  // CamMat2 / CamDist2 / RRotaMat / NewCamMat2
        double baseline_mm = 0;                 // |TranMat[0]| from rect log
        bool valid = false;
    };
    Calibration calibration() const;

    // Scales a matrix from the raster it is stored at to the published one.
    // /depth/camera_info and the point-cloud reprojection must agree here.
    static double raster_scale(int raster_height, int published_height) {
        return (raster_height > 0 && published_height > 0)
            ? static_cast<double>(published_height) / raster_height
            : 1.0;
    }

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
    // set_ir_value:
    //   value > 0  → enable projector and set raw level (clamped to ir_max)
    //   value == 0 → disable projector
    //   value < 0  → use the per-PID default
    bool set_ir_value(int value);

    // Thread-safe; takes effect on the next depth frame. Enabling clears the
    // per-pixel persistence history.
    //   alpha        0.0 .. 1.0
    //   delta        >= 1; raw disparity units with spatial_filter on, mm off
    //   persistence  0 .. 8 (TemporalFilterParams in temporal_filter.hpp)
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
        int  ir_value         = -1;   // -1 = read failed
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

    //   Active   fetch threads running, frames decoded and published.
    //   Paused   USB still drained, every frame dropped before decode.
    //   Standby  APC_CloseDevice releases the V4L2 fd; fetch threads joined.
    //            The SDK handle, calibration and ZD table are kept.
    //
    // Both controls move the colour + depth pair together: interleave modes
    // need both halves of the stream active.
    enum class StreamState : uint8_t { Active = 0, Paused = 1, Standby = 2 };

    // Called while Standby, it records the desired post-standby state; the
    // transition happens on the next standby(false). Shares lifecycle_mtx with
    // standby(). True on success or when the state already matches.
    bool pause(bool on);

    // standby(true)  :: stops fetch threads + APC_CloseDevice.
    // standby(false) :: APC_OpenDevice2 + spawn fetch threads, lands in
    //                   Active (or Paused if pause() was called while
    //                   Standby was in effect).
    // True on success or when the state already matches. False with the
    // device not open, either direction, and while a self-calibration session
    // holds it. A failed reopen leaves it closed for the reconnect loop.
    bool standby(bool on);

    StreamState stream_state() const;

    // Firmware USB self-reset: writes the eSP876 register sequence whose tail
    // write drops the USB link, so the host re-enumerates the device. Issuable
    // while Streaming, Paused or in Standby. The detach-triggering writes are
    // acknowledged unreliably, so individual register results are ignored.
    // False only when the device is not open.
    bool reset_usb();

    // Atomic per-stream counters maintained by the fetch + pc threads,
    // readable from any thread; fps comes from diffing them across a fixed
    // wall window.
    //   *_input_total    frames received from the SDK; always increments
    //                    while the camera streams.
    //   *_publish_total  frames emitted through the publish callback, gated
    //                    by subscriber state.
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
        // /depth/points subscriber). pc_publish_total == pc_compute_count.
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

    // --- Self-calibration (selfk) ---------------------------------------
    // Bound only when DeviceConfig::selfcal_enable was set and open() succeeded
    // in creating a selfk context on the device handle. Every method is a safe
    // no-op otherwise (selfcal_available() returns false).

    // True once a selfk context is bound to the open device.
    bool selfcal_available() const;
    // True while a session is running; callers use it to decline stream-control
    // actions (pause / standby / hw_reset) that would interrupt the session.
    bool selfcal_active() const;
    // Load <selfcal_config_dir>/<profile>.json and start an in-stream session.
    // The device must already be streaming a depth mode. Returns false on error
    // or when a session is already running.
    bool start_selfcal(const std::string& profile);
    // Stop the running session; the SDK restores the last stable probe value.
    bool stop_selfcal();
    // End the session and roll cy back to the snapshot taken at start (used when
    // the result is worse / not wanted).
    bool revert_selfcal();
    // End the session, keeping the converged value live (not committed).
    void keep_selfcal();
    // Commit the converged result to the G2 flash bank.
    bool commit_selfcal();
    // Snapshot of the current session state, for the diagnostics surface.
    SelfCalManager::Status selfcal_status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Spawn the color / depth / pc threads. Called from start() and from
    // standby(false) after a successful reopen. Always spawns all three;
    // the Paused state is handled by gating at the top of each iteration
    // rather than skipping the spawn.
    void spawn_fetch_threads_();

    // Guard for the set_*_gate() setters: the gates are lock-free reads on the
    // fetch threads, so they may only be assigned while stopped. Returns false
    // (and logs) if called while streaming. `which` names the caller for the
    // log line.
    bool gate_setter_allowed(const char* which);

    // Snapshot / restore the cy_R register around a self-calibration session so
    // a worse or abandoned run can be rolled back.
    bool snapshot_cy();
    bool restore_cy();

    // Read / write the two cy_R register bytes (both take sdk_mtx). Exposed to
    // the SelfCalManager A/B re-check via callbacks so it can toggle between the
    // converged and pre-session cy on the live stream.
    bool get_cy_regs(unsigned short& lo, unsigned short& hi);
    bool set_cy_regs(unsigned short lo, unsigned short hi);

    // Returns true (and logs) when a self-calibration session currently owns
    // the control channel, so a control-register setter must decline rather
    // than race the selfk worker's per-frame register writes. `which` names the
    // caller for the log line.
    bool selfcal_reject(const char* which);
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__ESPDI_DEVICE_HPP_
