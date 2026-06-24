#include "eys3d_camera/espdi_device.hpp"
#include "register_settings.hpp"
#include "spatial_filter.hpp"
#include "hole_filling.hpp"
#include "simd_kernels.hpp"
#include "temporal_filter.hpp"
#include "zd_lookup.hpp"

#include <csignal>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <cstring>
#include <memory>
#include <mutex>
#include <regex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eSPDI.h"
#include "eSPDI_def.h"
#include "turbojpeg.h"   // libjpeg-turbo 2.0.4 re-exported by libeSPDI

namespace eys3d_camera {

namespace {
// IR projector FW register. Read once at open() to clamp user-supplied
// intensity against the firmware-reported ceiling.
constexpr uint16_t kFwRegIrMax = 0xE2;

// Scope APC_Init's SIGINT handler to open(). APC_Init installs an
// exit()-on-delivery handler that would otherwise pre-empt
// rclcpp::shutdown() and skip every C++ destructor; the guard
// captures whatever handler is in place on entry and restores it on
// exit so Ctrl-C drives rclcpp::spin() to return normally. SIGTERM
// is carried for parity.
class SignalHandlerGuard {
public:
    SignalHandlerGuard() {
        sigaction(SIGINT,  nullptr, &sigint_);
        sigaction(SIGTERM, nullptr, &sigterm_);
    }
    ~SignalHandlerGuard() {
        sigaction(SIGINT,  &sigint_,  nullptr);
        sigaction(SIGTERM, &sigterm_, nullptr);
    }
    SignalHandlerGuard(const SignalHandlerGuard&) = delete;
    SignalHandlerGuard& operator=(const SignalHandlerGuard&) = delete;
private:
    struct sigaction sigint_{};
    struct sigaction sigterm_{};
};

// Per-PID IR default level applied at open(). G62 ships with a different
// IR LED part whose FW IR-MAX defaults to 96, so its "comfortable" level
// is higher; G100+/G100+i/R77 use level 3. The FW range is read back at
// open and logged.
int default_ir_level_for_pid(unsigned short pid) {
    switch (pid) {
    case APC_PID_8081:  return 60;   // G62
    case APC_PID_80362: /* G100+ */  [[fallthrough]];
    case APC_PID_IRIS:  /* G100+i */ [[fallthrough]];
    case APC_PID_8072:  /* R77 */    [[fallthrough]];
    default:            return 3;
    }
}

// Per-PID working depth range used as the default PointCloud clip. Caller's
// launch parameter > 0 overrides this.
struct DepthRange {
    int near_mm;
    int far_mm;
};

DepthRange depth_range_for_pid(unsigned short pid) {
    switch (pid) {
    case APC_PID_80362: return {250, 1900};   // G100+
    case APC_PID_IRIS:  return {250, 1900};   // G100+i
    case APC_PID_8072:  return {200, 1500};   // R77
    case APC_PID_8081:  return {100, 1500};   // G62
    default:            return {250, 1900};
    }
}

// Expected USB PID for the camera model token (the launch `model` parameter).
// Returns 0 for unknown models — the open path treats 0 as "skip the PID
// check" so untargeted models still work, but the supported lineup must hit
// one of the cases below.
unsigned short expected_pid_for_model(const std::string& model) {
    if (model == "G100P")  return APC_PID_80362;   // 0x0181
    if (model == "G100Pi") return APC_PID_IRIS;    // 0x0184
    if (model == "R77")    return APC_PID_8072;    // 0x0180
    if (model == "G62")    return APC_PID_8081;    // 0x0183
    return 0;
}

// Backoff applied on APC_DEVICE_TIMEOUT to avoid busy-spinning.
constexpr int kTimeoutBackoffUs = 100;

// Window after start() during which the per-stream subscriber gates are
// bypassed. DDS discovery can take several hundred milliseconds; this
// grace period lets late subscribers receive the initial frames.
constexpr int kGatePassThroughMs = 3000;

// Cached logger handle. rclcpp::get_logger() hashes the name and
// allocates an internal shared_ptr on every call, which the THROTTLE
// macros invoke unconditionally; at 30 fps × three fetch loops this
// is measurable. The function-local static is constructed once on first
// use and returned by reference thereafter.
const rclcpp::Logger& logger() {
    static const rclcpp::Logger kLogger = rclcpp::get_logger("EspdiDevice");
    return kLogger;
}

// One-shot stdout marker emitted on the first frame from either fetch
// thread so launch tooling can detect pipeline readiness. The marker
// and the follow-up tip are emitted in a single std::cout write so
// concurrent wakeups from the two fetch threads cannot interleave on
// stdout.
void emit_ready_marker() {
    std::cout
        << "EYS3D_CAMERA_READY\n"
           "[eys3d_camera] streaming. To see full RCLCPP logs "
           "append `log:=all` to the launch command "
           "(per-model launches default to `log:=sdk`)."
        << std::endl;
    std::cout.flush();
}

// Shared clock for RCLCPP_*_THROTTLE in the hot fetch loops. A fresh
// `rclcpp::Clock::make_shared()` would allocate a shared_ptr on every
// macro expansion (the THROTTLE macro evaluates its clock argument
// unconditionally to decide whether to emit) — at 30+ fps across three
// fetch loops this becomes measurable overhead. A single static instance
// avoids the per-iteration allocation.
rclcpp::Clock& throttle_clock() {
    static rclcpp::Clock c{RCL_STEADY_TIME};
    return c;
}

// Raw capture buffer fed to APC_GetColorImageWithTimestamp. YUYV is
// 2 bytes/pixel; MJPEG worst-case (JPEG bytes from the SDK) is bounded
// by 2 bytes/pixel too, so a single sizing covers both wire formats.
size_t color_raw_buffer_bytes(int w, int h) {
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
}

// Final published image is always rgb8 (3 bytes/pixel) for cross-vendor
// portability — RViz, image_pipeline, cv_bridge, depth_image_proc and
// the rest of the perception stack all consume rgb8 natively. Grayscale
// sensors (G62) and grayscale-ISP outputs (R77) flow through the same
// path: turbojpeg replicates Y → R=G=B at decode time, and the YUYV
// converter falls out to R=G=B=Y when chroma is neutral (U=V=128).
size_t color_rgb8_bytes(int w, int h) {
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
}

// YUYV → rgb8 conversion lives in simd_kernels.{hpp,cpp}. AArch64 picks
// the NEON intrinsic kernel; x86_64 / other archs use the scalar +
// OpenMP fallback. Both produce identical bytes (BT.601 limited range,
// 8.8 fixed point, round-to-nearest via vqrshrun).
using simd::yuyv_to_rgb8;


// Resolve "/dev/videoN" to its USB topology path ("2-3:1.0") via
// /sys/class/video4linux. Returns empty string on non-USB devices or
// sysfs failures. Stable across reboots; matches udev and lsusb output.
std::string resolve_usb_port(const std::string& v4l2_path) {
    const std::string dev_prefix = "/dev/video";
    if (v4l2_path.compare(0, dev_prefix.size(), dev_prefix) != 0) return {};
    const std::string vname = v4l2_path.substr(5);  // "video2"
    const std::string sysfs_link = "/sys/class/video4linux/" + vname + "/device";
    std::error_code ec;
    auto real = std::filesystem::canonical(sysfs_link, ec);
    if (ec) return {};

    // Iterate sysfs path components from deepest to root; return the first that
    // matches the USB interface pattern. Pattern allows hubs ("2-1.4.2:1.0").
    static const std::regex kUsbIfacePattern(R"(^\d+-\d+(?:\.\d+)*:\d+\.\d+$)");
    static const std::regex kUsbDevicePattern(R"(^\d+-\d+(?:\.\d+)*$)");
    // Prefer the more specific :config.interface form, fall back to the device
    // form. Two passes keep priority deterministic.
    for (auto it = real.end(); it != real.begin(); ) {
        --it;
        const std::string s = it->string();
        if (std::regex_match(s, kUsbIfacePattern)) return s;
    }
    for (auto it = real.end(); it != real.begin(); ) {
        --it;
        const std::string s = it->string();
        if (std::regex_match(s, kUsbDevicePattern)) return s;
    }
    return {};
}

// Depth buffer size in bytes. Most depth data types pack two bytes per pixel.
// 8-bit raw variants double the row width per SDK convention.
size_t depth_buffer_bytes(int w, int h, int depth_data_type) {
    const int base = depth_data_type % APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET;
    if (base == APC_DEPTH_DATA_8_BITS || base == APC_DEPTH_DATA_8_BITS_RAW) {
        return static_cast<size_t>(w) * 2 * static_cast<size_t>(h) * 2;
    }
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
}
}  // namespace

// Shared depth snapshot consumed by the point-cloud thread. The depth
// fetch thread copies the raw payload here after each parity-filtered
// frame; the point-cloud thread waits on the condition variable, samples
// the snapshot under the mutex, then releases the lock before
// reprojection.
struct LatestDepth {
    std::mutex mtx;
    std::condition_variable cv;
    // Reference-counted view of the most recent depth frame. The depth
    // fetch thread publishes a fresh shared_ptr per frame under the
    // mutex; the point-cloud thread takes a copy of the shared_ptr
    // (ref-count bump, no memcpy) and reads directly from the buffer
    // without holding the lock for the duration of the reprojection.
    std::shared_ptr<const std::vector<uint8_t>> depth;
    uint64_t depth_ts_us = 0;
    bool depth_pending = false;
};

// Shared_ptr snapshot of the most recently decoded color frame used by
// pc_thread during XYZRGB projection. Its own mutex avoids contention
// with the depth notification path.
struct LatestColor {
    std::mutex mtx;
    std::shared_ptr<const std::vector<uint8_t>> rgb;
    int      w = 0;
    int      h = 0;
    uint64_t ts_us = 0;
};

struct EspdiDevice::Impl {
    DeviceConfig cfg;
    Calibration calib;
    eSPCtrl_RectLogData cached_rect{};
    bool cached_rect_valid = false;
    float max_near_mm = 0.0f;
    float max_far_mm  = 0.0f;

    // Spatial filter state resolved at open(); ZD table cached at the
    // same point. pc_q4_buf and pc_mm_buf are sized once at pc_thread
    // start and reused per frame.
    bool                  spatial_filter_enabled = false;
    SpatialFilterParams spatial_params{};
    ZdTable               zd_table;
    std::vector<uint16_t> pc_q4_buf;
    std::vector<uint16_t> pc_mm_buf;

    // Temporal filter state. temporal_enabled is read by pc_thread on
    // every frame; temporal_params_pending and temporal_reset_pending
    // are written by set_temporal_filter() and consumed at the top of
    // each frame under temporal_mtx. temporal_state is owned by
    // pc_thread and never touched from outside.
    std::atomic<bool>     temporal_enabled{false};
    std::mutex            temporal_mtx;
    TemporalFilterParams  temporal_params_pending;
    bool                  temporal_reset_pending = false;
    TemporalState         temporal_state;

    // Z-domain hole filling state. Mode is launch-time and never
    // changes after open(). hole_fill_scratch is the frozen-input copy
    // used by the around modes; left empty for off and fill_from_left.
    HoleFillMode          hole_fill_mode = HoleFillMode::kOff;
    std::vector<uint16_t> hole_fill_scratch;

    void* handle = nullptr;
    DEVSELINFO sel{};
    DEVINFORMATION dev_info{};
    // Diagnostics + service reads run on the rclcpp executor; lifecycle
    // writes run on the watchdog timer or a service callback. Atomic to
    // avoid torn reads of these scalars on a multi-threaded executor.
    std::atomic<int> actual_fps{0};

    int ir_max_fw = 0;
    bool ir_range_valid = false;
    int ir_default_level = 3;
    LatestDepth latest;
    LatestColor latest_color;
    bool colored_pointcloud = false;
    // Precomputed per-pixel byte offsets into the color buffer so the
    // reprojection inner loop avoids any multiplies / divides per pixel.
    // cu_byte_off[u] = color_u(u) * 3; cv_byte_off[v] = color_v(v) * cw * 3.
    // Both are sized to the depth raster's W / H.
    std::vector<int32_t> colored_pointcloud_cu_byte_off;
    std::vector<int32_t> colored_pointcloud_cv_byte_off;

    std::thread color_fetch;
    std::thread depth_fetch;
    std::thread pc_thread;
    // Tracked register-tuning worker spawned by
    // apply_dm_quality_register_setting_async(). Joined in stop() so a
    // device close while the worker is mid-register-write cannot dereference
    // a released SDK handle.
    std::thread dm_quality_worker;
    std::atomic<bool> dm_quality_worker_running{false};

    // Serialises stop() and standby() so a service-thread close/reopen
    // cannot race with destruction. Also held by open() and by
    // const getters that read mutable string identity (serial_number,
    // usb_port). Mutable so const accessors can acquire it.
    mutable std::mutex lifecycle_mtx;

    // Serialises UVC control / sensor-register access (read_temperature,
    // CT/PU getters and setters) against itself. Frame DQBUF goes through
    // a separate V4L2 fd path and is not covered by this mutex.
    std::mutex sdk_mtx;

    std::atomic<bool> running{false};
    // Runtime stream control. Active = normal; Paused = SDK threads still
    // running but every callback dispatch drops the frame at the top of the
    // iteration (CPU near zero, resume on next frame); Standby = USB pipe
    // closed via APC_CloseDevice and fetch threads joined (zero USB traffic,
    // resume in ~200-400 ms). Both controls toggle the colour + depth pair
    // together — per-stream toggling is intentionally not supported.
    //
    // streams_present mirrors which of (color, depth) the *current* open()
    // configuration is delivering. Set once by open() based on the supplied
    // callbacks; never modified by pause / standby. The fetch threads read
    // these to decide whether to skip work (e.g. a D-only mode has no
    // color callback, so the color thread is never spawned).
    std::atomic<EspdiDevice::StreamState> stream_state{
        EspdiDevice::StreamState::Active};
    bool color_stream_present = true;
    bool depth_stream_present = true;
    // Remembers whether the caller was in Paused when standby(true) was
    // entered, OR whether pause() was called while standby was active.
    // standby(false) reads this to land in the right post-resume state.
    std::atomic<bool> pause_pending{false};

    // Health counters (atomic, single-writer / single-reader race-free).
    // Read by CameraNode's 1 Hz diagnostics timer; reset only at open(),
    // never touched at runtime so cumulative since open is well-defined.
    //
    // input_total ticks once per successful SDK frame (camera-side rate).
    // publish_total ticks once per frame that survives the subscriber
    // gate and reaches the publish callback (topic-side rate).
    std::atomic<uint64_t> color_input_total{0};
    std::atomic<uint64_t> depth_input_total{0};
    std::atomic<uint64_t> color_publish_total{0};
    std::atomic<uint64_t> depth_publish_total{0};
    // Frames lost in the USB / SDK layer (detected as a forward gap in
    // the FW serial-number sequence after the interleave parity filter).
    std::atomic<uint64_t> color_input_dropped{0};
    std::atomic<uint64_t> depth_input_dropped{0};
    // Last accepted SN per stream. -1 = "first frame after spawn".
    std::atomic<int> last_color_sn{-1};
    std::atomic<int> last_depth_sn{-1};
    // Color decode timing; only ticks on frames that are actually
    // decoded (a subscriber is present, or the startup grace window
    // is still active).
    std::atomic<uint64_t> color_decode_sum_us{0};
    std::atomic<uint64_t> color_decode_max_us{0};
    // Point-cloud reprojection — pc_publish_total ticks per compute that
    // completes (i.e. per published cloud).
    std::atomic<uint64_t> pc_publish_total{0};
    std::atomic<uint64_t> pc_compute_sum_us{0};
    std::atomic<uint64_t> pc_compute_max_us{0};
    // Post-processing filter invocation counters. Each ticks once per
    // kernel invocation in pc_thread; exposed through Stats.
    std::atomic<uint64_t> spatial_filter_total{0};
    std::atomic<uint64_t> temporal_filter_total{0};
    std::atomic<uint64_t> hole_fill_total{0};

    std::string serial_number;
    std::string usb_port;

    ColorFrameCb    on_color;
    DepthFrameCb    on_depth;
    PointCloudCb    on_pc;
    PointCloudGate  pc_gate;       // null = always run
    FrameStreamGate color_gate;    // null = always run
    FrameStreamGate depth_gate;    // null = always run

    // Streaming start timestamp used by the kGatePassThroughMs grace window.
    std::chrono::steady_clock::time_point stream_start_time{};

    // One-shot ready marker. Emitted on stdout the first time either fetch
    // thread successfully receives a frame so launch tooling can observe
    // pipeline readiness without depending on subscriber-side activity.
    std::atomic<bool> ready_marker_emitted{false};

    bool opened = false;

    // True if the kGatePassThroughMs startup grace window has not yet
    // elapsed, OR the gate is unset, OR the gate explicitly returns true.
    // Consolidates the gate check duplicated across the color, depth, and
    // point-cloud threads.
    bool gate_pass(const std::function<bool()>& gate) const {
        const auto since_start = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - stream_start_time).count();
        return since_start < kGatePassThroughMs || !gate || gate();
    }
};

EspdiDevice::EspdiDevice() : impl_(std::make_unique<Impl>()) {}
EspdiDevice::~EspdiDevice() {
    RCLCPP_INFO(logger(), "~EspdiDevice()");
    stop();
    close();
    RCLCPP_INFO(logger(), "~EspdiDevice() done");
}

bool EspdiDevice::open(const DeviceConfig& cfg) {
    // Hold lifecycle_mtx for the duration of the open sequence so a
    // concurrent stop() / close() / standby() cannot run
    // partway through APC_Init -> APC_OpenDevice2 -> ZD-table load.
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    impl_->cfg = cfg;

    SignalHandlerGuard signal_guard;
    int ret = APC_Init(&impl_->handle, /*bIsLogEnabled=*/false);
    if (ret != APC_OK || impl_->handle == nullptr) {
        RCLCPP_ERROR(logger(), "APC_Init failed (%d)", ret);
        return false;
    }

    const int dev_count = APC_GetDeviceNumber(impl_->handle);
    if (dev_count <= 0) {
        RCLCPP_ERROR(logger(),
                     "APC_GetDeviceNumber returned %d ; no cameras detected", dev_count);
        APC_Release(&impl_->handle);
        return false;
    }
    RCLCPP_INFO(logger(), "APC found %d device(s)", dev_count);

    // Device selection precedence:
    //   1. dev_serial_number substring match (printed on the module label).
    //   2. usb_port substring match against the V4L2 device's USB topology
    //      path (e.g. "2-3:1.0"), resolved via /sys/class/video4linux. Stable
    //      across reboots and plug order — suitable for production wiring.
    //   3. PID match against the launch `model` token. With multiple cameras
    //      attached this is what stops two launches from grabbing the same
    //      physical device.
    //   4. Index 0 only if expected_pid is 0 (i.e. unknown model).
    // The final chosen index's PID is re-validated against `model` below;
    // any mismatch hard-fails the open.
    int chosen_index = -1;
    struct DevEnum {
        std::string serial_number;
        std::string v4l2;     // /dev/videoN as reported by the SDK
        std::string usb_port; // sysfs-resolved, e.g. "2-3:1.0"
        unsigned short pid = 0;
    };
    auto enumerate = [&](int i) {
        DEVSELINFO tmp{i};
        DEVINFORMATION info{};
        // APC_GetSerialNumber returns a UTF-16 LE string: each character is 2
        // bytes (low byte then high byte). eYs3D serial numbers are ASCII so
        // the high byte is always zero; take every even index for a clean string.
        unsigned char sn_buf[256] = {0};
        int sn_len = 0;
        DevEnum e;
        if (APC_GetSerialNumber(impl_->handle, &tmp, sn_buf, sizeof(sn_buf), &sn_len) == APC_OK
            && sn_len > 0) {
            // Clamp against the buffer to bound the index walk below if the
            // returned sn_len ever exceeds the buffer it filled.
            const int chars = std::min(sn_len / 2,
                                       static_cast<int>(sizeof(sn_buf) / 2));
            e.serial_number.resize(static_cast<size_t>(chars));
            for (int j = 0; j < chars; ++j) {
                e.serial_number[j] = static_cast<char>(sn_buf[j * 2]);
            }
        }
        if (APC_GetDeviceInfo(impl_->handle, &tmp, &info) == APC_OK) {
            if (info.strDevName) e.v4l2.assign(info.strDevName);
            e.pid = info.wPID;
        }
        e.usb_port = resolve_usb_port(e.v4l2);
        return e;
    };

    const unsigned short want_pid = expected_pid_for_model(cfg.model);

    auto log_devices = [&]() {
        for (int i = 0; i < dev_count; ++i) {
            const auto e = enumerate(i);
            RCLCPP_ERROR(logger(),
                         "  [%d] PID=0x%04x sn='%s' v4l2='%s' usb_port='%s'",
                         i, e.pid, e.serial_number.c_str(), e.v4l2.c_str(), e.usb_port.c_str());
        }
    };

    if (!cfg.serial_number.empty() || !cfg.usb_port.empty()) {
        for (int i = 0; i < dev_count; ++i) {
            const auto e = enumerate(i);
            const bool sn_match  = !cfg.serial_number.empty() &&
                                   e.serial_number.find(cfg.serial_number) != std::string::npos;
            const bool bus_match = !cfg.usb_port.empty() &&
                                   !e.usb_port.empty() &&
                                   e.usb_port.find(cfg.usb_port) != std::string::npos;
            if (sn_match || (cfg.serial_number.empty() && bus_match)) {
                chosen_index = i;
                RCLCPP_INFO(logger(),
                            "Matched device at index %d (sn='%s', v4l2='%s', usb_port='%s') via %s",
                            i, e.serial_number.c_str(), e.v4l2.c_str(), e.usb_port.c_str(),
                            sn_match ? "serial_number" : "usb_port");
                break;
            }
        }
        if (chosen_index < 0) {
            RCLCPP_ERROR(logger(),
                         "No device matches binding hints (serial='%s', usb_port='%s'). "
                         "%d device(s) enumerated:",
                         cfg.serial_number.c_str(), cfg.usb_port.c_str(), dev_count);
            log_devices();
            APC_Release(&impl_->handle);
            return false;
        }
    } else if (want_pid != 0) {
        // No SN / usb_port hint — select the first device whose USB PID matches
        // the requested model. This prevents two launches with different models
        // from binding to the same physical device when both are connected.
        int pid_match_count = 0;
        for (int i = 0; i < dev_count; ++i) {
            const auto e = enumerate(i);
            if (e.pid == want_pid) {
                ++pid_match_count;
                if (chosen_index < 0) {
                    chosen_index = i;
                    RCLCPP_INFO(logger(),
                                "Matched device at index %d (PID=0x%04x sn='%s' v4l2='%s' usb_port='%s') "
                                "via model '%s' PID lookup",
                                i, e.pid, e.serial_number.c_str(), e.v4l2.c_str(), e.usb_port.c_str(),
                                cfg.model.c_str());
                }
            }
        }
        if (pid_match_count > 1) {
            RCLCPP_WARN(logger(),
                        "%d devices match PID=0x%04x; opened the first. "
                        "Pin a specific camera via usb_port or dev_serial_number to disambiguate.",
                        pid_match_count, want_pid);
        }
        if (chosen_index < 0) {
            RCLCPP_ERROR(logger(),
                         "No device matches model '%s' (expected PID=0x%04x). "
                         "%d device(s) enumerated:",
                         cfg.model.c_str(), want_pid, dev_count);
            log_devices();
            APC_Release(&impl_->handle);
            return false;
        }
    } else {
        chosen_index = 0;
        RCLCPP_WARN(logger(),
                    "Unknown model '%s' ; no expected PID to validate against; "
                    "falling back to device index 0.",
                    cfg.model.c_str());
    }
    impl_->sel.index = chosen_index;
    // Cache the device identity strings; immutable after selection
    // and read on every diagnostics tick.
    {
        const auto e = enumerate(impl_->sel.index);
        impl_->serial_number = e.serial_number;
        impl_->usb_port = e.usb_port;
    }

    if ((ret = APC_GetDeviceInfo(impl_->handle, &impl_->sel, &impl_->dev_info)) != APC_OK) {
        RCLCPP_ERROR(logger(), "APC_GetDeviceInfo failed (%d)", ret);
        APC_Release(&impl_->handle);
        return false;
    }
    RCLCPP_INFO(logger(),
                "Device PID=0x%04x VID=0x%04x name='%s' chip=%u type=%u",
                impl_->dev_info.wPID, impl_->dev_info.wVID,
                impl_->dev_info.strDevName ? impl_->dev_info.strDevName : "(null)",
                impl_->dev_info.nChipID, impl_->dev_info.nDevType);

    // Final PID sanity check: the chosen index's PID must match the model.
    // Catches the case where SN or usb_port hints resolve to a wrong-PID
    // device (e.g. the usb_port pinned in the launch file maps to a
    // different model, or two cameras have swapped sockets).
    if (want_pid != 0 && impl_->dev_info.wPID != want_pid) {
        RCLCPP_ERROR(logger(),
                     "Device PID mismatch: model '%s' expects PID=0x%04x but the "
                     "opened device reports PID=0x%04x. Refusing to push wrong-"
                     "model parameters at the firmware.",
                     cfg.model.c_str(), want_pid, impl_->dev_info.wPID);
        APC_Release(&impl_->handle);
        return false;
    }

    {
        char fw_buf[256] = {0};
        int fw_len = 0;
        const int fw_rc = APC_GetFwVersion(
            impl_->handle, &impl_->sel, fw_buf, sizeof(fw_buf) - 1, &fw_len);
        if (fw_rc == APC_OK && fw_len > 0) {
            RCLCPP_INFO(logger(), "FW version: %s", fw_buf);
        } else {
            RCLCPP_WARN(logger(),
                        "APC_GetFwVersion rc=%d (len=%d)", fw_rc, fw_len);
        }
    }

    // Configure V4L2 for non-blocking I/O. Wide-color modes (e.g.
    // G100+ 2560x720) deadlock in blocking-mode VIDIOC_DQBUF on the wide
    // endpoint; standard modes are unaffected.
    if ((ret = APC_SetupBlock(impl_->handle, &impl_->sel, false)) != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetupBlock(false) returned %d ; continuing", ret);
    }

    // The spatial filter shifts depth_data_type to its 11-bit-disparity
    // counterpart at (code + 2): 2 → 4, 7 → 9, 18 → 20, 50 → 52. The
    // shift is only valid when a depth stream is configured; color-only
    // modes carry depth_data_type 0 or 5 and must pass through unchanged.
    const bool depth_present = cfg.depth_width > 0 && cfg.depth_height > 0;
    const bool apply_disparity_shift = cfg.spatial_filter_enabled && depth_present;
    if (cfg.spatial_filter_enabled && !depth_present) {
        RCLCPP_WARN(logger(),
                    "spatial_filter requested but the active mode has no "
                    "depth stream ; filter disabled, depth_data_type left unchanged.");
    }
    const int effective_depth_dt = apply_disparity_shift
        ? cfg.depth_data_type + 2
        : cfg.depth_data_type;
    if (apply_disparity_shift) {
        RCLCPP_INFO(logger(),
                    "Spatial filter enabled: depth_data_type %d -> %d "
                    "(alpha=%.2f delta=%d magnitude=%d)",
                    cfg.depth_data_type, effective_depth_dt,
                    cfg.spatial_filter_alpha,
                    cfg.spatial_filter_delta,
                    cfg.spatial_filter_magnitude);
    }

    if ((ret = APC_SetDepthDataType(impl_->handle, &impl_->sel,
                                    static_cast<unsigned short>(effective_depth_dt))) != APC_OK) {
        RCLCPP_ERROR(logger(),
                     "APC_SetDepthDataType(%d) failed (%d)", effective_depth_dt, ret);
        APC_Release(&impl_->handle);
        return false;
    }
    // Store the effective dtype so downstream code (fetch threads,
    // pc_thread, buffer sizing) all see a consistent value.
    impl_->cfg.depth_data_type = effective_depth_dt;

    // 32 V4L2 buffers — enough headroom for the highest-fps modes without
    // starving depth fetch on bursty USB scheduling.
    if ((ret = APC_Setup_v4l2_requestbuffers(impl_->handle, &impl_->sel, 32)) != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_Setup_v4l2_requestbuffers(32) returned %d", ret);
    }

    if ((ret = APC_SetInterleaveMode(impl_->handle, &impl_->sel, cfg.interleave)) != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetInterleaveMode(%d) returned %d", cfg.interleave, ret);
    }

    // IR projector level is applied before APC_OpenDevice2 so the
    // V4L2 capture buffer fills at the configured illumination from the
    // first frame. cfg.ir_intensity ≥ 0 selects an explicit value;
    // -1 selects the per-PID default. IR-MAX and mode-mask are left at
    // their firmware boot values (G62 IR-MAX = 96; G100+/R77 IR-MAX ≥ 6).
    {
        const bool explicit_ir = cfg.ir_intensity >= 0;
        const int level = explicit_ir
            ? cfg.ir_intensity
            : default_ir_level_for_pid(impl_->dev_info.wPID);
        const int rc_cur = APC_SetCurrentIRValue(
            impl_->handle, &impl_->sel,
            static_cast<unsigned short>(level));
        RCLCPP_INFO(logger(),
                    "IR pre-open: SetCurrentIRValue(%d) rc=%d (%s)",
                    level, rc_cur, explicit_ir ? "explicit" : "default");
    }

    // APC_OpenDevice2 writes the negotiated fps through a raw int*; bounce
    // through a local int because atomic<int>::data() is not portable.
    int negotiated_fps = cfg.framerate;
    // Open the device with raw YUYV output (bIsOutputRGB24 = false) and
    // serial-number-synchronised color/depth pairing (IMAGE_SN_SYNC). The PC
    // thread performs the YUYV→RGB24 conversion only when a subscriber is
    // listening to the point-cloud topic, avoiding the unconditional cost.
    ret = APC_OpenDevice2(
        impl_->handle, &impl_->sel,
        cfg.color_width, cfg.color_height,
        static_cast<bool>(cfg.color_format),
        cfg.depth_width, cfg.depth_height,
        DEPTH_IMG_NON_TRANSFER,
        /*bIsOutputRGB24=*/true,
        /*phWndNotice=*/nullptr,
        &negotiated_fps,
        IMAGE_SN_SYNC);
    if (ret != APC_OK) {
        RCLCPP_ERROR(logger(),
                     "APC_OpenDevice2 failed (%d) ; requested fps=%d, got %d",
                     ret, cfg.framerate, negotiated_fps);
        APC_Release(&impl_->handle);
        return false;
    }
    impl_->actual_fps.store(negotiated_fps, std::memory_order_relaxed);
    RCLCPP_INFO(logger(),
                "Device opened: color %dx%d %s, depth %dx%d type=%d, fps=%d, interleave=%d",
                cfg.color_width, cfg.color_height,
                cfg.color_format == 0 ? "YUYV" : "MJPEG",
                cfg.depth_width, cfg.depth_height, cfg.depth_data_type,
                negotiated_fps, cfg.interleave ? 1 : 0);

    // Read the FW-reported IR ceiling (reg 0xE2) so set_ir_intensity can
    // clamp user-supplied values, and log the current level for diagnostics.
    {
        impl_->ir_default_level = default_ir_level_for_pid(impl_->dev_info.wPID);

        unsigned short ir_max = 0, ir_cur = 0;
        const int rc_max = APC_GetFWRegister(
            impl_->handle, &impl_->sel, kFwRegIrMax, &ir_max,
            FG_Address_1Byte | FG_Value_1Byte);
        APC_GetCurrentIRValue(impl_->handle, &impl_->sel, &ir_cur);
        if (rc_max == APC_OK) {
            impl_->ir_max_fw = static_cast<int>(ir_max);
            impl_->ir_range_valid = true;
        }
        RCLCPP_INFO(logger(),
                    "Camera IR state on open: current=%u (FW max=%u, default=%d)",
                    ir_cur, ir_max, impl_->ir_default_level);
    }

    // Load the rectification log into both lens calibration slots so left
    // and right camera_info topics can publish independently. The log
    // contains both intrinsics regardless of the active video mode.
    //
    // Retry on APC_READFLASHFAIL (-6): the initial flash read can fail
    // transiently after a fast reopen of the same device. A short backoff
    // and re-attempt recovers without needing to power-cycle the camera.
    {
        constexpr int kMaxRectifyAttempts = 4;
        constexpr int kRectifyBackoffMs   = 150;
        int rc = APC_OK;
        for (int attempt = 0; attempt < kMaxRectifyAttempts; ++attempt) {
            rc = APC_GetRectifyMatLogData(
                impl_->handle, &impl_->sel, &impl_->cached_rect, cfg.zd_index);
            if (rc == APC_OK) break;
            if (rc != APC_READFLASHFAIL) break;
            RCLCPP_WARN(logger(),
                        "APC_GetRectifyMatLogData(index=%d) rc=%d (flash read), "
                        "retry %d/%d after %d ms",
                        cfg.zd_index, rc, attempt + 1,
                        kMaxRectifyAttempts, kRectifyBackoffMs);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kRectifyBackoffMs));
        }
        if (rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "APC_GetRectifyMatLogData(index=%d) rc=%d ; camera_info will be empty",
                        cfg.zd_index, rc);
        } else {
            impl_->cached_rect_valid = true;
            const auto& log = impl_->cached_rect;
            auto& c = impl_->calib;
            // Use color dims when color is active; fall back to depth dims for
            // D-only modes so camera_info still carries valid width/height.
            c.width  = cfg.color_width  > 0 ? cfg.color_width  : cfg.depth_width;
            c.height = cfg.color_height > 0 ? cfg.color_height : cfg.depth_height;
            for (int i = 0; i < 9; ++i)  c.left.K[i]  = log.CamMat1[i];
            for (int i = 0; i < 5; ++i)  c.left.D[i]  = log.CamDist1[i];
            for (int i = 0; i < 9; ++i)  c.left.R[i]  = log.LRotaMat[i];
            for (int i = 0; i < 12; ++i) c.left.P[i]  = log.NewCamMat1[i];
            for (int i = 0; i < 9; ++i)  c.right.K[i] = log.CamMat2[i];
            for (int i = 0; i < 5; ++i)  c.right.D[i] = log.CamDist2[i];
            for (int i = 0; i < 9; ++i)  c.right.R[i] = log.RRotaMat[i];
            for (int i = 0; i < 12; ++i) c.right.P[i] = log.NewCamMat2[i];
            c.baseline_mm = std::abs(static_cast<double>(log.TranMat[0]));
            c.valid = true;
            RCLCPP_INFO(logger(),
                        "Rectify loaded (index=%d): L fx=%.2f cx=%.2f / R fx=%.2f cx=%.2f / baseline=%.2f mm",
                        cfg.zd_index,
                        c.left.K[0], c.left.K[2], c.right.K[0], c.right.K[2], c.baseline_mm);
        }
    }

    // Colored point cloud requires a configured color stream; D-only
    // modes use the XYZ-only path.
    impl_->colored_pointcloud = cfg.colored_pointcloud && cfg.color_width > 0 && cfg.color_height > 0;

    // Spatial filter: launch-time only. The depth stream is already
    // in 11-bit disparity mode at this point (apply_disparity_shift
    // forced the dtype +2 above), so a ZD table load failure is a
    // hard error rather than a silent fallback. Skipped entirely on
    // color-only modes — the WARN was emitted above when the dtype
    // shift was bypassed.
    impl_->spatial_filter_enabled = false;
    if (apply_disparity_shift) {
        const double a = std::clamp(cfg.spatial_filter_alpha, 0.0, 1.0);
        impl_->spatial_params.alpha_q8      = static_cast<int>(std::lround(a * 256.0));
        // Clamp keeps the Q4 shift inside uint16.
        impl_->spatial_params.delta_q4      = std::clamp(cfg.spatial_filter_delta, 1, 4095) << 4;
        impl_->spatial_params.magnitude     = std::clamp(cfg.spatial_filter_magnitude, 1, 5);
        impl_->spatial_params.holes_fill = std::max(0, cfg.spatial_filter_holes_fill);
        if (!load_zd_table(impl_->handle, &impl_->sel,
                           cfg.zd_index, impl_->zd_table)) {
            RCLCPP_ERROR(logger(),
                         "Spatial filter requested but ZD table load failed "
                         "(zd_index=%d). Refusing to open.",
                         cfg.zd_index);
            // APC_OpenDevice2 already succeeded above, so the device
            // pipe + V4L2 fds need to be released by APC_CloseDevice
            // before the SDK handle is freed by APC_Release. Skipping
            // close leaks the device resources across reconnect loops.
            APC_CloseDevice(impl_->handle, &impl_->sel);
            APC_Release(&impl_->handle);
            impl_->handle = nullptr;
            return false;
        }
        impl_->spatial_filter_enabled = true;
    }

    // Temporal filter: runs in whichever domain has a uint16 raster
    // available. With spatial_filter on, it lives between the IIR and
    // the ZD lookup (Q4 disparity units); with spatial_filter off, it
    // runs directly on the FW Z14 mm depth raster. delta is stored
    // raw and converted at the call site (`<< 4` for the Q4 pipeline,
    // used as mm for the Z pipeline). Seeded here from launch values
    // regardless of spatial state; runtime updates flow through
    // set_temporal_filter().
    {
        const double ta = std::clamp(cfg.temporal_filter_alpha, 0.0, 1.0);
        TemporalFilterParams tp;
        tp.alpha_q8    = static_cast<int>(std::lround(ta * 256.0));
        // Clamp keeps the Q4 promote (`<<= 4`) inside uint16.
        tp.delta       = std::clamp(cfg.temporal_filter_delta, 1, 4095);
        tp.persistence = std::clamp(cfg.temporal_filter_persistence, 0, 8);
        std::lock_guard<std::mutex> lk(impl_->temporal_mtx);
        impl_->temporal_params_pending = tp;
        impl_->temporal_reset_pending  = cfg.temporal_filter_enabled;
    }
    impl_->temporal_enabled.store(cfg.temporal_filter_enabled,
                                  std::memory_order_release);
    if (cfg.temporal_filter_enabled) {
        RCLCPP_INFO(logger(),
                    "Temporal filter enabled: alpha=%.2f delta=%d persistence=%d "
                    "(%s domain)",
                    cfg.temporal_filter_alpha,
                    cfg.temporal_filter_delta,
                    cfg.temporal_filter_persistence,
                    apply_disparity_shift ? "D11 disparity" : "Z14 mm");
    }

    // Hole filling: runs in Z14 mm domain. Available regardless of
    // spatial_filter — without it, the FW depth raster goes straight
    // into the kernel; with it, the post-ZD-lookup buffer does.
    impl_->hole_fill_mode = HoleFillMode::kOff;
    if (cfg.hole_filling > 0) {
        const int m = std::clamp(cfg.hole_filling, 0, 3);
        impl_->hole_fill_mode = static_cast<HoleFillMode>(m);
        const char* name = (m == 1) ? "fill_from_left"
                         : (m == 2) ? "farthest_from_around"
                                    : "nearest_from_around";
        RCLCPP_INFO(logger(), "Hole filling enabled: mode=%d (%s)", m, name);
    }

    // PointCloud Z clip range. Launch parameters depth_minimum_mm /
    // depth_maximum_mm > 0 override; -1 = use per-PID default.
    {
        const auto def = depth_range_for_pid(impl_->dev_info.wPID);
        const float resolved_near = (cfg.depth_minimum_mm > 0)
            ? static_cast<float>(cfg.depth_minimum_mm)
            : static_cast<float>(def.near_mm);
        const float resolved_far  = (cfg.depth_maximum_mm > 0)
            ? static_cast<float>(cfg.depth_maximum_mm)
            : static_cast<float>(def.far_mm);

        impl_->max_near_mm = resolved_near;
        impl_->max_far_mm  = resolved_far;

        RCLCPP_INFO(logger(),
                    "Depth clip range: near=%.0f mm (%s)  far=%.0f mm (%s)  (default=[%d, %d] mm)",
                    resolved_near,
                    (cfg.depth_minimum_mm > 0) ? "explicit" : "default",
                    resolved_far,
                    (cfg.depth_maximum_mm > 0) ? "explicit" : "default",
                    def.near_mm, def.far_mm);
    }

    {
        std::lock_guard<std::mutex> lk(impl_->latest.mtx);
        impl_->latest.depth.reset();
        impl_->latest.depth_pending = false;
    }

    impl_->opened = true;
    return true;
}

void EspdiDevice::close() {
    if (!impl_) return;
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    if (impl_->opened && impl_->handle) {
        RCLCPP_INFO(logger(), "close(): APC_CloseDevice...");
        APC_CloseDevice(impl_->handle, &impl_->sel);
        RCLCPP_INFO(logger(), "close(): APC_CloseDevice returned");
    }
    if (impl_->handle) {
        RCLCPP_INFO(logger(), "close(): APC_Release...");
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
        RCLCPP_INFO(logger(), "close(): APC_Release returned");
    }
    impl_->opened = false;
    // Drop the cached color snapshot so the next open paints XYZRGB
    // clouds from a fresh color frame rather than a pre-close one.
    {
        std::lock_guard<std::mutex> lk(impl_->latest_color.mtx);
        impl_->latest_color.rgb.reset();
        impl_->latest_color.w = 0;
        impl_->latest_color.h = 0;
        impl_->latest_color.ts_us = 0;
    }
}

void EspdiDevice::start(ColorFrameCb on_color, DepthFrameCb on_depth, PointCloudCb on_pc) {
    if (!impl_->opened) {
        RCLCPP_ERROR(logger(), "start() called before open() succeeded");
        return;
    }
    if (impl_->running.exchange(true)) return;

    impl_->on_color = std::move(on_color);
    impl_->on_depth = std::move(on_depth);
    impl_->on_pc    = std::move(on_pc);
    // Whether each stream is present in this open() configuration is
    // determined by callback presence — passing a null callback for a
    // stream means that stream is absent (e.g. D-only modes don't pass a
    // color callback). Stream-presence is independent of the runtime
    // pause/standby state and is never modified after start().
    impl_->color_stream_present = static_cast<bool>(impl_->on_color);
    impl_->depth_stream_present = static_cast<bool>(impl_->on_depth);
    impl_->stream_state.store(StreamState::Active, std::memory_order_relaxed);
    impl_->stream_start_time = std::chrono::steady_clock::now();
    impl_->ready_marker_emitted.store(false, std::memory_order_relaxed);
    spawn_fetch_threads_();
}

void EspdiDevice::spawn_fetch_threads_() {
    const auto& cfg = impl_->cfg;
    const bool interleave = cfg.interleave;
    void* handle = impl_->handle;
    DEVSELINFO* sel = &impl_->sel;
    const int depth_dt = cfg.depth_data_type;

    // Reset SN baselines so a close/reopen cycle doesn't register the SN
    // restart as a giant dropped-frame burst.
    impl_->last_color_sn.store(-1, std::memory_order_relaxed);
    impl_->last_depth_sn.store(-1, std::memory_order_relaxed);

    if (impl_->color_stream_present) {
    // Color fetch thread. The wire payload (YUYV or MJPEG) is read
    // into a long-lived raw buffer, decoded into FrameBuffer.data as
    // rgb8, then moved into the publisher callback. Publishing rgb8
    // uniformly lets downstream tooling (RViz, image_pipeline,
    // cv_bridge) consume the topic without a format-conversion step.
    impl_->color_fetch = std::thread([this, handle, sel, depth_dt, interleave, &cfg]() {
        const int  cw = cfg.color_width;
        const int  ch = cfg.color_height;
        const bool wire_is_mjpeg = (cfg.color_format == 1);
        // Split-aware YUYV decode emits two half-width rgb8 buffers in
        // one pass, avoiding the wide rgb8 intermediate and the
        // row-by-row split memcpy. MJPEG modes cannot be split during
        // decode (libjpeg-turbo's MCU blocks do not align with the
        // mid-row boundary) and continue to decode wide and slice in
        // camera_node.
        const bool split_yuyv = cfg.split_color && !wire_is_mjpeg &&
                                (cw % 2 == 0);
        const int  side_w   = split_yuyv ? cw / 2 : cw;
        const size_t raw_bytes = color_raw_buffer_bytes(cw, ch);
        // rgb_bytes sizes the per-frame fb.data:
        //   - YUYV split: side_w = cw/2, so this is the half-width buffer
        //     and the SIMD split writer fills both halves in one pass.
        //   - MJPEG split: side_w = cw, fb.data holds the full wide raster
        //     and tjDecompress2 writes cw*ch*3 bytes into it; the actual
        //     left/right split happens later in publish_split_color.
        //   - non-split: side_w = cw, single per-side buffer.
        const size_t rgb_bytes = color_rgb8_bytes(side_w, ch);

        // Reused across iterations — the SDK reads into this buffer, then
        // the loop decodes / converts into the per-frame `fb.data` (rgb8)
        // before moving fb into the callback. One allocation total instead
        // of one per frame.
        std::vector<uint8_t> raw(raw_bytes);

        // libjpeg-turbo decompressor for MJPEG modes. TJPF_RGB tells
        // turbojpeg to emit rgb8 directly; for grayscale-source JPEGs
        // (R77 / G62) it replicates Y → R=G=B internally. Symbols
        // (tjInitDecompress / tjDecompress2 / tjDestroy) are re-exported
        // from libeSPDI, so no external libjpeg link is required.
        tjhandle tj = wire_is_mjpeg ? tjInitDecompress() : nullptr;
        if (wire_is_mjpeg && !tj) {
            RCLCPP_ERROR(logger(),
                         "tjInitDecompress() failed ; MJPEG modes unusable");
        }

        while (impl_->running.load(std::memory_order_acquire)) {
          try {
            unsigned long got = 0;
            int serial = 0;
            int64_t tv_sec = 0, tv_usec = 0;
            const int rc = APC_GetColorImageWithTimestamp(
                handle, sel, raw.data(), &got, &serial,
                depth_dt, &tv_sec, &tv_usec);
            if (rc != APC_OK) {
                if (rc == APC_DEVICE_TIMEOUT) {
                    usleep(kTimeoutBackoffUs);
                } else {
                    // Throttle unexpected return codes so a misconfigured
                    // stream remains visible without flooding the log.
                    RCLCPP_WARN_THROTTLE(logger(),
                                         throttle_clock(), 5000,
                                         "APC_GetColorImageWithTimestamp rc=%d", rc);
                }
                continue;
            }
            // Interleave SN parity: in interleave mode both streams deliver
            // every frame and the consumer selects by serial number parity
            // (color = even, depth = odd).
            if (interleave && (serial % 2) != 0) {
                continue;
            }
            // Detect frames lost in transit. After parity filtering,
            // accepted SNs should advance by `step` per frame; any larger
            // forward gap means the USB / SDK layer dropped one or more
            // frames before they reached the fetch loop.
            const int step = interleave ? 2 : 1;
            const int prev_sn = impl_->last_color_sn.exchange(
                serial, std::memory_order_relaxed);
            if (prev_sn >= 0) {
                const int delta = serial - prev_sn;
                if (delta > step && delta < 10000) {
                    impl_->color_input_dropped.fetch_add(
                        static_cast<uint64_t>((delta / step) - 1),
                        std::memory_order_relaxed);
                }
            }
            impl_->color_input_total.fetch_add(1, std::memory_order_relaxed);

            if (!impl_->ready_marker_emitted.exchange(true)) emit_ready_marker();

            // Pause gate: when stream_state == Paused we keep draining the
            // USB so the SDK buffer doesn't back up, but the per-frame
            // decode (tjDecompress2 / YUYV→RGB) and publish are skipped.
            // CPU usage drops to roughly the cost of the APC_Get* read.
            if (impl_->stream_state.load(std::memory_order_relaxed)
                    == StreamState::Paused) {
                continue;
            }
            // Skip the decode + publish hop when nothing downstream needs
            // the color frame. The V4L2 buffer is already drained above;
            // only the per-frame CPU work (tjDecompress2 / YUYV→RGB /
            // on_color callback) is bypassed.
            if (!impl_->gate_pass(impl_->color_gate)) {
                continue;
            }

            FrameBuffer fb;
            fb.data.resize(rgb_bytes);
            if (split_yuyv) fb.data_right.resize(rgb_bytes);
            fb.serial_number = serial;
            fb.hw_timestamp_us =
                static_cast<uint64_t>(tv_sec) * 1000000ULL + static_cast<uint64_t>(tv_usec);
            fb.width  = side_w;
            fb.height = ch;

            const auto t_decode_begin = std::chrono::steady_clock::now();
            if (wire_is_mjpeg) {
                // MJPEG → rgb8 inline (libjpeg-turbo SIMD; ~5-10 ms / 1.2 MP).
                if (!tj) continue;
                const int drc = tjDecompress2(
                    tj, raw.data(), got,
                    fb.data.data(),
                    cw, /*pitch=*/cw * 3, ch,
                    TJPF_RGB, /*flags=*/0);
                if (drc != 0) {
                    RCLCPP_WARN_THROTTLE(logger(),
                                         throttle_clock(), 5000,
                                         "tjDecompress2 failed: %s",
                                         tjGetErrorStr2(tj));
                    continue;
                }
            } else if (split_yuyv) {
                // Wide YUYV → two half-width rgb8 buffers in one pass.
                // Saves the wide rgb8 intermediate (≈5.5 MB at 2560x720)
                // and the row-by-row memcpy split in camera_node.
                simd::yuyv_to_rgb8_split(raw.data(),
                                         fb.data.data(),
                                         fb.data_right.data(),
                                         side_w, ch);
            } else {
                // YUYV → rgb8 single buffer (NEON on aarch64, scalar+OMP elsewhere).
                yuyv_to_rgb8(raw.data(), fb.data.data(), cw, ch);
            }
            const auto t_decode_end = std::chrono::steady_clock::now();
            const uint64_t decode_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    t_decode_end - t_decode_begin).count());
            impl_->color_decode_sum_us.fetch_add(decode_us,
                                                 std::memory_order_relaxed);
            uint64_t prev_max = impl_->color_decode_max_us.load(std::memory_order_relaxed);
            while (decode_us > prev_max &&
                   !impl_->color_decode_max_us.compare_exchange_weak(
                       prev_max, decode_us)) {}

            // Snapshot the decoded rgb8 buffer for pc_thread's XYZRGB
            // projection. Gated by the pc subscriber so the per-frame
            // buffer copy is only paid when the cloud is being consumed.
            if (impl_->colored_pointcloud && impl_->gate_pass(impl_->pc_gate)) {
                auto shared_rgb =
                    std::make_shared<const std::vector<uint8_t>>(fb.data);
                std::lock_guard<std::mutex> lk(impl_->latest_color.mtx);
                impl_->latest_color.rgb   = std::move(shared_rgb);
                impl_->latest_color.w     = side_w;
                impl_->latest_color.h     = ch;
                impl_->latest_color.ts_us = fb.hw_timestamp_us;
            }

            if (impl_->on_color) {
                impl_->color_publish_total.fetch_add(1, std::memory_order_relaxed);
                impl_->on_color(std::move(fb));
            }
          } catch (const std::bad_alloc& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "color_fetch: bad_alloc (%s); dropping frame, continuing",
                                 e.what());
          } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "color_fetch: unhandled exception (%s); dropping frame, continuing",
                                 e.what());
          }
        }
        if (tj) tjDestroy(tj);
    });
    }  // end if (color_stream_present)

    if (!impl_->depth_stream_present) {
        RCLCPP_INFO(logger(),
                    "Depth stream not configured ; depth + pc threads not spawned.");
        return;
    }

    // 200 ms delay so the color V4L2 pipeline reaches STREAMON before depth
    // fetch begins; some firmware variants require color-first start ordering.
    if (impl_->color_stream_present) {
        usleep(200 * 1000);
    }

    // Depth fetch thread — symmetric, plus signals pc_thread.
    impl_->depth_fetch = std::thread([this, handle, sel, depth_dt, interleave, &cfg]() {
        const size_t buf_bytes = depth_buffer_bytes(cfg.depth_width, cfg.depth_height, cfg.depth_data_type);
        while (impl_->running.load(std::memory_order_acquire)) {
          try {
            FrameBuffer fb;
            fb.data.resize(buf_bytes);
            unsigned long got = 0;
            int serial = 0;
            int64_t tv_sec = 0, tv_usec = 0;
            const int rc = APC_GetDepthImageWithTimestamp(
                handle, sel, fb.data.data(), &got, &serial,
                depth_dt, &tv_sec, &tv_usec);
            if (rc != APC_OK) {
                if (rc == APC_DEVICE_TIMEOUT) {
                    usleep(kTimeoutBackoffUs);
                } else {
                    RCLCPP_WARN_THROTTLE(logger(),
                                         throttle_clock(), 5000,
                                         "APC_GetDepthImageWithTimestamp rc=%d", rc);
                }
                continue;
            }
            if (interleave && (serial % 2) != 1) {
                continue;
            }
            const int step = interleave ? 2 : 1;
            const int prev_sn = impl_->last_depth_sn.exchange(
                serial, std::memory_order_relaxed);
            if (prev_sn >= 0) {
                const int delta = serial - prev_sn;
                if (delta > step && delta < 10000) {
                    impl_->depth_input_dropped.fetch_add(
                        static_cast<uint64_t>((delta / step) - 1),
                        std::memory_order_relaxed);
                }
            }
            impl_->depth_input_total.fetch_add(1, std::memory_order_relaxed);

            if (!impl_->ready_marker_emitted.exchange(true)) emit_ready_marker();

            // Pause gate: when stream_state == Paused we keep draining the
            // USB so the SDK buffer doesn't back up, but skip the publish
            // + the staging-for-pc step (which would wake pc_thread and
            // burn CPU on filters / reprojection nobody is listening to).
            if (impl_->stream_state.load(std::memory_order_relaxed)
                    == StreamState::Paused) {
                continue;
            }

            // Drop frames whose size does not match the configured allocation.
            // Publishing a smaller buffer would advertise an incorrect Image
            // size to subscribers; a larger buffer would mean the FW returned
            // more bytes than the depth_width * depth_height * bpp budget.
            if (got != buf_bytes) {
                RCLCPP_WARN_THROTTLE(logger(),
                                     throttle_clock(), 5000,
                                     "depth frame size mismatch: got=%lu expected=%zu (dropped)",
                                     got, buf_bytes);
                continue;
            }
            fb.serial_number = serial;
            fb.hw_timestamp_us =
                static_cast<uint64_t>(tv_sec) * 1000000ULL + static_cast<uint64_t>(tv_usec);
            fb.width  = cfg.depth_width;
            fb.height = cfg.depth_height;

            // pc_thread becomes the depth publisher whenever any
            // post-processing filter is active (spatial / temporal /
            // hole_filling). depth_fetch's job in that case is to
            // stage the raw FW depth for the filter pipeline. With
            // all filters off, depth_fetch publishes the raw buffer
            // directly.
            const bool depth_gate_open = impl_->gate_pass(impl_->depth_gate);
            const bool pc_gate_open    = impl_->gate_pass(impl_->pc_gate);
            const bool any_filter = impl_->spatial_filter_enabled
                || impl_->temporal_enabled.load(std::memory_order_acquire)
                || impl_->hole_fill_mode != HoleFillMode::kOff;
            const bool need_snapshot = any_filter
                ? (depth_gate_open || pc_gate_open)
                : pc_gate_open;

            // No-filter publish path: apply the depth-range clip in
            // place before either downstream consumer reads from the
            // buffer, so /depth_image and the point-cloud snapshot
            // see the same clipped raster. The Z14 high 2 bits are
            // status flags and are stripped before the range test.
            // The row-0 serial-number watermark is passed through;
            // it is skipped at reprojection via kSerialSkipPixels
            // and its presence in /depth_image is by design.
            if (!any_filter && (depth_gate_open || pc_gate_open)) {
                constexpr uint16_t kDepthMask = 0x3FFF;
                const uint16_t z_min_clip = static_cast<uint16_t>(
                    std::clamp(impl_->max_near_mm, 0.0f, 65535.0f));
                const uint16_t z_max_clip = static_cast<uint16_t>(
                    std::clamp(impl_->max_far_mm,  0.0f, 65535.0f));
                uint16_t* mm = reinterpret_cast<uint16_t*>(fb.data.data());
                const size_t n = static_cast<size_t>(fb.width) * fb.height;
                for (size_t i = 0; i < n; ++i) {
                    const uint16_t z = mm[i] & kDepthMask;
                    mm[i] = (z < z_min_clip || z > z_max_clip) ? 0 : z;
                }
            }

            if (need_snapshot) {
                auto shared_buf =
                    std::make_shared<const std::vector<uint8_t>>(fb.data);
                {
                    std::lock_guard<std::mutex> lk(impl_->latest.mtx);
                    impl_->latest.depth         = std::move(shared_buf);
                    impl_->latest.depth_ts_us   = fb.hw_timestamp_us;
                    impl_->latest.depth_pending = true;
                }
                impl_->latest.cv.notify_one();
            }

            if (!any_filter
                && depth_gate_open
                && impl_->on_depth) {
                impl_->depth_publish_total.fetch_add(1, std::memory_order_relaxed);
                impl_->on_depth(std::move(fb));
            }
          } catch (const std::bad_alloc& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "depth_fetch: bad_alloc (%s); dropping frame, continuing",
                                 e.what());
          } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "depth_fetch: unhandled exception (%s); dropping frame, continuing",
                                 e.what());
          }
        }
    });

    // PointCloud thread. Reprojects depth → XYZ via a precomputed LUT
    // (count + project, both parallelised with OpenMP across rows).
    // Emits XYZ in ROS base convention (X forward, Y left, Z up) in
    // metres and passes (bytes, valid_count, hw_ts) to on_pc.
    impl_->pc_thread = std::thread([this, &cfg]() {
        // Without rectify, intrinsics are unavailable and the cloud
        // cannot be reprojected — but the depth raster itself is
        // independent of rectify, so pc_thread still owns the post-
        // filter depth publish whenever a filter is active. The
        // reprojection block in the loop is gated on can_reproject.
        const bool can_reproject = impl_->cached_rect_valid;
        if (!can_reproject) {
            RCLCPP_WARN(logger(),
                        "Rectify log unavailable: /pointcloud disabled; "
                        "filtered /depth_image still published");
        }
        const size_t pc_points   = static_cast<size_t>(cfg.depth_width) * cfg.depth_height;
        const int    W = cfg.depth_width;
        const int    H = cfg.depth_height;
        // Workspace stride: 12 bytes for XYZ float32, 16 for XYZRGB
        // (XYZ followed by a packed 0x00RRGGBB uint32). Sized once at
        // worst case; each publish copies only the populated prefix.
        const uint32_t pc_point_step = impl_->colored_pointcloud ? 16u : 12u;
        std::vector<uint8_t> workspace(pc_points * pc_point_step);

        // Filter workspace. pc_q4_buf only exists in the disparity
        // pipeline; pc_mm_buf is used by every filter (post-ZD-lookup
        // output for spatial, in-place Z14 mm buffer for the standalone
        // temporal / hole_filling paths). Sized once at thread start;
        // unused buffers stay empty. Temporal's enable can flip at
        // runtime, so the Z-domain branch reuses pc_mm_buf without
        // reallocating.
        if (impl_->spatial_filter_enabled) {
            impl_->pc_q4_buf.assign(pc_points, 0);
        }
        impl_->pc_mm_buf.assign(pc_points, 0);

        // Color-pixel byte-offset LUTs. The projection inner loop then
        // computes a per-pixel color address as two LUT reads plus one
        // add, with no per-pixel multiplication or division.
        //
        // The LUT must match the snapshot's actual width. In wide L|R
        // split modes the snapshot holds the left-half rgb8 buffer
        // (width = color_width / 2); the LUT divisor and the row
        // stride both use that half-width so sampling reads from the
        // left lens.
        if (impl_->colored_pointcloud) {
            // Two widths matter for the LUT and they are not always equal:
            //   sample_w — pixel range the LUT addresses. Split modes
            //              restrict this to the left half regardless of
            //              wire format so the cloud samples the left
            //              lens only.
            //   buf_w    — row stride (in pixels) of the snapshot buffer
            //              that ends up in latest_color. The YUYV split
            //              path produces a half-width buffer inline (see
            //              simd::yuyv_to_rgb8_split); the MJPEG path
            //              decodes the wide frame at its full width.
            const bool split_active     = cfg.split_color
                                          && (cfg.color_width % 2 == 0);
            const bool wire_is_mjpeg    = (cfg.color_format == 1);
            const bool split_yuyv_inline = split_active && !wire_is_mjpeg;
            const int sample_w = split_active
                ? cfg.color_width / 2
                : cfg.color_width;
            const int buf_w    = split_yuyv_inline
                ? cfg.color_width / 2
                : cfg.color_width;
            const int ch       = cfg.color_height;
            impl_->colored_pointcloud_cu_byte_off.assign(W, 0);
            impl_->colored_pointcloud_cv_byte_off.assign(H, 0);
            for (int u = 0; u < W; ++u) {
                impl_->colored_pointcloud_cu_byte_off[u] =
                    (u * sample_w) / W * 3;
            }
            for (int v = 0; v < H; ++v) {
                impl_->colored_pointcloud_cv_byte_off[v] =
                    ((v * ch) / H) * buf_w * 3;
            }
            RCLCPP_INFO(logger(),
                        "Colored point cloud enabled: depth %dx%d -> color %dx%d%s, point_step=16",
                        W, H, sample_w, ch,
                        split_active ? " (left half of wide L|R)" : "");
        }

        // Rectify-derived intrinsics. ratio_mat scales the rectified
        // projection matrix to the current depth resolution; it is 1.0
        // when depth matches OutImgHeight and < 1.0 for scale-down modes.
        // Only meaningful when can_reproject; the reprojection block in
        // the main loop is skipped otherwise.
        const auto& rl = impl_->cached_rect;
        const float ratio_mat = (can_reproject && rl.OutImgHeight > 0)
            ? static_cast<float>(H) / rl.OutImgHeight
            : 1.0f;

        // Pre-negated LUTs map (u,v) → axis-remapped ROS-base coords:
        //   base_x =  z_m
        //   base_y = -(u - cx)/fx * z_m  → u_inv_neg[u] * z_m
        //   base_z = -(v - cy)/fy * z_m  → v_inv_neg[v] * z_m
        // Precomputing the divisions removes 2 divs per valid pixel.
        std::vector<float>    u_inv_neg(W);
        std::vector<float>    v_inv_neg(H);
        std::vector<uint32_t> row_valid_counts(H, 0);
        std::vector<uint32_t> row_offsets(H, 0);
        // Conservative upper bound on the stereo left-edge dead-zone
        // width, evaluated at the configured Z minimum. The actual
        // dead-zone width is proportional to 1 / Z and shrinks with
        // farther objects; this fixed bound is used only by the
        // fill_from_left hole filling mode as left_skip, so that the
        // dead-zone columns are passed through unchanged instead of
        // seeding the per-row last_valid state. Columns that turn
        // out to be valid stereo matches at runtime (when scene Z
        // exceeds Z_min) keep their values either way — left_skip
        // never overwrites valid data.
        int dead_zone_left_px = 0;
        if (can_reproject) {
            const float fx = rl.NewCamMat1[0] * ratio_mat;
            const float fy = rl.NewCamMat1[5] * ratio_mat;
            const float cx = rl.NewCamMat1[2] * ratio_mat;
            const float cy = rl.NewCamMat1[6] * ratio_mat;
            if (fx == 0.0f || fy == 0.0f) {
                RCLCPP_ERROR(logger(),
                             "Reprojection LUT init: fx or fy is zero "
                             "(fx=%.2f fy=%.2f); /pointcloud disabled, "
                             "filtered /depth_image still published",
                             fx, fy);
                // Continue running for depth publishes; just leave the
                // reprojection LUTs empty so the loop below skips it.
                u_inv_neg.clear();
                v_inv_neg.clear();
            } else {
                const float inv_fx = 1.0f / fx;
                const float inv_fy = 1.0f / fy;
                for (int u = 0; u < W; ++u) u_inv_neg[u] = -(static_cast<float>(u) - cx) * inv_fx;
                for (int v = 0; v < H; ++v) v_inv_neg[v] = -(static_cast<float>(v) - cy) * inv_fy;

                const double baseline_mm = impl_->calib.baseline_mm;
                const double z_min_mm    = static_cast<double>(impl_->max_near_mm);
                if (baseline_mm > 0.0 && z_min_mm > 0.0) {
                    dead_zone_left_px = static_cast<int>(
                        std::ceil(baseline_mm * static_cast<double>(fx) / z_min_mm));
                    if (dead_zone_left_px > W) dead_zone_left_px = W;
                }
                RCLCPP_INFO(logger(),
                            "Reprojection LUT ready: fx=%.2f fy=%.2f cx=%.2f cy=%.2f (ratio_mat=%.3f); "
                            "stereo left dead-zone upper bound ~%d px (baseline=%.2f mm, Z_min=%.0f mm)",
                            fx, fy, cx, cy, ratio_mat,
                            dead_zone_left_px, baseline_mm, z_min_mm);
            }
        }

#ifdef _OPENMP
        // OMP_WAIT_POLICY and GOMP_SPINCOUNT are set in the launch
        // environment; libgomp reads them at the first parallel region.
        // omp_set_num_threads is set here because it overrides the env
        // and can be changed at runtime. Capped at 4 to limit
        // over-decomposition on cache-constrained hosts; the floor of 1
        // covers the rare case where hardware_concurrency() returns 0.
        {
            const unsigned hc = std::thread::hardware_concurrency();
            const int omp_n = std::max(1, std::min(4, static_cast<int>(hc)));
            omp_set_dynamic(0);
            omp_set_num_threads(omp_n);
            RCLCPP_INFO(logger(),
                        "Point-cloud OpenMP workers: %d (hardware_concurrency=%u)",
                        omp_n, hc);
        }
#endif

        while (impl_->running.load(std::memory_order_acquire)) {
          try {

            uint64_t depth_ts = 0;
            std::shared_ptr<const std::vector<uint8_t>> depth_view;
            // Subscriber gate decisions captured here, used both for the
            // early-continue check and again at the publish sites later.
            bool need_depth = false;
            bool need_pc    = false;
            // Filter enable snapshot — taken once at the top of the
            // iteration so a runtime toggle of temporal_filter between
            // here and the body cannot leave depth_publish_buf allocated
            // but unwritten (which would publish a zero-filled depth
            // frame). hole_fill_mode and spatial_filter_enabled are
            // launch-only, but capture them too so all three flags are
            // sampled from a single moment.
            const bool snap_spatial  = impl_->spatial_filter_enabled;
            const bool snap_temporal = impl_->temporal_enabled.load(std::memory_order_acquire);
            const bool snap_holes    = impl_->hole_fill_mode != HoleFillMode::kOff;
            const bool any_filter    = snap_spatial || snap_temporal || snap_holes;
            {
                std::unique_lock<std::mutex> lk(impl_->latest.mtx);
                impl_->latest.cv.wait(lk, [this]{
                    return impl_->latest.depth_pending
                        || !impl_->running.load(std::memory_order_acquire);
                });
                if (!impl_->running.load(std::memory_order_acquire)) break;
                // Clear depth_pending before releasing the lock so a
                // depth frame arriving during reprojection can re-signal
                // the condition variable.
                impl_->latest.depth_pending = false;
                // pc_thread is the depth publisher whenever any filter
                // is active, so it must run on the depth gate too. With
                // all filters off only the pc gate matters.
                need_depth = any_filter
                    && impl_->gate_pass(impl_->depth_gate);
                // Reprojection requires the rectify-derived intrinsics;
                // can_reproject is false when APC_GetRectifyMatLogData
                // failed at open() or fx/fy resolved to zero.
                need_pc    = can_reproject
                    && !u_inv_neg.empty()
                    && impl_->gate_pass(impl_->pc_gate);
                if (!need_depth && !need_pc) {
                    continue;
                }
                // Take a refcount on the latest depth buffer. The depth
                // fetch thread may publish a new shared_ptr later; this
                // local copy keeps the current buffer alive for the
                // duration of this reprojection.
                depth_view = impl_->latest.depth;
                depth_ts   = impl_->latest.depth_ts_us;
            }
            if (!depth_view) continue;

            // When this iteration is going to publish filtered depth
            // (filter active AND a depth subscriber exists), allocate
            // the publish-side byte buffer up front and route the
            // filter sink into it. Projection reads from the same
            // buffer; at the end of the loop the buffer is moved into
            // depth_fb.data with no per-frame copy. When need_depth is
            // false the filter writes through the persistent pc_mm_buf.
            //
            // Invariant: if depth_publish_buf is non-empty here, the
            // W-spatial or Z-domain filter branch below will fully
            // overwrite it before the late publish. The branch selector
            // (spatial vs. z_temporal||z_holes) is exhaustive while
            // any_filter is true, which is the same gate that produced
            // need_depth in this code path.
            std::vector<uint8_t> depth_publish_buf;
            uint16_t* filter_mm_sink = impl_->pc_mm_buf.data();
            if (need_depth && impl_->on_depth) {
                depth_publish_buf.resize(static_cast<size_t>(W) * H * 2);
                filter_mm_sink = reinterpret_cast<uint16_t*>(depth_publish_buf.data());
            }

            const auto t_compute_begin = std::chrono::steady_clock::now();

            constexpr float    kMmToM    = 1.0f / 1000.0f;
            constexpr uint16_t kDepthMask = 0x3FFF;     // Z14 high 2 bits are flags
            constexpr int      kSerialSkipPixels = 8;   // 16-byte SN watermark in row 0
            size_t valid = 0;

            // Three-step reprojection over the depth raster:
            //   1. count valid pixels per row (parallel, NEON on aarch64)
            //   2. prefix-sum to assign each row a contiguous output slot
            //   3. project + write compacted (parallel, scalar inner
            //      loop — empirically faster than a vqtbl2q + vst3q
            //      NEON kernel on the targeted aarch64 hardware)
            // Output is in ROS base convention (X forward, Y left, Z up),
            // metres.
            const uint16_t* d = reinterpret_cast<const uint16_t*>(depth_view->data());

            // Disparity-domain pipeline: promote to Q4, run the
            // 4-direction IIR, optionally chain the temporal filter,
            // then convert each pixel to Z (mm) via the ZD table.
            // The downstream count + reproject loop reads from the
            // resulting mm buffer.
            if (snap_spatial) {
                uint16_t* q4 = impl_->pc_q4_buf.data();
                uint16_t* mm = filter_mm_sink;
                disparity_promote_to_q4(d, q4, W, H);
                // The first kSerialSkipPixels of row 0 are the
                // firmware serial-number watermark. Mark them as
                // holes before the neighborhood filters run; the
                // reprojection loop already skips the same columns
                // via kSerialSkipPixels.
                std::fill_n(q4, kSerialSkipPixels, uint16_t{0});
                spatial_filter_q4(q4, W, H, impl_->spatial_params);
                impl_->spatial_filter_total.fetch_add(
                    1, std::memory_order_relaxed);

                // Temporal filter: snapshot the runtime-controlled
                // enable bit and parameters under the lock, then apply
                // outside the critical section. A pending reset is
                // honoured before the kernel runs so a freshly-enabled
                // filter does not start from stale per-pixel history.
                if (snap_temporal) {
                    TemporalFilterParams tp;
                    bool do_reset = false;
                    {
                        std::lock_guard<std::mutex> lk(impl_->temporal_mtx);
                        tp = impl_->temporal_params_pending;
                        do_reset = impl_->temporal_reset_pending;
                        impl_->temporal_reset_pending = false;
                    }
                    // Q4 domain: shift the raw user delta into Q4 units
                    // (raw_disparity << 4) so it matches the buffer.
                    tp.delta <<= 4;
                    impl_->temporal_state.resize(W, H);
                    if (do_reset) impl_->temporal_state.reset();
                    temporal_filter_apply(q4, W, H, impl_->temporal_state, tp);
                    impl_->temporal_filter_total.fetch_add(
                        1, std::memory_order_relaxed);
                }

                const ZdTable& tbl = impl_->zd_table;

                // ZD lookup result is clamped to the Z14 range so it
                // round-trips unchanged through the downstream
                // `z & 0x3FFF` mask. The depth-range clip is applied
                // in mm in the same loop so the boundary is exact.
                const uint16_t z_min_clip = static_cast<uint16_t>(
                    std::clamp(impl_->max_near_mm, 0.0f, 65535.0f));
                const uint16_t z_max_clip = static_cast<uint16_t>(
                    std::clamp(impl_->max_far_mm,  0.0f, 65535.0f));
                #pragma omp parallel for schedule(static)
                for (int v = 0; v < H; ++v) {
                    const uint16_t* qrow = q4 + static_cast<size_t>(v) * W;
                    uint16_t*       mrow = mm + static_cast<size_t>(v) * W;
                    for (int u = 0; u < W; ++u) {
                        if (qrow[u] == 0) {
                            mrow[u] = 0;
                            continue;
                        }
                        const uint16_t z = static_cast<uint16_t>(
                            std::clamp(zd_lookup_q4(tbl, qrow[u]), 0, 0x3FFF));
                        mrow[u] = (z < z_min_clip || z > z_max_clip) ? 0 : z;
                    }
                }
                d = mm;

                // Z-domain hole filling. Runs after the ZD lookup so
                // both /depth_image and the reprojected cloud see the
                // filled raster. dead_zone_left_px gates the
                // fill_from_left mode only; the around modes are
                // inherently dead-zone safe and ignore it.
                if (snap_holes) {
                    hole_fill_z(mm, W, H,
                                impl_->hole_fill_mode,
                                dead_zone_left_px,
                                impl_->hole_fill_scratch);
                    impl_->hole_fill_total.fetch_add(
                        1, std::memory_order_relaxed);
                }

                // Depth publish runs at the end of the loop so the
                // filter sink can move directly into depth_fb.
            } else {
                // Z-domain pipeline. spatial_filter is off, but the
                // temporal and / or hole-filling kernels may still be
                // enabled on the FW Z14 mm raster. The incoming depth
                // is copied into a mutable buffer, the requested
                // kernels run in place, then on_depth is published
                // here — depth_fetch suppressed its own publish path
                // because at least one filter is active.
                // Use the same snapshot as the publish-buffer gate so a
                // runtime toggle of temporal_filter between the snapshot
                // and this branch cannot leave depth_publish_buf
                // allocated but unwritten.
                const bool z_temporal = snap_temporal;
                const bool z_holes    = snap_holes;
                if (z_temporal || z_holes) {
                    uint16_t* mm = filter_mm_sink;
                    const size_t n = static_cast<size_t>(W) * H;
                    // Strip the Z14 status bits and clamp to the depth
                    // clip range in the same pass. Pixels outside
                    // [z_min_clip, z_max_clip] become 0; downstream
                    // temporal / hole_filling treat them as holes.
                    // The spatial path enforces the same range in mm
                    // domain after the ZD lookup completes.
                    const uint16_t z_min_clip = static_cast<uint16_t>(
                        std::clamp(impl_->max_near_mm, 0.0f, 65535.0f));
                    const uint16_t z_max_clip = static_cast<uint16_t>(
                        std::clamp(impl_->max_far_mm,  0.0f, 65535.0f));
                    for (size_t i = 0; i < n; ++i) {
                        const uint16_t z = d[i] & kDepthMask;
                        mm[i] = (z < z_min_clip || z > z_max_clip) ? 0 : z;
                    }
                    // See the spatial-path comment above: the first
                    // kSerialSkipPixels of row 0 are the serial-number
                    // watermark and must not feed any neighborhood
                    // filter.
                    std::fill_n(mm, kSerialSkipPixels, uint16_t{0});

                    if (z_temporal) {
                        TemporalFilterParams tp;
                        bool do_reset = false;
                        {
                            std::lock_guard<std::mutex> lk(impl_->temporal_mtx);
                            tp = impl_->temporal_params_pending;
                            do_reset = impl_->temporal_reset_pending;
                            impl_->temporal_reset_pending = false;
                        }
                        // Z14 domain: tp.delta is already in mm; no shift.
                        impl_->temporal_state.resize(W, H);
                        if (do_reset) impl_->temporal_state.reset();
                        temporal_filter_apply(mm, W, H, impl_->temporal_state, tp);
                        impl_->temporal_filter_total.fetch_add(
                            1, std::memory_order_relaxed);
                    }

                    if (z_holes) {
                        hole_fill_z(mm, W, H,
                                    impl_->hole_fill_mode,
                                    dead_zone_left_px,
                                    impl_->hole_fill_scratch);
                        impl_->hole_fill_total.fetch_add(
                            1, std::memory_order_relaxed);
                    }

                    d = mm;
                    // Depth publish runs at the end of the loop; see
                    // the note above the W-spatial branch.
                }
            }

            // Point-cloud reprojection runs only when a /pointcloud
            // subscriber is present; the filtered depth raster still
            // publishes from depth_publish_buf at the end of the loop
            // regardless of whether the cloud is consumed.
            uint32_t this_point_step = 0;
            std::shared_ptr<const std::vector<uint8_t>> color_view;
            const uint8_t* color_data = nullptr;
            bool have_color = false;
            uint8_t* work = nullptr;
            if (need_pc) {
                #pragma omp parallel for schedule(static)
                for (int v = 0; v < H; ++v) {
                    const uint16_t* row = d + static_cast<size_t>(v) * W;
                    const int u_start = (v == 0) ? kSerialSkipPixels : 0;
                    row_valid_counts[v] = simd::pc_count_nonzero(
                        row + u_start, W - u_start);
                }

                uint32_t running = 0;
                for (int v = 0; v < H; ++v) {
                    row_offsets[v] = running;
                    running += row_valid_counts[v];
                }
                valid = running;

                // Take a refcount on the most recent color snapshot for the
                // XYZRGB path. Falls back to XYZ-only when the snapshot is
                // not ready or has unexpected dimensions.
                if (impl_->colored_pointcloud) {
                    std::lock_guard<std::mutex> lk(impl_->latest_color.mtx);
                    color_view = impl_->latest_color.rgb;
                }
                have_color = color_view
                    && static_cast<int>(impl_->colored_pointcloud_cu_byte_off.size()) == W
                    && static_cast<int>(impl_->colored_pointcloud_cv_byte_off.size()) == H;
                if (have_color) color_data = color_view->data();
                this_point_step = have_color ? 16u : 12u;

                work = workspace.data();
            }

            if (need_pc && have_color) {
                const int32_t* cu_off = impl_->colored_pointcloud_cu_byte_off.data();
                const int32_t* cv_off = impl_->colored_pointcloud_cv_byte_off.data();
                #pragma omp parallel for schedule(static)
                for (int v = 0; v < H; ++v) {
                    const uint16_t* row = d + static_cast<size_t>(v) * W;
                    const float vn_neg = v_inv_neg[v];
                    uint8_t* dst = work +
                        static_cast<size_t>(row_offsets[v]) * 16;
                    const int u_start = (v == 0) ? kSerialSkipPixels : 0;
                    const uint8_t* color_row = color_data + cv_off[v];
                    for (int u = u_start; u < W; ++u) {
                        const uint16_t z_mm = row[u] & kDepthMask;
                        if (z_mm == 0) continue;
                        const float z_m = static_cast<float>(z_mm) * kMmToM;
                        float* xyz_dst = reinterpret_cast<float*>(dst);
                        xyz_dst[0] = z_m;
                        xyz_dst[1] = u_inv_neg[u] * z_m;
                        xyz_dst[2] = vn_neg       * z_m;
                        const uint8_t* px = color_row + cu_off[u];
                        const uint32_t rgb =
                            (static_cast<uint32_t>(px[0]) << 16) |
                            (static_cast<uint32_t>(px[1]) << 8)  |
                             static_cast<uint32_t>(px[2]);
                        std::memcpy(dst + 12, &rgb, sizeof(uint32_t));
                        dst += 16;
                    }
                }
            } else if (need_pc) {
                #pragma omp parallel for schedule(static)
                for (int v = 0; v < H; ++v) {
                    const uint16_t* row = d + static_cast<size_t>(v) * W;
                    const float vn_neg = v_inv_neg[v];
                    float* dst = reinterpret_cast<float*>(
                        work + static_cast<size_t>(row_offsets[v]) * 12);
                    const int u_start = (v == 0) ? kSerialSkipPixels : 0;
                    for (int u = u_start; u < W; ++u) {
                        const uint16_t z_mm = row[u] & kDepthMask;
                        if (z_mm == 0) continue;
                        const float z_m = static_cast<float>(z_mm) * kMmToM;
                        dst[0] = z_m;                 // forward (optical Z)
                        dst[1] = u_inv_neg[u] * z_m;  // left   (-(u-cx)/fx * z)
                        dst[2] = vn_neg       * z_m;  // up     (-(v-cy)/fy * z)
                        dst += 3;
                    }
                }
            }

            if (need_pc) {
                if (valid == 0) {
                    RCLCPP_WARN_ONCE(logger(),
                                     "PointCloud compaction kept 0 of %zu points "
                                     "(depth clip [%.0f..%.0f] mm applied upstream)",
                                     pc_points,
                                     impl_->max_near_mm, impl_->max_far_mm);
                } else if (impl_->on_pc) {
                    // Copy only the populated prefix of the workspace into a
                    // fresh publish buffer. std::vector's (InputIt, InputIt)
                    // constructor uses uninitialized_copy → memcpy for trivial
                    // types, with no intermediate value-init. Workspace itself
                    // is retained for the next iteration.
                    const size_t valid_bytes = static_cast<size_t>(valid)
                                             * this_point_step;
                    std::vector<uint8_t> msg_buf(workspace.begin(),
                                                 workspace.begin() + valid_bytes);
                    impl_->on_pc(std::move(msg_buf),
                                 static_cast<uint32_t>(valid),
                                 this_point_step,
                                 depth_ts);
                }
            }

            // Publish the filtered depth raster after projection has finished
            // reading from the same buffer. depth_publish_buf is only non-empty
            // when the filter pipeline routed its sink there at the top of the
            // iteration; one of the two filter branches above will have fully
            // overwritten the buffer by this point. The on_depth check mirrors
            // the precondition at allocation time and protects against late
            // teardown.
            if (!depth_publish_buf.empty() && impl_->on_depth) {
                FrameBuffer depth_fb;
                depth_fb.width  = W;
                depth_fb.height = H;
                depth_fb.hw_timestamp_us = depth_ts;
                depth_fb.data = std::move(depth_publish_buf);
                impl_->depth_publish_total.fetch_add(1, std::memory_order_relaxed);
                impl_->on_depth(std::move(depth_fb));
            }

            // Rolling per-frame compute-time stats. Only update when the
            // point-cloud path actually computed something this iteration.
            if (need_pc && valid > 0) {
                const auto t_compute_end = std::chrono::steady_clock::now();
                const uint64_t us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                        t_compute_end - t_compute_begin).count());
                impl_->pc_compute_sum_us.fetch_add(us, std::memory_order_relaxed);
                const uint64_t n = impl_->pc_publish_total.fetch_add(1, std::memory_order_relaxed) + 1;
                uint64_t prev_max = impl_->pc_compute_max_us.load(std::memory_order_relaxed);
                while (us > prev_max &&
                       !impl_->pc_compute_max_us.compare_exchange_weak(prev_max, us)) {}
                // Periodic INFO log every ~10s @ 30 Hz, decoupled from diag topic.
                if (n > 0 && (n % 300) == 0) {
                    const uint64_t total = impl_->pc_compute_sum_us.load(std::memory_order_relaxed);
                    RCLCPP_INFO(logger(),
                                "PC compute cumulative %lu frames: avg=%.2f ms (~%.1f Hz capacity)",
                                n, (total / 1000.0) / n, n * 1000000.0 / total);
                }
            }
          } catch (const std::bad_alloc& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "pc_thread: bad_alloc (%s); dropping frame, continuing",
                                 e.what());
          } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                 "pc_thread: unhandled exception (%s); dropping frame, continuing",
                                 e.what());
          }
        }
    });

    RCLCPP_INFO(logger(),
                "Streaming threads up: %scolor_fetch + depth_fetch + pc (XYZ-only).",
                impl_->color_stream_present ? "" : "(no color) ");
}

EspdiDevice::StreamState EspdiDevice::stream_state() const {
    return impl_->stream_state.load(std::memory_order_relaxed);
}

EspdiDevice::Stats EspdiDevice::stats() const {
    Stats s;
    s.color_input_total    = impl_->color_input_total.load(std::memory_order_relaxed);
    s.depth_input_total    = impl_->depth_input_total.load(std::memory_order_relaxed);
    s.color_input_dropped  = impl_->color_input_dropped.load(std::memory_order_relaxed);
    s.depth_input_dropped  = impl_->depth_input_dropped.load(std::memory_order_relaxed);
    s.color_publish_total  = impl_->color_publish_total.load(std::memory_order_relaxed);
    s.depth_publish_total  = impl_->depth_publish_total.load(std::memory_order_relaxed);
    s.color_decode_sum_us  = impl_->color_decode_sum_us.load(std::memory_order_relaxed);
    s.color_decode_max_us  = impl_->color_decode_max_us.load(std::memory_order_relaxed);
    s.pc_publish_total     = impl_->pc_publish_total.load(std::memory_order_relaxed);
    s.pc_compute_sum_us    = impl_->pc_compute_sum_us.load(std::memory_order_relaxed);
    s.pc_compute_max_us    = impl_->pc_compute_max_us.load(std::memory_order_relaxed);
    s.spatial_filter_total = impl_->spatial_filter_total.load(std::memory_order_relaxed);
    s.temporal_filter_total = impl_->temporal_filter_total.load(std::memory_order_relaxed);
    s.hole_fill_total      = impl_->hole_fill_total.load(std::memory_order_relaxed);
    return s;
}

std::string EspdiDevice::serial_number() const {
    std::lock_guard<std::mutex> lk(impl_->lifecycle_mtx);
    return impl_->serial_number;
}
std::string EspdiDevice::usb_port() const {
    std::lock_guard<std::mutex> lk(impl_->lifecycle_mtx);
    return impl_->usb_port;
}
int         EspdiDevice::actual_fps()        const { return impl_->actual_fps.load(std::memory_order_relaxed); }

bool EspdiDevice::pause(bool on) {
    // Pause flips stream_state; fetch threads keep running, drain USB,
    // and skip decode/filter/publish on the next iteration. Resume is
    // observed on the next frame (~33 ms @ 30 fps).
    //
    // Shares lifecycle_mtx with standby() so the two cannot race;
    // sub-µs uncontended and pause is not a hot-path control.
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    const StreamState cur = impl_->stream_state.load(std::memory_order_relaxed);
    if (cur == StreamState::Standby) {
        // Record the intent for the next standby(false) without touching
        // the SDK pipe.
        impl_->pause_pending.store(on, std::memory_order_relaxed);
        return true;
    }
    const StreamState target = on ? StreamState::Paused : StreamState::Active;
    if (cur == target) return true;
    impl_->stream_state.store(target, std::memory_order_release);
    RCLCPP_INFO(logger(), "Stream state: %s", on ? "Paused" : "Active");
    return true;
}

bool EspdiDevice::standby(bool on) {
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    const StreamState cur = impl_->stream_state.load(std::memory_order_relaxed);
    const bool already_standby = (cur == StreamState::Standby);
    if (on == already_standby) return true;

    if (on) {
        // Active|Paused -> Standby: remember whether the caller was paused
        // so standby(false) can land back in the same state, then tear the
        // SDK pipe down. pause() takes the same lifecycle_mtx, so cur
        // reflects the operator's intent at the moment standby was
        // dispatched and cannot drift while we hold the lock.
        impl_->pause_pending.store(cur == StreamState::Paused,
                                   std::memory_order_relaxed);
        impl_->running.store(false, std::memory_order_release);
        impl_->latest.cv.notify_all();
        if (impl_->color_fetch.joinable()) impl_->color_fetch.join();
        if (impl_->depth_fetch.joinable()) impl_->depth_fetch.join();
        if (impl_->pc_thread.joinable())   impl_->pc_thread.join();
        // Join the DM_Quality register-apply worker as well. It is spawned
        // from the first-depth-frame trigger and writes to the SDK handle;
        // if it is still in flight when APC_CloseDevice runs the writes
        // race the close, and on the next standby(false) the residual
        // writes hit the new session.
        if (impl_->dm_quality_worker.joinable()) impl_->dm_quality_worker.join();
        {
            // Drop the staged depth pointer (open()-symmetric reset) so a
            // stale frame cannot republish across a Standby cycle. The
            // shared_ptr release is cheap and matches the cleanup in
            // EspdiDevice::open() so the resume path starts from a known
            // empty state.
            std::lock_guard<std::mutex> latest_lk(impl_->latest.mtx);
            impl_->latest.depth.reset();
            impl_->latest.depth_pending = false;
        }
        {
            std::lock_guard<std::mutex> lk(impl_->latest_color.mtx);
            impl_->latest_color.rgb.reset();
            impl_->latest_color.w = 0;
            impl_->latest_color.h = 0;
            impl_->latest_color.ts_us = 0;
        }
        // Close the USB pipe but keep the SDK handle so calibration,
        // ZD table, register cache, etc. survive into the next resume.
        APC_CloseDevice(impl_->handle, &impl_->sel);
        impl_->stream_state.store(StreamState::Standby,
                                  std::memory_order_release);
        RCLCPP_INFO(logger(), "Stream state: Standby (USB pipe closed)");
        return true;
    }

    // Standby -> Active|Paused. Replay the same SDK init sequence as
    // open() (minus APC_Init / APC_GetDeviceInfo / GetRectifyMatLogData,
    // which survive APC_CloseDevice). Order matters: SetupBlock and
    // SetDepthDataType must precede SetInterleaveMode; v4l2_requestbuffers
    // must precede OpenDevice2 so the V4L2 queue is large enough for the
    // configured fps. Missing the requestbuffers step caps the depth pump
    // at the default ~3-buffer queue and frames stall.
    const auto& cfg = impl_->cfg;
    APC_SetupBlock(impl_->handle, &impl_->sel, false);
    if (impl_->depth_stream_present) {
        const int dtype_rc = APC_SetDepthDataType(
            impl_->handle, &impl_->sel, cfg.depth_data_type);
        if (dtype_rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "standby(false): APC_SetDepthDataType(%d) rc=%d",
                        cfg.depth_data_type, dtype_rc);
        }
    }
    // 32 V4L2 buffers — same headroom that open() requests; without this
    // the V4L2 queue defaults to a depth too small for 60 fps interleave
    // modes and the depth fetch starves until the watchdog reconnect runs
    // the full sequence again.
    {
        const int rb_rc = APC_Setup_v4l2_requestbuffers(
            impl_->handle, &impl_->sel, 32);
        if (rb_rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "standby(false): APC_Setup_v4l2_requestbuffers(32) rc=%d",
                        rb_rc);
        }
    }
    if (cfg.interleave) {
        const int il_rc = APC_SetInterleaveMode(impl_->handle, &impl_->sel, true);
        if (il_rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "standby(false): APC_SetInterleaveMode(true) rc=%d", il_rc);
        }
    }
    // Apply IR before APC_OpenDevice2 so the V4L2 capture buffer fills at
    // the configured illumination from the first frame. Same convention as
    // open(); a runtime ir_intensity override is re-applied separately by
    // the standby service handler after this function returns.
    {
        const int level = (cfg.ir_intensity >= 0)
            ? cfg.ir_intensity
            : default_ir_level_for_pid(impl_->dev_info.wPID);
        APC_SetCurrentIRValue(impl_->handle, &impl_->sel,
                              static_cast<unsigned short>(level));
    }
    int actual_fps = cfg.framerate;
    const int cw = impl_->color_stream_present ? cfg.color_width  : 0;
    const int ch = impl_->color_stream_present ? cfg.color_height : 0;
    const int dw = impl_->depth_stream_present ? cfg.depth_width  : 0;
    const int dh = impl_->depth_stream_present ? cfg.depth_height : 0;
    const int rc = APC_OpenDevice2(
        impl_->handle, &impl_->sel,
        cw, ch, static_cast<bool>(cfg.color_format),
        dw, dh, DEPTH_IMG_NON_TRANSFER,
        /*bIsOutputRGB24=*/true, /*phWndNotice=*/nullptr,
        &actual_fps, IMAGE_SN_SYNC);
    if (rc != APC_OK) {
        RCLCPP_ERROR(logger(),
                     "standby(false): APC_OpenDevice2(c=%dx%d, d=%dx%d) failed rc=%d ; device closed",
                     cw, ch, dw, dh, rc);
        impl_->opened = false;
        return false;
    }
    impl_->actual_fps.store(actual_fps, std::memory_order_relaxed);
    // Temporal filter history is invalidated by the gap — request a reset
    // so the next frame starts from a fresh per-pixel history.
    if (impl_->temporal_enabled.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(impl_->temporal_mtx);
        impl_->temporal_reset_pending = true;
    }
    const bool resume_paused = impl_->pause_pending.load(std::memory_order_relaxed);
    impl_->stream_state.store(resume_paused ? StreamState::Paused
                                            : StreamState::Active,
                              std::memory_order_release);
    impl_->running.store(true, std::memory_order_release);
    spawn_fetch_threads_();
    RCLCPP_INFO(logger(), "Stream state: %s (USB pipe reopened, fps=%d)",
                resume_paused ? "Paused" : "Active", actual_fps);
    return true;
}

void EspdiDevice::stop() {
    if (!impl_) return;
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    // Per-thread INFO lines help diagnose stalls if join blocks.
    auto join_named = [](std::thread& t, const char* name) {
        if (!t.joinable()) return;
        RCLCPP_INFO(logger(), "stop(): joining %s...", name);
        t.join();
        RCLCPP_INFO(logger(), "stop(): %s joined", name);
    };
    // The DM_Quality worker can be in flight even when running was never
    // flipped to true (e.g. it was launched from a callback before
    // start() ran). Always join it before returning so destruction does
    // not race the worker dereferencing impl_.
    if (impl_->running.exchange(false)) {
        RCLCPP_INFO(logger(), "stop(): signalling fetch threads to exit");
        {
            std::lock_guard<std::mutex> latest_lk(impl_->latest.mtx);
            impl_->latest.depth_pending = false;
        }
        impl_->latest.cv.notify_all();
        join_named(impl_->color_fetch, "color_fetch");
        join_named(impl_->depth_fetch, "depth_fetch");
        join_named(impl_->pc_thread,   "pc_thread");
    }
    join_named(impl_->dm_quality_worker, "dm_quality_worker");
}

EspdiDevice::Calibration EspdiDevice::calibration() const {
    return impl_->calib;
}

void EspdiDevice::set_pc_gate(PointCloudGate gate) {
    // The PC thread reads pc_gate without locking. The function-object copy
    // is small; a torn read can at worst produce one misclassified iteration,
    // which is acceptable on this path.
    impl_->pc_gate = std::move(gate);
}

void EspdiDevice::set_color_gate(FrameStreamGate gate) {
    impl_->color_gate = std::move(gate);
}

void EspdiDevice::set_depth_gate(FrameStreamGate gate) {
    impl_->depth_gate = std::move(gate);
}

bool EspdiDevice::set_ir_intensity(int value) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    // Negative value = use per-PID default resolved at open().
    const char* origin = "explicit";
    if (value < 0) {
        value = impl_->ir_default_level;
        origin = "default";
    }
    if (impl_->ir_range_valid && value > impl_->ir_max_fw) {
        RCLCPP_WARN(logger(),
                    "ir_intensity=%d exceeds FW max %d ; clamping",
                    value, impl_->ir_max_fw);
        value = impl_->ir_max_fw;
    }
    const unsigned short v = static_cast<unsigned short>(value);
    const int rc = APC_SetCurrentIRValue(impl_->handle, &impl_->sel, v);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetCurrentIRValue(%u) rc=%d", v, rc);
        return false;
    }
    RCLCPP_INFO(logger(),
                "ir_intensity -> %u (%s)", v, origin);
    return true;
}

EspdiDevice::TemperatureReading EspdiDevice::read_temperature() const {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    TemperatureReading t;
    if (!impl_->opened || !impl_->handle) return t;
    // G100+ and G100+i carry the on-die thermal sensor at sensor-register
    // ID 0x90 (hardware identical between the two variants).
    const unsigned short pid = impl_->dev_info.wPID;
    if (pid != APC_PID_80362 && pid != APC_PID_IRIS) {
        return t;  // supported=false
    }
    t.supported = true;

    constexpr int kThermalSensorId = 0x90;
    unsigned short reg = 0;
    const int rc = APC_GetSensorRegister(
        impl_->handle, &impl_->sel, kThermalSensorId,
        /*address=*/0x00, &reg,
        FG_Address_1Byte | FG_Value_2Byte,
        SENSOR_BOTH);
    if (rc != APC_OK) {
        return t;  // supported=true, read_ok=false
    }

    // Byte-swap (sensor returns big-endian), take 11-bit signed value
    // from bits 15:5, scale 0.125 °C / LSB. The sign-extend uses the
    // standard XOR-then-subtract pattern so it stays implementation-
    // defined-free across narrowing casts.
    const uint32_t swapped =
        ((static_cast<uint32_t>(reg) >> 8) & 0x00FFu) |
        ((static_cast<uint32_t>(reg) << 8) & 0xFF00u);
    const uint32_t v = (swapped >> 5) & 0x7FFu;        // 11 bits
    const int32_t  signed11 = static_cast<int32_t>(v ^ 0x400u) - 0x400;
    t.celsius = static_cast<float>(signed11) * 0.125f;
    t.read_ok = true;
    return t;
}

bool EspdiDevice::set_temporal_filter(bool enabled, double alpha,
                                      int delta, int persistence) {
    // The temporal filter runs in whichever domain has a uint16 raster
    // available (D11 disparity when spatial_filter is active, Z14 mm
    // otherwise). delta is stored raw here and converted at the
    // pc_thread call site so the same field serves both pipelines.
    TemporalFilterParams tp;
    const double a = std::clamp(alpha, 0.0, 1.0);
    tp.alpha_q8    = static_cast<int>(std::lround(a * 256.0));
    // Clamp keeps the Q4 promote (`<<= 4`) inside uint16.
    tp.delta       = std::clamp(delta, 1, 4095);
    tp.persistence = std::clamp(persistence, 0, 8);

    bool was_enabled = false;
    {
        std::lock_guard<std::mutex> lk(impl_->temporal_mtx);
        was_enabled = impl_->temporal_enabled.load(std::memory_order_relaxed);
        impl_->temporal_params_pending = tp;
        if (enabled && !was_enabled) {
            impl_->temporal_reset_pending = true;
        }
    }
    impl_->temporal_enabled.store(enabled, std::memory_order_release);
    RCLCPP_INFO(logger(),
                "Temporal filter %s: alpha=%.2f delta=%d persistence=%d (%s domain)%s",
                enabled ? "ON" : "OFF",
                a, delta, tp.persistence,
                impl_->spatial_filter_enabled ? "D11 disparity" : "Z14 mm",
                (enabled && !was_enabled) ? " (history reset)" : "");
    return true;
}

bool EspdiDevice::set_auto_exposure(bool enable) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    // Auto-exposure is switched via the UVC CT AUTO_EXPOSURE_MODE register:
    //   manual = 1 (AE_MOD_MANUAL_MODE)
    //   auto   = 3 (AE_MOD_APERTURE_PRIORITY_MODE)
    // The CT path also unlocks subsequent manual EXPOSURE_TIME_ABSOLUTE
    // writes when AE is set to manual.
    const long int mode = enable ? AE_MOD_APERTURE_PRIORITY_MODE
                                 : AE_MOD_MANUAL_MODE;
    const int rc = APC_SetCTPropVal(impl_->handle, &impl_->sel,
                                    CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, mode);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetCTPropVal(AE_MODE, %s) rc=%d",
                    enable ? "auto" : "manual", rc);
        return false;
    }
    RCLCPP_INFO(logger(), "auto_exposure -> %s",
                enable ? "on" : "off");
    return true;
}

bool EspdiDevice::set_exposure_time_step(int step) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    // Manual exposure is applied via UVC CT EXPOSURE_TIME_ABSOLUTE. The value
    // is a signed log-step (negative = darker, positive = brighter).
    // Effective only when auto-exposure is set to manual.
    const int rc = APC_SetCTPropVal(impl_->handle, &impl_->sel,
                                    CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL,
                                    static_cast<long int>(step));
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetCTPropVal(EXPOSURE_TIME_ABSOLUTE, %d) rc=%d", step, rc);
        return false;
    }
    RCLCPP_INFO(logger(), "exposure_time_step -> %d", step);
    return true;
}

bool EspdiDevice::set_auto_white_balance(bool enable) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    // Auto white-balance is switched via the UVC PU WHITE_BALANCE_AUTO_CTRL
    // register. On most depth-camera firmware AWB is fixed at the hardware
    // level: the setter returns success but the live state does not change.
    const int rc = APC_SetPUPropVal(impl_->handle, &impl_->sel,
                                    PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL,
                                    enable ? 1 : 0);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetPUPropVal(AWB_AUTO, %s) rc=%d",
                    enable ? "on" : "off", rc);
        return false;
    }
    RCLCPP_INFO(logger(), "auto_white_balance -> %s",
                enable ? "on" : "off");
    return true;
}

bool EspdiDevice::set_power_line_frequency(int mode) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (mode < 0 || mode > 3) {
        RCLCPP_WARN(logger(),
                    "set_power_line_frequency: mode must be 0/1/2/3, got %d", mode);
        return false;
    }
    const int rc = APC_SetPUPropVal(impl_->handle, &impl_->sel,
                                    PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL,
                                    mode);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetPUPropVal(POWER_LINE_FREQ, %d) rc=%d", mode, rc);
        return false;
    }
    const char* label = (mode == 0) ? "disabled"
                      : (mode == 1) ? "50Hz"
                      : (mode == 2) ? "60Hz" : "auto";
    RCLCPP_INFO(logger(),
                "power_line_frequency -> %d (%s)", mode, label);
    return true;
}

EspdiDevice::RuntimeState EspdiDevice::read_runtime_state() const {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    RuntimeState s;
    if (!impl_->opened || !impl_->handle) return s;

    unsigned short ir_cur = 0;
    if (APC_GetCurrentIRValue(impl_->handle, &impl_->sel, &ir_cur) == APC_OK) {
        s.ir_intensity = static_cast<int>(ir_cur);
        s.ir_read_ok = true;
    }
    long int ae_mode = 0;
    if (APC_GetCTPropVal(impl_->handle, &impl_->sel,
                         CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &ae_mode) == APC_OK) {
        // wrapper convention: manual=1, auto=3 (aperture priority)
        s.auto_exposure = (ae_mode != AE_MOD_MANUAL_MODE);
        s.auto_exposure_read_ok = true;
    }
    long int exp = 0;
    if (APC_GetCTPropVal(impl_->handle, &impl_->sel,
                         CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL, &exp) == APC_OK) {
        s.exposure_time_step = static_cast<int>(exp);
        s.exposure_read_ok = true;
    }
    long int awb = 0;
    if (APC_GetPUPropVal(impl_->handle, &impl_->sel,
                         PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, &awb) == APC_OK) {
        s.auto_white_balance = (awb != 0);
        s.awb_read_ok = true;
    }
    long int plf = 0;
    if (APC_GetPUPropVal(impl_->handle, &impl_->sel,
                         PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, &plf) == APC_OK) {
        s.power_line_frequency = static_cast<int>(plf);
        s.plf_read_ok = true;
    }
    return s;
}

void EspdiDevice::apply_dm_quality_register_setting_async(const std::string& cfg_dir) {
    if (!impl_->opened || impl_->handle == nullptr) {
        RCLCPP_WARN(logger(),
                    "apply_dm_quality_register_setting_async: device not open, skipping");
        return;
    }
    // Wait for any previous worker to finish before launching a new one so
    // there is at most one in flight per device.
    if (impl_->dm_quality_worker.joinable()) {
        impl_->dm_quality_worker.join();
    }
    void* handle = impl_->handle;
    const int index = impl_->sel.index;
    const unsigned short pid = impl_->dev_info.wPID;
    impl_->dm_quality_worker_running.store(true, std::memory_order_release);
    impl_->dm_quality_worker = std::thread([this, handle, index, pid, cfg_dir]() {
        apply_dm_quality_register_setting(handle, index, pid, cfg_dir, impl_->sdk_mtx);
        impl_->dm_quality_worker_running.store(false, std::memory_order_release);
    });
}

}  // namespace eys3d_camera
