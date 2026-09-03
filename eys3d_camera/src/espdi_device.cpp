#include "eys3d_camera/espdi_device.hpp"

#include "eys3d_camera/video_modes.hpp"

#include "espdi_error.hpp"
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
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
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

// Serializes device open / probe across every EspdiDevice in the process. The
// eSPDI SDK keeps per-process device bookkeeping, so two cameras setting up at
// once can interfere. A timed_mutex bounds the wait: a camera stuck in a
// mid-open SDK call (e.g. unplugged during APC_OpenDevice2) cannot block the
// others forever — they time out and report instead.
std::timed_mutex& device_setup_lock() {
    static std::timed_mutex m;
    return m;
}
constexpr int kSetupTurnTimeoutMs = 120000;

// Save and restore the process SIGINT/SIGTERM handlers across a scope.
// APC_Init installs its own exit()-on-delivery handler; this keeps it from
// outliving the open()/probe span.
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

// Per-model constants (IR default, depth range, PID, mono) come from the
// video-mode catalogue header via DeviceConfig — see
// launch/video_modes/<MODEL>.yaml.

// video_modes.hpp is installed and eSPDI_def.h is not, so color_is_rectified()
// carries these as literals. Pinned here so a renumbering breaks the build.
static_assert(APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET == 16 &&
              APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET == 32,
              "APC_DEPTH_DATA_* offsets moved; revisit color_is_rectified()");
static_assert(APC_DEPTH_DATA_8_BITS == 1 && APC_DEPTH_DATA_14_BITS == 2 &&
              APC_DEPTH_DATA_8_BITS_x80 == 3 && APC_DEPTH_DATA_11_BITS == 4 &&
              APC_DEPTH_DATA_OFF_RECTIFY == 5 &&
              APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY == 11 &&
              APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY == 13,
              "APC_DEPTH_DATA_* rectify codes moved; revisit color_is_rectified()");

// Backoff applied on APC_DEVICE_TIMEOUT to avoid busy-spinning.
constexpr int kTimeoutBackoffUs = 100;

// Backoff on any other SDK error. A persistent one -- APC_OPEN_DEVICE_FAIL
// repeats on every call -- would otherwise spin a core until the reconnect.
constexpr int kErrorBackoffUs = 2000;

// The firmware stamps a 16-byte serial-number watermark over the first
// 8 pixels of depth row 0. Those pixels carry no distance: every publish
// path zeroes them and reprojection skips the same columns, so the
// raster and the cloud agree on which pixels exist.
constexpr int kSerialSkipPixels = 8;

// Window after start() during which the per-stream subscriber gates are
// bypassed. DDS discovery can take several hundred milliseconds; this
// grace period lets late subscribers receive the initial frames.
constexpr int kGatePassThroughMs = 3000;

// Cached logger: rclcpp::get_logger() hashes the name and allocates on
// every call, and the THROTTLE macros invoke it unconditionally.
const rclcpp::Logger& logger() {
    static const rclcpp::Logger kLogger = rclcpp::get_logger("EspdiDevice");
    return kLogger;
}

// One-shot stdout marker emitted on the first frame from either fetch
// thread so launch tooling can detect pipeline readiness. Written in a
// single std::cout call so concurrent wakeups from the two fetch threads
// cannot interleave on stdout.
void emit_ready_marker() {
    std::cout << "EYS3D_CAMERA_READY\n[eys3d_camera] streaming." << std::endl;
    std::cout.flush();
}

// Shared clock for RCLCPP_*_THROTTLE in the fetch loops. Static, not a
// per-call make_shared: the THROTTLE macro evaluates its clock argument on
// every expansion.
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

// The published image is always rgb8 (3 bytes/pixel), the encoding the
// perception stack consumes natively. Monochrome modules (G62 / R77)
// decode to one gray plane and replicate it to exact-gray rgb8.
size_t color_rgb8_bytes(int w, int h) {
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
}

// YUYV → rgb8 conversion lives in simd_kernels.{hpp,cpp}. AArch64 picks
// the NEON intrinsic kernel; x86_64 / other archs use the scalar +
// OpenMP fallback. Both produce identical bytes (BT.601 limited range,
// 8.8 fixed point, round-to-nearest via vqrshrun).
using simd::yuyv_to_rgb8;


// APC_GetSerialNumber returns UTF-16 LE: two bytes per character, low byte
// first. eYs3D serials are ASCII, so every even index is the character.
// sn_len is a byte count and includes the terminating NUL on some modules.
std::string decode_serial(const unsigned char* buf, int sn_len, size_t buf_size) {
    const int chars = std::min(sn_len / 2, static_cast<int>(buf_size / 2));
    std::string out(static_cast<size_t>(std::max(chars, 0)), '\0');
    for (int j = 0; j < chars; ++j) out[j] = static_cast<char>(buf[j * 2]);
    while (!out.empty() && out.back() == '\0') out.pop_back();
    return out;
}

// Resolve "/dev/videoN" to its USB topology path ("2-3:1.0") via
// /sys/class/video4linux. Returns empty string on non-USB devices or
// sysfs failures. Stable across reboots; matches udev and lsusb output.
std::string resolve_usb_port(const std::string& v4l2_path) {
    const std::string dev_prefix = "/dev/video";
    if (v4l2_path.compare(0, dev_prefix.size(), dev_prefix) != 0) return {};
    const std::string vname = v4l2_path.substr(5);  // "video2"
    const std::string sysfs_link = "/sys/class/video4linux/" + vname + "/device";

    // Not std::filesystem::canonical(): <filesystem> needs GCC 8 and the
    // bundled eSPDI SDK is built with 7.5.
    std::unique_ptr<char, decltype(&std::free)> resolved(
        ::realpath(sysfs_link.c_str(), nullptr), &std::free);
    if (!resolved) return {};

    // Split the resolved path into its components, deepest last.
    std::vector<std::string> parts;
    {
        const std::string real(resolved.get());
        std::string::size_type start = 0;
        while (start <= real.size()) {
            const auto slash = real.find('/', start);
            const auto end = (slash == std::string::npos) ? real.size() : slash;
            if (end > start) parts.emplace_back(real, start, end - start);
            if (slash == std::string::npos) break;
            start = slash + 1;
        }
    }

    // Walk the components from deepest to root; return the first that matches
    // the USB interface pattern. Pattern allows hubs ("2-1.4.2:1.0").
    static const std::regex kUsbIfacePattern(R"(^\d+-\d+(?:\.\d+)*:\d+\.\d+$)");
    static const std::regex kUsbDevicePattern(R"(^\d+-\d+(?:\.\d+)*$)");
    // Prefer the more specific :config.interface form, fall back to the device
    // form. Two passes keep priority deterministic.
    for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
        if (std::regex_match(*it, kUsbIfacePattern)) return *it;
    }
    for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
        if (std::regex_match(*it, kUsbDevicePattern)) return *it;
    }
    return {};
}

// Depth buffer size in bytes. Every depth data type any supported eYs3D
// module produces is 2 bytes/pixel, and the SDK sizes its callback buffer at
// w*h*2 regardless of type; a frame that does not match is dropped by
// ingest_depth rather than reinterpreted.
size_t depth_buffer_bytes(int w, int h) {
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
    float depth_near_mm = 0.0f;
    float depth_far_mm  = 0.0f;

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

    // G2/G1 flash-bank offset (0 or 5) from FW register 0xF6; added to zd_index
    // for every calibration read. Resolved in open().
    int calib_bank_offset = 0;

    // Optional self-calibration session bound to `handle`: created in open() when
    // cfg.selfcal_enable is set, fed by depth_fetch, reset in close() before the
    // handle is released. Null otherwise; thread-safety in SelfCalManager.
    std::unique_ptr<SelfCalManager> selfcal_;
    // Pre-session snapshot of the cy_R register (the only register a cy session
    // dithers), taken by start_selfcal so a worse or abandoned run can be rolled
    // back. Valid = a session is holding it and has not yet been resolved
    // (reverted / kept / committed); close() rolls back if still valid.
    unsigned short cy_snapshot_lo = 0;
    unsigned short cy_snapshot_hi = 0;
    bool cy_snapshot_valid = false;
    // Diagnostics + service reads run on the rclcpp executor; lifecycle
    // writes run on the watchdog timer or a service callback. Atomic to
    // avoid torn reads of these scalars on a multi-threaded executor.
    std::atomic<int> actual_fps{0};

    int ir_max_fw = 0;
    bool ir_range_valid = false;
    int ir_default_level = 0;  // mode-resolved at open()
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
    // Guards dm_quality_worker and the opened/handle check in
    // apply_dm_quality_register_setting_async(). Lock order: lifecycle_mtx
    // first (this one is never held across a fetch-thread join).
    std::mutex dm_quality_mtx;

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

    // Health counters (atomic, single-writer / single-reader). Reset only at
    // open(), so they are cumulative since open. input_total ticks once per
    // successful SDK frame; publish_total once per frame that survives the
    // subscriber gate and reaches the publish callback.
    std::atomic<uint64_t> color_input_total{0};
    std::atomic<uint64_t> depth_input_total{0};
    std::atomic<uint64_t> color_publish_total{0};
    std::atomic<uint64_t> depth_publish_total{0};
    // Frames lost in the USB / SDK layer (detected as a forward gap in
    // the FW frame-number sequence after the interleave parity filter).
    std::atomic<uint64_t> color_input_dropped{0};
    std::atomic<uint64_t> depth_input_dropped{0};
    // Last accepted frame number per stream. -1 = first frame after spawn.
    std::atomic<int> last_color_frame{-1};
    std::atomic<int> last_depth_frame{-1};
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

std::optional<int> EspdiDevice::probe_usb_type(const DeviceConfig& cfg) {
    // Uses a throwaway handle; the selection precedence below mirrors open()
    // so the probe reports the link of the device open() will bind.
    //
    // Take the process-wide setup turn (bounded) — this probe does its own
    // APC_Init/enumeration and must serialize with open()s of other cameras.
    std::unique_lock<std::timed_mutex> setup_lk(
        device_setup_lock(), std::chrono::milliseconds(kSetupTurnTimeoutMs));
    if (!setup_lk.owns_lock()) {
        RCLCPP_ERROR(logger(),
                     "probe_usb_type(): timed out waiting for the device-setup turn");
        return std::nullopt;
    }
    // Scope the APC signal handlers to the APC_Init..APC_Release span, as
    // open() does (see SignalHandlerGuard). The signature-auto default
    // (mode_id=-1) runs this probe on every launch, so an unguarded APC_Init
    // would leave its handler installed afterward.
    SignalHandlerGuard signal_guard;
    void* handle = nullptr;
    if (APC_Init(&handle, /*bIsLogEnabled=*/false) != APC_OK || handle == nullptr)
        return std::nullopt;
    const int dev_count = APC_GetDeviceNumber(handle);
    if (dev_count <= 0) { APC_Release(&handle); return std::nullopt; }

    struct DevEnum { std::string sn, node, port; unsigned short pid = 0; };
    auto enumerate = [&](int i) {
        DEVSELINFO dev_sel{i};
        DEVINFORMATION info{};
        unsigned char sn_buf[256] = {0};
        int sn_len = 0;
        DevEnum d;
        if (APC_GetSerialNumber(handle, &dev_sel, sn_buf, sizeof(sn_buf), &sn_len) == APC_OK
            && sn_len > 0) {
            d.sn = decode_serial(sn_buf, sn_len, sizeof(sn_buf));
        }
        // Same transient-failure retry as open(): a zeroed descriptor empties
        // usb_port and drops the PID, breaking the match below.
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (APC_GetDeviceInfo(handle, &dev_sel, &info) == APC_OK) {
                if (info.strDevName) d.node.assign(info.strDevName);
                d.pid = info.wPID;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        d.port = resolve_usb_port(d.node);
        return d;
    };

    int idx = -1;
    if (!cfg.serial_number.empty() || !cfg.usb_port.empty()) {
        for (int i = 0; i < dev_count; ++i) {
            const auto d = enumerate(i);
            const bool sn_ok  = cfg.serial_number.empty() ||
                                d.sn.find(cfg.serial_number) != std::string::npos;
            const bool bus_ok = cfg.usb_port.empty() ||
                                (!d.port.empty() && d.port.find(cfg.usb_port) != std::string::npos);
            if (sn_ok && bus_ok) { idx = i; break; }
        }
    } else if (cfg.expected_pid != 0) {
        for (int i = 0; i < dev_count; ++i) {
            if (enumerate(i).pid == cfg.expected_pid) { idx = i; break; }
        }
    } else {
        idx = 0;
    }
    if (idx < 0) { APC_Release(&handle); return std::nullopt; }

    DEVSELINFO sel{idx};
    USB_PORT_TYPE port_type = USB_PORT_TYPE_UNKNOW;
    const int rc = APC_GetDevicePortType(handle, &sel, &port_type);
    APC_Release(&handle);
    if (rc != APC_OK) return std::nullopt;
    if (port_type == USB_PORT_TYPE_3_0) return 3;
    if (port_type == USB_PORT_TYPE_2_0) return 2;
    return std::nullopt;
}

bool EspdiDevice::open(const DeviceConfig& cfg) {
    // Take the process-wide setup turn (bounded) before anything else, so a
    // concurrent open of another camera in the same process is serialized and a
    // stuck one cannot block this open forever. Outer to lifecycle_mtx.
    std::unique_lock<std::timed_mutex> setup_lk(
        device_setup_lock(), std::chrono::milliseconds(kSetupTurnTimeoutMs));
    if (!setup_lk.owns_lock()) {
        RCLCPP_ERROR(logger(),
                     "open(): timed out after %d ms waiting for the device-setup "
                     "turn; another camera may be stuck mid-open",
                     kSetupTurnTimeoutMs);
        return false;
    }
    // Hold lifecycle_mtx for the duration of the open sequence so a
    // concurrent stop() / close() / standby() cannot run
    // partway through APC_Init -> APC_OpenDevice2 -> ZD-table load.
    std::lock_guard<std::mutex> lifecycle_lk(impl_->lifecycle_mtx);
    impl_->cfg = cfg;

    SignalHandlerGuard signal_guard;
    int ret = APC_Init(&impl_->handle, /*bIsLogEnabled=*/false);
    if (ret != APC_OK || impl_->handle == nullptr) {
        RCLCPP_ERROR(logger(), "APC_Init failed: %s", espdi_strerror(ret).c_str());
        // APC_Init writes the context before it can fail, so a failed init
        // still owns one.
        if (impl_->handle != nullptr) {
            APC_Release(&impl_->handle);
            impl_->handle = nullptr;
        }
        return false;
    }

    const int dev_count = APC_GetDeviceNumber(impl_->handle);
    if (dev_count <= 0) {
        RCLCPP_ERROR(logger(),
                     "APC_GetDeviceNumber returned %d ; no cameras detected", dev_count);
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
        return false;
    }
    RCLCPP_INFO(logger(), "APC found %d device(s)", dev_count);

    // Device selection precedence: dev_serial_number, then usb_port (the USB
    // topology path from /sys/class/video4linux), then a PID match against the
    // launch `model`, then index 0 only when expected_pid is 0. The chosen
    // index's PID is re-validated against `model` below; a mismatch fails the
    // open.
    int chosen_index = -1;
    struct DevEnum {
        std::string serial_number;
        std::string dev_node; // /dev/videoN as reported by the SDK
        std::string usb_port; // sysfs-resolved, e.g. "2-3:1.0"
        unsigned short pid = 0;
    };
    auto enumerate = [&](int i) {
        DEVSELINFO dev_sel{i};
        DEVINFORMATION info{};
        unsigned char sn_buf[256] = {0};
        int sn_len = 0;
        DevEnum dev;
        if (APC_GetSerialNumber(impl_->handle, &dev_sel, sn_buf, sizeof(sn_buf), &sn_len) == APC_OK
            && sn_len > 0) {
            dev.serial_number = decode_serial(sn_buf, sn_len, sizeof(sn_buf));
        }
        // APC_GetDeviceInfo can fail transiently right after enumeration; a
        // zeroed descriptor empties usb_port and breaks a usb_port pin match.
        // Retry before giving up.
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (APC_GetDeviceInfo(impl_->handle, &dev_sel, &info) == APC_OK) {
                if (info.strDevName) dev.dev_node.assign(info.strDevName);
                dev.pid = info.wPID;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        dev.usb_port = resolve_usb_port(dev.dev_node);
        return dev;
    };

    const unsigned short want_pid = cfg.expected_pid;

    auto log_devices = [&]() {
        for (int i = 0; i < dev_count; ++i) {
            const auto dev = enumerate(i);
            RCLCPP_ERROR(logger(),
                         "  [%d] PID=0x%04x sn='%s' v4l2='%s' usb_port='%s'",
                         i, dev.pid, dev.serial_number.c_str(), dev.dev_node.c_str(), dev.usb_port.c_str());
        }
    };

    if (!cfg.serial_number.empty() || !cfg.usb_port.empty()) {
        for (int i = 0; i < dev_count; ++i) {
            const auto dev = enumerate(i);
            // Both hints must hold: an unset hint always passes, a set hint
            // must match. When both are pinned, a matching serial on the
            // wrong port is rejected (not silently accepted).
            const bool sn_ok  = cfg.serial_number.empty() ||
                                dev.serial_number.find(cfg.serial_number) != std::string::npos;
            const bool bus_ok = cfg.usb_port.empty() ||
                                (!dev.usb_port.empty() &&
                                 dev.usb_port.find(cfg.usb_port) != std::string::npos);
            if (sn_ok && bus_ok) {
                chosen_index = i;
                RCLCPP_INFO(logger(),
                            "Matched device at index %d (sn='%s', v4l2='%s', usb_port='%s')",
                            i, dev.serial_number.c_str(), dev.dev_node.c_str(), dev.usb_port.c_str());
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
            impl_->handle = nullptr;
            return false;
        }
    } else if (want_pid != 0) {
        // No SN / usb_port hint — select the first device whose USB PID matches
        // the requested model. This prevents two launches with different models
        // from binding to the same physical device when both are connected.
        int pid_match_count = 0;
        for (int i = 0; i < dev_count; ++i) {
            const auto dev = enumerate(i);
            if (dev.pid == want_pid) {
                ++pid_match_count;
                if (chosen_index < 0) {
                    chosen_index = i;
                    RCLCPP_INFO(logger(),
                                "Matched device at index %d (PID=0x%04x sn='%s' v4l2='%s' usb_port='%s') "
                                "via model '%s' PID lookup",
                                i, dev.pid, dev.serial_number.c_str(), dev.dev_node.c_str(), dev.usb_port.c_str(),
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
            impl_->handle = nullptr;
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
        const auto dev = enumerate(impl_->sel.index);
        impl_->serial_number = dev.serial_number;
        impl_->usb_port = dev.usb_port;
    }

    // Reject a mode whose USB port type the negotiated link cannot carry
    // (e.g. a USB3-only mode on a USB2 link) up front, before configuring
    // streams — otherwise the device opens but delivers no frames.
    if (cfg.mode_usb == 2 || cfg.mode_usb == 3) {
        USB_PORT_TYPE port = USB_PORT_TYPE_UNKNOW;
        if (APC_GetDevicePortType(impl_->handle, &impl_->sel, &port) == APC_OK &&
            (port == USB_PORT_TYPE_2_0 || port == USB_PORT_TYPE_3_0)) {
            const int link = (port == USB_PORT_TYPE_3_0) ? 3 : 2;
            if (link != cfg.mode_usb) {
                RCLCPP_ERROR(logger(),
                             "mode_id needs a USB%d link but the camera negotiated USB%d. "
                             "Pick a USB%d mode_id, or move the camera to a USB%d port.",
                             cfg.mode_usb, link, link, cfg.mode_usb);
                APC_Release(&impl_->handle);
                impl_->handle = nullptr;
                return false;
            }
            RCLCPP_INFO(logger(), "Negotiated USB link: USB%d", link);
        }
    }

    if ((ret = APC_GetDeviceInfo(impl_->handle, &impl_->sel, &impl_->dev_info)) != APC_OK) {
        RCLCPP_ERROR(logger(), "APC_GetDeviceInfo failed: %s", espdi_strerror(ret).c_str());
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
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
        impl_->handle = nullptr;
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
                        "APC_GetFwVersion %s (len=%d)", espdi_strerror(fw_rc).c_str(), fw_len);
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
    bool apply_disparity_shift = cfg.spatial_filter_enabled && depth_present;
    if (cfg.spatial_filter_enabled && !depth_present) {
        RCLCPP_WARN(logger(),
                    "spatial_filter requested but the active mode has no "
                    "depth stream ; filter disabled, depth_data_type left unchanged.");
    }
    // The +2 shift reaches the 11-bit counterpart only for the types that
    // have one. video_modes_dir is public, so the type is not assumed valid.
    static const std::set<int> kShiftableDepthTypes = {2, 7, 18, 34, 50};
    if (apply_disparity_shift &&
        kShiftableDepthTypes.count(cfg.depth_data_type) == 0) {
        RCLCPP_ERROR(logger(),
                     "depth_data_type %d has no 11-bit counterpart at %d; "
                     "spatial filter disabled",
                     cfg.depth_data_type, cfg.depth_data_type + 2);
        apply_disparity_shift = false;
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
                     "APC_SetDepthDataType(%d) failed: %s", effective_depth_dt,
                     espdi_strerror(ret).c_str());
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
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
                    "APC_SetInterleaveMode(%d): %s", cfg.interleave,
                    espdi_strerror(ret).c_str());
    }

    // Applied before APC_OpenDevice2 so the first frame already carries the
    // configured illumination. ir_value >= 0 is explicit (0 = off); -1 resolves
    // to the catalogue default when the mode has depth or the module is
    // monochrome (a mono sensor needs IR to light the scene even for colour),
    // and to off for a colour-only mode on a colour sensor. IR-MAX and the
    // mode-mask keep their firmware boot values.
    {
        const bool needs_ir = (cfg.depth_width > 0 && cfg.depth_height > 0)
                              || cfg.mono;
        impl_->ir_default_level = needs_ir ? cfg.ir_default : 0;
        const bool explicit_ir = cfg.ir_value >= 0;
        const int level = explicit_ir
            ? cfg.ir_value
            : impl_->ir_default_level;
        const int rc_cur = APC_SetCurrentIRValue(
            impl_->handle, &impl_->sel,
            static_cast<unsigned short>(level));
        RCLCPP_INFO(logger(),
                    "IR pre-open: SetCurrentIRValue(%d) %s (%s)",
                    level, espdi_strerror(rc_cur).c_str(),
                    explicit_ir ? "explicit" : "default");
    }

    // Read calibration and the ZD table BEFORE opening the streams: against an
    // already-open handle the read can hang indefinitely if the camera is
    // unplugged mid-open, against an un-opened handle it fails cleanly.
    //
    // Both come from zd_index, the G1 factory bank, and stay correct after a
    // self-calibration commit: GetRectifyMatLogData reads the Calibration Log
    // (flash file 240), not the Rectify Table (file 40) selfk writes, and the
    // factory ships G1 == G2. The G2/G1 bank offset (FW reg 0xF6) is only valid
    // after APC_OpenDevice2 and is read there.

    // Rectification log -> both lens calibration slots so the left and right
    // camera_info topics can publish independently; it carries both intrinsics
    // regardless of the active video mode. Retry on APC_READFLASHFAIL (-6),
    // which can occur transiently right after a fast reopen of the same device.
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
                        "APC_GetRectifyMatLogData(index=%d) %s (flash read), "
                        "retry %d/%d after %d ms",
                        cfg.zd_index, espdi_strerror(rc).c_str(), attempt + 1,
                        kMaxRectifyAttempts, kRectifyBackoffMs);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kRectifyBackoffMs));
        }
        if (rc != APC_OK) {
            // Drop the previous open's values; leaving them would publish the
            // old calibration as if it were this mode's.
            impl_->cached_rect_valid = false;
            impl_->calib = {};
            RCLCPP_WARN(logger(),
                        "APC_GetRectifyMatLogData(index=%d) %s ; camera_info will be empty",
                        cfg.zd_index, espdi_strerror(rc).c_str());
        } else {
            impl_->cached_rect_valid = true;
            const auto& rect = impl_->cached_rect;
            auto& calib = impl_->calib;
            calib.in_height  = rect.InImgHeight;
            calib.out_height = rect.OutImgHeight;
            for (int i = 0; i < 9; ++i)  calib.left.K[i]  = rect.CamMat1[i];
            for (int i = 0; i < 8; ++i)  calib.left.D[i]  = rect.CamDist1[i];
            for (int i = 0; i < 9; ++i)  calib.left.R[i]  = rect.LRotaMat[i];
            for (int i = 0; i < 12; ++i) calib.left.P[i]  = rect.NewCamMat1[i];
            for (int i = 0; i < 9; ++i)  calib.right.K[i] = rect.CamMat2[i];
            for (int i = 0; i < 8; ++i)  calib.right.D[i] = rect.CamDist2[i];
            for (int i = 0; i < 9; ++i)  calib.right.R[i] = rect.RRotaMat[i];
            for (int i = 0; i < 12; ++i) calib.right.P[i] = rect.NewCamMat2[i];
            calib.baseline_mm = std::abs(static_cast<double>(rect.TranMat[0]));
            calib.valid = true;
            RCLCPP_INFO(logger(),
                        "Rectify loaded (index=%d): L fx=%.2f cx=%.2f / R fx=%.2f cx=%.2f / baseline=%.2f mm",
                        cfg.zd_index,
                        calib.left.K[0], calib.left.K[2],
                        calib.right.K[0], calib.right.K[2], calib.baseline_mm);
        }
    }
    // ZD (disparity->mm) table — needed only when the spatial filter runs.
    // A load failure with the filter requested is a hard error. The device is
    // not open yet, so release the handle without APC_CloseDevice.
    if (apply_disparity_shift &&
        !load_zd_table(impl_->handle, &impl_->sel, cfg.zd_index, impl_->zd_table)) {
        RCLCPP_ERROR(logger(),
                     "Spatial filter requested but ZD table load failed "
                     "(index=%d). Refusing to open.", cfg.zd_index);
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
        return false;
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
        /*bIsOutputRGB24=*/false,
        /*phWndNotice=*/nullptr,
        &negotiated_fps,
        IMAGE_SN_SYNC);
    if (ret != APC_OK) {
        RCLCPP_ERROR(logger(),
                     "APC_OpenDevice2 failed: %s ; requested fps=%d, got %d",
                     espdi_strerror(ret).c_str(), cfg.framerate, negotiated_fps);
        APC_Release(&impl_->handle);
        impl_->handle = nullptr;
        return false;
    }
    impl_->actual_fps.store(negotiated_fps, std::memory_order_relaxed);
    RCLCPP_INFO(logger(),
                "Device opened: color %dx%d %s, depth %dx%d type=%d, fps=%d, interleave=%d",
                cfg.color_width, cfg.color_height,
                cfg.color_format == 0 ? "YUYV" : "MJPEG",
                cfg.depth_width, cfg.depth_height, cfg.depth_data_type,
                negotiated_fps, cfg.interleave ? 1 : 0);

    // Active calibration bank from FW register 0xF6: 5 = the module has a G2
    // user bank (self-calibration may commit there); 0 = G1 factory only.
    // Read here, not before APC_OpenDevice2 — the SDK's open is what sets 0xF6
    // to 5 on a G2 unit (its power-on default is 0). Used only for the
    // self-calibration commit target and the "has G2" support check.
    {
        unsigned short bank = 0;
        const int rc = APC_GetFWRegister(impl_->handle, &impl_->sel, 0xF6, &bank,
                                         FG_Address_1Byte | FG_Value_1Byte);
        impl_->calib_bank_offset = (rc == APC_OK && bank == 5) ? 5 : 0;
        RCLCPP_INFO(logger(), "Calibration banks: %s (0xF6=%u)",
                    impl_->calib_bank_offset == 5 ? "G1 factory + G2 user"
                                                  : "G1 factory only",
                    bank);
    }

    // Read the FW-reported IR ceiling (reg 0xE2) so set_ir_value can
    // clamp user-supplied values, and log the current level for diagnostics.
    {
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

    // Colored point cloud requires a configured color stream; D-only
    // modes use the XYZ-only path.
    impl_->colored_pointcloud = cfg.colored_pointcloud && cfg.color_width > 0 && cfg.color_height > 0;

    // Spatial filter params: launch-time only. The depth stream is in 11-bit
    // disparity mode here (apply_disparity_shift forced the dtype +2 above).
    // Skipped on color-only modes.
    impl_->spatial_filter_enabled = false;
    if (apply_disparity_shift) {
        const double alpha = std::clamp(cfg.spatial_filter_alpha, 0.0, 1.0);
        impl_->spatial_params.alpha_q8      = static_cast<int>(std::lround(alpha * 256.0));
        // Clamp keeps the Q4 shift inside uint16.
        impl_->spatial_params.delta_q4      = std::clamp(cfg.spatial_filter_delta, 1, 4095) << 4;
        impl_->spatial_params.magnitude     = std::clamp(cfg.spatial_filter_magnitude, 1, 5);
        impl_->spatial_params.holes_fill = std::max(0, cfg.spatial_filter_holes_fill);
        // The ZD table it needs was loaded from flash before the streams
        // opened (a load failure there already refused the open).
        impl_->spatial_filter_enabled = true;
    }

    // Runs in whichever domain has a uint16 raster: between the IIR and
    // the ZD lookup (Q4 disparity) when spatial_filter is on, on the FW
    // Z14 mm raster when it is off. delta is stored raw and converted at
    // the call site -- `<< 4` for Q4, used as mm for Z.
    {
        const double alpha = std::clamp(cfg.temporal_filter_alpha, 0.0, 1.0);
        TemporalFilterParams tp;
        tp.alpha_q8    = static_cast<int>(std::lround(alpha * 256.0));
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
        const int mode = std::clamp(cfg.hole_filling, 0, 3);
        impl_->hole_fill_mode = static_cast<HoleFillMode>(mode);
        const char* name = (mode == 1) ? "fill_from_left"
                         : (mode == 2) ? "farthest_from_around"
                                       : "nearest_from_around";
        RCLCPP_INFO(logger(), "Hole filling enabled: mode=%d (%s)", mode, name);
    }

    // PointCloud working range. Launch parameters depth_near_mm /
    // depth_far_mm > 0 override; -1 = use the catalogue default.
    {
        const float resolved_near = (cfg.depth_near_mm > 0)
            ? static_cast<float>(cfg.depth_near_mm)
            : static_cast<float>(cfg.default_near_mm);
        const float resolved_far  = (cfg.depth_far_mm > 0)
            ? static_cast<float>(cfg.depth_far_mm)
            : static_cast<float>(cfg.default_far_mm);

        impl_->depth_near_mm = resolved_near;
        impl_->depth_far_mm  = resolved_far;

        RCLCPP_INFO(logger(),
                    "Depth range: near=%.0f mm (%s)  far=%.0f mm (%s)  (default=[%d, %d] mm)",
                    resolved_near,
                    (cfg.depth_near_mm > 0) ? "explicit" : "default",
                    resolved_far,
                    (cfg.depth_far_mm > 0) ? "explicit" : "default",
                    cfg.default_near_mm, cfg.default_far_mm);
    }

    {
        std::lock_guard<std::mutex> lk(impl_->latest.mtx);
        impl_->latest.depth.reset();
        impl_->latest.depth_pending = false;
    }

    // Bind a self-calibration context to the open handle; dormant until
    // start_selfcal(). Creation does not touch the stream.
    if (cfg.selfcal_enable) {
        impl_->selfcal_ = std::make_unique<SelfCalManager>(
            impl_->handle, &impl_->sel, &impl_->dev_info,
            impl_->serial_number, cfg.selfcal_config_dir, logger());
        // Let the A/B re-check toggle cy on the live stream through the locked
        // register accessors.
        impl_->selfcal_->set_cy_accessors(
            [this](unsigned short& lo, unsigned short& hi) {
                return get_cy_regs(lo, hi);
            },
            [this](unsigned short lo, unsigned short hi) {
                return set_cy_regs(lo, hi);
            });
        if (impl_->selfcal_->available()) {
            RCLCPP_INFO(logger(),
                        "Self-calibration bound (profiles dir: '%s')",
                        cfg.selfcal_config_dir.c_str());
        }
    }

    impl_->opened = true;
    return true;
}

void EspdiDevice::close() {
    if (!impl_) return;
    // stop() (idempotent) before releasing the handle, so a close() while
    // streaming cannot free it under the fetch threads. It takes lifecycle_mtx
    // itself, so it must run before the lock below.
    stop();
    // Tear down the session before the handle is released — ~SelfCalManager
    // calls EYS3D_SC_Stop/Destroy through it. stop() already joined depth_fetch.
    impl_->selfcal_.reset();
    if (impl_->cy_snapshot_valid) {
        // A session abandoned mid-flight (node shutdown / disconnect): roll cy
        // back before releasing the handle so a live device is not left on a
        // half-search value. On a disconnect the handle is already dead and the
        // write logs one warning; the reconnect reloads calibration from flash.
        restore_cy();
        impl_->cy_snapshot_valid = false;
    }
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
    impl_->last_color_frame.store(-1, std::memory_order_relaxed);
    impl_->last_depth_frame.store(-1, std::memory_order_relaxed);

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
        // Split-aware YUYV decode emits both half-width rgb8 buffers in one
        // pass. MJPEG cannot be split during decode -- libjpeg-turbo's MCU
        // blocks do not align with the mid-row boundary -- so those modes
        // decode wide and are sliced in camera_node.
        // %4, not %2: the half-width (cw/2) must itself be even so each eye's
        // split lands on a YUYV macropixel (2 px / 4 bytes) boundary.
        const bool split_yuyv = cfg.split_color && !wire_is_mjpeg &&
                                (cw % 4 == 0);
        const int  side_w   = split_yuyv ? cw / 2 : cw;
        const bool mono = cfg.mono;
        const size_t raw_bytes = color_raw_buffer_bytes(cw, ch);
        // rgb_bytes sizes fb.data:
        //   YUYV split   side_w = cw/2; both halves are filled in one pass.
        //   MJPEG split  side_w = cw; fb.data holds the wide raster and the
        //                left/right split happens in publish_split_color.
        //   non-split    side_w = cw, one buffer.
        const size_t rgb_bytes = color_rgb8_bytes(side_w, ch);

        // Reused across iterations — the SDK reads into this buffer, then
        // the loop decodes / converts into the per-frame `fb.data` (rgb8)
        // before moving fb into the callback. One allocation total instead
        // of one per frame.
        std::vector<uint8_t> raw(raw_bytes);

        std::vector<uint8_t> gray;
        if (mono) gray.resize(static_cast<size_t>(cw) * ch);

        // libjpeg-turbo decompressor for MJPEG modes. Symbols
        // (tjInitDecompress / tjDecompress2 / tjDestroy) are re-exported
        // from libeSPDI, so no external libjpeg link is required.
        tjhandle tj = wire_is_mjpeg ? tjInitDecompress() : nullptr;
        if (wire_is_mjpeg && !tj) {
            RCLCPP_ERROR(logger(),
                         "tjInitDecompress() failed ; MJPEG modes unusable");
        }

        while (impl_->running.load(std::memory_order_acquire)) {
          try {
            unsigned long image_bytes = 0;
            int frame_number = 0;
            int64_t tv_sec = 0, tv_usec = 0;
            const int rc = APC_GetColorImageWithTimestamp(
                handle, sel, raw.data(), &image_bytes, &frame_number,
                depth_dt, &tv_sec, &tv_usec);
            if (rc != APC_OK) {
                if (rc == APC_DEVICE_TIMEOUT) {
                    usleep(kTimeoutBackoffUs);
                } else {
                    usleep(kErrorBackoffUs);
                    // Throttle unexpected return codes so a misconfigured
                    // stream remains visible without flooding the log.
                    RCLCPP_WARN_THROTTLE(logger(),
                                         throttle_clock(), 5000,
                                         "APC_GetColorImageWithTimestamp %s", espdi_strerror(rc).c_str());
                }
                continue;
            }
            // Interleave SN parity: in interleave mode both streams deliver
            // every frame and the consumer selects by frame number parity
            // (color = even, depth = odd).
            if (interleave && (frame_number % 2) != 0) {
                continue;
            }
            // Detect frames lost in transit. After parity filtering,
            // accepted SNs should advance by `step` per frame; any larger
            // forward gap means the USB / SDK layer dropped one or more
            // frames before they reached the fetch loop.
            const int step = interleave ? 2 : 1;
            const int prev = impl_->last_color_frame.exchange(
                frame_number, std::memory_order_relaxed);
            if (prev >= 0) {
                const int delta = frame_number - prev;
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

            // The YUYV kernels read cw*ch*2 bytes from raw; a truncated wire
            // frame would run them off the buffer. MJPEG carries its own
            // length (tjDecompress2 is bounded by image_bytes), so guard only
            // the raw path.
            if (!wire_is_mjpeg &&
                image_bytes < static_cast<unsigned long>(cw) * ch * 2) {
                RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 5000,
                                     "short color frame (%lu B < %d expected); dropping",
                                     image_bytes, cw * ch * 2);
                continue;
            }

            FrameBuffer fb;
            fb.data.resize(rgb_bytes);
            if (split_yuyv) fb.data_right.resize(rgb_bytes);
            fb.frame_number = frame_number;
            fb.hw_timestamp_us =
                static_cast<uint64_t>(tv_sec) * 1000000ULL + static_cast<uint64_t>(tv_usec);
            fb.width  = side_w;
            fb.height = ch;

            const auto t_decode_begin = std::chrono::steady_clock::now();
            if (mono) {
                // MJPEG split modes stay wide here — camera_node slices,
                // matching the color MJPEG path below.
                if (wire_is_mjpeg) {
                    if (!tj) continue;
                    const int drc = tjDecompress2(
                        tj, raw.data(), image_bytes,
                        gray.data(),
                        cw, /*pitch=*/cw, ch,
                        TJPF_GRAY, /*flags=*/0);
                    if (drc != 0) {
                        RCLCPP_WARN_THROTTLE(logger(),
                                             throttle_clock(), 5000,
                                             "tjDecompress2 failed: %s",
                                             tjGetErrorStr2(tj));
                        continue;
                    }
                } else {
                    simd::yuyv_extract_y(raw.data(), gray.data(), cw, ch);
                }
                if (split_yuyv) {
                    simd::gray_to_rgb8_split(gray.data(),
                                             fb.data.data(),
                                             fb.data_right.data(),
                                             side_w, ch);
                } else {
                    simd::gray_to_rgb8(gray.data(), fb.data.data(), cw, ch);
                }
            } else if (wire_is_mjpeg) {
                // MJPEG → rgb8 inline (libjpeg-turbo SIMD; ~5-10 ms / 1.2 MP).
                if (!tj) continue;
                const int drc = tjDecompress2(
                    tj, raw.data(), image_bytes,
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
                // Wide YUYV -> two half-width rgb8 buffers in one pass.
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
        const size_t buf_bytes = depth_buffer_bytes(cfg.depth_width, cfg.depth_height);
        // Flag a depth stream that never yields a frame, as distinct from a
        // transient timeout. The threshold clears the several-second
        // firmware cold start; the warning fires once until a frame arrives.
        constexpr auto kDepthSilenceWarn = std::chrono::seconds(10);
        std::chrono::steady_clock::time_point depth_empty_since{};
        bool depth_silence_warned = false;
        while (impl_->running.load(std::memory_order_acquire)) {
          try {
            FrameBuffer fb;
            fb.data.resize(buf_bytes);
            unsigned long image_bytes = 0;
            int frame_number = 0;
            int64_t tv_sec = 0, tv_usec = 0;
            const int rc = APC_GetDepthImageWithTimestamp(
                handle, sel, fb.data.data(), &image_bytes, &frame_number,
                depth_dt, &tv_sec, &tv_usec);
            if (rc != APC_OK) {
                if (rc == APC_DEVICE_TIMEOUT) {
                    usleep(kTimeoutBackoffUs);
                } else {
                    usleep(kErrorBackoffUs);
                    RCLCPP_WARN_THROTTLE(logger(),
                                         throttle_clock(), 5000,
                                         "APC_GetDepthImageWithTimestamp %s", espdi_strerror(rc).c_str());
                }
                const auto now = std::chrono::steady_clock::now();
                if (depth_empty_since.time_since_epoch().count() == 0) {
                    depth_empty_since = now;
                } else if (!depth_silence_warned
                           && now - depth_empty_since > kDepthSilenceWarn) {
                    depth_silence_warned = true;
                    RCLCPP_WARN(logger(),
                                "no depth frame for %llds: "
                                "APC_GetDepthImageWithTimestamp %s (dtype=%d, %dx%d) ; "
                                "color may be streaming while depth is not",
                                static_cast<long long>(kDepthSilenceWarn.count()),
                                espdi_strerror(rc).c_str(), depth_dt,
                                cfg.depth_width, cfg.depth_height);
                }
                continue;
            }
            depth_empty_since = std::chrono::steady_clock::time_point{};
            depth_silence_warned = false;
            if (interleave && (frame_number % 2) != 1) {
                continue;
            }
            const int step = interleave ? 2 : 1;
            const int prev = impl_->last_depth_frame.exchange(
                frame_number, std::memory_order_relaxed);
            if (prev >= 0) {
                const int delta = frame_number - prev;
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
            if (image_bytes != buf_bytes) {
                RCLCPP_WARN_THROTTLE(logger(),
                                     throttle_clock(), 5000,
                                     "depth frame size mismatch: got=%lu expected=%zu (dropped)",
                                     image_bytes, buf_bytes);
                continue;
            }
            fb.frame_number = frame_number;
            fb.hw_timestamp_us =
                static_cast<uint64_t>(tv_sec) * 1000000ULL + static_cast<uint64_t>(tv_usec);
            fb.width  = cfg.depth_width;
            fb.height = cfg.depth_height;

            // Feed the raw, unclipped FW depth raster to an in-progress session
            // (no-op when none runs), before the range clip and filters below.
            // Either depth domain works (Z14 mm or D11 disparity): selfk scores
            // only the non-zero valid-pixel ratio. selfcal_ is stable here —
            // created in open() before the threads, reset in close() after join.
            if (impl_->selfcal_) {
                impl_->selfcal_->submit_latest(
                    reinterpret_cast<const uint16_t*>(fb.data.data()),
                    static_cast<uint32_t>(cfg.depth_width),
                    static_cast<uint32_t>(cfg.depth_height),
                    /*temperature_c=*/0.0f, fb.hw_timestamp_us);
            }

            // With any filter active (spatial / temporal / hole_filling) pc_thread
            // is the depth publisher and depth_fetch only stages the raw FW depth.
            // With all filters off, depth_fetch publishes directly.
            const bool depth_gate_open = impl_->gate_pass(impl_->depth_gate);
            const bool pc_gate_open    = impl_->gate_pass(impl_->pc_gate);
            const bool any_filter = impl_->spatial_filter_enabled
                || impl_->temporal_enabled.load(std::memory_order_acquire)
                || impl_->hole_fill_mode != HoleFillMode::kOff;
            const bool need_snapshot = any_filter
                ? (depth_gate_open || pc_gate_open)
                : pc_gate_open;

            // No-filter path: clip in place before either consumer reads the
            // buffer, so the depth image and the point-cloud snapshot see the same
            // raster. The Z14 high 2 bits are status flags, stripped before the
            // range test. The row-0 watermark is cleared after the clip -- its byte
            // patterns can fall inside the range and would survive as a plausible
            // short reading.
            if (!any_filter && (depth_gate_open || pc_gate_open)) {
                constexpr uint16_t kDepthMask = 0x3FFF;
                const uint16_t z_near = static_cast<uint16_t>(
                    std::clamp(impl_->depth_near_mm, 0.0f, 65535.0f));
                const uint16_t z_far = static_cast<uint16_t>(
                    std::clamp(impl_->depth_far_mm,  0.0f, 65535.0f));
                uint16_t* mm = reinterpret_cast<uint16_t*>(fb.data.data());
                const size_t n = static_cast<size_t>(fb.width) * fb.height;
                for (size_t i = 0; i < n; ++i) {
                    const uint16_t z = mm[i] & kDepthMask;
                    mm[i] = (z < z_near || z > z_far) ? 0 : z;
                }
                std::fill_n(mm, kSerialSkipPixels, uint16_t{0});
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
                        "Rectify log unavailable: /depth/points disabled; "
                        "filtered /depth/image_raw still published");
        }
        const size_t pc_points   = static_cast<size_t>(cfg.depth_width) * cfg.depth_height;
        const int    W = cfg.depth_width;
        const int    H = cfg.depth_height;
        // Workspace stride: 12 bytes for XYZ float32, 16 for XYZRGB
        // (XYZ followed by a packed 0x00RRGGBB uint32). Sized once at
        // worst case; each publish copies only the populated prefix.
        const uint32_t pc_point_step = impl_->colored_pointcloud ? 16u : 12u;
        std::vector<uint8_t> workspace(pc_points * pc_point_step);

        // Filter workspace, sized once at thread start; unused buffers stay
        // empty. pc_q4_buf exists only in the disparity pipeline. pc_mm_buf
        // serves every filter, and the Z-domain branch reuses it without
        // reallocating because temporal's enable can flip at runtime.
        if (impl_->spatial_filter_enabled) {
            impl_->pc_q4_buf.assign(pc_points, 0);
        }
        impl_->pc_mm_buf.assign(pc_points, 0);

        // Color-pixel byte-offset LUTs: the projection inner loop resolves a
        // per-pixel colour address with two LUT reads and an add, no multiply
        // or divide. The LUT must match the snapshot's actual width -- in wide
        // L|R split modes that is the left half, so the cloud samples the left
        // lens.
        if (impl_->colored_pointcloud) {
            // Two widths, not always equal:
            //   sample_w  pixel range the LUT addresses; split modes restrict it
            //             to the left half whatever the wire format.
            //   buf_w     row stride of the snapshot buffer in latest_color. The
            //             YUYV split path produces a half-width buffer inline;
            //             the MJPEG path decodes wide, at full width.
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
        const float ratio_mat = can_reproject
            ? static_cast<float>(raster_scale(impl_->calib.out_height, H))
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
        // Upper bound on the stereo left-edge dead-zone, evaluated at the
        // configured Z minimum (the real width goes as 1 / Z). Used only as
        // fill_from_left's left_skip, which passes those columns through
        // unchanged instead of seeding the per-row last_valid state;
        // left_skip never overwrites valid data.
        int dead_zone_left_px = 0;
        if (can_reproject) {
            const float fx = rl.NewCamMat1[0] * ratio_mat;
            const float fy = rl.NewCamMat1[5] * ratio_mat;
            const float cx = rl.NewCamMat1[2] * ratio_mat;
            const float cy = rl.NewCamMat1[6] * ratio_mat;
            if (fx == 0.0f || fy == 0.0f) {
                RCLCPP_ERROR(logger(),
                             "Reprojection LUT init: fx or fy is zero "
                             "(fx=%.2f fy=%.2f); /depth/points disabled, "
                             "filtered /depth/image_raw still published",
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
                const double z_min_mm    = static_cast<double>(impl_->depth_near_mm);
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
        // OMP_WAIT_POLICY and GOMP_SPINCOUNT come from the launch environment;
        // libgomp reads them at the first parallel region. Thread count is set
        // here because it overrides the environment. Capped at 4 against
        // over-decomposition on cache-constrained hosts, floored at 1 for a
        // hardware_concurrency() of 0.
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
            // All three filter flags are sampled from one moment: a runtime toggle
            // of temporal_filter mid-iteration would otherwise leave
            // depth_publish_buf allocated but unwritten, publishing a zero frame.
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

            // Filter sink and projection source, moved into depth_fb.data at
            // the end of the loop. Invariant: non-empty here means a filter
            // branch below fully overwrites it before the publish.
            std::vector<uint8_t> depth_publish_buf;
            uint16_t* filter_mm_sink = impl_->pc_mm_buf.data();
            if (need_depth && impl_->on_depth) {
                depth_publish_buf.resize(static_cast<size_t>(W) * H * 2);
                filter_mm_sink = reinterpret_cast<uint16_t*>(depth_publish_buf.data());
            }

            const auto t_compute_begin = std::chrono::steady_clock::now();

            constexpr float    kMmToM    = 1.0f / 1000.0f;
            constexpr uint16_t kDepthMask = 0x3FFF;     // Z14 high 2 bits are flags
            size_t valid_points = 0;

            // Three-step reprojection over the depth raster:
            //   1. count valid pixels per row (parallel, NEON on aarch64)
            //   2. prefix-sum to assign each row a contiguous output slot
            //   3. project + write compacted (parallel, scalar inner loop)
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
                // Mark the watermark columns as holes so the
                // neighborhood filters never read them as disparity.
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
                const uint16_t z_near = static_cast<uint16_t>(
                    std::clamp(impl_->depth_near_mm, 0.0f, 65535.0f));
                const uint16_t z_far = static_cast<uint16_t>(
                    std::clamp(impl_->depth_far_mm,  0.0f, 65535.0f));
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
                        mrow[u] = (z < z_near || z > z_far) ? 0 : z;
                    }
                }
                d = mm;

                // Z-domain hole filling. Runs after the ZD lookup so
                // both /depth/image_raw and the reprojected cloud see the
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
                // Same snapshot as the publish-buffer gate: a runtime toggle
                // between the two would leave the buffer allocated but
                // unwritten.
                const bool z_temporal = snap_temporal;
                const bool z_holes    = snap_holes;
                if (z_temporal || z_holes) {
                    uint16_t* mm = filter_mm_sink;
                    const size_t n = static_cast<size_t>(W) * H;
                    // Strip the Z14 status bits and clamp to [z_near, z_far] in one pass;
                    // pixels outside become 0 and downstream filters treat them as holes.
                    // The spatial path enforces the same range after its ZD lookup.
                    const uint16_t z_near = static_cast<uint16_t>(
                        std::clamp(impl_->depth_near_mm, 0.0f, 65535.0f));
                    const uint16_t z_far = static_cast<uint16_t>(
                        std::clamp(impl_->depth_far_mm,  0.0f, 65535.0f));
                    for (size_t i = 0; i < n; ++i) {
                        const uint16_t z = d[i] & kDepthMask;
                        mm[i] = (z < z_near || z > z_far) ? 0 : z;
                    }
                    // Watermark columns must not feed the kernels below.
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

            // Point-cloud reprojection runs only when a /depth/points
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

                uint32_t valid_prefix = 0;
                for (int v = 0; v < H; ++v) {
                    row_offsets[v] = valid_prefix;
                    valid_prefix += row_valid_counts[v];
                }
                valid_points = valid_prefix;

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
                if (valid_points == 0) {
                    RCLCPP_WARN_ONCE(logger(),
                                     "PointCloud compaction kept 0 of %zu points "
                                     "(depth range [%.0f..%.0f] mm applied upstream)",
                                     pc_points,
                                     impl_->depth_near_mm, impl_->depth_far_mm);
                } else if (impl_->on_pc) {
                    // Copy only the populated prefix of the workspace into a
                    // fresh publish buffer. std::vector's (InputIt, InputIt)
                    // constructor uses uninitialized_copy → memcpy for trivial
                    // types, with no intermediate value-init. Workspace itself
                    // is retained for the next iteration.
                    const size_t valid_bytes = static_cast<size_t>(valid_points)
                                             * this_point_step;
                    std::vector<uint8_t> msg_buf(workspace.begin(),
                                                 workspace.begin() + valid_bytes);
                    impl_->on_pc(std::move(msg_buf),
                                 static_cast<uint32_t>(valid_points),
                                 this_point_step,
                                 depth_ts);
                }
            }

            // After projection has finished reading the same buffer.
            // depth_publish_buf is non-empty only when the filter pipeline routed
            // its sink there, in which case one of the branches above has fully
            // overwritten it. The on_depth check mirrors the allocation
            // precondition and covers a late teardown.
            if (!depth_publish_buf.empty() && impl_->on_depth) {
                // Re-clear the watermark columns: the pre-filter zeroes them
                // (marking them as holes), after which a hole-filling kernel
                // may have refilled them from a neighbour.
                std::fill_n(reinterpret_cast<uint16_t*>(depth_publish_buf.data()),
                            kSerialSkipPixels, uint16_t{0});
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
            if (need_pc && valid_points > 0) {
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

bool EspdiDevice::reset_usb() {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;

    // eSP876 USB self-reset register sequence, {address, value}. The
    // leading writes reconfigure the link; the final write (0xF01E) drops
    // it, so the host re-enumerates the device as if it had been replugged.
    // Two-byte address, one-byte value, matching the DM_Quality register
    // writes in register_settings.cpp.
    struct ResetReg { unsigned short addr; unsigned char val; };
    static constexpr ResetReg kResetSeq[] = {
        {0xF069, 0xF3}, {0xF0B0, 0x00}, {0xF0D2, 0x80}, {0xF0D2, 0x00},
        {0xF11A, 0x44}, {0xF11A, 0x40}, {0xF0F0, 0x00}, {0xF0FC, 0x20},
        {0xF0FC, 0x00}, {0xF0E0, 0x00}, {0xF500, 0x40}, {0xF500, 0x00},
        {0xF01E, 0x45},
    };
    const int flags = FG_Address_2Byte | FG_Value_1Byte;
    // The detach-triggering tail writes are acknowledged unreliably, so
    // individual results are ignored — a missing ack on the write that
    // drops the link is expected, not a failure.
    for (const auto& r : kResetSeq)
        APC_SetHWRegister(impl_->handle, &impl_->sel, r.addr, r.val, flags);

    RCLCPP_INFO(logger(),
                "reset_usb: USB self-reset issued; the link will drop and the "
                "watchdog will reopen the device");
    return true;
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
    if (selfcal_reject("pause")) return false;
    // Flips stream_state; the fetch threads keep draining USB and skip
    // decode/filter/publish on the next iteration, so a resume is observed
    // one frame later. Shares lifecycle_mtx with standby() so the two
    // cannot race.
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
    if (selfcal_reject("standby")) return false;
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
        // Join the DM_Quality worker too — it writes to the SDK handle, so it
        // must not outlive APC_CloseDevice. Under dm_quality_mtx (uncontended —
        // the fetch threads are joined above).
        {
            std::lock_guard<std::mutex> dmq_lk(impl_->dm_quality_mtx);
            if (impl_->dm_quality_worker.joinable()) impl_->dm_quality_worker.join();
        }
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

    // Standby -> Active|Paused. Replays open()'s SDK init sequence, minus
    // APC_Init / APC_GetDeviceInfo / GetRectifyMatLogData, which survive
    // APC_CloseDevice. Order matters: SetupBlock and SetDepthDataType
    // before SetInterleaveMode, and v4l2_requestbuffers before
    // OpenDevice2 -- without it the V4L2 queue stays at its ~3-buffer
    // default and the depth pump stalls.
    const auto& cfg = impl_->cfg;
    APC_SetupBlock(impl_->handle, &impl_->sel, false);
    if (impl_->depth_stream_present) {
        const int dtype_rc = APC_SetDepthDataType(
            impl_->handle, &impl_->sel, cfg.depth_data_type);
        if (dtype_rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "standby(false): APC_SetDepthDataType(%d) %s",
                        cfg.depth_data_type, espdi_strerror(dtype_rc).c_str());
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
                        "standby(false): APC_Setup_v4l2_requestbuffers(32) %s",
                        espdi_strerror(rb_rc).c_str());
        }
    }
    if (cfg.interleave) {
        const int il_rc = APC_SetInterleaveMode(impl_->handle, &impl_->sel, true);
        if (il_rc != APC_OK) {
            RCLCPP_WARN(logger(),
                        "standby(false): APC_SetInterleaveMode(true) %s", espdi_strerror(il_rc).c_str());
        }
    }
    // Apply IR before APC_OpenDevice2 so the V4L2 capture buffer fills at
    // the configured illumination from the first frame. ir_default_level
    // carries open()'s mode-resolved default; a runtime ir_value
    // override is re-applied separately by the standby service handler
    // after this function returns.
    {
        const int level = (cfg.ir_value >= 0)
            ? cfg.ir_value
            : impl_->ir_default_level;
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
        /*bIsOutputRGB24=*/false, /*phWndNotice=*/nullptr,
        &actual_fps, IMAGE_SN_SYNC);
    if (rc != APC_OK) {
        RCLCPP_ERROR(logger(),
                     "standby(false): APC_OpenDevice2(c=%dx%d, d=%dx%d) failed: %s ; device closed",
                     cw, ch, dw, dh, espdi_strerror(rc).c_str());
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
    // Under dm_quality_mtx (uncontended here — the fetch threads are joined
    // above).
    {
        std::lock_guard<std::mutex> dmq_lk(impl_->dm_quality_mtx);
        join_named(impl_->dm_quality_worker, "dm_quality_worker");
    }
}

EspdiDevice::Calibration EspdiDevice::calibration() const {
    return impl_->calib;
}

// The gate std::functions are read lock-free on the per-frame hot path, so
// they may only be assigned while stopped. Every caller sets them before
// start(); the setters reject a call made while streaming.
bool EspdiDevice::gate_setter_allowed(const char* which) {
    if (impl_->running.load(std::memory_order_acquire)) {
        RCLCPP_ERROR(logger(),
                     "%s called while streaming; ignoring to avoid a torn "
                     "std::function read on the fetch threads", which);
        return false;
    }
    return true;
}

bool EspdiDevice::selfcal_active() const {
    return impl_->selfcal_ && impl_->selfcal_->active();
}

bool EspdiDevice::selfcal_reject(const char* which) {
    if (selfcal_active()) {
        RCLCPP_WARN(logger(),
                    "%s refused: a self-calibration session is in progress",
                    which);
        return true;
    }
    return false;
}

bool EspdiDevice::selfcal_available() const {
    return impl_->selfcal_ && impl_->selfcal_->available();
}

// eSP876 cy_R (right-imager vertical) live register: 0xF56A low byte, 0xF56B high
// byte, 1-byte values. This is the only register a cy session dithers.
namespace {
constexpr unsigned short kRegCyLow  = 0xF56A;
constexpr unsigned short kRegCyHigh = 0xF56B;
}  // namespace

bool EspdiDevice::get_cy_regs(unsigned short& lo, unsigned short& hi) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    unsigned short l = 0, h = 0;
    const int r1 = APC_GetHWRegister(impl_->handle, &impl_->sel, kRegCyLow, &l,
                                     FG_Address_2Byte | FG_Value_1Byte);
    const int r2 = APC_GetHWRegister(impl_->handle, &impl_->sel, kRegCyHigh, &h,
                                     FG_Address_2Byte | FG_Value_1Byte);
    if (r1 != APC_OK || r2 != APC_OK) {
        RCLCPP_WARN(logger(), "cy read failed: %s / %s",
                    espdi_strerror(r1).c_str(), espdi_strerror(r2).c_str());
        return false;
    }
    lo = l;
    hi = h;
    return true;
}

bool EspdiDevice::set_cy_regs(unsigned short lo, unsigned short hi) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    const int r1 = APC_SetHWRegister(impl_->handle, &impl_->sel, kRegCyLow, lo,
                                     FG_Address_2Byte | FG_Value_1Byte);
    const int r2 = APC_SetHWRegister(impl_->handle, &impl_->sel, kRegCyHigh, hi,
                                     FG_Address_2Byte | FG_Value_1Byte);
    if (r1 != APC_OK || r2 != APC_OK) {
        RCLCPP_WARN(logger(), "cy write failed: %s / %s",
                    espdi_strerror(r1).c_str(), espdi_strerror(r2).c_str());
        return false;
    }
    return true;
}

bool EspdiDevice::snapshot_cy() {
    unsigned short lo = 0, hi = 0;
    if (!get_cy_regs(lo, hi)) {
        impl_->cy_snapshot_valid = false;
        return false;
    }
    impl_->cy_snapshot_lo = lo;
    impl_->cy_snapshot_hi = hi;
    impl_->cy_snapshot_valid = true;
    return true;
}

bool EspdiDevice::restore_cy() {
    if (!impl_->cy_snapshot_valid) return false;
    if (!set_cy_regs(impl_->cy_snapshot_lo, impl_->cy_snapshot_hi)) return false;
    RCLCPP_INFO(logger(), "cy register restored to the pre-session value");
    return true;
}

bool EspdiDevice::start_selfcal(const std::string& profile) {
    if (!impl_->selfcal_ || !impl_->selfcal_->available()) {
        RCLCPP_WARN(logger(), "start_selfcal: self-calibration is not available");
        return false;
    }
    // No G2 user bank (0xF6=0) -> nowhere to commit; self-cal unsupported.
    if (impl_->calib_bank_offset != 5) {
        RCLCPP_WARN(logger(),
                    "start_selfcal: this module has no G2 user bank (0xF6=0); "
                    "self-calibration is not supported");
        return false;
    }
    if (!impl_->running.load(std::memory_order_acquire)) {
        RCLCPP_WARN(logger(),
                    "start_selfcal: the device must be streaming a depth mode");
        return false;
    }
    // The pause gate in depth_fetch returns before the selfcal feed, so a
    // session started here would only ever time out.
    if (impl_->stream_state.load(std::memory_order_relaxed) != StreamState::Active) {
        RCLCPP_WARN(logger(),
                    "start_selfcal: the stream is paused; resume before calibrating");
        return false;
    }
    if (!impl_->depth_stream_present) {
        RCLCPP_WARN(logger(),
                    "start_selfcal: the active mode delivers no depth stream");
        return false;
    }
    // Snapshot cy before the search starts so a worse or abandoned run can be
    // rolled back. Refuse to start without it, rather than run an undoable
    // session.
    if (!snapshot_cy()) {
        RCLCPP_WARN(logger(), "start_selfcal: cannot snapshot cy; refusing");
        return false;
    }
    // Commit target = the mode's G2 slot (zd_index + the 0xF6 offset, 5).
    const int flash_index = impl_->cfg.zd_index + impl_->calib_bank_offset;
    // Hand the pre-session cy to the manager so its A/B re-check can measure the
    // "before" fill-rate at exactly the snapshotted value.
    if (!impl_->selfcal_->start(profile, flash_index,
                                impl_->cy_snapshot_lo, impl_->cy_snapshot_hi)) {
        impl_->cy_snapshot_valid = false;
        return false;
    }
    return true;
}

bool EspdiDevice::stop_selfcal() {
    if (!impl_->selfcal_) return false;
    impl_->selfcal_->stop();
    return true;
}

bool EspdiDevice::revert_selfcal() {
    // Roll the cy register back to its pre-session value and end the session.
    // Stop first (the SDK writes its stable value), then restore ours so it
    // wins. Clears the snapshot so close() does not roll back again.
    stop_selfcal();
    const bool ok = restore_cy();
    // Only on success: a discarded snapshot also disables close()'s rollback.
    if (ok) impl_->cy_snapshot_valid = false;
    return ok;
}

bool EspdiDevice::commit_selfcal() {
    if (!impl_->selfcal_) return false;
    const bool ok = impl_->selfcal_->commit();  // CommitToFlash (must precede Stop)
    if (ok) {
        impl_->selfcal_->stop();                // finalize the session
        // Committed to flash: the live value is the intended one; drop the
        // snapshot so it is not rolled back.
        impl_->cy_snapshot_valid = false;
    }
    return ok;
}

void EspdiDevice::keep_selfcal() {
    // Keep the converged value live; leave the session completed so a later
    // commit_selfcal() works (CommitToFlash precedes Stop). Drop the snapshot
    // (close() must not roll back) and the control gate (the worker held it up
    // to here).
    impl_->cy_snapshot_valid = false;
    if (impl_->selfcal_) impl_->selfcal_->clear_active();
}

SelfCalManager::Status EspdiDevice::selfcal_status() const {
    if (!impl_->selfcal_) return {};
    return impl_->selfcal_->status();
}

void EspdiDevice::set_pc_gate(PointCloudGate gate) {
    if (!gate_setter_allowed("set_pc_gate")) return;
    impl_->pc_gate = std::move(gate);
}

void EspdiDevice::set_color_gate(FrameStreamGate gate) {
    if (!gate_setter_allowed("set_color_gate")) return;
    impl_->color_gate = std::move(gate);
}

void EspdiDevice::set_depth_gate(FrameStreamGate gate) {
    if (!gate_setter_allowed("set_depth_gate")) return;
    impl_->depth_gate = std::move(gate);
}

bool EspdiDevice::set_ir_value(int value) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (selfcal_reject("set_ir_value")) return false;
    // Negative value = use the mode-resolved default from open().
    const char* origin = "explicit";
    if (value < 0) {
        value = impl_->ir_default_level;
        origin = "default";
    }
    if (impl_->ir_range_valid && value > impl_->ir_max_fw) {
        RCLCPP_WARN(logger(),
                    "ir_value=%d exceeds FW max %d ; clamping",
                    value, impl_->ir_max_fw);
        value = impl_->ir_max_fw;
    }
    const unsigned short ir_level = static_cast<unsigned short>(value);
    const int rc = APC_SetCurrentIRValue(impl_->handle, &impl_->sel, ir_level);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetCurrentIRValue(%u) %s", ir_level, espdi_strerror(rc).c_str());
        return false;
    }
    RCLCPP_INFO(logger(),
                "ir_value -> %u (%s)", ir_level, origin);
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
    const uint32_t temp_raw11 = (swapped >> 5) & 0x7FFu;   // 11 bits
    const int32_t  signed11 = static_cast<int32_t>(temp_raw11 ^ 0x400u) - 0x400;
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
    const double alpha_clamped = std::clamp(alpha, 0.0, 1.0);
    tp.alpha_q8    = static_cast<int>(std::lround(alpha_clamped * 256.0));
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
                alpha_clamped, delta, tp.persistence,
                impl_->spatial_filter_enabled ? "D11 disparity" : "Z14 mm",
                (enabled && !was_enabled) ? " (history reset)" : "");
    return true;
}

bool EspdiDevice::set_auto_exposure(bool enable) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (selfcal_reject("set_auto_exposure")) return false;
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
                    "APC_SetCTPropVal(AE_MODE, %s) %s",
                    enable ? "auto" : "manual", espdi_strerror(rc).c_str());
        return false;
    }
    RCLCPP_INFO(logger(), "auto_exposure -> %s",
                enable ? "on" : "off");
    return true;
}

bool EspdiDevice::set_exposure_time_step(int step) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (selfcal_reject("set_exposure_time_step")) return false;
    // Manual exposure is applied via UVC CT EXPOSURE_TIME_ABSOLUTE. The value
    // is a signed log-step (negative = darker, positive = brighter).
    // Effective only when auto-exposure is set to manual.
    const int rc = APC_SetCTPropVal(impl_->handle, &impl_->sel,
                                    CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL,
                                    static_cast<long int>(step));
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetCTPropVal(EXPOSURE_TIME_ABSOLUTE, %d) %s", step,
                    espdi_strerror(rc).c_str());
        return false;
    }
    RCLCPP_INFO(logger(), "exposure_time_step -> %d", step);
    return true;
}

bool EspdiDevice::set_auto_white_balance(bool enable) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (selfcal_reject("set_auto_white_balance")) return false;
    // Auto white-balance is switched via the UVC PU WHITE_BALANCE_AUTO_CTRL
    // register. On most depth-camera firmware AWB is fixed at the hardware
    // level: the setter returns success but the live state does not change.
    const int rc = APC_SetPUPropVal(impl_->handle, &impl_->sel,
                                    PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL,
                                    enable ? 1 : 0);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetPUPropVal(AWB_AUTO, %s) %s",
                    enable ? "on" : "off", espdi_strerror(rc).c_str());
        return false;
    }
    RCLCPP_INFO(logger(), "auto_white_balance -> %s",
                enable ? "on" : "off");
    return true;
}

bool EspdiDevice::set_power_line_frequency(int mode) {
    std::lock_guard<std::mutex> sdk_lk(impl_->sdk_mtx);
    if (!impl_->opened || !impl_->handle) return false;
    if (selfcal_reject("set_power_line_frequency")) return false;
    // Firmware only supports 50/60 Hz; UVC 0 (off) and 3 (auto) are rejected.
    if (mode != 1 && mode != 2) {
        RCLCPP_WARN(logger(),
                    "set_power_line_frequency: mode must be 1 (50Hz) or 2 (60Hz), got %d", mode);
        return false;
    }
    const int rc = APC_SetPUPropVal(impl_->handle, &impl_->sel,
                                    PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL,
                                    mode);
    if (rc != APC_OK) {
        RCLCPP_WARN(logger(),
                    "APC_SetPUPropVal(POWER_LINE_FREQ, %d) %s", mode,
                    espdi_strerror(rc).c_str());
        return false;
    }
    const char* label = (mode == 1) ? "50Hz" : "60Hz";
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
        s.ir_value = static_cast<int>(ir_cur);
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
    // Runs on the depth-fetch thread. Guard the opened/handle read and the
    // worker join/spawn with dm_quality_mtx (see the member comment).
    std::lock_guard<std::mutex> dmq_lk(impl_->dm_quality_mtx);
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
