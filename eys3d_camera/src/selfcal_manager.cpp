#include "eys3d_camera/selfcal_manager.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <utility>

#include <sys/stat.h>
#include <unistd.h>

#ifdef EYS3D_WITH_SELFCAL
#include "eys3d_selfcal.h"
#endif

namespace eys3d_camera {
namespace {

// The calibration history is the one selfk log the node keeps: it carries the
// absolute cy of every session and of every flash write, which nothing else
// records across power cycles. selfk appends to it forever and rescans it whole
// at each session start, so the node bounds it -- one rotation at 4 MiB, which
// is some nine thousand records.
constexpr long kHistoryMaxBytes = 4L * 1024 * 1024;

// Path components come from the device serial, which carries trailing padding
// on some models; keep it to what a filename takes on any filesystem.
std::string sanitise(const std::string& in) {
    std::string out;
    for (char c : in) {
        out += (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_')
               ? c : '_';
    }
    const size_t first = out.find_first_not_of('_');
    if (first == std::string::npos) {
        return std::string();
    }
    return out.substr(first, out.find_last_not_of('_') - first + 1);
}

// mkdir -p. Silent on failure: the caller's open() reports it.
void make_dirs(const std::string& path) {
    for (size_t i = 1; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            mkdir(path.substr(0, i).c_str(), 0755);
        }
    }
}

// Rotate the history when it passes the cap, keeping one generation. Returns
// the size rotated away, or -1 if nothing was moved.
long rotate_history(const std::string& path) {
    struct stat st {};
    if (stat(path.c_str(), &st) != 0 || st.st_size <= kHistoryMaxBytes) {
        return -1;
    }
    if (std::rename(path.c_str(), (path + ".1").c_str()) != 0) {
        return -1;
    }
    return static_cast<long>(st.st_size);
}

}  // namespace

#ifdef EYS3D_WITH_SELFCAL
namespace {

// EspdiDevice resolves the G2 flash slot and passes it to start(); this layer
// only writes what it is given.

// Route the selfk core's log stream into the node logger.
void sc_log_cb(EYS3D_SC_LogLevel level, const char* msg, void* user) {
    if (!user || !msg) {
        return;
    }
    auto* logger = static_cast<rclcpp::Logger*>(user);
    switch (level) {
        case EYS3D_SC_LOG_ERROR:   RCLCPP_ERROR(*logger, "[selfcal] %s", msg); break;
        case EYS3D_SC_LOG_WARNING: RCLCPP_WARN(*logger, "[selfcal] %s", msg); break;
        case EYS3D_SC_LOG_INFO:    RCLCPP_INFO(*logger, "[selfcal] %s", msg); break;
        default:                   RCLCPP_DEBUG(*logger, "[selfcal] %s", msg); break;
    }
}

inline EYS3D_SC_Handle as_handle(void* p) {
    return static_cast<EYS3D_SC_Handle>(p);
}

}  // namespace
#endif  // EYS3D_WITH_SELFCAL

SelfCalManager::SelfCalManager(void* espdi_handle, void* dev_sel, void* dev_info,
                               std::string serial, std::string config_dir,
                               rclcpp::Logger logger)
    : logger_(logger),
      serial_(std::move(serial)),
      config_dir_(std::move(config_dir)) {
#ifdef EYS3D_WITH_SELFCAL
    EYS3D_SC_CreateInfo ci{};
    ci.struct_size = sizeof(ci);
    ci.espdi_handle = espdi_handle;
    ci.espdi_device_selection = dev_sel;
    ci.espdi_device_information = dev_info;
    ci.log_callback = &sc_log_cb;
    ci.log_user_data = &logger_;
    ci.device_id = serial_.empty() ? nullptr : serial_.c_str();

    EYS3D_SC_Handle h = nullptr;
    const EYS3D_SC_Result r = EYS3D_SC_Create(&ci, &h);
    if (r == EYS3D_SC_OK) {
        sc_ = h;
        RCLCPP_INFO(logger_, "Self-calibration context created (device_id='%s')",
                    serial_.c_str());
    } else {
        RCLCPP_WARN(logger_, "EYS3D_SC_Create failed: %s",
                    EYS3D_SC_ResultToString(r));
    }
#else
    (void)espdi_handle;
    (void)dev_sel;
    (void)dev_info;
    RCLCPP_DEBUG(logger_, "Self-calibration not built in (EYS3D_WITH_SELFCAL off)");
#endif
}

SelfCalManager::~SelfCalManager() {
    stop();
#ifdef EYS3D_WITH_SELFCAL
    if (sc_) {
        EYS3D_SC_Destroy(as_handle(sc_));
        sc_ = nullptr;
    }
#endif
}

bool SelfCalManager::available() const {
    return sc_ != nullptr;
}

void SelfCalManager::set_cy_accessors(CyReadFn read_fn, CyWriteFn write_fn) {
    cy_read_fn_ = std::move(read_fn);
    cy_write_fn_ = std::move(write_fn);
}

#ifdef EYS3D_WITH_SELFCAL
namespace {
// A/B re-check tuning (run_ab_verify): compare depth fill-rate at the converged
// vs the pre-session cy on the live stream.
constexpr int   kAbPairs         = 6;      // samples/side; keep EVEN for balanced ABBA
constexpr int   kAbSkipFrames    = 3;      // frames dropped after each cy change
constexpr int   kAbMinSamples    = 4;      // need at least this many per side to judge
constexpr float kAbImproveMargin = 0.01f;  // mean per-pair diff must exceed this
constexpr float kAbStabilityMax  = 0.05f;  // max stddev of the per-pair diff to trust it

// One self-calibration session per process: the SelfK2 core binds the hardware
// handle in a process global (g_pHandleApcDI). Claimed in start(), released in
// stop().
std::atomic<void*> g_session_owner{nullptr};

float mean_of(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    double s = 0.0;
    for (float x : v) s += x;
    return static_cast<float>(s / v.size());
}
float stddev_of(const std::vector<float>& v, float m) {
    if (v.size() < 2) return 0.0f;
    double s = 0.0;
    for (float x : v) s += (x - m) * (x - m);
    return static_cast<float>(std::sqrt(s / v.size()));
}
}  // namespace
#endif

std::string SelfCalManager::history_path() const {
    const char* home = std::getenv("ROS_HOME");
    std::string root = (home != nullptr && *home != '\0') ? home : "";
    if (root.empty()) {
        const char* h = std::getenv("HOME");
        if (h == nullptr || *h == '\0') {
            return std::string();
        }
        root = std::string(h) + "/.ros";
    }
    const std::string key = sanitise(serial_);
    const std::string dir = root + "/eys3d_camera/selfcal/" +
                            (key.empty() ? std::string("unknown") : key);
    make_dirs(dir);
    return dir + "/calibration_history.jsonl";
}

bool SelfCalManager::prepare_profile(const std::string& profile,
                                     int flash_index,
                                     std::string& out) {
#ifdef EYS3D_WITH_SELFCAL
    const std::string path = config_dir_ + "/" + profile + ".json";
    std::ifstream f(path);
    if (!f) {
        RCLCPP_ERROR(logger_, "cannot open self-calibration profile '%s'",
                     path.c_str());
        return false;
    }
    // Bound the resolved slot to the eSPDI [0,9] range before a flash write.
    if (flash_index < 0 || flash_index > 9) {
        RCLCPP_ERROR(logger_,
                     "resolved flash index %d is outside the eSPDI range [0,9]",
                     flash_index);
        return false;
    }

    std::stringstream ss;
    ss << f.rdbuf();
    const std::string text = ss.str();

    // Rewrite flash.rectify_table_index to the resolved slot; require the field
    // so a commit never lands on an unknown slot.
    const std::regex re("(\"rectify_table_index\"\\s*:\\s*)(-?\\d+)");
    if (!std::regex_search(text, re)) {
        RCLCPP_ERROR(logger_,
                     "self-calibration profile '%s' has no "
                     "flash.rectify_table_index field", path.c_str());
        return false;
    }
    out = std::regex_replace(text, re, "$01" + std::to_string(flash_index));
    RCLCPP_INFO(logger_,
                "self-calibration flash target: rectify_table_index=%d "
                "(G2 user slot); G1 factory slot left untouched", flash_index);

    // Repoint the history at an absolute per-device path: selfk writes it
    // relative, which lands wherever the node was launched from and collides
    // between cameras. Spliced rather than regex_replace'd so a '$' in the
    // path is not read as a capture reference.
    const std::string hist = history_path();
    if (hist.empty()) {
        RCLCPP_WARN(logger_,
                    "neither ROS_HOME nor HOME is set; the calibration history "
                    "stays at the profile's path, relative to the working "
                    "directory");
        return true;
    }
    const std::regex path_re(
        "(\"calibration_history\"\\s*:\\s*\\{[^}]*?\"path\"\\s*:\\s*\")([^\"]*)(\")");
    std::smatch m;
    if (!std::regex_search(out, m, path_re)) {
        RCLCPP_WARN(logger_,
                    "self-calibration profile '%s' has no "
                    "logging.calibration_history.path; leaving it as written",
                    path.c_str());
        return true;
    }
    const long rotated = rotate_history(hist);
    if (rotated > 0) {
        RCLCPP_INFO(logger_,
                    "calibration history reached %ld bytes; rotated to %s.1",
                    rotated, hist.c_str());
    }
    out = out.substr(0, m.position(2)) + hist +
          out.substr(m.position(2) + m.length(2));
    RCLCPP_INFO(logger_, "self-calibration history: %s", hist.c_str());
    return true;
#else
    (void)profile;
    (void)flash_index;
    (void)out;
    return false;
#endif
}

bool SelfCalManager::start(const std::string& profile, int flash_index,
                           unsigned short init_cy_lo, unsigned short init_cy_hi) {
#ifdef EYS3D_WITH_SELFCAL
    if (!sc_) {
        return false;
    }
    if (active_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(logger_, "Self-calibration already running");
        return false;
    }
    init_cy_lo_ = init_cy_lo;
    init_cy_hi_ = init_cy_hi;
    init_cy_valid_ = true;
    // Join a previous completed session's worker (the "keep" path leaves it
    // finished but joinable) before reassigning the std::thread below.
    worker_run_.store(false, std::memory_order_release);
    if (worker_.joinable()) {
        frame_cv_.notify_all();
        worker_.join();
    }

    // Claim the process-wide session slot: refuse a different camera's manager,
    // let this one re-claim its own re-run. Every failure path below must release
    // it; stop() releases it on resolution.
    void* cur = g_session_owner.load(std::memory_order_acquire);
    if (cur != nullptr && cur != this) {
        RCLCPP_WARN(logger_,
                    "another camera's self-calibration is not yet resolved in this "
                    "process; commit or discard it first (one session at a time)");
        return false;
    }
    g_session_owner.store(this, std::memory_order_release);
    flash_index_ = flash_index;

    // Clear any session left completed-but-not-stopped by a previous run whose
    // result was kept (LoadConfig refuses otherwise). A no-op from the initial
    // created state.
    EYS3D_SC_Stop(as_handle(sc_));

    // Inject the commit target at load time: CommitToFlash uses the loaded
    // config's flash.rectify_table_index, which cannot change mid-session.
    std::string json_text;
    if (!prepare_profile(profile, flash_index, json_text)) {
        release_process_slot();
        return false;  // helper logged the reason
    }
    EYS3D_SC_Result r = EYS3D_SC_LoadConfigJson(
        as_handle(sc_), json_text.data(), json_text.size());
    if (r != EYS3D_SC_OK) {
        RCLCPP_ERROR(logger_, "EYS3D_SC_LoadConfigJson('%s') failed: %s (%s)",
                     profile.c_str(), EYS3D_SC_ResultToString(r),
                     EYS3D_SC_GetLastError(as_handle(sc_)));
        release_process_slot();
        return false;
    }
    r = EYS3D_SC_Start(as_handle(sc_));
    if (r != EYS3D_SC_OK) {
        RCLCPP_ERROR(logger_, "EYS3D_SC_Start failed: %s (%s)",
                     EYS3D_SC_ResultToString(r),
                     EYS3D_SC_GetLastError(as_handle(sc_)));
        release_process_slot();
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        frame_pending_ = false;
    }
    // Reset the previous run's terminal snapshot before the worker starts, so the
    // node cannot resolve this run on stale status before the first frame arrives.
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        status_ = Status{};
    }
    worker_run_.store(true, std::memory_order_release);
    active_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { worker_loop(); });

    RCLCPP_INFO(logger_,
                "Self-calibration session started (profile='%s', flash_index=%d)",
                profile.c_str(), flash_index);
    return true;
#else
    (void)profile;
    (void)flash_index;
    (void)init_cy_lo;
    (void)init_cy_hi;
    return false;
#endif
}

void SelfCalManager::submit_latest(const uint16_t* depth, uint32_t width,
                                   uint32_t height, float temperature_c,
                                   uint64_t timestamp_us) {
    if (!active_.load(std::memory_order_acquire) || !depth || width == 0 ||
        height == 0) {
        return;
    }
    const size_t n = static_cast<size_t>(width) * height;
    {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        frame_buf_.assign(depth, depth + n);
        frame_w_ = width;
        frame_h_ = height;
        frame_temp_ = temperature_c;
        frame_ts_ = timestamp_us;
        frame_pending_ = true;
    }
    frame_cv_.notify_one();  // outside the lock: don't wake the worker onto it
}

bool SelfCalManager::commit() {
#ifdef EYS3D_WITH_SELFCAL
    if (!sc_) {
        return false;
    }
    // SDK contract: commit only when can_commit is set (a settled result), before
    // Stop; the G2 target was fixed at start(). The node serialises this against
    // the worker (resolve_ready + gating the manual commit on an in-flight goal).
    refresh_status();
    const Status s = status();
    if (!s.can_commit) {
        RCLCPP_WARN(logger_,
                    "commit refused: no committable result yet "
                    "(state=%s, result_available=%d)",
                    s.state.c_str(), s.result_available);
        return false;
    }
    const EYS3D_SC_Result r = EYS3D_SC_CommitToFlash(as_handle(sc_));
    if (r != EYS3D_SC_OK) {
        RCLCPP_ERROR(logger_, "EYS3D_SC_CommitToFlash failed: %s (%s)",
                     EYS3D_SC_ResultToString(r),
                     EYS3D_SC_GetLastError(as_handle(sc_)));
        return false;
    }
    RCLCPP_INFO(logger_,
                "Self-calibration result committed to flash (G2 user slot)");
    refresh_status();
    return true;
#else
    return false;
#endif
}

void SelfCalManager::stop() {
    if (worker_run_.exchange(false)) {
        frame_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }
    active_.store(false, std::memory_order_release);
    release_process_slot();
#ifdef EYS3D_WITH_SELFCAL
    if (sc_) {
        EYS3D_SC_Stop(as_handle(sc_));
    }
#endif
}

void SelfCalManager::clear_active() {
    active_.store(false, std::memory_order_release);
}

#ifdef EYS3D_WITH_SELFCAL
void SelfCalManager::release_process_slot() {
    void* self = this;
    g_session_owner.compare_exchange_strong(self, nullptr);
}
#else
void SelfCalManager::release_process_slot() {}
#endif

SelfCalManager::Status SelfCalManager::status() const {
    std::lock_guard<std::mutex> lk(status_mtx_);
    return status_;
}

void SelfCalManager::worker_loop() {
#ifdef EYS3D_WITH_SELFCAL
    std::vector<uint16_t> local;
    uint32_t w = 0, h = 0;
    float temp = 0.0f;
    uint64_t ts = 0;
    auto last_warn = std::chrono::steady_clock::now();

    while (worker_run_.load(std::memory_order_acquire)) {
        {
            std::unique_lock<std::mutex> lk(frame_mtx_);
            frame_cv_.wait_for(lk, std::chrono::milliseconds(200), [this] {
                return frame_pending_ || !worker_run_.load(std::memory_order_acquire);
            });
            if (!worker_run_.load(std::memory_order_acquire)) {
                break;
            }
            if (!frame_pending_) {
                continue;
            }
            local.swap(frame_buf_);  // take the newest, leave the slot empty
            w = frame_w_;
            h = frame_h_;
            temp = frame_temp_;
            ts = frame_ts_;
            frame_pending_ = false;
        }
        if (local.empty()) {
            continue;
        }

        EYS3D_SC_Frame frame{};
        frame.struct_size = sizeof(frame);
        frame.depth_data = local.data();
        frame.width = w;
        frame.height = h;
        frame.stride_bytes = w * static_cast<uint32_t>(sizeof(uint16_t));
        frame.format = EYS3D_SC_DEPTH_U16;
        frame.temperature_c = temp;
        frame.timestamp_us = ts;

        const EYS3D_SC_Result r = EYS3D_SC_ProcessFrame(as_handle(sc_), &frame);
        refresh_status();

        // Completed: stop feeding, but keep active_ up until the node resolves the
        // run (revert/keep/commit) so no setter slips into the resolve window; the
        // context stays alive for status/result/commit until stop().
        if (r == EYS3D_SC_STATUS_COMPLETED || r == EYS3D_SC_ERROR_INVALID_STATE) {
            // Log the final result while the handle is still owned, before
            // publishing resolve_ready (the node may commit right after).
            log_result();
            // SUCCESS: verify on the live stream before the node keeps/commits.
            // resolve_ready is set last so the node acts only once the worker is
            // done with the handle.
            const bool success = (status().outcome == "SUCCESS");
            if (success) {
                run_ab_verify();  // sets ab_* + resolve_ready
            } else {
                std::lock_guard<std::mutex> lk(status_mtx_);
                status_.ab_verdict = "skipped";
                status_.resolve_ready = true;
            }
            break;  // slot stays held until stop()
        }
        if (r < 0) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_warn > std::chrono::seconds(2)) {
                RCLCPP_WARN(logger_, "EYS3D_SC_ProcessFrame: %s",
                            EYS3D_SC_ResultToString(r));
                last_warn = now;
            }
        }
    }
#endif
}

void SelfCalManager::log_result() {
#ifdef EYS3D_WITH_SELFCAL
    if (!sc_) {
        return;
    }
    EYS3D_SC_CalibrationResult res{};
    res.struct_size = sizeof(res);
    if (EYS3D_SC_GetResult(as_handle(sc_), &res) != EYS3D_SC_OK) {
        RCLCPP_INFO(logger_, "Self-calibration session ended (no result available)");
        return;
    }
    RCLCPP_INFO(logger_,
                "Self-calibration session ended: outcome=%s applied=%d "
                "committed=%d valid_ratio %.3f->%.3f (delta %+.3f) ; %s",
                EYS3D_SC_OutcomeToString(res.outcome),
                res.correction_applied, res.committed_to_flash,
                res.input_valid_ratio_first, res.input_valid_ratio_latest,
                res.input_valid_ratio_delta, res.summary);
#endif
}

#ifdef EYS3D_WITH_SELFCAL
bool SelfCalManager::next_fresh_ratio(float& out_ratio) {
    std::vector<uint16_t> local;
    uint32_t w = 0, h = 0;
    {
        std::unique_lock<std::mutex> lk(frame_mtx_);
        const bool got = frame_cv_.wait_for(
            lk, std::chrono::milliseconds(500), [this] {
                return frame_pending_ ||
                       !worker_run_.load(std::memory_order_acquire);
            });
        if (!got || !worker_run_.load(std::memory_order_acquire) ||
            !frame_pending_) {
            return false;
        }
        local.swap(frame_buf_);
        w = frame_w_;
        h = frame_h_;
        frame_pending_ = false;
    }
    const size_t n = static_cast<size_t>(w) * h;
    if (local.empty() || n == 0) {
        return false;
    }
    size_t valid = 0;
    for (size_t i = 0; i < n; ++i) {
        valid += (local[i] != 0);
    }
    out_ratio = static_cast<float>(static_cast<double>(valid) /
                                   static_cast<double>(n));
    return true;
}

void SelfCalManager::run_ab_verify() {
    // Augment (not clobber) the completed status with the re-check result.
    Status s;
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        s = status_;
    }
    auto finish = [&](const char* verdict, float ri, float rf) {
        s.ab_verdict = verdict;
        s.ab_ratio_initial = ri;
        s.ab_ratio_final = rf;
        s.ab_ratio_delta = rf - ri;
        s.resolve_ready = true;
        std::lock_guard<std::mutex> lk(status_mtx_);
        status_ = s;
    };

    if (!cy_read_fn_ || !cy_write_fn_ || !init_cy_valid_) {
        finish("skipped", 0.0f, 0.0f);
        return;
    }
    unsigned short fin_lo = 0, fin_hi = 0;
    if (!cy_read_fn_(fin_lo, fin_hi)) {  // the converged cy
        finish("inconclusive", 0.0f, 0.0f);
        return;
    }
    // cy register: signed int16 of cy*4 (Q2). One 0.25 px search step is 1 LSB,
    // so |delta| / 4 is the shift in pixels.
    const int v_init =
        static_cast<int16_t>((static_cast<uint16_t>(init_cy_hi_) << 8) | init_cy_lo_);
    const int v_final =
        static_cast<int16_t>((static_cast<uint16_t>(fin_hi) << 8) | fin_lo);
    s.cy_shift_px = std::abs(static_cast<float>(v_final - v_init)) / 4.0f;  // finish() writes s back

    // Interleave converged/pre-session cy so scene drift cancels, dropping
    // kAbSkipFrames after each change for the rectification to settle. Alternate
    // the within-pair order (ABBA) so linear drift cancels across pairs too.
    std::vector<float> fin, ini;
    auto sample_side = [&](unsigned short lo, unsigned short hi,
                           std::vector<float>& out) -> bool {
        if (!cy_write_fn_(lo, hi)) return false;
        for (int skip = 0; skip < kAbSkipFrames; ++skip) {
            float r;
            if (!next_fresh_ratio(r)) return false;
        }
        float r;
        if (!next_fresh_ratio(r)) return false;
        out.push_back(r);
        return true;
    };
    auto sample_final = [&] { return sample_side(fin_lo, fin_hi, fin); };
    auto sample_init = [&] { return sample_side(init_cy_lo_, init_cy_hi_, ini); };
    bool aborted = false;
    for (int k = 0; k < kAbPairs && worker_run_.load(std::memory_order_acquire) &&
                    !aborted;
         ++k) {
        if (k % 2 == 0) {  // even pair: final then initial
            aborted = !sample_final() || !sample_init();
        } else {           // odd pair: initial then final -> ABBA over two pairs
            aborted = !sample_init() || !sample_final();
        }
    }
    // Always leave cy at the converged value; the node then keeps it (cy stays)
    // or reverts (restores the pre-session cy) from this point.
    cy_write_fn_(fin_lo, fin_hi);

    if (static_cast<int>(fin.size()) < kAbMinSamples ||
        static_cast<int>(ini.size()) < kAbMinSamples) {
        finish("inconclusive", mean_of(ini), mean_of(fin));
        return;
    }
    const float mf = mean_of(fin), mi = mean_of(ini);
    // Judge on the per-pair difference, not each side's absolute spread: ABBA
    // pairs are frames apart, so scene motion shifts both cy measurements alike
    // and cancels in the difference.
    const size_t np = fin.size() < ini.size() ? fin.size() : ini.size();
    std::vector<float> diffs;
    diffs.reserve(np);
    for (size_t k = 0; k < np; ++k) diffs.push_back(fin[k] - ini[k]);
    const float mean_d = mean_of(diffs);
    const bool stable = stddev_of(diffs, mean_d) <= kAbStabilityMax;
    const char* verdict;
    if (!stable) {
        verdict = "inconclusive";
    } else if (mean_d > kAbImproveMargin) {
        verdict = "improved";
    } else if (mean_d < -kAbImproveMargin) {
        verdict = "worse";
    } else {
        verdict = "inconclusive";  // improvement below the margin
    }
    RCLCPP_INFO(logger_,
                "self-cal A/B re-check: initial=%.3f final=%.3f (pair-mean %+.3f, "
                "spread %.3f) -> %s",
                mi, mf, mean_d, stddev_of(diffs, mean_d), verdict);
    finish(verdict, mi, mf);
}
#else
bool SelfCalManager::next_fresh_ratio(float&) { return false; }
void SelfCalManager::run_ab_verify() {}
#endif

void SelfCalManager::refresh_status() {
#ifdef EYS3D_WITH_SELFCAL
    if (!sc_) {
        return;
    }
    EYS3D_SC_Status st{};
    st.struct_size = sizeof(st);
    if (EYS3D_SC_GetStatus(as_handle(sc_), &st) != EYS3D_SC_OK) {
        return;
    }
    Status s;
    s.running = (st.state == EYS3D_SC_STATE_RUNNING);
    s.completed = (st.state == EYS3D_SC_STATE_COMPLETED);
    s.can_commit = (st.can_commit_to_flash != 0);
    s.result_available = (st.result_available != 0);
    s.progress = st.progress;
    s.processed_frames = st.processed_frame_count;
    s.state = EYS3D_SC_StateToString(st.state);
    s.phase = EYS3D_SC_PhaseToString(st.phase);
    s.message = st.message;  // fixed-size char[], always NUL-terminated by the SDK

    // Result metrics for the operator's commit decision. Valid once a result
    // exists; GetResult returns NO_RESULT before then, so leave the defaults.
    EYS3D_SC_CalibrationResult res{};
    res.struct_size = sizeof(res);
    if (EYS3D_SC_GetResult(as_handle(sc_), &res) == EYS3D_SC_OK) {
        s.outcome = EYS3D_SC_OutcomeToString(res.outcome);
        s.valid_ratio_first = res.input_valid_ratio_first;
        s.valid_ratio_latest = res.input_valid_ratio_latest;
        s.valid_ratio_delta = res.input_valid_ratio_delta;
        s.correction_level = res.correction_level;
        s.correction_applied = (res.correction_applied != 0);
        s.committed = (res.committed_to_flash != 0);
    }

    std::lock_guard<std::mutex> lk(status_mtx_);
    // Refresh only the SDK-derived state; preserve the driver-owned A/B fields so
    // a refresh after run_ab_verify does not blank the verdict / resolve_ready.
    s.resolve_ready    = status_.resolve_ready;
    s.ab_verdict       = status_.ab_verdict;
    s.ab_ratio_initial = status_.ab_ratio_initial;
    s.ab_ratio_final   = status_.ab_ratio_final;
    s.ab_ratio_delta   = status_.ab_ratio_delta;
    s.cy_shift_px      = status_.cy_shift_px;
    status_ = std::move(s);
#endif
}

}  // namespace eys3d_camera
