#ifndef EYS3D_CAMERA__SELFCAL_MANAGER_HPP_
#define EYS3D_CAMERA__SELFCAL_MANAGER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace eys3d_camera {

// Optional self-calibration (selfk) session bound to an already-open eSPDI
// device. Owned by EspdiDevice, which feeds it the raw U16 depth via
// submit_latest() and yields the control-register channel while a session runs
// (see active()). Built only when EYS3D_WITH_SELFCAL is defined; otherwise every
// method is a safe no-op and available() returns false.
class SelfCalManager {
public:
    struct Status {
        bool running = false;
        bool completed = false;
        bool can_commit = false;
        bool result_available = false;
        float progress = 0.0f;
        uint64_t processed_frames = 0;
        std::string state;
        std::string phase;
        std::string message;

        // Result metrics (from EYS3D_SC_GetResult) for the commit decision;
        // meaningful once result_available is true.
        std::string outcome;          // SUCCESS / NO_CHANGE / INSUFFICIENT_INPUT
                                      // / TIMEOUT / FAILED / NOT_AVAILABLE
        float valid_ratio_first  = 0.0f;  // fill rate at the first accepted frame
        float valid_ratio_latest = 0.0f;  // fill rate at the latest frame
        float valid_ratio_delta  = 0.0f;  // latest - first (the improvement)
        float correction_level   = 0.0f;  // normalized magnitude, 0..1
        bool  correction_applied = false;
        bool  committed          = false;  // written to flash

        // Post-SUCCESS controlled A/B re-check on the live stream (driver-side
        // non-regression guard; see run_ab_verify). The node must wait for
        // resolve_ready before acting on a SUCCESS: it is only set once the
        // re-check (or its skip) has finished.
        bool  resolve_ready    = false;
        std::string ab_verdict;            // improved / worse / inconclusive / skipped
        float ab_ratio_initial = 0.0f;     // mean fill-rate measured at the pre-session cy
        float ab_ratio_final   = 0.0f;     // mean fill-rate measured at the converged cy
        float ab_ratio_delta   = 0.0f;     // final - initial (driver-side controlled measure)
        float cy_shift_px      = 0.0f;     // |converged cy - pre-session cy| in pixels,
                                           // measured from the hardware registers
    };

    using CyReadFn  = std::function<bool(unsigned short&, unsigned short&)>;
    using CyWriteFn = std::function<bool(unsigned short, unsigned short)>;

    // Binds to the caller's existing eSPDI device. config_dir holds the JSON
    // profiles. serial is passed as the persistent history device id.
    SelfCalManager(void* espdi_handle, void* dev_sel, void* dev_info,
                   std::string serial, std::string config_dir,
                   rclcpp::Logger logger);
    ~SelfCalManager();

    SelfCalManager(const SelfCalManager&) = delete;
    SelfCalManager& operator=(const SelfCalManager&) = delete;

    // True when the selfk context exists (feature built in and Create succeeded).
    bool available() const;

    // Inject the device's cy register accessors (both take the device SDK lock).
    // Used by the post-SUCCESS A/B re-check to toggle between the converged and
    // the pre-session cy on the live stream. Optional; without them the re-check
    // is skipped and a SUCCESS is trusted as-is.
    void set_cy_accessors(CyReadFn read_fn, CyWriteFn write_fn);

    // Load <config_dir>/<profile>.json, start a session, and spawn the worker.
    // flash_index is the fully resolved rectify-table slot a later commit will
    // write (the caller has already mapped it to the G2 user bank).
    // init_cy_{lo,hi} is the pre-session cy register the A/B re-check restores to
    // when measuring the "before" fill-rate. Returns false on error or if a
    // session is already running.
    bool start(const std::string& profile, int flash_index,
               unsigned short init_cy_lo, unsigned short init_cy_hi);

    // Newest U16 depth raster -> single-slot latest-frame queue (drop old).
    // Either domain works (Z14 mm or D11 disparity): selfk scores only the
    // non-zero valid-pixel ratio, which is identical between them. Cheap and
    // lock-guarded; safe from the depth-fetch thread. No-op when no session runs.
    void submit_latest(const uint16_t* depth, uint32_t width, uint32_t height,
                       float temperature_c, uint64_t timestamp_us);

    // Commit the latest valid candidate to flash. Control thread only. (P3.)
    bool commit();

    // Stop the session and join the worker. Idempotent.
    void stop();

    // Drop the control-register gate without stopping the SDK session, so a
    // "keep result" resolution unblocks the setters while leaving the completed
    // context alive for a later commit(). Called by the node on resolution.
    void clear_active();

    // True while a session is active — drives the driver's calibration-mode gate.
    bool active() const { return active_.load(std::memory_order_acquire); }

    Status status() const;

private:
    void worker_loop();
    void refresh_status();
    void log_result();  // GetResult -> one INFO line when a session ends
    // Post-SUCCESS non-regression guard: interleave the converged cy and the
    // pre-session cy on the live stream, measure fill-rate on each, and record a
    // verdict (improved / worse / inconclusive) plus a stability check into
    // status_. Runs on the worker thread; leaves cy at the converged value.
    void run_ab_verify();
    // Wait for the next fresh depth frame and return its non-zero fill-rate.
    // Worker thread only; returns false if the worker is asked to stop or no
    // frame arrives in time.
    bool next_fresh_ratio(float& out_ratio);
    // Release this manager's claim on the process-wide selfk session slot (the
    // SelfK2 core binds the hardware handle in a process global, so only one
    // session may run per process). A no-op if this manager does not hold it.
    void release_process_slot();
    // Read <config_dir>/<profile>.json and rewrite the two fields the node owns:
    // flash.rectify_table_index (the caller's resolved G2 user slot) and
    // logging.calibration_history.path. Returns the patched JSON in `out`.
    bool prepare_profile(const std::string& profile, int flash_index,
                         std::string& out);

    // Absolute path for this device's calibration history, under ROS_HOME
    // (~/.ros when unset). Empty when neither ROS_HOME nor HOME is set, which
    // leaves the profile's own path in place.
    std::string history_path() const;

    rclcpp::Logger logger_;
    std::string serial_;
    std::string config_dir_;
    int flash_index_ = -1;

    void* sc_ = nullptr;   // EYS3D_SC_Handle (opaque)

    std::atomic<bool> active_{false};
    std::atomic<bool> worker_run_{false};
    std::thread worker_;

    // cy register accessors + the pre-session cy for the A/B re-check.
    CyReadFn cy_read_fn_;
    CyWriteFn cy_write_fn_;
    unsigned short init_cy_lo_ = 0;
    unsigned short init_cy_hi_ = 0;
    bool init_cy_valid_ = false;

    // Single-slot latest frame handed from the fetch thread to the worker.
    std::mutex frame_mtx_;
    std::condition_variable frame_cv_;
    std::vector<uint16_t> frame_buf_;
    uint32_t frame_w_ = 0;
    uint32_t frame_h_ = 0;
    float frame_temp_ = 0.0f;
    uint64_t frame_ts_ = 0;
    bool frame_pending_ = false;

    mutable std::mutex status_mtx_;
    Status status_;
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__SELFCAL_MANAGER_HPP_
