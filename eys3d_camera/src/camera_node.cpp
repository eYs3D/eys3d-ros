#include "eys3d_camera/camera_node.hpp"

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/intra_process_setting.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace eys3d_camera {

namespace {
rclcpp::QoS image_qos() {
    return rclcpp::SensorDataQoS();
}

rclcpp::QoS info_qos() {
    // RELIABLE + VOLATILE + KEEP_LAST(5) matches image_pipeline /
    // message_filters::Subscriber defaults. Late subscribers wait for the
    // next periodic publish instead of a latched value.
    rclcpp::QoS q(5);
    q.reliable();
    return q;
}

// No ranges here: declare_parameter() throws on one, unhandled across a
// component boundary. build_device_config() enforces them.
rcl_interfaces::msg::ParameterDescriptor ro(const char* text) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = text;
    d.read_only = true;
    return d;
}

rcl_interfaces::msg::ParameterDescriptor rw(const char* text) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = text;
    return d;
}

std::string join_frame(const std::string& prefix, const std::string& leaf) {
    if (prefix.empty()) return leaf;
    return prefix + "_" + leaf;
}

// Normalise the launch-supplied model token. Trims whitespace, accepts a
// small set of casual aliases (G100+, G100Plus), and otherwise returns
// the original string for the YAML loader and PID lookup to handle.
std::string normalize_model(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    s = s.substr(first, last - first + 1);
    if (s == "G100+" || s == "G100Plus" || s == "g100p" || s == "G100p") return "G100P";
    if (s == "G100+i" || s == "g100pi" || s == "G100pi")                return "G100Pi";
    if (s == "r77")                                                     return "R77";
    if (s == "g62")                                                     return "G62";
    return s;
}

// rclcpp::init() on Foxy installs a SIGINT handler only. Re-raise SIGINT
// from SIGTERM so a container runtime's stop reaches the same rclcpp
// deferred-shutdown path. Installed once per process, covering both the
// executable and the component container.
void install_sigterm_to_shutdown() {
    static std::once_flag once;
    std::call_once(once, [] {
        struct sigaction sa{};
        sa.sa_handler = [](int) { raise(SIGINT); };
        sigemptyset(&sa.sa_mask);
        sigaction(SIGTERM, &sa, nullptr);
    });
}
}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("eys3d_camera", options),
      device_(std::make_unique<EspdiDevice>()) {
    install_sigterm_to_shutdown();
    declare_params();

    // Some rclcpp versions reject a TRANSIENT_LOCAL publisher in a node with
    // intra-process comms on, so IPC is disabled for this publisher only;
    // every other publisher follows the node-level setting.
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> tf_static_opts;
    tf_static_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
        this, tf2_ros::StaticBroadcasterQoS(), std::move(tf_static_opts));

    camera_name_        = get_parameter("camera_name").as_string();
    dm_quality_cfg_dir_ = get_parameter("dm_quality_cfg_dir").as_string();

    // Frame ids default to <camera_name>_<leaf>, ensuring TF uniqueness in
    // multi-camera setups. Individual frame names can be overridden via
    // ROS parameters for integration into an existing TF tree.
    base_frame_        = get_parameter("base_frame").as_string();
    left_color_frame_  = get_parameter("left_color_frame").as_string();
    right_color_frame_ = get_parameter("right_color_frame").as_string();
    depth_frame_       = get_parameter("depth_frame").as_string();
    points_frame_      = get_parameter("points_frame").as_string();
    if (base_frame_.empty())        base_frame_        = join_frame(camera_name_, "link");
    if (left_color_frame_.empty())  left_color_frame_  = join_frame(camera_name_, "left_color_frame");
    if (right_color_frame_.empty()) right_color_frame_ = join_frame(camera_name_, "right_color_frame");
    if (depth_frame_.empty())       depth_frame_       = join_frame(camera_name_, "depth_frame");
    if (points_frame_.empty())      points_frame_      = join_frame(camera_name_, "points_frame");
    left_color_optical_frame_  = join_frame(camera_name_, "left_color_optical_frame");
    right_color_optical_frame_ = join_frame(camera_name_, "right_color_optical_frame");
    depth_optical_frame_       = join_frame(camera_name_, "depth_optical_frame");

    VideoMode vm;
    std::string err;
    if (!load_video_mode(vm, err)) {
        RCLCPP_ERROR(get_logger(),
                     "Video mode lookup failed: %s; node will idle", err.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(),
                "Selected mode %d: %s", vm.id, vm.name.c_str());

    // The video-mode catalogue declares whether the wire frame is wide
    // L|R. LR modes always split; non-LR modes never do.
    split_color_ = vm.color_split;

    // Unprefixed: the launch sets the ROS namespace. camera_info must stay a
    // sibling of image_raw for image_transport::CameraSubscriber.
    const std::string color_topic       = "left_color";
    const std::string color_right_topic = "right_color";
    const std::string depth_topic       = "depth";
    const std::string points_topic      = "depth/points";

    if (vm.has_color) {
        // espdi_device decodes every wire format to rgb8, so the publish path is
        // uniform; the wide L|R split just slices that raster row by row.
        pub_color_ = create_publisher<sensor_msgs::msg::Image>(color_topic + "/image_raw", image_qos());
        pub_color_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            color_topic + "/camera_info", info_qos());
        if (split_color_) {
            pub_color_right_ = create_publisher<sensor_msgs::msg::Image>(color_right_topic + "/image_raw", image_qos());
            pub_color_right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                color_right_topic + "/camera_info", info_qos());
        }
    }
    if (vm.has_depth) {
        pub_depth_  = create_publisher<sensor_msgs::msg::Image>(depth_topic + "/image_raw", image_qos());
        pub_depth_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            depth_topic + "/camera_info", info_qos());
        pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic, image_qos());
    }

    // build_device_config() throws on an out-of-range depth-range or filter
    // value. Catch it and idle like any other config error, rather than letting
    // the exception escape the constructor (std::terminate standalone; a failed
    // component load inside a container).
    DeviceConfig initial_cfg;
    try {
        initial_cfg = build_device_config(vm);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid configuration: %s; node will idle", e.what());
        return;
    }
    {
        // Initial population. No reader can race here — the parameter
        // callback is installed below this point.
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cached_cfg_ = initial_cfg;
    }
    // A failed open does not return: the service, watchdog and diagnostics
    // surface is built regardless and the failure drops into the reconnect
    // loop, which also covers a camera that enumerates shortly after start.
    // Every device_ accessor below is safe on a closed handle.
    const bool device_opened = device_->open(cached_cfg_);
    if (!device_opened) {
        RCLCPP_ERROR(get_logger(),
                     "EspdiDevice::open() failed. Check that no other process holds the device "
                     "(lsof /dev/video*), the model PID matches the connected camera, and the "
                     "USB cable supports the required bandwidth. The watchdog will keep retrying.");
    } else {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        cached_calib_ = device_->calibration();
    }

    if (device_opened) publish_static_tf();

    // Apply initial CT/PU values (ir_value, AE, AWB, exposure_time_step)
    // before registering the parameter-set callback to avoid feedback from
    // the initial declares.
    declare_and_apply_runtime_params();
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& p){ return on_set_parameters(p); });

    // pause / standby / hw_reset. The state contract is on StreamState in
    // espdi_device.hpp. Both stream controls move colour and depth together:
    // interleave modes need both halves of the V4L2 stream active.
    using SetBool = std_srvs::srv::SetBool;
    srv_pause_ = create_service<SetBool>(
        "pause",
        [this](const std::shared_ptr<SetBool::Request> req,
               std::shared_ptr<SetBool::Response> res) {
            if (device_->selfcal_active()) {
                res->success = false;
                res->message = "refused: a self-calibration session is in progress";
                return;
            }
            const bool ok = device_->pause(req->data);
            res->success = ok;
            res->message = std::string("stream ") + (req->data ? "paused" : "resumed");
        });
    srv_standby_ = create_service<SetBool>(
        "standby",
        [this](const std::shared_ptr<SetBool::Request> req,
               std::shared_ptr<SetBool::Response> res) {
            if (device_->selfcal_active()) {
                res->success = false;
                res->message = "refused: a self-calibration session is in progress";
                return;
            }
            // Gated on connected: close() leaves stream_state stale.
            const bool in_standby =
                device_->stream_state() == EspdiDevice::StreamState::Standby;
            if (conn_state_.load() == ConnState::kStreaming &&
                req->data == in_standby) {
                res->success = true;
                res->message = std::string("standby already ") +
                               (req->data ? "entered" : "exited");
                return;
            }

            if (req->data) {
                user_wants_standby_.store(true, std::memory_order_release);
            } else {
                // Re-anchor and force a DM_Quality re-apply before the resume
                // restarts the fetch threads — see try_reconnect(). Defensive:
                // firmware that resets either on STREAMON must not stamp the
                // first post-resume frame in the past.
                for (auto& latch : off_raster_warned_) latch.store(false, std::memory_order_relaxed);
    dm_quality_applied_.store(false);
                hw_anchor_us_.store(0, std::memory_order_relaxed);
                ros_anchor_ns_.store(0, std::memory_order_relaxed);
                time_anchor_set_.store(false, std::memory_order_release);
            }
            const bool ok = device_->standby(req->data);
            res->success = ok;
            if (ok) {
                if (!req->data) {
                    // Before the calls below, which can throw.
                    user_wants_standby_.store(false, std::memory_order_release);
                    // The reopen restores the launch-time cfg.ir_value; a
                    // runtime override lived only in the cleared FW register.
                    device_->set_ir_value(
                        get_parameter("ir_value").as_int());
                    // Re-sync the flash-persisted CT/PU settings from the
                    // firmware into the parameter store.
                    resync_ct_pu_from_device();
                    watchdog_prev_stats_ = device_->stats();
                    color_silent_seconds_.store(0, std::memory_order_relaxed);
                    depth_silent_seconds_.store(0, std::memory_order_relaxed);
                    startup_grace_seconds_.store(0, std::memory_order_relaxed);
                    cold_start_reopens_.store(0, std::memory_order_relaxed);
                    color_armed_.store(false, std::memory_order_relaxed);
                    depth_armed_.store(false, std::memory_order_relaxed);
                }
                res->message = std::string("standby ") + (req->data ? "entered" : "exited");
            } else {
                // Handle is known closed; skip the watchdog timeout.
                user_wants_standby_.store(false, std::memory_order_release);
                if (!req->data) declare_disconnected();
                res->message = req->data
                    ? "standby(true) failed ; device is not open"
                    : "standby(false) reopen failed ; device left closed";
            }
        });
    using Empty = std_srvs::srv::Empty;
    srv_hw_reset_ = create_service<Empty>(
        "hw_reset",
        [this](const std::shared_ptr<Empty::Request>,
               std::shared_ptr<Empty::Response>) {
            // Empty service has no response field, so a refusal can only be
            // logged; re-enumerating the device would kill a running session.
            if (device_->selfcal_active()) {
                RCLCPP_WARN(get_logger(),
                            "hw_reset refused: a self-calibration session is in "
                            "progress (stop it first)");
                return;
            }
            RCLCPP_WARN(get_logger(),
                        "hw_reset: resetting the camera over USB");
            // stop() (join the fetch threads while the link is still up),
            // reset_usb() (write the detach sequence; the host re-enumerates),
            // then close() (release the now-stale handle).
            device_->stop();
            device_->reset_usb();
            device_->close();
            // Drive the node to Disconnected and let the watchdog reopen by the
            // pinned serial / port. Clearing user_wants_standby_ lets the
            // watchdog resume if the reset was issued from Standby. Lock-free:
            // the executor is single-threaded and stop()/close() are idempotent.
            user_wants_standby_.store(false, std::memory_order_release);
            conn_state_.store(ConnState::kDisconnected);
            watchdog_prev_stats_ = device_->stats();
            color_armed_.store(false, std::memory_order_relaxed);
            depth_armed_.store(false, std::memory_order_relaxed);
            startup_grace_seconds_.store(0, std::memory_order_relaxed);
            cold_start_reopens_.store(0, std::memory_order_relaxed);
            color_silent_seconds_.store(0, std::memory_order_relaxed);
            depth_silent_seconds_.store(0, std::memory_order_relaxed);
            reconnect_poll_counter_ = 0;
        });

    // Self-calibration, exposed only when selfcal_enable is set.
    //   /<cam>/selfcal/run    (action)  — run one session to completion and
    //                                     resolve it (revert / keep / commit)
    //   /<cam>/selfcal/commit (Trigger) — persist a kept result to the G2 slot
    if (get_parameter("selfcal_enable").as_bool()) {
        act_selfcal_run_ = rclcpp_action::create_server<SelfCalAction>(
            this, "selfcal/run",
            std::bind(&CameraNode::selfcal_handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&CameraNode::selfcal_handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&CameraNode::selfcal_handle_accepted, this,
                      std::placeholders::_1));
        // Irreversible flash write; only ever on this explicit call.
        srv_selfcal_commit_ = create_service<std_srvs::srv::Trigger>(
            "selfcal/commit",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                // Refuse while a run is in flight (would race the worker on the
                // handle); the manual commit persists a kept result afterwards.
                if (selfcal_run_goal_) {
                    res->success = false;
                    res->message = "commit refused: a self-calibration run is in "
                                   "progress; wait for it to finish";
                    return;
                }
                res->success = device_->commit_selfcal();
                res->message = res->success
                    ? "self-calibration committed to flash (G2 user slot)"
                    : "commit refused: no committable result (see node log)";
            });
    }

    // Five tasks (device / color / depth / pointcloud / thermal) contribute
    // one DiagnosticStatus each per array. diagnostics_rate_hz below 0.001
    // disables the Updater entirely.
    const double diag_hz = get_parameter("diagnostics_rate_hz").as_double();
    if (diag_hz >= 0.001) {
        updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        // Pin the device serial as hardware_id — immutable after open().
        const auto sn = device_->serial_number();
        updater_->setHardwareID(sn.empty() ? "eys3d_camera" : sn);
        updater_->setPeriod(1.0 / diag_hz);
        updater_->add("device",     this, &CameraNode::diagnose_device);
        updater_->add("color",      this, &CameraNode::diagnose_color);
        updater_->add("depth",      this, &CameraNode::diagnose_depth);
        updater_->add("pointcloud", this, &CameraNode::diagnose_pc);
        updater_->add("thermal",    this, &CameraNode::diagnose_thermal);
        // Self-calibration is an on-demand action, fully described by the
        // selfcal/run feedback + result; it is not a continuous-stream health
        // subsystem, so it gets no /diagnostics task.
        prev_stats_ = device_->stats();
        prev_stats_wall_ = now();
        RCLCPP_INFO(get_logger(), "/diagnostics rate: %.2f Hz", diag_hz);
    }



    // Callbacks are wired only for channels that the active mode produces;
    // unused callbacks no-op when the matching publisher is null.
    auto color_cb = [this](FrameBuffer&& f) {
        if (split_color_) publish_split_color(std::move(f));
        else              on_color(std::move(f));
    };
    auto depth_cb = [this](FrameBuffer&& f) { on_depth(std::move(f)); };
    auto pc_cb    = [this](std::vector<uint8_t>&& xyz_bytes, uint32_t n,
                           uint32_t point_step, uint64_t ts) {
        on_point_cloud(std::move(xyz_bytes), n, point_step, ts);
    };
    cached_color_cb_ = vm.has_color ? color_cb : ColorFrameCb{};
    cached_depth_cb_ = vm.has_depth ? depth_cb : DepthFrameCb{};
    cached_pc_cb_    = vm.has_depth ? pc_cb    : PointCloudCb{};
    color_configured_ = vm.has_color;
    depth_configured_ = vm.has_depth;
    color_rectified_  = color_is_rectified(vm.depth_data_type);

    // Install the subscriber gates BEFORE start() spawns the fetch threads
    // so the threads observe a fully-initialised gate on the very first
    // frame; setting them after start() is a data race on std::function.
    device_->set_pc_gate([this]() {
        return pub_points_ && pub_points_->get_subscription_count() > 0;
    });
    device_->set_color_gate([this]() {
        const bool left_subs  = pub_color_ &&
                                pub_color_->get_subscription_count() > 0;
        const bool right_subs = pub_color_right_ &&
                                pub_color_right_->get_subscription_count() > 0;
        // Keep the color path running whenever colored_pointcloud is
        // on and a /depth/points subscriber exists, so the cloud paints
        // from a current color frame.
        const bool pc_needs_color = cached_cfg_.colored_pointcloud
            && pub_points_
            && pub_points_->get_subscription_count() > 0;
        return left_subs || right_subs || pc_needs_color;
    });
    device_->set_depth_gate([this]() {
        return pub_depth_ && pub_depth_->get_subscription_count() > 0;
    });
    if (device_opened) {
        device_->start(ColorFrameCb(cached_color_cb_),
                       DepthFrameCb(cached_depth_cb_),
                       PointCloudCb(cached_pc_cb_));
    } else {
        // The initial open failed: enter the reconnect loop rather than
        // stream. The watchdog polls kDisconnected and drives try_reconnect(),
        // which publishes the static TF and re-applies CT/PU once the device
        // appears.
        conn_state_.store(ConnState::kDisconnected);
    }

    // 1 Hz watchdog: detect USB disconnect and drive the reconnect loop.
    watchdog_prev_stats_ = device_->stats();
    watchdog_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]{ watchdog_tick(); });

    const char* color_topic_label =
        split_color_ ? "left_color + right_color" : "left_color";
    if (device_opened) {
        RCLCPP_INFO(get_logger(),
                    "eys3d_camera '%s' running. Streams under '/%s/' (%s, depth) plus depth/points.",
                    camera_name_.c_str(), camera_name_.c_str(), color_topic_label);
    } else {
        RCLCPP_WARN(get_logger(),
                    "eys3d_camera '%s' started without a device; waiting for the camera "
                    "to appear on '/%s/'.",
                    camera_name_.c_str(), camera_name_.c_str());
    }
}

void CameraNode::declare_disconnected() {
    device_->stop();
    device_->close();
    conn_state_.store(ConnState::kDisconnected);
    color_armed_.store(false, std::memory_order_relaxed);
    depth_armed_.store(false, std::memory_order_relaxed);
    color_silent_seconds_.store(0, std::memory_order_relaxed);
    depth_silent_seconds_.store(0, std::memory_order_relaxed);
    startup_grace_seconds_.store(0, std::memory_order_relaxed);
    // Cadence not reset here: the standby handler may retry faster than the poll.
}

void CameraNode::watchdog_tick() {
    if (!device_) return;

    // Operator-requested Standby suppresses the watchdog: with no fetch
    // threads the stats never advance, so the zero-frame check would misread
    // the pause as a disconnect. Skip the tick, refresh the stats baseline,
    // and bail.
    if (user_wants_standby_.load(std::memory_order_acquire)) {
        watchdog_prev_stats_ = device_->stats();
        color_silent_seconds_.store(0, std::memory_order_relaxed);
        depth_silent_seconds_.store(0, std::memory_order_relaxed);
        return;
    }

    if (conn_state_.load() == ConnState::kStreaming) {
        const auto cur = device_->stats();
        const uint64_t color_frames = cur.color_input_total - watchdog_prev_stats_.color_input_total;
        const uint64_t depth_frames = cur.depth_input_total - watchdog_prev_stats_.depth_input_total;
        watchdog_prev_stats_ = cur;

        // Frames since the baseline; the lifetime total is never reset.
        if (color_frames > 0) color_armed_.store(true, std::memory_order_relaxed);
        if (depth_frames > 0) depth_armed_.store(true, std::memory_order_relaxed);
        const bool color_armed = color_armed_.load(std::memory_order_relaxed);
        const bool depth_armed = depth_armed_.load(std::memory_order_relaxed);

        // The SDK starts capture on the first fetch and leaves the stream
        // unstarted when that fails, retrying forever; only a reopen clears it.
        // 10 s, not the steady-state 3 s: R77 mode 1 at 7 fps over USB 2.0 takes
        // several seconds to deliver its first frame.
        const bool color_pending = color_configured_ && !color_armed;
        const bool depth_pending = depth_configured_ && !depth_armed;
        if (color_pending || depth_pending) {
            const int grace_seconds =
                startup_grace_seconds_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (grace_seconds >= 10) {
                // A standby() that landed between the top-of-tick guard and
                // here would also produce zero frames. Bail rather than tear
                // the SDK down.
                if (user_wants_standby_.load(std::memory_order_acquire)) return;
                const char* which = (color_pending && depth_pending) ? "colour+depth"
                                    : color_pending ? "colour" : "depth";
                // A reopen clears a stream the SDK failed to start, not a mode
                // the link cannot carry; past the cap the device stays open so
                // a stream that does work keeps running.
                if (cold_start_reopens_.load(std::memory_order_relaxed)
                        < kMaxColdStartReopens) {
                    cold_start_reopens_.fetch_add(1, std::memory_order_relaxed);
                    RCLCPP_ERROR(get_logger(),
                                 "watchdog: %s delivered no frame within %d s of "
                                 "open; declaring camera disconnected",
                                 which, grace_seconds);
                    declare_disconnected();
                    return;
                }
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 60000,
                    "watchdog: %s has delivered no frame after %d reopens; "
                    "leaving the device open. Check that the active mode fits "
                    "the negotiated USB link",
                    which, kMaxColdStartReopens);
            }
            // Fall through: a stream that did arm is still watched below.
        } else {
            cold_start_reopens_.store(0, std::memory_order_relaxed);
        }

        // At least one stream is live. An armed stream that then goes silent
        // is a per-stream stall — e.g. depth wedges in firmware while colour
        // keeps flowing — which reconnect recovers even though the other
        // stream is still delivering frames.
        const bool color_silent = color_armed && color_frames == 0;
        const bool depth_silent = depth_armed && depth_frames == 0;
        // A stream that delivers resets only its own counter.
        const int color_silent_seconds =
            color_silent ? color_silent_seconds_.fetch_add(1, std::memory_order_relaxed) + 1
                         : (color_silent_seconds_.store(0, std::memory_order_relaxed), 0);
        const int depth_silent_seconds =
            depth_silent ? depth_silent_seconds_.fetch_add(1, std::memory_order_relaxed) + 1
                         : (depth_silent_seconds_.store(0, std::memory_order_relaxed), 0);

        const int silent_seconds = std::max(color_silent_seconds, depth_silent_seconds);
        if (silent_seconds >= 3) {
            if (user_wants_standby_.load(std::memory_order_acquire)) return;
            const bool color_stalled = color_silent_seconds >= 3;
            const bool depth_stalled = depth_silent_seconds >= 3;
            const char* which = (color_stalled && depth_stalled) ? "colour+depth"
                                : color_stalled ? "colour" : "depth";
            RCLCPP_ERROR(get_logger(),
                         "watchdog: %s stream silent for %d s; "
                         "declaring camera disconnected",
                         which, silent_seconds);
            declare_disconnected();
        }
        return;
    }

    // kDisconnected: poll every 2 s for the device to come back.
    if (++reconnect_poll_counter_ < 2) return;
    reconnect_poll_counter_ = 0;
    const uint64_t attempts_now = reconnect_attempts_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (try_reconnect()) {
        RCLCPP_INFO(get_logger(),
                    "watchdog: reconnect succeeded after %lu attempt(s)",
                    static_cast<unsigned long>(attempts_now));
        watchdog_prev_stats_ = device_->stats();
        conn_state_.store(ConnState::kStreaming);
        color_armed_.store(false, std::memory_order_relaxed);
        depth_armed_.store(false, std::memory_order_relaxed);
        startup_grace_seconds_.store(0, std::memory_order_relaxed);
        color_silent_seconds_.store(0, std::memory_order_relaxed);
        depth_silent_seconds_.store(0, std::memory_order_relaxed);
        reconnect_attempts_.store(0, std::memory_order_relaxed);
    }
}

bool CameraNode::try_reconnect() {
    DeviceConfig cfg_snapshot;
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cfg_snapshot = cached_cfg_;
    }
    if (!device_->open(cfg_snapshot)) return false;
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        cached_calib_ = device_->calibration();
    }
    // Re-publish the static TF from the (re-read) calibration. Latched
    // TRANSIENT_LOCAL, so this is harmless on an ordinary reconnect and is the
    // only place the tree is published when the initial open failed and the
    // device appeared later.
    publish_static_tf();
    // Reinstall gates before start() — see constructor for the rationale.
    device_->set_pc_gate([this]() {
        return pub_points_ && pub_points_->get_subscription_count() > 0;
    });
    device_->set_color_gate([this]() {
        const bool left_subs  = pub_color_ &&
                                pub_color_->get_subscription_count() > 0;
        const bool right_subs = pub_color_right_ &&
                                pub_color_right_->get_subscription_count() > 0;
        // Keep the color path running whenever colored_pointcloud is
        // on and a /depth/points subscriber exists, so the cloud paints
        // from a current color frame.
        const bool pc_needs_color = cached_cfg_.colored_pointcloud
            && pub_points_
            && pub_points_->get_subscription_count() > 0;
        return left_subs || right_subs || pc_needs_color;
    });
    device_->set_depth_gate([this]() {
        return pub_depth_ && pub_depth_->get_subscription_count() > 0;
    });
    // USB re-enumeration restarts the hw timestamp counter and power-cycles
    // the registers, so the anchor and the DM_Quality flag are reset before
    // start() respawns the fetch threads. Both anchor halves must go to zero
    // -- stamp_from_hw_us elects its initialiser by CAS-from-zero.
    for (auto& latch : off_raster_warned_) latch.store(false, std::memory_order_relaxed);
    dm_quality_applied_.store(false);
    hw_anchor_us_.store(0, std::memory_order_relaxed);
    ros_anchor_ns_.store(0, std::memory_order_relaxed);
    time_anchor_set_.store(false, std::memory_order_release);
    device_->start(ColorFrameCb(cached_color_cb_),
                   DepthFrameCb(cached_depth_cb_),
                   PointCloudCb(cached_pc_cb_));
    // Re-apply IR — projector boots OFF after every open().
    device_->set_ir_value(get_parameter("ir_value").as_int());
    // Re-sync the flash-persisted CT/PU settings from the firmware into the
    // parameter store.
    resync_ct_pu_from_device();
    return true;
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(get_logger(), "~CameraNode()");
    // Service and parameter callbacks go first, so no executor thread can
    // enter pause() / standby() / hw_reset() / on_set_parameters() against a
    // device that is mid-shutdown.
    srv_pause_.reset();
    srv_standby_.reset();
    srv_hw_reset_.reset();
    if (selfcal_run_timer_) selfcal_run_timer_->cancel();
    selfcal_run_timer_.reset();
    // Abort an in-flight self-cal goal so the client's result future is
    // fulfilled instead of hanging on a silently-destroyed goal.
    if (selfcal_run_goal_) {
        auto res = std::make_shared<SelfCalAction::Result>();
        res->outcome = "FAILED";
        res->message = "node shutting down; self-calibration aborted";
        selfcal_run_goal_->abort(res);
        selfcal_run_goal_.reset();
    }
    act_selfcal_run_.reset();
    srv_selfcal_commit_.reset();
    param_cb_handle_.reset();
    // Cancel every wall timer before any member destructs: on a
    // multi-threaded executor a tick on another thread would otherwise
    // dereference device_ after it is destroyed. Updater owns an internal
    // timer, stopped by releasing the unique_ptr below.
    if (watchdog_timer_)     watchdog_timer_->cancel();
    updater_.reset();
    if (device_) device_->stop();
    RCLCPP_INFO(get_logger(), "~CameraNode() done");
}

void CameraNode::declare_params() {
    // Helper: any parameter that the node reads exactly once at startup
    // (or only when a re-open happens) is declared read_only so a
    // ros2 param set against it fails fast instead of returning success
    // and silently doing nothing.

    // Identity + binding (launch-only)
    declare_parameter<std::string>("camera_name",       "eys3d_camera",
        ro("ROS namespace and frame-id prefix."));
    declare_parameter<std::string>("model",             "G100P",
        ro("Camera model (G100P, G100Pi, R77, G62). Selects the video-mode catalogue."));
    declare_parameter<int>        ("mode_id",           -1,
        ro("Index into the per-model video-mode catalogue; -1 selects the signature mode for the negotiated USB link."));
    declare_parameter<std::string>("video_modes_dir",   "",
        ro("Directory holding the video-mode catalogues; empty uses the installed share directory."));
    declare_parameter<std::string>("dev_serial_number", "",
        ro("Bind to the device whose serial number contains this substring; empty binds the first device of the model."));
    declare_parameter<std::string>("usb_port",          "",
        ro("Bind by USB topology path, e.g. 2-3:1.0; empty binds the first device of the model."));
    declare_parameter<int>        ("depth_near_mm",     -1,
        ro("Near cutoff in mm for depth and the cloud; nearer pixels are zeroed. -1 selects the per-model default."));
    declare_parameter<int>        ("depth_far_mm",      -1,
        ro("Far cutoff in mm for depth and the cloud; farther pixels are zeroed. -1 selects the per-model default."));
    // Declared here, with the open()-time parameters, so the launch value
    // reaches DeviceConfig in time for the IR pre-open write.
    declare_parameter<int>        ("ir_value",          -1,
        rw("IR projector level. -1 resolves per mode: the model default when the mode carries depth or the module is mono, off for a colour-only mode on a colour sensor. 0 forces it off. Firmware range 0-6 on G100+/R77, 0-96 on G62."));
    declare_parameter<bool>       ("colored_pointcloud",       false,
        ro("Publish XYZRGB sampled from the latest colour frame; depth-only modes fall back to XYZ."));
    declare_parameter<bool>       ("spatial_filter",           false,
        ro("Enable the disparity-domain edge-aware IIR filter."));
    declare_parameter<double>     ("spatial_filter_alpha",     0.5,
        ro("Spatial smoothing strength, 0.0 to 1.0."));
    declare_parameter<int>        ("spatial_filter_delta",     20,
        ro("Spatial edge threshold in raw disparity units, 1 to 4095."));
    declare_parameter<int>        ("spatial_filter_magnitude", 2,
        ro("Spatial four-direction iterations, 1 to 5."));
    declare_parameter<int>        ("spatial_filter_holes_fill",  0,
        ro("Consecutive holes the spatial pass bridges per direction, 0 to 255; 0 disables bridging."));
    declare_parameter<bool>       ("temporal_filter",             false,
        rw("Enable the temporal filter. Switching it on resets the per-pixel history."));
    declare_parameter<double>     ("temporal_filter_alpha",       0.4,
        rw("Temporal blend weight, 0.0 to 1.0."));
    declare_parameter<int>        ("temporal_filter_delta",       20,
        rw("Temporal edge guard, 1 to 4095; disparity units after the spatial filter, mm otherwise."));
    declare_parameter<int>        ("temporal_filter_persistence", 3,
        rw("Validity-history pattern an invalid pixel must match to keep its last value, 0 to 8. 0 disables and 8 holds indefinitely; 1 is the strictest of the patterns and 7 the loosest."));
    declare_parameter<int>        ("hole_filling",                0,
        ro("Z-domain hole filling: 0 off, 1 fill_from_left, 2 farthest_from_around, 3 nearest_from_around."));
    declare_parameter<double>     ("diagnostics_rate_hz",   1.0,
        ro("/diagnostics publish rate in Hz; below 0.001 disables the publisher."));
    // The runtime CT/PU controls are declared after the open, defaulting to
    // the SDK's current values. A launch override is written back only when
    // explicitly supplied; otherwise the firmware keeps its configuration.

    declare_parameter<std::string>("base_frame",        "",
        ro("frame_id of the camera base link; empty derives it from camera_name."));
    declare_parameter<std::string>("left_color_frame",  "",
        ro("frame_id of the left colour sensor; empty derives it from camera_name."));
    declare_parameter<std::string>("right_color_frame", "",
        ro("frame_id of the right colour sensor; empty derives it from camera_name."));
    declare_parameter<std::string>("depth_frame",       "",
        ro("frame_id of the depth sensor; empty derives it from camera_name."));
    declare_parameter<std::string>("points_frame",      "",
        ro("frame_id stamped on the point cloud; empty derives it from camera_name."));

    declare_parameter<std::string>("dm_quality_cfg_dir", "",
        ro("Directory holding the DM_Quality register tables; empty uses the installed share directory."));

    // selfcal_profile's default must name a shipped profile.
    declare_parameter<bool>       ("selfcal_enable",     false,
        ro("Expose the self-calibration action and commit service."));
    declare_parameter<std::string>("selfcal_config_dir", "",
        ro("Directory holding the self-calibration profiles; empty uses the installed share directory."));
    declare_parameter<std::string>("selfcal_profile",    "maintenance_dock",
        rw("Self-calibration tuning profile name, resolved under the profile directory."));
}

bool CameraNode::load_video_mode(VideoMode& out, std::string& err) {
    std::string dir = get_parameter("video_modes_dir").as_string();
    if (dir.empty()) {
        try {
            dir = ament_index_cpp::get_package_share_directory("eys3d_camera") +
                  "/launch/video_modes";
        } catch (const std::exception& e) {
            err = std::string("cannot resolve package share dir: ") + e.what();
            return false;
        }
    }
    const std::string model = normalize_model(get_parameter("model").as_string());
    int mode_id             = get_parameter("mode_id").as_int();

    const auto modes = load_video_modes(dir, model);
    if (modes.empty()) {
        err = "no modes loaded from " + dir + "/" + model + ".yaml "
              "(accepted models: G100P, G100Pi, R77, G62)";
        return false;
    }
    RCLCPP_INFO(get_logger(), "%s", format_mode_table(model, modes).c_str());

    const auto info = load_model_info(dir, model);
    if (!info) {
        err = "model header missing or invalid in " + model + ".yaml";
        return false;
    }
    model_info_ = *info;

    // mode_id < 0 = auto: probe the negotiated USB link and pick the model's
    // signature default mode for it. An explicit mode_id is instead validated
    // against the link at open time.
    if (mode_id < 0) {
        if (model_info_.signature_mode.empty()) {
            err = "mode_id=auto: no signature_mode entry in " + model + ".yaml";
            return false;
        }
        DeviceConfig probe_cfg;
        probe_cfg.serial_number = get_parameter("dev_serial_number").as_string();
        probe_cfg.usb_port      = get_parameter("usb_port").as_string();
        probe_cfg.expected_pid  = model_info_.pid;
        const auto usb = device_->probe_usb_type(probe_cfg);

        // Fall back to the lowest-bandwidth signature (USB2 when declared):
        // it opens on either link, and open() validates the real one.
        auto it = model_info_.signature_mode.begin();
        if (usb) {
            const auto exact = model_info_.signature_mode.find(*usb);
            if (exact != model_info_.signature_mode.end()) {
                it = exact;
                RCLCPP_INFO(get_logger(),
                            "mode_id=auto -> %d (signature mode for the negotiated USB%d link)",
                            it->second, *usb);
            } else {
                RCLCPP_WARN(get_logger(),
                            "mode_id=auto: no signature_mode entry for a USB%d link in %s.yaml; "
                            "falling back to mode %d (USB%d signature)",
                            *usb, model.c_str(), it->second, it->first);
            }
        } else {
            RCLCPP_WARN(get_logger(),
                        "mode_id=auto: could not probe the USB link type (camera not attached?); "
                        "falling back to mode %d (USB%d signature)",
                        it->second, it->first);
        }
        mode_id = it->second;
    }

    const auto found = find_mode(modes, mode_id);
    if (!found) {
        err = "mode_id=" + std::to_string(mode_id) + " not found in " + model + ".yaml";
        return false;
    }

    out = *found;
    return true;
}

DeviceConfig CameraNode::build_device_config(const VideoMode& vm) const {
    DeviceConfig cfg;
    cfg.color_width      = vm.color_width;
    cfg.color_height     = vm.color_height;
    cfg.color_format     = vm.color_format;
    cfg.depth_width      = vm.depth_width;
    cfg.depth_height     = vm.depth_height;
    cfg.depth_data_type  = vm.depth_data_type;
    cfg.zd_index         = vm.zd_index;
    cfg.framerate        = vm.framerate;
    cfg.interleave       = vm.interleave;
    cfg.mode_usb         = vm.usb;
    cfg.depth_near_mm    = get_parameter("depth_near_mm").as_int();
    cfg.depth_far_mm     = get_parameter("depth_far_mm").as_int();
    // Z14 depth is a 14-bit distance in mm: the far plane cannot exceed
    // 2^14 - 1, and near must sit in front of far. A launch value past that
    // is a config error, caught here before the device opens.
    if (cfg.depth_far_mm > 16383)
        throw std::invalid_argument("depth_far_mm exceeds the 16383 mm Z14 limit");
    if (cfg.depth_near_mm > 16383)
        throw std::invalid_argument("depth_near_mm exceeds the 16383 mm Z14 limit");
    // Order near against the far plane actually applied; depth_far_mm may be
    // -1, meaning the per-model default.
    const int effective_far =
        cfg.depth_far_mm > 0 ? cfg.depth_far_mm : model_info_.depth_far_mm;
    if (cfg.depth_near_mm > 0 && effective_far > 0 &&
        cfg.depth_near_mm >= effective_far)
        throw std::invalid_argument("depth_near_mm must be < depth_far_mm");
    cfg.ir_value         = get_parameter("ir_value").as_int();
    cfg.colored_pointcloud     = get_parameter("colored_pointcloud").as_bool();
    cfg.spatial_filter_enabled   = get_parameter("spatial_filter").as_bool();
    cfg.spatial_filter_alpha     = get_parameter("spatial_filter_alpha").as_double();
    cfg.spatial_filter_delta     = get_parameter("spatial_filter_delta").as_int();
    cfg.spatial_filter_magnitude = get_parameter("spatial_filter_magnitude").as_int();
    cfg.spatial_filter_holes_fill = get_parameter("spatial_filter_holes_fill").as_int();
    cfg.temporal_filter_enabled     = get_parameter("temporal_filter").as_bool();
    cfg.temporal_filter_alpha       = get_parameter("temporal_filter_alpha").as_double();
    cfg.temporal_filter_delta       = get_parameter("temporal_filter_delta").as_int();
    cfg.temporal_filter_persistence = get_parameter("temporal_filter_persistence").as_int();
    cfg.hole_filling                = get_parameter("hole_filling").as_int();
    // Drives the wide-YUYV split decode in the device layer.
    cfg.split_color      = split_color_;
    cfg.serial_number    = get_parameter("dev_serial_number").as_string();
    cfg.usb_port         = get_parameter("usb_port").as_string();
    cfg.model            = normalize_model(get_parameter("model").as_string());
    // Per-model constants from the catalogue header.
    cfg.expected_pid     = model_info_.pid;
    cfg.mono             = model_info_.mono;
    cfg.ir_default       = model_info_.ir_default;
    cfg.default_near_mm  = model_info_.depth_near_mm;
    cfg.default_far_mm   = model_info_.depth_far_mm;
    cfg.selfcal_enable     = get_parameter("selfcal_enable").as_bool();
    cfg.selfcal_config_dir = get_parameter("selfcal_config_dir").as_string();
    // Empty selfcal_config_dir resolves to the in-package profiles, mirroring
    // how video_modes_dir falls back to the package share.
    if (cfg.selfcal_enable && cfg.selfcal_config_dir.empty()) {
        try {
            cfg.selfcal_config_dir =
                ament_index_cpp::get_package_share_directory("eys3d_camera") +
                "/config/selfcal";
        } catch (const std::exception&) {
            // Leave empty; start_selfcal() will report a missing profile.
        }
    }

    // The device layer would silently clamp these (a
    // spatial_filter_magnitude of 99 becomes 5), so they are range-checked
    // here; the constructor catches the throw and idles rather than running
    // a configuration the operator did not ask for. Bounds match the kernel
    // ceilings and on_set_parameters().
    auto check_int = [](const char* name, long v, long lo, long hi) {
        if (v < lo || v > hi)
            throw std::invalid_argument(
                std::string(name) + " must be in [" + std::to_string(lo) + ", " +
                std::to_string(hi) + "]");
    };
    auto check_unit = [](const char* name, double v) {
        if (v < 0.0 || v > 1.0)
            throw std::invalid_argument(std::string(name) + " must be in [0.0, 1.0]");
    };
    check_int ("spatial_filter_delta",        cfg.spatial_filter_delta,        1, 4095);
    check_int ("spatial_filter_magnitude",    cfg.spatial_filter_magnitude,    1, 5);
    check_int ("spatial_filter_holes_fill",   cfg.spatial_filter_holes_fill,   0, 255);
    check_int ("temporal_filter_delta",       cfg.temporal_filter_delta,       1, 4095);
    check_int ("temporal_filter_persistence", cfg.temporal_filter_persistence, 0, 8);
    check_int ("hole_filling",                cfg.hole_filling,                0, 3);
    check_unit("spatial_filter_alpha",        cfg.spatial_filter_alpha);
    check_unit("temporal_filter_alpha",       cfg.temporal_filter_alpha);
    return cfg;
}

rclcpp::Time CameraNode::stamp_from_hw_us(uint64_t hw_us) {
    if (hw_us == 0) return now();
    if (!time_anchor_set_.load(std::memory_order_acquire)) {
        const auto now_t = now();
        int64_t expected = 0;
        if (hw_anchor_us_.compare_exchange_strong(expected, static_cast<int64_t>(hw_us))) {
            ros_anchor_ns_.store(now_t.nanoseconds(), std::memory_order_release);
            time_anchor_set_.store(true, std::memory_order_release);
            return now_t;
        }
        // CAS lost: the other fetch thread is mid-init. Spin until it has
        // published ros_anchor_ns_ — otherwise reading it below could
        // return 0 and stamp this frame near the epoch, violating REP-117.
        while (!time_anchor_set_.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    const int64_t hw_anchor = hw_anchor_us_.load(std::memory_order_acquire);
    const int64_t ros_anchor = ros_anchor_ns_.load(std::memory_order_acquire);
    const int64_t delta_ns = (static_cast<int64_t>(hw_us) - hw_anchor) * 1000;
    return rclcpp::Time(ros_anchor + delta_ns, RCL_ROS_TIME);
}

EspdiDevice::Calibration CameraNode::snapshot_calib() const {
    std::lock_guard<std::mutex> lk(calib_mtx_);
    return cached_calib_;
}

sensor_msgs::msg::CameraInfo CameraNode::build_camera_info(
    const std::string& frame_id, const rclcpp::Time& stamp,
    int width, int height, const EspdiDevice::Calibration& calib,
    InfoStream stream) const {
    sensor_msgs::msg::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = frame_id;
    ci.width  = static_cast<uint32_t>(width);
    ci.height = static_cast<uint32_t>(height);
    // Left zeroed when the rectify log did not load: a client may read
    // k[0] == 0.0 as an uncalibrated camera.
    if (calib.valid) {
        const bool left_eye  = stream != InfoStream::kRightColor;
        const bool rectified = stream == InfoStream::kDepth || color_rectified_;
        const auto& lens = left_eye ? calib.left : calib.right;
        const double ps = EspdiDevice::raster_scale(calib.out_height, height);
        for (size_t i = 0; i < 12; ++i) ci.p[i] = lens.P[i];
        for (int i : {0, 2, 3, 5, 6, 7}) ci.p[i] *= ps;
        // Tx = -fx' * B, Ty = -fy' * B; the log's baseline is millimetres.
        ci.p[3] /= 1000.0;
        ci.p[7] /= 1000.0;

        if (rectified) {
            // Zeroed rather than emptied: image_geometry reads an empty d as
            // UNKNOWN and throws on rectifyImage().
            ci.k = {ci.p[0], ci.p[1], ci.p[2],
                    ci.p[4], ci.p[5], ci.p[6],
                    ci.p[8], ci.p[9], ci.p[10]};
            ci.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
            ci.distortion_model = "plumb_bob";
            ci.d.assign(5, 0.0);
        } else {
            // D acts on normalised coordinates and never scales. Eight are
            // stored; a five-coefficient lens leaves k4..k6 zero.
            const double ks = EspdiDevice::raster_scale(calib.in_height, height);
            for (size_t i = 0; i < 9; ++i) ci.k[i] = lens.K[i];
            for (int i : {0, 2, 4, 5}) ci.k[i] *= ks;
            for (size_t i = 0; i < 9; ++i) ci.r[i] = lens.R[i];
            const bool eight = std::any_of(lens.D.begin() + 5, lens.D.end(),
                                           [](double v) { return v != 0.0; });
            ci.distortion_model = eight ? "rational_polynomial" : "plumb_bob";
            ci.d.assign(lens.D.begin(), lens.D.begin() + (eight ? 8 : 5));
        }
        warn_if_off_raster(ci, stream);
    }
    return ci;
}

// A principal point far off centre means the intrinsics are at another
// raster. Both K and P: the raw form scales them by independent factors.
void CameraNode::warn_if_off_raster(const sensor_msgs::msg::CameraInfo& ci,
                                    InfoStream stream) const {
    constexpr double kMaxOffCentre = 0.25;
    const double w = static_cast<double>(ci.width);
    const double h = static_cast<double>(ci.height);
    if (w <= 0.0 || h <= 0.0) return;
    const auto off = [](double c, double extent) {
        return std::abs(c / extent - 0.5) > kMaxOffCentre;
    };
    const bool bad_k = off(ci.k[2], w) || off(ci.k[5], h);
    const bool bad_p = off(ci.p[2], w) || off(ci.p[6], h);
    if (!bad_k && !bad_p) return;
    auto& latch = off_raster_warned_[static_cast<size_t>(stream)];
    if (latch.exchange(true, std::memory_order_relaxed)) return;
    RCLCPP_WARN(get_logger(),
                "%s: principal point k (%.2f, %.2f) p (%.2f, %.2f) is not "
                "centred in the %ux%u image; the intrinsics do not describe "
                "this raster",
                ci.header.frame_id.c_str(), ci.k[2], ci.k[5],
                ci.p[2], ci.p[6], ci.width, ci.height);
}

void CameraNode::on_color(FrameBuffer&& f) {
    if (!pub_color_ || pub_color_->get_subscription_count() == 0) return;
    RCLCPP_INFO_ONCE(get_logger(),
                     "on_color: first frame (frame=%d, %dx%d, %zu bytes, rgb8)",
                     f.frame_number, f.width, f.height, f.data.size());
    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = left_color_optical_frame_;
    msg->height = static_cast<uint32_t>(f.height);
    msg->width  = static_cast<uint32_t>(f.width);
    msg->encoding = "rgb8";
    msg->step = static_cast<uint32_t>(f.width) * 3;
    msg->is_bigendian = 0;
    msg->data = std::move(f.data);
    pub_color_->publish(std::move(msg));

    const auto calib = snapshot_calib();
    pub_color_info_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(
        build_camera_info(left_color_optical_frame_, stamp, f.width,
                          f.height, calib, InfoStream::kLeftColor)));
}

void CameraNode::publish_split_color(FrameBuffer&& f) {
    // Wide-color modes pack L|R into one raster, in two shapes:
    //   YUYV   f.data and f.data_right already hold the two halves; both
    //          move straight into Image messages.
    //   MJPEG  f.data holds the wide raster and f.data_right is empty;
    //          the half-width buffers come from a row-by-row memcpy.
    if (!pub_color_) return;
    const bool publish_left  = pub_color_->get_subscription_count() > 0;
    const bool publish_right = pub_color_right_ &&
                               pub_color_right_->get_subscription_count() > 0;
    if (!publish_left && !publish_right) return;

    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);
    const bool pre_split = !f.data_right.empty();

    // In the pre-split path f.width is already the per-side width;
    // in the wide-intermediate path it is the full wide width and
    // must be halved here.
    const int half_w = pre_split ? f.width : (f.width / 2);
    if (!pre_split && (f.width % 2 != 0)) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "split_color requested but frame width %d is odd; falling back to L-only",
                         f.width);
        on_color(std::move(f));
        return;
    }
    constexpr int bytes_per_pixel = 3;
    const size_t out_step = static_cast<size_t>(half_w) * bytes_per_pixel;
    const size_t half_bytes = out_step * static_cast<size_t>(f.height);

    auto fill_header = [&](sensor_msgs::msg::Image& m, const std::string& frame) {
        m.header.stamp = stamp;
        m.header.frame_id = frame;
        m.height = static_cast<uint32_t>(f.height);
        m.width  = static_cast<uint32_t>(half_w);
        m.encoding = "rgb8";
        m.is_bigendian = 0;
        m.step = static_cast<uint32_t>(out_step);
    };

    std::unique_ptr<sensor_msgs::msg::Image> left;
    if (publish_left) {
        left = std::make_unique<sensor_msgs::msg::Image>();
        fill_header(*left, left_color_optical_frame_);
    }
    std::unique_ptr<sensor_msgs::msg::Image> right;
    if (publish_right) {
        right = std::make_unique<sensor_msgs::msg::Image>();
        fill_header(*right, right_color_optical_frame_);
    }

    if (pre_split) {
        // Buffers are already half-width; transfer ownership.
        if (left)  left->data  = std::move(f.data);
        if (right) right->data = std::move(f.data_right);
    } else {
        // Wide intermediate: slice row-by-row into half-width buffers.
        const size_t in_step = static_cast<size_t>(f.width) * bytes_per_pixel;
        if (left)  left->data.resize(half_bytes);
        if (right) right->data.resize(half_bytes);
        const uint8_t* src = f.data.data();
        uint8_t* lp = left  ? left->data.data()  : nullptr;
        uint8_t* rp = right ? right->data.data() : nullptr;
        if (lp && rp) {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(lp + r * out_step, src + r * in_step,            out_step);
                std::memcpy(rp + r * out_step, src + r * in_step + out_step, out_step);
            }
        } else if (lp) {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(lp + r * out_step, src + r * in_step, out_step);
            }
        } else {
            for (int r = 0; r < f.height; ++r) {
                std::memcpy(rp + r * out_step, src + r * in_step + out_step, out_step);
            }
        }
    }

    if (left)  pub_color_->publish(std::move(left));
    if (right) pub_color_right_->publish(std::move(right));

    const auto calib = snapshot_calib();
    if (publish_left) {
        pub_color_info_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(
            build_camera_info(left_color_optical_frame_, stamp, half_w,
                              f.height, calib, InfoStream::kLeftColor)));
    }
    if (publish_right && pub_color_right_info_) {
        pub_color_right_info_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(
            build_camera_info(right_color_optical_frame_, stamp, half_w,
                              f.height, calib, InfoStream::kRightColor)));
    }
}

void CameraNode::on_depth(FrameBuffer&& f) {
    if (!pub_depth_) return;
    RCLCPP_INFO_ONCE(get_logger(), "on_depth: first frame (frame=%d, %dx%d, %zu bytes)",
                     f.frame_number, f.width, f.height, f.data.size());

    if (!dm_quality_applied_.exchange(true) && !dm_quality_cfg_dir_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "First depth frame in; applying DM_Quality cfg from '%s'",
                    dm_quality_cfg_dir_.c_str());
        device_->apply_dm_quality_register_setting_async(dm_quality_cfg_dir_);
    }

    if (pub_depth_->get_subscription_count() == 0) return;

    const auto stamp = stamp_from_hw_us(f.hw_timestamp_us);

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = depth_optical_frame_;
    msg->height = static_cast<uint32_t>(f.height);
    msg->width  = static_cast<uint32_t>(f.width);
    msg->encoding = "16UC1";
    msg->is_bigendian = 0;
    msg->step = static_cast<uint32_t>(f.width) * 2;
    msg->data = std::move(f.data);
    pub_depth_->publish(std::move(msg));

    const auto calib = snapshot_calib();
    pub_depth_info_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(
        build_camera_info(depth_optical_frame_, stamp, f.width,
                          f.height, calib, InfoStream::kDepth)));
}

void CameraNode::on_point_cloud(std::vector<uint8_t>&& xyz_bytes,
                                uint32_t valid_points,
                                uint32_t point_step,
                                uint64_t hw_timestamp_us) {
    if (!pub_points_ || valid_points == 0 || xyz_bytes.empty()) return;
    const float* xyz = reinterpret_cast<const float*>(xyz_bytes.data());
    RCLCPP_INFO_ONCE(get_logger(),
                     "on_point_cloud: first cloud (%u valid points, sample xyz=[%.3f, %.3f, %.3f] m, point_step=%u)",
                     valid_points, xyz[0], xyz[1], xyz[2], point_step);
    // Static clock: the THROTTLE macro evaluates it on every expansion.
    static rclcpp::Clock pc_throttle_clock{RCL_STEADY_TIME};
    RCLCPP_DEBUG_THROTTLE(get_logger(), pc_throttle_clock, 10000,
                          "pointcloud: %u valid points, sample xyz=[%.3f, %.3f, %.3f] m",
                          valid_points, xyz[0], xyz[1], xyz[2]);

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = stamp_from_hw_us(hw_timestamp_us);
    msg->header.frame_id = points_frame_;
    msg->height = 1;
    msg->width  = valid_points;
    msg->is_dense    = false;
    msg->is_bigendian = false;

    auto set_field = [](sensor_msgs::msg::PointField& fld, const char* name,
                        uint32_t offset, uint8_t datatype) {
        fld.name = name;
        fld.offset = offset;
        fld.datatype = datatype;
        fld.count = 1;
    };
    const bool colored = (point_step == 16);
    msg->fields.resize(colored ? 4 : 3);
    set_field(msg->fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    set_field(msg->fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    set_field(msg->fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    if (colored) {
        // `rgb` as float32 with the bits 0x00RRGGBB is the PCL convention.
        set_field(msg->fields[3], "rgb", 12, sensor_msgs::msg::PointField::FLOAT32);
    }

    msg->point_step = point_step;
    msg->row_step   = msg->point_step * msg->width;

    // Take ownership of the buffer (PC thread sized it to valid_points * 12
    // before invoking the callback). No allocation or memcpy here.
    msg->data = std::move(xyz_bytes);

    pub_points_->publish(std::move(msg));
}

void CameraNode::declare_and_apply_runtime_params() {
    if (!device_) return;
    const auto state = device_->read_runtime_state();
    RCLCPP_INFO(get_logger(),
                "Camera CT/PU state on open: IR=%d (read_ok=%d), AE=%s (ok=%d), "
                "exposure_step=%d (ok=%d), AWB=%s (ok=%d), power_line=%d (ok=%d)",
                state.ir_value, state.ir_read_ok,
                state.auto_exposure ? "auto" : "manual", state.auto_exposure_read_ok,
                state.exposure_time_step, state.exposure_read_ok,
                state.auto_white_balance ? "auto" : "manual", state.awb_read_ok,
                state.power_line_frequency, state.plf_read_ok);

    // Seeded from the SDK's current state so `ros2 param get` reflects the
    // camera. ir_value is declared in declare_params() to reach open().
    declare_parameter<bool> ("auto_exposure",      state.auto_exposure,
        rw("Automatic exposure. Seeded from the camera; written back only when set."));
    declare_parameter<bool> ("auto_white_balance", state.auto_white_balance,
        rw("Automatic white balance. Seeded from the camera; written back only when set. Rejected on monochrome modules."));
    declare_parameter<int>  ("exposure_time_step",   state.exposure_time_step,
        rw("Manual exposure as a log2 register step, -13 to 3. Applies only while auto_exposure is false."));
    declare_parameter<int>  ("power_line_frequency", state.power_line_frequency,
        rw("Flicker rejection: 1 = 50 Hz, 2 = 60 Hz."));

    // Always apply ir_value at launch so the projector is lit without
    // an extra parameter call. -1 = mode-resolved default (see
    // EspdiDevice::open), 0 = off, positive integer = raw level.
    device_->set_ir_value(get_parameter("ir_value").as_int());

    // Other CT/PU values are written back only when explicitly overridden
    // at launch. The overrides map covers both NodeOptions overrides and
    // --ros-args / params-file values.
    const auto& overrides =
        get_node_parameters_interface()->get_parameter_overrides();
    auto was_overridden = [&](const std::string& name) {
        return overrides.find(name) != overrides.end();
    };
    if (was_overridden("auto_exposure"))
        device_->set_auto_exposure(get_parameter("auto_exposure").as_bool());
    if (!model_info_.mono && was_overridden("auto_white_balance"))
        device_->set_auto_white_balance(get_parameter("auto_white_balance").as_bool());
    if (was_overridden("exposure_time_step") && !get_parameter("auto_exposure").as_bool())
        (void)device_->set_exposure_time_step(get_parameter("exposure_time_step").as_int());
    if (was_overridden("power_line_frequency"))
        device_->set_power_line_frequency(get_parameter("power_line_frequency").as_int());
}

void CameraNode::resync_ct_pu_from_device() {
    if (!device_) return;
    // The firmware is the source of truth (see the declaration): read it and
    // update the store to match. Fields the firmware did not report are
    // skipped; exposure_time_step applies only in manual mode and
    // power_line_frequency must be a valid mains value.
    const auto s = device_->read_runtime_state();
    std::vector<rclcpp::Parameter> updates;
    // auto_exposure=false makes on_set_parameters re-apply exposure_time_step,
    // so hold it back until the exposure value reads back too.
    if (s.auto_exposure_read_ok && (s.auto_exposure || s.exposure_read_ok))
        updates.emplace_back("auto_exposure", s.auto_exposure);
    if (s.awb_read_ok && !model_info_.mono)
        updates.emplace_back("auto_white_balance", s.auto_white_balance);
    if (s.exposure_read_ok && !s.auto_exposure)
        updates.emplace_back("exposure_time_step", s.exposure_time_step);
    if (s.plf_read_ok && (s.power_line_frequency == 1 || s.power_line_frequency == 2))
        updates.emplace_back("power_line_frequency", s.power_line_frequency);
    if (!updates.empty())
        set_parameters(updates);
}

rcl_interfaces::msg::SetParametersResult CameraNode::on_set_parameters(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (!device_) {
        result.successful = false;
        result.reason = "device not open";
        return result;
    }
    // Validate first, apply only if all changes are individually accepted —
    // partial writes leave the FW in a confusing state.
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name == "ir_value") {
            // Range comes from the model catalogue, the same source the
            // driver opens the device with. Rejecting here keeps an
            // out-of-range write from being silently pinned to the FW
            // limit further down. Negative = re-resolve for the active mode.
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                (p.as_int() >= 0 &&
                 (p.as_int() < model_info_.ir_min || p.as_int() > model_info_.ir_max))) {
                result.successful = false;
                result.reason = "ir_value must be int in [" +
                                std::to_string(model_info_.ir_min) + ", " +
                                std::to_string(model_info_.ir_max) + "] for model " +
                                model_info_.model + ", or negative for the mode default";
                return result;
            }
        } else if (name == "auto_exposure" || name == "auto_white_balance") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = name + " must be bool";
                return result;
            }
            // Monochrome sensors have no colour path, so white balance is
            // meaningless — reject it instead of letting the FW error out.
            if (name == "auto_white_balance" && model_info_.mono) {
                result.successful = false;
                result.reason = model_info_.model +
                                " is a monochrome camera; it has no white balance";
                return result;
            }
        } else if (name == "exposure_time_step") {
            // Written straight to CT_EXPOSURE_TIME_ABSOLUTE as a signed log2
            // step; the hardware register only spans [-13, 3].
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < -13 || p.as_int() > 3) {
                result.successful = false;
                result.reason = "exposure_time_step must be int in [-13, 3] (signed log2 exposure register)";
                return result;
            }
        } else if (name == "power_line_frequency") {
            // Firmware only accepts 50/60 Hz; UVC 0 (off) and 3 (auto) are rejected.
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                (p.as_int() != 1 && p.as_int() != 2)) {
                result.successful = false;
                result.reason = "power_line_frequency must be 1 (50Hz) or 2 (60Hz)";
                return result;
            }
        } else if (name == "temporal_filter") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "temporal_filter must be bool";
                return result;
            }
        } else if (name == "temporal_filter_alpha") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                p.as_double() < 0.0 || p.as_double() > 1.0) {
                result.successful = false;
                result.reason = "temporal_filter_alpha must be double in [0.0, 1.0]";
                return result;
            }
        } else if (name == "temporal_filter_delta") {
            // Upper bound 4095 keeps the Q4 promotion (delta << 4) inside
            // a positive int32 and bounds it well above any realistic
            // disparity change. Larger values gain no filtering effect.
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < 1 || p.as_int() > 4095) {
                result.successful = false;
                result.reason = "temporal_filter_delta must be int in [1, 4095] (raw disparity units)";
                return result;
            }
        } else if (name == "temporal_filter_persistence") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < 0 || p.as_int() > 8) {
                result.successful = false;
                result.reason = "temporal_filter_persistence must be int in [0, 8]";
                return result;
            }
        }
        // Unknown parameters (including immutable settings such as model and
        // mode_id) are accepted unchanged; rclcpp permits arbitrary parameters
        // to be set even when the node does not consume them.
    }
    // Merge temporal-filter changes in this batch with the current values so
    // one API call carries the full {enabled, alpha, delta, persistence} tuple.
    // get_parameter() still returns the pre-update values here.
    bool   t_enabled     = get_parameter("temporal_filter").as_bool();
    double t_alpha       = get_parameter("temporal_filter_alpha").as_double();
    int    t_delta       = get_parameter("temporal_filter_delta").as_int();
    int    t_persistence = get_parameter("temporal_filter_persistence").as_int();
    bool   t_changed     = false;
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if      (name == "temporal_filter")             { t_enabled     = p.as_bool();   t_changed = true; }
        else if (name == "temporal_filter_alpha")       { t_alpha       = p.as_double(); t_changed = true; }
        else if (name == "temporal_filter_delta")       { t_delta       = p.as_int();    t_changed = true; }
        else if (name == "temporal_filter_persistence") { t_persistence = p.as_int();    t_changed = true; }
    }

    // A batch can have more than one failing write; append, do not overwrite.
    auto add_reason = [&result](const std::string& msg) {
        if (!result.reason.empty()) result.reason += "; ";
        result.reason += msg;
    };

    for (const auto& p : params) {
        const auto& name = p.get_name();
        bool ok = true;
        if (name == "ir_value") {
            ok = device_->set_ir_value(p.as_int());
        } else if (name == "auto_exposure") {
            ok = device_->set_auto_exposure(p.as_bool());
            // Re-apply exposure_time_step when AE goes off so the manual
            // value takes effect immediately; a firmware reject is
            // reported through result.reason.
            if (ok && !p.as_bool()) {
                // This callback runs before the store commits, so an
                // exposure_time_step in the same batch is not in the store yet.
                int step = get_parameter("exposure_time_step").as_int();
                for (const auto& q : params) {
                    if (q.get_name() == "exposure_time_step") { step = q.as_int(); break; }
                }
                if (!device_->set_exposure_time_step(step)) {
                    add_reason("exposure_time_step accepted into parameter store, "
                                    "but firmware rejected the immediate write; the "
                                    "value will be retried on the next auto_exposure transition");
                }
            }
        } else if (name == "exposure_time_step") {
            const bool ae_on = get_parameter("auto_exposure").as_bool();
            if (!ae_on) {
                // Some firmware rejects specific exposure values over the UVC CT
                // path. Report it in result.reason but keep successful=true, so
                // the store holds the value for the next auto_exposure transition.
                if (!device_->set_exposure_time_step(p.as_int())) {
                    add_reason("exposure_time_step accepted into parameter store, "
                                    "but firmware rejected the immediate write; the "
                                    "value will be retried on the next auto_exposure transition");
                }
            }
            // When AE is on the value is held for the next manual transition.
        } else if (name == "auto_white_balance") {
            ok = device_->set_auto_white_balance(p.as_bool());
        } else if (name == "power_line_frequency") {
            ok = device_->set_power_line_frequency(p.as_int());
        }
        if (!ok) {
            // A failure here is a firmware reject of an already-validated value.
            // successful stays true rather than discarding the batch -- earlier
            // writes in this loop already reached the chip. The value is retried
            // on the next reconnect or auto_exposure transition.
            add_reason("firmware rejected the immediate write of " + name +
                            "; the value is kept in the parameter store and retried later");
        }
    }
    if (t_changed) {
        // Temporal filter runs in whichever domain has a uint16
        // raster available; the device dispatches based on the
        // current spatial_filter state.
        (void)device_->set_temporal_filter(t_enabled, t_alpha,
                                           t_delta, t_persistence);
        // Mirror the runtime change into cached_cfg_ so a reconnect
        // re-opens with the current values. Held under cfg_mtx_ to
        // serialise with try_reconnect()'s read.
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cached_cfg_.temporal_filter_enabled     = t_enabled;
        cached_cfg_.temporal_filter_alpha       = t_alpha;
        cached_cfg_.temporal_filter_delta       = t_delta;
        cached_cfg_.temporal_filter_persistence = t_persistence;
    }
    return result;
}

// Refresh per-tick derived rates from the device stats counter. Called
// once per Updater round by diagnose_device() (always first in
// registration order) so the four remaining tasks reuse the same
// numbers and the per-tick math runs only once.
void CameraNode::refresh_diag_snapshot() {
    const auto stamp = now();
    const auto cur = device_->stats();
    const double dt = std::max(1e-3, (stamp - prev_stats_wall_).seconds());

    diag_snap_.cur = cur;
    diag_snap_.color_input_fps =
        static_cast<double>(cur.color_input_total   - prev_stats_.color_input_total)   / dt;
    diag_snap_.depth_input_fps =
        static_cast<double>(cur.depth_input_total   - prev_stats_.depth_input_total)   / dt;
    diag_snap_.color_publish_fps =
        static_cast<double>(cur.color_publish_total - prev_stats_.color_publish_total) / dt;
    diag_snap_.depth_publish_fps =
        static_cast<double>(cur.depth_publish_total - prev_stats_.depth_publish_total) / dt;
    diag_snap_.color_decode_delta_count =
        cur.color_publish_total - prev_stats_.color_publish_total;
    const uint64_t color_decode_delta_sum =
        cur.color_decode_sum_us - prev_stats_.color_decode_sum_us;
    diag_snap_.color_decode_avg_ms =
        diag_snap_.color_decode_delta_count > 0
            ? color_decode_delta_sum / 1000.0 / diag_snap_.color_decode_delta_count
            : 0.0;
    diag_snap_.pc_count_delta =
        cur.pc_publish_total - prev_stats_.pc_publish_total;
    const uint64_t pc_sum_delta =
        cur.pc_compute_sum_us - prev_stats_.pc_compute_sum_us;
    diag_snap_.pc_publish_fps =
        diag_snap_.pc_count_delta > 0
            ? static_cast<double>(diag_snap_.pc_count_delta) / dt
            : 0.0;
    diag_snap_.pc_compute_avg_ms =
        diag_snap_.pc_count_delta > 0
            ? pc_sum_delta / 1000.0 / diag_snap_.pc_count_delta
            : 0.0;

    prev_stats_ = cur;
    prev_stats_wall_ = stamp;
}

void CameraNode::diagnose_device(diagnostic_updater::DiagnosticStatusWrapper& s) {
    // Always first in registration order — refresh the shared snapshot
    // here so the other tasks in this round reuse the same numbers.
    refresh_diag_snapshot();

    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    using SS = EspdiDevice::StreamState;
    const ConnState state = conn_state_.load();
    const auto stream_state = device_->stream_state();
    const int actual_fps = device_->actual_fps();
    const int half_expected = std::max(1, actual_fps / 2);

    if (state == ConnState::kDisconnected) {
        s.summary(DS::ERROR, "camera disconnected; Linux device node not present");
    } else if (stream_state == SS::Standby) {
        s.summary(DS::OK, "standby (USB pipe closed by operator)");
    } else if (stream_state == SS::Paused) {
        s.summary(DS::OK, "streaming (paused — publish gated by operator)");
    } else {
        // Connected and Active but every configured stream below half its
        // expected rate -> ERROR; the per-stream tasks already carry their own
        // WARN. color_input_fps == 0 is normal in D-only modes, so colour
        // counts as dead only once it has been seen running.
        const bool color_configured = diag_snap_.cur.color_input_total > 0
            || diag_snap_.color_input_fps > 0.0;
        const bool color_dead = color_configured
            && diag_snap_.color_input_fps < 0.5 * half_expected;
        const bool depth_dead =
            diag_snap_.depth_input_fps < 0.5 * half_expected;
        if (color_dead && depth_dead) {
            s.summary(DS::ERROR, "no frames flowing on any configured stream");
        } else {
            s.summary(DS::OK, "streaming");
        }
    }

    s.add("connection_state",   state == ConnState::kStreaming ? "streaming" : "disconnected");
    s.add("device_present",     state == ConnState::kStreaming ? "true" : "false");
    s.add("reconnect_attempts", reconnect_attempts_.load(std::memory_order_relaxed));
    s.add("usb_port",           device_->usb_port().empty() ? "n/a" : device_->usb_port());
    s.add("serial_number",      device_->serial_number().empty() ? "n/a" : device_->serial_number());
    s.add("actual_fps",         device_->actual_fps());
    s.add("stream_state",
          (stream_state == SS::Active)  ? "Active"
        : (stream_state == SS::Paused)  ? "Paused"
                                        : "Standby");
}

// Per-stream liveness shared by color and depth tasks. Returns the level
// + message for the input rate against the expected per-stream rate
// (half of actual_fps in interleave modes where both streams share one
// USB endpoint). `allow_zero` covers the color side in D-only modes
// where no color stream is configured.
namespace {
struct StreamHealth { unsigned char level; const char* message; };
StreamHealth classify_stream(double input_fps, int actual_fps,
                             EspdiDevice::StreamState stream_state,
                             bool allow_zero) {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    using SS = EspdiDevice::StreamState;
    if (stream_state == SS::Standby) return {DS::OK, "standby"};
    const int per_stream_expected = std::max(1, actual_fps / 2);
    if (allow_zero && input_fps == 0.0) {
        return {DS::OK, "not configured (D-only mode)"};
    }
    if (input_fps < 0.5 * per_stream_expected) {
        return {DS::WARN, "input rate below 50% of expected"};
    }
    return {DS::OK, "streaming"};
}
}  // namespace

void CameraNode::diagnose_color(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const auto h = classify_stream(diag_snap_.color_input_fps,
                                   device_->actual_fps(),
                                   device_->stream_state(),
                                   /*allow_zero=*/true);
    s.summary(h.level, h.message);

    s.addf("input_fps",     "%.2f", diag_snap_.color_input_fps);
    s.addf("publish_fps",   "%.2f", diag_snap_.color_publish_fps);
    s.add ("input_total",   diag_snap_.cur.color_input_total);
    s.add ("input_dropped", diag_snap_.cur.color_input_dropped);
    s.add ("publish_total", diag_snap_.cur.color_publish_total);
    // Decode timings only when frames decoded in the window (i.e. a
    // subscriber is present).
    if (diag_snap_.color_decode_delta_count > 0) {
        s.addf("decode_avg_ms", "%.2f", diag_snap_.color_decode_avg_ms);
        s.addf("decode_max_ms", "%.2f", diag_snap_.cur.color_decode_max_us / 1000.0);
    }
}

void CameraNode::diagnose_depth(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const auto h = classify_stream(diag_snap_.depth_input_fps,
                                   device_->actual_fps(),
                                   device_->stream_state(),
                                   /*allow_zero=*/false);
    s.summary(h.level, h.message);

    s.addf("input_fps",     "%.2f", diag_snap_.depth_input_fps);
    s.addf("publish_fps",   "%.2f", diag_snap_.depth_publish_fps);
    s.add ("input_total",   diag_snap_.cur.depth_input_total);
    s.add ("input_dropped", diag_snap_.cur.depth_input_dropped);
    s.add ("publish_total", diag_snap_.cur.depth_publish_total);
}

void CameraNode::diagnose_pc(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — see device task)");
        return;
    }
    const char* status_str;
    if (diag_snap_.pc_count_delta > 0) {
        status_str = "active";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", diag_snap_.pc_publish_fps);
        s.addf("compute_avg_ms", "%.2f", diag_snap_.pc_compute_avg_ms);
        s.addf("compute_max_ms", "%.2f", diag_snap_.cur.pc_compute_max_us / 1000.0);
        s.add ("publish_total",  diag_snap_.cur.pc_publish_total);
    } else if (diag_snap_.cur.pc_publish_total > 0) {
        status_str = "idle (no /depth/points subscriber)";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", 0.0);
        s.addf("compute_max_ms", "%.2f", diag_snap_.cur.pc_compute_max_us / 1000.0);
        s.add ("publish_total",  diag_snap_.cur.pc_publish_total);
    } else {
        status_str = "idle (never run ; no subscriber since start)";
        s.summary(DS::OK, status_str);
        s.addf("publish_fps",    "%.2f", 0.0);
    }
    s.add("compute_status",        status_str);
    s.add("spatial_filter_total",  diag_snap_.cur.spatial_filter_total);
    s.add("temporal_filter_total", diag_snap_.cur.temporal_filter_total);
    s.add("hole_fill_total",       diag_snap_.cur.hole_fill_total);
}

void CameraNode::diagnose_thermal(diagnostic_updater::DiagnosticStatusWrapper& s) {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (conn_state_.load() == ConnState::kDisconnected) {
        s.summary(DS::OK, "(disconnected — SDK handle released)");
        return;
    }
    // Pause the sensor read during a self-cal session: selfk's worker writes cy
    // on the shared handle without sdk_mtx, so a concurrent control read could
    // interleave two UVC transfers on one handle.
    if (device_->selfcal_active()) {
        s.summary(DS::OK, "paused (self-calibration in progress)");
        return;
    }
    // APC_GetTemperature is a USB roundtrip; the Updater calls this at
    // diagnostics_rate_hz so the cost is one read per tick.
    const auto thermal = device_->read_temperature();
    if (!thermal.supported) {
        s.summary(DS::OK, "not supported on this model");
    } else if (!thermal.read_ok) {
        s.summary(DS::WARN, "read failed");
    } else {
        s.summary(DS::OK, "ok");
        s.addf("temperature_c", "%.2f", thermal.celsius);
    }
}

rclcpp_action::GoalResponse CameraNode::selfcal_handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const SelfCalAction::Goal>) {
    if (!device_->selfcal_available()) {
        RCLCPP_WARN(get_logger(), "selfcal/run rejected: not available");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (device_->selfcal_active() || selfcal_run_goal_) {
        RCLCPP_WARN(get_logger(),
                    "selfcal/run rejected: a session is already running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CameraNode::selfcal_handle_cancel(
    std::shared_ptr<SelfCalGoalHandle>) {
    // A session runs to completion and cannot be interrupted; if the result is
    // worse it auto-reverts, so there is nothing a cancel needs to undo.
    RCLCPP_WARN(get_logger(),
                "selfcal/run: cancel rejected — a session cannot be interrupted");
    return rclcpp_action::CancelResponse::REJECT;
}

void CameraNode::selfcal_handle_accepted(std::shared_ptr<SelfCalGoalHandle> gh) {
    const auto goal = gh->get_goal();
    const std::string profile = goal->profile.empty()
        ? get_parameter("selfcal_profile").as_string() : goal->profile;
    selfcal_run_auto_commit_shift_px_ = goal->auto_commit_shift_px;

    if (!device_->start_selfcal(profile)) {
        auto res = std::make_shared<SelfCalAction::Result>();
        res->outcome = "FAILED";
        res->message = "could not start self-calibration (see node log)";
        gh->abort(res);
        return;
    }
    selfcal_run_goal_ = gh;
    // The one-shot profile runs ~20-30 s, then the A/B re-check adds a few more;
    // give a generous deadline before declaring a timeout and reverting.
    selfcal_run_deadline_ = now() + rclcpp::Duration(60, 0);
    selfcal_run_timer_ = create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&CameraNode::selfcal_run_tick, this));
    RCLCPP_INFO(get_logger(), "selfcal/run started (profile='%s')", profile.c_str());
}

void CameraNode::selfcal_run_tick() {
    if (!selfcal_run_goal_) {
        if (selfcal_run_timer_) selfcal_run_timer_->cancel();
        return;
    }
    // A disconnect mid-run is recovered by the watchdog's reopen (which reloads
    // flash); fail the goal promptly instead of polling a dead session.
    if (conn_state_.load() != ConnState::kStreaming) {
        auto res = std::make_shared<SelfCalAction::Result>();
        res->outcome = "FAILED";
        res->message = "camera disconnected during calibration; aborted";
        selfcal_run_goal_->abort(res);
        selfcal_run_goal_.reset();
        if (selfcal_run_timer_) {
            selfcal_run_timer_->cancel();
            selfcal_run_timer_.reset();
        }
        return;
    }
    const auto st = device_->selfcal_status();

    const bool terminal = st.state == "COMPLETED" || st.state == "STOPPED" ||
                          st.state == "ERROR";
    // After the SDK converges, the worker runs the A/B re-check before the
    // session is resolvable; surface that as a distinct feedback phase.
    const bool rechecking = terminal && !st.resolve_ready;

    auto fb = std::make_shared<SelfCalAction::Feedback>();
    fb->phase = rechecking ? "RECHECK" : st.phase;
    fb->progress = st.progress;
    fb->processed_frames = static_cast<uint32_t>(st.processed_frames);
    fb->valid_ratio_latest = st.valid_ratio_latest;
    selfcal_run_goal_->publish_feedback(fb);

    const bool timed_out = now() >= selfcal_run_deadline_;
    // Wait for the A/B re-check to finish (resolve_ready) before acting, unless
    // the deadline has passed.
    if (!(terminal && st.resolve_ready) && !timed_out) {
        return;
    }

    auto res = std::make_shared<SelfCalAction::Result>();
    // The driver's own wall-clock deadline can fire before the SDK ever
    // produces a result, leaving st.outcome empty; report that case as
    // TIMEOUT rather than shipping an empty outcome string.
    res->outcome = timed_out ? "TIMEOUT" : st.outcome;
    res->valid_ratio_first = st.valid_ratio_first;
    res->valid_ratio_latest = st.valid_ratio_latest;
    res->valid_ratio_delta = st.valid_ratio_delta;
    res->correction_level = st.correction_level;
    res->cy_shift_px = st.cy_shift_px;
    res->recheck_verdict = st.ab_verdict;
    res->recheck_ratio_before = st.ab_ratio_initial;
    res->recheck_ratio_after = st.ab_ratio_final;

    // Resolve on the SDK outcome plus the A/B re-check. Anything that is not
    // SUCCESS or NO_CHANGE (including a hard ERROR / empty outcome) is a failure
    // and reverts; only a SUCCESS reaches the keep/commit paths below.
    const bool failed = timed_out ||
                        (st.outcome != "SUCCESS" && st.outcome != "NO_CHANGE");

    if (failed) {
        // Roll the registers back to the pre-session calibration.
        res->reverted = device_->revert_selfcal();
        const char* why = timed_out ? "timed out" : "calibration failed";
        res->message = res->reverted
            ? std::string(why) + "; reverted to the previous calibration"
            : std::string(why) + "; ROLLBACK ALSO FAILED — the camera is on the "
              "rejected alignment until it is power-cycled";
    } else if (st.outcome == "NO_CHANGE") {
        // Nothing changed; restore the pre-session cy (a safe no-op). Not
        // applied, reverted, or committed.
        device_->revert_selfcal();
        res->message = "calibration already optimal; no change made";
    } else if (st.ab_verdict == "worse") {
        // SUCCESS moved cy, but the live A/B re-check shows the new alignment is
        // actually worse on this scene. Roll it back.
        res->reverted = device_->revert_selfcal();
        res->message = res->reverted
            ? "the re-check showed the new alignment was worse; reverted"
            : "the re-check showed the new alignment was worse, but the rollback "
              "failed — the camera is on the rejected alignment until it is "
              "power-cycled";
    } else if (st.ab_verdict == "inconclusive") {
        // Could not certify an improvement (scene too unstable during the
        // re-check, or the difference was within noise). Be conservative.
        res->reverted = device_->revert_selfcal();
        res->message = res->reverted
            ? "could not verify an improvement (unstable scene during the "
              "re-check); reverted — hold the camera still and re-run"
            : "could not verify an improvement, and the rollback failed — the "
              "camera is on the rejected alignment until it is power-cycled";
    } else if (selfcal_run_auto_commit_shift_px_ >= 0.0f && st.can_commit &&
               st.cy_shift_px >= selfcal_run_auto_commit_shift_px_) {
        // Verified better (or re-check skipped) and the cy shift is large enough
        // to persist automatically.
        if (device_->commit_selfcal()) {
            res->applied = true;
            res->committed = true;
            res->message = "verified improved and committed to flash";
        } else {
            device_->keep_selfcal();
            res->applied = true;
            res->message = "verified improved and kept live; flash commit failed (see log)";
        }
    } else {
        // Verified better (or re-check skipped), kept live for review.
        device_->keep_selfcal();
        res->applied = true;
        res->message = "verified improved and kept live; call selfcal/commit to persist";
    }

    RCLCPP_INFO(get_logger(),
                "selfcal/run resolved: outcome=%s recheck=%s(%.3f->%.3f) "
                "reverted=%d committed=%d",
                res->outcome.c_str(), res->recheck_verdict.c_str(),
                res->recheck_ratio_before, res->recheck_ratio_after,
                res->reverted ? 1 : 0, res->committed ? 1 : 0);

    // Abort when the session did not reach a usable answer, or when a rollback
    // it intended did not take; a deliberate revert is the designed outcome and
    // succeeds. res->outcome carries the detail either way.
    const bool meant_to_revert =
        (st.ab_verdict == "worse" || st.ab_verdict == "inconclusive");
    if (failed || (meant_to_revert && !res->reverted)) {
        selfcal_run_goal_->abort(res);
    } else {
        selfcal_run_goal_->succeed(res);
    }
    selfcal_run_goal_.reset();
    if (selfcal_run_timer_) {
        selfcal_run_timer_->cancel();
        selfcal_run_timer_.reset();
    }
}

void CameraNode::publish_static_tf() {
    // Two-layer TF tree. Position links sit in ROS-base orientation
    // so robot-frame math does not need to unwind an optical rotation;
    // optical leaves carry the (-π/2, 0, -π/2) RPY that image_geometry,
    // depth_image_proc, and the RViz Image display expect.
    bool calib_valid = false;
    double baseline_mm = 0.0;
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        calib_valid = cached_calib_.valid;
        baseline_mm = cached_calib_.baseline_mm;
    }
    if (!calib_valid) {
        RCLCPP_WARN(get_logger(),
                    "publish_static_tf: calibration not valid; TF tree will use baseline=0");
    }
    const double half_baseline_m = calib_valid ? baseline_mm * 0.5e-3 : 0.0;

    tf2::Quaternion q_opt;
    q_opt.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

    const auto stamp = now();
    auto make_pos_tf = [&](const std::string& parent, const std::string& child, double y_m) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = y_m;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;
        return t;
    };
    auto make_opt_tf = [&](const std::string& parent, const std::string& child) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.rotation.x = q_opt.x();
        t.transform.rotation.y = q_opt.y();
        t.transform.rotation.z = q_opt.z();
        t.transform.rotation.w = q_opt.w();
        return t;
    };

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(7);
    // Layer 1 — base → position-only links (ROS-base orientation).
    tfs.push_back(make_pos_tf(base_frame_, left_color_frame_,  +half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, right_color_frame_, -half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, depth_frame_,       +half_baseline_m));
    tfs.push_back(make_pos_tf(base_frame_, points_frame_,      +half_baseline_m));
    // Layer 2 — position links → optical leaves (rotation only).
    tfs.push_back(make_opt_tf(left_color_frame_,  left_color_optical_frame_));
    tfs.push_back(make_opt_tf(right_color_frame_, right_color_optical_frame_));
    tfs.push_back(make_opt_tf(depth_frame_,       depth_optical_frame_));

    // /tf_static is TRANSIENT_LOCAL so any subscriber that joins later
    // receives the cached value immediately — no /tf re-stamping needed.
    tf_static_->sendTransform(tfs);
    RCLCPP_INFO(get_logger(),
                "Published static TF (%zu links) under %s, baseline=%.2f mm",
                tfs.size(), base_frame_.c_str(), baseline_mm);
}

}  // namespace eys3d_camera

RCLCPP_COMPONENTS_REGISTER_NODE(eys3d_camera::CameraNode)
