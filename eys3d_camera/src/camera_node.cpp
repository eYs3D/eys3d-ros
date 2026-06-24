#include "eys3d_camera/camera_node.hpp"

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

// rclcpp::init() on Foxy installs a SIGINT handler only. Re-raise
// SIGINT from SIGTERM so container runtimes (docker stop, systemctl
// stop, kubectl delete pod) reach the same rclcpp deferred-shutdown
// path. Installed once per process so both the standalone executable
// and the ComposableNodeContainer path get covered.
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

    // Static TF goes on /tf_static (TRANSIENT_LOCAL) regardless of
    // the node-level intra-process setting. Some rclcpp versions
    // reject TRANSIENT_LOCAL publishers in nodes with intra-process
    // comms enabled; disabling IPC on this single publisher keeps
    // /tf_static portable across distros. Other publishers on this
    // node still follow the node-level IPC setting.
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

    // Topic names are unprefixed — the launch sets the ROS namespace to
    // camera_name, so the fully-qualified form is /<camera_name>/<leaf>.
    // Prefixing here would produce /<camera_name>/<camera_name>/<leaf>.
    const std::string color_topic       = "left_color";
    const std::string color_right_topic = "right_color";
    const std::string depth_topic       = "depth_image";
    const std::string points_topic      = "pointcloud";

    // Image publishers are gated by the active video mode (no point producing a
    // depth topic in a color-only mode). camera_info publishers follow the
    // same gating so /right_color/camera_info does not appear without a
    // matching right_color image.
    pub_color_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        color_topic + "/camera_info", info_qos());
    pub_depth_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        depth_topic + "/camera_info", info_qos());

    if (vm.has_color) {
        // espdi_device decodes both YUYV and MJPEG inputs to rgb8 inline,
        // so the publish path emits a uniform `sensor_msgs/Image` with
        // `encoding=rgb8` regardless of the source wire format. Wide L|R
        // splitting just slices the rgb8 raster row-by-row.
        pub_color_ = create_publisher<sensor_msgs::msg::Image>(color_topic, image_qos());
        if (split_color_) {
            pub_color_right_ = create_publisher<sensor_msgs::msg::Image>(color_right_topic, image_qos());
            pub_color_right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                color_right_topic + "/camera_info", info_qos());
        }
    }
    if (vm.has_depth) {
        pub_depth_  = create_publisher<sensor_msgs::msg::Image>(depth_topic, image_qos());
        pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic, image_qos());
    }

    {
        // Initial population. No reader can race here — the parameter
        // callback is installed below this point.
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        cached_cfg_ = build_device_config(vm);
    }
    if (!device_->open(cached_cfg_)) {
        RCLCPP_ERROR(get_logger(),
                     "EspdiDevice::open() failed. Check that no other process holds the device "
                     "(lsof /dev/video*), the model PID matches the connected camera, and the "
                     "USB cable supports the required bandwidth. Node will idle until restarted.");
        return;
    }
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        cached_calib_ = device_->calibration();
    }

    publish_static_tf();

    // Apply initial CT/PU values (ir_intensity, AE, AWB, exposure_time_step)
    // before registering the parameter-set callback to avoid feedback from
    // the initial declares.
    declare_and_apply_runtime_params();
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& p){ return on_set_parameters(p); });

    // Runtime stream control — two distinct cost / latency tiers.
    //
    //   /<cam>/pause   (SetBool: true=pause, false=resume)
    //     Cheap state flip. SDK + USB keep streaming so the next frame
    //     after resume is published immediately (~33 ms @ 30 fps), but
    //     decode + filter + publish are all skipped while paused so the
    //     driver's CPU load drops to roughly the cost of the USB drain.
    //
    //   /<cam>/standby (SetBool: true=standby, false=resume)
    //     Heavy. APC_CloseDevice releases the V4L2 fd so USB traffic
    //     stops entirely. The SDK handle, calibration, and ZD table are
    //     kept, so resume reopens in ~200-400 ms without redoing the
    //     full open() sequence. The watchdog reconnect loop is suppressed
    //     while the operator has explicitly requested Standby — any
    //     real USB disconnect during Standby is detected on the next
    //     standby(false) attempt.
    //
    // Both controls toggle the colour + depth pair together — per-stream
    // toggling was removed because interleave modes require both halves
    // of the V4L2 stream to be active simultaneously and the disabled-
    // single-stream code path silently stalled on those.
    using SetBool = std_srvs::srv::SetBool;
    srv_pause_ = create_service<SetBool>(
        "pause",
        [this](const std::shared_ptr<SetBool::Request> req,
               std::shared_ptr<SetBool::Response> res) {
            const bool ok = device_->pause(req->data);
            res->success = ok;
            res->message = std::string("stream ") + (req->data ? "paused" : "resumed");
        });
    srv_standby_ = create_service<SetBool>(
        "standby",
        [this](const std::shared_ptr<SetBool::Request> req,
               std::shared_ptr<SetBool::Response> res) {
            // Publish the operator's intent to the watchdog before
            // closing or reopening the device, so a tick that races
            // with the transition does not read the zero-frame
            // interval as a disconnect.
            user_wants_standby_.store(req->data, std::memory_order_release);
            const bool ok = device_->standby(req->data);
            res->success = ok;
            if (ok) {
                if (!req->data) {
                    // Re-apply the operator's current ir_intensity (the
                    // SDK reopen restores cfg.ir_intensity which is the
                    // launch-time value; runtime overrides written via
                    // /parameters live only in the FW register that
                    // APC_CloseDevice cleared).
                    device_->set_ir_intensity(
                        get_parameter("ir_intensity").as_int());
                    // Force the next depth frame to re-apply DM_Quality —
                    // V4L2 close keeps USB enumerated so today's firmware
                    // preserves the register block across standby, but
                    // mirror try_reconnect()'s defensive reset so a
                    // future FW change that power-gates the sensor on
                    // STREAMOFF cannot silently regress depth quality.
                    dm_quality_applied_.store(false);
                    // Re-anchor the timestamp pipeline: APC_OpenDevice2
                    // re-negotiates IMAGE_SN_SYNC, and any firmware
                    // variant that resets the hw timestamp counter on
                    // STREAMOFF/STREAMON would otherwise stamp the first
                    // post-resume frame in the past — violating REP-117
                    // and freezing tf2 buffers. The reset re-establishes
                    // the hw_us → rclcpp::Time mapping on the next
                    // frame, mirroring try_reconnect(). Both halves of
                    // the anchor (hw and ros) must be zeroed too;
                    // stamp_from_hw_us uses CAS-from-zero to elect the
                    // initialising thread, and a stale non-zero
                    // hw_anchor_us_ would make every CAS fail and the
                    // anchor never get re-established.
                    hw_anchor_us_.store(0, std::memory_order_relaxed);
                    ros_anchor_ns_.store(0, std::memory_order_relaxed);
                    time_anchor_set_.store(false, std::memory_order_release);
                    // Refresh the watchdog baseline AND re-arm the
                    // startup grace so the first tick after standby(false)
                    // does not fall through to the zero-frame check before
                    // the SDK has had a chance to deliver the first
                    // post-resume frame (~500 ms typical).
                    watchdog_prev_stats_ = device_->stats();
                    zero_frame_seconds_.store(0, std::memory_order_relaxed);
                    startup_grace_seconds_.store(0, std::memory_order_relaxed);
                    watchdog_armed_.store(false, std::memory_order_relaxed);
                }
                res->message = std::string("standby ") + (req->data ? "entered" : "exited");
            } else {
                // Reopen failed — clear the operator intent so the
                // watchdog can try to recover the device.
                user_wants_standby_.store(false, std::memory_order_release);
                res->message = "standby(false) reopen failed ; device left closed";
            }
        });
    // Diagnostics — diagnostic_updater::Updater publishes on /diagnostics.
    // Five tasks (device / color / depth / pointcloud / thermal) contribute
    // one DiagnosticStatus each per array; the standard ROS diagnostic
    // stack aggregates max(level) across them. Set diagnostics_rate_hz
    // below 0.001 to disable the Updater entirely.
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
        // on and a /pointcloud subscriber exists, so the cloud paints
        // from a current color frame.
        const bool pc_needs_color = cached_cfg_.colored_pointcloud
            && pub_points_
            && pub_points_->get_subscription_count() > 0;
        return left_subs || right_subs || pc_needs_color;
    });
    device_->set_depth_gate([this]() {
        return pub_depth_ && pub_depth_->get_subscription_count() > 0;
    });
    device_->start(ColorFrameCb(cached_color_cb_),
                   DepthFrameCb(cached_depth_cb_),
                   PointCloudCb(cached_pc_cb_));

    // 1 Hz watchdog: detect USB disconnect and drive the reconnect loop.
    watchdog_prev_stats_ = device_->stats();
    watchdog_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]{ watchdog_tick(); });

    const char* color_topic_label =
        split_color_ ? "left_color + right_color" : "left_color";
    RCLCPP_INFO(get_logger(),
                "eys3d_camera '%s' running. Topics under '/%s/' (%s, depth_image, pointcloud).",
                camera_name_.c_str(), camera_name_.c_str(), color_topic_label);
}

void CameraNode::watchdog_tick() {
    if (!device_) return;

    // Operator-requested Standby suppresses the watchdog entirely. With
    // no fetch threads running the stats counters never advance, so the
    // standard zero-frame check would otherwise misread the intentional
    // pause as a disconnect and tear the SDK down. Skip the tick, refresh
    // the stats baseline so the next non-standby tick starts from the
    // current counts, and bail.
    if (user_wants_standby_.load(std::memory_order_acquire)) {
        watchdog_prev_stats_ = device_->stats();
        zero_frame_seconds_.store(0, std::memory_order_relaxed);
        return;
    }

    if (conn_state_.load() == ConnState::kStreaming) {
        const auto cur = device_->stats();
        const uint64_t f_color = cur.color_input_total - watchdog_prev_stats_.color_input_total;
        const uint64_t f_depth = cur.depth_input_total - watchdog_prev_stats_.depth_input_total;
        watchdog_prev_stats_ = cur;
        // Slow modes (e.g. R77 mode 1 at 7 fps over USB 2.0) can take several
        // seconds to deliver the first frame after open(). Hold the
        // disconnect detector until streaming is proven to have started.
        if (!watchdog_armed_.load(std::memory_order_relaxed)) {
            if (cur.color_input_total > 0 || cur.depth_input_total > 0) {
                watchdog_armed_.store(true, std::memory_order_relaxed);
                zero_frame_seconds_.store(0, std::memory_order_relaxed);
            } else {
                const int grace =
                    startup_grace_seconds_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (grace >= 10) {
                    // Final defensive check: a standby() that landed
                    // between the top-of-tick guard and here would also
                    // produce zero frames. Bail rather than tear the
                    // SDK down.
                    if (user_wants_standby_.load(std::memory_order_acquire)) return;
                    RCLCPP_ERROR(get_logger(),
                                 "watchdog: no frames within %d s of open; "
                                 "declaring camera disconnected",
                                 grace);
                    device_->stop();
                    device_->close();
                    conn_state_.store(ConnState::kDisconnected);
                    startup_grace_seconds_.store(0, std::memory_order_relaxed);
                    reconnect_poll_counter_ = 0;
                }
            }
            return;
        }
        if (f_color == 0 && f_depth == 0) {
            const int silent =
                zero_frame_seconds_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (silent >= 3) {
                if (user_wants_standby_.load(std::memory_order_acquire)) return;
                RCLCPP_ERROR(get_logger(),
                             "watchdog: no frames for %d s; declaring camera disconnected",
                             silent);
                device_->stop();
                device_->close();
                conn_state_.store(ConnState::kDisconnected);
                zero_frame_seconds_.store(0, std::memory_order_relaxed);
                watchdog_armed_.store(false, std::memory_order_relaxed);
                startup_grace_seconds_.store(0, std::memory_order_relaxed);
                reconnect_poll_counter_ = 0;
            }
        } else {
            zero_frame_seconds_.store(0, std::memory_order_relaxed);
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
        watchdog_armed_.store(false, std::memory_order_relaxed);
        startup_grace_seconds_.store(0, std::memory_order_relaxed);
        zero_frame_seconds_.store(0, std::memory_order_relaxed);
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
        // on and a /pointcloud subscriber exists, so the cloud paints
        // from a current color frame.
        const bool pc_needs_color = cached_cfg_.colored_pointcloud
            && pub_points_
            && pub_points_->get_subscription_count() > 0;
        return left_subs || right_subs || pc_needs_color;
    });
    device_->set_depth_gate([this]() {
        return pub_depth_ && pub_depth_->get_subscription_count() > 0;
    });
    device_->start(ColorFrameCb(cached_color_cb_),
                   DepthFrameCb(cached_depth_cb_),
                   PointCloudCb(cached_pc_cb_));
    // Re-apply IR — projector boots OFF after every open().
    device_->set_ir_intensity(get_parameter("ir_intensity").as_int());
    // The firmware power-cycles register state on USB re-enumeration, so
    // the next depth frame must re-apply the DM_Quality cfg.
    dm_quality_applied_.store(false);
    // The firmware hardware-timestamp counter restarts on USB re-enumeration.
    // Clear the time anchor so the first post-reconnect frame re-establishes
    // the mapping from hw_us to rclcpp::Time; otherwise published stamps
    // would jump backwards and trip REP-117 consumers. Both halves of the
    // anchor must be zeroed too — see standby() for the rationale.
    hw_anchor_us_.store(0, std::memory_order_relaxed);
    ros_anchor_ns_.store(0, std::memory_order_relaxed);
    time_anchor_set_.store(false, std::memory_order_release);
    return true;
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(get_logger(), "~CameraNode()");
    // Drop service and parameter callbacks first so no executor thread
    // can enter pause() / standby() / on_set_parameters() against a
    // device that is mid-shutdown. Reset releases the handles immediately;
    // any in-flight callback returns to a stub before the next dispatch.
    srv_pause_.reset();
    srv_standby_.reset();
    param_cb_handle_.reset();
    // Cancel every wall timer before any member starts destructing.
    // On a multi-threaded executor a timer tick on another thread can
    // otherwise dereference device_ after it is destroyed.
    // diagnostic_updater::Updater owns its own internal timer; releasing
    // the unique_ptr below stops its callbacks before device_ goes away.
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
    auto ro = []() {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.read_only = true;
        return d;
    };

    // Identity + binding (launch-only)
    declare_parameter<std::string>("camera_name",       "eys3d_camera", ro());
    declare_parameter<std::string>("model",             "G100P",        ro());
    declare_parameter<int>        ("mode_id",           1,              ro());
    declare_parameter<std::string>("video_modes_dir",   "",             ro());
    declare_parameter<std::string>("dev_serial_number", "",             ro());
    declare_parameter<std::string>("usb_port",          "",             ro());
    declare_parameter<int>        ("depth_minimum_mm", -1,              ro());
    declare_parameter<int>        ("depth_maximum_mm", -1,              ro());
    // ir_intensity is declared with the other open()-time parameters so
    // its launch value reaches DeviceConfig and is applied at IR pre-open.
    // Runtime-tunable via ros2 param set.
    declare_parameter<int>        ("ir_intensity",   -1);
    declare_parameter<bool>       ("colored_pointcloud",       false, ro());
    // Spatial filter (launch-only)
    declare_parameter<bool>       ("spatial_filter",           false, ro());
    declare_parameter<double>     ("spatial_filter_alpha",     0.5,   ro());
    declare_parameter<int>        ("spatial_filter_delta",     20,    ro());
    declare_parameter<int>        ("spatial_filter_magnitude", 2,     ro());
    declare_parameter<int>        ("spatial_filter_holes_fill",  5,   ro());
    // Temporal filter — runtime-tunable via ros2 param set; the
    // device applies the change on the next depth frame. A false→true
    // transition resets per-pixel history.
    declare_parameter<bool>       ("temporal_filter",             false);
    declare_parameter<double>     ("temporal_filter_alpha",       0.4);
    declare_parameter<int>        ("temporal_filter_delta",       20);
    declare_parameter<int>        ("temporal_filter_persistence", 3);
    // Hole filling — launch-time only. Mode 0=off, 1=fill_from_left,
    // 2=farthest_from_around, 3=nearest_from_around.
    declare_parameter<int>        ("hole_filling",                0,   ro());
    declare_parameter<double>     ("diagnostics_rate_hz", 1.0, ro());  // 0 = disabled
    // Runtime CT/PU image controls (enable_auto_exposure,
    // enable_auto_white_balance, exposure_time_step, power_line_frequency)
    // are declared after the device is opened, using the SDK's current
    // values as defaults. Launch overrides are written back to the camera
    // only when explicitly supplied; otherwise the existing firmware
    // configuration is preserved.

    // Frames — all default to "" → derived from camera_name at init.
    declare_parameter<std::string>("base_frame",        "", ro());
    declare_parameter<std::string>("left_color_frame",  "", ro());
    declare_parameter<std::string>("right_color_frame", "", ro());
    declare_parameter<std::string>("depth_frame",       "", ro());
    declare_parameter<std::string>("points_frame",      "", ro());

    declare_parameter<std::string>("dm_quality_cfg_dir", "", ro());
}

bool CameraNode::load_video_mode(VideoMode& out, std::string& err) const {
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
    const int mode_id       = get_parameter("mode_id").as_int();

    const auto modes = load_video_modes(dir, model);
    if (modes.empty()) {
        err = "no modes loaded from " + dir + "/" + model + ".yaml "
              "(accepted models: G100P, G100Pi, R77, G62)";
        return false;
    }
    RCLCPP_INFO(get_logger(), "%s", format_mode_table(model, modes).c_str());

    const auto found = find_mode(modes, mode_id);
    if (!found) {
        err = "mode_id=" + std::to_string(mode_id) + " not found in " + model + ".yaml";
        return false;
    }
    out = *found;
    return true;
}

DeviceConfig CameraNode::build_device_config(const VideoMode& vm) const {
    DeviceConfig c;
    c.color_width      = vm.color_width;
    c.color_height     = vm.color_height;
    c.color_format     = vm.color_format;
    c.depth_width      = vm.depth_width;
    c.depth_height     = vm.depth_height;
    c.depth_data_type  = vm.depth_data_type;
    c.zd_index         = vm.zd_index;
    c.framerate        = vm.framerate;
    c.interleave       = vm.interleave;
    c.depth_minimum_mm = get_parameter("depth_minimum_mm").as_int();
    c.depth_maximum_mm = get_parameter("depth_maximum_mm").as_int();
    c.ir_intensity               = get_parameter("ir_intensity").as_int();
    c.colored_pointcloud                   = get_parameter("colored_pointcloud").as_bool();
    c.spatial_filter_enabled   = get_parameter("spatial_filter").as_bool();
    c.spatial_filter_alpha     = get_parameter("spatial_filter_alpha").as_double();
    c.spatial_filter_delta     = get_parameter("spatial_filter_delta").as_int();
    c.spatial_filter_magnitude = get_parameter("spatial_filter_magnitude").as_int();
    c.spatial_filter_holes_fill  = get_parameter("spatial_filter_holes_fill").as_int();
    c.temporal_filter_enabled     = get_parameter("temporal_filter").as_bool();
    c.temporal_filter_alpha       = get_parameter("temporal_filter_alpha").as_double();
    c.temporal_filter_delta       = get_parameter("temporal_filter_delta").as_int();
    c.temporal_filter_persistence = get_parameter("temporal_filter_persistence").as_int();
    c.hole_filling                = get_parameter("hole_filling").as_int();
    // Drives the wide-YUYV split decode in the device layer.
    c.split_color      = split_color_;
    c.serial_number    = get_parameter("dev_serial_number").as_string();
    c.usb_port         = get_parameter("usb_port").as_string();
    c.model            = normalize_model(get_parameter("model").as_string());
    return c;
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
    int width, int height, const EspdiDevice::LensCalibration& lens,
    bool valid) const {
    sensor_msgs::msg::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = frame_id;
    ci.width  = static_cast<uint32_t>(width);
    ci.height = static_cast<uint32_t>(height);
    if (valid) {
        ci.distortion_model = "plumb_bob";
        ci.d.assign(lens.D.begin(), lens.D.end());
        for (size_t i = 0; i < 9;  ++i) ci.k[i] = lens.K[i];
        for (size_t i = 0; i < 9;  ++i) ci.r[i] = lens.R[i];
        for (size_t i = 0; i < 12; ++i) ci.p[i] = lens.P[i];
    }
    return ci;
}

void CameraNode::on_color(FrameBuffer&& f) {
    if (!pub_color_ || pub_color_->get_subscription_count() == 0) return;
    RCLCPP_INFO_ONCE(get_logger(),
                     "on_color: first frame (sn=%d, %dx%d, %zu bytes, rgb8)",
                     f.serial_number, f.width, f.height, f.data.size());
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
                          f.height, calib.left, calib.valid)));
}

void CameraNode::publish_split_color(FrameBuffer&& f) {
    // Wide-color modes pack L|R into one wide raster. Two cases:
    //
    //   Pre-split (YUYV wide, default):  espdi_device decoded the wide
    //     YUYV directly into f.data (left) and f.data_right (right) via
    //     simd::yuyv_to_rgb8_split. Both buffers are moved straight into
    //     Image messages — no per-pixel copy at this layer.
    //
    //   Wide intermediate (MJPEG wide):  espdi_device decoded the whole
    //     wide raster into f.data and left f.data_right empty. The
    //     half-width Image buffers are produced by row-by-row memcpy.
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
                              f.height, calib.left, calib.valid)));
    }
    if (publish_right && pub_color_right_info_) {
        pub_color_right_info_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(
            build_camera_info(right_color_optical_frame_, stamp, half_w,
                              f.height, calib.right, calib.valid)));
    }
}

void CameraNode::on_depth(FrameBuffer&& f) {
    if (!pub_depth_) return;
    RCLCPP_INFO_ONCE(get_logger(), "on_depth: first frame (sn=%d, %dx%d, %zu bytes)",
                     f.serial_number, f.width, f.height, f.data.size());

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
                          f.height, calib.left, calib.valid)));
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
    // Periodic point-cloud sample at DEBUG level only; use --log-level DEBUG
    // when tuning. Static clock avoids the per-frame shared_ptr allocation
    // inside the macro.
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
                state.ir_intensity, state.ir_read_ok,
                state.auto_exposure ? "auto" : "manual", state.auto_exposure_read_ok,
                state.exposure_time_step, state.exposure_read_ok,
                state.auto_white_balance ? "auto" : "manual", state.awb_read_ok,
                state.power_line_frequency, state.plf_read_ok);

    // Declare runtime parameters seeded from current SDK state so
    // `ros2 param get` reflects the actual camera configuration.
    // ir_intensity is declared in declare_params() to reach open().
    declare_parameter<bool> ("enable_auto_exposure",             state.auto_exposure);
    declare_parameter<bool> ("enable_auto_white_balance",            state.auto_white_balance);
    declare_parameter<int>  ("exposure_time_step",   state.exposure_time_step);
    declare_parameter<int>  ("power_line_frequency", state.power_line_frequency);

    // Always apply ir_intensity at launch so the projector is lit without
    // an extra parameter call. -1 = per-PID default, 0 = off, positive
    // integer = raw level.
    device_->set_ir_intensity(get_parameter("ir_intensity").as_int());

    // Other CT/PU values are written back only when explicitly overridden
    // at launch. The overrides map covers both NodeOptions overrides and
    // --ros-args / params-file values.
    const auto& overrides =
        get_node_parameters_interface()->get_parameter_overrides();
    auto was_overridden = [&](const std::string& name) {
        return overrides.find(name) != overrides.end();
    };
    if (was_overridden("enable_auto_exposure"))
        device_->set_auto_exposure(get_parameter("enable_auto_exposure").as_bool());
    if (was_overridden("enable_auto_white_balance"))
        device_->set_auto_white_balance(get_parameter("enable_auto_white_balance").as_bool());
    if (was_overridden("exposure_time_step") && !get_parameter("enable_auto_exposure").as_bool())
        (void)device_->set_exposure_time_step(get_parameter("exposure_time_step").as_int());
    if (was_overridden("power_line_frequency"))
        device_->set_power_line_frequency(get_parameter("power_line_frequency").as_int());
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
        if (name == "ir_intensity") {
            // Per-model firmware ranges: G100+ 0..9, R77 0..6, G62 0..96.
            // Clamp at the parameter layer so out-of-range writes are
            // rejected with a clear message instead of being silently
            // pinned to the FW limit.
            int max_ir = 9;
            const std::string model = normalize_model(get_parameter("model").as_string());
            if      (model == "G62") max_ir = 96;
            else if (model == "R77") max_ir = 6;
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < 0 || p.as_int() > max_ir) {
                result.successful = false;
                result.reason = "ir_intensity must be int in [0, " +
                                std::to_string(max_ir) + "] for model " + model;
                return result;
            }
        } else if (name == "enable_auto_exposure" || name == "enable_auto_white_balance") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = name + " must be bool";
                return result;
            }
        } else if (name == "exposure_time_step") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < -100 || p.as_int() > 100) {
                result.successful = false;
                result.reason = "exposure_time_step must be int in [-100, 100]";
                return result;
            }
        } else if (name == "power_line_frequency") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                p.as_int() < 0 || p.as_int() > 3) {
                result.successful = false;
                result.reason = "power_line_frequency must be int in [0..3] (0=off, 1=50Hz, 2=60Hz, 3=auto)";
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
    // Collect any temporal-filter changes in the incoming batch and
    // merge them with the current parameter values, so a single API
    // call carries the full {enabled, alpha, delta, persistence}
    // tuple. get_parameter() still returns the pre-update values at
    // this point, which is the desired baseline for the merge.
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

    for (const auto& p : params) {
        const auto& name = p.get_name();
        bool ok = true;
        if (name == "ir_intensity") {
            ok = device_->set_ir_intensity(p.as_int());
        } else if (name == "enable_auto_exposure") {
            ok = device_->set_auto_exposure(p.as_bool());
            // Re-apply the current exposure_time_step parameter when
            // AE transitions to off so the manual value takes effect
            // immediately. A firmware reject is surfaced through
            // result.reason so the operator sees a hint when the
            // parameter store and FW state diverge.
            if (ok && !p.as_bool()) {
                const int step = get_parameter("exposure_time_step").as_int();
                if (!device_->set_exposure_time_step(step)) {
                    result.reason = "exposure_time_step accepted into parameter store, "
                                    "but firmware rejected the immediate write; the "
                                    "value will be retried on the next enable_auto_exposure transition";
                }
            }
        } else if (name == "exposure_time_step") {
            const bool ae_on = get_parameter("enable_auto_exposure").as_bool();
            if (!ae_on) {
                // Some firmware versions reject specific exposure values via
                // the UVC CT path. Surface the SDK rejection in result.reason
                // so a consumer using the parameter API sees a hint, but keep
                // result.successful=true so the parameter store still holds
                // the value for the next enable_auto_exposure transition.
                if (!device_->set_exposure_time_step(p.as_int())) {
                    result.reason = "exposure_time_step accepted into parameter store, "
                                    "but firmware rejected the immediate write; the "
                                    "value will be retried on the next enable_auto_exposure transition";
                }
            }
            // When AE is on the value is held for the next manual transition.
        } else if (name == "enable_auto_white_balance") {
            ok = device_->set_auto_white_balance(p.as_bool());
        } else if (name == "power_line_frequency") {
            ok = device_->set_power_line_frequency(p.as_int());
        }
        if (!ok) {
            result.successful = false;
            result.reason = "SDK rejected " + name;
            return result;
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
        // Aggregate liveness: connected and Active but every configured
        // stream is below 50% of expected (or zero) → ERROR at the
        // device level. Per-stream WARN is already surfaced by the
        // color and depth tasks; this branch catches the "device looks
        // open but firmware has stopped delivering anything" case.
        // color_input_fps == 0 in D-only modes is normal — only count
        // it as dead when the configured stream stops.
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
        status_str = "idle (no /pointcloud subscriber)";
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
