#ifndef EYS3D_CAMERA__CAMERA_NODE_HPP_
#define EYS3D_CAMERA__CAMERA_NODE_HPP_

// API stability note
// ------------------
// CameraNode is an rclcpp::Node and the rclcpp::components plugin
// "eys3d_camera::CameraNode". Stable across the 2.x series:
//
//   * published topics, their types and the frame ids
//   * service names and types: pause, standby (std_srvs/srv/SetBool),
//     hw_reset (std_srvs/srv/Empty)
//   * declared parameters, their types and read-only flags
//   * QoS on every endpoint (image_qos() / info_qos() in camera_node.cpp)
//   * the /diagnostics KeyValue keys the README lists
//   * the component plugin name
//
// Everything else in this header is implementation detail. Do not subclass
// CameraNode, depend on the field layout, or take the address of a member.
//
// Lifecycle: CameraNode must be destroyed only after rclcpp::shutdown() has
// returned. The destructor relies on that to guarantee no service callback is
// dispatched while it tears down device_.

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <eys3d_camera_interfaces/action/self_cal.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "eys3d_camera/espdi_device.hpp"
#include "eys3d_camera/video_modes.hpp"

namespace eys3d_camera {

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

private:
    void declare_params();
    // Loads the model's catalogue (mode entry + per-model header) from
    // launch/video_modes/<MODEL>.yaml; fills model_info_.
    bool load_video_mode(VideoMode& out, std::string& err_msg);
    DeviceConfig build_device_config(const VideoMode& vm) const;
    ModelInfo model_info_;

    void on_color(FrameBuffer&& f);
    void on_depth(FrameBuffer&& f);
    void on_point_cloud(std::vector<uint8_t>&& xyz_bytes,
                        uint32_t valid_points,
                        uint32_t point_step,
                        uint64_t hw_timestamp_us);

    rclcpp::Time stamp_from_hw_us(uint64_t hw_us);
    // Depth is rectified in every mode; color follows the mode's depth type.
    enum class InfoStream { kLeftColor, kRightColor, kDepth, kCount };

    // Describes the image on that topic only. Rectified form: K is P's left
    // 3x3, D zero, R identity. Raw form: the factory lens model.
    sensor_msgs::msg::CameraInfo build_camera_info(
        const std::string& frame_id, const rclcpp::Time& stamp,
        int width, int height, const EspdiDevice::Calibration& calib,
        InfoStream stream) const;
    void warn_if_off_raster(const sensor_msgs::msg::CameraInfo& ci,
                            InfoStream stream) const;
    // Cleared on reconnect, which may land on a different calibration.
    mutable std::array<std::atomic<bool>,
                       static_cast<size_t>(InfoStream::kCount)> off_raster_warned_{};

    // Locked snapshot of cached_calib_ for per-frame CameraInfo publishes.
    EspdiDevice::Calibration snapshot_calib() const;

    // Wide-frame (L|R packed) slicer used when split_color is true. Yields
    // two per-half ROS messages from a single fetched FrameBuffer. The
    // sliced halves share a hw timestamp so consumers can synchronise them.
    void publish_split_color(FrameBuffer&& f);

    // Static TF: <cam>_link → {left/right/depth/points}_frame, with left/depth/
    // points placed at +baseline/2 on the Y (ROS-base left) axis and right at
    // -baseline/2. Sent once after open() reports a valid calibration; cost is
    // an O(4) broadcast, then zero (latched by tf2).
    void publish_static_tf();

    // rclcpp parameter-set callback. Accepts ir_value,
    // auto_exposure, exposure_time_step, auto_white_balance,
    // power_line_frequency and dispatches to the EspdiDevice
    // setters. Unknown parameters and out-of-range values are rejected.
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter>& params);

    // Reads current CT/PU state from the SDK and declares matching ROS
    // parameters with those values as defaults. Only parameters explicitly
    // overridden in the launch command (detected via
    // NodeOptions::parameter_overrides) are written back to the device.
    void declare_and_apply_runtime_params();

    // Re-sync the parameter store from the camera's persisted CT/PU state
    // after a reopen. Those values live in the module's flash and survive a
    // USB re-enumerate, so the firmware is the source of truth: read it back
    // rather than re-pushing the store. Called from try_reconnect() and
    // standby(false); ir_value is re-applied separately by those callers
    // because the projector boots OFF on open.
    void resync_ct_pu_from_device();

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    // Runtime stream-control services. /<cam>/pause is a cheap atomic
    // gate (USB keeps running, frames are dropped before decode/publish);
    // /<cam>/standby tears the USB pipe down via APC_CloseDevice and
    // sets user_wants_standby_ so the watchdog does not interpret the
    // intentional idle as a disconnect.
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pause_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_standby_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_hw_reset_;
    // Self-calibration (selfk) control, created only when selfcal_enable is set.
    // The run action drives one full session to completion and resolves it;
    // commit persists a kept result to flash. No start/stop — a session runs to
    // completion and cannot be interrupted (a running one auto-reverts if worse).
    using SelfCalAction = eys3d_camera_interfaces::action::SelfCal;
    using SelfCalGoalHandle = rclcpp_action::ServerGoalHandle<SelfCalAction>;
    rclcpp_action::Server<SelfCalAction>::SharedPtr act_selfcal_run_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_selfcal_commit_;
    std::shared_ptr<SelfCalGoalHandle> selfcal_run_goal_;
    rclcpp::TimerBase::SharedPtr selfcal_run_timer_;
    rclcpp::Time selfcal_run_deadline_;
    float selfcal_run_auto_commit_shift_px_ = -1.0f;

    rclcpp_action::GoalResponse selfcal_handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const SelfCalAction::Goal> goal);
    rclcpp_action::CancelResponse selfcal_handle_cancel(
        std::shared_ptr<SelfCalGoalHandle> gh);
    void selfcal_handle_accepted(std::shared_ptr<SelfCalGoalHandle> gh);
    void selfcal_run_tick();  // wall-timer: feed feedback + resolve on completion

    std::atomic<bool> user_wants_standby_{false};

    // /diagnostics via diagnostic_updater::Updater. The five per-task
    // callbacks (device / color / depth / pointcloud / thermal) and
    // their registration order live in camera_node.cpp.
    std::unique_ptr<diagnostic_updater::Updater> updater_;
    void diagnose_device   (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_color    (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_depth    (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_pc       (diagnostic_updater::DiagnosticStatusWrapper& s);
    void diagnose_thermal  (diagnostic_updater::DiagnosticStatusWrapper& s);
    // Per-tick derived rates. Refreshed by diagnose_device() (registered
    // first, so runs first) and reused by the other tasks.
    struct DiagSnapshot {
        double color_input_fps   = 0.0;
        double depth_input_fps   = 0.0;
        double color_publish_fps = 0.0;
        double depth_publish_fps = 0.0;
        double color_decode_avg_ms = 0.0;
        double pc_publish_fps    = 0.0;
        double pc_compute_avg_ms = 0.0;
        uint64_t color_decode_delta_count = 0;
        uint64_t pc_count_delta  = 0;
        EspdiDevice::Stats cur{};
    } diag_snap_;
    void refresh_diag_snapshot();
    // Previous totals + wall time, so per-second fps is delta / dt.
    EspdiDevice::Stats prev_stats_{};
    rclcpp::Time prev_stats_wall_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<EspdiDevice> device_;
    // cached_calib_ is read by the camera_info wall timer and the static-TF
    // republish timer, and written by open() / try_reconnect(). On a
    // multi-threaded executor those accesses overlap, so the mutex
    // protects against torn reads of the float arrays inside Calibration.
    mutable std::mutex calib_mtx_;
    EspdiDevice::Calibration cached_calib_{};

    // tf_static_ publishes the static TF tree on /tf_static once with
    // TRANSIENT_LOCAL durability. It is constructed with intra-process
    // comms explicitly disabled per-publisher so the /tf_static path
    // stays portable across distros even when the node has node-level
    // IPC enabled — any late-joining subscriber receives the cached
    // transforms immediately.
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_right_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_right_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;

    // camera_name is the launch-supplied label (e.g. "G100P_1"). It is used
    // both as ROS namespace prefix for topics and as the frame_id prefix for
    // all coordinate frames published by this node.
    std::string camera_name_;
    // Two-layer TF: <cam>_link (ROS base) → per-stream sensor frame →
    // _optical_frame leaf (REP-103). Image messages stamp the leaf;
    // PointCloud2 stamps <cam>_points_frame (already in base axes).
    std::string base_frame_;
    std::string left_color_frame_;
    std::string right_color_frame_;
    std::string depth_frame_;
    std::string points_frame_;
    std::string left_color_optical_frame_;
    std::string right_color_optical_frame_;
    std::string depth_optical_frame_;
    std::string dm_quality_cfg_dir_;
    bool split_color_ = false;

    std::atomic<bool> dm_quality_applied_{false};

    // Connection state machine driven by the 1 Hz watchdog timer.
    enum class ConnState { kStreaming, kDisconnected };
    std::atomic<ConnState> conn_state_{ConnState::kStreaming};
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    EspdiDevice::Stats watchdog_prev_stats_{};
    // Written by the wall-timer callback and by the standby() handler, which
    // on a multi-threaded executor can run on different threads. Relaxed
    // atomics suffice: independent counters, no ordering dependency on other
    // state.
    // Per-stream disconnect detection. Which streams the active mode runs is
    // fixed at construction; each is watched from the open, through the startup
    // grace until it delivers a frame and through its own silence counter after.
    bool color_configured_{false};
    bool depth_configured_{false};
    // Fixed at construction from the mode's depth data type.
    bool color_rectified_{true};
    std::atomic<bool> color_armed_{false};
    std::atomic<bool> depth_armed_{false};
    std::atomic<int>  color_silent_seconds_{0};    // consecutive 1-s ticks with no colour frame
    std::atomic<int>  depth_silent_seconds_{0};    // consecutive 1-s ticks with no depth frame
    std::atomic<int>  startup_grace_seconds_{0};   // ticks elapsed before any frame seen
    // Reopens spent on a stream that has never delivered. Cleared once every
    // configured stream has armed, and not by the reopen itself -- that is what
    // bounds the loop.
    std::atomic<int>  cold_start_reopens_{0};
    static constexpr int kMaxColdStartReopens = 3;
    // Counted on the watchdog timer; read on the diagnostics timer.
    // Atomic so a multi-threaded executor cannot tear the 64-bit value
    // on 32-bit ARM targets.
    std::atomic<uint64_t> reconnect_attempts_{0};
    // Written by the watchdog and by the standby and hw_reset handlers; they
    // cannot overlap while the node's callbacks share one callback group.
    int reconnect_poll_counter_ = 0;
    // Cached so the watchdog can re-issue open() + start() after a disconnect.
    // on_set_parameters() mutates the temporal_filter_* fields from the
    // executor thread; try_reconnect() reads the struct under cfg_mtx_ so
    // those writes do not tear the std::string members read by open().
    mutable std::mutex cfg_mtx_;
    DeviceConfig cached_cfg_{};
    ColorFrameCb cached_color_cb_;
    DepthFrameCb cached_depth_cb_;
    PointCloudCb cached_pc_cb_;
    void watchdog_tick();
    // Stop and close the device, force Disconnected, and reset the watchdog
    // arming / grace / poll state so the reconnect loop starts clean.
    void declare_disconnected();
    bool try_reconnect();

    std::atomic<bool> time_anchor_set_{false};
    std::atomic<int64_t> hw_anchor_us_{0};
    std::atomic<int64_t> ros_anchor_ns_{0};
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__CAMERA_NODE_HPP_
