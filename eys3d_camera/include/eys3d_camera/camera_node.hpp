#ifndef EYS3D_CAMERA__CAMERA_NODE_HPP_
#define EYS3D_CAMERA__CAMERA_NODE_HPP_

// API stability note
// ------------------
// The CameraNode class is registered as an rclcpp::Node and as an
// rclcpp::components plugin "eys3d_camera::CameraNode". 1.0.0 is the
// first version where this pact applies:
//
//   Stable across the 1.x series:
//     * published topics + types + frame IDs
//     * service names + types ("pause", "standby" — both
//       std_srvs/srv/SetBool, see camera_node.cpp for semantics; the
//       pre-1.0 "enable_color" / "enable_depth" services were replaced
//       and will NOT come back)
//     * declared parameters + types + read-only flags
//     * QoS profiles on every endpoint (image_qos() and info_qos() in
//       camera_node.cpp)
//     * the /diagnostics KeyValue keys listed in the README diagnostics
//       table (the "stream_state" key replaces the pre-1.0
//       "color_enabled" / "depth_enabled" pair)
//     * the component plugin name above (so composition consumers keep
//       working without code changes)
//
// Everything else in this header — the private member layout, the
// types pulled in from espdi_device.hpp / video_modes.hpp, and the
// nested helper functions — is implementation detail and may change
// without bumping the major version. Do NOT subclass CameraNode, do
// NOT depend on the field layout, and do NOT take the address of any
// member. Treat this header as opaque except for the constructor and
// destructor.
//
// Lifecycle: CameraNode is destroyed only after rclcpp::shutdown() has
// returned (or the owning executor's spin loop has exited). The
// destructor relies on this to guarantee no service callback is
// dispatched while it tears down device_; if the rclcpp executor is
// still spinning when the node goes out of scope, the standby() /
// pause() callbacks can race with member destruction.

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
    bool load_video_mode(VideoMode& out, std::string& err_topic) const;
    DeviceConfig build_device_config(const VideoMode& vm) const;

    void on_color(FrameBuffer&& f);
    void on_depth(FrameBuffer&& f);
    void on_point_cloud(std::vector<uint8_t>&& xyz_bytes,
                        uint32_t valid_points,
                        uint32_t point_step,
                        uint64_t hw_timestamp_us);

    rclcpp::Time stamp_from_hw_us(uint64_t hw_us);
    sensor_msgs::msg::CameraInfo build_camera_info(
        const std::string& frame_id, const rclcpp::Time& stamp,
        int width, int height, const EspdiDevice::LensCalibration& lens,
        bool valid) const;

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

    // Republish all three camera_info topics on a low-rate timer (default 1 Hz).
    // rclcpp parameter-set callback. Accepts ir_intensity,
    // enable_auto_exposure, exposure_time_step, enable_auto_white_balance,
    // power_line_frequency and dispatches to the EspdiDevice
    // setters. Unknown parameters and out-of-range values are rejected.
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter>& params);

    // Reads current CT/PU state from the SDK and declares matching ROS
    // parameters with those values as defaults. Only parameters explicitly
    // overridden in the launch command (detected via
    // NodeOptions::parameter_overrides) are written back to the device.
    void declare_and_apply_runtime_params();

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    // Runtime stream-control services. /<cam>/pause is a cheap atomic
    // gate (USB keeps running, frames are dropped before decode/publish);
    // /<cam>/standby tears the USB pipe down via APC_CloseDevice and
    // sets user_wants_standby_ so the watchdog does not interpret the
    // intentional idle as a disconnect.
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pause_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_standby_;
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
    // Watchdog counters are written by the wall-timer callback AND by the
    // standby() service handler (which re-arms the startup grace after a
    // successful resume). On a multi-threaded executor these can run on
    // different threads; atomic relaxed loads/stores are sufficient
    // because the values are independent counters with no ordering
    // dependency on other state.
    std::atomic<int>  zero_frame_seconds_{0};      // consecutive 1-s ticks with no frames
    std::atomic<bool> watchdog_armed_{false};      // disconnect-detect active only after first frame
    std::atomic<int>  startup_grace_seconds_{0};   // ticks elapsed before any frame seen
    // Counted on the watchdog timer; read on the diagnostics timer.
    // Atomic so a multi-threaded executor cannot tear the 64-bit value
    // on 32-bit ARM targets.
    std::atomic<uint64_t> reconnect_attempts_{0};
    int reconnect_poll_counter_ = 0;        // throttles re-open attempts (watchdog-thread only)
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
    bool try_reconnect();

    std::atomic<bool> time_anchor_set_{false};
    std::atomic<int64_t> hw_anchor_us_{0};
    std::atomic<int64_t> ros_anchor_ns_{0};
};

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__CAMERA_NODE_HPP_
