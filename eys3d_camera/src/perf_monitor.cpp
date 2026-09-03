// Live performance monitor for the eys3d_camera driver. Subscribes with
// SensorDataQoS so the driver leaves its no-subscriber idle state.
//
//   SDK : frames received from the camera
//   Pub : frames the driver emitted to the topic
//   Rx  : frames this monitor received over DDS
//
// Usage:
//   ros2 run eys3d_camera perf_monitor                # auto-detect namespace
//   ros2 run eys3d_camera perf_monitor --ns /G100P_1
//   ros2 run eys3d_camera perf_monitor --interval 0.5

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// First matching stream on the graph; empty when nothing matches.
std::string autodetect_namespace(rclcpp::Node::SharedPtr probe) {
    // Give DDS discovery a brief moment.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const auto topics = probe->get_topic_names_and_types();
    for (const char* suffix : {"/left_color/image_raw", "/depth/image_raw"}) {
        const std::string suffix_s = suffix;
        for (const auto& [name, types] : topics) {
            if (!ends_with(name, suffix_s)) continue;
            for (const auto& t : types) {
                if (t == "sensor_msgs/msg/Image") {
                    return name.substr(0, name.size() - suffix_s.size());
                }
            }
        }
    }
    return {};
}

class PerfMonitor : public rclcpp::Node {
public:
    PerfMonitor(std::string ns, double interval_s)
        : rclcpp::Node("eys3d_perf_monitor"),
          namespace_(std::move(ns)),
          interval_s_(interval_s),
          last_tick_(now()) {
        const auto qos = rclcpp::SensorDataQoS();

        // SerializedMessage callbacks: rclcpp delivers raw CDR bytes
        // and skips deserialisation. Only the message count is used.
        auto bump = [](uint64_t& counter) {
            return [&counter](std::shared_ptr<rclcpp::SerializedMessage>) {
                ++counter;
            };
        };
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            namespace_ + "/left_color/image_raw",  qos, bump(rx_color_));
        sub_right_color_ = create_subscription<sensor_msgs::msg::Image>(
            namespace_ + "/right_color/image_raw", qos, bump(rx_right_color_));
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            namespace_ + "/depth/image_raw", qos, bump(rx_depth_));
        sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            namespace_ + "/depth/points",  qos, bump(rx_pc_));

        // /diagnostics is small and the KeyValue contents are read on
        // every tick, so a regular deserialised subscription is fine.
        sub_diag_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(10),
            [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
                on_diag(*msg);
            });

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(interval_s_));
        timer_ = create_wall_timer(period, [this] { print_tick(); });

        RCLCPP_INFO(get_logger(),
                    "Listening on %s/* every %.1f s",
                    namespace_.c_str(), interval_s_);
    }

private:
    void on_diag(const diagnostic_msgs::msg::DiagnosticArray& msg) {
        // diagnostic_updater names each status "<hardware_id>: <task>".
        // The five tasks (device, color, depth, pointcloud, thermal)
        // are flattened into a single "<task>.<key>" lookup so the
        // print formatter resolves each metric by its task / key pair.
        std::map<std::string, std::string> flattened;
        for (const auto& st : msg.status) {
            const auto sep = st.name.find(": ");
            if (sep == std::string::npos) continue;
            const std::string task = st.name.substr(sep + 2);
            for (const auto& kv : st.values) {
                flattened[task + "." + kv.key] = kv.value;
            }
        }
        diag_ = std::move(flattened);
    }

    std::string lookup_float(const std::string& key) const {
        auto it = diag_.find(key);
        if (it == diag_.end()) return "  --";
        char buf[16];
        try {
            std::snprintf(buf, sizeof(buf), "%5.1f", std::stod(it->second));
            return buf;
        } catch (...) {
            return it->second;
        }
    }

    std::string lookup_str(const std::string& key) const {
        auto it = diag_.find(key);
        return it == diag_.end() ? std::string{"--"} : it->second;
    }

    static std::string fmt_rx(double v) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%5.1f", v);
        return buf;
    }

    void print_tick() {
        const auto t_now = now();
        const double dt = std::max(1e-3, (t_now - last_tick_).seconds());
        const double rx_color_hz       = (rx_color_       - last_rx_color_)       / dt;
        const double rx_right_color_hz = (rx_right_color_ - last_rx_right_color_) / dt;
        const double rx_depth_hz       = (rx_depth_       - last_rx_depth_)       / dt;
        const double rx_pc_hz          = (rx_pc_          - last_rx_pc_)          / dt;
        last_rx_color_       = rx_color_;
        last_rx_right_color_ = rx_right_color_;
        last_rx_depth_       = rx_depth_;
        last_rx_pc_          = rx_pc_;
        last_tick_           = t_now;

        std::time_t now_t =
            std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char ts[16];
        std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&now_t));

        std::cout
            << "=== eys3d_camera perf [" << namespace_ << "] @ " << ts << " ===\n"
            << "  color | SDK " << lookup_float("color.input_fps")
            << " \xe2\x86\x92 Pub " << lookup_float("color.publish_fps")
            << " \xe2\x86\x92 Rx " << fmt_rx(rx_color_hz)
            << "  | decode avg " << lookup_float("color.decode_avg_ms") << " ms"
            << "   max " << lookup_float("color.decode_max_ms") << " ms"
            << "   dropped " << lookup_str("color.input_dropped") << "\n";
        if (rx_right_color_ > 0) {
            std::cout
                << "  right | (split_color)                                 "
                << "                Rx " << fmt_rx(rx_right_color_hz) << "\n";
        }
        std::cout
            << "  depth | SDK " << lookup_float("depth.input_fps")
            << " \xe2\x86\x92 Pub " << lookup_float("depth.publish_fps")
            << " \xe2\x86\x92 Rx " << fmt_rx(rx_depth_hz)
            << "  |                                       "
            << "   dropped " << lookup_str("depth.input_dropped") << "\n"
            << "  pc    |              Pub " << lookup_float("pointcloud.publish_fps")
            << " \xe2\x86\x92 Rx " << fmt_rx(rx_pc_hz)
            << "  | compute avg " << lookup_float("pointcloud.compute_avg_ms") << " ms"
            << "   max " << lookup_float("pointcloud.compute_max_ms") << " ms"
            << "   total " << lookup_str("pointcloud.publish_total") << "\n"
            << "  temperature_c " << lookup_str("thermal.temperature_c")
            << std::endl;
    }

    const std::string namespace_;
    const double      interval_s_;
    rclcpp::Time      last_tick_;

    uint64_t rx_color_       = 0;
    uint64_t rx_right_color_ = 0;
    uint64_t rx_depth_       = 0;
    uint64_t rx_pc_          = 0;
    uint64_t last_rx_color_       = 0;
    uint64_t last_rx_right_color_ = 0;
    uint64_t last_rx_depth_       = 0;
    uint64_t last_rx_pc_          = 0;

    std::map<std::string, std::string> diag_;

    // SerializedMessage callbacks produce a
    // Subscription<rclcpp::SerializedMessage>; /diagnostics uses a
    // deserialised DiagnosticArray subscription. Storing all of them
    // through the common base avoids two parallel typed members.
    rclcpp::SubscriptionBase::SharedPtr sub_color_;
    rclcpp::SubscriptionBase::SharedPtr sub_right_color_;
    rclcpp::SubscriptionBase::SharedPtr sub_depth_;
    rclcpp::SubscriptionBase::SharedPtr sub_pc_;
    rclcpp::SubscriptionBase::SharedPtr sub_diag_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string ns;
    double interval = 1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--ns" || arg == "-n") && i + 1 < argc) {
            ns = argv[++i];
        } else if ((arg == "--interval" || arg == "-i") && i + 1 < argc) {
            interval = std::stod(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cerr <<
                "Usage: perf_monitor [--ns /<camera_name>] [--interval <seconds>]\n";
            rclcpp::shutdown();
            return 0;
        }
    }

    if (ns.empty()) {
        auto probe = rclcpp::Node::make_shared("eys3d_perf_probe");
        ns = autodetect_namespace(probe);
        if (ns.empty()) {
            std::cerr <<
                "Could not auto-detect a camera namespace from the topic graph. "
                "Pass --ns /<camera_name> explicitly.\n";
            rclcpp::shutdown();
            return 1;
        }
    }
    if (!ns.empty() && ns[0] != '/') ns = "/" + ns;

    auto node = std::make_shared<PerfMonitor>(ns, interval);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
