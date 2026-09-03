#include "eys3d_camera/video_modes.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace eys3d_camera {

namespace {
const char* logger() { return "VideoModes"; }

// Pull a scalar with a default. The YAML may legitimately omit fields
// (e.g. mode 10 "D-only" has no color block) — those defaults are fine.
template <typename T>
T value_or(const YAML::Node& n, const T& def) {
    if (!n || n.IsNull()) return def;
    try { return n.as<T>(); }
    catch (const YAML::Exception&) { return def; }
}

VideoMode parse_one(int id, const YAML::Node& m) {
    VideoMode mode;
    mode.id   = id;
    mode.name = value_or<std::string>(m["name"], "");

    // Default both streams to OFF; the explicit YAML blocks below
    // flip them on. Modes that omit `color:` or `depth:` (D-only and
    // L'+R' modes) keep the corresponding stream disabled.
    mode.has_color = false;
    mode.has_depth = false;
    if (const auto c = m["color"]) {
        mode.color_width  = value_or<int>(c["w"],   0);
        mode.color_height = value_or<int>(c["h"],   0);
        mode.color_format = value_or<int>(c["fmt"], 0);
        mode.color_split  = value_or<bool>(c["split_lr"], false);
        mode.has_color    = mode.color_width > 0 && mode.color_height > 0;
    }
    if (const auto d = m["depth"]) {
        mode.depth_width     = value_or<int>(d["w"],     0);
        mode.depth_height    = value_or<int>(d["h"],     0);
        mode.depth_data_type = value_or<int>(d["dtype"], 0);
        mode.has_depth       = mode.depth_width > 0 && mode.depth_height > 0;
    }
    // zd_index is a per-mode property: it picks the rectify LUT row for
    // the active resolution group.
    mode.zd_index = value_or<int>(m["zd_index"], 0);
    mode.framerate  = value_or<int>(m["fps"], 0);
    mode.interleave = value_or<bool>(m["interleave"], false);
    mode.usb        = value_or<int>(m["usb"], 0);   // 2 = USB2.0, 3 = USB3.0
    return mode;
}
}  // namespace

std::optional<ModelInfo> load_model_info(const std::string& yaml_dir,
                                         const std::string& model) {
    const std::string path = yaml_dir + "/" + model + ".yaml";
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node ir   = root["ir"];
        const YAML::Node range = root["depth_range_mm"];
        if (!root["model"] || !root["pid"] || !root["mono"] ||
            !ir || !ir["min"] || !ir["max"] || !ir["default"] ||
            !range || !range["near"] || !range["far"]) {
            RCLCPP_ERROR(rclcpp::get_logger(logger()),
                         "catalogue '%s' is missing a required model header "
                         "field (model / pid / mono / ir / depth_range_mm)",
                         path.c_str());
            return std::nullopt;
        }
        ModelInfo info;
        info.model = root["model"].as<std::string>();
        // base 0 accepts both the catalogue's 0x-prefixed hex and decimal.
        info.pid = static_cast<unsigned short>(
            std::stoul(root["pid"].as<std::string>(), nullptr, 0));
        info.mono  = root["mono"].as<bool>();
        info.ir_min     = ir["min"].as<int>();
        info.ir_max     = ir["max"].as<int>();
        info.ir_default = ir["default"].as<int>();
        info.depth_near_mm = range["near"].as<int>();
        info.depth_far_mm  = range["far"].as<int>();
        // Optional: USB port type -> default mode_id (the "signature" mode
        // opened when mode_id is left at its auto sentinel).
        if (const YAML::Node sig = root["signature_mode"]) {
            for (const auto& kv : sig)
                info.signature_mode[kv.first.as<int>()] = kv.second.as<int>();
        }
        return info;
    } catch (const std::exception& e) {  // YAML parse + pid conversion
        RCLCPP_ERROR(rclcpp::get_logger(logger()),
                     "model header parse failed for '%s': %s",
                     path.c_str(), e.what());
        return std::nullopt;
    }
}

std::vector<VideoMode> load_video_modes(const std::string& yaml_dir,
                                        const std::string& model) {
    const std::string path = yaml_dir + "/" + model + ".yaml";
    std::vector<VideoMode> out;
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node modes = root["modes"];
        if (!modes || !modes.IsMap()) {
            RCLCPP_ERROR(rclcpp::get_logger(logger()),
                         "video mode catalogue '%s' missing top-level 'modes:' map", path.c_str());
            return out;
        }
        for (auto it = modes.begin(); it != modes.end(); ++it) {
            const int id = it->first.as<int>();
            out.push_back(parse_one(id, it->second));
        }
        std::sort(out.begin(), out.end(),
                  [](const VideoMode& a, const VideoMode& b){ return a.id < b.id; });
        RCLCPP_INFO(rclcpp::get_logger(logger()),
                    "Loaded %zu video modes from %s", out.size(), path.c_str());
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger(logger()),
                     "YAML parse failed for '%s': %s", path.c_str(), e.what());
    }
    return out;
}

std::optional<VideoMode> find_mode(const std::vector<VideoMode>& modes, int id) {
    for (const auto& m : modes) if (m.id == id) return m;
    return std::nullopt;
}

std::string format_mode_table(const std::string& model,
                              const std::vector<VideoMode>& modes) {
    std::ostringstream os;
    os << "Video modes for " << model << " (" << modes.size() << " entries):\n";
    os << "  ID  Color           Depth         FPS  Itlv  Notes\n";
    os << "  --  --------------  ------------  ---  ----  -----------------------------\n";
    for (const auto& m : modes) {
        std::ostringstream color;
        if (m.has_color) {
            color << m.color_width << "x" << m.color_height
                  << (m.color_format == 0 ? " YUYV" : " MJPG")
                  << (m.color_split ? " L|R" : "");
        } else {
            color << "-";
        }
        std::ostringstream depth;
        if (m.has_depth) {
            depth << m.depth_width << "x" << m.depth_height
                  << " dt" << m.depth_data_type
                  << "/zd" << m.zd_index;
        } else {
            depth << "-";
        }
        os << "  " << std::setw(2) << m.id << "  "
           << std::left << std::setw(14) << color.str() << "  "
           << std::left << std::setw(12) << depth.str() << "  "
           << std::right << std::setw(3) << m.framerate << "  "
           << std::setw(4) << (m.interleave ? "yes" : "no") << "  "
           << m.name << "\n";
    }
    return os.str();
}

}  // namespace eys3d_camera
