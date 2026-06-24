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
    VideoMode v;
    v.id   = id;
    v.name = value_or<std::string>(m["name"], "");

    // Default both streams to OFF; the explicit YAML blocks below
    // flip them on. Modes that omit `color:` or `depth:` (D-only and
    // L'+R' modes) keep the corresponding stream disabled.
    v.has_color = false;
    v.has_depth = false;
    if (const auto c = m["color"]) {
        v.color_width  = value_or<int>(c["w"],   0);
        v.color_height = value_or<int>(c["h"],   0);
        v.color_format = value_or<int>(c["fmt"], 0);
        v.color_split  = value_or<bool>(c["split_lr"], false);
        v.has_color    = v.color_width > 0 && v.color_height > 0;
    }
    if (const auto d = m["depth"]) {
        v.depth_width     = value_or<int>(d["w"],     0);
        v.depth_height    = value_or<int>(d["h"],     0);
        v.depth_data_type = value_or<int>(d["dtype"], 0);
        v.has_depth       = v.depth_width > 0 && v.depth_height > 0;
    }
    // zd_index is a per-mode property (it picks the rectify LUT row for the
    // active resolution group); the color and depth blocks no longer carry
    // their own copy.
    v.zd_index = value_or<int>(m["zd_index"], 0);
    v.framerate  = value_or<int>(m["fps"], 0);
    v.interleave = value_or<bool>(m["interleave"], false);
    return v;
}
}  // namespace

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
