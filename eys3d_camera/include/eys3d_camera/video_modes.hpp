#ifndef EYS3D_CAMERA__VIDEO_MODES_HPP_
#define EYS3D_CAMERA__VIDEO_MODES_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace eys3d_camera {

// One entry from a per-model video-mode catalogue. The YAML schema is
// documented in launch/video_modes/<MODEL>.yaml.
struct VideoMode {
    int  id = 0;
    std::string name;

    int  color_width  = 0;
    int  color_height = 0;
    int  color_format = 0;         // 0 = YUYV, 1 = MJPEG
    bool color_split  = false;     // true when L|R are packed in one wide frame

    int  depth_width  = 0;
    int  depth_height = 0;
    int  depth_data_type = 0;      // see APC_DEPTH_DATA_* in eSPDI_def.h
    int  zd_index        = 0;

    int  framerate    = 0;
    bool interleave   = false;

    // USB port type the mode needs (2 = USB2.0, 3 = USB3.0). Opening a mode
    // whose usb type differs from the negotiated link is rejected up front.
    int  usb          = 0;

    // Indicates which streams the mode produces. The YAML parser sets these
    // only when a `color:` or `depth:` block is present and contains valid
    // dimensions, allowing publishers and fetch threads to be skipped for
    // modes that omit a stream entirely.
    bool has_color = false;
    bool has_depth = false;
};

// APC_DEPTH_DATA_* is a base code plus +16 interleave and +32 scale-down, so
// the base is the code modulo 16. The base covers the color output only:
// depth is rectified in every type, which is why APC_DEPTH_DATA_14_BITS_RAW
// still carries valid depth.
inline bool color_is_rectified(int depth_data_type) {
    switch (depth_data_type % 16) {
    case 1:   // APC_DEPTH_DATA_8_BITS
    case 2:   // APC_DEPTH_DATA_14_BITS
    case 3:   // APC_DEPTH_DATA_8_BITS_x80
    case 4:   // APC_DEPTH_DATA_11_BITS
    case 5:   // APC_DEPTH_DATA_OFF_RECTIFY
    case 11:  // APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY
    case 13:  // APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY
        return true;
    default:  // *_RAW, OFF_RAW, OFF_BAYER_RAW
        return false;
    }
}

// Per-model constants from the catalogue header — the single source of
// truth for values that vary by camera model. The code layer keeps no
// per-model tables; adding a camera means adding a YAML file.
struct ModelInfo {
    std::string model;
    unsigned short pid = 0;      // USB product ID
    bool mono = false;           // monochrome sensor pair (color stream is luma)
    int ir_min = 0;              // projector level range
    int ir_max = 0;
    int ir_default = 0;          // level applied when ir_value = -1
    int depth_near_mm = 0;       // working-range defaults for the point cloud
    int depth_far_mm = 0;
    // USB port type (2 / 3) -> the mode_id to open when the launch left
    // mode_id at its "auto" sentinel. Empty when the catalogue omits it.
    std::map<int, int> signature_mode;
};

// Loads the model header from <yaml_dir>/<model>.yaml. Every header field
// is required; returns nullopt and logs on a missing field or parse error.
std::optional<ModelInfo> load_model_info(const std::string& yaml_dir,
                                         const std::string& model);

// Loads <yaml_dir>/<model>.yaml and returns every mode in the catalogue.
// Returns an empty list and logs an error on parse failure.
std::vector<VideoMode> load_video_modes(const std::string& yaml_dir,
                                        const std::string& model);

// Looks up a mode by its catalogue ID. Returns nullopt when the ID is not
// present.
std::optional<VideoMode> find_mode(const std::vector<VideoMode>& modes, int id);

// Returns a human-readable table of the catalogue. Used by the node at
// startup so the supported modes appear in the log output.
std::string format_mode_table(const std::string& model,
                              const std::vector<VideoMode>& modes);

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__VIDEO_MODES_HPP_
