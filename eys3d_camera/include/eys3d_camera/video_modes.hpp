#ifndef EYS3D_CAMERA__VIDEO_MODES_HPP_
#define EYS3D_CAMERA__VIDEO_MODES_HPP_

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

    // Indicates which streams the mode produces. The YAML parser sets these
    // only when a `color:` or `depth:` block is present and contains valid
    // dimensions, allowing publishers and fetch threads to be skipped for
    // modes that omit a stream entirely.
    bool has_color = false;
    bool has_depth = false;
};

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
