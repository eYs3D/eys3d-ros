#include "register_settings.hpp"

#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eSPDI.h"
#include "eSPDI_def.h"

namespace eys3d_camera {

namespace {

const char* logger() { return "RegisterSettings"; }

// Maps a camera USB PID to model tokens used to locate the matching
// `<model>_DM_Quality_Register_Setting.cfg`. Each entry tries the
// model-specific name first then falls back to the bundled DEFAULT cfg
// when a per-model file isn't shipped (e.g. G62 doesn't ship YX8081).
std::vector<const char*> model_names_for_pid(unsigned short pid) {
    switch (pid) {
    case APC_PID_80362: return {"YX80362", "DEFAULT"};   // G100+  (0x0181)
    case APC_PID_IRIS:  return {"YX80362", "DEFAULT"};   // G100+i (0x0184) — shares G100+ profile
    case APC_PID_8072:  return {"YX8072",  "DEFAULT"};   // R77    (0x0180)
    case APC_PID_8081:  return {"YX8081",  "DEFAULT"};   // G62    (0x0183)
    default:            return {"DEFAULT"};
    }
}

}  // namespace

int apply_dm_quality_register_setting(
    void* sdk_handle,
    int dev_index,
    unsigned short pid,
    const std::string& cfg_dir,
    std::mutex& sdk_mtx)
{
    if (!sdk_handle) {
        RCLCPP_ERROR(rclcpp::get_logger(logger()), "sdk_handle is null");
        return -1;
    }

    const auto candidates = model_names_for_pid(pid);
    std::string path;
    std::ifstream in;
    const char* chosen = nullptr;
    for (const char* name : candidates) {
        path = cfg_dir + "/" + name + "_DM_Quality_Register_Setting.cfg";
        in.open(path);
        if (in) {
            chosen = name;
            break;
        }
    }
    if (!chosen) {
        std::string tried;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (i) tried += ", ";
            tried += candidates[i];
        }
        RCLCPP_ERROR(rclcpp::get_logger(logger()),
                     "No cfg found in %s for PID=0x%04x (tried: %s)",
                     cfg_dir.c_str(), pid, tried.c_str());
        return -2;
    }
    RCLCPP_INFO(rclcpp::get_logger(logger()),
                "Applying %s (PID=0x%04x, chosen alias=%s)",
                path.c_str(), pid, chosen);

    // The SDK takes a DEVSELINFO*; build one matching the device index used
    // when the device was opened.
    DEVSELINFO sel{};
    sel.index = dev_index;

    int lines_applied = 0;
    int lines_failed  = 0;
    constexpr unsigned int kMaxRetry = 100;
    const int rmw_flags = FG_Address_2Byte | FG_Value_1Byte;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const char first = line[0];
        if (first == '#' || first == ';') continue;  // skip comments

        unsigned int addr = 0, mask = 0, data = 0;
        const int parsed = std::sscanf(line.c_str(), "%x, %x, %x", &addr, &mask, &data);
        if (parsed != 3) {
            RCLCPP_WARN(rclcpp::get_logger(logger()),
                        "skip unparseable line: '%s'", line.c_str());
            ++lines_failed;
            continue;
        }

        // Serialise the read-modify-write loop with other UVC control
        // traffic on the same FW endpoint (image controls, temperature
        // reads). Held for the whole RMW + verify so a concurrent
        // SetIRValue / SetExposureStep cannot land between the read and
        // the write.
        std::lock_guard<std::mutex> lk(sdk_mtx);

        unsigned short existing = 0;
        APC_GetHWRegister(sdk_handle, &sel,
                          static_cast<unsigned short>(addr),
                          &existing, rmw_flags);

        const unsigned short not_mask = static_cast<unsigned short>(~mask);
        const unsigned short target =
            static_cast<unsigned short>((existing & not_mask) |
                                        static_cast<unsigned short>(data));

        unsigned int retry = 0;
        unsigned short verify = 0;
        for (; retry < kMaxRetry; ++retry) {
            APC_SetHWRegister(sdk_handle, &sel,
                              static_cast<unsigned short>(addr),
                              target, rmw_flags);
            APC_GetHWRegister(sdk_handle, &sel,
                              static_cast<unsigned short>(addr),
                              &verify, rmw_flags);
            if (verify == target) break;
            usleep(5000);  // 5 ms pacing between register read-back retries
        }

        if (retry >= kMaxRetry) {
            RCLCPP_WARN(rclcpp::get_logger(logger()),
                        "reg 0x%04x: failed to set after %u retries "
                        "(want=0x%04x got=0x%04x existing=0x%04x mask=0x%04x data=0x%04x)",
                        addr, kMaxRetry, target, verify, existing, mask, data);
            ++lines_failed;
        } else {
            ++lines_applied;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger(logger()),
                "DM_Quality apply done: %d ok, %d failed",
                lines_applied, lines_failed);
    return lines_applied;
}

}  // namespace eys3d_camera
