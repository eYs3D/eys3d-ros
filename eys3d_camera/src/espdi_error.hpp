// Human-readable eSPDI SDK error codes.
//
// espdi_error_name() maps a return code to its eSPDI_def.h define name via a
// table generated at build time (see cmake/gen_espdi_error_names.py), so it
// tracks the bundled SDK automatically. espdi_strerror() adds a short hint for
// the codes users actually hit in the field.

#pragma once

#include <string>

#include "eSPDI_def.h"

namespace eys3d_camera {

inline const char* espdi_error_name(int rc) {
    switch (rc) {
#include "espdi_error_names.inc"
        default: return "unknown";
    }
}

inline std::string espdi_strerror(int rc) {
    std::string s = "rc=" + std::to_string(rc) + " " + espdi_error_name(rc);
    switch (rc) {
        case APC_NoDevice:
            s += " (no camera enumerated; check the USB connection and that "
                 "the user can access /dev/video*)";
            break;
        case APC_Init_Fail:
            s += " (SDK initialization failed; check libeSPDI loaded and "
                 "/dev/video* permissions)";
            break;
        case APC_OPEN_DEVICE_FAIL:
            s += " (could not open the device stream; transient during cold "
                 "start before the first frame arrives, or the requested mode "
                 "is incompatible with the negotiated USB link)";
            break;
        case APC_NOT_SUPPORT_RES:
            s += " (the device does not support this resolution/mode; e.g. a "
                 "USB2 link cannot open a USB3-only mode; check the negotiated "
                 "USB speed)";
            break;
#ifdef APC_DEVICE_BUSY
        case APC_DEVICE_BUSY:
            s += " (device busy; another process may hold the camera)";
            break;
#endif
#ifdef APC_DEVICE_TIMEOUT
        case APC_DEVICE_TIMEOUT:
            s += " (device timed out; try re-plugging the camera)";
            break;
#endif
        case APC_READFLASHFAIL:
            s += " (on-camera flash read failed; usually transient)";
            break;
        default:
            break;
    }
    return s;
}

}  // namespace eys3d_camera
