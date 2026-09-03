#pragma once

#include "eys3d_selfcal.h"

#if defined(_WIN32)
#include "eSPDI_Common.h"
#include "eSPDI_DM.h"
#else
#include "eSPDI.h"
#include "eSPDI_def.h"
#endif

namespace eys3d {
namespace selfcal {

inline EYS3D_SC_CreateInfo MakeCreateInfo(
    void* pHandleApcDI,
    DEVSELINFO* pDevSelInfo,
    DEVINFORMATIONEX* pDevInfo,
    EYS3D_SC_LogCallback callback = nullptr,
    void* userData = nullptr,
    const char* deviceId = nullptr) {
    EYS3D_SC_CreateInfo info{};
    info.struct_size = sizeof(info);
    info.espdi_handle = pHandleApcDI;
    info.espdi_device_selection = pDevSelInfo;
    info.espdi_device_information = pDevInfo;
    info.log_callback = callback;
    info.log_user_data = userData;
    info.device_id = deviceId;
    return info;
}

} // namespace selfcal
} // namespace eys3d
