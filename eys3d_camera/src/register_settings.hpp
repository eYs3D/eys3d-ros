#ifndef EYS3D_CAMERA__REGISTER_SETTINGS_HPP_
#define EYS3D_CAMERA__REGISTER_SETTINGS_HPP_

#include <mutex>
#include <string>

namespace eys3d_camera {

// Applies the per-chip register tuning file
// (YX<chip>_DM_Quality_Register_Setting.cfg) to the camera firmware. Each
// line is `<RegAddr>, <ValidDataMask>, <Data>` in hex; the function
// performs a read-modify-write of every register with read-back verification.
//
// Must be invoked after streaming has begun and at least one depth
// frame has been received; firmware writes before that point are
// discarded.
//
// The caller holds sdk_mtx for the whole read-modify-write of each register:
// concurrent UVC control traffic (image controls, temperature reads) would
// otherwise interleave with this loop and corrupt the FW control channel.
//
// Returns the number of registers programmed, or a negative APC_* error code
// if the config file is missing or unparsable.
int apply_dm_quality_register_setting(
    void* sdk_handle,
    int dev_index,
    unsigned short pid,
    const std::string& cfg_dir,
    std::mutex& sdk_mtx);

}  // namespace eys3d_camera

#endif  // EYS3D_CAMERA__REGISTER_SETTINGS_HPP_
