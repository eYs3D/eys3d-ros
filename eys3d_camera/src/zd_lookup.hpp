// ZD-table lookup for disparity → Z (mm) conversion.
//
// The eSPDI firmware ships a per-rectify-index lookup table indexed by
// disparity value. The table is cached once at open(); per-pixel Z is
// recovered by indexing with linear interpolation across the integer
// boundary so Q4-promoted disparities preserve sub-integer precision.
//
// Build switch:
//   -DEYS3D_ZD_ROUND_NEAREST   selects integer round-to-nearest instead
//                              of linear interpolation.

#pragma once

#include <cstdint>
#include <vector>

namespace eys3d_camera {

struct ZdTable {
    // table[disparity_index] = Z in millimetres. Disparity index is the
    // raw 11-bit value reported by the depth stream (0..2047). Out-of-
    // range disparities clamp to the last entry.
    std::vector<int32_t> mm;
    bool                 valid = false;
};

// Load the ZD table for the active rectify index. The firmware returns
// a byte buffer of big-endian uint16 millimetre values, widened to
// int32 for arithmetic headroom during interpolation. Returns false on
// a firmware read failure; APC_READFLASHFAIL is retried with backoff.
//
// `zd_index` follows the same group rule as the rectify-log index
// (1280x720 → 0, 640x480 → 1, ...). The dtype field of ZDTABLEINFO is
// pinned to APC_DEPTH_DATA_11_BITS (4) so it matches the 11-bit buffer
// size APC_GetZDTable expects; the firmware ignores the field beyond
// that consistency check, so the call is independent of the active
// depth stream's data type.
bool load_zd_table(void* sdk_handle, void* dev_sel_info,
                   int zd_index, ZdTable& out);

// Convert a Q4 disparity (raw_disparity << 4) to Z in millimetres.
// Returns 0 for the hole sentinel (d_q4 == 0).
int32_t zd_lookup_q4(const ZdTable& tbl, uint16_t d_q4);

}  // namespace eys3d_camera
