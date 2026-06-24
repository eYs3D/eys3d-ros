// Z-domain hole filling, applied after the ZD lookup and before
// reprojection. Operates in place on the Z14 millimetre buffer;
// a hole is any pixel equal to 0.
//
//   Mode 1 (fill_from_left)
//     Per scanline, an invalid pixel inherits the last valid pixel
//     to its left. The leftmost left_skip columns are passed through
//     and excluded from the last-valid state.
//
//   Mode 2 (farthest_from_around)
//     Each invalid pixel takes the largest valid Z in its 8-pixel
//     neighbourhood (background bias).
//
//   Mode 3 (nearest_from_around)
//     Each invalid pixel takes the smallest valid Z in its 8-pixel
//     neighbourhood (foreground bias).
//
// Modes 2 and 3 read from a frozen copy of the buffer so the result
// is independent of raster traversal order; the caller owns the
// scratch storage. The around modes are dead-zone safe and ignore
// left_skip.

#pragma once

#include <cstdint>
#include <vector>

namespace eys3d_camera {

enum class HoleFillMode : int {
    kOff             = 0,
    kFillFromLeft    = 1,
    kFarthestAround  = 2,
    kNearestAround   = 3,
};

// Apply the selected hole filling mode in place. `scratch` is only
// touched by the around modes; it is grown to w*h on first use and
// retained for later calls. `left_skip` is honoured only by mode 1;
// the around modes ignore it. `scratch` storage must not alias `z_mm`.
void hole_fill_z(uint16_t* __restrict__ z_mm, int w, int h,
                 HoleFillMode mode,
                 int left_skip,
                 std::vector<uint16_t>& scratch);

}  // namespace eys3d_camera
