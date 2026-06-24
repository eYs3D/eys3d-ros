// Disparity-domain edge-aware spatial filter with integrated hole fill.
//
// Four-direction recursive IIR applied to D11 disparity promoted into
// Q4 fixed-point form (4 fractional bits) so sub-integer edges survive
// the rounded mix.
// Each pixel either smooths against its scanline neighbour when the gap
// is below the edge threshold, or starts a new segment when the gap
// exceeds it. Hole pixels are extended by the previous valid value
// within the same directional pass.

#pragma once

#include <cstdint>

namespace eys3d_camera {

struct SpatialFilterParams {
    // Smoothing strength, fixed-point Q8 (0..256). alpha_user * 256.
    // 256 → no filtering. 0 → carry prev value (degenerate).
    int alpha_q8 = 128;     // 0.5
    // Edge threshold in Q4 disparity units (delta_user << 4). A pixel
    // whose distance from prev exceeds this is treated as an edge: no
    // mixing, prev jumps to the new value.
    int delta_q4 = 20 << 4;
    // Number of full four-direction passes (L→R, R→L, T→B, B→T).
    // Total cost scales linearly.
    int magnitude = 2;
    // Maximum run length (in pixels) over which a hole is bridged
    // from the last valid neighbour during a directional pass.
    // 0 = no bridging (the spatial pass does smoothing only).
    int holes_fill = 0;
};

// In-place four-direction filter on Q4-promoted disparity. `disp_q4`
// stores W×H uint16 values; 0 is the hole sentinel and is preserved
// across edge boundaries but extended within a continuous scanline.
// Hole-fill is integrated into every direction pass (no separate pass).
void spatial_filter_q4(uint16_t* disp_q4, int w, int h,
                         const SpatialFilterParams& params);

// Promote raw 11-bit disparity (LE uint16, high bits masked) to Q4.
// `raw` is `w*h` source uint16; `out_q4` is the same size. Holes
// (raw == 0) become 0 in the output. Both buffers must not alias.
void disparity_promote_to_q4(const uint16_t* __restrict__ raw,
                             uint16_t* __restrict__ out_q4,
                             int w, int h);

}  // namespace eys3d_camera
