// Temporal filter on a uint16 raster. The buffer's semantic unit is
// decided by the caller — typically D11 disparity (in Q4 fixed-point
// form when chained after the spatial IIR) or Z14 mm depth (when the
// spatial filter is off).
// The hole sentinel is always 0; alpha and delta apply in the
// caller's units.
//
// Two effects in one pass:
//   1. Alpha blend: when both the current and previous outputs are
//      valid and the gap stays within delta, the output is mixed
//      against the previous frame in Q8 (low-pass on jitter).
//   2. Persistence: when the current pixel is invalid, the previous
//      output may be re-emitted while the 8-frame validity history
//      satisfies the configured persistence index.
//
// Per-pixel state (prev + an 8-bit validity mask) is held in a
// TemporalState owned by the caller and resized once per stream.

#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace eys3d_camera {

struct TemporalFilterParams {
    // alpha in Q8 (0..256). Output = (alpha*cur + (256-alpha)*prev) >> 8.
    // Applied only when both cur and prev are valid and the per-pixel
    // gap stays below delta (edge guard).
    int alpha_q8 = 102;    // ≈ 0.4 in Q8

    // Maximum per-pixel gap at which the alpha blend stays engaged.
    // Units match the caller's buffer: Q4 disparity (raw_disparity
    // << 4) when chained after the spatial IIR, mm when applied
    // directly to a Z14 depth raster. A gap at or above delta is
    // treated as a depth edge and the blend is skipped.
    int delta = 20;        // value in the caller's domain unit

    // Persistence index:
    //   0 — disabled (invalid stays invalid)
    //   1 — 8/8 of the last 8 frames must have been valid
    //   2 — 2 valid in the last 3 frames
    //   3 — 2 valid in the last 4 frames
    //   4 — 2 valid in the last 8 frames
    //   5 — 1 valid in the last 2 frames
    //   6 — 1 valid in the last 5 frames
    //   7 — 1 valid in the last 8 frames
    //   8 — persist indefinitely (re-emit prev as long as it is valid)
    int persistence = 3;
};

struct TemporalState {
    // prev[p] holds the last emitted value for pixel p. Zero means the
    // pixel has never produced a valid output, or the state was reset.
    std::vector<uint16_t> prev;
    // valid_history[p] is a rolling 8-frame validity bitmask. After
    // each frame the mask is shifted left and the new validity is
    // written into bit 0; bit 7 then carries the oldest tracked frame.
    std::vector<uint8_t>  valid_history;
    int w = 0;
    int h = 0;

    void resize(int width, int height) {
        if (w == width && h == height && !prev.empty()) return;
        w = width;
        h = height;
        const size_t n = static_cast<size_t>(width) * height;
        prev.assign(n, 0);
        valid_history.assign(n, 0);
    }

    // Drop all history. Called on a disabled-to-enabled transition
    // and after stream restarts.
    void reset() {
        std::fill(prev.begin(), prev.end(), uint16_t{0});
        std::fill(valid_history.begin(), valid_history.end(), uint8_t{0});
    }
};

// Apply the temporal filter in place. `cur` is overwritten with the
// temporally filtered values; state is updated with the new prev and
// validity history for the next call. `cur` must not alias
// state.prev.data() or state.valid_history.data().
void temporal_filter_apply(uint16_t* __restrict__ cur, int w, int h,
                           TemporalState& state,
                           const TemporalFilterParams& params);

}  // namespace eys3d_camera
