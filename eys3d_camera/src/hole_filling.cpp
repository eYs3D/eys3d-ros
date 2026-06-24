#include "hole_filling.hpp"

#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace eys3d_camera {

namespace {

constexpr uint16_t kHole = 0;

// Per-scanline left-to-right fill. Each row is independent so the
// outer loop is OMP-parallel; within a row the running last_valid
// value chains forward. The leftmost left_skip columns are passed
// through and excluded from the last_valid state so a low-confidence
// boundary match coming out of the stereo dead zone is not chained
// across the rest of the row.
void fill_from_left(uint16_t* z, int w, int h, int left_skip) {
    const int start = (left_skip < 0) ? 0
                    : (left_skip > w) ? w
                                      : left_skip;
    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        uint16_t* row = z + static_cast<size_t>(v) * w;
        uint16_t last_valid = kHole;
        for (int u = start; u < w; ++u) {
            if (row[u] == kHole) {
                row[u] = last_valid;     // stays 0 until the first valid
            } else {
                last_valid = row[u];
            }
        }
    }
}

// Common 8-neighbourhood pass used by farthest_from_around and
// nearest_from_around. The selection rule on the candidate Z value
// (max for farthest, min for nearest) is supplied by the caller as a
// template parameter so the inner loop carries no per-iteration
// branch on the mode. dst and src are owned by the caller and never
// alias — src is a frozen copy of the input made before this pass.
template <bool kFarthest>
void fill_around_impl(uint16_t* __restrict__ dst,
                      const uint16_t* __restrict__ src,
                      int w, int h) {
    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        const int v_lo = (v > 0)     ? v - 1 : v;
        const int v_hi = (v < h - 1) ? v + 1 : v;
        uint16_t* __restrict__ row = dst + static_cast<size_t>(v) * w;
        for (int u = 0; u < w; ++u) {
            if (row[u] != kHole) continue;
            const int u_lo = (u > 0)     ? u - 1 : u;
            const int u_hi = (u < w - 1) ? u + 1 : u;

            uint16_t best = kHole;
            for (int nv = v_lo; nv <= v_hi; ++nv) {
                const uint16_t* __restrict__ src_row = src + static_cast<size_t>(nv) * w;
                for (int nu = u_lo; nu <= u_hi; ++nu) {
                    if (nu == u && nv == v) continue;
                    const uint16_t cand = src_row[nu];
                    if (cand == kHole) continue;
                    if (best == kHole) {
                        best = cand;
                    } else if (kFarthest) {
                        if (cand > best) best = cand;
                    } else {
                        if (cand < best) best = cand;
                    }
                }
            }
            row[u] = best;             // stays 0 when no valid neighbour
        }
    }
}

}  // namespace

void hole_fill_z(uint16_t* __restrict__ z_mm, int w, int h,
                 HoleFillMode mode,
                 int left_skip,
                 std::vector<uint16_t>& scratch) {
    if (mode == HoleFillMode::kOff || w <= 0 || h <= 0) return;

    if (mode == HoleFillMode::kFillFromLeft) {
        fill_from_left(z_mm, w, h, left_skip);
        return;
    }

    // Around modes read a frozen copy of the input so the result is
    // independent of raster traversal order and safe to parallelise.
    const size_t n = static_cast<size_t>(w) * h;
    if (scratch.size() != n) scratch.resize(n);
    std::memcpy(scratch.data(), z_mm, n * sizeof(uint16_t));

    if (mode == HoleFillMode::kFarthestAround) {
        fill_around_impl<true>(z_mm, scratch.data(), w, h);
    } else {
        fill_around_impl<false>(z_mm, scratch.data(), w, h);
    }
}

}  // namespace eys3d_camera
