#include "spatial_filter.hpp"

#include <cstdlib>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace eys3d_camera {

namespace {

constexpr uint16_t kHole       = 0;
constexpr uint16_t kRawDispMask = 0x07FF;  // 11-bit disparity

// mixed = alpha*cur + (1-alpha)*prev, Q8 weights on Q4 disparity.
inline uint16_t mix_q8(int prev, int cur, int alpha_q8) {
    const int v = (alpha_q8 * cur + (256 - alpha_q8) * prev) >> 8;
    return static_cast<uint16_t>(v);
}

// Per-scanline kernel, templated on the iteration direction so all four
// pass functions share one definition.
template <int Step>
inline void filter_scanline(uint16_t* base, int n, int stride,
                            int alpha_q8, int delta_q4, int max_fill) {
    const auto at = [base, stride](int i) -> uint16_t& {
        return base[static_cast<size_t>(i) * stride];
    };

    int i = (Step > 0) ? 0 : n - 1;
    const int end = (Step > 0) ? n : -1;
    while (i != end && at(i) == kHole) i += Step;
    if (i == end) return;
    int prev = at(i);
    int fill_run = 0;
    i += Step;
    for (; i != end; i += Step) {
        const int cur = at(i);
        if (cur == kHole) {
            // max_fill == 0 → no hole bridging in this pass.
            // max_fill > 0  → bridge up to max_fill consecutive holes.
            if (max_fill > 0 && fill_run < max_fill) {
                at(i) = static_cast<uint16_t>(prev);
                ++fill_run;
            }
            continue;
        }
        fill_run = 0;
        if (std::abs(cur - prev) < delta_q4) {
            const uint16_t mixed = mix_q8(prev, cur, alpha_q8);
            at(i) = mixed;
            prev = mixed;
        } else {
            prev = cur;
        }
    }
}

// Each scanline is independent; the outer loop is parallelised with
// static scheduling so per-thread working sets stay contiguous.
void filter_horizontal_lr(uint16_t* img, int w, int h,
                          int alpha_q8, int delta_q4, int max_fill) {
    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        filter_scanline<+1>(img + static_cast<size_t>(v) * w, w, 1,
                            alpha_q8, delta_q4, max_fill);
    }
}

void filter_horizontal_rl(uint16_t* img, int w, int h,
                          int alpha_q8, int delta_q4, int max_fill) {
    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        filter_scanline<-1>(img + static_cast<size_t>(v) * w, w, 1,
                            alpha_q8, delta_q4, max_fill);
    }
}

#ifdef __aarch64__
// Mix two Q4-disparity lanes with a Q8 alpha:
//   out = (alpha * cur + (256 - alpha) * prev) >> 8
// Both inputs are u16; the intermediate widens to u32 to avoid the
// 16-bit overflow on alpha*cur (alpha <= 256, cur <= 2047 << 4).
inline uint16x8_t mix_q8_neon(uint16x8_t prev, uint16x8_t cur,
                              uint16x8_t alpha_v, uint16x8_t inv_alpha_v) {
    uint32x4_t lo = vmull_u16(vget_low_u16(alpha_v),  vget_low_u16(cur));
    uint32x4_t hi = vmull_u16(vget_high_u16(alpha_v), vget_high_u16(cur));
    lo = vmlal_u16(lo, vget_low_u16(inv_alpha_v),  vget_low_u16(prev));
    hi = vmlal_u16(hi, vget_high_u16(inv_alpha_v), vget_high_u16(prev));
    return vcombine_u16(vshrn_n_u32(lo, 8), vshrn_n_u32(hi, 8));
}

// Cross-column vertical IIR for aarch64. Processes 8 columns per OMP
// work item, marching rows down (Step=+1) or up (Step=-1) so the
// per-lane state (prev / fill_run / init mask) stays in vector
// registers and memory access is unit-stride per row.
//
// Semantics match the scalar filter_scanline:
//   * leading-hole pixels are left untouched until the first valid
//     pixel initialises prev for that lane;
//   * the first valid pixel becomes prev and is not modified;
//   * a hole following an initialised lane is filled with prev when
//     fill_run is below max_fill (or max_fill <= 0 for unbounded);
//   * a valid pixel within delta of prev is blended via mix_q8 and
//     becomes the new prev.
template <int Step>
void filter_vertical_q4_neon(uint16_t* base, int w_stride, int w_neon, int h,
                             int alpha_q8, int delta_q4, int max_fill) {
    const uint16x8_t alpha_v     = vdupq_n_u16(static_cast<uint16_t>(alpha_q8));
    const uint16x8_t inv_alpha_v = vdupq_n_u16(static_cast<uint16_t>(256 - alpha_q8));
    const uint16x8_t delta_v     = vdupq_n_u16(static_cast<uint16_t>(delta_q4));
    const uint16x8_t max_fill_v  = vdupq_n_u16(static_cast<uint16_t>(max_fill));
    const uint16x8_t zero_v      = vdupq_n_u16(0);
    const uint16x8_t one_v       = vdupq_n_u16(1);
    // max_fill == 0 → hole bridging disabled in this pass.
    const bool no_fill = (max_fill <= 0);

    #pragma omp parallel for schedule(static)
    for (int u0 = 0; u0 < w_neon; u0 += 8) {
        uint16x8_t prev      = zero_v;
        uint16x8_t fill_run  = zero_v;
        uint16x8_t init_mask = zero_v;        // 0 = uninitialised, 0xFFFF = prev valid

        const int row_start = (Step > 0) ? 0 : h - 1;
        const int row_end   = (Step > 0) ? h : -1;
        for (int row = row_start; row != row_end; row += Step) {
            uint16_t* p = base + static_cast<size_t>(row) * w_stride + u0;
            const uint16x8_t cur = vld1q_u16(p);

            const uint16x8_t cur_is_hole  = vceqq_u16(cur, zero_v);
            const uint16x8_t cur_is_valid = vmvnq_u16(cur_is_hole);

            // Hole + initialised + still within fill budget → re-emit prev.
            // no_fill disables the entire bridge path for this pass.
            const uint16x8_t can_fill = no_fill
                ? zero_v
                : vcltq_u16(fill_run, max_fill_v);
            const uint16x8_t do_fill = vandq_u16(
                vandq_u16(init_mask, cur_is_hole), can_fill);

            // Valid + initialised + within delta → blend with prev
            const uint16x8_t abs_diff     = vabdq_u16(cur, prev);
            const uint16x8_t within_delta = vcltq_u16(abs_diff, delta_v);
            const uint16x8_t do_blend = vandq_u16(
                vandq_u16(init_mask, cur_is_valid), within_delta);

            // Compute mixed unconditionally; per-lane select picks
            // whether to commit it.
            const uint16x8_t mixed = mix_q8_neon(prev, cur, alpha_v, inv_alpha_v);

            uint16x8_t out = cur;
            out = vbslq_u16(do_fill,  prev,  out);
            out = vbslq_u16(do_blend, mixed, out);
            vst1q_u16(p, out);

            // prev: hole lanes keep prev; valid lanes adopt the new
            // output (cur or mixed depending on do_blend).
            uint16x8_t new_prev = vbslq_u16(cur_is_valid, cur, prev);
            new_prev = vbslq_u16(do_blend, mixed, new_prev);
            prev = new_prev;

            // fill_run: valid resets to 0; do_fill increments; else holds.
            const uint16x8_t fr_incr = vaddq_u16(fill_run, one_v);
            uint16x8_t new_fr = vbslq_u16(do_fill, fr_incr, fill_run);
            new_fr = vbslq_u16(cur_is_valid, zero_v, new_fr);
            fill_run = new_fr;

            // init_mask: latches on the first valid pixel per lane.
            init_mask = vorrq_u16(init_mask, cur_is_valid);
        }
    }
}
#endif  // __aarch64__

void filter_vertical_tb(uint16_t* img, int w, int h,
                        int alpha_q8, int delta_q4, int max_fill) {
#ifdef __aarch64__
    const int w_neon = (w / 8) * 8;
    if (w_neon > 0) {
        filter_vertical_q4_neon<+1>(img, w, w_neon, h,
                                    alpha_q8, delta_q4, max_fill);
    }
    if (w > w_neon) {
        #pragma omp parallel for schedule(static)
        for (int u = w_neon; u < w; ++u) {
            filter_scanline<+1>(img + u, h, w, alpha_q8, delta_q4, max_fill);
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int u = 0; u < w; ++u) {
        filter_scanline<+1>(img + u, h, w, alpha_q8, delta_q4, max_fill);
    }
#endif
}

void filter_vertical_bt(uint16_t* img, int w, int h,
                        int alpha_q8, int delta_q4, int max_fill) {
#ifdef __aarch64__
    const int w_neon = (w / 8) * 8;
    if (w_neon > 0) {
        filter_vertical_q4_neon<-1>(img, w, w_neon, h,
                                    alpha_q8, delta_q4, max_fill);
    }
    if (w > w_neon) {
        #pragma omp parallel for schedule(static)
        for (int u = w_neon; u < w; ++u) {
            filter_scanline<-1>(img + u, h, w, alpha_q8, delta_q4, max_fill);
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int u = 0; u < w; ++u) {
        filter_scanline<-1>(img + u, h, w, alpha_q8, delta_q4, max_fill);
    }
#endif
}

}  // namespace

void disparity_promote_to_q4(const uint16_t* __restrict__ raw,
                             uint16_t* __restrict__ out_q4,
                             int w, int h) {
#ifdef __aarch64__
    const uint16x8_t mask_v = vdupq_n_u16(kRawDispMask);
#endif
    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        const uint16_t* __restrict__ p = raw   + static_cast<size_t>(v) * w;
        uint16_t*       __restrict__ q = out_q4 + static_cast<size_t>(v) * w;
        int u = 0;
#ifdef __aarch64__
        // 8-lane NEON: mask the 11-bit disparity field and shift into Q4.
        // The hole sentinel (raw == 0) maps to 0 in both branches of the
        // scalar conditional, so an unconditional shift is byte-equivalent.
        const int w_neon = (w / 8) * 8;
        for (; u < w_neon; u += 8) {
            const uint16x8_t r = vandq_u16(vld1q_u16(p + u), mask_v);
            vst1q_u16(q + u, vshlq_n_u16(r, 4));
        }
#endif
        for (; u < w; ++u) {
            const uint16_t r = p[u] & kRawDispMask;
            q[u] = (r == 0) ? 0 : static_cast<uint16_t>(r << 4);
        }
    }
}

void spatial_filter_q4(uint16_t* disp_q4, int w, int h,
                         const SpatialFilterParams& params) {
    const int a = params.alpha_q8;
    const int d = params.delta_q4;
    const int f = params.holes_fill;
    for (int iter = 0; iter < params.magnitude; ++iter) {
        filter_horizontal_lr(disp_q4, w, h, a, d, f);
        filter_horizontal_rl(disp_q4, w, h, a, d, f);
        filter_vertical_tb  (disp_q4, w, h, a, d, f);
        filter_vertical_bt  (disp_q4, w, h, a, d, f);
    }
}

}  // namespace eys3d_camera
